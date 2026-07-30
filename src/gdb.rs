//! Run-forever serve loop for the emulated SP (and, in the two-core setup, an
//! in-process RoT), plus a debug port for humility.
//!
//! The debug transport is the Glasgow SWD probe (`crate::glasgow`): a stock
//! humility (probe-rs) attaches to it via a `20b7:9db1:tcp:127.0.0.1:<port>`
//! selector, which drives halt/run/step/read/write over an emulated SWD DP/AP.
//! The older GDB-RSP (`-p ocdgdb`) and OpenOCD (`-p ocd`) transports this module
//! used to serve were dropped once humility removed those probe backends.
//!
//! Between connections the emulator keeps running, so time advances across a
//! series of humility commands, and the MGS bridge (`crate::bridge`) stays live
//! for faux-mgs / sp-test.

use crate::cpu::Cpu;
use crate::host::HostIo;
use crate::mem::Bus;
use anyhow::Result;
use std::net::TcpListener;

/// Safety bound on prompt-halt servicing: iterations the serve loop will freeze the
/// SP waiting for the RoT to halt it after a self-reset. Generous (the RoT only needs
/// to take its SP_RESET IRQ and run its handler); a backstop so an armed-but-unhalting
/// RoT can never wedge the SP, which then resumes free-running as it would today.
const SP_RESET_HALT_ITERS: u32 = 1000;

/// Instructions the SP may run this serve-loop iteration, in priority order: a
/// debug-halted core is not stepped; a self-reset being serviced freezes the SP at
/// its reset vector so the RoT halts it before it runs reset-vector work; a core
/// running an injected program under debug (endoscope, after the RoT resumed it)
/// sprints so it reaches its terminal BKPT within the RoT's halt-poll; a phase-2
/// sprot reply uses a small burst so the SP never clocks past the RoT's refill;
/// otherwise the full eth-service quantum.
fn sp_burst_for(
    halted: bool,
    sp_reset_servicing: bool,
    debug_en: bool,
    replying: bool,
    quantum: u32,
) -> u32 {
    // Freeze the SP (burst 0) when it is debug-halted (only the RoT may move it) or
    // when a self-reset is being serviced (hold it at the reset vector until the RoT
    // halts it). The remaining cases run it.
    if halted || sp_reset_servicing {
        0
    } else if debug_en {
        20_000_000
    } else if replying {
        48
    } else {
        quantum
    }
}

/// Whether a genuine SP self-reset should enter prompt-halt servicing. Only once the
/// RoT is watching (its swd task armed SP_RESET, `rot_armed`); not if the SP is
/// already halted (a vector catch caught the reset). While the RoT is unarmed (early
/// boot), the SP's measurement loop must keep free-running, so this stays false.
fn enter_sp_reset_service(sp_reset_edge: bool, rot_armed: bool, sp_halted: bool) -> bool {
    sp_reset_edge && rot_armed && !sp_halted
}

/// Advance prompt-halt servicing by one iteration. Cleared once the RoT takes control
/// (halted the SP, or resumed it under debug for endoscope), or when the safety bound
/// is exhausted (the RoT never halted: give up and free-run). Returns whether
/// servicing is still active; decrements `iters` only on the still-waiting path.
fn continue_sp_reset_service(
    servicing: bool,
    sp_halted: bool,
    sp_debug_en: bool,
    iters: &mut u32,
) -> bool {
    if !servicing || sp_halted || sp_debug_en {
        return false;
    }
    *iters = iters.saturating_sub(1);
    *iters != 0
}

// JTAG_DETECT (SP_TO_ROT_JTAG_DETECT_L): the RoT input an SP SWD probe asserts. PINT
// slot 1 -> NVIC IRQ 5 (the sibling of SP_RESET's slot 0 -> IRQ 4). The slot bit is
// written into the flat PINT RegFile at the same register SP_RESET uses; the level
// (PIO0_20) is synthesized separately in LpcGpio from `SprotLink::jtag_detect`.
const JTAG_DETECT_IRQ: u16 = 5;
const JTAG_DETECT_PINT_REG: u32 = 0x4000_4020;
const JTAG_DETECT_PINT_BIT: u32 = 1 << 1; // PINT slot 1
/// Bound on the pre-glasgow RoT pump: iterations to step the RoT so its swd task can
/// service a just-delivered JTAG_DETECT edge (invalidate the attestation log) before
/// glasgow::serve freezes the RoT for the session. Generous; the handler is short.
const JTAG_PUMP_ITERS: u32 = 200;

/// Deliver a JTAG_DETECT falling edge to the RoT, if its firmware has armed the IRQ.
/// Sets the PINT slot-1 detect bit (OR, so a coincident SP_RESET slot-0 bit survives)
/// and pends IRQ 5. Returns whether it injected: `false` when the firmware has not
/// enabled JTAG_DETECT (e.g. an older RoT image), so the whole feature stays inert.
fn inject_jtag_detect(rb: &mut Bus) -> bool {
    if !rb.irq_enabled(JTAG_DETECT_IRQ) {
        return false;
    }
    let cur = rb.read32(JTAG_DETECT_PINT_REG);
    rb.write32(JTAG_DETECT_PINT_REG, cur | JTAG_DETECT_PINT_BIT);
    rb.pend_irq(JTAG_DETECT_IRQ);
    true
}

/// Step the RoT until it goes idle again or a bounded cap, so a just-pended
/// notification (JTAG_DETECT) is serviced before the caller freezes the RoT.
fn pump_rot_briefly(rc: &mut Cpu, rb: &mut Bus, host: &mut dyn HostIo, quantum: u32) {
    for _ in 0..JTAG_PUMP_ITERS {
        let mut idled = false;
        for _ in 0..quantum {
            if rc.step(rb, host).is_err() {
                return;
            }
            rc.maybe_tick(rb);
            rc.maybe_interrupt(rb);
            if rc.idle_skip > 0 {
                rc.idle_skip = 0;
                idled = true;
                break;
            }
        }
        if idled {
            break;
        }
    }
}

/// Pre-boot to steady state, then serve the Glasgow SWD debug probe on
/// 127.0.0.1:<swd_port> (`humility -p 20b7:9db1:tcp:127.0.0.1:<swd_port>`).
///
/// Between connections the emulator keeps running, so time advances across a
/// series of humility commands.
pub fn serve(
    mut cpu: Cpu,
    mut bus: Bus,
    mut rot: Option<(Cpu, Bus)>,
    mut rot_client: Option<crate::rot_service::RotClient>,
    host: &mut dyn HostIo,
    preboot: u64,
) -> Result<()> {
    eprintln!(
        "[gdb] pre-booting {} instructions to steady state...",
        preboot
    );
    let (twin_from, twin_to) = (
        crate::config::get().trace_from,
        crate::config::get().trace_to,
    );
    // Pay the per-instruction disasm-formatting cost only when the windowed trace is on.
    cpu.record_disasm = twin_from.is_some();
    let preboot_start = std::time::Instant::now();
    for _ in 0..preboot {
        let pc = cpu.pc;
        if cpu.step(&mut bus, host).is_err() {
            break;
        }
        if let (Some(lo), Some(hi)) = (twin_from, twin_to) {
            if cpu.cycles >= lo && cpu.cycles <= hi {
                eprintln!("c{} {:08x}: {:<28} | r0={:08x} r1={:08x} r2={:08x} r3={:08x} r4={:08x} r5={:08x} r6={:08x} r7={:08x} sp={:08x} lr={:08x}",
                    cpu.cycles, pc, cpu.last_disasm,
                    cpu.r[0], cpu.r[1], cpu.r[2], cpu.r[3], cpu.r[4], cpu.r[5], cpu.r[6], cpu.r[7], cpu.r[13], cpu.r[14]);
            }
        }
        cpu.maybe_tick(&mut bus);
        cpu.maybe_interrupt(&mut bus);
    }
    let secs = preboot_start.elapsed().as_secs_f64();
    eprintln!(
        "[gdb] booted to {} instructions (pc={:#010x}) in {:.2}s = {:.1}M instr/s",
        cpu.cycles,
        cpu.pc,
        secs,
        cpu.cycles as f64 / secs / 1e6
    );

    // Boot the in-process RoT core (LPC55/M33) to its sprot dispatch idle.
    if let Some((rc, rb)) = rot.as_mut() {
        eprintln!("[rot] pre-booting RoT core...");
        rc.wfi_throttle = false;
        let t = std::time::Instant::now();
        let dbgtrap = crate::sprot::dbg();
        // Budget high enough for the dice-self startup to finish its DICE cert
        // generation (PUF seed + several Ed25519 keygens/signs + SHA3) -- it
        // breaks early on idle (WFI) once startup completes. Override with
        // SP_EMU_ROT_PREBOOT.
        let rot_preboot = crate::config::get().rot_preboot.unwrap_or(400_000_000);
        // Optional windowed instruction trace during preboot (where DICE runs),
        // SP_EMU_ROT_TRACE_FROM/TO as hex pc bounds.
        let (pb_from, pb_to) = (
            crate::config::get().rot_trace_from,
            crate::config::get().rot_trace_to,
        );
        rc.record_disasm |= pb_from.is_some();
        for _ in 0..rot_preboot {
            let pc_before = rc.pc;
            if let Err(t) = rc.step(rb, host) {
                if dbgtrap {
                    eprintln!("[rottrap-preboot] {:?}", t);
                }
                break;
            }
            if let (Some(f), Some(t)) = (pb_from, pb_to) {
                if (f..=t).contains(&pc_before) {
                    eprintln!(
                        "[rotpb] {:#010x}: {:<24} r0={:08x} r1={:08x} r2={:08x} r4={:08x} r5={:08x} r6={:08x}",
                        pc_before, rc.last_disasm, rc.r[0], rc.r[1], rc.r[2], rc.r[4], rc.r[5], rc.r[6]
                    );
                }
            }
            // Spin detector: a `b .` self-branch (pc unchanged) is Hubris's panic/
            // fault loop. Dump registers + on-stack return addresses to locate the
            // panicking call site. Gated on SP_EMU_SPROTDBG.
            if dbgtrap && rc.pc == pc_before {
                eprintln!(
                    "[rot-spin] stuck at pc={:#010x} lr={:#010x} sp={:#010x} cyc={}",
                    rc.pc, rc.r[14], rc.r[13], rc.cycles
                );
                eprintln!("[rot-spin] r0..r7 = {:08x?}", &rc.r[0..8]);
                let sp = rc.r[13];
                for i in 0..64u32 {
                    let v = rb.read32(sp.wrapping_add(i * 4));
                    if (0x0001_0000..0x0002_0000).contains(&v) {
                        eprintln!(
                            "[rot-spin] stack[{:#010x}] = {:#010x}",
                            sp.wrapping_add(i * 4),
                            v
                        );
                    }
                }
                break;
            }
            rc.maybe_tick(rb);
            rc.maybe_interrupt(rb);
            if rc.idle_skip > 0 {
                rc.idle_skip = 0;
                break;
            }
        }
        rc.wfi_throttle = true;
        rc.trace_svc = crate::config::get().rotsvc;
        eprintln!(
            "[rot] RoT core booted (pc={:#010x}, {} insns) in {:.2}s",
            rc.pc,
            rc.cycles,
            t.elapsed().as_secs_f64()
        );
    }

    let rotpc_every = crate::config::get().rotpc;
    let mut rotpc_next = 0u64;
    let mut last_rottrap = u32::MAX;
    // SP_EMU_ROT_TRACE_FROM/TO="0xADDR": log the RoT's pc + disasm + r0..r3,r6 for
    // every instruction whose pc is in [FROM,TO] — an instruction-level window to
    // debug a specific RoT function's execution.
    let rot_trace_from = crate::config::get().rot_trace_from;
    let rot_trace_to = crate::config::get().rot_trace_to;
    // SP_EMU_ROTDUMP="0xADDR:LEN" dumps that RoT RAM range every ~8s for task-table introspection.
    let rotdump: Option<(u32, u32)> = crate::config::get().rotdump;
    let mut rotdump_last = std::time::Instant::now();

    // Post-preboot: enable the WFI idle-throttle so an idle SP sleeps the host
    // instead of pegging a core (preboot ran with it off, at full spin speed).
    cpu.wfi_throttle = true;
    // Per idle WFI, sleep this long (ms) instead of spinning. Larger = lower
    // idle CPU but slower background sim-time; an incoming packet's eth-irq wakes
    // the SP immediately regardless. 10ms ≈ 4x CPU cut; tune via SP_EMU_IDLE_MS
    // for denser fleets.
    let idle_ms: u64 = crate::config::get().idle_ms;

    // Eth-service quantum: instructions the SP runs between bridge pumps (the
    // only place TX frames flush out and RX frames poll in). Under sustained MGS
    // load the SP never goes idle, so the batch never breaks early on `idle_skip`
    // and every request/reply round-trip pays up to a full batch of wall-clock
    // latency in each direction. On a contended host (the rack runs several SP
    // instances next to the whole control plane) a batch's wall-clock inflates,
    // so a few-hundred-ms MGS attempt budget (e.g. the inventory collector's
    // GET /ignition) times out -> empty SP inventory. A small quantum bounds
    // inbound latency; the `eth_has_tx` break (below) bounds outbound. The preboot
    // loop is separate, so full-speed boot throughput is unaffected.
    let quantum: u32 = crate::config::get().eth_quantum;
    // TX-break: end the batch the instant the SP queues a reply so it flushes
    // immediately instead of waiting out the rest of the quantum. On by default;
    // SP_EMU_ETH_TXBREAK=0 disables it (A/B against the once-per-batch behavior).
    let txbreak = crate::config::get().eth_txbreak;
    // sprot SP->RoT artificial flow-control threshold (0 = disabled). See the
    // phase-1 lockstep break in the SP burst below and config::sprot_flowctl.
    let sprot_flowctl = crate::config::get().sprot_flowctl as usize;
    // sprot SysTick coupling: while the SP is blocked on an sprot request
    // the RoT has accepted, pace the SP's SysTick by the RoT's elapsed 1ms tick
    // events so the SP's sprot timeout doesn't out-run the slow emulated RoT.
    let sprot_couple = crate::config::get().sprot_couple;
    let coupledbg = crate::config::get().coupledbg;
    eprintln!(
        "[gdb] eth-service: quantum={} txbreak={} sprot_flowctl={} sprot_couple={}",
        quantum, txbreak, sprot_flowctl, sprot_couple
    );

    // Production/in-zone mode (SP_EMU_NO_DEBUG): skip the SWD debug listener
    // entirely — MGS only needs the bridge UDP. Otherwise bind it.
    let listeners = if crate::config::get().no_debug {
        eprintln!("[gdb] debug servers disabled (SP_EMU_NO_DEBUG) — serving the bridge only");
        None
    } else {
        // Per-instance port so every sp-emu in a shared switch zone is debuggable
        // simultaneously: offset the SWD port by the bridge port (33300->0,
        // 33310->10, ...), so swd = 4444 + off. Pair with the matching selector.
        let off: u16 = crate::config::get()
            .bridge
            .as_deref()
            .and_then(|b| b.rsplit(':').next())
            .and_then(|p| p.parse::<u16>().ok())
            .map(|p| p.wrapping_sub(33300))
            .unwrap_or(0);
        let swd_port = 4444u16.wrapping_add(off);
        let swd_l = TcpListener::bind(("127.0.0.1", swd_port))?;
        swd_l.set_nonblocking(true)?;
        eprintln!("[gdb] ready (swd :{swd_port}). attach with:");
        eprintln!("[gdb]   humility -a <archive.zip> -p 20b7:9db1:tcp:127.0.0.1:{swd_port} <cmd>   (Glasgow SWD debug port: halt/run/hiffy; stock humility)");
        Some(swd_l)
    };

    // Pump-cadence diagnostics (SP_EMU_PUMPSTATS): distinguishes the SP being
    // descheduled by the host from the SP running a long batch. For each gap
    // between bridge pumps, log the wall-clock elapsed and the instructions
    // executed. A long gap with ~quantum instructions = the SP ran a full batch
    // (a smaller quantum / TX-break helps); a long gap with ~0 instructions = the
    // OS descheduled the whole process (only CPU priority helps, not the quantum).
    // Logged only for gaps over the threshold (default 50ms).
    let pumpstats = crate::config::get().pumpstats;
    let pump_thresh_us: u128 = crate::config::get().pumpstats_ms as u128 * 1000;
    let mut last_pump = std::time::Instant::now();
    let mut last_cyc = cpu.cycles;

    // Guest-PC sampling profiler (SP_EMU_PCPROF): histogram the executing PC to
    // find hot firmware (e.g. an SPI/IPC spin loop behind bulk-ignition latency).
    // Sampled every 256 instrs; cumulative top-30 dumped every 15s. Map PCs to
    // functions offline with the Hubris archive (addr2line/nm).
    let pcprof = crate::config::get().pcprof;
    let mut pchist: std::collections::HashMap<u32, u64> = std::collections::HashMap::new();
    let mut pcprof_samp: u64 = 0;
    let mut pcprof_last = std::time::Instant::now();

    // On-demand crash dump (SP_EMU_DUMP_DIR): when the file `<dir>/.trigger`
    // appears, write a humility-hydrate-compatible RAM dump to <dir> and swap the
    // trigger for `.done`. Reads a wedged SP's task table with no probe:
    //   touch <dir>/.trigger; zip <dir>; humility -a <ar> hydrate; humility -d tasks
    let dump_dir = crate::config::get().dump_dir.clone();
    let dump_archive_id = crate::config::get().dump_archive_id.clone();
    let mut dump_last = std::time::Instant::now();
    // Previous rot-irq level, for edge-detecting ROT_IRQ to raise the SP's EXTI.
    let mut prev_rot_irq = false;
    // Shared-RoT IPC state (SP_EMU_ROT_SERVICE mode): accumulate the request the
    // SP clocks out; `awaiting_reply` is set while a reply sits in `miso` for the
    // SP's phase-2 read.
    let mut req_buf: Vec<u8> = Vec::new();
    let mut awaiting_reply = false;
    // Synthetic one-shot RoT measurement trigger (SP_EMU_SWD_TRIGGER): pend the
    // RoT's sp_reset-irq once, to exercise the RoT-drives-SP-SWD path without a
    // full SP self-reset (whose measurement gate depends on the SP image).
    let swd_trigger = crate::config::get().swd_trigger;
    let mut swd_triggered = false;
    let jtag_trigger = crate::config::get().jtag_trigger;
    let mut jtag_triggered = false;
    // The SP's debug port, driven by the in-process RoT over the internal SWD link.
    // Loop-lifetime so its DP/AP/CoreDebug state persists across transactions.
    let mut sp_swdp = crate::debugport::SwDp::new();

    // sprot coupling state: the RoT's tick_events at the last iteration,
    // to compute per-iteration RoT elapsed-time delta. CREDIT_CAP bounds the SP's
    // owed-tick bucket so a stuck-true request_in_flight can't accumulate unbounded
    // credit (a few thousand ticks = a few seconds; not a wedge cap -- the
    // request_in_flight gate handles wedges).
    let mut prev_rot_ticks = rot.as_ref().map(|(rc, _)| rc.tick_events).unwrap_or(0);
    let mut prev_req_dbg = false; // coupledbg: edge-detect request_in_flight for the trace
    const CREDIT_CAP: u32 = 5000;

    // Prompt-halt servicing: while an armed RoT is halting the SP after a self-reset,
    // freeze the SP (burst 0) and give the RoT the full budget so it halts the SP
    // before it runs significant reset-vector work. Bounded by `sp_reset_halt_iters`.
    let mut sp_reset_halt_pending = false;
    let mut sp_reset_halt_iters: u32 = 0;

    loop {
        if let Some(swd_l) = &listeners {
            match swd_l.accept() {
                Ok((stream, peer)) => {
                    eprintln!("[gdb] SWD (Glasgow applet) client {peer}");
                    // A debug probe attached to the SP: assert SP_TO_ROT_JTAG_DETECT_L
                    // so the RoT invalidates its attestation log. The RoT is frozen for
                    // the whole glasgow::serve call below, so deliver the edge and step
                    // the RoT to service it here, before handing over the SP.
                    if let Some(l) = crate::sprot::link() {
                        l.borrow_mut().jtag_detect = true;
                    }
                    if let Some((rc, rb)) = rot.as_mut() {
                        if inject_jtag_detect(rb) {
                            pump_rot_briefly(rc, rb, host, quantum);
                        }
                    }
                    // Restore the running-halt state on disconnect so the main
                    // loop resumes the SP after a humility command detaches.
                    let r = crate::glasgow::serve(stream, &mut cpu, &mut bus, host);
                    cpu.halted = false;
                    cpu.debug_en = false;
                    cpu.bkpt_hit = false;
                    // Probe detached: deassert JTAG_DETECT (PIO0_20 level back high). No
                    // edge/IRQ on release; the firmware handler is falling-edge only.
                    if let Some(l) = crate::sprot::link() {
                        l.borrow_mut().jtag_detect = false;
                    }
                    if let Err(e) = r {
                        eprintln!("[gdb] SWD connection ended: {e}");
                    }
                    continue;
                }
                Err(e) if e.kind() != std::io::ErrorKind::WouldBlock => return Err(e.into()),
                Err(_) => {}
            }
        }
        // No client waiting: run the SP so time advances between commands. Stop
        // the batch early if the SP goes idle (WFI with nothing pending), to sleep
        // below instead of spinning through idle nops.
        // During a reply (phase 2), the RoT has asserted rot-irq and the SP clocks
        // the response back in one CS-asserted window. The response can be much
        // larger than the 16-byte FIFO (e.g. a 512-byte CMPA page), so the RoT must
        // keep refilling `miso` as the SP drains it. To interleave that, run the SP
        // in small bursts (not a full quantum) while a reply is in flight, and
        // yield the instant `miso` drains so the SP never clocks past what the RoT
        // has produced (reading an empty miso feeds zeros into the response and
        // corrupts its CRC). Gated on rot_irq so the request phase (phase 1), where
        // miso is just primed zeros, is unaffected.
        let replying = crate::sprot::link()
            .map(|l| {
                let l = l.borrow();
                l.rot_irq && l.cs
            })
            .unwrap_or(false);
        // While the SP is halted through its debug port (the RoT drove DHCSR.C_HALT,
        // or a vector catch stopped it at the reset vector), do not step it: the
        // core is stopped and only the RoT, over SWD, may move it. Skipping the
        // burst (not just `cpu.step`) also avoids ticking systick / taking
        // interrupts on a halted core.
        //
        // When the RoT is running an injected program under debug (C_DEBUGEN set,
        // not halted -- i.e. endoscope after the RoT resumed it), let the SP sprint
        // in a large burst so it reaches its terminal BKPT within the RoT's 500ms
        // halt-poll timeout. The RoT's timer only advances while the RoT itself
        // runs, so the SP finishing the flash measurement here costs little
        // RoT-time; without it the RoT times out (DidNotHalt) before the SP halts.
        let sp_burst = sp_burst_for(
            cpu.halted,
            sp_reset_halt_pending,
            cpu.debug_en,
            replying,
            quantum,
        );
        for _ in 0..sp_burst {
            // Stop the burst the instant the SP halts (e.g. endoscope's terminal
            // BKPT), so the debug-run burst above doesn't spin on the BKPT.
            if cpu.halted {
                break;
            }
            if cpu.step(&mut bus, host).is_err() {
                break;
            }
            // Firmware wrote AIRCR.SYSRESETREQ during that step: stop the burst so
            // the self-reset is applied below, outside the loop.
            if bus.reset_pending {
                break;
            }
            if pcprof {
                pcprof_samp = pcprof_samp.wrapping_add(1);
                if pcprof_samp & 0xFF == 0 {
                    *pchist.entry(cpu.pc).or_insert(0) += 1;
                }
            }
            cpu.maybe_tick(&mut bus);
            cpu.maybe_interrupt(&mut bus);
            if cpu.idle_skip > 0 {
                break;
            }
            // sprot lockstep. On hardware the SP TX FIFO and the RoT RX FIFO are
            // separate devices joined only by the SPI wires -- there is NO flow
            // control, the SP clocks at its own pace and the RoT must keep up. The
            // emulator collapses both into the one shared `mosi`/`miso` buffer and
            // runs the SP a whole quantum before the RoT ever steps, so a >16-byte
            // request would overrun the RoT's 16-deep RX FIFO and truncate the frame
            // (the cause of "sprot: timeout" on RoT/stage0 updates). These two breaks
            // add a deliberately non-physical flow-control signal to compensate for
            // that coarse scheduling; they are not on the schematic.
            if let Some(l) = crate::sprot::link() {
                let l = l.borrow();
                // Phase-2 (RoT->SP): the RoT is replying and its TX FIFO (miso) has
                // drained -- stop so the RoT can refill before the SP clocks more.
                if l.rot_irq && l.cs && l.miso.is_empty() {
                    break;
                }
                // Phase-1 (SP->RoT request): the SP has filled the shared buffer to
                // the RoT RX-FIFO depth -- stop so the RoT is stepped to drain FIFORD
                // before the next byte overruns it. Gate on !rot_irq: during a reply
                // (rot_irq asserted) the SP's TXDR clock-in dummies also land in
                // `mosi` but the RoT is sending, not draining, so applying this then
                // would wedge the SP mid-reply. `sprot_flowctl == 0` disables it.
                if sprot_flowctl != 0 && !l.rot_irq && l.mosi.len() >= sprot_flowctl {
                    break;
                }
            }
            // Flush the moment a reply is queued: the round-trip then costs ~one
            // pump instead of the rest of the quantum (matters most under load,
            // when the SP never goes idle so this is the only early break).
            if txbreak && bus.eth_has_tx() {
                break;
            }
        }
        // Apply a firmware system reset (AIRCR.SYSRESETREQ): re-boot the SP from its
        // slot-A vector table. This is the reset the SP does when the RFD 568
        // measurement token is absent; it also wakes the RoT to measure the SP.
        let mut sp_reset_edge = false;
        if bus.reset_pending {
            // Persist + latch any committed bank swap (a completed firmware
            // update) before reading the vector table, so the reboot enters the
            // now-active bank; then drop stale decodes for the swapped-in image.
            bus.flash_reset_latch();
            let sp = bus.read32(0x0800_0000);
            let pc = bus.read32(0x0800_0004) & !1;
            cpu.reset_for_reboot(sp, pc);
            // On silicon DEMCR.VC_CORERESET catches a reset from any source, a
            // firmware SYSRESETREQ included. If the RoT armed reset-and-halt, the SP
            // halts at its reset vector here (0 instructions); otherwise it falls to
            // the prompt-halt servicing window below.
            sp_swdp.honor_vector_catch(&mut cpu);
            cpu.flush_decode_cache();
            bus.reset_pending = false;
            sp_reset_edge = true;
        }
        bus.pump_eth(host);
        // host-sp-comms (UART7 / IPCC + host console): drain the SP's TX to the
        // host and feed host input into the SP's RX. Pumped here (not cycle-gated)
        // so it runs even on the idle path — a host byte injects into uart_rx,
        // collect_irqs pends IRQ 82, and the idle SP wakes (otherwise an idle WFI
        // would never see the RX and the channel would deadlock).
        bus.pump_uart(host);
        // Whether the RoT is mid-exchange (a request in flight or still building a
        // reply). When true, do not sleep the host below: an idle SP parked in
        // wait_rot_irq would otherwise pay a full idle_ms (~20ms) per poll cycle
        // while the RoT works, turning a sprot round-trip (read-cmpa, rot_boot_info)
        // into seconds. Sleep only when both cores are quiescent, which also keeps
        // the two-core instance's idle CPU low so its timeshare priority doesn't
        // decay (the cause of the multi-second `voxel sp state` latency).
        let mut rot_busy = false;
        // Step the in-process RoT core a quantum (it mostly idles, waking to
        // answer the SP over the sprot link).
        if let Some((rc, rb)) = rot.as_mut() {
            // Wake the RoT to measure the SP, exactly as real hardware reacts to an
            // SP reset: pend its sp_reset-irq (pint.irq0 = NVIC IRQ 4) and record a
            // falling edge on the SP_RESET PINT slot 0 (PINT.FALL @ 0x4000_4020), so
            // do_handle_sp_reset passes its pint_detect check and drives SWD instead
            // of returning "SpResetNotAsserted". Fired on a real SP self-reset, or
            // once via SP_EMU_SWD_TRIGGER to exercise the path when the SP image has
            // no measurement gate to self-reset it.
            let synthetic = swd_trigger && !swd_triggered && rb.irq_enabled(4);
            if sp_reset_edge || synthetic {
                rb.write32(0x4000_4020, 0x1); // PINT.FALL slot 0 = SP_RESET falling edge
                rb.pend_irq(4);
                swd_triggered = true;
            }
            // On a genuine self-reset, if the RoT is armed to service SP_RESET, freeze
            // the SP (below) until the RoT halts it, so it does not run reset-vector
            // work first. Gated on a real reset edge (not the synthetic trigger) and
            // skipped when a vector catch already halted the SP.
            if enter_sp_reset_service(sp_reset_edge, rb.irq_enabled(4), cpu.halted) {
                sp_reset_halt_pending = true;
                sp_reset_halt_iters = SP_RESET_HALT_ITERS;
                if coupledbg {
                    eprintln!(
                        "[reset] SP self-reset (real SYSRESETREQ), RoT armed: freezing the SP at its reset vector until the RoT halts it (backstop {SP_RESET_HALT_ITERS} iters)"
                    );
                }
            } else if sp_reset_edge && coupledbg {
                // A real self-reset that does not enter servicing: either a vector
                // catch already halted the SP (0 reset-vector instructions), or the RoT
                // has not armed SP_RESET yet (early boot), so the SP free-runs.
                if cpu.halted {
                    eprintln!("[reset] SP self-reset: vector-caught at the reset vector (0 reset-vector instrs)");
                } else {
                    eprintln!("[reset] SP self-reset: RoT not armed (SP_RESET IRQ disabled); SP free-runs (early boot)");
                }
            }
            // SP_EMU_JTAG_TRIGGER: fire one synthetic JTAG_DETECT edge once the RoT has
            // armed the IRQ, to exercise attestation-log invalidation without a real SWD
            // probe. The RoT is stepped normally here (no glasgow freeze), so it services
            // the pended IRQ on the following iterations; no pre-pump needed.
            if jtag_trigger && !jtag_triggered && inject_jtag_detect(rb) {
                jtag_triggered = true;
            }
            if rot_trace_from.is_some() {
                rc.record_disasm = true; // populate last_disasm for the window trace
            }
            // Wake the RoT's FLEXCOMM8 slave (irq 59) whenever it owes a receive —
            // i.e. an un-processed slave-select assert is latched (`ssa`) or a
            // transfer is active (`cs`). Keying off the latched `ssa`, not just
            // current CS, is required: the SP can assert->clock->deassert CS within
            // its own quantum, so CS is already de-asserted by now, yet the RoT
            // still owes the receive and is asleep in sys_recv_notification(SPI_IRQ)
            // — without the IRQ it sleeps forever and the request is never read.
            // Also wake the SP's spi-core (irq 84) while CS is asserted, since its
            // transfer loop sleeps when RX momentarily drains during a multi-FIFO
            // reply.
            // "sprot active" = an assert is latched (ssa), CS is asserted (cs), or a
            // reply is pending (rot_irq, waiting for the SP to clock phase 2).
            let (ssa_or_cs, cs_now, req_in_flight) = crate::sprot::link()
                .map(|l| {
                    let l = l.borrow();
                    (l.ssa || l.cs || l.rot_irq, l.cs, l.request_in_flight)
                })
                .unwrap_or((false, false, false));
            if ssa_or_cs {
                rb.pend_irq(59);
            }
            if cs_now {
                bus.pend_irq(84);
            }
            // Stay full-speed only during an actual exchange (clocking, or a request
            // being processed), not for the RoT's idle housekeeping, so the instance
            // sleeps when quiescent and keeps its scheduling priority.
            rot_busy = ssa_or_cs || req_in_flight || sp_reset_halt_pending;
            // Run the RoT many quanta back-to-back so it finishes a request's
            // handler in one go — IPC to update_server, up to 32 flash reads for a
            // CMPA page, building + CRCing the response — and asserts rot-irq before
            // the SP's response-wait times out. With one quantum per outer iteration
            // the SP's poll-timer out-ran the RoT on a large reply (read-cmpa), so the
            // SP saw a stale irq and retried until timeout. Stop the instant the RoT
            // idles (the common case, so no overhead at rest) or the reply is ready
            // (rot-irq asserted), so the SP isn't starved during phase-2 clocking
            // (where it ping-pongs with the SP one quantum at a time). Grant the big
            // back-to-back budget only while an exchange is happening; when idle, one
            // quantum per outer iteration keeps CPU near the baseline single-core
            // instance so the host scheduler doesn't decay this instance's priority.
            let rot_budget = if ssa_or_cs || req_in_flight || sp_reset_halt_pending {
                256
            } else {
                1
            };
            'rot_burst: for _ in 0..rot_budget {
                let mut rot_idled = false;
                for _ in 0..quantum {
                    let rpc = rc.pc; // pc of the instruction about to execute (for the window trace)
                    if let Err(t) = rc.step(rb, host) {
                        // A RoT task hitting an unimplemented/undecodable instruction
                        // would re-fault every quantum, silently wedged (the kernel
                        // never sees a fault exception here). Surface it once.
                        let tpc = t.pc();
                        if crate::sprot::dbg() && tpc != last_rottrap {
                            last_rottrap = tpc;
                            match &t {
                                crate::cpu::Trap::Unimplemented {
                                    pc,
                                    bytes,
                                    len,
                                    disasm,
                                } => eprintln!(
                                    "[rottrap] UNIMPL pc={:#010x} len={} bytes={:02x?} : {}",
                                    pc,
                                    len,
                                    &bytes[..(*len as usize).min(4)],
                                    disasm
                                ),
                                crate::cpu::Trap::Decode { pc } => {
                                    eprintln!("[rottrap] DECODE pc={:#010x}", pc)
                                }
                                crate::cpu::Trap::Halt { pc, why } => {
                                    eprintln!("[rottrap] HALT pc={:#010x} {}", pc, why)
                                }
                            }
                        }
                        break 'rot_burst;
                    }
                    if let (Some(f), Some(t)) = (rot_trace_from, rot_trace_to) {
                        if (f..=t).contains(&rpc) {
                            eprintln!(
                                "[rottrace] {:#010x}: {:<26} r0={:08x} r1={:08x} r2={:08x} r3={:08x} r6={:08x}",
                                rpc, rc.last_disasm, rc.r[0], rc.r[1], rc.r[2], rc.r[3], rc.r[6]
                            );
                        }
                    }
                    if crate::sprot::rot_trace_tick() {
                        eprintln!("[rottr] {:#010x}", rc.pc);
                    }
                    rc.maybe_tick(rb);
                    rc.maybe_interrupt(rb);
                    if rc.idle_skip > 0 {
                        rc.idle_skip = 0;
                        rot_idled = true;
                        break;
                    }
                }
                // Stop the extra-quanta burst once the RoT idles (nothing left to do)
                // or the reply is ready (rot-irq asserted) — then the SP runs phase 2.
                if rot_idled {
                    break;
                }
                // The RoT-side reply direction already has flow control: FIFOSTAT
                // (sprot.rs) reports TXNOTFULL gated on `miso` depth, so the RoT
                // firmware stalls itself when its TX FIFO is full, and the existing
                // rot-irq break lets the SP clock the reply out. (A symmetric
                // serve-loop break on `miso` full was tried but starved the RoT
                // mid-reply-construction -- before it asserts rot-irq the SP isn't
                // clocking, so a full buffer can't drain and the exchange hangs.)
                if crate::sprot::link()
                    .map(|l| l.borrow().rot_irq)
                    .unwrap_or(false)
                {
                    break;
                }
            }
            // Drain the internal SWD link: run each ADIv5 transaction the RoT
            // clocked out against the SP's debug port, and hand back a read
            // result. This is what lets the emulated RoT actually read/write the
            // SP over SWD (the RoT stalls on FIFOSTAT until a read result lands).
            if let Some(swd) = crate::rotswd::link() {
                loop {
                    let req = swd.borrow_mut().req.pop_front();
                    let Some(r) = req else { break };
                    if let crate::debugport::Ack::Ok(Some(d)) =
                        sp_swdp.transfer(&mut cpu, &mut bus, r.ap, r.rnw, r.a, r.wdata)
                    {
                        swd.borrow_mut().resp = Some(d);
                    }
                }
            }
            // The RoT released ROT_TO_SP_RESET_L (PIO0_13 low->high) in
            // `sp_reset_leave`: pulse the SP's reset through its debug port. Done
            // after draining the SWD link so the DHCSR/DEMCR writes that arm the
            // vector catch are already applied to `sp_swdp` -- the SP then halts at
            // its reset vector (DFSR.VCATCH), which is what reset_into_debug_halt
            // waits for before injecting endoscope.
            let sp_reset_released = crate::sprot::link()
                .map(|l| std::mem::take(&mut l.borrow_mut().sp_reset_release))
                .unwrap_or(false);
            if sp_reset_released {
                sp_swdp.pin_reset(&mut cpu, &mut bus);
            }
            // RoT PC sampling (SP_EMU_ROTPC=N): log the RoT pc every N instructions,
            // only while a sprot exchange is in flight (CS has been touched) to
            // bound the noise. Locates where the RoT wedges when it reads a request
            // but never replies.
            if let Some(n) = rotpc_every {
                if rc.cycles >= rotpc_next {
                    rotpc_next = rc.cycles + n;
                    eprintln!(
                        "[rotpc] pc={:#010x} lr={:#010x} sp={:#010x} cyc={}",
                        rc.pc, rc.r[14], rc.r[13], rc.cycles
                    );
                }
            }
            if let Some((addr, len)) = rotdump {
                if rotdump_last.elapsed().as_secs() >= 8 {
                    rotdump_last = std::time::Instant::now();
                    let mut a = addr;
                    while a < addr + len {
                        eprintln!(
                            "[rotdump] {:08x}: {:08x} {:08x} {:08x} {:08x}",
                            a,
                            rb.read32(a),
                            rb.read32(a + 4),
                            rb.read32(a + 8),
                            rb.read32(a + 12)
                        );
                        a += 16;
                    }
                }
            }
            // sprot SysTick coupling: while the SP is blocked on an sprot
            // request the RoT has accepted (request_in_flight), advance the SP's
            // SysTick by the RoT's elapsed 1ms tick events this iteration (1:1 -- both
            // kernels tick at 1ms) instead of the fabricated one-per-iteration idle
            // tick, so the SP's SysTick-paced sprot timeout counts down at the true
            // RoT-relative rate. Gate strictly on request_in_flight (NOT rot_busy,
            // which a wedge latches true via ssa) and skip while the SP is debug-halted
            // (its time is genuinely frozen then -- e.g. a RoT task-dump SWD read). A
            // wedged RoT never sets request_in_flight, so the SP falls to the normal
            // throttle and times out; no wedge-detector needed.
            // saturating_sub: tick_events is monotonic (not reset on RoT reboot), so
            // prev <= cur, but stay underflow-proof regardless. Cap the delta before
            // the u32 cast so a pathological value saturates rather than truncating,
            // then saturating_add + min(CREDIT_CAP) bound the bucket.
            let d_rot = rc.tick_events.saturating_sub(prev_rot_ticks);
            prev_rot_ticks = rc.tick_events;
            if sprot_couple && req_in_flight && !cpu.halted {
                let add = d_rot.min(CREDIT_CAP as u64) as u32;
                cpu.sp_tick_credit = cpu.sp_tick_credit.saturating_add(add).min(CREDIT_CAP);
                if coupledbg && (add > 0 || !prev_req_dbg) {
                    eprintln!(
                        "[couple] req_in_flight +{} credit={} sp_ticks={} rot_ticks={}",
                        add, cpu.sp_tick_credit, cpu.tick_events, rc.tick_events
                    );
                }
            } else {
                if coupledbg && prev_req_dbg {
                    eprintln!(
                        "[couple] exchange end: credit was {} sp_ticks={} rot_ticks={}",
                        cpu.sp_tick_credit, cpu.tick_events, rc.tick_events
                    );
                }
                cpu.sp_tick_credit = 0;
            }
            prev_req_dbg = req_in_flight;
            // Clear prompt-halt servicing once the RoT has taken control of the SP
            // (halted it over SWD, or resumed it under debug to run endoscope), or
            // when the safety bound expires; then the SP resumes normal scheduling.
            let was_servicing = sp_reset_halt_pending;
            sp_reset_halt_pending = continue_sp_reset_service(
                sp_reset_halt_pending,
                cpu.halted,
                cpu.debug_en,
                &mut sp_reset_halt_iters,
            );
            if coupledbg && was_servicing && !sp_reset_halt_pending {
                // The SP burst is 0 while servicing, so no reset-vector instructions
                // ran between the reset edge and this point; report how the RoT took
                // control and how many serve iterations it took.
                let iters = SP_RESET_HALT_ITERS - sp_reset_halt_iters;
                if cpu.halted {
                    eprintln!("[reset] RoT halted the SP after {iters} serve iterations, SP ran 0 reset-vector instrs (prompt)");
                } else if cpu.debug_en {
                    eprintln!("[reset] RoT resumed the SP under debug after {iters} serve iterations (endoscope), SP ran 0 reset-vector instrs");
                } else {
                    eprintln!("[reset] backstop: RoT did not halt within {SP_RESET_HALT_ITERS} iterations; SP resumes free-running");
                }
            }
        } else if let Some(client) = rot_client.as_mut() {
            // Shared-RoT IPC path: no in-process RoT core. Act as the SP's link
            // peer — accumulate the request the SP clocks out, ship it to the
            // shared rot-service on CS-deassert, stuff the reply into `miso` and
            // raise rot-irq (the EXTI block below wakes the SP). The 16-byte TX FIFO
            // requires draining `mosi` as the SP clocks, or a >16B request caps.
            if let Some(l) = crate::sprot::link() {
                let ssd = {
                    let mut lk = l.borrow_mut();
                    if awaiting_reply {
                        lk.mosi.clear(); // discard phase-2 dummy clocks
                    } else {
                        while let Some(b) = lk.mosi.pop_front() {
                            req_buf.push(b);
                        }
                    }
                    lk.ssd
                };
                if ssd && !awaiting_reply && !req_buf.is_empty() {
                    {
                        let mut lk = l.borrow_mut();
                        lk.ssa = false;
                        lk.ssd = false;
                        lk.sot_pending = false;
                    }
                    let resp = client.exchange(&req_buf);
                    req_buf.clear();
                    let mut lk = l.borrow_mut();
                    lk.miso.clear();
                    lk.miso.extend(resp);
                    lk.rot_irq = true; // EXTI block below pends irq 9 -> wakes SP
                    lk.request_in_flight = false;
                    awaiting_reply = true;
                } else if awaiting_reply && ssd {
                    // The SP deasserted CS after clocking in the reply -> this sprot
                    // transaction is complete. Deassert rot-irq and drop any unread
                    // reply bytes, so the next request the SP clocks (e.g. the
                    // caboose's multi-step follow-up read) is captured whole.
                    //
                    // Keying end-of-transaction on `miso.is_empty()` was a bug: if
                    // the SP left even one reply byte unread, `awaiting_reply` stuck
                    // true and the head of the next request got eaten by the phase-2
                    // `mosi.clear()` above -> a truncated request -> the RoT never
                    // sees a complete frame and grinds in its TX loop forever. The
                    // SP's CS edge is the protocol boundary; use it.
                    let mut lk = l.borrow_mut();
                    lk.rot_irq = false;
                    lk.ssa = false;
                    lk.ssd = false;
                    lk.miso.clear();
                    awaiting_reply = false;
                }
                // Keep the host full-speed while a request/reply is outstanding.
                rot_busy = awaiting_reply || !req_buf.is_empty();
            }
        }
        // ROT_IRQ -> SP EXTI: when the RoT toggles rot-irq (PE3 / EXTI line 3),
        // latch the SP's EXTI pending bit and pend the EXTI3 NVIC IRQ (9, routed to
        // the sys task's exti wildcard). The sys task then posts the ROT_IRQ
        // notification and sprot's wait_rot_irq returns at once, instead of waiting
        // out its fallback poll-timer (which made sprot round-trips slow).
        {
            let now_irq = crate::sprot::link()
                .map(|l| l.borrow().rot_irq)
                .unwrap_or(false);
            if now_irq != prev_rot_irq {
                prev_rot_irq = now_irq;
                if let Some(l) = crate::sprot::link() {
                    l.borrow_mut().sp_rot_irq_pending = true;
                }
                bus.pend_irq(9);
            }
        }
        if let Some(ref ddir) = dump_dir {
            if dump_last.elapsed().as_millis() >= 500 {
                dump_last = std::time::Instant::now();
                let trig = format!("{}/.trigger", ddir);
                if std::path::Path::new(&trig).exists() {
                    match bus.write_hydrate_dump(ddir, &dump_archive_id) {
                        Ok(_) => eprintln!("[dump] wrote hydrate RAM dump to {}", ddir),
                        Err(e) => eprintln!("[dump] FAILED: {}", e),
                    }
                    let _ = std::fs::remove_file(&trig);
                    let _ = std::fs::write(format!("{}/.done", ddir), b"done\n");
                }
            }
        }
        if pumpstats {
            let dt = last_pump.elapsed().as_micros();
            if dt >= pump_thresh_us {
                eprintln!(
                    "[pumpstats] gap={}us instrs={} ({:.2}M/s eff)",
                    dt,
                    cpu.cycles - last_cyc,
                    (cpu.cycles - last_cyc) as f64 / (dt as f64 / 1e6) / 1e6
                );
            }
            last_pump = std::time::Instant::now();
            last_cyc = cpu.cycles;
        }
        if pcprof && pcprof_last.elapsed().as_secs() >= 15 {
            let total: u64 = pchist.values().sum();
            let mut v: Vec<(u64, u32)> = pchist.iter().map(|(&pc, &c)| (c, pc)).collect();
            v.sort_unstable_by(|a, b| b.0.cmp(&a.0));
            eprintln!("[pcprof] total_samples={} (every 256th instr) top:", total);
            for (c, pc) in v.iter().take(30) {
                eprintln!(
                    "[pcprof] {:#010x} {} ({:.1}%)",
                    pc,
                    c,
                    *c as f64 * 100.0 / total.max(1) as f64
                );
            }
            pcprof_last = std::time::Instant::now();
        }
        // Sleep only when genuinely idle: the SP hit WFI with nothing pending and
        // pump_eth didn't just inject an MGS packet to handle. Under real MGS load
        // (continuous sensor polling) there's almost always a pending packet, so
        // the loop stays full-speed and responsive; sleep only when MGS is quiet.
        if cpu.idle_skip > 0 {
            cpu.idle_skip = 0;
            // Don't sleep while host-UART (IPCC / host console) input is pending:
            // pump_uart just injected it into uart_rx but collect_irqs hasn't run
            // yet, so any_pending_irq() doesn't see IRQ 82 here. Sleeping would
            // stall the SP for idle_ms per byte -- and host_sp_comms only services
            // the UART when it is back in sys_recv, not while blocked calling
            // gimlet_seq, so the SP must keep running for both to make progress.
            let host_uart_pending = !bus.uart_rx.borrow().is_empty();
            if !bus.any_pending_irq() && !rot_busy && !host_uart_pending {
                std::thread::sleep(std::time::Duration::from_millis(idle_ms));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const Q: u32 = 340;

    #[test]
    fn inject_jtag_detect_gated_on_armed_and_preserves_sp_reset() {
        let mut bus = Bus::new();
        bus.log_unmapped = false;
        bus.add_ram(0x4000_4000, 0x100); // stand in for the flat PINT RegFile
        bus.write32(0x4000_4020, 0x1); // a SP_RESET (slot 0) edge already latched

        // Firmware without JTAG_DETECT (IRQ 5 not enabled): a no-op that disturbs
        // nothing, so an older RoT image is unaffected.
        assert!(!inject_jtag_detect(&mut bus));
        assert_eq!(bus.read32(0x4000_4020), 0x1);
        assert_eq!(bus.next_irq(), None);

        // Firmware arms JTAG_DETECT (NVIC ISER0 bit 5 = IRQ 5).
        bus.write32(0xE000_E100, 1 << JTAG_DETECT_IRQ);
        assert!(inject_jtag_detect(&mut bus));
        assert_eq!(
            bus.read32(0x4000_4020),
            0x3,
            "slot-1 bit set, coincident slot-0 (SP_RESET) preserved"
        );
        assert_eq!(
            bus.next_irq(),
            Some(JTAG_DETECT_IRQ),
            "IRQ 5 pended + enabled"
        );
    }

    #[test]
    fn sp_burst_priority_order() {
        // Halted wins over everything: a halted core is not stepped.
        assert_eq!(sp_burst_for(true, true, true, true, Q), 0);
        // Servicing a self-reset freezes the SP even though it is not halted and not
        // in a reply, and outranks the endoscope debug sprint.
        assert_eq!(sp_burst_for(false, true, true, false, Q), 0);
        // Endoscope: running an injected program under debug sprints to its BKPT.
        assert_eq!(sp_burst_for(false, false, true, false, Q), 20_000_000);
        // Phase-2 sprot reply: small burst so the SP never outruns the RoT refill.
        assert_eq!(sp_burst_for(false, false, false, true, Q), 48);
        // Otherwise the full eth-service quantum.
        assert_eq!(sp_burst_for(false, false, false, false, Q), Q);
    }

    #[test]
    fn enter_service_only_when_armed_edge_and_not_halted() {
        // Genuine reset edge, RoT armed, SP still running: enter servicing.
        assert!(enter_sp_reset_service(true, true, false));
        // RoT not yet armed (early boot): the SP's measurement loop must free-run.
        assert!(!enter_sp_reset_service(true, false, false));
        // No reset this iteration.
        assert!(!enter_sp_reset_service(false, true, false));
        // A vector catch already halted the SP: no servicing needed.
        assert!(!enter_sp_reset_service(true, true, true));
    }

    #[test]
    fn service_stays_until_rot_takes_control() {
        let mut iters = SP_RESET_HALT_ITERS;
        // Still waiting (RoT has not halted or resumed the SP): stays active, and the
        // safety countdown ticks down.
        assert!(continue_sp_reset_service(true, false, false, &mut iters));
        assert_eq!(iters, SP_RESET_HALT_ITERS - 1);
        // The RoT halted the SP over SWD: servicing ends.
        assert!(!continue_sp_reset_service(true, true, false, &mut iters));
        // The RoT resumed the SP under debug (endoscope): servicing ends.
        assert!(!continue_sp_reset_service(true, false, true, &mut iters));
    }

    #[test]
    fn service_backstop_clears_when_bound_exhausted() {
        // The RoT armed SP_RESET but never halts the SP: after the bound, give up so
        // the SP resumes free-running rather than wedging.
        let mut iters = 1;
        assert!(!continue_sp_reset_service(true, false, false, &mut iters));
        assert_eq!(iters, 0);
        // And once inactive it stays inactive.
        assert!(!continue_sp_reset_service(false, false, false, &mut iters));
    }
}
