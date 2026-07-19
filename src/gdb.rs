//! Minimal GDB Remote Serial Protocol (RSP) server for humility to attach to
//! the running emulated SP and inspect it.
//!
//! humility's GDB core (humility-probes-core/src/gdb.rs) connects to
//! 127.0.0.1:3333, speaks RSP, and is read-only — it sends `qSupported`,
//! `m addr,len` (read memory), `p reg` (read register), `c` (continue), and a
//! raw 0x03 byte (halt). It never writes target state. A server answering
//! those reads lets `humility tasks` / `dump` / `ringbuf` work against sp-emu.
//!
//! Ack discipline (matched to humility's `recv`):
//!  - a data response to a `$..#xx` command is `+` then `$payload#xx`,
//!  - `c` (continue) is answered with a bare `+`,
//!  - the 0x03 halt is answered with a stop reply `$S05#xx` (no leading `+`).

use crate::cpu::Cpu;
use crate::host::HostIo;
use crate::mem::Bus;
use anyhow::Result;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};

/// One message pulled off the wire.
enum Msg {
    Interrupt,      // raw 0x03 — halt request
    Packet(String), // a `$payload#xx` command (payload only)
}

/// Pull the next complete message out of `buf`, discarding bare `+`/`-` acks and
/// junk. Returns None if only a partial packet is buffered (wait for more bytes).
fn take_message(buf: &mut Vec<u8>) -> Option<Msg> {
    loop {
        match buf.first().copied() {
            None => return None,
            Some(0x03) => {
                buf.remove(0);
                return Some(Msg::Interrupt);
            }
            Some(b'+') | Some(b'-') => {
                buf.remove(0);
            } // ack — skip
            Some(b'$') => {
                let hash = buf.iter().position(|&c| c == b'#')?;
                if buf.len() < hash + 3 {
                    return None;
                } // checksum bytes not here yet
                let payload = String::from_utf8_lossy(&buf[1..hash]).into_owned();
                buf.drain(0..hash + 3);
                return Some(Msg::Packet(payload));
            }
            Some(_) => {
                buf.remove(0);
            } // stray byte — skip
        }
    }
}

/// Frame a payload as `$payload#xx` with the RSP modulo-256 checksum.
fn frame(payload: &str) -> Vec<u8> {
    let cksum: u32 = payload.bytes().map(|b| b as u32).sum::<u32>() & 0xff;
    format!("${}#{:02x}", payload, cksum).into_bytes()
}

/// 32-bit value as little-endian hex (target byte order), the way GDB sends
/// register/word values: 0x12345678 -> "78563412".
fn le_hex32(v: u32) -> String {
    v.to_le_bytes()
        .iter()
        .map(|b| format!("{:02x}", b))
        .collect()
}

/// Reply with data: ack the command (`+`) then the response packet.
fn reply_data(s: &mut TcpStream, payload: &str) -> Result<()> {
    s.write_all(b"+")?;
    s.write_all(&frame(payload))?;
    Ok(())
}

fn parse_m(p: &str) -> Option<(u32, u32)> {
    let rest = &p[1..];
    let (a, l) = rest.split_once(',')?;
    Some((
        u32::from_str_radix(a, 16).ok()?,
        u32::from_str_radix(l, 16).ok()?,
    ))
}

/// Handle one command packet. Returns true if the target should resume running.
fn handle(s: &mut TcpStream, p: &str, cpu: &Cpu, bus: &mut Bus) -> Result<bool> {
    match p.as_bytes().first().copied() {
        Some(b'q') => {
            if p.starts_with("qSupported") {
                reply_data(s, "PacketSize=4000")?;
            } else if p.starts_with("qAttached") {
                reply_data(s, "1")?;
            } else if p == "qC" {
                reply_data(s, "QC1")?;
            } else if p.starts_with("qfThreadInfo") {
                reply_data(s, "m1")?;
            } else if p.starts_with("qsThreadInfo") {
                reply_data(s, "l")?;
            } else {
                reply_data(s, "")?;
            }
        }
        Some(b'?') => reply_data(s, "S05")?, // last-stop reason
        Some(b'm') => {
            let payload = match parse_m(p) {
                Some((addr, len)) => (0..len)
                    .map(|i| format!("{:02x}", bus.read8(addr.wrapping_add(i))))
                    .collect::<String>(),
                None => "E01".to_string(),
            };
            reply_data(s, &payload)?;
        }
        Some(b'p') => {
            let reg = u16::from_str_radix(&p[1..], 16).unwrap_or(0xffff);
            reply_data(s, &le_hex32(cpu.gdb_reg(reg)))?;
        }
        Some(b'g') => {
            // r0..r15 then xPSR — the core general-register block GDB expects.
            let mut out = String::new();
            for n in 0..16 {
                out.push_str(&le_hex32(cpu.gdb_reg(n)));
            }
            out.push_str(&le_hex32(cpu.gdb_reg(16)));
            reply_data(s, &out)?;
        }
        Some(b'c') => {
            s.write_all(b"+")?;
            return Ok(true);
        } // continue: bare ack, resume
        // Unsupported (vCont, H thread-select, write packets, ...): empty reply.
        _ => reply_data(s, "")?,
    }
    Ok(false)
}

// ---- OpenOCD Tcl RPC (humility's `-p ocd` core: reads and writes) -----------
//
// Protocol: each command and each reply is terminated by a 0x1a byte. humility
// treats a reply containing "Error: " / "invalid command name " as failure.
// Commands used: `version`, `mrw addr` (read word, decimal), `array unset
// output` + `mem2array output 8 addr len` + `return $output` (read bytes as
// space-separated `idx val` decimal pairs), `reg N` (read register), `mww/mwb
// addr val` (write). halt/run are client-side no-ops, so the target stays
// frozen for the duration of an OpenOCD connection (a consistent snapshot).

const OCD_DELIM: u8 = 0x1a;

fn parse_hex_or_dec(s: &str) -> Option<u32> {
    let s = s.trim();
    if let Some(h) = s.strip_prefix("0x") {
        u32::from_str_radix(h, 16).ok()
    } else {
        s.parse().ok()
    }
}

/// Produce the reply body for one OpenOCD Tcl command. `pending` carries the
/// `mem2array` result across to the following `return $output`.
fn handle_ocd(cmd: &str, cpu: &Cpu, bus: &mut Bus, pending: &mut String) -> String {
    let mut t = cmd.split_whitespace();
    match t.next() {
        Some("version") => "Open On-Chip Debugger 0.12.0 (sp-emu)".to_string(),
        Some("mrw") => match t.next().and_then(parse_hex_or_dec) {
            Some(addr) => format!("{}", bus.read32(addr)), // decimal, per humility
            None => String::new(),
        },
        Some("mem2array") => {
            // mem2array output 8 <addr> <len>
            let _var = t.next();
            let _width = t.next();
            let addr = t.next().and_then(parse_hex_or_dec).unwrap_or(0);
            let len = t.next().and_then(parse_hex_or_dec).unwrap_or(0);
            // Stash "idx val idx val ..." (decimal) for the next `return $output`.
            let mut s = String::new();
            for i in 0..len {
                if i > 0 {
                    s.push(' ');
                }
                s.push_str(&format!("{} {}", i, bus.read8(addr.wrapping_add(i))));
            }
            *pending = s;
            String::new()
        }
        Some("return") => std::mem::take(pending), // `return $output`
        Some("reg") => match t.next().and_then(|n| n.parse::<u16>().ok()) {
            Some(n) => format!("reg{} (/32): 0x{:08x}", n, cpu.gdb_reg(n)),
            None => String::new(),
        },
        Some("mww") => {
            let addr = t.next().and_then(parse_hex_or_dec);
            let val = t.next().and_then(parse_hex_or_dec);
            if let (Some(a), Some(v)) = (addr, val) {
                bus.write32(a, v);
            }
            String::new()
        }
        Some("mwb") => {
            let addr = t.next().and_then(parse_hex_or_dec);
            let val = t.next().and_then(parse_hex_or_dec);
            if let (Some(a), Some(v)) = (addr, val) {
                bus.write8(a, v as u8);
            }
            String::new()
        }
        // array unset / capture / unknown: succeed silently (no "Error:").
        _ => String::new(),
    }
}

/// Serve one OpenOCD-Tcl connection to completion (target frozen throughout).
fn serve_ocd(mut stream: TcpStream, cpu: &Cpu, bus: &mut Bus) -> Result<()> {
    // Accepted sockets can inherit the listener's non-blocking flag; this loop
    // uses blocking reads, so force it.
    stream.set_nonblocking(false)?;
    let mut buf: Vec<u8> = Vec::new();
    let mut rbuf = [0u8; 8192];
    let mut pending = String::new();
    loop {
        // Accumulate a 0x1a-delimited command.
        while !buf.contains(&OCD_DELIM) {
            let n = stream.read(&mut rbuf)?;
            if n == 0 {
                return Ok(());
            } // client closed
            buf.extend_from_slice(&rbuf[..n]);
        }
        let pos = buf.iter().position(|&b| b == OCD_DELIM).unwrap();
        let cmd = String::from_utf8_lossy(&buf[..pos]).into_owned();
        buf.drain(0..=pos);
        let mut reply = handle_ocd(cmd.trim(), cpu, bus, &mut pending).into_bytes();
        reply.push(OCD_DELIM);
        stream.write_all(&reply)?;
    }
}

/// Serve one GDB-RSP connection to completion.
fn serve_gdb(
    mut stream: TcpStream,
    cpu: &mut Cpu,
    bus: &mut Bus,
    host: &mut dyn HostIo,
) -> Result<()> {
    stream.set_nonblocking(true)?;
    stream.set_nodelay(true).ok();
    let mut inbuf: Vec<u8> = Vec::new();
    let mut rbuf = [0u8; 8192];
    let mut running = false;
    loop {
        match stream.read(&mut rbuf) {
            Ok(0) => return Ok(()),
            Ok(n) => inbuf.extend_from_slice(&rbuf[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(e) => return Err(e.into()),
        }
        while let Some(msg) = take_message(&mut inbuf) {
            match msg {
                Msg::Interrupt => {
                    running = false;
                    stream.write_all(&frame("S05"))?;
                }
                Msg::Packet(p) => {
                    running = handle(&mut stream, &p, cpu, bus)?;
                }
            }
        }
        if running {
            for _ in 0..50_000 {
                if cpu.step(bus, host).is_err() {
                    running = false;
                    break;
                }
                cpu.maybe_tick(bus);
                cpu.maybe_interrupt(bus);
            }
            bus.pump_eth(host);
        } else {
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
    }
}

/// Pre-boot to steady state, then serve both humility debug transports:
///  - GDB RSP on :3333  (`humility -p ocdgdb`, read-only)
///  - OpenOCD Tcl on :6666 (`humility -p ocd`, reads + writes)
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
    let parse_env = |k: &str| std::env::var(k).ok().and_then(|s| s.parse::<u64>().ok());
    let (twin_from, twin_to) = (parse_env("SP_EMU_TRACE_FROM"), parse_env("SP_EMU_TRACE_TO"));
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
        for _ in 0..40_000_000u64 {
            if let Err(t) = rc.step(rb, host) {
                if dbgtrap {
                    eprintln!("[rottrap-preboot] {:?}", t);
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
        rc.trace_svc = std::env::var("SP_EMU_ROTSVC").is_ok();
        eprintln!(
            "[rot] RoT core booted (pc={:#010x}, {} insns) in {:.2}s",
            rc.pc,
            rc.cycles,
            t.elapsed().as_secs_f64()
        );
    }

    let rotpc_every = parse_env("SP_EMU_ROTPC");
    let mut rotpc_next = 0u64;
    let mut last_rottrap = u32::MAX;
    // SP_EMU_ROTDUMP="0xADDR:LEN" dumps that RoT RAM range every ~8s for task-table introspection.
    let rotdump: Option<(u32, u32)> = std::env::var("SP_EMU_ROTDUMP").ok().and_then(|s| {
        let (a, l) = s.split_once(':')?;
        Some((
            u32::from_str_radix(a.trim_start_matches("0x"), 16).ok()?,
            l.parse().ok()?,
        ))
    });
    let mut rotdump_last = std::time::Instant::now();

    // Post-preboot: enable the WFI idle-throttle so an idle SP sleeps the host
    // instead of pegging a core (preboot ran with it off, at full spin speed).
    cpu.wfi_throttle = true;
    // Per idle WFI, sleep this long (ms) instead of spinning. Larger = lower
    // idle CPU but slower background sim-time; an incoming packet's eth-irq wakes
    // the SP immediately regardless. 10ms ≈ 4x CPU cut; tune via SP_EMU_IDLE_MS
    // for denser fleets.
    let idle_ms: u64 = std::env::var("SP_EMU_IDLE_MS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(10);

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
    let quantum: u32 = std::env::var("SP_EMU_ETH_QUANTUM")
        .ok()
        .and_then(|s| s.parse().ok())
        .filter(|&q| q > 0)
        .unwrap_or(4096);
    // TX-break: end the batch the instant the SP queues a reply so it flushes
    // immediately instead of waiting out the rest of the quantum. On by default;
    // SP_EMU_ETH_TXBREAK=0 disables it (A/B against the once-per-batch behavior).
    let txbreak = std::env::var("SP_EMU_ETH_TXBREAK")
        .map(|v| v != "0")
        .unwrap_or(true);
    eprintln!("[gdb] eth-service: quantum={} txbreak={}", quantum, txbreak);

    // Production/in-zone mode (SP_EMU_NO_DEBUG): skip the gdb/ocd debug listeners
    // entirely — MGS only needs the bridge UDP. Otherwise bind them.
    let listeners = if std::env::var("SP_EMU_NO_DEBUG").is_ok() {
        eprintln!("[gdb] debug servers disabled (SP_EMU_NO_DEBUG) — serving the bridge only");
        None
    } else {
        // Per-instance ports so every sp-emu in a shared switch zone is
        // debuggable simultaneously: offset by the bridge port (33300->0,
        // 33310->10, ...). gdb=3333+off, ocd=6666+off. Pair with humility's
        // HUMILITY_OCD_PORT env to attach to a specific SP.
        let off: u16 = std::env::var("SP_EMU_BRIDGE")
            .ok()
            .and_then(|b| b.rsplit(':').next().map(str::to_string))
            .and_then(|p| p.parse::<u16>().ok())
            .map(|p| p.wrapping_sub(33300))
            .unwrap_or(0);
        let gdb_port = 3333u16.wrapping_add(off);
        let ocd_port = 6666u16.wrapping_add(off);
        let swd_port = 4444u16.wrapping_add(off);
        let gdb_l = TcpListener::bind(("127.0.0.1", gdb_port))?;
        let ocd_l = TcpListener::bind(("127.0.0.1", ocd_port))?;
        let swd_l = TcpListener::bind(("127.0.0.1", swd_port))?;
        gdb_l.set_nonblocking(true)?;
        ocd_l.set_nonblocking(true)?;
        swd_l.set_nonblocking(true)?;
        eprintln!("[gdb] ready (gdb :{gdb_port}, ocd :{ocd_port}, swd :{swd_port}). attach with:");
        eprintln!("[gdb]   humility -a <archive.zip> -p ocdgdb <cmd>   (reads: tasks, readmem, ringbuf, ...)");
        eprintln!("[gdb]   humility -a <archive.zip> -p ocd    <cmd>   (reads + writes: writemem, hiffy, ...)");
        eprintln!("[gdb]   humility -a <archive.zip> -p 20b7:9db1:tcp:127.0.0.1:{swd_port} <cmd>   (real SWD debug port: halt/run/hiffy)");
        Some((gdb_l, ocd_l, swd_l))
    };

    // Pump-cadence diagnostics (SP_EMU_PUMPSTATS): distinguishes the SP being
    // descheduled by the host from the SP running a long batch. For each gap
    // between bridge pumps, log the wall-clock elapsed and the instructions
    // executed. A long gap with ~quantum instructions = the SP ran a full batch
    // (a smaller quantum / TX-break helps); a long gap with ~0 instructions = the
    // OS descheduled the whole process (only CPU priority helps, not the quantum).
    // Logged only for gaps over the threshold (default 50ms).
    let pumpstats = std::env::var("SP_EMU_PUMPSTATS").is_ok();
    let pump_thresh_us: u128 = std::env::var("SP_EMU_PUMPSTATS_MS")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(50)
        * 1000;
    let mut last_pump = std::time::Instant::now();
    let mut last_cyc = cpu.cycles;

    // Guest-PC sampling profiler (SP_EMU_PCPROF): histogram the executing PC to
    // find hot firmware (e.g. an SPI/IPC spin loop behind bulk-ignition latency).
    // Sampled every 256 instrs; cumulative top-30 dumped every 15s. Map PCs to
    // functions offline with the Hubris archive (addr2line/nm).
    let pcprof = std::env::var("SP_EMU_PCPROF").is_ok();
    let mut pchist: std::collections::HashMap<u32, u64> = std::collections::HashMap::new();
    let mut pcprof_samp: u64 = 0;
    let mut pcprof_last = std::time::Instant::now();

    // On-demand crash dump (SP_EMU_DUMP_DIR): when the file `<dir>/.trigger`
    // appears, write a humility-hydrate-compatible RAM dump to <dir> and swap the
    // trigger for `.done`. Reads a wedged SP's task table with no probe:
    //   touch <dir>/.trigger; zip <dir>; humility -a <ar> hydrate; humility -d tasks
    let dump_dir = std::env::var("SP_EMU_DUMP_DIR").ok();
    let dump_archive_id = std::env::var("SP_EMU_DUMP_ARCHIVE_ID").unwrap_or_default();
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
    let swd_trigger = std::env::var("SP_EMU_SWD_TRIGGER").is_ok();
    let mut swd_triggered = false;

    loop {
        if let Some((gdb_l, ocd_l, swd_l)) = &listeners {
            match gdb_l.accept() {
                Ok((stream, peer)) => {
                    eprintln!("[gdb] RSP client {peer}");
                    if let Err(e) = serve_gdb(stream, &mut cpu, &mut bus, host) {
                        eprintln!("[gdb] RSP connection ended: {e}");
                    }
                    continue;
                }
                Err(e) if e.kind() != std::io::ErrorKind::WouldBlock => return Err(e.into()),
                Err(_) => {}
            }
            match ocd_l.accept() {
                Ok((stream, peer)) => {
                    eprintln!("[gdb] OpenOCD client {peer}");
                    if let Err(e) = serve_ocd(stream, &cpu, &mut bus) {
                        eprintln!("[gdb] OpenOCD connection ended: {e}");
                    }
                    continue;
                }
                Err(e) if e.kind() != std::io::ErrorKind::WouldBlock => return Err(e.into()),
                Err(_) => {}
            }
            match swd_l.accept() {
                Ok((stream, peer)) => {
                    eprintln!("[gdb] SWD (Glasgow applet) client {peer}");
                    // Restore the running-halt state on disconnect so the main
                    // loop resumes the SP after a humility command detaches.
                    let r = crate::glasgow::serve(stream, &mut cpu, &mut bus, host);
                    cpu.halted = false;
                    cpu.debug_en = false;
                    cpu.bkpt_hit = false;
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
        let sp_burst = if replying { 48 } else { quantum };
        for _ in 0..sp_burst {
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
            // Phase-2 lockstep: if the RoT is replying and its TX FIFO (miso) has
            // drained, stop so the RoT can refill before clocking more.
            if let Some(l) = crate::sprot::link() {
                let l = l.borrow();
                if l.rot_irq && l.cs && l.miso.is_empty() {
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
            let sp = bus.read32(0x0800_0000);
            let pc = bus.read32(0x0800_0004) & !1;
            cpu.reset_for_reboot(sp, pc);
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
            rot_busy = ssa_or_cs || req_in_flight;
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
            let rot_budget = if ssa_or_cs || req_in_flight { 256 } else { 1 };
            'rot_burst: for _ in 0..rot_budget {
                let mut rot_idled = false;
                for _ in 0..quantum {
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
                if crate::sprot::link()
                    .map(|l| l.borrow().rot_irq)
                    .unwrap_or(false)
                {
                    break;
                }
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
            if !bus.any_pending_irq() && !rot_busy {
                std::thread::sleep(std::time::Duration::from_millis(idle_ms));
            }
        }
    }
}
