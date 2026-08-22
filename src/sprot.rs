//! In-process sprot bridge: connects the emulated SP (STM32H7) to the emulated
//! RoT (LPC55) over their SPI link, so the SP's real `drv-stm32h7-sprot-server`
//! talks to the RoT's real `drv-lpc55-sprot-server`.
//!
//! Wiring (all single-threaded — SP and RoT cores interleave in `gdb::serve`):
//!   - SP `SPI4` master  @ 0x4001_3400  (this crate's `SpiMaster`)
//!   - RoT `FLEXCOMM8` slave @ 0x4009_F000 (`RotSpiSlave`)
//!   - CS:     SP GPIO PE4 (low = asserted) — driven via `soc::GpioBank`, set into `cs`
//!   - rot-irq: RoT GPIO P0_18 (low = asserted) — `LpcGpio` -> `rot_irq`; the SP
//!     reads it on GPIO PE3 (the SP's sprot waits on a timer and re-reads the pin,
//!     so no EXTI model is needed for correctness, just for low latency).
//!
//! The link is a process-global so the SP-bus and RoT-bus device models can
//! share it without threading a handle through every constructor. The SP and
//! RoT cores run on separate threads; the link models the wires between them
//! as a mutex-guarded state block plus one condvar per waiting side. Silicon
//! guarantees the RoT services its FIFO IRQ ahead of the SP's SPI clock;
//! thread scheduling does not, so the SP side blocks briefly on RoT progress
//! (`SpiMaster`), the bridge's one non-physical mechanism.
use crate::mem::Mmio;
use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex, MutexGuard, OnceLock};

/// The RoT's FLEXCOMM8 RX FIFO depth in bytes (8 frames × 2 bytes); the SP-side
/// SPI master mirrors the same TX-FIFO depth. Bytes clocked past it are dropped on
/// real hardware, so the bridge bounds both `mosi`/`miso` buffers here.
const SPROT_FIFO_BYTES: usize = 16;

/// EXTI line-3 bit (PE3 = ROT_IRQ), surfaced on the SP's EXTI CPUPR1 register.
const ROT_IRQ_EXTI_BIT: u32 = 1 << 3;

#[derive(Default)]
pub struct SprotLink {
    pub mosi: VecDeque<u8>, // SP -> RoT (request bytes the master clocks out)
    pub miso: VecDeque<u8>, // RoT -> SP (response bytes the slave queued)
    pub cs: bool,           // chip select asserted (SP PE4 low)
    pub rot_irq: bool,      // RoT asserting rot-irq (P0_18 low)
    // Sticky SPI slave-select latches. Set by the SP-side CS driver (soc.rs, the
    // PE4 GPIO write) on each CS edge, not by the RoT polling `cs`. Fixes the
    // two-core interleaving race: the SP can assert/clock/deassert CS entirely
    // within its own quantum before the RoT ever runs, so an RoT that derives
    // SSA/SSD from edge-detecting `cs` at register-access time misses the edges and
    // `wait_for_csn_asserted` hangs (or fires inconsistently across boots). Latching
    // on the SP side, which always observes its own writes, plus `mosi`/`miso`
    // buffering makes the handshake deterministic regardless of how the cores
    // interleave. On real silicon these flags are latched in the SPI block
    // independent of the CPU; this mirrors that. Cleared by the RoT via STAT
    // write-1-clear (ssa/ssd) or consumed on the first FIFORD read (sot).
    pub ssa: bool,         // slave-select asserted latch (CS went low/asserted)
    pub ssd: bool,         // slave-select de-asserted latch (CS went high)
    pub sot_pending: bool, // next FIFORD frame carries Start-Of-Transfer
    // SP-side EXTI pending latch for the ROT_IRQ line (PE3 = EXTI line 3). Set by
    // the serve loop when the RoT toggles rot-irq, surfaced on the SP's EXTI
    // CPUPR1 register so the SP's sys task delivers the ROT_IRQ notification and
    // sprot's wait_rot_irq wakes immediately, instead of polling out a timer (which
    // made every sprot round-trip pay a multi-ms-to-second timeout).
    pub sp_rot_irq_pending: bool,
    // True from when the RoT starts receiving a request (first FIFORD read) until
    // it signals the reply (rot-irq asserted). Used by the serve loop to not sleep
    // the host while the RoT is actively processing a request, so the round-trip
    // runs full-speed, without pegging the CPU during the RoT's idle housekeeping
    // (which an over-broad "RoT ran a full quantum" heuristic did, decaying the
    // instance's scheduling priority and making `voxel sp state` slow/variable).
    pub request_in_flight: bool,
    // Rising-edge latch: the RoT released ROT_TO_SP_RESET_L (PIO0_13 low->high) in
    // `sp_reset_leave`, i.e. it pulsed the SP's reset pin. The serve loop consumes
    // this to reset the SP through its debug port, so that with DEMCR.VC_CORERESET
    // armed the SP halts at its reset vector -- the reset-into-debug-halt the RoT's
    // endoscope measurement depends on. Set by `LpcGpio`, taken by the serve loop.
    pub sp_reset_release: bool,
    // True while an SWD debug probe (a humility/probe-rs client on the SP's SWD TCP
    // port) is attached. Drives SP_TO_ROT_JTAG_DETECT_L (PIO0_20, active-low): the
    // RoT firmware level-reads it to gate its own SWD activity, and edge-detects its
    // assertion to invalidate the attestation log. Set/cleared by the serve loop on
    // the probe's connect/disconnect; read by `LpcGpio` to synthesize the pin level.
    pub jtag_detect: bool,
    // SP thread -> RoT thread events, modeling the PINT edge lines. The SP loop
    // sets these; the RoT thread consumes them and pends its own PINT/NVIC state
    // (it owns the RoT bus, the SP thread must not touch it).
    pub sp_reset_pint: bool, // SP self-reset edge (SP_RESET slot 0 -> IRQ 4)
    pub jtag_pint: bool,     // probe-attach edge (JTAG_DETECT slot 1 -> IRQ 5)
    // True once the in-process RoT thread is running. Gates the SP-side blocking
    // waits: in the shared-RoT IPC mode (SP_EMU_ROT_SERVICE) the SP serve loop is
    // the link peer and fills `miso` itself, so waiting on a RoT would deadlock.
    pub rot_live: bool,
    // Activity counters for the stuck-link watchdog: frames the RoT read from
    // its RX FIFO, frames it queued to TX, and bytes the SP clocked out. A busy
    // link whose counters stop moving is wedged.
    pub rx_frames: u64,
    pub tx_frames: u64,
    pub sp_txdr: u64,
}

/// The wires between the two cores: link state under one mutex, plus a condvar
/// per waiting side. `sp_cv` parks the SP thread (waiting on RoT progress:
/// a `miso` refill, a `mosi` drain, a rot-irq edge, an SWD read result);
/// `rot_cv` parks the idle RoT thread (woken by CS edges, request bytes, and
/// the PINT edge events above).
pub struct LinkCell {
    inner: Mutex<SprotLink>,
    sp_cv: Condvar,
    rot_cv: Condvar,
}
impl LinkCell {
    pub fn new() -> Self {
        LinkCell {
            inner: Mutex::new(SprotLink::default()),
            sp_cv: Condvar::new(),
            rot_cv: Condvar::new(),
        }
    }
    /// Lock the link state. Named to match the old `Rc<RefCell>` call sites.
    pub fn borrow(&self) -> MutexGuard<'_, SprotLink> {
        self.inner.lock().unwrap()
    }
    pub fn borrow_mut(&self) -> MutexGuard<'_, SprotLink> {
        self.inner.lock().unwrap()
    }
    /// Park the calling (SP) thread until the RoT signals progress or `ms` pass.
    pub fn wait_sp<'a>(
        &'a self,
        g: MutexGuard<'a, SprotLink>,
        ms: u64,
    ) -> MutexGuard<'a, SprotLink> {
        self.sp_cv
            .wait_timeout(g, std::time::Duration::from_millis(ms))
            .unwrap()
            .0
    }
    /// Park the calling (RoT) thread until the SP signals work or `ms` pass.
    pub fn wait_rot<'a>(
        &'a self,
        g: MutexGuard<'a, SprotLink>,
        ms: u64,
    ) -> MutexGuard<'a, SprotLink> {
        self.rot_cv
            .wait_timeout(g, std::time::Duration::from_millis(ms))
            .unwrap()
            .0
    }
    pub fn wake_sp(&self) {
        self.sp_cv.notify_all();
    }
    pub fn wake_rot(&self) {
        self.rot_cv.notify_all();
    }
}
pub type Link = Arc<LinkCell>;

// The sprot debug flag (memoized in `dbg.rs`); re-exported so the existing
// `crate::sprot::dbg()` call sites (lpc55/soc/gdb) and this module keep working.
pub use crate::dbg::sprot as dbg;

// One-shot RoT instruction-trace window: armed when the first sprot request frame
// is read, consumed by the gdb serve loop to log the RoT pc for N instructions so
// the exact path through wait_for_request/handler.handle can be reconstructed.
static ROT_TRACE: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
pub fn arm_rot_trace(n: u32) {
    use std::sync::atomic::Ordering;
    if dbg() {
        eprintln!("[rottr] ARM called n={}", n);
    }
    let _ = ROT_TRACE.compare_exchange(0, n, Ordering::SeqCst, Ordering::SeqCst);
}
pub fn rot_trace_tick() -> bool {
    use std::sync::atomic::Ordering;
    let v = ROT_TRACE.load(Ordering::SeqCst);
    if v > 0 {
        ROT_TRACE.store(v - 1, Ordering::SeqCst);
        true
    } else {
        false
    }
}

static LINK: OnceLock<Link> = OnceLock::new();
/// Enable the bridge for this run (called once, before building the buses).
/// Process-global so the SP thread, the RoT thread, and both buses' device
/// models all see the same wires.
pub fn enable() {
    let _ = LINK.set(Arc::new(LinkCell::new()));
}
/// The shared link, if the bridge is enabled.
pub fn link() -> Option<Link> {
    LINK.get().cloned()
}

// ---- SP side: STM32H7 SPI4 master ------------------------------------------

/// Full-duplex SPI master: each TXDR write clocks one byte out to the RoT and
/// one byte in from the RoT (8-bit frames). SR/CR1/CR2 modeled enough for
/// drv-stm32h7-spi to drive it.
pub struct SpiMaster {
    link: Link,
    tsize: u32,
    sent: u32,
    rx: VecDeque<u8>,
    spe: bool,
    n_sr: u64,
}
impl SpiMaster {
    pub fn new(link: Link) -> Self {
        SpiMaster {
            link,
            tsize: 0,
            sent: 0,
            rx: VecDeque::new(),
            spe: false,
            n_sr: 0,
        }
    }
}
impl Mmio for SpiMaster {
    fn name(&self) -> &str {
        "SPI4-sprot"
    }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x14 => {
                // SR: TXP(1) always (tx space), RXP(0)/RXWNE(15)/RXPLVL(11:10) from
                // the rx fifo, EOT(3)/TXC(12) once TSIZE bytes have been clocked.
                let mut sr = 1 << 1;
                if !self.rx.is_empty() {
                    sr |= 1 << 0;
                }
                // RXWNE(15) = a full 32-bit word (>=4 bytes); RXPLVL(14:13) = the
                // 0..3 trailing bytes otherwise. The driver's can_rx_byte() reads
                // both, so these bits must be correct for it to drain the tail.
                if self.rx.len() >= 4 {
                    sr |= 1 << 15;
                } else {
                    sr |= ((self.rx.len() as u32) & 0x3) << 13;
                }
                if self.tsize != 0 && self.sent >= self.tsize {
                    sr |= (1 << 3) | (1 << 12);
                }
                self.n_sr += 1;
                if dbg() && self.n_sr % 5000 == 1 {
                    eprintln!(
                        "[spi] SR#{} = {:#06x} (sent={} tsize={} rx={})",
                        self.n_sr,
                        sr,
                        self.sent,
                        self.tsize,
                        self.rx.len()
                    );
                }
                sr
            }
            0x30 => {
                let b = self.rx.pop_front().unwrap_or(0) as u32;
                if dbg() {
                    eprintln!("[spi] RXDR -> {:#04x}", b);
                }
                b
            } // RXDR
            _ => 0,
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        match off & !3 {
            0x00 => {
                // CR1: SPE bit0 (transfer-active edge resets counters)
                let spe = val & 1 != 0;
                if spe && !self.spe {
                    self.sent = 0;
                    self.rx.clear();
                }
                self.spe = spe;
                if dbg() {
                    eprintln!("[spi] CR1={:#x} SPE={}", val, spe);
                }
            }
            0x04 => {
                self.tsize = val & 0xFFFF;
                if dbg() {
                    eprintln!("[spi] CR2 TSIZE={}", self.tsize);
                }
            } // CR2.TSIZE
            0x20 => {
                // TXDR: clock one byte out + one in
                let mut lk = self.link.borrow_mut();
                // With a live RoT thread, pace the master on RoT progress
                // (bounded; a timeout falls through to the drop/zero-fill
                // below, a normal sprot timeout + retry to the firmware).
                if lk.rot_live {
                    // Phase 2: an empty `miso` mid-reply would clock a zero
                    // into the response and corrupt its CRC; wait for refill.
                    let mut budget = 400;
                    while lk.rot_irq && lk.cs && lk.miso.is_empty() && budget > 0 {
                        lk = self.link.wait_sp(lk, 5);
                        budget -= 1;
                    }
                    // Phase 1: RoT RX FIFO full; wait for its drain. Gated on
                    // !rot_irq: during a reply the SP's dummy clock-outs land
                    // in `mosi` but the RoT is sending, not draining.
                    budget = 400;
                    while !lk.rot_irq && lk.mosi.len() >= SPROT_FIFO_BYTES && budget > 0 {
                        lk = self.link.wait_sp(lk, 5);
                        budget -= 1;
                    }
                }
                // The RoT's FLEXCOMM8 RX FIFO is 8 frames (16 bytes) deep. On real
                // hardware, bytes clocked in while the FIFO is full are dropped
                // (overrun). Model that bound: without it, the SP's rapid retries
                // (when it doesn't see rot-irq) pile unbounded bytes into `mosi`,
                // and the RoT's `while has_entry { read_fifo }` drain never ends —
                // the receive loop livelocks and never delivers the request.
                if lk.mosi.len() < SPROT_FIFO_BYTES {
                    lk.mosi.push_back((val & 0xFF) as u8);
                }
                lk.sp_txdr = lk.sp_txdr.wrapping_add(1);
                let inb = lk.miso.pop_front().unwrap_or(0);
                drop(lk);
                self.link.wake_rot();
                self.rx.push_back(inb);
                self.sent = self.sent.wrapping_add(1);
                if dbg() {
                    eprintln!("[spi] TXDR <- {:#04x} (sent={})", val & 0xFF, self.sent);
                }
            }
            _ => {}
        }
    }
}

// ---- RoT side: LPC55 FLEXCOMM8 SPI slave -----------------------------------

pub struct RotSpiSlave {
    link: Link,
    n_ford: u32,
    n_fwr: u32,
    n_stat: u32,
}
impl RotSpiSlave {
    pub fn new(link: Link) -> Self {
        RotSpiSlave {
            link,
            n_ford: 0,
            n_fwr: 0,
            n_stat: 0,
        }
    }
}
impl Mmio for RotSpiSlave {
    fn name(&self) -> &str {
        "FLEXCOMM8-sprot"
    }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x408 | 0x428 => {
                // STAT / INTSTAT: SSA(4), SSD(5) — read from the link latches
                let lk = self.link.borrow();
                let v = (if lk.ssa { 1 << 4 } else { 0 }) | (if lk.ssd { 1 << 5 } else { 0 });
                self.n_stat += 1;
                if dbg() && self.n_stat <= 60 {
                    eprintln!(
                        "[rot] {} #{} -> ssa={} ssd={} (cs={})",
                        if off & !3 == 0x408 { "STAT" } else { "INTSTAT" },
                        self.n_stat,
                        lk.ssa,
                        lk.ssd,
                        lk.cs
                    );
                }
                v
            }
            0xE04 => {
                // FIFOSTAT: TXNOTFULL(5) until 8 frames (16 bytes) queued,
                // RXNOTEMPTY(6) once a full 16-bit frame (>=2 bytes) is available.
                let lk = self.link.borrow();
                (if lk.miso.len() < SPROT_FIFO_BYTES {
                    1 << 5
                } else {
                    0
                }) | (if lk.mosi.len() >= 2 { 1 << 6 } else { 0 })
            }
            0xE30 => {
                // FIFORD: 16-bit frame (RXDATA[15:0]) + SOT(20). The SP clocks
                // 8-bit frames; the RoT packs two wire bytes per FIFO entry,
                // first byte = upper (see read_u16_with_sot / get_u16).
                let (hi, lo, sot) = {
                    let mut lk = self.link.borrow_mut();
                    // A request is now being received; stays in flight (keeping the
                    // host full-speed) until the RoT asserts rot-irq with the reply.
                    lk.request_in_flight = true;
                    lk.rx_frames = lk.rx_frames.wrapping_add(1);
                    let hi = lk.mosi.pop_front().unwrap_or(0) as u32;
                    let lo = lk.mosi.pop_front().unwrap_or(0) as u32;
                    // SOT is latched on CS assert (soc.rs) and consumed by the first
                    // FIFORD read of the transfer; the firmware checks it to detect
                    // a desynchronized exchange.
                    let sot = if lk.sot_pending {
                        lk.sot_pending = false;
                        1 << 20
                    } else {
                        0
                    };
                    (hi, lo, sot)
                };
                let frame = (hi << 8) | lo;
                // Drained two request bytes: a full-FIFO SP master may be waiting.
                self.link.wake_sp();
                self.n_ford += 1;
                if self.n_ford == 1 && dbg() {
                    arm_rot_trace(40000);
                }
                if dbg() {
                    eprintln!(
                        "[rot] FIFORD#{} -> {:#06x}{}",
                        self.n_ford,
                        frame,
                        if sot != 0 { " SOT" } else { "" }
                    );
                }
                frame | sot
            }
            _ => 0,
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        match off & !3 {
            0x408 => {
                // STAT write-1-clear
                let mut lk = self.link.borrow_mut();
                if val & (1 << 4) != 0 {
                    lk.ssa = false;
                }
                if val & (1 << 5) != 0 {
                    lk.ssd = false;
                }
                if dbg() && self.n_stat <= 60 {
                    eprintln!(
                        "[rot] STAT-clr val={:#x} (ssa->{} ssd->{})",
                        val, lk.ssa, lk.ssd
                    );
                }
                drop(lk);
                // The SSA/SSD ack is what CS-assert transaction pacing waits on.
                self.link.wake_sp();
            }
            0xE00 => {
                // FIFOCFG: EMPTYTX(16) drains TX (miso), EMPTYRX(17) drains RX (mosi)
                let mut lk = self.link.borrow_mut();
                if val & (1 << 16) != 0 {
                    lk.miso.clear();
                }
                if val & (1 << 17) != 0 {
                    lk.mosi.clear();
                }
                drop(lk);
                self.link.wake_sp();
            }
            0xE20 => {
                // FIFOWR: 16-bit frame, upper byte first on the wire (get_u16)
                let frame = (val & 0xFFFF) as u16;
                let mut lk = self.link.borrow_mut();
                lk.tx_frames = lk.tx_frames.wrapping_add(1);
                lk.miso.push_back((frame >> 8) as u8);
                lk.miso.push_back((frame & 0xFF) as u8);
                drop(lk);
                // Refilled the reply: an SP master mid-phase-2 may be waiting.
                self.link.wake_sp();
                self.n_fwr += 1;
                if dbg() && self.n_fwr <= 64 {
                    eprintln!("[rot] FIFOWR#{} <- {:#06x}", self.n_fwr, frame);
                }
            }
            _ => {}
        }
    }
}

// ---- RoT side: LPC55 GPIO (only to surface P0_18 = rot-irq) -----------------

pub struct LpcGpio {
    link: Link,
    p0: u32,
    // Last observed level of ROT_TO_SP_RESET_L (PIO0_13), so `refresh` can spot the
    // low->high edge (`sp_reset_leave`) that releases the SP from reset.
    sp_reset_asserted: bool,
}
impl LpcGpio {
    pub fn new(link: Link) -> Self {
        LpcGpio {
            link,
            p0: 0xFFFF_FFFF,
            sp_reset_asserted: false,
        }
    }
    fn refresh(&mut self) {
        let asserted = (self.p0 >> 18) & 1 == 0;
        // ROT_TO_SP_RESET_L = PIO0_13, active-low: 0 asserts reset, 1 releases it.
        // The RoT drives it low in `sp_reset_enter`, then high in `sp_reset_leave`.
        let reset_asserted = (self.p0 >> 13) & 1 == 0;
        let reset_released = self.sp_reset_asserted && !reset_asserted;
        self.sp_reset_asserted = reset_asserted;
        let mut lk = self.link.borrow_mut();
        if asserted != lk.rot_irq && dbg() {
            eprintln!(
                "[sprot] rot-irq {} (P0_18={})",
                if asserted { "ASSERT" } else { "deassert" },
                (self.p0 >> 18) & 1
            );
        }
        if reset_released {
            if dbg() {
                eprintln!("[sprot] SP_RESET released (PIO0_13 high)");
            }
            lk.sp_reset_release = true;
        }
        // Reply is ready: the request is no longer "in flight" (the SP will now be
        // woken via EXTI to clock the response).
        if asserted {
            lk.request_in_flight = false;
        }
        let changed = asserted != lk.rot_irq || reset_released;
        lk.rot_irq = asserted;
        drop(lk);
        // A rot-irq edge or reset release is exactly what a parked SP waits on.
        if changed {
            self.link.wake_sp();
        }
    }
}
impl Mmio for LpcGpio {
    fn name(&self) -> &str {
        "LPC55-GPIO"
    }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x2100 => {
                // PIN[0]: firmware output levels in `p0`, but bit 20 = PIO0_20 =
                // SP_TO_ROT_JTAG_DETECT_L is an SP-driven input, so synthesize it from
                // the shared link (like CS at 0x2104): active-low, so asserted (a probe
                // attached) reads 0, deasserted reads 1. `p0` defaults all-high, so
                // with no probe this reads 1 and the RoT's SWD path is unaffected.
                let mut v = self.p0;
                if self.link.borrow().jtag_detect {
                    v &= !(1 << 20);
                } else {
                    v |= 1 << 20;
                }
                v
            }
            0x2104 => {
                // PIN[1]: bit 1 = CHIP_SELECT (P1_1), the SP's RoT chip-select,
                // active-low. The RoT's wait_for_csn_deasserted spins until it reads
                // this pin high, so it must reflect the SP's CS (link.cs): asserted
                // -> 0, de-asserted -> 1. Without this the RoT never observes CS go
                // high and loops forever after a transfer.
                let cs = self.link.borrow().cs;
                if cs {
                    0
                } else {
                    1 << 1
                }
            }
            _ => 0,
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        // LPC55 GPIO: B byte regs @0x0 (off=pin), W word regs @0x1000 (off=pin*4),
        // PIN[0]@0x2100, SET[0]@0x2200, CLR[0]@0x2280, NOT[0]@0x2300.
        match off & !3 {
            0x2100 => {
                self.p0 = val;
                self.refresh();
            }
            0x2200 => {
                self.p0 |= val;
                self.refresh();
            }
            0x2280 => {
                self.p0 &= !val;
                self.refresh();
            }
            0x2300 => {
                self.p0 ^= val;
                self.refresh();
            }
            o if o < 0x20 => {
                // B[0][pin] byte register (offset = pin index)
                let pin = o & 0x1F;
                if val & 0xFF != 0 {
                    self.p0 |= 1 << pin;
                } else {
                    self.p0 &= !(1 << pin);
                }
                self.refresh();
            }
            o if (0x1000..0x1080).contains(&o) => {
                // W[0][pin] word register
                let pin = (o - 0x1000) / 4;
                if val != 0 {
                    self.p0 |= 1 << pin;
                } else {
                    self.p0 &= !(1 << pin);
                }
                self.refresh();
            }
            _ => {}
        }
    }
}

// ---- SP side: STM32H7 EXTI (just enough for ROT_IRQ interrupt delivery) -------
//
// The SP's `sys` task uses an EXTI interrupt on PE3 (ROT_IRQ, EXTI line 3) so
// drv-stm32h7-sprot-server's `wait_rot_irq` can sleep until the RoT signals a
// reply, rather than polling. The sys ISR reads CPUPR1 (0x88) to find pending
// lines, AND-masks with CPUIMR1 (0x80, the enable mask it wrote), posts the
// owning task's notification, then write-1-clears the pending bit. CPUPR1 bit 3 is
// modeled from the shared rot-irq pending latch (set by the serve loop on a
// rot-irq edge, which also pends NVIC IRQ 9 = exti3); every other EXTI register is
// plain store/return so the sys task's edge/enable config reads back. Without this,
// EXTI fell into the catch-all and CPUPR1 never set, so the SP always waited out
// its fallback timer, the cause of slow/variable `voxel sp state`.
pub struct SpExti {
    link: Link,
    regs: std::collections::HashMap<u32, u32>,
}
impl SpExti {
    pub fn new(link: Link) -> Self {
        SpExti {
            link,
            regs: std::collections::HashMap::new(),
        }
    }
}
impl Mmio for SpExti {
    fn name(&self) -> &str {
        "SP-EXTI"
    }
    fn read(&mut self, off: u32) -> u32 {
        let off = off & !3;
        if off == 0x88 {
            // CPUPR1: bit 3 (EXTI line 3 = PE3 = ROT_IRQ) from the pending latch.
            let p = if self.link.borrow().sp_rot_irq_pending {
                ROT_IRQ_EXTI_BIT
            } else {
                0
            };
            return (self.regs.get(&off).copied().unwrap_or(0) & !ROT_IRQ_EXTI_BIT) | p;
        }
        *self.regs.get(&off).unwrap_or(&0)
    }
    fn write(&mut self, off: u32, val: u32) {
        let off = off & !3;
        if off == 0x88 {
            // Write-1-clear the ROT_IRQ pending latch.
            if val & ROT_IRQ_EXTI_BIT != 0 {
                self.link.borrow_mut().sp_rot_irq_pending = false;
            }
            return;
        }
        self.regs.insert(off, val);
    }
}

// ---- Stuck-link watchdog ----------------------------------------------------
//
// Detects the wedge class seen on voxel: an sprot exchange starts (ssa latched,
// a request in flight, or rot-irq held) and then nothing moves again, leaving
// every later sprot op to time out until the instance is restarted. The SP
// serve loop calls `watchdog_tick` once per iteration; when the link has been
// busy with no counter movement for `WATCHDOG_FIRST`, it logs one line of link
// state plus the RoT's pc, then repeats every `WATCHDOG_REPEAT` while stuck.
// Diagnostic only: it never mutates the link.

const WATCHDOG_FIRST: std::time::Duration = std::time::Duration::from_secs(5);
const WATCHDOG_REPEAT: std::time::Duration = std::time::Duration::from_secs(60);

pub struct Watchdog {
    counters: (u64, u64, u64),
    since: std::time::Instant,
    next_log: std::time::Duration,
}
impl Watchdog {
    pub fn new() -> Self {
        Watchdog {
            counters: (0, 0, 0),
            since: std::time::Instant::now(),
            next_log: WATCHDOG_FIRST,
        }
    }
}

/// Sample the link; log if it has been busy without progress. `rot_pc` and
/// `rot_ticks` come from the RoT thread's published state.
pub fn watchdog_tick(wd: &mut Watchdog, rot_pc: u32, rot_ticks: u64) {
    let Some(lk) = link() else { return };
    let l = lk.borrow();
    let busy = l.ssa || l.ssd || l.rot_irq || l.request_in_flight;
    let counters = (l.rx_frames, l.tx_frames, l.sp_txdr);
    if !busy || counters != wd.counters {
        wd.counters = counters;
        wd.since = std::time::Instant::now();
        wd.next_log = WATCHDOG_FIRST;
        return;
    }
    let stuck = wd.since.elapsed();
    if stuck >= wd.next_log {
        eprintln!(
            "[sprotwd] link stuck {}s: cs={} ssa={} ssd={} rot_irq={} req_in_flight={} \
             mosi={} miso={} rx_frames={} tx_frames={} sp_txdr={} rot_pc={:#010x} rot_ticks={}",
            stuck.as_secs(),
            l.cs,
            l.ssa,
            l.ssd,
            l.rot_irq,
            l.request_in_flight,
            l.mosi.len(),
            l.miso.len(),
            l.rx_frames,
            l.tx_frames,
            l.sp_txdr,
            rot_pc,
            rot_ticks,
        );
        wd.next_log = stuck + WATCHDOG_REPEAT;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // PIO0_20 = SP_TO_ROT_JTAG_DETECT_L is an SP-driven input, active-low, synthesized
    // at GPIO read time from SprotLink::jtag_detect: high (deasserted) with no probe,
    // low (asserted) while one is attached, without disturbing other port-0 pins.
    #[test]
    fn jtag_detect_level_synthesized_on_pio0_20() {
        let link = Arc::new(LinkCell::new());
        let mut g = LpcGpio::new(link.clone());

        // No probe: PIO0_20 reads high (p0 defaults all-high), SWD path unaffected.
        assert_ne!(g.read(0x2100) & (1 << 20), 0, "PIO0_20 high when no probe");

        // Probe attached: PIO0_20 reads low; a nearby pin (P0_18) is untouched.
        link.borrow_mut().jtag_detect = true;
        let v = g.read(0x2100);
        assert_eq!(v & (1 << 20), 0, "PIO0_20 low when a probe is attached");
        assert_ne!(v & (1 << 18), 0, "other port-0 pins are unchanged");
    }
}
