//! STM32H753 SoC assembly + the boot-critical peripheral models.
//!
//! "Boot-critical" here means exactly the peripherals whose busy-wait status
//! bits the Hubris startup path (`drv/stm32h7-startup`) spins on — if these
//! don't read back "ready", the firmware hangs before reaching the kernel:
//!   PWR.CSR1.ACTVOSRDY, PWR.D3CR.VOSRDY, RCC.CR.HSERDY, RCC.CR.PLL1RDY,
//!   RCC.CFGR.SWS == PLL1.
//! Everything else can start life as a stub (reads 0, swallows writes).

use crate::mem::{Bus, Mmio};
use std::cell::{Cell, RefCell};
use std::rc::Rc;

/// Which device is selected on a shared SPI bus, derived from the active (low)
/// chip-select GPIO. The GPIO bank sets this; the SPI peripheral reads it to
/// route a transaction. 0 = none/other, 1 = sequencer FPGA (PB5), 2 = KSZ8463 (PI0).
pub type Spi2Cs = Rc<Cell<u8>>;

/// Sidecar SPI5 user-design chip-select "assert generation" — incremented by the
/// GPIO bank each time PJ6 (CS_USER_L) goes asserted (low). Each FPGA command is
/// one CS lock (header write + data read across two SPE cycles); Spi5 resets its
/// per-command state whenever this counter changes. A counter (not a bool) is
/// used because the deassert between commands happens with no Spi5 access, so a
/// bool edge would be missed — the GPIO bank sees every PJ6 write and counts.
pub type Spi5Cs = Rc<Cell<u32>>;

// ---- standard STM32H7 memory map (gimletlet uses the AXI-SRAM layout) -------

pub fn install_memory(bus: &mut Bus) {
    bus.add_ram(0x0800_0000, 0x0020_0000); // Flash bank 1 (2 MB window)
    bus.add_ram(0x0000_0000, 0x0001_0000); // ITCM (also boot alias)
    bus.add_ram(0x2000_0000, 0x0002_0000); // DTCM (128 KB)
    bus.add_ram(0x2400_0000, 0x0008_0000); // AXI SRAM (512 KB) — initial SP lives here
    bus.add_ram(0x3000_0000, 0x0004_8000); // SRAM1/2/3 (D2)
    bus.add_ram(0x3800_0000, 0x0001_0000); // SRAM4 (D3)
    bus.add_ram(0x3880_0000, 0x0000_1000); // Backup SRAM
}

pub fn install_peripherals(bus: &mut Bus) {
    bus.add_device(0x5802_4400, 0x400, Box::new(Rcc::new()));
    bus.add_device(0x5802_4800, 0x400, Box::new(Pwr::new()));
    bus.add_device(0xE000_E000, 0x1000, Box::new(Scs::new())); // SysTick/NVIC/SCB/CPACR
    // FLASH controller: config registers are write-then-readback (e.g. ACR
    // latency), so a plain store/return register file is the correct model.
    bus.add_device(0x5200_2000, 0x100, Box::new(RegFile::new("FLASH")));

    // Ethernet MAC/MTL/DMA (0x40028000) is modeled directly in the Bus (src/mem.rs
    // `EthDma`) — its DMA engine needs to read/write descriptor rings + packet
    // buffers in RAM, which a standalone `Mmio` device can't reach.

    // TIM16 (MDIO bit-timer for the eth driver) — raises its IRQ when armed.
    bus.add_device(0x4001_4400, 0x400, Box::new(Tim16::new()));

    // SPI4 + the KSZ8463 switch behind it (net's management interface).
    if let Some(lk) = crate::sprot::link() {
        bus.add_device(0x4001_3400, 0x400, Box::new(crate::sprot::SpiMaster::new(lk))); // SP<->RoT sprot link
    } else {
        bus.add_device(0x4001_3400, 0x400, Box::new(Spi4::new()));
    }

    // GPIO bank (0x5802_0000, ports A-K @ 0x400 each). Store/return except the
    // input-data register IDR (+0x10): gimlet's boot polls power-good + board-rev
    // pins that are externally driven, so synthesize them per port. The bank also
    // drives the shared SPI2 chip-select (PB5=sequencer, PI0=KSZ8463).
    let spi2_cs: Spi2Cs = Rc::new(Cell::new(0));
    let spi5_cs: Spi5Cs = Rc::new(Cell::new(0));
    bus.add_device(0x5802_0000, 0x3000, Box::new(GpioBank::new(spi2_cs.clone(), spi5_cs.clone())));

    // SPI bus wiring differs by board. On gimlet, SPI2 (0x4000_3800) is the iCE40
    // sequencer FPGA + KSZ8463, CS-routed. On the sidecar, SPI2 is monorail's
    // VSC7448 management switch, net's KSZ8463 is on SPI3 (0x4000_3C00), and the
    // mainboard ECP5 (drv-fpga-server) is on SPI5 (0x4001_5000). The sidecar
    // devices are only installed for that board so they don't shadow gimlet's map.
    if std::env::var("SP_EMU_BOARD").map(|b| b == "sidecar").unwrap_or(false) {
        bus.add_device(0x4000_3800, 0x400, Box::new(Vsc7448::new()));    // monorail ⇄ VSC7448
        bus.add_device(0x4000_3C00, 0x400, Box::new(Spi4::new()));       // net ⇄ KSZ8463 (reuse KSZ model)
        bus.add_device(0x4001_5000, 0x400, Box::new(Spi5::new(spi5_cs)));
    } else {
        bus.add_device(0x4000_3800, 0x400, Box::new(Spi2::new(spi2_cs))); // gimlet sequencer/KSZ
    }

    // I2C controllers (gimlet: i2c1 spd, i2c2/3/4 sensors/power). Minimal FSM
    // model: report ready/complete so the driver's transactions succeed (writes
    // accepted, reads return 0). Lets gimlet_seq's vcore_soc_off + sensors pass.
    // One shared sensor environment (scriptable physical values) across controllers.
    let sensors = SensorEnv::from_env();
    let vpd = build_vpd_eeprom();
    // One shared I2C bridge socket (SP_EMU_I2C_BRIDGE sniff / SP_EMU_I2C_DEVICE
    // delegate) carries every bus.
    let bridge = crate::i2c_bridge::I2cBridge::from_env();
    for (i, (base, ev_irq)) in
        [(0x4000_5400u32, 31u16), (0x4000_5800, 33), (0x4000_5C00, 72), (0x5800_1C00, 95)]
            .into_iter()
            .enumerate()
    {
        let dev = I2c::new(ev_irq, sensors.clone(), vpd.clone(), bridge.clone(), (i + 1) as u8);
        bus.add_device(base, 0x400, Box::new(dev));
    }

    // STM32H7 HASH (0x4802_1400, irq 80): gimlet's hash_driver starts a digest
    // (STR.DCAL) then blocks on the HASH irq. Unmodeled, that irq never fires =>
    // hash_driver never replies => hf (host-flash) waits forever => CPA deadlocks
    // on `send to hf` and the whole gimlet SP goes dark the moment MGS does its
    // inventory phase1 host-flash hash. Model it (below) so the digest completes.
    bus.add_device(0x4802_1400, 0x1000, Box::new(Hash::new()));

    // QUADSPI (0x5200_5000): command-aware host-flash model for the `hf` task.
    // Answers RDID with a recognized Micron 32 MiB chip + blank flash, so hf's
    // init completes and it returns to its dispatch loop — which unblocks
    // gimlet_seq's A0 host-power transition (it sends to hf) and thus the
    // control_plane_agent get_state / MGS `state` path. See the Qspi impl.
    bus.add_device(0x5200_5000, 0x400, Box::new(Qspi::new()));

    // SYSCFG (0x5800_0400): gimlet's kernel reads PKGR (+0x124) on boot and
    // panics unless pkg[3:0] == 0b1000 (TFBGA240) — a guard against flashing
    // gimlet firmware onto a gimletlet. Synthesize the gimlet package.
    bus.add_device(0x5800_0400, 0x400, Box::new(Syscfg::new()));

    // EXTI (0x5800_0000): when the sprot bridge is active, model it so the SP's
    // sys task can deliver the ROT_IRQ (PE3 / EXTI line 3) interrupt and sprot's
    // wait_rot_irq wakes the instant the RoT replies, instead of polling out a
    // fallback timer. Added before the catch-all so it owns the EXTI range.
    if let Some(lk) = crate::sprot::link() {
        bus.add_device(0x5800_0000, 0x400, Box::new(crate::sprot::SpExti::new(lk)));
    }

    // STM32H7 96-bit unique device ID @ 0x1FF1E800 (system/OTP memory, not RAM).
    // `net` hashes it into its MAC address; if unmapped the read returns 0 and a
    // downstream slice panics, so provide a stable fake UID.
    bus.add_device(0x1FF1_E800, 0x10, Box::new(Uid));

    // UART7 (0x4000_7800, IRQ 82): the SP<->host-CPU link the real Hubris
    // `host_sp_comms` task drives (host-sp-comms / IPCC + the host serial
    // console). Unmodeled it hits the store/return catch-all below and the channel
    // is dead. Installed before the catch-all so it owns the UART7 range. TX/RX
    // bytes ride the shared queues the Bus pumps to/from the host (`pump_uart`);
    // the RX IRQ is delivered Bus-side (collect_irqs) like the eth DMA, since host
    // input is asynchronous (not gated by the dev-touched IRQ-poll optimization).
    let (utx, urx) = (bus.uart_tx.clone(), bus.uart_rx.clone());
    bus.add_device(0x4000_7800, 0x400, Box::new(Uart7::new(utx, urx)));

    // Broad store-and-return model for the rest of the peripheral space (GPIO,
    // SPI, I2C, USART, timers, ...). Added LAST so the specific devices above
    // (RCC/PWR/FLASH, which synthesize ready bits) take precedence. This lets
    // readback-style peripheral use work and keeps the differential harness in
    // sync; status-bit polls that need real hardware still read 0 (a task that
    // depends on them will block until interrupt delivery is modeled).
    bus.add_device(0x4000_0000, 0x2000_0000, Box::new(RegFile::new("periph")));
}

/// TIM16 (0x40014400), used by the `net`/eth driver as the MDIO bit-timer. The
/// driver arms it as a one-pulse timer (CR1.CEN=1), then blocks on its IRQ
/// (mdio-timer-irq = IRQ 117). We model that: arming raises the IRQ once, sets
/// SR.UIF, and self-clears CR1.CEN so the driver's `while cen {}` wait breaks.
pub struct Tim16 { regs: std::collections::HashMap<u32, u32>, armed: bool }
impl Tim16 { pub fn new() -> Self { Tim16 { regs: std::collections::HashMap::new(), armed: false } } }
impl Mmio for Tim16 {
    fn name(&self) -> &str { "TIM16" }
    fn read(&mut self, off: u32) -> u32 { *self.regs.get(&(off & !3)).unwrap_or(&0) }
    fn write(&mut self, off: u32, val: u32) {
        self.regs.insert(off & !3, val);
        if off & !3 == 0x00 && val & 1 != 0 { self.armed = true; } // CR1.CEN set
    }
    fn take_irq(&mut self) -> Option<u16> {
        if !self.armed { return None; }
        self.armed = false;
        *self.regs.entry(0x10).or_insert(0) |= 1; // SR.UIF (update interrupt flag)
        *self.regs.entry(0x00).or_insert(0) &= !1; // CR1.CEN self-clears (one-pulse)
        Some(117)
    }
}

/// Shared byte queue between the UART7 device and the host bridge — the same
/// Rc-sharing idiom as `Spi2Cs`. TX = SP->host, RX = host->SP. `Bus::pump_uart`
/// drains/fills these against the `HostIo` (the propolis IPCC COM port).
pub type UartQueue = Rc<RefCell<std::collections::VecDeque<u8>>>;

/// UART7 (0x4000_7800, IRQ 82) — the SP<->host-CPU link the real Hubris
/// `host_sp_comms` task drives (host-sp-comms / IPCC + the host serial console).
/// Faithful enough for the unmodified `drv-stm32h7-usart` (verified against the
/// task's captured register usage): the transmit path is always ready
/// (TXFNF|TC|TXFE + TEACK|REACK), so a write to TDR pushes straight to the host
/// TX queue; a byte in the RX queue sets ISR.RXNE and a read of RDR pops it. The
/// RX interrupt (IRQ 82) is raised Bus-side in `collect_irqs` while the RX queue
/// is non-empty (level-triggered, matching the H7 FIFO RXFNE the task enables) —
/// host input is asynchronous, so it can't go through the dev-touched IRQ poll.
/// The task uses no TX interrupt (it polls TXFNF), so none is modeled.
pub struct Uart7 {
    regs: std::collections::HashMap<u32, u32>,
    tx: UartQueue, // SP -> host
    rx: UartQueue, // host -> SP
    dbg: bool,
}
impl Uart7 {
    pub fn new(tx: UartQueue, rx: UartQueue) -> Self {
        Uart7 {
            regs: std::collections::HashMap::new(),
            tx,
            rx,
            dbg: std::env::var("SP_EMU_UARTDBG").is_ok(),
        }
    }
}
impl Mmio for Uart7 {
    fn name(&self) -> &str { "UART7" }
    fn read(&mut self, off: u32) -> u32 {
        let r = off & !3;
        match r {
            // ISR: transmit path always ready (TXFNF7|TC6|TXFE23|TEACK21|REACK22);
            // RXNE/RXFNE(5) set whenever a host byte is waiting in the RX queue.
            0x1C => {
                let mut isr = (1 << 7) | (1 << 6) | (1 << 23) | (1 << 21) | (1 << 22);
                if !self.rx.borrow().is_empty() {
                    isr |= 1 << 5;
                }
                isr
            }
            // RDR: pop one received byte (clears RXNE for the next ISR read).
            0x24 => {
                let b = self.rx.borrow_mut().pop_front();
                if self.dbg {
                    if let Some(b) = b { eprintln!("[uart7] RX {:#04x}", b); }
                }
                b.map(|b| b as u32).unwrap_or(0)
            }
            _ => *self.regs.get(&r).unwrap_or(&0),
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        let r = off & !3;
        if r == 0x28 {
            // TDR: a transmitted byte -> the host TX queue.
            self.tx.borrow_mut().push_back(val as u8);
            if self.dbg { eprintln!("[uart7] TX {:#04x}", val & 0xff); }
        } else {
            self.regs.insert(r, val);
        }
    }
}

/// The KSZ8463 switch's SPI register model, shared by the gimlet's net `Spi4` and
/// the sequencer-multiplexed `Spi2`. Mostly store/return, except CIDER (chip id) =
/// 0x8452 and IADR5's high word = 0x4000, both of which the driver polls.
#[derive(Default)]
struct Ksz8463 {
    regs: std::collections::HashMap<u16, u16>,
}
impl Ksz8463 {
    fn read(&self, addr: u16) -> u16 {
        match addr {
            0x0C => 0x8452, // CIDER (register 0x000): KSZ8463 chip id — driver requires this
            // IADR5 (register 0x02E, packed 0x2F0): high word of an indirect-access
            // MIB-counter read. read_mib_counter() spin-loops here until bit 14
            // ("valid") is set; counter value 0 → driver returns Count(0).
            0x2F0 => 0x4000,
            _ => *self.regs.get(&addr).unwrap_or(&0),
        }
    }
    fn write(&mut self, addr: u16, val: u16) {
        self.regs.insert(addr, val);
    }
}

/// SPI4 (0x40013400) with an attached KSZ8463 switch slave. `net` talks to the
/// switch over SPI: a 4-byte exchange [addr_be_hi, addr_be_lo, d0, d1] where the
/// MSB of byte0 means write. Reads return the 16-bit register (little-endian) in
/// bytes 2..4. We model the SPI master synchronously — each TX byte immediately
/// produces its RX byte — so the driver's transfer loop never has to sleep on
/// spi-irq. The KSZ is mostly store/return, except CIDER (chip id) = 0x8452,
/// which the driver checks before proceeding.
pub struct Spi4 {
    regs: std::collections::HashMap<u32, u32>,
    ksz: Ksz8463,
    rx: Vec<u8>,
    idx: u32,      // byte index within the current SPI transaction
    cmd: u16,      // accumulated command word (address + write bit)
    is_write: bool,
    val: u16,      // register value being read out (response bytes)
    dlo: u8,       // low data byte captured during a write
}
impl Spi4 {
    pub fn new() -> Self {
        Spi4 { regs: std::collections::HashMap::new(), ksz: Ksz8463::default(),
            rx: Vec::new(), idx: 0, cmd: 0, is_write: false, val: 0, dlo: 0 }
    }
    /// Clock one byte out (and one in) of the KSZ, by position in the 4-byte xfer.
    fn xfer_byte(&mut self, b: u8) -> u8 {
        let pos = self.idx % 4;
        self.idx += 1;
        match pos {
            0 => { self.cmd = (b as u16) << 8; 0 }
            1 => {
                self.cmd |= b as u16;
                self.is_write = self.cmd & 0x8000 != 0;
                self.val = self.ksz.read(self.cmd & 0x7FFF);
                0
            }
            2 => if self.is_write { self.dlo = b; 0 } else { (self.val & 0xFF) as u8 },
            3 => {
                if self.is_write {
                    self.ksz.write(self.cmd & 0x7FFF, ((b as u16) << 8) | self.dlo as u16);
                    0
                } else { (self.val >> 8) as u8 }
            }
            _ => 0,
        }
    }
}
/// Synthesize the STM32H7 SPI `SR` for the simple transfer model the SPI device
/// blocks share: TXP always set (tx space available), RXPLVL when rx has data, and
/// EOT+TXC once `done_count` clocked bytes reach the CR2 `tsize` (`done_count` is
/// each block's `idx`, except Spi5's `xfer_cnt`).
fn spi_sr(done_count: u32, tsize: u32, rx_nonempty: bool) -> u32 {
    let mut sr = 1 << 1; // TXP: tx space always available
    if rx_nonempty {
        sr |= 1 << 13; // RXPLVL != 0: rx data available
    }
    if done_count >= tsize.max(1) {
        sr |= (1 << 3) | (1 << 12); // EOT + TXC
    }
    sr
}

impl Mmio for Spi4 {
    fn name(&self) -> &str { "SPI4" }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x14 => { // SR
                let tsize = self.regs.get(&0x04).copied().unwrap_or(0) & 0xFFFF;
                spi_sr(self.idx, tsize, !self.rx.is_empty())
            }
            0x30 => if self.rx.is_empty() { 0 } else { self.rx.remove(0) as u32 }, // RXDR: pop
            0x20 => 0, // TXDR read (only happens via byte-write RMW) — harmless
            o => *self.regs.get(&o).unwrap_or(&0),
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        match off & !3 {
            0x20 => { let rx = self.xfer_byte(val as u8); self.rx.push(rx); } // TXDR
            0x00 => { // CR1: SPE 0->1 starts a fresh transaction
                let was_spe = self.regs.get(&0).map(|v| v & 1 != 0).unwrap_or(false);
                self.regs.insert(0, val);
                if val & 1 != 0 && !was_spe { self.idx = 0; self.rx.clear(); }
            }
            o => { self.regs.insert(o, val); }
        }
    }
}

/// SPI2 (0x4000_3800, sidecar) — `monorail` talks to the VSC7448 management
/// switch over this bus (`monorail` uses the embedded SPI core, `use-spi-core`;
/// on the sidecar SPI2 is the VSC7448, whereas on gimlet SPI2 is the iCE40
/// sequencer — see the board-conditional install). The VSC7448 SPI protocol
/// (drv/vsc7448/src/spi.rs, datasheet §5.5.2): each transaction carries a 24-bit
/// *word* address = `(reg_addr & 0x00FFFFFF) >> 2`, big-endian, byte0 MSB = write:
///   READ:  [a23..16 (MSB=0), a15..8, a7..0] + 1 pad byte + 4 data bytes (BE)
///   WRITE: [a23..16 (MSB=1), a15..8, a7..0] + 4 data bytes (BE)
/// We model the master synchronously (each TXDR byte immediately yields its RXDR
/// byte and SR reports TXP+RXP) so the spi-core transfer loop never has to sleep
/// on spi-irq — same trick as the gimlet `Spi4`/KSZ8463 model. The one value the
/// driver insists on is CHIP_ID (reg 0x71010000 → word addr 0x4000): it must
/// decode rev_id=0x3, part_id=0x7468, mfg_id=0x74, one=0x1 (= 0x374680E9), or
/// `monorail` panics `BadChipId`. Every other register is store/return-0.
pub struct Vsc7448 {
    regs: std::collections::HashMap<u32, u32>,
    vsc: std::collections::HashMap<u32, u32>, // VSC7448 reg file, keyed by 24-bit word addr
    rx: Vec<u8>,
    idx: u32,        // byte index within the current SPI transaction
    is_write: bool,
    waddr: u32,      // accumulated 24-bit word address
    rval: u32,       // register value being read out
    wval: u32,       // register value being assembled during a write
    vscdbg: bool,    // SP_EMU_VSCDBG: trace every VSC7448 register read/write
}
impl Vsc7448 {
    pub fn new() -> Self {
        Vsc7448 { regs: std::collections::HashMap::new(), vsc: std::collections::HashMap::new(),
            rx: Vec::new(), idx: 0, is_write: false, waddr: 0, rval: 0, wval: 0,
            vscdbg: std::env::var("SP_EMU_VSCDBG").is_ok() }
    }
    fn vsc_read(&self, waddr: u32) -> u32 {
        match waddr {
            // DEVCPU_GCB:CHIP_REGS:CHIP_ID (reg 0x71010000): rev_id=3, part_id=0x7468,
            // mfg_id=0x74, one=1 — see drv/vsc7448/src/lib.rs:397.
            0x4000 => 0x374680E9,
            // HSIO:PLL5G_STATUS(0):PLL5G_STATUS1 (reg 0x7146003c): pll5g_setup polls
            // gain_stat (bits[18:14]) and only accepts 2 < v < 0xa, else retries 10x
            // then errors LcPllInitFailed → monorail BspInitFailed panic. Report a
            // locked PLL (gain_stat=5). See drv/vsc7448/src/lib.rs pll5g_setup.
            0x11800f => 5 << 14,
            _ => *self.vsc.get(&waddr).unwrap_or(&0),
        }
    }
    /// Clock one byte out (and one in) of the VSC7448, by position in the xfer.
    fn xfer_byte(&mut self, b: u8) -> u8 {
        let pos = self.idx;
        self.idx += 1;
        match pos {
            0 => { self.is_write = b & 0x80 != 0; self.waddr = ((b & 0x7f) as u32) << 16; 0 }
            1 => { self.waddr |= (b as u32) << 8; 0 }
            2 => {
                self.waddr |= b as u32;
                if !self.is_write {
                    self.rval = self.vsc_read(self.waddr);
                    if self.vscdbg { eprintln!("[vsc] R reg={:#010x} val={:#010x}", 0x7100_0000 | (self.waddr << 2), self.rval); }
                }
                0
            }
            _ => {
                if self.is_write {
                    // value bytes (BE) land at positions 3,4,5,6
                    let sh = (6 - pos) * 8;
                    self.wval = (self.wval & !(0xFFu32 << sh)) | ((b as u32) << sh);
                    if pos == 6 {
                        let (a, mut v) = (self.waddr, self.wval);
                        // RAM_CTRL.RAM_INIT (bit 1) is a self-clearing init strobe in
                        // QSYS/REW/VOP/ANA_AC/ASM/DSM — the driver writes 1 then polls
                        // for it to read back 0 (~40us), else BspInitFailed(RamInitFailed).
                        // Store it pre-cleared (preserving ram_ena, bit 0). Full reg addr
                        // = 0x7100_0000 | (word_addr << 2).
                        const RAM_INIT_REGS: [u32; 6] = [
                            0x717e_03ec, 0x71b5_3528, 0x71c4_3638, // QSYS, REW, VOP
                            0x71f9_4358, 0x7141_39b8, 0x7145_0008, // ANA_AC, ASM, DSM
                        ];
                        if RAM_INIT_REGS.contains(&(0x7100_0000 | (a << 2))) { v &= !0x2; }
                        if self.vscdbg { eprintln!("[vsc] W reg={:#010x} val={:#010x}", 0x7100_0000 | (a << 2), v); }
                        self.vsc.insert(a, v);
                    }
                    0
                } else {
                    // read: 1 pad byte at position 3, then 4 data bytes (BE) at 4..8
                    if pos == 3 { 0 } else { ((self.rval >> ((7 - pos) * 8)) & 0xFF) as u8 }
                }
            }
        }
    }
}
impl Mmio for Vsc7448 {
    fn name(&self) -> &str { "VSC7448(SPI2)" }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x14 => { // SR
                let tsize = self.regs.get(&0x04).copied().unwrap_or(0) & 0xFFFF;
                spi_sr(self.idx, tsize, !self.rx.is_empty())
            }
            0x30 => if self.rx.is_empty() { 0 } else { self.rx.remove(0) as u32 }, // RXDR: pop
            0x20 => 0, // TXDR read (only via byte-write RMW) — harmless
            o => *self.regs.get(&o).unwrap_or(&0),
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        match off & !3 {
            0x20 => { let rx = self.xfer_byte(val as u8); self.rx.push(rx); } // TXDR
            0x00 => { // CR1: SPE 0->1 starts a fresh transaction
                let was_spe = self.regs.get(&0).map(|v| v & 1 != 0).unwrap_or(false);
                self.regs.insert(0, val);
                if val & 1 != 0 && !was_spe {
                    self.idx = 0; self.rx.clear(); self.wval = 0; self.is_write = false;
                }
            }
            o => { self.regs.insert(o, val); }
        }
    }
}

/// SPI2 (0x4000_3800) — gimlet's bus shared (by chip-select) between the iCE40
/// sequencer FPGA (CS PB5) and the KSZ8463 switch (CS PI0). The active device is
/// read from the shared `Spi2Cs` cell (set by the GPIO bank). The transaction is
/// latched at SPE 0→1 so a whole exchange goes to one device.
///
/// Sequencer FPGA protocol: a 3-byte header [cmd, addr_be_hi, addr_be_lo] then
/// data; READ (cmd=1) returns reg[addr++] for bytes after the header. Registers
/// modeled: ID0/1 = 0x01/0xDE (ident 0x1DE), CS0..3 = 0x74753981 LE (matches
/// GIMLET_BITSTREAM_CHECKSUM so the SP skips reprogramming), PWR_CTRL(0x13)=0 (A2).
pub struct Spi2 {
    regs: std::collections::HashMap<u32, u32>,
    cs: Spi2Cs,
    target: u8,   // device latched at SPE (1=seq, 2=ksz)
    idx: u32,
    rx: Vec<u8>,
    // sequencer FPGA register file + per-transaction header accumulation
    seq: std::collections::HashMap<u16, u8>,
    seq_cmd: u8,
    seq_addr: u16,
    // KSZ8463 (same model as Spi4)
    ksz: Ksz8463,
    kcmd: u16, kwrite: bool, kval: u16, kdlo: u8,
    dbg_txn: u32,
}
impl Spi2 {
    pub fn new(cs: Spi2Cs) -> Self {
        Spi2 { regs: Default::default(), cs, target: 0, idx: 0, rx: Vec::new(),
            seq: Default::default(), seq_cmd: 0, seq_addr: 0,
            ksz: Ksz8463::default(), kcmd: 0, kwrite: false, kval: 0, kdlo: 0, dbg_txn: 0 }
    }
    fn seq_read(&self, addr: u16) -> u8 {
        match addr {
            0x0 => 0xDE, 0x1 => 0x01,                  // ID0/ID1 (LE) → ident 0x01DE = 0x1DE
            0xa => 0x81, 0xb => 0x39, 0xc => 0x75, 0xd => 0x74, // CS0..3 → 0x74753981 (LE)
            0x13 => 0x00,                              // PWR_CTRL → 0 (A2 resting)
            a => *self.seq.get(&a).unwrap_or(&0),
        }
    }
    fn xfer(&mut self, b: u8) -> u8 {
        let pos = self.idx; self.idx += 1;
        // Latch the CS target on the first byte: the SPI server sets SPE *before*
        // asserting the chip-select GPIO, so CS isn't valid until data flows.
        if pos == 0 { self.target = self.cs.get(); self.dbg_txn = self.dbg_txn.wrapping_add(1); }
        let r = self.xfer_inner(b, pos);
        if crate::dbg::spi() && self.dbg_txn < 6 && pos <= 6 {
            eprintln!("[spi2x] txn={} tgt={} cs={} pos={} in={:#04x} out={:#04x} cmd={} addr={:#x}",
                self.dbg_txn, self.target, self.cs.get(), pos, b, r, self.seq_cmd, self.seq_addr);
        }
        r
    }
    fn xfer_inner(&mut self, b: u8, pos: u32) -> u8 {
        match self.target {
            1 => { // sequencer FPGA: 3-byte header then data
                match pos {
                    0 => { self.seq_cmd = b; 0 }
                    1 => { self.seq_addr = (b as u16) << 8; 0 }
                    2 => { self.seq_addr |= b as u16; 0 }
                    n => {
                        let a = self.seq_addr.wrapping_add(n as u16 - 3);
                        if self.seq_cmd == 1 { self.seq_read(a) }       // Read
                        else { self.seq.insert(a, b); 0 }               // Write/BitSet/Clear (approx)
                    }
                }
            }
            2 => { // KSZ8463: 4-byte [addr_hi, addr_lo, d0, d1]
                match pos % 4 {
                    0 => { self.kcmd = (b as u16) << 8; 0 }
                    1 => { self.kcmd |= b as u16; self.kwrite = self.kcmd & 0x8000 != 0;
                           self.kval = self.ksz.read(self.kcmd & 0x7FFF); 0 }
                    2 => if self.kwrite { self.kdlo = b; 0 } else { (self.kval & 0xFF) as u8 },
                    _ => { if self.kwrite { self.ksz.write(self.kcmd & 0x7FFF, ((b as u16) << 8) | self.kdlo as u16); 0 }
                           else { (self.kval >> 8) as u8 } }
                }
            }
            _ => 0,
        }
    }
}
impl Mmio for Spi2 {
    fn name(&self) -> &str { "SPI2" }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x14 => { // SR
                let tsize = self.regs.get(&0x04).copied().unwrap_or(0) & 0xFFFF;
                spi_sr(self.idx, tsize, !self.rx.is_empty())
            }
            0x30 => if self.rx.is_empty() { 0 } else { self.rx.remove(0) as u32 },
            o => *self.regs.get(&o).unwrap_or(&0),
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        match off & !3 {
            0x20 => { let rx = self.xfer(val as u8); self.rx.push(rx); }
            0x00 => { // CR1: SPE 0→1 latches the CS target and starts a transaction
                let was = self.regs.get(&0).map(|v| v & 1 != 0).unwrap_or(false);
                self.regs.insert(0, val);
                if val & 1 != 0 && !was { self.idx = 0; self.rx.clear(); } // target latched at 1st byte
            }
            o => { self.regs.insert(o, val); }
        }
    }
}

/// SPI5 (0x4001_5000, irq85) — sidecar's mainboard ECP5 FPGA via drv-fpga-server
/// (`use-spi-core`, so the task drives SPI5 directly). The ECP5 is reported
/// "configured" via GPIO (done=PJ15 high → DeviceState::RunningUserDesign), so
/// SPI5 only carries the FPGA *user-design* register protocol: a 3-byte header
/// [op, addr_be_hi, addr_be_lo] then data. op: Read=1, Write=0, BitSet=2,
/// BitClear=3, ReadNoAddrIncr=6, WriteNoAddrIncr=5; Read auto-increments addr.
/// `read_ident()` reads 16 bytes from Addr::ID0(0x0) as FpgaUserDesignIdent
/// { id, checksum, version, sha } (BE u32 each). The sequencer requires
/// id == EXPECTED_ID 0x01de5bae and checksum == bitstream-checksum prefix
/// 0x5e470764 (else it resets the FPGA + panics). SR flags mirror Spi2/Spi4.
pub struct Spi5 {
    regs: std::collections::HashMap<u32, u32>,
    rx: Vec<u8>,
    idx: u32,
    op: u8,
    addr: u16,
    dpos: u32,       // data bytes consumed this command (after the 3-byte header)
    xfer_cnt: u32,   // bytes in the CURRENT spi-core transfer (for EOT); resets per SPE
    dbg_n: u32,      // SP_EMU_SPIDBG trace counter (cap output)
    cs: Spi5Cs,      // user-design CS assert-generation; reset the command when it changes
    last_gen: u32,
    fpga: std::collections::HashMap<u16, u8>, // FPGA user-design register file (byte-addressed)
}
/// Seed the FPGA ignition-controller register block so the emulated sidecar SP
/// answers MGS `ignition`. The sidecar is the rack's ignition hub: MGS issues
/// GET /ignition as step 1 of SP enumeration, so without a populated controller
/// no SPs — and therefore no switches or switch-ports — are ever discovered
/// (the symptom downstream is rack-init failing on `qsfp0 not found`).
///
/// Register layout (drv-sidecar-mainboard-controller / drv-ignition-api):
///   IGNITION_CONTROLLERS_COUNT @ 0x300  u8   port count (35)
///   IGNITION_TARGETS_PRESENT0  @ 0x301  u64  presence bitmap (LE), bit per port
///   per-port PortState         @ 0x400 + 0x100*port  u64 (LE byte fields):
///     [0] CONTROLLER_STATE      TARGET_PRESENT(0x01)
///     [1] CONTROLLER_LINK_STATUS RECEIVER_ALIGNED|RECEIVER_LOCKED (0x03)
///     [2] TARGET_SYSTEM_TYPE     RFD-141 id (gimlet 0x11, sidecar 0x12, psc 0x13, cosmo 0x04)
///     [3] TARGET_SYSTEM_STATUS   CONTROLLER0_DETECTED(0x01)|SYSTEM_POWER_ENABLED(0x04) → On
///     [5] TARGET_REQUEST_STATUS  0 (no power transition in progress)
///     [6] TARGET_LINK0_STATUS / [7] TARGET_LINK1_STATUS  aligned|locked (0x03)
///
/// Topology is rack-specific, so it's env-configurable via SP_EMU_IGNITION as a
/// comma-separated `port:type` list, e.g. "0:gimlet,1:sidecar,2:gimlet,3:gimlet".
/// Listed ports read present/powered-on/link-locked; all others read absent.
/// Default is the 3-sled a4x2 reference rack.
fn seed_ignition(fpga: &mut std::collections::HashMap<u16, u8>) {
    const CONTROLLERS_COUNT: u16 = 0x300;
    const TARGETS_PRESENT0: u16 = 0x301;
    const PORT_BASE: u16 = 0x400;
    const PORT_STRIDE: u16 = 0x100;
    const NUM_PORTS: u8 = 35;

    let spec = std::env::var("SP_EMU_IGNITION")
        .unwrap_or_else(|_| "0:gimlet,1:sidecar,2:gimlet,3:gimlet".to_string());

    fpga.insert(CONTROLLERS_COUNT, NUM_PORTS);

    let mut present: u64 = 0;
    for entry in spec.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        let (port_s, type_s) = match entry.split_once(':') {
            Some(x) => x,
            None => { eprintln!("[sp-emu] SP_EMU_IGNITION: ignoring malformed entry {:?}", entry); continue; }
        };
        let port: u8 = match port_s.trim().parse() {
            Ok(p) if p < NUM_PORTS => p,
            _ => { eprintln!("[sp-emu] SP_EMU_IGNITION: ignoring out-of-range port {:?}", port_s); continue; }
        };
        let sys_type: u8 = match type_s.trim().to_ascii_lowercase().as_str() {
            "gimlet" => 0x11,
            "sidecar" => 0x12,
            "psc" => 0x13,
            "cosmo" => 0x04,
            other => { eprintln!("[sp-emu] SP_EMU_IGNITION: unknown type {:?}, defaulting to gimlet", other); 0x11 }
        };
        present |= 1u64 << port;
        let bytes = [0x01u8, 0x03, sys_type, 0x05, 0x00, 0x00, 0x03, 0x03];
        let base = PORT_BASE + PORT_STRIDE * port as u16;
        for (i, b) in bytes.iter().enumerate() {
            fpga.insert(base + i as u16, *b);
        }
    }
    for i in 0..8u16 {
        fpga.insert(TARGETS_PRESENT0 + i, ((present >> (i * 8)) & 0xFF) as u8);
    }
}

impl Spi5 {
    pub fn new(cs: Spi5Cs) -> Self {
        let mut fpga = std::collections::HashMap::new();
        // IDENT @ ID0..: id=0x01de5bae (ident.id is BE → bytes 01 de 5b ae).
        // checksum: the driver reads ident.checksum BE but compares to the LE
        // interpretation of SIDECAR_MAINBOARD_BITSTREAM_CHECKSUM[..4]=[5e,47,07,64]
        // = 0x6407475e, so CS0..3 must be 64 07 47 5e (BE → 0x6407475e). version/sha=0.
        for (a, v) in [(0u16, 0x01u8), (1, 0xde), (2, 0x5b), (3, 0xae),
                       (4, 0x64), (5, 0x07), (6, 0x47), (7, 0x5e),
                       // FRONT_IO_STATE (0x30): STATE field is bits[7:4]; set it to
                       // PowerRailStatus::Enabled(4) → 4<<4 = 0x40, so the sequencer's
                       // front-IO hot-swap preinit loop completes (status == Enabled).
                       (0x30, 0x40),
                       // Tofino sequencer register block (drv-sidecar-mainboard-controller
                       // tofino2.rs / generated reg map). Report a coherent, healthy A2
                       // resting state with no abort — the natural stay-in-a2 state, and
                       // what a4x2 needs (its Tofino dataplane is SoftNPU/P4 in software,
                       // so the SP only has to report the switch as present/powered, not
                       // sequence real silicon). TofinoSeqStatus decodes 6 bytes at
                       // 0x100..0x105: CTRL, STATE=A2(1), STEP=Init(0), ERROR=None(0),
                       // ERROR_STATE=Init(0), ERROR_STEP=Init(0) → abort=None.
                       (0x100, 0x00),  // TOFINO_SEQ_CTRL (EN=0; we are at rest in A2)
                       (0x101, 0x01),  // TOFINO_SEQ_STATE = A2
                       (0x102, 0x00),  // TOFINO_SEQ_STEP = Init
                       (0x103, 0x00),  // TOFINO_SEQ_ERROR = None
                       (0x104, 0x00),  // TOFINO_SEQ_ERROR_STATE = Init
                       (0x105, 0x00)]  // TOFINO_SEQ_ERROR_STEP = Init
    {
            fpga.insert(a, v);
        }
        seed_ignition(&mut fpga);
        Spi5 { regs: std::collections::HashMap::new(), rx: Vec::new(), idx: 0, op: 0, addr: 0, dpos: 0, xfer_cnt: 0, dbg_n: 0, cs, last_gen: 0, fpga }
    }
    /// Reset per-command state on the CS deasserted→asserted edge — a command
    /// (header write + data read) spans two SPE cycles under one CS lock.
    fn check_cs(&mut self) {
        let gen = self.cs.get();
        if gen != self.last_gen { // a new CS lock began → new FPGA command
            self.idx = 0; self.dpos = 0; self.rx.clear(); self.op = 0; self.addr = 0; self.xfer_cnt = 0;
            self.last_gen = gen;
        }
    }
    /// The next data byte for the current (read) command, with address auto-increment.
    fn next_data(&mut self) -> u8 {
        let incr = self.op != 5 && self.op != 6; // No-AddrIncr variants hold addr
        let a = self.addr.wrapping_add(if incr { self.dpos } else { 0 } as u16);
        self.dpos += 1;
        *self.fpga.get(&a).unwrap_or(&0)
    }
    /// One TXDR byte (full-duplex). Header is the first 3 bytes [op, addr_be];
    /// after that, data — reads emit register bytes, writes store them.
    fn xfer(&mut self, b: u8) -> u8 {
        self.xfer_cnt += 1; // bytes in this spi-core transfer (drives EOT)
        let out = match self.idx {
            0 => { self.op = b; self.idx += 1; 0 }
            1 => { self.addr = (b as u16) << 8; self.idx += 1; 0 }
            2 => { self.addr |= b as u16; self.idx += 1; 0 }
            _ => {
                if self.op == 1 || self.op == 6 { self.next_data() } // Read / ReadNoAddrIncr
                else { // Write(0) / BitSet(2) / BitClear(3) / WriteNoAddrIncr(5)
                    let incr = self.op != 5; // only WriteNoAddrIncr holds the address
                    let a = self.addr.wrapping_add(if incr { self.dpos } else { 0 } as u16);
                    self.dpos += 1;
                    let cur = *self.fpga.get(&a).unwrap_or(&0);
                    let nv = match self.op {
                        2 => cur | b,  // BitSet: read-modify-write OR
                        3 => cur & !b, // BitClear: read-modify-write AND-NOT
                        _ => b,        // Write / WriteNoAddrIncr: overwrite
                    };
                    self.fpga.insert(a, nv);
                    0
                }
            }
        };
        if crate::dbg::spi() && self.dbg_n < 120 {
            self.dbg_n += 1;
            eprintln!("[spi5] idx={} op={} addr={:#x} dpos={} in={:#04x} out={:#04x}",
                self.idx, self.op, self.addr, self.dpos, b, out);
        }
        out
    }
}
impl Mmio for Spi5 {
    fn name(&self) -> &str { "SPI5" }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x14 => { // SR (EOT/TXC keyed on xfer_cnt, not idx — full-duplex byte count)
                let tsize = self.regs.get(&0x04).copied().unwrap_or(0) & 0xFFFF;
                spi_sr(self.xfer_cnt, tsize, !self.rx.is_empty())
            }
            0x30 => { // RXDR
                if let Some(b) = (!self.rx.is_empty()).then(|| self.rx.remove(0)) {
                    b as u32 // full-duplex: byte produced by a TXDR write
                } else if self.idx >= 3 && (self.op == 1 || self.op == 6) {
                    self.next_data() as u32 // receive-only read: produce on demand
                } else { 0 }
            }
            o => *self.regs.get(&o).unwrap_or(&0),
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        self.check_cs(); // resets per-command state on the CS asserting edge
        match off & !3 {
            0x20 => { let rx = self.xfer(val as u8); self.rx.push(rx); } // TXDR
            0x04 => { self.xfer_cnt = 0; self.regs.insert(0x04, val); // CR2.TSIZE: new transfer → reset EOT count
                if crate::dbg::spi() && self.dbg_n < 120 { eprintln!("[spi5] CR2/TSIZE <- {:#x} (xfer_cnt reset)", val); } }
            o => {
                if crate::dbg::spi() && self.dbg_n < 120 && (o == 0x00) { eprintln!("[spi5] CR1 <- {:#x}", val); }
                self.regs.insert(o, val);
            }
        }
    }
}

/// GPIO bank — store/return, but the read-only input register IDR (+0x10 within
/// each 0x400 port) is synthesized for the boot-critical externally-driven pins:
///  - GPIOC (port 2): PC6/PC7 = sequencer V3P3/V1P2 power-good → bits 6,7 high.
///  - GPIOG (port 6): PG[2:0] = board revision → 0b010 for gimlet-c.
/// Other ports' IDR mirrors their ODR (+0x14) so output read-back works.
pub struct GpioBank { regs: std::collections::HashMap<u32, u32>, cs: Spi2Cs, spi5_cs: Spi5Cs, prev_pj6_low: Cell<bool>, sidecar: bool }
impl GpioBank {
    pub fn new(cs: Spi2Cs, spi5_cs: Spi5Cs) -> Self {
        // $SP_EMU_BOARD selects the board profile for synthesized input pins.
        let sidecar = std::env::var("SP_EMU_BOARD").map(|b| b == "sidecar").unwrap_or(false);
        GpioBank { regs: std::collections::HashMap::new(), cs, spi5_cs, prev_pj6_low: Cell::new(false), sidecar }
    }
}
impl GpioBank {
    /// Recompute the shared SPI2 chip-select from the port-B/port-I ODR state.
    /// CS is active-low: pin driven low selects the device. PB5 → sequencer,
    /// PI0 → KSZ8463 (per app/gimlet/base.toml config.spi.spi2.devices).
    fn update_cs(&self) {
        let pb = *self.regs.get(&(1 * 0x400 + 0x14)).unwrap_or(&0); // GPIOB ODR
        let pi = *self.regs.get(&(8 * 0x400 + 0x14)).unwrap_or(&0); // GPIOI ODR
        self.cs.set(if pb & (1 << 5) == 0 { 1 } else if pi & (1 << 0) == 0 { 2 } else { 0 });
        // Sidecar SPI5 user-design CS = Port J (port 9) pin 6, active-low. Count
        // each deasserted→asserted edge so Spi5 can delimit FPGA commands even
        // when the deassert happens between (Spi5-invisible) GPIO writes.
        let pj = *self.regs.get(&(9 * 0x400 + 0x14)).unwrap_or(&0); // GPIOJ ODR
        let pj6_low = pj & (1 << 6) == 0;
        if pj6_low && !self.prev_pj6_low.get() { self.spi5_cs.set(self.spi5_cs.get().wrapping_add(1)); }
        self.prev_pj6_low.set(pj6_low);
    }
}
impl Mmio for GpioBank {
    fn name(&self) -> &str { "GPIO" }
    fn read(&mut self, off: u32) -> u32 {
        let (port, reg) = (off / 0x400, off & 0x3FF & !3);
        if reg == 0x10 { // IDR
            if port == 4 {
                // GPIOE: PE3 is rot-irq (input from the RoT, active-low). Surface
                // the sprot link's rot_irq on bit 3 so the SP's sprot-server sees it.
                if let Some(lk) = crate::sprot::link() {
                    let odr = *self.regs.get(&(4 * 0x400 + 0x14)).unwrap_or(&0);
                    let bit3 = if lk.borrow().rot_irq { 0 } else { 1 << 3 };
                    return (odr & !(1 << 3)) | bit3;
                }
            }
            if self.sidecar {
                return match port {
                    // GPIOC PC6/PC7/PC13 → board rev[0,1,2]; sidecar-c = 0b010 → PC7 only.
                    2 => 1 << 7,
                    // GPIOF PF12 = front-IO POWER_GOOD (input) → high so the sequencer's
                    // front-IO preinit passes the PG check.
                    5 => self.regs.get(&(5 * 0x400 + 0x14)).copied().unwrap_or(0) | (1 << 12),
                    // GPIOJ: mainboard ECP5 config pins — done=PJ15 high (=configured →
                    // device_state RunningUserDesign, skip bitstream) + program_n=PJ13
                    // high (not in reset). init_n=PJ12 (don't-care once done is high).
                    9 => (1 << 15) | (1 << 13),
                    _ => *self.regs.get(&(port * 0x400 + 0x14)).unwrap_or(&0),
                };
            }
            return match port {
                2 => 0b11 << 6,  // GPIOC: PG lines good
                6 => 0b010,      // GPIOG: gimlet-c board rev
                _ => *self.regs.get(&(port * 0x400 + 0x14)).unwrap_or(&0), // mirror ODR
            };
        }
        *self.regs.get(&(off & !3)).unwrap_or(&0)
    }
    fn write(&mut self, off: u32, val: u32) {
        let (port, reg) = (off / 0x400, off & 0x3FF & !3);
        // BSRR (+0x18): set bits [15:0], reset bits [31:16] → fold into ODR.
        if reg == 0x18 {
            let odr_key = port * 0x400 + 0x14;
            let mut odr = *self.regs.get(&odr_key).unwrap_or(&0);
            odr |= val & 0xFFFF;
            odr &= !(val >> 16);
            self.regs.insert(odr_key, odr);
        } else {
            self.regs.insert(off & !3, val);
        }
        if port == 4 {
            // GPIOE PE4 is the RoT chip-select (active-low). Drive the sprot link CS
            // and latch the SSA/SSD slave-select events on each edge. We latch here,
            // on the SP side, because this write always runs inside the SP's quantum
            // and so never misses a CS edge — unlike the RoT, which only samples the
            // line when it happens to touch its FLEXCOMM8 registers and can sleep
            // through an entire assert→clock→deassert cycle. See SprotLink.
            if let Some(lk) = crate::sprot::link() {
                let odr = *self.regs.get(&(4 * 0x400 + 0x14)).unwrap_or(&0);
                let new_cs = (odr >> 4) & 1 == 0;
                let mut l = lk.borrow_mut();
                if new_cs != l.cs {
                    if new_cs {
                        // CS asserted: start of a transfer. Latch SSA + the SOT bit
                        // for the first FIFORD frame the RoT reads.
                        l.ssa = true;
                        l.sot_pending = true;
                    } else {
                        // CS de-asserted: end of a transfer. Latch SSD.
                        l.ssd = true;
                    }
                    if crate::sprot::dbg() {
                        eprintln!("[gpio] PE4 CS {} (mosi={} miso={})", if new_cs {"ASSERT"} else {"deassert"}, l.mosi.len(), l.miso.len());
                    }
                }
                l.cs = new_cs;
            }
        }
        // GPIOB/GPIOI affect SPI2 CS; GPIOJ (port 9) affects the sidecar SPI5 CS.
        if port == 1 || port == 8 || port == 9 { self.update_cs(); }
    }
}

/// STM32H7 I2C controller — minimal FSM so the driver's transactions complete.
/// ISR (+0x18) always reports TXE|TXIS|RXNE|TC (ready to send / data available /
/// transfer complete) with BUSY and NACKF clear; the driver writes bytes to TXDR
/// (+0x28, discarded) and reads RXDR (+0x24, returns 0). Real devices would
/// return meaningful data — fine for now (turn-off writes succeed; sensor reads
/// read 0). Other registers store/return.
/// Synthetic-but-scriptable physical environment for the modeled sensors. The
/// sensor *chips* are emulated accurately (real register protocol); the physical
/// quantity they'd measure (temperature, …) has no real source in a virtual rack,
/// so it's injected here. This is the honest "must be faked" layer — and unlike a
/// static config it can drive fault scenarios. Configure via env:
///   SP_EMU_AMBIENT_C=<°C>             default temperature for every sensor
///   SP_EMU_SENSORS=0x48=45.0,0x18=60  per-address °C overrides
pub struct SensorEnv {
    default_temp_c: f32,
    temp_override: std::collections::HashMap<u8, f32>,
}
pub type Sensors = Rc<RefCell<SensorEnv>>;
impl SensorEnv {
    pub fn from_env() -> Sensors {
        let default_temp_c = std::env::var("SP_EMU_AMBIENT_C").ok()
            .and_then(|s| s.trim().parse().ok()).unwrap_or(30.0);
        let mut temp_override = std::collections::HashMap::new();
        if let Ok(s) = std::env::var("SP_EMU_SENSORS") {
            for kv in s.split(',') {
                if let Some((a, v)) = kv.split_once('=') {
                    let a = a.trim().trim_start_matches("0x");
                    if let (Ok(addr), Ok(t)) = (u8::from_str_radix(a, 16), v.trim().parse::<f32>()) {
                        temp_override.insert(addr, t);
                    }
                }
            }
        }
        Rc::new(RefCell::new(SensorEnv { default_temp_c, temp_override }))
    }
    fn temp_c(&self, addr: u8) -> f32 {
        *self.temp_override.get(&addr).unwrap_or(&self.default_temp_c)
    }
}

/// CRC-32/ISCSI (Castagnoli) — the body checksum the `tlvc` crate uses
/// (`crc::CRC_32_ISCSI`). Reflected poly 0x82F6_3B78, init/xorout 0xFFFF_FFFF.
fn crc32c(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0x82F6_3B78 } else { crc >> 1 };
        }
    }
    !crc
}

/// TLV-C header checksum — mirrors `tlvc::header_checksum` exactly:
/// `!(le_u32(tag).wrapping_mul(HEADER_MAGIC).wrapping_add(len))`.
fn tlvc_header_checksum(tag: [u8; 4], len: u32) -> u32 {
    const HEADER_MAGIC: u32 = 0x6b32_9f69;
    !u32::from_le_bytes(tag).wrapping_mul(HEADER_MAGIC).wrapping_add(len)
}

/// Serialize one TLV-C chunk: header { tag, len(LE), header_checksum(LE) },
/// then the body, zero-padded to a 4-byte boundary, then the body CRC (LE).
fn tlvc_chunk(tag: &[u8; 4], body: &[u8]) -> Vec<u8> {
    let len = body.len() as u32;
    let mut v = Vec::new();
    v.extend_from_slice(tag);
    v.extend_from_slice(&len.to_le_bytes());
    v.extend_from_slice(&tlvc_header_checksum(*tag, len).to_le_bytes());
    v.extend_from_slice(body);
    while v.len() % 4 != 0 { v.push(0); } // header is 12B, so this pads the body
    v.extend_from_slice(&crc32c(body).to_le_bytes());
    v
}

/// Build the 1024-byte AT24CSW080 VPD image the sidecar firmware expects:
/// a `FRU0` root whose body holds a `MAC0` chunk (task_packrat_api::MacAddressBlock
/// = base_mac[6] + count(u16 LE) + stride(u8)) and a `BARC` chunk (an 0XV2 Oxide
/// barcode string). This lets drv_packrat_vpd_loader::read_vpd_and_load_packrat
/// succeed on the first attempt instead of mem-faulting on garbage. Non-sidecar
/// boards get a blank (all-0xFF) EEPROM, preserving the proven gimlet behavior
/// (its sharkfin VPD reads fail cleanly as "Truncated", which the firmware
/// tolerates).
/// STM32H7 HASH (0x4802_1400, irq 80). Minimal model so drv-stm32h7-hash
/// completes: report DINIS (ready for data) + not BUSY, and when the driver
/// writes STR.DCAL (start digest) set SR.DCIS and raise irq 80 so its
/// `sys_recv_notification` wakes. Returns a fixed digest — MGS only records the
/// phase1 hash for inventory; it is not checked against the flash here.
struct Hash {
    irq_pending: bool,
    dcis: bool,
}
impl Hash {
    pub fn new() -> Self { Hash { irq_pending: false, dcis: false } }
}
impl Mmio for Hash {
    fn name(&self) -> &str { "HASH" }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x24 => (1 << 0) | if self.dcis { 1 << 1 } else { 0 }, // SR: DINIS + DCIS, BUSY=0
            0x0C | 0x10 | 0x14 | 0x18 | 0x1C => 0xA5A5_A5A5,       // HR0-4
            o if (0x310..=0x32C).contains(&o) => 0xA5A5_A5A5,      // HR0-7 alias (SHA-256)
            _ => 0,
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        match off & !3 {
            0x00 => { if val & (1 << 2) != 0 { self.dcis = false; } }                 // CR.INIT
            0x08 => { if val & (1 << 8) != 0 { self.dcis = true; self.irq_pending = true; } } // STR.DCAL
            _ => {}
        }
    }
    fn take_irq(&mut self) -> Option<u16> {
        if self.irq_pending { self.irq_pending = false; Some(80) } else { None }
    }
}

fn build_vpd_eeprom() -> Rc<Vec<u8>> {
    let mut img = vec![0xFFu8; 1024];
    let sidecar = std::env::var("SP_EMU_BOARD").map(|b| b == "sidecar").unwrap_or(false);
    // Per-instance index from the bridge port (33300->0, 33310->1, ...) so the
    // emulated gimlet SPs get DISTINCT serials AND MACs. Inventory keys SPs on
    // serial, and a shared MAC (the old blank-VPD gimlet default) caused L2
    // collisions => intermittent "no answer" on the management net.
    let idx: u8 = std::env::var("SP_EMU_BRIDGE").ok()
        .and_then(|b| b.rsplit(':').next().map(str::to_string))
        .and_then(|p| p.parse::<u32>().ok())
        .map(|p| ((p.wrapping_sub(33300)) / 10) as u8)
        .unwrap_or(0);
    // MAC0: 128-MAC block. sidecar base ...45:30; gimlets ...45:21/22/23.
    let mac_last = if sidecar { 0x30 } else { 0x20u8.wrapping_add(idx) };
    let mut mac0 = Vec::new();
    mac0.extend_from_slice(&[0x0e, 0x1d, 0xb7, 0xfe, 0x45, mac_last]); // base_mac
    mac0.extend_from_slice(&128u16.to_le_bytes());                    // count
    mac0.push(1);                                                     // stride
    // BARC: 0XV2 barcode "version:part(<=11):rev:serial(<=11)".
    let serial = if sidecar { "BRM42220001".to_string() } else { format!("BRM4422000{}", idx) };
    let barc = format!("0XV2:913-0000019:002:{}", serial);
    let mut fru0 = tlvc_chunk(b"MAC0", &mac0);
    fru0.extend_from_slice(&tlvc_chunk(b"BARC", barc.as_bytes()));
    let root = tlvc_chunk(b"FRU0", &fru0);
    img[..root.len()].copy_from_slice(&root);
    Rc::new(img)
}

pub struct I2c {
    regs: std::collections::HashMap<u32, u32>,
    ev_irq: u16,
    active: bool,
    env: Sensors,
    // --- transaction state, so we can model real device registers ---
    addr: u8,        // current 7-bit target (from CR2.SADD)
    reg_ptr: u8,     // device register pointer (from the write phase)
    read_idx: u16,   // byte index within the current read phase
    writing: bool,   // current phase is a master write (register-pointer set)
    wrote_ptr: bool, // captured the register-pointer byte this write phase
    eeprom: Rc<Vec<u8>>, // AT24CSW080 VPD/FRUID backing store (1024 bytes)
    bridge: crate::i2c_bridge::I2cBridge, // SP_EMU_I2C_BRIDGE sniff / _DEVICE delegate (no-op when off)
    bus: u8,         // 1-based bus number (i2c1..i2c4) for the trace
}
impl I2c {
    pub fn new(
        ev_irq: u16,
        env: Sensors,
        eeprom: Rc<Vec<u8>>,
        bridge: crate::i2c_bridge::I2cBridge,
        bus: u8,
    ) -> Self {
        I2c { regs: std::collections::HashMap::new(), ev_irq, active: false, env,
            addr: 0, reg_ptr: 0, read_idx: 0, writing: false, wrote_ptr: false, eeprom, bridge, bus }
    }
    /// Accurate device-register model, keyed by I2C address. Returns the 16-bit
    /// value of `reg` (drivers read big-endian: high byte first; single-byte reads
    /// take the high byte). Physical values come from the SensorEnv. `None` = no
    /// modeled device here → bus reads 0 → that device honestly stays "Failed"
    /// until modeled. Add accurate devices by extending this match.
    fn device_reg(&self, addr: u8, reg: u8) -> Option<u16> {
        let env = self.env.borrow();
        match addr {
            // TMP117 temperature sensors (front/rear, 0x48-0x4a): 7.8125 m°C/LSB,
            // DeviceID must read 0x0117.
            0x48 | 0x49 | 0x4a => Some(match reg {
                0x0f => 0x0117,                                          // DeviceID
                0x00 => (env.temp_c(addr) / 0.0078125) as i16 as u16,    // TempResult
                0x01 => 0x0220,                                          // Configuration
                _ => 0,
            }),
            // TSE2004av DIMM temp sensors (bus "mid", 0x18-0x1f): DeviceIdRevision
            // upper byte must be 0x22; AmbientTemp is a 13-bit value (raw = °C*16).
            0x18..=0x1f => Some(match reg {
                0x07 => 0x2200,                                          // DeviceIdRevision
                0x05 => (((env.temp_c(addr) / 0.0078125) as i16 >> 3) as u16) & 0x1fff, // AmbientTemp
                _ => 0,
            }),
            // AT24CSW080 VPD/FRUID EEPROMs are handled out-of-band in the RXDR
            // read path (addresses 0x50..0x53), not here — see the Mmio::read
            // EEPROM branch. They need real sequential, auto-incrementing reads
            // off the `eeprom` backing store, not the 16-bit-register split below.
            // TMP451 (T6 NIC temp, behind the M.2 mux seg 4, addr 0x4c). Reads are
            // SINGLE-byte → the value goes in the high byte (read_idx 0). ManufacturerId
            // (0xFE) must be 0x55 (TI); Local/Remote temp hi byte = integer °C.
            0x4c => Some(match reg {
                0xFE => 0x5500,                                          // ManufacturerId = 0x55
                0x00 | 0x01 => ((env.temp_c(addr) as i16) << 8) as u16,  // Local/Remote temp hi byte
                _ => 0,
            }),
            _ => None,
        }
    }
}
impl Mmio for I2c {
    fn name(&self) -> &str { "I2C" }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x18 => (1 << 0) | (1 << 1) | (1 << 2) | (1 << 6), // ISR: TXE|TXIS|RXNE|TC
            0x24 => { // RXDR: serve the modeled device register / EEPROM byte
                if crate::dbg::vpd() {
                    eprintln!("[i2c{:#x}] RD RXDR addr={:#04x} ptr={} ridx={}", self.ev_irq, self.addr, self.reg_ptr, self.read_idx);
                }
                // DELEGATE (SP_EMU_I2C_DEVICE): a local device server may answer
                // this read; `None` falls through to the built-in model below.
                if let Some(b) = self.bridge.on_read(self.bus, self.addr, self.reg_ptr, self.read_idx) {
                    self.read_idx = self.read_idx.wrapping_add(1);
                    return b as u32;
                }
                // AT24CSW080 (0x50..0x53): the EEPROM folds address bits A9:A8 into
                // the I2C device address and uses a single-byte word pointer (the
                // driver writes addr-low, then reads N bytes with auto-increment).
                // So the EEPROM offset = ((addr & 3) << 8) | reg_ptr, advancing one
                // byte per RXDR read. (drv-i2c-devices/at24csw080.rs read_into.)
                if (0x50..=0x53).contains(&self.addr) {
                    let off = ((self.addr as u16 & 3) << 8) | self.reg_ptr as u16;
                    let idx = (off.wrapping_add(self.read_idx) & 0x3FF) as usize;
                    let byte = self.eeprom[idx];
                    if crate::dbg::vpd() {
                        eprintln!("[vpd] rd addr={:#04x} ptr={} ridx={} off={} -> {:#04x}",
                            self.addr, self.reg_ptr, self.read_idx, idx, byte);
                    }
                    self.bridge.on_read_served(self.bus, self.addr, self.reg_ptr, self.read_idx, byte);
                    self.read_idx = self.read_idx.wrapping_add(1);
                    return byte as u32;
                }
                let v = self.device_reg(self.addr, self.reg_ptr).unwrap_or(0);
                let byte = if self.read_idx == 0 { (v >> 8) & 0xFF } else { v & 0xFF };
                self.bridge.on_read_served(self.bus, self.addr, self.reg_ptr, self.read_idx, byte as u8);
                self.read_idx = self.read_idx.wrapping_add(1);
                byte as u32
            }
            o => *self.regs.get(&o).unwrap_or(&0),
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        if crate::dbg::vpd() && self.ev_irq == 95 {
            eprintln!("[i2c4] WR off={:#05x} val={:#010x}", off & !3, val);
        }
        if off & !3 == 0x28 { // TXDR: first byte of a write phase is the register pointer
            let byte = (val & 0xFF) as u8;
            if self.writing && !self.wrote_ptr { self.reg_ptr = byte; self.wrote_ptr = true; }
            self.bridge.on_write(self.bus, self.addr, byte);
            return;
        }
        if off & !3 == 0x04 { // CR2: START begins a master transfer, STOP ends it.
            if val & (1 << 13) != 0 { // START
                self.active = true;
                self.addr = ((val >> 1) & 0x7F) as u8; // SADD[7:1] = 7-bit address
                if val & (1 << 10) != 0 { // RD_WRN set → read phase
                    self.read_idx = 0;
                    self.writing = false;
                } else { // write phase (sets the register pointer)
                    self.writing = true;
                    self.wrote_ptr = false;
                }
                if crate::dbg::vpd() {
                    eprintln!("[i2c{:#x}] START addr={:#04x} rd={} nbytes={}", self.ev_irq, self.addr,
                        (val >> 10) & 1, (val >> 16) & 0xFF);
                }
                self.bridge.on_start(self.bus, self.addr, val & (1 << 10) != 0, (val >> 16) & 0xFF);
            }
            if val & (1 << 14) != 0 { self.active = false; } // STOP
            // START/STOP are command bits that auto-clear in hardware; store them
            // cleared so a later read-modify-write doesn't carry a stale START.
            self.regs.insert(0x04, val & !((1 << 13) | (1 << 14)));
            return;
        }
        self.regs.insert(off & !3, val);
    }
    // The master read path waits (wfi) for the event IRQ before checking RXNE.
    // Raise it only while a master transfer is active (CR2.START..STOP) so I2C
    // slave mode (gimlet-spd's operate_as_target, which never gets addressed in
    // the emulator) stays quietly blocked instead of busy-looping on stray IRQs.
    fn take_irq(&mut self) -> Option<u16> { if self.active { Some(self.ev_irq) } else { None } }
}

/// STM32H7 QUADSPI — minimal model so the host-flash driver's transfers finish.
/// SR(+0x08) always reports TCF|FTF (transfer complete / FIFO ready), BUSY clear;
/// DR(+0x20) reads 0xFF (erased flash); FCR(+0x0C) flag clears accepted.
/// QUADSPI (0x5200_5000) — command-aware model of the gimlet host flash so the
/// `hf` task's init completes (it reads the JEDEC ID and *fails hard* unless it
/// recognizes the chip, then scans for persistent data). We answer just enough:
///
///   * RDID (0x9F)      -> [0x20, 0xBA, 0x19, ...]  (Micron MT25Q, 32 MiB:
///                         byte0=Micron, byte1=3.3V, byte2=log2(capacity)=25)
///   * RDSR (0x05)      -> 0x00                     (status: not busy, WIP=0)
///   * memory reads     -> 0xFF                     (blank flash => the hf
///                         persistent-data scan finds nothing => clean
///                         "initial power-on" path, no writes)
///   * writes/erase     -> accepted (discarded)
///
/// The driver (drv-stm32h7-qspi) drives transfers by polling SR.FLEVEL (FIFO
/// level, bits 8..13) and SR.TCF (transfer complete, bit1), reading data one
/// byte at a time from DR (offset 0x20) via byte-wide accesses. We present the
/// whole response immediately with a large FLEVEL and set TCF once it is
/// drained, so the driver never has to wait on the qspi-irq — which sidesteps
/// the busy-loop/irq-storm the naive stub caused.
pub struct Qspi {
    dlr: u32,           // transfer length register (holds len-1)
    resp: Vec<u8>,      // pending read response
    resp_pos: usize,    // bytes drained from `resp`
    mode_read: bool,    // current transfer is an indirect read
    tcf: bool,          // transfer-complete latch
    cr: u32,            // control register (stored, for EN bit etc.)
    dcr: u32,           // device config (stored)
}
impl Qspi {
    pub fn new() -> Self {
        Qspi { dlr: 0, resp: Vec::new(), resp_pos: 0, mode_read: false, tcf: false, cr: 0, dcr: 0 }
    }
    fn build_response(&self, instruction: u8, len: usize) -> Vec<u8> {
        let mut v = vec![0u8; len];
        match instruction {
            0x9F => { // ReadId (RDID): Micron MT25Q, 32 MiB (log2 capacity = 0x19)
                let id = [0x20u8, 0xBA, 0x19];
                for (i, b) in id.iter().enumerate() { if i < len { v[i] = *b; } }
            }
            0x05 => { /* ReadStatusReg: 0x00 (not busy) — zeros already */ }
            // Read / QuadRead / DdrRead / page-data etc.: erased NOR flash.
            _ => { for b in v.iter_mut() { *b = 0xFF; } }
        }
        v
    }
}
impl Mmio for Qspi {
    fn name(&self) -> &str { "QUADSPI" }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x00 => self.cr,
            0x04 => self.dcr,
            0x08 => { // SR
                let remaining = self.resp.len().saturating_sub(self.resp_pos);
                if self.mode_read && remaining == 0 { self.tcf = true; }
                let flevel = if self.mode_read { remaining.min(32) as u32 } else { 0 };
                let mut sr = 0u32;
                if self.tcf { sr |= 1 << 1; }              // TCF
                if !self.mode_read || remaining > 0 { sr |= 1 << 2; } // FTF
                sr |= flevel << 8;                          // FLEVEL[5:0]
                sr
            }
            0x10 => self.dlr,
            0x20 => { // DR: pop one byte (driver reads the low byte via byte access)
                if self.resp_pos < self.resp.len() {
                    let b = self.resp[self.resp_pos] as u32;
                    self.resp_pos += 1;
                    b
                } else { 0xFF }
            }
            _ => 0,
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        match off & !3 {
            0x00 => self.cr = val,
            0x04 => self.dcr = val,
            0x0C => { if val & (1 << 1) != 0 { self.tcf = false; } } // FCR.CTCF
            0x10 => self.dlr = val,                                  // DLR (len-1)
            0x14 => { // CCR: instruction in bits[7:0], FMODE in bits[27:26]
                let instruction = (val & 0xFF) as u8;
                let fmode = (val >> 26) & 0b11;
                if fmode == 0b01 { // indirect read
                    let len = (self.dlr as usize).wrapping_add(1);
                    self.resp = self.build_response(instruction, len);
                    self.resp_pos = 0;
                    self.mode_read = true;
                    self.tcf = false;
                } else { // indirect write (write-enable / program / erase): instant
                    self.mode_read = false;
                    self.tcf = true;
                }
                if crate::dbg::eth() {
                    eprintln!("[qspi] CCR instr={:#04x} fmode={:#b} dlr={}", instruction, fmode, self.dlr);
                }
            }
            _ => {} // AR, DR-writes, interrupt-enable bits in CR: accept/ignore
        }
    }
    // No irq needed: the driver completes by polling FLEVEL/TCF, which we always
    // satisfy immediately. Staying quiet keeps hf from busy-looping.
    fn take_irq(&mut self) -> Option<u16> { None }
}

/// SYSCFG — store/return except PKGR (+0x124), whose pkg[3:0] field reads back
/// 0b1000 (TFBGA240) so gimlet's package-guard accepts the firmware.
pub struct Syscfg { regs: std::collections::HashMap<u32, u32> }
impl Syscfg { pub fn new() -> Self { Syscfg { regs: std::collections::HashMap::new() } } }
impl Mmio for Syscfg {
    fn name(&self) -> &str { "SYSCFG" }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x124 => (*self.regs.get(&0x124).unwrap_or(&0) & !0xF) | 0b1000, // PKGR.pkg
            o => *self.regs.get(&o).unwrap_or(&0),
        }
    }
    fn write(&mut self, off: u32, val: u32) { self.regs.insert(off & !3, val); }
}

/// STM32H7 unique device ID (96 bits / 3 words) — a stable fake identity.
pub struct Uid;
impl Mmio for Uid {
    fn name(&self) -> &str { "UID" }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 { 0x0 => 0x5350_4D45, 0x4 => 0x2D45_4D55, _ => 0x0000_0001 }
    }
    fn write(&mut self, _: u32, _: u32) {}
}

/// Generic peripheral that stores writes and returns them (sparse) — models
/// config registers whose only requirement is readback consistency.
pub struct RegFile { name: &'static str, regs: std::collections::HashMap<u32, u32> }
impl RegFile { pub fn new(name: &'static str) -> Self { RegFile { name, regs: std::collections::HashMap::new() } } }
impl Mmio for RegFile {
    fn name(&self) -> &str { self.name }
    fn read(&mut self, off: u32) -> u32 { *self.regs.get(&(off & !3)).unwrap_or(&0) }
    fn write(&mut self, off: u32, val: u32) { self.regs.insert(off & !3, val); }
}

// ---- RCC: clock tree. Ready bits mirror their enable bits. -----------------

pub struct Rcc {
    regs: [u32; 0x100],
}

impl Rcc {
    pub fn new() -> Self { Rcc { regs: [0; 0x100] } }
}

impl Mmio for Rcc {
    fn name(&self) -> &str { "RCC" }
    fn read(&mut self, off: u32) -> u32 {
        let i = (off / 4) as usize & 0xff;
        let mut v = self.regs[i];
        match off {
            0x00 => {
                // CR: synthesize *RDY immediately from each *ON request.
                v |= 1 << 1; // HSIRDY  (HSI is always running out of reset)
                v |= 1 << 2; // HSIDIVF / CSIRDY-ish stand-in (harmless)
                if v & (1 << 16) != 0 { v |= 1 << 17; } // HSEON  -> HSERDY
                if v & (1 << 24) != 0 { v |= 1 << 25; } // PLL1ON -> PLL1RDY
                if v & (1 << 26) != 0 { v |= 1 << 27; } // PLL2ON -> PLL2RDY
                if v & (1 << 28) != 0 { v |= 1 << 29; } // PLL3ON -> PLL3RDY
            }
            0x10 => {
                // CFGR: SWS (bits 5:3) tracks the requested SW (bits 2:0),
                // so `while sws() != PLL1` terminates once SW selects PLL1.
                let sw = v & 0x7;
                v = (v & !(0x7 << 3)) | (sw << 3);
            }
            _ => {}
        }
        v
    }
    fn write(&mut self, off: u32, val: u32) {
        let i = (off / 4) as usize & 0xff;
        self.regs[i] = val;
    }
}

// ---- PWR: voltage scaling. All "ready" bits read back set. ------------------

pub struct Pwr {
    regs: [u32; 0x40],
}

impl Pwr {
    pub fn new() -> Self { Pwr { regs: [0; 0x40] } }
}

impl Mmio for Pwr {
    fn name(&self) -> &str { "PWR" }
    fn read(&mut self, off: u32) -> u32 {
        let i = (off / 4) as usize & 0x3f;
        let v = self.regs[i];
        match off {
            0x04 => v | (1 << 13),       // CSR1.ACTVOSRDY  (startup spins on this)
            0x18 => v | (1 << 13),       // D3CR.VOSRDY     (offset 0x18 on STM32H7)
            _ => v,
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        let i = (off / 4) as usize & 0x3f;
        self.regs[i] = val;
    }
}

// ---- SCS: ARM System Control Space (SysTick, NVIC, SCB, CPACR). -------------
//
// Fixed by the architecture at 0xE000_E000 (not in chip.toml). Phase 1 stores
// register writes and logs the architecturally interesting ones (VTOR, CPACR,
// SysTick) so we can see the kernel bring the system up. NVIC interrupt
// delivery is Phase 1.5.

pub struct Scs {
    regs: [u32; 0x400], // 0x1000 bytes / 4
}

impl Scs {
    pub fn new() -> Self { Scs { regs: [0; 0x400] } }
}

impl Mmio for Scs {
    fn name(&self) -> &str { "SCS" }
    fn read(&mut self, off: u32) -> u32 {
        let i = (off / 4) as usize & 0x3ff;
        match off {
            0x010 => self.regs[i] | (1 << 16), // SYST_CSR: report COUNTFLAG set
            0xD00 => 0x4110_FC27,              // CPUID (Cortex-M7 r1p1-ish)
            _ => self.regs[i],
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        let i = (off / 4) as usize & 0x3ff;
        self.regs[i] = val;
        match off {
            0xD08 => eprintln!("[scs] VTOR  = {:#010x}", val),
            0xD88 => eprintln!("[scs] CPACR = {:#010x} (FPU enable)", val),
            _ => {}
        }
    }
}

