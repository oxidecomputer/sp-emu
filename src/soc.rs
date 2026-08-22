// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! STM32H753 SoC assembly + the boot-critical peripheral models.
//!
//! "Boot-critical" here means exactly the peripherals whose busy-wait status
//! bits the Hubris startup path (`drv/stm32h7-startup`) spins on; if these
//! don't read back "ready", the firmware hangs before reaching the kernel:
//!   PWR.CSR1.ACTVOSRDY, PWR.D3CR.VOSRDY, RCC.CR.HSERDY, RCC.CR.PLL1RDY,
//!   RCC.CFGR.SWS == PLL1.
//! Everything else can start life as a stub (reads 0, swallows writes).

use crate::flash::ERASED;
use crate::mem::{Bus, Mmio};
use std::cell::{Cell, RefCell};
use std::rc::Rc;

/// Which device is selected on a shared SPI bus, derived from the active (low)
/// chip-select GPIO. The GPIO bank sets this; the SPI peripheral reads it to
/// route a transaction. 0 = none/other, 1 = sequencer FPGA (PB5), 2 = KSZ8463 (PI0).
pub type Spi2Cs = Rc<Cell<u8>>;

/// Sidecar SPI5 user-design chip-select "assert generation", incremented by the
/// GPIO bank each time PJ6 (CS_USER_L) goes asserted (low). Each FPGA command is
/// one CS lock (header write + data read across two SPE cycles); Spi5 resets its
/// per-command state whenever this counter changes. A counter (not a bool) is
/// used because the deassert between commands happens with no Spi5 access, so a
/// bool edge would be missed; the GPIO bank sees every PJ6 write and counts.
pub type Spi5Cs = Rc<Cell<u32>>;

// ---- standard STM32H7 memory map (gimletlet uses the AXI-SRAM layout) -------

pub fn install_memory(bus: &mut Bus) {
    // The 2 MB flash window (0x0800_0000, both banks) is not a flat RAM: it is the
    // `Flash` model installed via `bus.install_flash()` in `setup()`, which gives
    // it real program/erase/bank-swap/persistence semantics.
    bus.add_ram(0x0000_0000, 0x0001_0000); // ITCM (also boot alias)
    bus.add_ram(0x2000_0000, 0x0002_0000); // DTCM (128 KB)
    bus.add_ram(0x2400_0000, 0x0008_0000); // AXI SRAM (512 KB); initial SP lives here
    bus.add_ram(0x3000_0000, 0x0004_8000); // SRAM1/2/3 (D2)
    bus.add_ram(0x3800_0000, 0x0001_0000); // SRAM4 (D3)
    bus.add_ram(0x3880_0000, 0x0000_1000); // Backup SRAM
}

pub fn install_peripherals(bus: &mut Bus) {
    bus.add_device(0x5802_4400, 0x400, Box::new(Rcc::new()));
    bus.add_device(0x5802_4800, 0x400, Box::new(Pwr::new()));
    bus.add_device(0xE000_E000, 0x1000, Box::new(Scs::new())); // SysTick/NVIC/SCB/CPACR
                                                               // The FLASH controller (0x5200_2000) and the flash memory aperture are the
                                                               // `Flash` model owned by the Bus (installed in `setup()`), not a device here.

    // Ethernet MAC/MTL/DMA (0x40028000) is modeled directly in the Bus (src/mem.rs
    // `EthDma`): its DMA engine needs to read/write descriptor rings + packet
    // buffers in RAM, which a standalone `Mmio` device can't reach.

    // TIM16 (MDIO bit-timer for the eth driver): raises its IRQ when armed.
    bus.add_device(0x4001_4400, 0x400, Box::new(Tim16::new()));

    // SPI4 + the KSZ8463 switch behind it (net's management interface).
    if let Some(lk) = crate::sprot::link() {
        bus.add_device(
            0x4001_3400,
            0x400,
            Box::new(crate::sprot::SpiMaster::new(lk)),
        ); // SP<->RoT sprot link
    } else {
        bus.add_device(0x4001_3400, 0x400, Box::new(Spi4::new()));
    }

    // GPIO bank (0x5802_0000, ports A-K @ 0x400 each). Store/return except the
    // input-data register IDR (+0x10): gimlet's boot polls power-good + board-rev
    // pins that are externally driven, so synthesize them per port. The bank also
    // drives the shared SPI2 chip-select (PB5=sequencer, PI0=KSZ8463).
    let spi2_cs: Spi2Cs = Rc::new(Cell::new(0));
    let spi5_cs: Spi5Cs = Rc::new(Cell::new(0));
    bus.add_device(
        0x5802_0000,
        0x3000,
        Box::new(GpioBank::new(spi2_cs.clone(), spi5_cs.clone())),
    );

    // SPI bus wiring differs by board. On gimlet, SPI2 (0x4000_3800) is the iCE40
    // sequencer FPGA + KSZ8463, CS-routed. On the sidecar, SPI2 is monorail's
    // VSC7448 management switch, net's KSZ8463 is on SPI3 (0x4000_3C00), and the
    // mainboard ECP5 (drv-fpga-server) is on SPI5 (0x4001_5000). The sidecar
    // devices are only installed for that board so they don't shadow gimlet's map.
    if crate::config::get().board().is_sidecar() {
        bus.add_device(0x4000_3800, 0x400, Box::new(Vsc7448::new())); // monorail ⇄ VSC7448
        bus.add_device(0x4000_3C00, 0x400, Box::new(Spi4::new())); // net ⇄ KSZ8463 (reuse KSZ model)
        bus.add_device(0x4001_5000, 0x400, Box::new(Spi5::new(spi5_cs)));
    } else {
        bus.add_device(0x4000_3800, 0x400, Box::new(Spi2::new(spi2_cs))); // gimlet sequencer/KSZ
    }

    // I2C controllers (gimlet: i2c1 spd, i2c2/3/4 sensors/power). Minimal FSM
    // model: report ready/complete so the driver's transactions succeed (writes
    // accepted, reads return 0), letting gimlet_seq's vcore_soc_off + sensors pass.
    // One shared sensor environment (scriptable physical values) across controllers.
    let sensors = SensorEnv::from_env();
    let vpd = build_vpd_eeprom();
    // One shared I2C bridge socket (SP_EMU_I2C_BRIDGE sniff / SP_EMU_I2C_DEVICE
    // delegate) carries every bus.
    let bridge = crate::i2c_bridge::I2cBridge::from_env();
    for (i, (base, ev_irq)) in [
        (0x4000_5400u32, 31u16),
        (0x4000_5800, 33),
        (0x4000_5C00, 72),
        (0x5800_1C00, 95),
    ]
    .into_iter()
    .enumerate()
    {
        let dev = I2c::new(
            ev_irq,
            sensors.clone(),
            vpd.clone(),
            bridge.clone(),
            (i + 1) as u8,
        );
        bus.add_device(base, 0x400, Box::new(dev));
    }

    // STM32H7 HASH (0x4802_1400, irq 80): gimlet's hash_driver starts a digest
    // (STR.DCAL) then blocks on the HASH irq. Unmodeled, that irq never fires =>
    // hash_driver never replies => hf (host-flash) waits forever => CPA deadlocks
    // on `send to hf`, stalling the gimlet SP when MGS does its inventory phase1
    // host-flash hash. Modeled below so the digest completes.
    // Size must be 0x400: the HASH block is 0x4802_1400..0x4802_1800, and the RNG
    // starts at 0x4802_1800. A wider span would shadow the RNG, whose status
    // register would then read back 0 from HASH's fallthrough.
    bus.add_device(0x4802_1400, 0x400, Box::new(Hash::new()));

    // STM32H7 RNG (0x4802_1800): the true random number generator. gimlet's
    // rng_driver enables it, polls SR.DRDY, and reads DR to seed packrat's
    // ereport restart id (drv-stm32h7-rng, `packrat`/`ereport` feature).
    // Unmodeled, SR reads 0 from the catch-all below so DRDY never sets: the rng
    // task spins forever, packrat never gets a restart id, and the snitch drops
    // every ereport request (EreportReadError::RestartIdNotSet), so MGS ereport
    // polls get no reply and time out. Modeled below so entropy is always ready.
    bus.add_device(0x4802_1800, 0x400, Box::new(Rng::new()));

    // QUADSPI (0x5200_5000): writable NOR-flash model. On gimlet the `hf` task
    // drives it (host flash: RDID must answer a recognized Micron 32 MiB chip or
    // hf's init fails, blocking gimlet_seq's A0 transition and thus MGS `state`);
    // on sidecar the `auxflash` task does (FPGA-blob slots: the MGS SP-update
    // path erases + programs a slot and re-reads its CHCK). See the Qspi impl.
    bus.add_device(0x5200_5000, 0x400, Box::new(Qspi::new()));

    // SYSCFG (0x5800_0400): gimlet's kernel reads PKGR (+0x124) on boot and
    // panics unless pkg[3:0] == 0b1000 (TFBGA240), a guard against flashing
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
    // SPI, I2C, USART, timers, ...). Added last so the specific devices above
    // (RCC/PWR/FLASH, which synthesize ready bits) take precedence. Supports
    // readback-style peripheral use and keeps the differential harness in
    // sync; status-bit polls that need real hardware still read 0 (a task that
    // depends on them blocks until interrupt delivery is modeled).
    bus.add_device(0x4000_0000, 0x2000_0000, Box::new(RegFile::new("periph")));
}

/// TIM16 (0x40014400), used by the `net`/eth driver as the MDIO bit-timer. The
/// driver arms it as a one-pulse timer (CR1.CEN=1), then blocks on its IRQ
/// (mdio-timer-irq = IRQ 117). Arming raises the IRQ once, sets SR.UIF, and
/// self-clears CR1.CEN so the driver's `while cen {}` wait breaks.
pub struct Tim16 {
    regs: std::collections::HashMap<u32, u32>,
    armed: bool,
}
impl Tim16 {
    pub fn new() -> Self {
        Tim16 {
            regs: std::collections::HashMap::new(),
            armed: false,
        }
    }
}
impl Mmio for Tim16 {
    fn name(&self) -> &str {
        "TIM16"
    }
    fn read(&mut self, off: u32) -> u32 {
        *self.regs.get(&(off & !3)).unwrap_or(&0)
    }
    fn write(&mut self, off: u32, val: u32) {
        self.regs.insert(off & !3, val);
        if off & !3 == 0x00 && val & 1 != 0 {
            self.armed = true;
        } // CR1.CEN set
    }
    fn take_irq(&mut self) -> Option<u16> {
        if !self.armed {
            return None;
        }
        self.armed = false;
        *self.regs.entry(0x10).or_insert(0) |= 1; // SR.UIF (update interrupt flag)
        *self.regs.entry(0x00).or_insert(0) &= !1; // CR1.CEN self-clears (one-pulse)
        Some(117)
    }
}

/// Shared byte queue between the UART7 device and the host bridge, the same
/// Rc-sharing idiom as `Spi2Cs`. TX = SP->host, RX = host->SP. `Bus::pump_uart`
/// drains/fills these against the `HostIo` (the propolis IPCC COM port).
pub type UartQueue = Rc<RefCell<std::collections::VecDeque<u8>>>;

/// UART7 (0x4000_7800, IRQ 82): the SP<->host-CPU link the real Hubris
/// `host_sp_comms` task drives (host-sp-comms / IPCC + the host serial console).
/// Sufficient for the unmodified `drv-stm32h7-usart` (matches the task's
/// captured register usage): the transmit path is always ready
/// (TXFNF|TC|TXFE + TEACK|REACK), so a write to TDR pushes straight to the host
/// TX queue; a byte in the RX queue sets ISR.RXNE and a read of RDR pops it. The
/// RX interrupt (IRQ 82) is raised Bus-side in `collect_irqs` while the RX queue
/// is non-empty (level-triggered, matching the H7 FIFO RXFNE the task enables);
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
            dbg: crate::config::get().uartdbg(),
        }
    }
}
impl Mmio for Uart7 {
    fn name(&self) -> &str {
        "UART7"
    }
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
                    if let Some(b) = b {
                        eprintln!("[uart7] RX {:#04x}", b);
                    }
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
            if self.dbg {
                eprintln!("[uart7] TX {:#04x}", val & 0xff);
            }
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
            0x0C => 0x8452, // CIDER (register 0x000): KSZ8463 chip id; driver requires this
            // IADR5 (register 0x02E, packed 0x2F0): high word of an indirect-access
            // MIB-counter read. read_mib_counter() spin-loops here until bit 14
            // ("valid") is set; counter value 0 -> driver returns Count(0).
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
/// bytes 2..4. Master is modeled synchronously (each TX byte immediately
/// produces its RX byte), so the driver's transfer loop never sleeps on
/// spi-irq. The KSZ is mostly store/return, except CIDER (chip id) = 0x8452,
/// which the driver checks before proceeding.
pub struct Spi4 {
    regs: std::collections::HashMap<u32, u32>,
    ksz: Ksz8463,
    rx: Vec<u8>,
    idx: u32, // byte index within the current SPI transaction
    cmd: u16, // accumulated command word (address + write bit)
    is_write: bool,
    val: u16, // register value being read out (response bytes)
    dlo: u8,  // low data byte captured during a write
}
impl Spi4 {
    pub fn new() -> Self {
        Spi4 {
            regs: std::collections::HashMap::new(),
            ksz: Ksz8463::default(),
            rx: Vec::new(),
            idx: 0,
            cmd: 0,
            is_write: false,
            val: 0,
            dlo: 0,
        }
    }
    /// Clock one byte out (and one in) of the KSZ, by position in the 4-byte xfer.
    fn xfer_byte(&mut self, b: u8) -> u8 {
        let pos = self.idx % 4;
        self.idx += 1;
        match pos {
            0 => {
                self.cmd = (b as u16) << 8;
                0
            }
            1 => {
                self.cmd |= b as u16;
                self.is_write = self.cmd & 0x8000 != 0;
                self.val = self.ksz.read(self.cmd & 0x7FFF);
                0
            }
            2 => {
                if self.is_write {
                    self.dlo = b;
                    0
                } else {
                    (self.val & 0xFF) as u8
                }
            }
            3 => {
                if self.is_write {
                    self.ksz
                        .write(self.cmd & 0x7FFF, ((b as u16) << 8) | self.dlo as u16);
                    0
                } else {
                    (self.val >> 8) as u8
                }
            }
            _ => 0,
        }
    }
}
/// STM32H7 SPI `SR` for the simple transfer model the SPI device blocks share:
/// TXP always set (tx space available), RXPLVL when rx has data, and EOT+TXC
/// once `done_count` clocked bytes reach the CR2 `tsize` (`done_count` is each
/// block's `idx`, except Spi5's `xfer_cnt`).
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
    fn name(&self) -> &str {
        "SPI4"
    }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x14 => {
                // SR
                let tsize = self.regs.get(&0x04).copied().unwrap_or(0) & 0xFFFF;
                spi_sr(self.idx, tsize, !self.rx.is_empty())
            }
            0x30 => {
                if self.rx.is_empty() {
                    0
                } else {
                    self.rx.remove(0) as u32
                }
            } // RXDR: pop
            0x20 => 0, // TXDR read (only happens via byte-write RMW); harmless
            o => *self.regs.get(&o).unwrap_or(&0),
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        match off & !3 {
            0x20 => {
                let rx = self.xfer_byte(val as u8);
                self.rx.push(rx);
            } // TXDR
            0x00 => {
                // CR1: SPE 0->1 starts a fresh transaction
                let was_spe = self.regs.get(&0).map(|v| v & 1 != 0).unwrap_or(false);
                self.regs.insert(0, val);
                if val & 1 != 0 && !was_spe {
                    self.idx = 0;
                    self.rx.clear();
                }
            }
            o => {
                self.regs.insert(o, val);
            }
        }
    }
}

/// SPI2 (0x4000_3800, sidecar): `monorail` talks to the VSC7448 management
/// switch over this bus (`monorail` uses the embedded SPI core, `use-spi-core`;
/// on the sidecar SPI2 is the VSC7448, whereas on gimlet SPI2 is the iCE40
/// sequencer; see the board-conditional install). The VSC7448 SPI protocol
/// (drv/vsc7448/src/spi.rs, datasheet §5.5.2): each transaction carries a 24-bit
/// word address = `(reg_addr & 0x00FFFFFF) >> 2`, big-endian, byte0 MSB = write:
///   READ:  [a23..16 (MSB=0), a15..8, a7..0] + 1 pad byte + 4 data bytes (BE)
///   WRITE: [a23..16 (MSB=1), a15..8, a7..0] + 4 data bytes (BE)
/// Master is modeled synchronously (each TXDR byte immediately yields its RXDR
/// byte and SR reports TXP+RXP) so the spi-core transfer loop never sleeps
/// on spi-irq; same model as the gimlet `Spi4`/KSZ8463.
///
/// The register plane is store/return with the values and behaviors the stock
/// monorail init checks on top: CHIP_ID (0x374680E9, else BadChipId), PLL5G
/// gain_stat, self-clearing RAM_INIT strobes and serdes1g/6g MCB one-shots,
/// 10G serdes RCPLL lock/FSM status and APC offset-cal done, and the
/// DEVCPU_GCB MIIM(0) controller bridging to a VSC8504 (Tesla) quad-PHY
/// register model on MIIM addresses 4..=7 (identity, revision, port index,
/// instant micro commands, and a patch CRC answer that skips the 8051
/// firmware download).
pub struct Vsc7448 {
    regs: std::collections::HashMap<u32, u32>,
    vsc: std::collections::HashMap<u32, u32>, // VSC7448 reg file, keyed by 24-bit word addr
    rx: Vec<u8>,
    idx: u32, // byte index within the current SPI transaction
    is_write: bool,
    waddr: u32,   // accumulated 24-bit word address
    rval: u32,    // register value being read out
    wval: u32,    // register value being assembled during a write
    vscdbg: bool, // SP_EMU_VSCDBG: trace every VSC7448 register read/write
    // MIIM bridge to the on-board VSC8504 quad PHY (monorail drives it through
    // DEVCPU_GCB MIIM(0): MII_CMD word 0x4034, MII_DATA 0x4035, MII_STATUS
    // 0x4032). Register file keyed by (miim address, page, register); page
    // per MIIM address via register 31.
    phy_regs: std::collections::HashMap<(u8, u16, u8), u16>,
    phy_page: std::collections::HashMap<u8, u16>,
    mii_data: u32, // last MIIM read result (SUCCESS bits [17:16] = 0 = ok)
}
impl Vsc7448 {
    pub fn new() -> Self {
        Vsc7448 {
            regs: std::collections::HashMap::new(),
            vsc: std::collections::HashMap::new(),
            rx: Vec::new(),
            idx: 0,
            is_write: false,
            waddr: 0,
            rval: 0,
            wval: 0,
            vscdbg: crate::config::get().vscdbg(),
            phy_regs: std::collections::HashMap::new(),
            phy_page: std::collections::HashMap::new(),
            mii_data: 0,
        }
    }
    fn vsc_read(&self, waddr: u32) -> u32 {
        match waddr {
            // DEVCPU_GCB:CHIP_REGS:CHIP_ID (reg 0x71010000): rev_id=3, part_id=0x7468,
            // mfg_id=0x74, one=1; see drv/vsc7448/src/lib.rs:397.
            0x4000 => 0x374680E9,
            // HSIO:PLL5G_STATUS(0):PLL5G_STATUS1 (reg 0x7146003c): pll5g_setup polls
            // gain_stat (bits[18:14]) and only accepts 2 < v < 0xa, else retries 10x
            // then errors LcPllInitFailed -> monorail BspInitFailed panic. Report a
            // locked PLL (gain_stat=5). See drv/vsc7448/src/lib.rs pll5g_setup.
            0x11800f => 5 << 14,
            // MIIM(0) MII_STATUS (reg 0x710100c8): never pending or busy.
            0x4032 => 0,
            // MIIM(0) MII_DATA (reg 0x710100d4): last PHY read, SUCCESS = ok.
            0x4035 => self.mii_data,
            _ => {
                let reg = 0x7100_0000 | (waddr << 2);
                let stored = *self.vsc.get(&waddr).unwrap_or(&0);
                // XGANA (0x71480000, 4 instances of 0x10000): the 10G serdes
                // RCPLL status. STAT0 (TX +0x18c, RX +0xcc) reads
                // pllf_lock_stat (bit 31) set; STAT1 (TX +0x190, RX +0xd0)
                // reads pllf_fsm_stat (bits [3:0]) = 13. serdes10g apply
                // checks both, else Tx/RxPllLockFailed / Tx/RxPllFsmFailed
                // and monorail panics BspInitFailed.
                if (0x7148_0000..0x714C_0000).contains(&reg) {
                    match reg & 0xFFFF {
                        0x18c | 0xcc => stored | 0x8000_0000,
                        0x190 | 0xd0 => stored | 0xd,
                        _ => stored,
                    }
                // XGDIG (0x714C0000, 4 instances of 0x10000):
                // APC_IS_CAL_CFG1 (+0x20) reads offscal_done (bit 1) set;
                // serdes10g apply polls it once after starting the offset
                // calibration, else OffsetCalFailed.
                } else if (0x714C_0000..0x7150_0000).contains(&reg) && reg & 0xFFFF == 0x20 {
                    stored | 0x2
                } else {
                    stored
                }
            }
        }
    }
    /// MIIM(0) MII_CMD (reg 0x710100d0) written: run the PHY access. Fields:
    /// VLD bit 31, PHYAD [29:25], REGAD [24:20], WRDATA [19:4], OPR [2:1]
    /// (01 write, 10 read).
    fn mii_cmd(&mut self, v: u32) {
        if v >> 31 == 0 {
            return;
        }
        let phy = ((v >> 25) & 0x1f) as u8;
        let reg = ((v >> 20) & 0x1f) as u8;
        match (v >> 1) & 0b11 {
            0b01 => self.phy_write(phy, reg, ((v >> 4) & 0xffff) as u16),
            0b10 => self.mii_data = self.phy_read(phy, reg) as u32,
            _ => {}
        }
        if self.vscdbg {
            eprintln!(
                "[vsc] MIIM phy={} reg={} opr={} data={:#06x}",
                phy,
                reg,
                (v >> 1) & 0b11,
                self.mii_data
            );
        }
    }
    /// VSC8504 (Tesla) quad PHY on MIIM addresses 4..=7, port index = addr-4.
    /// Store/return per (page, register), with the identity and status values
    /// the vsc85xx driver checks on top. Register 31 selects the page.
    fn phy_read(&self, phy: u8, reg: u8) -> u16 {
        if reg == 31 {
            return *self.phy_page.get(&phy).unwrap_or(&0);
        }
        let page = *self.phy_page.get(&phy).unwrap_or(&0);
        let stored = *self.phy_regs.get(&(phy, page, reg)).unwrap_or(&0);
        match (page, reg) {
            (0, 2) => 0x0007, // IDENTIFIER_1: VSC8504_ID = 0x704c2
            (0, 3) => 0x04c2, // IDENTIFIER_2
            // EXTENDED_PHY_CONTROL_4: bits [15:11] = the PHY's own port index
            // (get_port / the tesla patch base-port check).
            (1, 23) => (stored & 0x07ff) | ((phy.wrapping_sub(4) as u16 & 0x1f) << 11),
            // GPIO EXTENDED_REVISION: tesla_e (bit 0) set; the driver refuses
            // non-rev-E parts (BadPhyRev).
            (16, 30) => 0x0001,
            _ => stored,
        }
    }
    fn phy_write(&mut self, phy: u8, reg: u8, v: u16) {
        if reg == 31 {
            self.phy_page.insert(phy, v);
            return;
        }
        let page = *self.phy_page.get(&phy).unwrap_or(&0);
        let v = match (page, reg) {
            // MODE_CONTROL: sw_reset (bit 15) self-clears (software_reset
            // polls for it).
            (0, 0) => v & !0x8000,
            (16, 18) => {
                // GPIO MICRO_PAGE: an 8051 micro command; completes at once.
                // The driver's cmd() polls for bit 15 (busy) clear and fails
                // on bit 14 (error), so both read back clear. Command 0x8008
                // computes the firmware CRC into VERIPHY_CTRL_REG2; answer
                // the Tesla patch's expected CRC so the (very long) 8051
                // patch download is skipped as on an already-patched part.
                if v == 0x8008 {
                    self.phy_regs.insert((phy, 1, 25), 0x29E8);
                }
                v & !0xC000
            }
            _ => v,
        };
        self.phy_regs.insert((phy, page, reg), v);
    }
    /// Clock one byte out (and one in) of the VSC7448, by position in the xfer.
    fn xfer_byte(&mut self, b: u8) -> u8 {
        let pos = self.idx;
        self.idx += 1;
        match pos {
            0 => {
                self.is_write = b & 0x80 != 0;
                self.waddr = ((b & 0x7f) as u32) << 16;
                0
            }
            1 => {
                self.waddr |= (b as u32) << 8;
                0
            }
            2 => {
                self.waddr |= b as u32;
                if !self.is_write {
                    self.rval = self.vsc_read(self.waddr);
                    if self.vscdbg {
                        eprintln!(
                            "[vsc] R reg={:#010x} val={:#010x}",
                            0x7100_0000 | (self.waddr << 2),
                            self.rval
                        );
                    }
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
                        // QSYS/REW/VOP/ANA_AC/ASM/DSM; the driver writes 1 then polls
                        // for it to read back 0 (~40us), else BspInitFailed(RamInitFailed).
                        // Store it pre-cleared (preserving ram_ena, bit 0). Full reg addr
                        // = 0x7100_0000 | (word_addr << 2).
                        const RAM_INIT_REGS: [u32; 6] = [
                            0x717e_03ec,
                            0x71b5_3528,
                            0x71c4_3638, // QSYS, REW, VOP
                            0x71f9_4358,
                            0x7141_39b8,
                            0x7145_0008, // ANA_AC, ASM, DSM
                        ];
                        if RAM_INIT_REGS.contains(&(0x7100_0000 | (a << 2))) {
                            v &= !0x2;
                        }
                        // HSIO MCB_SERDES1G_ADDR_CFG (0x714600e8) and
                        // MCB_SERDES6G_ADDR_CFG (0x71460168): the wr/rd
                        // one-shot strobes (bits 31/30) self-clear when the
                        // MCB transfer completes; serdes1g/6g_read/write poll
                        // them 32 times then error (Serdes*Timeout, a monorail
                        // BspInitFailed panic).
                        if a == 0x11803a || a == 0x11805a {
                            v &= !0xC000_0000;
                        }
                        if self.vscdbg {
                            eprintln!(
                                "[vsc] W reg={:#010x} val={:#010x}",
                                0x7100_0000 | (a << 2),
                                v
                            );
                        }
                        if a == 0x4034 {
                            self.mii_cmd(v); // MIIM(0) MII_CMD: PHY access
                        }
                        self.vsc.insert(a, v);
                    }
                    0
                } else {
                    // read: 1 pad byte at position 3, then 4 data bytes (BE) at 4..8
                    if pos == 3 {
                        0
                    } else {
                        ((self.rval >> ((7 - pos) * 8)) & 0xFF) as u8
                    }
                }
            }
        }
    }
}
impl Mmio for Vsc7448 {
    fn name(&self) -> &str {
        "VSC7448(SPI2)"
    }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x14 => {
                // SR
                let tsize = self.regs.get(&0x04).copied().unwrap_or(0) & 0xFFFF;
                spi_sr(self.idx, tsize, !self.rx.is_empty())
            }
            0x30 => {
                if self.rx.is_empty() {
                    0
                } else {
                    self.rx.remove(0) as u32
                }
            } // RXDR: pop
            0x20 => 0, // TXDR read (only via byte-write RMW); harmless
            o => *self.regs.get(&o).unwrap_or(&0),
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        match off & !3 {
            0x20 => {
                let rx = self.xfer_byte(val as u8);
                self.rx.push(rx);
            } // TXDR
            0x00 => {
                // CR1: SPE 0->1 starts a fresh transaction
                let was_spe = self.regs.get(&0).map(|v| v & 1 != 0).unwrap_or(false);
                self.regs.insert(0, val);
                if val & 1 != 0 && !was_spe {
                    self.idx = 0;
                    self.rx.clear();
                    self.wval = 0;
                    self.is_write = false;
                }
            }
            o => {
                self.regs.insert(o, val);
            }
        }
    }
}

/// SPI2 (0x4000_3800): gimlet's bus shared (by chip-select) between the iCE40
/// sequencer FPGA (CS PB5) and the KSZ8463 switch (CS PI0). The active device is
/// read from the shared `Spi2Cs` cell (set by the GPIO bank). The target is
/// latched at SPE 0->1 so a whole exchange goes to one device.
///
/// Sequencer FPGA protocol: a 3-byte header [cmd, addr_be_hi, addr_be_lo] then
/// data; READ (cmd=1) returns reg[addr++] for bytes after the header. Registers
/// modeled: ID0/1 = 0x01/0xDE (ident 0x1DE), CS0..3 = 0x74753981 LE (matches
/// GIMLET_BITSTREAM_CHECKSUM, so the SP skips reprogramming), PWR_CTRL(0x13)=0 (A2).
pub struct Spi2 {
    regs: std::collections::HashMap<u32, u32>,
    cs: Spi2Cs,
    target: u8, // device latched at SPE (1=seq, 2=ksz)
    idx: u32,
    rx: Vec<u8>,
    // sequencer FPGA register file + per-transaction header accumulation
    seq: std::collections::HashMap<u16, u8>,
    seq_cmd: u8,
    seq_addr: u16,
    // KSZ8463 (same model as Spi4)
    ksz: Ksz8463,
    kcmd: u16,
    kwrite: bool,
    kval: u16,
    kdlo: u8,
    dbg_txn: u32,
}
impl Spi2 {
    pub fn new(cs: Spi2Cs) -> Self {
        Spi2 {
            regs: Default::default(),
            cs,
            target: 0,
            idx: 0,
            rx: Vec::new(),
            seq: Default::default(),
            seq_cmd: 0,
            seq_addr: 0,
            ksz: Ksz8463::default(),
            kcmd: 0,
            kwrite: false,
            kval: 0,
            kdlo: 0,
            dbg_txn: 0,
        }
    }
    fn seq_read(&self, addr: u16) -> u8 {
        match addr {
            0x0 => 0xDE,
            0x1 => 0x01, // ID0/ID1 (LE) -> ident 0x01DE = 0x1DE
            0xa => 0x81,
            0xb => 0x39,
            0xc => 0x75,
            0xd => 0x74,  // CS0..3 -> 0x74753981 (LE)
            0x13 => 0x00, // PWR_CTRL -> 0 (A2 resting)
            a => *self.seq.get(&a).unwrap_or(&0),
        }
    }
    fn xfer(&mut self, b: u8) -> u8 {
        let pos = self.idx;
        self.idx += 1;
        // Latch the CS target on the first byte: the SPI server sets SPE before
        // asserting the chip-select GPIO, so CS isn't valid until data flows.
        if pos == 0 {
            self.target = self.cs.get();
            self.dbg_txn = self.dbg_txn.wrapping_add(1);
        }
        let r = self.xfer_inner(b, pos);
        if crate::dbg::spi() && self.dbg_txn < 6 && pos <= 6 {
            eprintln!(
                "[spi2x] txn={} tgt={} cs={} pos={} in={:#04x} out={:#04x} cmd={} addr={:#x}",
                self.dbg_txn,
                self.target,
                self.cs.get(),
                pos,
                b,
                r,
                self.seq_cmd,
                self.seq_addr
            );
        }
        r
    }
    fn xfer_inner(&mut self, b: u8, pos: u32) -> u8 {
        match self.target {
            1 => {
                // sequencer FPGA: 3-byte header then data
                match pos {
                    0 => {
                        self.seq_cmd = b;
                        0
                    }
                    1 => {
                        self.seq_addr = (b as u16) << 8;
                        0
                    }
                    2 => {
                        self.seq_addr |= b as u16;
                        0
                    }
                    n => {
                        let a = self.seq_addr.wrapping_add(n as u16 - 3);
                        if self.seq_cmd == 1 {
                            // Read
                            self.seq_read(a)
                        } else {
                            // Write(0) / BitSet(2) / BitClear(3), as in the
                            // Spi5 mainboard model; other ops store as written.
                            let cur = *self.seq.get(&a).unwrap_or(&0);
                            let nv = match self.seq_cmd {
                                2 => cur | b,  // BitSet: read-modify-write OR
                                3 => cur & !b, // BitClear: RMW AND-NOT
                                _ => b,
                            };
                            self.seq.insert(a, nv);
                            0
                        }
                    }
                }
            }
            2 => {
                // KSZ8463: 4-byte [addr_hi, addr_lo, d0, d1]
                match pos % 4 {
                    0 => {
                        self.kcmd = (b as u16) << 8;
                        0
                    }
                    1 => {
                        self.kcmd |= b as u16;
                        self.kwrite = self.kcmd & 0x8000 != 0;
                        self.kval = self.ksz.read(self.kcmd & 0x7FFF);
                        0
                    }
                    2 => {
                        if self.kwrite {
                            self.kdlo = b;
                            0
                        } else {
                            (self.kval & 0xFF) as u8
                        }
                    }
                    _ => {
                        if self.kwrite {
                            self.ksz
                                .write(self.kcmd & 0x7FFF, ((b as u16) << 8) | self.kdlo as u16);
                            0
                        } else {
                            (self.kval >> 8) as u8
                        }
                    }
                }
            }
            _ => 0,
        }
    }
}
impl Mmio for Spi2 {
    fn name(&self) -> &str {
        "SPI2"
    }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x14 => {
                // SR
                let tsize = self.regs.get(&0x04).copied().unwrap_or(0) & 0xFFFF;
                spi_sr(self.idx, tsize, !self.rx.is_empty())
            }
            0x30 => {
                if self.rx.is_empty() {
                    0
                } else {
                    self.rx.remove(0) as u32
                }
            }
            o => *self.regs.get(&o).unwrap_or(&0),
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        match off & !3 {
            0x20 => {
                let rx = self.xfer(val as u8);
                self.rx.push(rx);
            }
            0x00 => {
                // CR1: SPE 0->1 latches the CS target and starts a transaction
                let was = self.regs.get(&0).map(|v| v & 1 != 0).unwrap_or(false);
                self.regs.insert(0, val);
                if val & 1 != 0 && !was {
                    self.idx = 0;
                    self.rx.clear();
                } // target latched at 1st byte
            }
            o => {
                self.regs.insert(o, val);
            }
        }
    }
}

/// SPI5 (0x4001_5000, irq85): sidecar's mainboard ECP5 FPGA via drv-fpga-server
/// (`use-spi-core`, so the task drives SPI5 directly). The ECP5 is reported
/// "configured" via GPIO (done=PJ15 high -> DeviceState::RunningUserDesign), so
/// SPI5 only carries the FPGA *user-design* register protocol: a 3-byte header
/// [op, addr_be_hi, addr_be_lo] then data. op: Read=1, Write=0, BitSet=2,
/// BitClear=3, ReadNoAddrIncr=6, WriteNoAddrIncr=5; Read auto-increments addr.
/// `read_ident()` reads 16 bytes from Addr::ID0(0x0) as FpgaUserDesignIdent
/// { id, checksum, version, sha } (BE u32 each). The sequencer requires
/// id == EXPECTED_ID 0x01de5bae and checksum == bitstream-checksum prefix
/// 0x5e470764 (else it resets the FPGA and panics). SR flags mirror Spi2/Spi4.
pub struct Spi5 {
    regs: std::collections::HashMap<u32, u32>,
    rx: Vec<u8>,
    idx: u32,
    op: u8,
    addr: u16,
    dpos: u32,     // data bytes consumed this command (after the 3-byte header)
    xfer_cnt: u32, // bytes in the current spi-core transfer (for EOT); resets per SPE
    dbg_n: u32,    // SP_EMU_SPIDBG trace counter (cap output)
    cs: Spi5Cs,    // user-design CS assert-generation; reset the command when it changes
    last_gen: u32,
    fpga: std::collections::HashMap<u16, u8>, // FPGA user-design register file (byte-addressed)
    // Tofino debug port (TOFINO_DEBUG_PORT_BUFFER 0x200 / _STATE 0x201): the
    // sequencer queues an opcode + address (+ data) into the buffer, sets
    // REQUEST_IN_PROGRESS, polls for it to clear, then reads the response out
    // of the buffer. Requests complete instantly against `tofino_regs`, a
    // sparse register file of the Tofino behind the port, so the driver's
    // read-modify-write-read-back sequences see their own writes.
    dbg_req: Vec<u8>,
    dbg_resp: std::collections::VecDeque<u8>,
    tofino_regs: std::collections::HashMap<u32, u32>,
}
/// Seed the FPGA ignition-controller register block so the emulated sidecar SP
/// answers MGS `ignition`. The sidecar is the rack's ignition hub: MGS issues
/// GET /ignition as step 1 of SP enumeration, so without a populated controller
/// no SPs (and therefore no switches or switch-ports) are discovered
/// (downstream symptom: rack-init fails on `qsfp0 not found`).
///
/// Register layout (drv-sidecar-mainboard-controller / drv-ignition-api):
///   IGNITION_CONTROLLERS_COUNT @ 0x300  u8   port count (35)
///   IGNITION_TARGETS_PRESENT0  @ 0x301  u64  presence bitmap (LE), bit per port
///   per-port PortState         @ 0x400 + 0x100*port  u64 (LE byte fields):
///     [0] CONTROLLER_STATE      TARGET_PRESENT(0x01)
///     [1] CONTROLLER_LINK_STATUS RECEIVER_ALIGNED|RECEIVER_LOCKED (0x03)
///     [2] TARGET_SYSTEM_TYPE     RFD-141 id (gimlet 0x11, sidecar 0x12, psc 0x13, cosmo 0x04)
///     [3] TARGET_SYSTEM_STATUS   CONTROLLER0_DETECTED(0x01)|SYSTEM_POWER_ENABLED(0x04) -> On
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

    let spec = crate::config::get().ignition();

    fpga.insert(CONTROLLERS_COUNT, NUM_PORTS);

    let mut present: u64 = 0;
    for entry in spec.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        let (port_s, type_s) = match entry.split_once(':') {
            Some(x) => x,
            None => {
                eprintln!(
                    "[sp-emu] SP_EMU_IGNITION: ignoring malformed entry {:?}",
                    entry
                );
                continue;
            }
        };
        let port: u8 = match port_s.trim().parse() {
            Ok(p) if p < NUM_PORTS => p,
            _ => {
                eprintln!(
                    "[sp-emu] SP_EMU_IGNITION: ignoring out-of-range port {:?}",
                    port_s
                );
                continue;
            }
        };
        let sys_type: u8 = match type_s.trim().to_ascii_lowercase().as_str() {
            "gimlet" => 0x11,
            "sidecar" => 0x12,
            "psc" => 0x13,
            "cosmo" => 0x04,
            other => {
                eprintln!(
                    "[sp-emu] SP_EMU_IGNITION: unknown type {:?}, defaulting to gimlet",
                    other
                );
                0x11
            }
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
        // IDENT @ ID0..: id=0x01de5bae (ident.id is BE -> bytes 01 de 5b ae).
        // checksum: the driver reads ident.checksum BE but compares to the LE
        // interpretation of SIDECAR_MAINBOARD_BITSTREAM_CHECKSUM[..4]=[5e,47,07,64]
        // = 0x6407475e, so CS0..3 must be 64 07 47 5e (BE -> 0x6407475e). version/sha=0.
        for (a, v) in [
            (0u16, 0x01u8),
            (1, 0xde),
            (2, 0x5b),
            (3, 0xae),
            (4, 0x64),
            (5, 0x07),
            (6, 0x47),
            (7, 0x5e),
            // FRONT_IO_STATE (0x30): STATE field is bits[7:4]; set it to
            // PowerRailStatus::Enabled(4) -> 4<<4 = 0x40, so the sequencer's
            // front-IO hot-swap preinit loop completes (status == Enabled).
            (0x30, 0x40),
            // Tofino sequencer register block (drv-sidecar-mainboard-controller
            // tofino2.rs / generated reg map). Reports a coherent A2 resting
            // state with no abort. a4x2's Tofino dataplane is SoftNPU/P4 in
            // software, so the SP only reports the switch as present/powered,
            // not sequence real silicon. TofinoSeqStatus decodes 6 bytes at
            // 0x100..0x105: CTRL, STATE=A2(1), STEP=Init(0), ERROR=None(0),
            // ERROR_STATE=Init(0), ERROR_STEP=Init(0) -> abort=None.
            (0x100, 0x00), // TOFINO_SEQ_CTRL (EN=0; at rest in A2)
            (0x101, 0x01), // TOFINO_SEQ_STATE = A2
            (0x102, 0x00), // TOFINO_SEQ_STEP = Init
            (0x103, 0x00), // TOFINO_SEQ_ERROR = None
            (0x104, 0x00), // TOFINO_SEQ_ERROR_STATE = Init
            (0x105, 0x00),
            // TOFINO_DEBUG_PORT_STATE: SEND_BUFFER_EMPTY | RECEIVE_BUFFER_EMPTY,
            // else the sequencer's read_direct/write_direct bail InvalidState.
            (0x201, 0x05),
        ]
        // TOFINO_SEQ_ERROR_STEP = Init
        {
            fpga.insert(a, v);
        }
        seed_ignition(&mut fpga);
        Spi5 {
            regs: std::collections::HashMap::new(),
            rx: Vec::new(),
            idx: 0,
            op: 0,
            addr: 0,
            dpos: 0,
            xfer_cnt: 0,
            dbg_n: 0,
            cs,
            last_gen: 0,
            fpga,
            dbg_req: Vec::new(),
            dbg_resp: std::collections::VecDeque::new(),
            tofino_regs: std::collections::HashMap::new(),
        }
    }
    /// Run a queued Tofino debug-port request (REQUEST_IN_PROGRESS written to
    /// TOFINO_DEBUG_PORT_STATE). Request layout: opcode, 4-byte LE address,
    /// then for writes a 4-byte LE value. DirectRead (0xA0) queues the 4-byte
    /// LE register value as the response; DirectWrite (0x80) stores it.
    /// Unknown opcodes complete with no side effect. State returns to
    /// buffers-empty / not-in-progress, so the driver's poll exits at once.
    fn tofino_debug_request(&mut self) {
        let req = std::mem::take(&mut self.dbg_req);
        if req.len() >= 5 {
            let addr = u32::from_le_bytes([req[1], req[2], req[3], req[4]]);
            match req[0] {
                0xA0 => {
                    // DirectRead
                    let v = *self.tofino_regs.get(&addr).unwrap_or(&0);
                    self.dbg_resp.extend(v.to_le_bytes());
                }
                0x80 if req.len() >= 9 => {
                    // DirectWrite
                    let v = u32::from_le_bytes([req[5], req[6], req[7], req[8]]);
                    self.tofino_regs.insert(addr, v);
                }
                _ => {}
            }
        }
        self.fpga.insert(0x201, 0x05); // buffers empty, request complete
    }
    /// Reset per-command state on the CS deasserted->asserted edge: a command
    /// (header write + data read) spans two SPE cycles under one CS lock.
    fn check_cs(&mut self) {
        let gen = self.cs.get();
        if gen != self.last_gen {
            // a new CS lock began -> new FPGA command
            self.idx = 0;
            self.dpos = 0;
            self.rx.clear();
            self.op = 0;
            self.addr = 0;
            self.xfer_cnt = 0;
            self.last_gen = gen;
        }
    }
    /// Tofino sequencing FSM, run after any write to TOFINO_SEQ_CTRL (0x100:
    /// CLEAR_ERROR bit 0, EN bit 1, ACK_VID bit 2). EN set: the power-up
    /// completes instantly; STATE (0x101) = A0(2), the VID (0x10c) reads
    /// valid, and the six power rails (0x106..0x10b) read ENABLE|GOOD. EN
    /// clear: back to A2(1), rails off, VID invalid. The command bits
    /// CLEAR_ERROR and ACK_VID self-clear as on the real controller; ERROR
    /// (0x103) never sets. The sequencer's power_up polls STATE for A0 after
    /// its VID handshake (SequencerTimeoutNotInA0 otherwise), and its timer
    /// tick powers up whenever policy is LatchOffOnFault and STATE reads A2.
    fn tofino_seq_ctrl_written(&mut self) {
        let ctrl = *self.fpga.get(&0x100).unwrap_or(&0);
        let en = ctrl & 0x02 != 0;
        // Command bits self-clear.
        self.fpga.insert(0x100, ctrl & 0x02);
        let (state, vid, rail) = if en {
            (2u8, 0x80 | 0b1000, 0x03u8) // A0, VID_VALID | V0P759, ENABLE|GOOD
        } else {
            (1, 0, 0) // A2
        };
        self.fpga.insert(0x101, state);
        self.fpga.insert(0x10c, vid);
        for a in 0x106..=0x10bu16 {
            self.fpga.insert(a, rail);
        }
    }
    /// The next data byte for the current (read) command, with address auto-increment.
    fn next_data(&mut self) -> u8 {
        let incr = self.op != 5 && self.op != 6; // No-AddrIncr variants hold addr
        let a = self
            .addr
            .wrapping_add(if incr { self.dpos } else { 0 } as u16);
        self.dpos += 1;
        // Debug-port buffer reads pop response bytes.
        if a == 0x200 {
            return self.dbg_resp.pop_front().unwrap_or(0);
        }
        *self.fpga.get(&a).unwrap_or(&0)
    }
    /// One TXDR byte (full-duplex). Header is the first 3 bytes [op, addr_be];
    /// after that, data: reads emit register bytes, writes store them.
    fn xfer(&mut self, b: u8) -> u8 {
        self.xfer_cnt += 1; // bytes in this spi-core transfer (drives EOT)
        let out = match self.idx {
            0 => {
                self.op = b;
                self.idx += 1;
                0
            }
            1 => {
                self.addr = (b as u16) << 8;
                self.idx += 1;
                0
            }
            2 => {
                self.addr |= b as u16;
                self.idx += 1;
                0
            }
            _ => {
                if self.op == 1 || self.op == 6 {
                    self.next_data()
                }
                // Read / ReadNoAddrIncr
                else {
                    // Write(0) / BitSet(2) / BitClear(3) / WriteNoAddrIncr(5)
                    let incr = self.op != 5; // only WriteNoAddrIncr holds the address
                    let a = self
                        .addr
                        .wrapping_add(if incr { self.dpos } else { 0 } as u16);
                    self.dpos += 1;
                    // Debug-port buffer: writes queue request bytes, they do
                    // not land in the register file.
                    if a == 0x200 {
                        self.dbg_req.push(b);
                    } else {
                        let cur = *self.fpga.get(&a).unwrap_or(&0);
                        let nv = match self.op {
                            2 => cur | b,  // BitSet: read-modify-write OR
                            3 => cur & !b, // BitClear: read-modify-write AND-NOT
                            _ => b,        // Write / WriteNoAddrIncr: overwrite
                        };
                        self.fpga.insert(a, nv);
                        if a == 0x100 {
                            self.tofino_seq_ctrl_written();
                        }
                        // Debug-port state: REQUEST_IN_PROGRESS runs the queued
                        // request; other writes (e.g. reset to buffers-empty)
                        // are plain stores.
                        if a == 0x201 && nv & 0x10 != 0 {
                            self.tofino_debug_request();
                        }
                    }
                    0
                }
            }
        };
        if crate::dbg::spi() && self.dbg_n < 120 {
            self.dbg_n += 1;
            eprintln!(
                "[spi5] idx={} op={} addr={:#x} dpos={} in={:#04x} out={:#04x}",
                self.idx, self.op, self.addr, self.dpos, b, out
            );
        }
        out
    }
}
impl Mmio for Spi5 {
    fn name(&self) -> &str {
        "SPI5"
    }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x14 => {
                // SR (EOT/TXC keyed on xfer_cnt, the full-duplex byte count, not idx)
                let tsize = self.regs.get(&0x04).copied().unwrap_or(0) & 0xFFFF;
                spi_sr(self.xfer_cnt, tsize, !self.rx.is_empty())
            }
            0x30 => {
                // RXDR
                if let Some(b) = (!self.rx.is_empty()).then(|| self.rx.remove(0)) {
                    b as u32 // full-duplex: byte produced by a TXDR write
                } else if self.idx >= 3 && (self.op == 1 || self.op == 6) {
                    // Receive-only read: bump xfer_cnt (xfer() does it for
                    // full-duplex) so EOT still fires in SR.
                    self.xfer_cnt += 1;
                    self.next_data() as u32
                } else {
                    0
                }
            }
            o => *self.regs.get(&o).unwrap_or(&0),
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        self.check_cs(); // resets per-command state on the CS asserting edge
        match off & !3 {
            0x20 => {
                let rx = self.xfer(val as u8);
                self.rx.push(rx);
            } // TXDR
            0x04 => {
                self.xfer_cnt = 0;
                self.regs.insert(0x04, val); // CR2.TSIZE: new transfer -> reset EOT count
                if crate::dbg::spi() && self.dbg_n < 120 {
                    eprintln!("[spi5] CR2/TSIZE <- {:#x} (xfer_cnt reset)", val);
                }
            }
            o => {
                if crate::dbg::spi() && self.dbg_n < 120 && (o == 0x00) {
                    eprintln!("[spi5] CR1 <- {:#x}", val);
                }
                self.regs.insert(o, val);
            }
        }
    }
}

/// GPIO bank: store/return, but the read-only input register IDR (+0x10 within
/// each 0x400 port) is synthesized for the boot-critical externally-driven pins:
///  - GPIOC (port 2): PC6/PC7 = sequencer V3P3/V1P2 power-good -> bits 6,7 high.
///  - GPIOG (port 6): PG[2:0] = board revision -> 0b010 for gimlet-c.
///
/// Other ports' IDR mirrors their ODR (+0x14) so output read-back works.
pub struct GpioBank {
    regs: std::collections::HashMap<u32, u32>,
    cs: Spi2Cs,
    spi5_cs: Spi5Cs,
    prev_pj6_low: Cell<bool>,
    sidecar: bool,
}
impl GpioBank {
    pub fn new(cs: Spi2Cs, spi5_cs: Spi5Cs) -> Self {
        // $SP_EMU_BOARD selects the board profile for synthesized input pins.
        let sidecar = crate::config::get().board().is_sidecar();
        GpioBank {
            regs: std::collections::HashMap::new(),
            cs,
            spi5_cs,
            prev_pj6_low: Cell::new(false),
            sidecar,
        }
    }
}
impl GpioBank {
    /// Recompute the shared SPI2 chip-select from the port-B/port-I ODR state.
    /// CS is active-low: pin driven low selects the device. PB5 -> sequencer,
    /// PI0 -> KSZ8463 (per app/gimlet/base.toml config.spi.spi2.devices).
    fn update_cs(&self) {
        let pb = *self.regs.get(&(0x400 + 0x14)).unwrap_or(&0); // GPIOB ODR
        let pi = *self.regs.get(&(8 * 0x400 + 0x14)).unwrap_or(&0); // GPIOI ODR
        self.cs.set(if pb & (1 << 5) == 0 {
            1
        } else if pi & (1 << 0) == 0 {
            2
        } else {
            0
        });
        // Sidecar SPI5 user-design CS = Port J (port 9) pin 6, active-low. Count
        // each deasserted->asserted edge so Spi5 can delimit FPGA commands even
        // when the deassert happens between (Spi5-invisible) GPIO writes.
        let pj = *self.regs.get(&(9 * 0x400 + 0x14)).unwrap_or(&0); // GPIOJ ODR
        let pj6_low = pj & (1 << 6) == 0;
        if pj6_low && !self.prev_pj6_low.get() {
            self.spi5_cs.set(self.spi5_cs.get().wrapping_add(1));
        }
        self.prev_pj6_low.set(pj6_low);
    }
}
impl Mmio for GpioBank {
    fn name(&self) -> &str {
        "GPIO"
    }
    fn read(&mut self, off: u32) -> u32 {
        let (port, reg) = (off / 0x400, off & 0x3FF & !3);
        if reg == 0x10 {
            // IDR
            if port == 4 {
                // GPIOE: PE3 is rot-irq (input from the RoT, active-low). With a RoT
                // attached, reflect the sprot link's rot_irq on bit 3. With no RoT,
                // the line sits deasserted (high, pulled up) as on a board with no
                // RoT populated; otherwise the SP's sprot driver sees ROT_IRQ stuck
                // asserted and fails every RoT request with RotIrqRemainsAsserted
                // (faux-mgs tolerates that field error; sp-test treats it as fatal).
                let odr = *self.regs.get(&(4 * 0x400 + 0x14)).unwrap_or(&0);
                let asserted = crate::sprot::link()
                    .map(|lk| lk.borrow().rot_irq)
                    .unwrap_or(false);
                let bit3 = if asserted { 0 } else { 1 << 3 };
                return (odr & !(1 << 3)) | bit3;
            }
            if self.sidecar {
                return match port {
                    // GPIOC PC6/PC7/PC13 -> board rev[0,1,2]; sidecar-c = 0b010 -> PC7 only.
                    2 => 1 << 7,
                    // GPIOF PF12 = front-IO POWER_GOOD (input) -> high so the sequencer's
                    // front-IO preinit passes the PG check.
                    5 => self.regs.get(&(5 * 0x400 + 0x14)).copied().unwrap_or(0) | (1 << 12),
                    // GPIOJ: mainboard ECP5 config pins; done=PJ15 high (=configured ->
                    // device_state RunningUserDesign, skip bitstream) + program_n=PJ13
                    // high (not in reset). init_n=PJ12 (don't-care once done is high).
                    9 => (1 << 15) | (1 << 13),
                    _ => *self.regs.get(&(port * 0x400 + 0x14)).unwrap_or(&0),
                };
            }
            return match port {
                2 => 0b11 << 6,                                            // GPIOC: PG lines good
                6 => 0b010, // GPIOG: gimlet-c board rev
                _ => *self.regs.get(&(port * 0x400 + 0x14)).unwrap_or(&0), // mirror ODR
            };
        }
        *self.regs.get(&(off & !3)).unwrap_or(&0)
    }
    fn write(&mut self, off: u32, val: u32) {
        let (port, reg) = (off / 0x400, off & 0x3FF & !3);
        // BSRR (+0x18): set bits [15:0], reset bits [31:16] -> fold into ODR.
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
            // and latch the SSA/SSD slave-select events on each edge. Latched here,
            // on the SP side, because this write always runs inside the SP's quantum
            // and so never misses a CS edge, unlike the RoT, which only samples the
            // line when it touches its FLEXCOMM8 registers and can sleep through an
            // entire assert->clock->deassert cycle. See SprotLink.
            if let Some(lk) = crate::sprot::link() {
                let odr = *self.regs.get(&(4 * 0x400 + 0x14)).unwrap_or(&0);
                let new_cs = (odr >> 4) & 1 == 0;
                let mut l = lk.borrow_mut();
                let edge = new_cs != l.cs;
                if edge {
                    if new_cs {
                        // Transaction pacing (live RoT thread): a new transfer
                        // must not start while the RoT still owes the previous
                        // one an SSA/SSD ack, or its cleanup would race this
                        // transfer's bytes (bounded; timeout proceeds).
                        if l.rot_live {
                            let mut budget = crate::sprot::STALL_BUDGET_ITERS;
                            while (l.ssa || l.ssd) && budget > 0 {
                                l = lk.wait_sp(l, crate::sprot::WAIT_STEP_MS);
                                budget -= 1;
                            }
                            // Unread request bytes belong to a dead transaction
                            // (the firmware's start-of-transfer cleanup would
                            // discard them); drop them so this transfer's first
                            // frame carries the SOT.
                            l.mosi.clear();
                        }
                        // CS asserted: start of a transfer. Latch SSA + the SOT bit
                        // for the first FIFORD frame the RoT reads.
                        l.ssa = true;
                        l.sot_pending = true;
                    } else {
                        // CS de-asserted: end of a transfer. Latch SSD.
                        l.ssd = true;
                    }
                    if crate::sprot::dbg() {
                        eprintln!(
                            "[gpio] PE4 CS {} (mosi={} miso={})",
                            if new_cs { "ASSERT" } else { "deassert" },
                            l.mosi.len(),
                            l.miso.len()
                        );
                    }
                }
                l.cs = new_cs;
                drop(l);
                // A CS edge is the start/end of a transfer: wake a parked RoT.
                if edge {
                    lk.wake_rot();
                }
            }
        }
        // GPIOB/GPIOI affect SPI2 CS; GPIOJ (port 9) affects the sidecar SPI5 CS.
        if port == 1 || port == 8 || port == 9 {
            self.update_cs();
        }
    }
}

/// Scriptable physical environment for the modeled sensors. The sensor chips are
/// emulated with their real register protocol; the physical quantity they'd
/// measure (temperature, …) has no source in a virtual rack, so it's injected
/// here. Configurable so it can drive fault scenarios. Configure via env:
///   SP_EMU_AMBIENT_C=<°C>             default temperature for every sensor
///   SP_EMU_SENSORS=0x48=45.0,0x18=60  per-address °C overrides
pub struct SensorEnv {
    default_temp_c: f32,
    temp_override: std::collections::HashMap<u8, f32>,
}
pub type Sensors = Rc<RefCell<SensorEnv>>;
impl SensorEnv {
    pub fn from_env() -> Sensors {
        let default_temp_c = crate::config::get().ambient_c();
        let mut temp_override = std::collections::HashMap::new();
        if let Some(s) = crate::config::get().sensors() {
            for kv in s.split(',') {
                if let Some((a, v)) = kv.split_once('=') {
                    let a = a.trim().trim_start_matches("0x");
                    if let (Ok(addr), Ok(t)) = (u8::from_str_radix(a, 16), v.trim().parse::<f32>())
                    {
                        temp_override.insert(addr, t);
                    }
                }
            }
        }
        Rc::new(RefCell::new(SensorEnv {
            default_temp_c,
            temp_override,
        }))
    }
    fn temp_c(&self, addr: u8) -> f32 {
        *self
            .temp_override
            .get(&addr)
            .unwrap_or(&self.default_temp_c)
    }
}

/// CRC-32/ISCSI (Castagnoli): the body checksum the `tlvc` crate uses
/// (`crc::CRC_32_ISCSI`). Reflected poly 0x82F6_3B78, init/xorout 0xFFFF_FFFF.
fn crc32c(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0x82F6_3B78
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// TLV-C header checksum; mirrors `tlvc::header_checksum` exactly:
/// `!(le_u32(tag).wrapping_mul(HEADER_MAGIC).wrapping_add(len))`.
fn tlvc_header_checksum(tag: [u8; 4], len: u32) -> u32 {
    const HEADER_MAGIC: u32 = 0x6b32_9f69;
    !u32::from_le_bytes(tag)
        .wrapping_mul(HEADER_MAGIC)
        .wrapping_add(len)
}

/// Serialize one TLV-C chunk: header { tag, len(LE), header_checksum(LE) },
/// then the body, zero-padded to a 4-byte boundary, then the body CRC (LE).
/// Shared with lpc55.rs, which synthesizes a stage0 caboose in the same format.
pub(crate) fn tlvc_chunk(tag: &[u8; 4], body: &[u8]) -> Vec<u8> {
    let len = body.len() as u32;
    let mut v = Vec::new();
    v.extend_from_slice(tag);
    v.extend_from_slice(&len.to_le_bytes());
    v.extend_from_slice(&tlvc_header_checksum(*tag, len).to_le_bytes());
    v.extend_from_slice(body);
    while v.len() % 4 != 0 {
        v.push(0);
    } // header is 12B, so this pads the body
    v.extend_from_slice(&crc32c(body).to_le_bytes());
    v
}

/// STM32H7 HASH (0x4802_1400, irq 80). Minimal model so drv-stm32h7-hash
/// completes: report DINIS (ready for data) + not BUSY, and when the driver
/// writes STR.DCAL (start digest) set SR.DCIS and raise irq 80 so its
/// `sys_recv_notification` wakes. Returns a fixed digest: MGS only records the
/// phase1 hash for inventory; it is not checked against the flash here.
struct Hash {
    irq_pending: bool,
    dcis: bool,
}
impl Hash {
    pub fn new() -> Self {
        Hash {
            irq_pending: false,
            dcis: false,
        }
    }
}
impl Mmio for Hash {
    fn name(&self) -> &str {
        "HASH"
    }
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
            0x00 => {
                if val & (1 << 2) != 0 {
                    self.dcis = false;
                }
            } // CR.INIT
            0x08 => {
                if val & (1 << 8) != 0 {
                    self.dcis = true;
                    self.irq_pending = true;
                }
            } // STR.DCAL
            _ => {}
        }
    }
    fn take_irq(&mut self) -> Option<u16> {
        if self.irq_pending {
            self.irq_pending = false;
            Some(80)
        } else {
            None
        }
    }
}

/// STM32H7 RNG (0x4802_1800). Minimal model: report data always ready (SR.DRDY)
/// with no error bits, and return a non-zero pseudo-random word from DR. The
/// PRNG is deterministic (fixed seed) so a given boot yields a reproducible
/// ereport restart id; the driver only requires
/// DRDY set, CEIS/SEIS clear, and DR != 0 (drv-stm32h7-rng `read`).
struct Rng {
    state: u64,
    cr: u32,
}
impl Rng {
    pub fn new() -> Self {
        // Fixed non-zero seed -> stable restart id per boot.
        Rng {
            state: 0x9E37_79B9_7F4A_7C15,
            cr: 0,
        }
    }
    fn next_word(&mut self) -> u32 {
        // xorshift64*: a non-zero state stays non-zero, so DR is never 0.
        let mut x = self.state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.state = x;
        let v = (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 32) as u32;
        if v == 0 {
            1
        } else {
            v
        }
    }
}
impl Mmio for Rng {
    fn name(&self) -> &str {
        "RNG"
    }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x00 => self.cr,      // CR: read back the last write
            0x04 => 1,            // SR: DRDY set (bit 0), CEIS/SEIS clear
            0x08 => self.next_word(), // DR: non-zero random word
            _ => 0,
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        if off & !3 == 0x00 {
            self.cr = val; // CR (RNGEN/IE); SR error-clear writes are no-ops
        }
    }
}

/// Force a VPD/FRUID barcode field to the printable 7-bit ASCII the 0XV2 format
/// stores, and bound its length. Drop any byte outside `0x20..=0x7E` and the
/// `:` field delimiter, then truncate to `max` bytes (one byte per character
/// now that the field is ASCII). Warn on either change: a non-ASCII byte, a
/// stray `:`, or a silent truncation yields a barcode that parses wrong rather
/// than one that obviously fails.
fn vpd_ascii_field(what: &str, value: &str, max: usize) -> String {
    let mut clean: String = value
        .chars()
        .filter(|&c| (' '..='~').contains(&c) && c != ':')
        .collect();
    if clean.chars().count() != value.chars().count() {
        eprintln!(
            "[vpd] {what} {value:?} has characters outside printable 7-bit ASCII \
             (or a ':'); using {clean:?}"
        );
    }
    if clean.len() > max {
        eprintln!("[vpd] {what} {clean:?} exceeds {max} bytes; truncating");
        clean.truncate(max); // clean is ASCII, so max is a char boundary
    }
    clean
}

/// Build the 1024-byte AT24CSW080 VPD image the sidecar firmware expects:
/// a `FRU0` root whose body holds a `MAC0` chunk (task_packrat_api::MacAddressBlock
/// = base_mac[6] + count(u16 LE) + stride(u8)) and a `BARC` chunk (an 0XV2 Oxide
/// barcode string). This lets drv_packrat_vpd_loader::read_vpd_and_load_packrat
/// succeed on the first attempt instead of mem-faulting on garbage. Non-sidecar
/// boards get a blank (all-0xFF) EEPROM, preserving gimlet behavior: its sharkfin
/// VPD reads fail cleanly as "Truncated", which the firmware tolerates.
fn build_vpd_eeprom() -> Rc<Vec<u8>> {
    let mut img = vec![0xFFu8; 1024];
    let sidecar = crate::config::get().board().is_sidecar();
    // Per-instance index from the bridge port (33300->0, 33310->1, ...) so the
    // emulated gimlet SPs get distinct serials and MACs. Inventory keys SPs on
    // serial, and MACs must be unique per instance: a shared MAC causes L2
    // collisions that intermittently drop management-net traffic.
    let idx: u8 = crate::config::get()
        .bridge()
        .and_then(|b| crate::bridge::a4x2_offset(&b))
        .map(|off| (off / 10) as u8)
        .unwrap_or(0);
    // MAC0: 128-MAC block. sidecar base ...45:30; gimlet k gets ...45:(20+k).
    let mac_last = if sidecar {
        0x30
    } else {
        0x20u8.wrapping_add(idx)
    };
    let mut mac0 = Vec::new();
    mac0.extend_from_slice(&[0x0e, 0x1d, 0xb7, 0xfe, 0x45, mac_last]); // base_mac
    mac0.extend_from_slice(&128u16.to_le_bytes()); // count
    mac0.push(1); // stride

    // BARC: 0XV2 barcode "version:part:rev:serial". The defaults are Oxide-style,
    // which reads as real hardware in inventory; SP_EMU_VPD_SERIAL / _PART / _REV
    // override them so an emulated SP can be told apart from a shipped one. Each
    // field is forced to printable 7-bit ASCII and bounded to the format's 11
    // bytes (see vpd_ascii_field), since a non-ASCII byte or a stray ':' would
    // otherwise produce a barcode nothing can parse.
    let cfg = crate::config::get();
    let serial = cfg.vpd_serial().map(str::to_string).unwrap_or_else(|| {
        if sidecar {
            "BRM42220001".to_string()
        } else {
            format!("BRM4422000{}", idx)
        }
    });
    // The part number must name the board actually modeled. gimlet-c is
    // 913-0000019 (omicron `GIMLET_SLED_MODEL`). sp-emu models sidecar-c (the
    // board-rev straps below encode 0b010), but no sidecar part number is
    // recorded in the sources sp-emu can see, so rather than report a gimlet
    // part number for a sidecar, say so and use an obvious placeholder until
    // SP_EMU_VPD_PART supplies the real one.
    let part = cfg.vpd_part().map(str::to_string).unwrap_or_else(|| {
        if sidecar {
            eprintln!(
                "[vpd] no sidecar part number known; reporting a placeholder. \
                 Set SP_EMU_VPD_PART to the real one."
            );
            "SIDECAR-C".to_string()
        } else {
            "913-0000019".to_string()
        }
    });
    // Board revision: both modeled boards are rev C.
    let rev = cfg
        .vpd_rev()
        .map(str::to_string)
        .unwrap_or_else(|| "002".to_string());
    let serial = vpd_ascii_field("serial", &serial, 11);
    let part = vpd_ascii_field("part", &part, 11);
    let rev = vpd_ascii_field("rev", &rev, 11);
    if crate::dbg::vpd() {
        eprintln!("[vpd] BARC part={part} rev={rev} serial={serial}");
    }
    let barc = format!("0XV2:{part}:{rev}:{serial}");
    let mut fru0 = tlvc_chunk(b"MAC0", &mac0);
    fru0.extend_from_slice(&tlvc_chunk(b"BARC", barc.as_bytes()));
    let root = tlvc_chunk(b"FRU0", &fru0);
    img[..root.len()].copy_from_slice(&root);
    Rc::new(img)
}

/// STM32H7 I2C controller: minimal FSM so the driver's transactions complete.
/// ISR (+0x18) always reports TXE|TXIS|RXNE|TC (ready to send / data available /
/// transfer complete) with BUSY and NACKF clear; the driver writes bytes to TXDR
/// (+0x28, discarded) and reads RXDR (+0x24, returns 0): turn-off writes
/// succeed; unmodeled sensor reads return 0. Other registers store/return.
pub struct I2c {
    regs: std::collections::HashMap<u32, u32>,
    ev_irq: u16,
    active: bool,
    env: Sensors,
    // --- transaction state for modeling real device registers ---
    addr: u8,                             // current 7-bit target (from CR2.SADD)
    reg_ptr: u8,                          // device register pointer (from the write phase)
    read_idx: u16,                        // byte index within the current read phase
    writing: bool,                        // current phase is a master write (register-pointer set)
    wrote_ptr: bool,                      // captured the register-pointer byte this write phase
    eeprom: Rc<Vec<u8>>,                  // AT24CSW080 VPD/FRUID backing store (1024 bytes)
    bridge: crate::i2c_bridge::I2cBridge, // SP_EMU_I2C_BRIDGE sniff / _DEVICE delegate (no-op when off)
    bus: u8,                              // 1-based bus number (i2c1..i2c4) for the trace
    // NACK every target on this controller. Sidecar I2C2 carries only the
    // front IO board (front_io + frontgps ports); with no board modeled, the
    // sequencer's FrontIOBoard::present probe must see NoDevice, not a false
    // ACK that pulls in the whole front-IO bring-up (which then panics it).
    nack_all: bool,
}
impl I2c {
    pub fn new(
        ev_irq: u16,
        env: Sensors,
        eeprom: Rc<Vec<u8>>,
        bridge: crate::i2c_bridge::I2cBridge,
        bus: u8,
    ) -> Self {
        I2c {
            regs: std::collections::HashMap::new(),
            ev_irq,
            active: false,
            env,
            addr: 0,
            reg_ptr: 0,
            read_idx: 0,
            writing: false,
            wrote_ptr: false,
            eeprom,
            bridge,
            bus,
            nack_all: crate::config::get().board().is_sidecar() && bus == 2,
        }
    }
    /// Accurate device-register model, keyed by I2C address. Returns the 16-bit
    /// value of `reg` (drivers read big-endian: high byte first; single-byte reads
    /// take the high byte). Physical values come from the SensorEnv. `None` = no
    /// modeled device here -> bus reads 0 -> that device stays "Failed" until
    /// modeled. Add devices by extending this match.
    fn device_reg(&self, addr: u8, reg: u8) -> Option<u16> {
        let env = self.env.borrow();
        match addr {
            // TMP117 temperature sensors (front/rear, 0x48-0x4a): 7.8125 m°C/LSB,
            // DeviceID must read 0x0117.
            0x48..=0x4a => Some(match reg {
                0x0f => 0x0117,                                       // DeviceID
                0x00 => (env.temp_c(addr) / 0.0078125) as i16 as u16, // TempResult
                0x01 => 0x0220,                                       // Configuration
                _ => 0,
            }),
            // TSE2004av DIMM temp sensors (bus "mid", 0x18-0x1f): DeviceIdRevision
            // upper byte must be 0x22; AmbientTemp is a 13-bit value (raw = °C*16).
            0x18..=0x1f => Some(match reg {
                0x07 => 0x2200, // DeviceIdRevision
                0x05 => (((env.temp_c(addr) / 0.0078125) as i16 >> 3) as u16) & 0x1fff, // AmbientTemp
                _ => 0,
            }),
            // AT24CSW080 VPD/FRUID EEPROMs are handled out-of-band in the RXDR
            // read path (addresses 0x50..0x53), not here; see the Mmio::read
            // EEPROM branch. They need real sequential, auto-incrementing reads
            // off the `eeprom` backing store, not the 16-bit-register split below.
            // TMP451 (T6 NIC temp, behind the M.2 mux seg 4, addr 0x4c). Reads are
            // single-byte -> the value goes in the high byte (read_idx 0). ManufacturerId
            // (0xFE) must be 0x55 (TI); Local/Remote temp hi byte = integer °C.
            0x4c => Some(match reg {
                0xFE => 0x5500,                                         // ManufacturerId = 0x55
                0x00 | 0x01 => ((env.temp_c(addr) as i16) << 8) as u16, // Local/Remote temp hi byte
                _ => 0,
            }),
            _ => None,
        }
    }
}
impl Mmio for I2c {
    fn name(&self) -> &str {
        "I2C"
    }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x18 => {
                // ISR. An absent target NACKs: NACKF(4)|STOPF(5), no TXIS/RXNE,
                // so the driver returns NoDevice (stm32xx-i2c checks NACKF
                // before waiting on TXIS/RXNE/TC).
                if self.nack_all && self.active {
                    (1 << 4) | (1 << 5)
                } else {
                    (1 << 0) | (1 << 1) | (1 << 2) | (1 << 6) // TXE|TXIS|RXNE|TC
                }
            }
            0x24 => {
                // RXDR: serve the modeled device register / EEPROM byte
                if crate::dbg::vpd() {
                    eprintln!(
                        "[i2c{:#x}] RD RXDR addr={:#04x} ptr={} ridx={}",
                        self.ev_irq, self.addr, self.reg_ptr, self.read_idx
                    );
                }
                // DELEGATE (SP_EMU_I2C_DEVICE): a local device server may answer
                // this read; `None` falls through to the built-in model below.
                if let Some(b) =
                    self.bridge
                        .on_read(self.bus, self.addr, self.reg_ptr, self.read_idx)
                {
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
                        eprintln!(
                            "[vpd] rd addr={:#04x} ptr={} ridx={} off={} -> {:#04x}",
                            self.addr, self.reg_ptr, self.read_idx, idx, byte
                        );
                    }
                    self.bridge.on_read_served(
                        self.bus,
                        self.addr,
                        self.reg_ptr,
                        self.read_idx,
                        byte,
                    );
                    self.read_idx = self.read_idx.wrapping_add(1);
                    return byte as u32;
                }
                let v = self.device_reg(self.addr, self.reg_ptr).unwrap_or(0);
                let byte = if self.read_idx == 0 {
                    (v >> 8) & 0xFF
                } else {
                    v & 0xFF
                };
                self.bridge.on_read_served(
                    self.bus,
                    self.addr,
                    self.reg_ptr,
                    self.read_idx,
                    byte as u8,
                );
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
        if off & !3 == 0x28 {
            // TXDR: first byte of a write phase is the register pointer
            let byte = (val & 0xFF) as u8;
            if self.writing && !self.wrote_ptr {
                self.reg_ptr = byte;
                self.wrote_ptr = true;
            }
            self.bridge.on_write(self.bus, self.addr, byte);
            return;
        }
        if off & !3 == 0x04 {
            // CR2: START begins a master transfer, STOP ends it.
            if val & (1 << 13) != 0 {
                // START
                self.active = true;
                self.addr = ((val >> 1) & 0x7F) as u8; // SADD[7:1] = 7-bit address
                if val & (1 << 10) != 0 {
                    // RD_WRN set -> read phase
                    self.read_idx = 0;
                    self.writing = false;
                } else {
                    // write phase (sets the register pointer)
                    self.writing = true;
                    self.wrote_ptr = false;
                }
                if crate::dbg::vpd() {
                    eprintln!(
                        "[i2c{:#x}] START addr={:#04x} rd={} nbytes={}",
                        self.ev_irq,
                        self.addr,
                        (val >> 10) & 1,
                        (val >> 16) & 0xFF
                    );
                }
                self.bridge.on_start(
                    self.bus,
                    self.addr,
                    val & (1 << 10) != 0,
                    (val >> 16) & 0xFF,
                );
            }
            if val & (1 << 14) != 0 {
                self.active = false;
            } // STOP
              // START/STOP are command bits that auto-clear in hardware; store them
              // cleared so a later read-modify-write doesn't carry a stale START.
            self.regs.insert(0x04, val & !((1 << 13) | (1 << 14)));
            return;
        }
        self.regs.insert(off & !3, val);
    }
    // The master read path waits (wfi) for the event IRQ before checking RXNE.
    // Raise it only while a master transfer is active (CR2.START..STOP) so I2C
    // slave mode (gimlet-spd's operate_as_target, never addressed in the
    // emulator) stays blocked instead of busy-looping on stray IRQs.
    fn take_irq(&mut self) -> Option<u16> {
        if self.active {
            Some(self.ev_irq)
        } else {
            None
        }
    }
}

/// QUADSPI (0x5200_5000): writable model of the SPI NOR flash driven by the
/// gimlet `hf` task (host flash) and the sidecar `auxflash` task (FPGA blob
/// slots). 32 MiB array with real command semantics, so both the boot-time
/// scans and the MGS update path (slot erase, program, CHCK read-back) work.
///
/// * RDID (0x9F): [0x20, 0xBA, 0x19, ...], Micron MT25Q 32 MiB. byte0
///   manufacturer, byte1 voltage, byte2 log2(capacity). hf's init fails
///   unless it recognizes this id.
/// * RDSR (0x05): WIP (bit 0) always 0, operations complete instantly.
///   WEL (bit 1) from the write-enable latch. auxflash's
///   set_and_check_write_enable fails the update if WEL does not read back.
/// * WREN (0x06) / WRDI (0x04): set/clear WEL.
/// * SectorErase (0xDC): 64 KiB sector at the 4-byte address to 0xFF.
/// * BulkErase (0xC7): whole array to 0xFF.
/// * PageProgram (0x12): AND the data into the array at the 4-byte address.
/// * Any addressed read (Read 0x13, QuadRead 0x6C, ...): array contents.
///
/// Program and erase require WEL and clear it on completion. Contents persist
/// to `qspi-flash.bin` next to the SP flash NVM file, write-through like
/// `Flash`. The file is created lazily on the first program or erase, so a
/// gimlet that never writes its host flash creates no file.
///
/// The driver (drv-stm32h7-qspi) polls SR.FLEVEL (bits 8..13) and SR.TCF
/// (bit 1), moving data one byte at a time through DR (offset 0x20). Reads
/// are presented whole, with TCF set once drained. Writes report FLEVEL 0 and
/// TCF as soon as the command's address and data have arrived, so the driver
/// never waits on the qspi irq. None is raised; a stray qspi irq would
/// busy-loop hf.
const QSPI_SIZE: usize = 32 * 1024 * 1024;
const QSPI_SECTOR: usize = 65_536;

/// An indirect transfer decoded from CCR that still needs its address (AR
/// write) and/or data (DR writes) before it can execute.
enum QspiXfer {
    Idle,
    /// Addressed read. The response is built once AR arrives.
    ReadAtAddr { len: usize },
    /// Addressed write. On the AR write, a data-less command (sector erase)
    /// executes; `data: true` (page program) starts collecting data bytes.
    WriteAtAddr { instruction: u8, data: bool },
    /// Page program with AR seen, collecting `remaining` data bytes via DR.
    WriteData { remaining: usize },
}

pub struct Qspi {
    dlr: u32,        // transfer length register (holds len-1)
    ar: u32,         // address register (last AR write)
    resp: Vec<u8>,   // pending read response
    resp_pos: usize, // bytes drained from `resp`
    mode_read: bool, // current transfer is an indirect read
    tcf: bool,       // transfer-complete latch
    cr: u32,         // control register (stored, for EN bit etc.)
    dcr: u32,        // device config (stored)
    wel: bool,       // write-enable latch (RDSR bit 1)
    xfer: QspiXfer,  // multi-register transfer in progress
    wr_buf: Vec<u8>, // page-program data collected so far
    mem: Vec<u8>,    // the 32 MiB NOR array
    /// Write-through handle to the backing file, as in `Flash::file`. Opened
    /// at construction if a persisted image exists, else created and seeded on
    /// the first program or erase. `None` before that, or after an I/O error;
    /// the model then continues RAM-only.
    file: Option<std::fs::File>,
    /// Backing file path. `None` disables persistence (tests).
    path: Option<String>,
}
impl Qspi {
    pub fn new() -> Self {
        // Persist next to the SP flash NVM file, one array per instance.
        let nvm = crate::config::instance_file("SP_EMU_FLASH", crate::config::get().flash_path());
        let path = crate::flash::instance_base(&nvm)
            .join("qspi-flash.bin")
            .display()
            .to_string();
        Self::with_backing(Some(path))
    }
    /// Build the model, loading a persisted array from `path` if one exists.
    fn with_backing(path: Option<String>) -> Self {
        let mut mem = vec![ERASED; QSPI_SIZE];
        let mut file = None;
        if let Some(p) = path.as_deref() {
            if std::path::Path::new(p).exists() {
                match std::fs::OpenOptions::new().read(true).write(true).open(p) {
                    Ok(mut f) => {
                        use std::io::Read;
                        let mut buf = Vec::new();
                        match f.read_to_end(&mut buf) {
                            Ok(_) => {
                                let n = buf.len().min(QSPI_SIZE);
                                mem[..n].copy_from_slice(&buf[..n]);
                                eprintln!("[qspi] loaded persisted QSPI flash from {p}");
                                file = Some(f);
                            }
                            Err(e) => eprintln!("[qspi] read {p} failed: {e}; starting erased"),
                        }
                    }
                    Err(e) => eprintln!("[qspi] open {p} failed: {e}; running RAM-only"),
                }
            }
        }
        Qspi {
            dlr: 0,
            ar: 0,
            resp: Vec::new(),
            resp_pos: 0,
            mode_read: false,
            tcf: false,
            cr: 0,
            dcr: 0,
            wel: false,
            xfer: QspiXfer::Idle,
            wr_buf: Vec::new(),
            mem,
            file,
            path,
        }
    }
    /// Response for a non-addressed read command (RDID, RDSR, unique id).
    fn build_response(&self, instruction: u8, len: usize) -> Vec<u8> {
        let mut v = vec![0u8; len];
        match instruction {
            0x9F => {
                // ReadId (RDID): Micron MT25Q, 32 MiB (log2 capacity = 0x19)
                let id = [0x20u8, 0xBA, 0x19];
                for (i, b) in id.iter().enumerate() {
                    if i < len {
                        v[i] = *b;
                    }
                }
            }
            0x05 => {
                // ReadStatusReg: WIP (bit 0) always clear, WEL (bit 1) live.
                if self.wel {
                    v[0] = 0x02;
                }
            }
            // Unknown non-addressed reads (e.g. Winbond unique id): 0xFF fill.
            _ => {
                for b in v.iter_mut() {
                    *b = 0xFF;
                }
            }
        }
        v
    }
    /// Response for an addressed read: the array contents (wrapping at 32 MiB).
    fn read_mem(&self, addr: u32, len: usize) -> Vec<u8> {
        let mut v = vec![ERASED; len];
        for (i, b) in v.iter_mut().enumerate() {
            *b = self.mem[(addr as usize + i) & (QSPI_SIZE - 1)];
        }
        v
    }
    /// Open (creating + seeding if needed) the backing file on first write.
    /// A failure clears `path` so a broken target is not retried per write.
    fn ensure_file(&mut self) {
        if self.file.is_some() {
            return;
        }
        let Some(p) = self.path.clone() else { return };
        use std::io::{Seek, SeekFrom, Write};
        match std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(&p)
        {
            Ok(mut f) => {
                let r = f
                    .seek(SeekFrom::Start(0))
                    .and_then(|_| f.write_all(&self.mem))
                    .and_then(|_| f.set_len(QSPI_SIZE as u64));
                match r {
                    Ok(()) => {
                        eprintln!("[qspi] created backing file {p}");
                        self.file = Some(f);
                    }
                    Err(e) => {
                        eprintln!("[qspi] seed {p} failed: {e}; RAM-only");
                        self.path = None;
                    }
                }
            }
            Err(e) => {
                eprintln!("[qspi] create {p} failed: {e}; RAM-only");
                self.path = None;
            }
        }
    }
    /// Write `mem[off..off+len]` through to the backing file (see
    /// `Flash::write_through`).
    fn write_through(&mut self, off: usize, len: usize) {
        use std::io::{Seek, SeekFrom, Write};
        if len == 0 {
            return;
        }
        self.ensure_file();
        if let Some(f) = self.file.as_mut() {
            let r = f
                .seek(SeekFrom::Start(off as u64))
                .and_then(|_| f.write_all(&self.mem[off..off + len]));
            if let Err(e) = r {
                eprintln!(
                    "[qspi] write-through to {} failed: {e}",
                    self.path.as_deref().unwrap_or("?")
                );
                self.file = None; // stop retrying a broken handle
                self.path = None;
            }
        }
    }
    /// Sector (or bulk) erase at `self.ar`. Requires WEL; consumes it.
    fn do_erase(&mut self, instruction: u8) {
        if !self.wel {
            if crate::dbg::eth() {
                eprintln!("[qspi] erase {:#04x} without WEL, ignored", instruction);
            }
            self.tcf = true;
            return;
        }
        self.wel = false;
        let (base, len) = match instruction {
            0xC7 => (0, QSPI_SIZE), // BulkErase
            // SectorErase (0xDC 4-byte / 0xD8 3-byte): 64 KiB, aligned down.
            _ => (
                self.ar as usize & (QSPI_SIZE - 1) & !(QSPI_SECTOR - 1),
                QSPI_SECTOR,
            ),
        };
        self.mem[base..base + len].fill(ERASED);
        self.write_through(base, len);
        self.tcf = true;
        if crate::dbg::eth() {
            eprintln!("[qspi] erase {:#04x} @ {:#010x} len={}", instruction, base, len);
        }
    }
    /// Page program `self.wr_buf` at `self.ar`. Requires WEL; consumes it.
    /// NOR semantics: programming only clears bits.
    fn do_program(&mut self) {
        let buf = std::mem::take(&mut self.wr_buf);
        if !self.wel {
            if crate::dbg::eth() {
                eprintln!("[qspi] program without WEL, ignored");
            }
            self.tcf = true;
            return;
        }
        self.wel = false;
        let base = self.ar as usize & (QSPI_SIZE - 1);
        for (i, b) in buf.iter().enumerate() {
            self.mem[(base + i) & (QSPI_SIZE - 1)] &= *b;
        }
        // Write through both segments if the program wraps past the end.
        let first = buf.len().min(QSPI_SIZE - base);
        self.write_through(base, first);
        if first < buf.len() {
            self.write_through(0, buf.len() - first);
        }
        self.tcf = true;
        if crate::dbg::eth() {
            eprintln!("[qspi] program @ {:#010x} len={}", base, buf.len());
        }
    }
}
impl Mmio for Qspi {
    fn name(&self) -> &str {
        "QUADSPI"
    }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x00 => self.cr,
            0x04 => self.dcr,
            0x08 => {
                // SR
                let remaining = self.resp.len().saturating_sub(self.resp_pos);
                if self.mode_read && remaining == 0 {
                    self.tcf = true;
                }
                let flevel = if self.mode_read {
                    remaining.min(32) as u32
                } else {
                    0
                };
                let mut sr = 0u32;
                if self.tcf {
                    sr |= 1 << 1;
                } // TCF
                if !self.mode_read || remaining > 0 {
                    sr |= 1 << 2;
                } // FTF
                sr |= flevel << 8; // FLEVEL[5:0]
                sr
            }
            0x10 => self.dlr,
            0x18 => self.ar,
            0x20 => {
                // DR: pop one byte (driver reads the low byte via byte access)
                if self.resp_pos < self.resp.len() {
                    let b = self.resp[self.resp_pos] as u32;
                    self.resp_pos += 1;
                    b
                } else {
                    0xFF
                }
            }
            _ => 0,
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        match off & !3 {
            0x00 => self.cr = val,
            0x04 => self.dcr = val,
            0x0C => {
                if val & (1 << 1) != 0 {
                    self.tcf = false;
                }
            } // FCR.CTCF
            0x10 => self.dlr = val, // DLR (len-1)
            0x14 => {
                // CCR: instruction bits[7:0], ADMODE bits[11:10], DMODE
                // bits[25:24], FMODE bits[27:26]. The driver writes CCR first;
                // for addressed commands AR follows immediately (nothing polls
                // SR in between), then data moves through DR.
                let instruction = (val & 0xFF) as u8;
                let admode = (val >> 10) & 0b11;
                let dmode = (val >> 24) & 0b11;
                let fmode = (val >> 26) & 0b11;
                let len = (self.dlr as usize).wrapping_add(1);
                if fmode == 0b01 {
                    // Indirect read. Addressed (memory) reads wait for AR; the
                    // rest (RDID/RDSR/unique-id) answer from the command alone.
                    if admode != 0 {
                        self.resp.clear();
                        self.xfer = QspiXfer::ReadAtAddr { len };
                    } else {
                        self.resp = self.build_response(instruction, len);
                        self.xfer = QspiXfer::Idle;
                    }
                    self.resp_pos = 0;
                    self.mode_read = true;
                    self.tcf = false;
                } else {
                    // Indirect write: write-enable / erase / program.
                    self.mode_read = false;
                    self.wr_buf.clear();
                    match instruction {
                        0x06 => {
                            // WREN
                            self.wel = true;
                            self.xfer = QspiXfer::Idle;
                            self.tcf = true;
                        }
                        0x04 => {
                            // WRDI
                            self.wel = false;
                            self.xfer = QspiXfer::Idle;
                            self.tcf = true;
                        }
                        0xC7 => {
                            // BulkErase: no address, executes now.
                            self.xfer = QspiXfer::Idle;
                            self.do_erase(instruction);
                        }
                        _ if admode != 0 => {
                            // Addressed write: PageProgram (data follows) or
                            // SectorErase (data-less); resolved at the AR write.
                            self.xfer = QspiXfer::WriteAtAddr {
                                instruction,
                                data: dmode != 0,
                            };
                            self.tcf = false;
                        }
                        _ => {
                            // Unknown address-less write command: accept, done.
                            self.xfer = QspiXfer::Idle;
                            self.tcf = true;
                        }
                    }
                }
                if crate::dbg::eth() {
                    eprintln!(
                        "[qspi] CCR instr={:#04x} fmode={:#b} admode={:#b} dmode={:#b} dlr={}",
                        instruction, fmode, admode, dmode, self.dlr
                    );
                }
            }
            0x18 => {
                // AR: completes the address phase of the pending transfer.
                self.ar = val;
                match self.xfer {
                    QspiXfer::ReadAtAddr { len } => {
                        self.resp = self.read_mem(val, len);
                        self.resp_pos = 0;
                        self.xfer = QspiXfer::Idle;
                    }
                    QspiXfer::WriteAtAddr { instruction, data } => {
                        // Data-less (erase) commands execute here; a program
                        // now collects its DLR+1 data bytes through DR.
                        if data {
                            self.xfer = QspiXfer::WriteData {
                                remaining: (self.dlr as usize).wrapping_add(1),
                            };
                        } else {
                            self.xfer = QspiXfer::Idle;
                            self.do_erase(instruction);
                        }
                    }
                    _ => {}
                }
            }
            0x20 => {
                // DR write: one data byte of a page program (byte-wide access,
                // like DR reads).
                if let QspiXfer::WriteData { remaining } = self.xfer {
                    self.wr_buf.push((val & 0xFF) as u8);
                    if self.wr_buf.len() >= remaining {
                        self.xfer = QspiXfer::Idle;
                        self.do_program();
                    }
                }
            }
            _ => {} // interrupt-enable bits in CR etc.: accept/ignore
        }
    }
    // No irq needed: the driver completes by polling FLEVEL/TCF, satisfied
    // immediately. Raising no irq keeps hf from busy-looping.
    fn take_irq(&mut self) -> Option<u16> {
        None
    }
}

/// SYSCFG: store/return except PKGR (+0x124), whose pkg[3:0] field reads back
/// 0b1000 (TFBGA240) so gimlet's package-guard accepts the firmware.
pub struct Syscfg {
    regs: std::collections::HashMap<u32, u32>,
}
impl Syscfg {
    pub fn new() -> Self {
        Syscfg {
            regs: std::collections::HashMap::new(),
        }
    }
}
impl Mmio for Syscfg {
    fn name(&self) -> &str {
        "SYSCFG"
    }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x124 => (*self.regs.get(&0x124).unwrap_or(&0) & !0xF) | 0b1000, // PKGR.pkg
            o => *self.regs.get(&o).unwrap_or(&0),
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        self.regs.insert(off & !3, val);
    }
}

/// STM32H7 unique device ID (96 bits / 3 words): a stable fake identity.
pub struct Uid;
impl Mmio for Uid {
    fn name(&self) -> &str {
        "UID"
    }
    fn read(&mut self, off: u32) -> u32 {
        // Per-instance 96-bit UID (3 words at 0x0/0x4/0x8); higher offsets read a
        // stable non-zero filler. Firmware reads the UID early in boot (a zero /
        // unmapped read faults a downstream slice), so it must be non-zero. The
        // SP's MAC comes from the VPD EEPROM (`build_vpd_eeprom`), not this UID.
        crate::identity::sp_uid_word(off & !3)
    }
    fn write(&mut self, _: u32, _: u32) {}
}

/// Generic peripheral that stores writes and returns them (sparse); models
/// config registers whose only requirement is readback consistency.
pub struct RegFile {
    name: &'static str,
    regs: std::collections::HashMap<u32, u32>,
}
impl RegFile {
    pub fn new(name: &'static str) -> Self {
        RegFile {
            name,
            regs: std::collections::HashMap::new(),
        }
    }
}
impl Mmio for RegFile {
    fn name(&self) -> &str {
        self.name
    }
    fn read(&mut self, off: u32) -> u32 {
        *self.regs.get(&(off & !3)).unwrap_or(&0)
    }
    fn write(&mut self, off: u32, val: u32) {
        self.regs.insert(off & !3, val);
    }
}

// ---- RCC: clock tree. Ready bits mirror their enable bits. -----------------

pub struct Rcc {
    regs: [u32; 0x100],
}

impl Rcc {
    pub fn new() -> Self {
        Rcc { regs: [0; 0x100] }
    }
}

impl Mmio for Rcc {
    fn name(&self) -> &str {
        "RCC"
    }
    fn read(&mut self, off: u32) -> u32 {
        let i = (off / 4) as usize & 0xff;
        let mut v = self.regs[i];
        match off {
            0x00 => {
                // CR: synthesize *RDY immediately from each *ON request.
                v |= 1 << 1; // HSIRDY  (HSI always running out of reset)
                v |= 1 << 2; // HSIDIVF / CSIRDY stand-in
                if v & (1 << 16) != 0 {
                    v |= 1 << 17;
                } // HSEON  -> HSERDY
                if v & (1 << 24) != 0 {
                    v |= 1 << 25;
                } // PLL1ON -> PLL1RDY
                if v & (1 << 26) != 0 {
                    v |= 1 << 27;
                } // PLL2ON -> PLL2RDY
                if v & (1 << 28) != 0 {
                    v |= 1 << 29;
                } // PLL3ON -> PLL3RDY
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
    pub fn new() -> Self {
        Pwr { regs: [0; 0x40] }
    }
}

impl Mmio for Pwr {
    fn name(&self) -> &str {
        "PWR"
    }
    fn read(&mut self, off: u32) -> u32 {
        let i = (off / 4) as usize & 0x3f;
        let v = self.regs[i];
        match off {
            0x04 => v | (1 << 13), // CSR1.ACTVOSRDY  (startup spins on this)
            0x18 => v | (1 << 13), // D3CR.VOSRDY     (offset 0x18 on STM32H7)
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
// Fixed by the architecture at 0xE000_E000 (not in chip.toml). Stores register
// writes and logs the architecturally interesting ones (VTOR, CPACR, SysTick).
// NVIC interrupt delivery is not modeled here.

pub struct Scs {
    regs: [u32; 0x400], // 0x1000 bytes / 4
}

impl Scs {
    pub fn new() -> Self {
        Scs { regs: [0; 0x400] }
    }
}

impl Mmio for Scs {
    fn name(&self) -> &str {
        "SCS"
    }
    fn read(&mut self, off: u32) -> u32 {
        let i = (off / 4) as usize & 0x3ff;
        match off {
            0x010 => self.regs[i] | (1 << 16), // SYST_CSR: report COUNTFLAG set
            // CPUID for STM32H753 = Cortex-M7 r1p1: implementer 0x41, variant 0x1,
            // architecture 0xF (ARMv7-M), partno 0xC27 (Cortex-M7), revision 0x1.
            0xD00 => 0x411F_C271,
            _ => self.regs[i],
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        let i = (off / 4) as usize & 0x3ff;
        self.regs[i] = val;
        if crate::dbg::exc() {
            match off {
                0xD08 => eprintln!("[scs] VTOR  = {:#010x}", val),
                0xD88 => eprintln!("[scs] CPACR = {:#010x} (FPU enable)", val),
                _ => {}
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- QSPI: drive the register interface like drv-stm32h7-qspi does ----
    // CCR first (instruction, ADMODE, DMODE, FMODE), then AR for addressed
    // commands, then byte-wide DR traffic. SR.TCF is polled for completion.

    fn qspi_ccr(instr: u8, fmode: u32, addressed: bool, data: bool) -> u32 {
        let mut v = instr as u32 | (fmode << 26);
        if addressed {
            v |= (0b01 << 10) | (0b11 << 12); // ADMODE single line, ADSIZE 32-bit
        }
        if data {
            v |= 0b01 << 24; // DMODE single line
        }
        v
    }

    fn qspi_read(q: &mut Qspi, instr: u8, addr: Option<u32>, len: usize) -> Vec<u8> {
        q.write(0x10, (len - 1) as u32); // DLR
        q.write(0x14, qspi_ccr(instr, 0b01, addr.is_some(), true));
        if let Some(a) = addr {
            q.write(0x18, a);
        }
        (0..len).map(|_| q.read(0x20) as u8).collect()
    }

    fn qspi_cmd(q: &mut Qspi, instr: u8, addr: Option<u32>, data: &[u8]) {
        if !data.is_empty() {
            q.write(0x10, (data.len() - 1) as u32); // DLR
        }
        q.write(0x14, qspi_ccr(instr, 0b00, addr.is_some(), !data.is_empty()));
        if let Some(a) = addr {
            q.write(0x18, a);
        }
        for b in data {
            q.write(0x20, *b as u32);
        }
        // The driver ends every write by polling SR.TCF; the model must have
        // completed by now (operations are instant).
        assert_ne!(q.read(0x08) & (1 << 1), 0, "TCF after {instr:#04x}");
    }

    #[test]
    fn qspi_write_enable_latch_reads_back_in_rdsr() {
        let mut q = Qspi::with_backing(None);
        // auxflash's set_and_check_write_enable: WREN then RDSR, bit 1 must
        // set, else WriteEnableFailed (MGS "code 17"). WIP (bit 0) clear.
        assert_eq!(qspi_read(&mut q, 0x05, None, 1)[0], 0x00);
        qspi_cmd(&mut q, 0x06, None, &[]);
        assert_eq!(qspi_read(&mut q, 0x05, None, 1)[0], 0x02);
        // A program consumes WEL, as on a real part.
        qspi_cmd(&mut q, 0x12, Some(0), &[0xAB]);
        assert_eq!(qspi_read(&mut q, 0x05, None, 1)[0], 0x00);
        // RDID still answers the Micron MT25Q id the hf init check requires.
        assert_eq!(qspi_read(&mut q, 0x9F, None, 3), vec![0x20, 0xBA, 0x19]);
    }

    #[test]
    fn qspi_erase_program_readback() {
        let mut q = Qspi::with_backing(None);
        let base = 0x0002_0000u32; // slot-interior, sector-aligned
        let data = [0xDE, 0xAD, 0xBE, 0xEF];

        // The auxflash update sequence: write-enable + sector erase, then
        // write-enable + page program, then read back (the CHCK scan).
        qspi_cmd(&mut q, 0x06, None, &[]);
        qspi_cmd(&mut q, 0xDC, Some(base), &[]);
        qspi_cmd(&mut q, 0x06, None, &[]);
        qspi_cmd(&mut q, 0x12, Some(base), &data);
        assert_eq!(qspi_read(&mut q, 0x13, Some(base), 4), data.to_vec());
        // Neighbors are untouched (erased).
        assert_eq!(qspi_read(&mut q, 0x13, Some(base + 4), 2), vec![0xFF; 2]);

        // NOR semantics: a second program only clears bits (0xDE & 0x21 = 0x00).
        qspi_cmd(&mut q, 0x06, None, &[]);
        qspi_cmd(&mut q, 0x12, Some(base), &[0x21]);
        assert_eq!(qspi_read(&mut q, 0x13, Some(base), 1), vec![0xDE & 0x21]);

        // Sector erase clears the whole 64 KiB sector back to 0xFF.
        qspi_cmd(&mut q, 0x06, None, &[]);
        qspi_cmd(&mut q, 0xDC, Some(base + 7), &[]); // interior address: aligned down
        assert_eq!(qspi_read(&mut q, 0x13, Some(base), 4), vec![0xFF; 4]);

        // Program/erase without a preceding WREN are ignored.
        qspi_cmd(&mut q, 0x12, Some(base), &[0x00]);
        assert_eq!(qspi_read(&mut q, 0x13, Some(base), 1), vec![0xFF]);
    }

    #[test]
    fn qspi_persists_to_backing_file() {
        let path = std::env::temp_dir()
            .join(format!("sp-emu-qspi-test-{}.bin", std::process::id()))
            .display()
            .to_string();
        let _ = std::fs::remove_file(&path);

        let base = 0x0001_0000u32;
        {
            let mut q = Qspi::with_backing(Some(path.clone()));
            qspi_cmd(&mut q, 0x06, None, &[]);
            qspi_cmd(&mut q, 0x12, Some(base), &[0x5A, 0xA5]);
        }
        // A fresh instance (a new sp-emu run) sees the programmed bytes.
        let mut q = Qspi::with_backing(Some(path.clone()));
        assert_eq!(qspi_read(&mut q, 0x13, Some(base), 2), vec![0x5A, 0xA5]);
        std::fs::remove_file(&path).unwrap();
    }

    // ---- VSC7448 MIIM bridge + VSC8504 PHY model -----------------------------

    /// MII_CMD encoding per DEVCPU_GCB MIIM: VLD bit 31, PHYAD [29:25],
    /// REGAD [24:20], WRDATA [19:4], OPR [2:1] (01 write, 10 read).
    fn miim(phy: u8, reg: u8, wr: Option<u16>) -> u32 {
        (1 << 31)
            | ((phy as u32) << 25)
            | ((reg as u32) << 20)
            | ((wr.unwrap_or(0) as u32) << 4)
            | if wr.is_some() { 0b01 << 1 } else { 0b10 << 1 }
    }

    #[test]
    fn vsc7448_miim_serves_vsc8504_identity_and_micro_commands() {
        let mut v = Vsc7448::new();
        // read_id: STANDARD (page 0) IDENTIFIER_1/2 must yield VSC8504_ID
        // 0x704c2, else monorail panics BspInitFailed(BadPhyId).
        v.mii_cmd(miim(4, 31, Some(0)));
        v.mii_cmd(miim(4, 2, None));
        assert_eq!(v.mii_data, 0x0007);
        v.mii_cmd(miim(4, 3, None));
        assert_eq!(v.mii_data, 0x04c2);

        // GPIO page (0x10): EXTENDED_REVISION reads tesla_e (bit 0) set.
        v.mii_cmd(miim(4, 31, Some(0x10)));
        v.mii_cmd(miim(4, 30, None));
        assert_eq!(v.mii_data & 1, 1);

        // A micro command completes at once: busy (15) and error (14) clear.
        // The CRC command leaves the Tesla patch's expected CRC in
        // VERIPHY_CTRL_REG2 (EXTENDED page reg 25) so the download is skipped.
        v.mii_cmd(miim(4, 18, Some(0x8008)));
        v.mii_cmd(miim(4, 18, None));
        assert_eq!(v.mii_data & 0xC000, 0);
        v.mii_cmd(miim(4, 31, Some(1)));
        v.mii_cmd(miim(4, 25, None));
        assert_eq!(v.mii_data, 0x29E8);

        // EXTENDED_PHY_CONTROL_4 bits [15:11]: the port index (MIIM addr - 4).
        v.mii_cmd(miim(4, 23, None));
        assert_eq!(v.mii_data >> 11, 0);
        v.mii_cmd(miim(6, 31, Some(1)));
        v.mii_cmd(miim(6, 23, None));
        assert_eq!(v.mii_data >> 11, 2);
    }

    // ---- Mainboard FPGA: Tofino sequencing FSM + debug port ------------------

    #[test]
    fn spi5_tofino_power_up_and_down() {
        let mut s = Spi5::new(Rc::new(Cell::new(0)));
        assert_eq!(s.fpga[&0x101], 1, "resting state A2");

        // EN set: A0, VID valid, all six rails ENABLE|GOOD.
        s.fpga.insert(0x100, 0x02);
        s.tofino_seq_ctrl_written();
        assert_eq!(s.fpga[&0x101], 2, "A0");
        assert_eq!(s.fpga[&0x10c], 0x88, "VID_VALID | V0P759");
        for a in 0x106..=0x10bu16 {
            assert_eq!(s.fpga[&a], 0x03);
        }
        // ACK_VID self-clears and does not disturb the state.
        s.fpga.insert(0x100, 0x02 | 0x04);
        s.tofino_seq_ctrl_written();
        assert_eq!(s.fpga[&0x100], 0x02, "command bits self-clear");
        assert_eq!(s.fpga[&0x101], 2);

        // EN clear: back to A2, VID invalid, rails off.
        s.fpga.insert(0x100, 0x00);
        s.tofino_seq_ctrl_written();
        assert_eq!(s.fpga[&0x101], 1);
        assert_eq!(s.fpga[&0x10c], 0);
    }

    #[test]
    fn spi5_tofino_debug_port_write_then_read_back() {
        let mut s = Spi5::new(Rc::new(Cell::new(0)));
        assert_eq!(s.fpga[&0x201], 0x05, "buffers empty at rest");

        // DirectWrite 0xCAFEF00D to address 0x01234: opcode, LE addr, LE value.
        s.dbg_req.extend([0x80, 0x34, 0x12, 0x00, 0x00]);
        s.dbg_req.extend(0xCAFEF00Du32.to_le_bytes());
        s.tofino_debug_request();
        assert_eq!(s.fpga[&0x201], 0x05, "request complete, buffers empty");

        // DirectRead of the same address returns the value, LE, via the buffer.
        s.dbg_req.extend([0xA0, 0x34, 0x12, 0x00, 0x00]);
        s.tofino_debug_request();
        let resp: Vec<u8> = s.dbg_resp.drain(..).collect();
        assert_eq!(resp, 0xCAFEF00Du32.to_le_bytes());
    }

    #[test]
    fn vpd_field_forces_printable_ascii_and_bounds_length() {
        // Clean ASCII within the bound passes through unchanged.
        assert_eq!(vpd_ascii_field("serial", "BRM44220001", 11), "BRM44220001");
        // Over-length truncates to the byte bound (bytes == chars once ASCII).
        assert_eq!(
            vpd_ascii_field("serial", "BRM4422000123", 11),
            "BRM44220001"
        );
        // Non-ASCII characters are dropped, not counted as their several UTF-8
        // bytes: an en-dash between the digits leaves the ASCII remainder.
        assert_eq!(
            vpd_ascii_field("part", "913\u{2013}0000019", 11),
            "9130000019"
        );
        // The ':' field delimiter is not allowed inside a field.
        assert_eq!(vpd_ascii_field("rev", "0:0:2", 11), "002");
        // Control characters go too.
        assert_eq!(vpd_ascii_field("serial", "AB\nCD", 11), "ABCD");
    }
}
