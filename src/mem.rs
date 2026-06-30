//! Physical memory + MMIO dispatch for the emulated SoC.
//!
//! Two kinds of regions: flat RAM/flash (backed by a `Vec<u8>`) and MMIO
//! peripherals (a `dyn Mmio`). Anything unmapped is logged and returns 0 / is
//! swallowed, so the trace keeps moving and surfaces unmodeled accesses rather
//! than faulting on the first one.

use anyhow::{bail, Result};

/// A memory-mapped peripheral. Phase 1 models everything at word granularity;
/// the bus synthesizes narrower (8/16-bit) accesses from word ops.
pub trait Mmio {
    fn name(&self) -> &str;
    fn read(&mut self, off: u32) -> u32;
    fn write(&mut self, off: u32, val: u32);
    /// Poll for a peripheral interrupt to raise. Returns an IRQ number to set
    /// pending in the NVIC (consumed — returns it once per event). Default: none.
    fn take_irq(&mut self) -> Option<u16> { None }
}

struct Ram {
    base: u32,
    data: Vec<u8>,
}

struct Device {
    base: u32,
    size: u32,
    dev: Box<dyn Mmio>,
}

pub struct Bus {
    rams: Vec<Ram>,
    devs: Vec<Device>,
    pub unmapped_reads: u64,
    pub unmapped_writes: u64,
    /// When true, log each unmapped access (noisy; useful during bringup).
    pub log_unmapped: bool,
    /// Optional address to log writes to (set via $SP_EMU_WATCH); a debugging aid.
    pub watch: Option<u32>,
    /// Current instruction PC (set by the CPU each step) — for watch logging.
    pub cur_pc: u32,
    pub cur_cyc: u64,
    /// Set when the most recent access hit an MMIO device (for the differential
    /// harness, which can't mirror peripheral semantics). Reset by the caller.
    pub mmio_hit: bool,
    /// Set whenever a peripheral device is accessed; lets `collect_irqs` skip the
    /// all-device IRQ poll on instructions that touch no device. Sound because
    /// every modeled device raises its IRQ only in response to an MMIO access
    /// (TIM16 arming, I2C/SPI transactions) — none autonomously.
    dev_touched: bool,
    /// When true, record every memory write (for the differential harness to
    /// replay into the reference model, incl. writes from skipped instructions).
    pub rec: bool,
    pub writes: Vec<(u32, u32, u8)>, // (addr, value, size-in-bytes)
    // NVIC state (IRQ 0..255): enable + pending bitmaps, and per-IRQ priority.
    nvic_en: [u32; 8],
    nvic_pend: [u32; 8],
    nvic_prio: [u8; 256],
    // PendSV pending bit (SCB ICSR.PENDSVSET) — the kernel pends this to defer a
    // context switch out of SysTick/interrupt handlers.
    pend_pendsv: bool,
    // STM32H7 Ethernet MAC/DMA — modeled in the Bus (not as an `Mmio` device)
    // because the DMA engine must read/write descriptor rings + packet buffers
    // that live in RAM, which only the Bus can reach.
    eth: EthDma,
    // host-sp-comms (UART7) byte queues, shared with the `Uart7` Mmio device. The
    // host bridge pumps them to/from the host (the propolis IPCC COM port) via
    // `pump_uart`; the RX IRQ is raised in `collect_irqs` while `uart_rx` is
    // non-empty (host input is async, outside the dev-touched IRQ poll).
    pub uart_tx: crate::soc::UartQueue, // SP -> host
    pub uart_rx: crate::soc::UartQueue, // host -> SP
}

const SCB_ICSR: u32 = 0xE000_ED04;

// ---- STM32H7 Ethernet (base 0x4002_8000; DMA block at +0x1000) -------------
const ETH_BASE: u32 = 0x4002_8000;
const ETH_END: u32 = 0x4002_A000;
const ETH_IRQ: u16 = 61;
const UART7_IRQ: u16 = 82; // host-sp-comms USART (UART7) global interrupt (H7)
// Register offsets relative to ETH_BASE.
const MACMDIOAR: u32 = 0x0200; // MDIO address/control; MB (bit0) = busy, self-clears
const DMAMR: u32 = 0x1000; // DMA mode; SWR (bit0) = soft reset, self-clears
const DMAISR: u32 = 0x1008; // interrupt summary; dc0is (bit0) = channel-0 interrupt
const DMACTXDLAR: u32 = 0x1114; // TX descriptor list base (word-aligned address)
const DMACRXDLAR: u32 = 0x111C; // RX descriptor list base
const DMACTXDTPR: u32 = 0x1120; // TX tail pointer — writing it kicks the TX DMA
const DMACRXDTPR: u32 = 0x1128; // RX tail pointer
const DMACTXRLR: u32 = 0x112C; // TX ring length minus 1
const DMACRXRLR: u32 = 0x1130; // RX ring length minus 1
const DMACSR: u32 = 0x1160; // channel status; ti=bit0, ri=bit6, nis=bit15 (W1C)
const BUFSZ: u32 = 1536; // drv/stm32h7-eth ring::BUFSZ

/// Ethernet MAC/DMA register state + the descriptor-ring DMA engine.
#[derive(Default)]
struct EthDma {
    regs: std::collections::HashMap<u32, u32>,
    rx_next: u32, // index into the RX descriptor ring
    tx_frames: Vec<Vec<u8>>, // frames emitted by the SP, awaiting the host bridge
    irq: bool, // ETH IRQ (61) pending (set on TX/RX completion)
    pending_vid: Option<u16>, // VID from the last TX VLAN context descriptor
    mdio_page: u16, // VSC85x2 PHY page selected via reg 31 (PAGE)
}

const MACMDIODR: u32 = 0x0204; // MDIO data register (read/write payload)

/// 802.1Q TPID. net runs in VLAN mode with a distinct per-port MAC per VLAN, so
/// the HostIo boundary carries tagged frames (dst|src|8100|vid|…) to preserve
/// which VLAN each frame belongs to; the bridge replies on the matching VLAN.
const VLAN_TPID: u16 = 0x8100;

const NVIC_LO: u32 = 0xE000_E100;
const NVIC_HI: u32 = 0xE000_E500;

impl Default for Bus {
    fn default() -> Self {
        Self::new()
    }
}

impl Bus {
    pub fn new() -> Self {
        let watch = std::env::var("SP_EMU_WATCH").ok()
            .and_then(|s| u32::from_str_radix(s.trim_start_matches("0x"), 16).ok());
        Bus { rams: Vec::new(), devs: Vec::new(), unmapped_reads: 0, unmapped_writes: 0, log_unmapped: true, watch, cur_pc: 0, cur_cyc: 0, mmio_hit: false, rec: false, writes: Vec::new(),
            nvic_en: [0; 8], nvic_pend: [0; 8], nvic_prio: [0; 256], pend_pendsv: false,
            dev_touched: false, eth: EthDma::default(),
            uart_tx: std::rc::Rc::new(std::cell::RefCell::new(std::collections::VecDeque::new())),
            uart_rx: std::rc::Rc::new(std::cell::RefCell::new(std::collections::VecDeque::new())) }
    }

    /// Consume a pending PendSV (the deferred context-switch request).
    pub fn take_pendsv(&mut self) -> bool {
        let p = self.pend_pendsv;
        self.pend_pendsv = false;
        p
    }

    /// True if any enabled+pending NVIC IRQ or a pending PendSV could wake a
    /// WFI-idle core. Non-consuming. Used by the gdb serve loop's idle throttle
    /// to decide whether a WFI is genuinely idle (safe to sleep the host).
    pub fn any_pending_irq(&mut self) -> bool {
        self.collect_irqs();
        self.next_irq().is_some() || self.pend_pendsv
    }

    // ---- Ethernet MAC/DMA -------------------------------------------------

    fn eth_reg(&self, off: u32) -> u32 { *self.eth.regs.get(&(off & !3)).unwrap_or(&0) }

    fn eth_read(&mut self, off: u32) -> u32 {
        let v = self.eth_reg(off);
        if off & !3 == DMACSR && crate::dbg::eth() {
            eprintln!("[eth] DMACSR read = {:#x} (net on_interrupt)", v);
        }
        match off & !3 {
            DMAMR => v & !1,     // SWR self-clears once the reset "completes"
            MACMDIOAR => v & !1, // MB clears once the MDIO op "completes"
            _ => v,
        }
    }

    /// VSC85x2 management PHY register read (over MDIO). net's vsc85xx driver
    /// reads the PHY ID, revision, and an 8051-patch CRC; returns the values
    /// that make `init_sgmii` accept the chip and skip the firmware download.
    fn phy_read(&self, page: u16, reg: u8) -> u16 {
        match (page, reg) {
            // BMCR (reg 0): the soft-reset bit (15) self-clears when reset completes;
            // the driver spins on it. Return the stored value with reset cleared.
            (0, 0) => *self.eth.regs.get(&0x1_0000).unwrap_or(&0) as u16 & !0x8000,
            (0, 2) => 0x0007,  // IDENTIFIER_1  ┐ id = 0x0007_04e2 = VSC8552_ID
            (0, 3) => 0x04e2,  // IDENTIFIER_2  ┘
            (1, 23) => 0x0000, // EXTENDED_PHY_CONTROL_4: port>>11 == 0 (base port)
            (1, 25) => 0x29e8, // VERIPHY_CTRL_REG2: 8051 CRC == EXPECTED_CRC -> skip patch
            // MICRO_PAGE: the driver writes a command (bit15=go) and polls until
            // bit15 clears (done) with bit14 clear (no error) -> mask both off.
            (16, 18) => *self.eth.regs.get(&(0x1_0000 | (16u32 << 8) | 18)).unwrap_or(&0) as u16 & 0x3FFF,
            (16, 30) => 0x0001, // EXTENDED_REVISION: tesla_e == 1
            _ => *self.eth.regs.get(&(0x1_0000 | (page as u32) << 8 | reg as u32)).unwrap_or(&0) as u16,
        }
    }

    fn eth_write(&mut self, off: u32, val: u32) {
        let off = off & !3;
        // MDIO transaction: writing MACMDIOAR with MB performs a PHY read/write.
        if off == MACMDIOAR && val & 1 != 0 {
            let (goc, rda) = ((val >> 2) & 3, ((val >> 16) & 0x1F) as u8);
            if goc == 0b11 { // read -> latch the result into MACMDIODR
                let v = self.phy_read(self.eth.mdio_page, rda);
                if crate::dbg::mdio() {
                    eprintln!("[mdio] RD page={} reg={} -> {:#06x}", self.eth.mdio_page, rda, v);
                }
                self.eth.regs.insert(MACMDIODR, v as u32);
            } else if goc == 0b01 { // write
                let data = self.eth_reg(MACMDIODR) as u16;
                if rda == 31 { self.eth.mdio_page = data; } // PAGE select
                else {
                    if crate::dbg::mdio() {
                        eprintln!("[mdio] WR page={} reg={} <- {:#06x}", self.eth.mdio_page, rda, data);
                    }
                    self.eth.regs.insert(0x1_0000 | (self.eth.mdio_page as u32) << 8 | rda as u32, data as u32);
                }
            }
            self.eth.regs.insert(MACMDIOAR, val & !1); // MB clears (op complete)
            return;
        }
        match off {
            DMACSR => {
                // Write-1-to-clear: clear the interrupt bits the driver acks.
                let cur = self.eth_reg(DMACSR);
                self.eth.regs.insert(DMACSR, cur & !val);
            }
            _ => { self.eth.regs.insert(off, val); }
        }
        // Writing the TX tail pointer hands new descriptors to the DMA -> transmit.
        if off == DMACTXDTPR && !self.rec { self.eth_tx_walk(); }
    }

    /// Walk the TX descriptor ring from `tx_next`, emitting every descriptor the
    /// driver owns (OWN bit set), and hand ownership back. Mirrors the Synopsys
    /// DMA: tdes0=buffer addr, tdes2=length, tdes3 bit31=OWN.
    fn eth_tx_walk(&mut self) {
        let base = self.eth_reg(DMACTXDLAR);
        let ring_len = (self.eth_reg(DMACTXRLR) & 0xFFFF) + 1;
        let mut sent = false;
        // Scan the whole ring in order, emitting every descriptor the driver owns
        // (OWN set) and handing it back. Scanning all (rather than tracking a
        // position) is robust to the VLAN layout's 2-descriptors-per-slot
        // indexing, and guarantees `can_send()` sees a free TX ring afterwards.
        for i in 0..ring_len {
            let d = base.wrapping_add(i.wrapping_mul(16));
            let tdes3 = self.read32(d + 12);
            if tdes3 & (1 << 31) == 0 { continue; } // driver owns it: nothing to send
            self.write32(d + 12, tdes3 & !(1 << 31)); // clear OWN — back to the driver
            // VLAN context descriptor (CTXT bit30): captures the VID the MAC will
            // insert into the following packet. Not a packet itself — don't emit.
            if tdes3 & (1 << 30) != 0 {
                if tdes3 & (1 << 16) != 0 { self.eth.pending_vid = Some((tdes3 & 0xFFF) as u16); }
                continue;
            }
            let buf = self.read32(d);
            let len = (self.read32(d + 8) & 0x3FFF).min(BUFSZ);
            let mut frame = Vec::with_capacity(len as usize + 4);
            for i in 0..len { frame.push(self.read8(buf.wrapping_add(i))); }
            // The MAC inserts the 802.1Q tag from the context descriptor.
            if let Some(vid) = self.eth.pending_vid.take() {
                if frame.len() >= 12 {
                    let tag = [(VLAN_TPID >> 8) as u8, VLAN_TPID as u8, (vid >> 8) as u8, vid as u8];
                    frame.splice(12..12, tag);
                }
            }
            self.eth.tx_frames.push(frame);
            sent = true;
        }
        if sent {
            let csr = self.eth_reg(DMACSR) | (1 << 0) | (1 << 15); // TI | NIS
            self.eth.regs.insert(DMACSR, csr);
            self.eth.regs.insert(DMAISR, self.eth_reg(DMAISR) | 1); // dc0is
            self.eth.irq = true;
        }
    }

    /// Drain frames the SP has transmitted (for the host bridge to forward).
    pub fn eth_take_tx(&mut self) -> Vec<Vec<u8>> { std::mem::take(&mut self.eth.tx_frames) }

    /// Write a humility-`hydrate`-compatible raw memory dump into `dir`: each RAM
    /// region as `0x<base>.bin` plus a `dump.json`. Flash (0x08000000) is omitted
    /// because hydrate reconstructs it from the Hubris archive. Zip the dir, then
    ///   humility -a <archive> hydrate <zip>  &&  humility -d <core> tasks
    /// reads the live (possibly wedged) task table/ringbufs off the emulated SP
    /// with no probe or gdb attach.
    pub fn write_hydrate_dump(&self, dir: &str, archive_id: &str) -> std::io::Result<()> {
        std::fs::create_dir_all(dir)?;
        for r in &self.rams {
            if r.base == 0x0800_0000 { continue; } // flash comes from the archive
            std::fs::write(format!("{}/0x{:x}.bin", dir, r.base), &r.data)?;
        }
        let json = format!(
            "{{\"format\":1,\"task_index\":0,\"crash_time\":0,\"board_name\":\"sidecar-c\",\"git_commit\":\"emu\",\"archive_id\":\"{}\",\"fw_version\":null}}",
            archive_id);
        std::fs::write(format!("{}/dump.json", dir), json)
    }

    /// True if the SP has queued one or more frames for transmit but the serve
    /// loop hasn't flushed them to the bridge yet. The serve loop polls this to
    /// break its instruction batch as soon as a reply is ready, so a round-trip
    /// costs ~one small quantum instead of a full batch (the eth latency that,
    /// under rack contention, exceeds MGS's per-attempt budget).
    pub fn eth_has_tx(&self) -> bool { !self.eth.tx_frames.is_empty() }

    /// Glue between the ETH DMA and the host network bridge: forward transmitted
    /// frames out, and inject any frames the bridge has for us. Called
    /// periodically from the run/gdb loops.
    pub fn pump_eth(&mut self, host: &mut dyn crate::host::HostIo) {
        for f in self.eth_take_tx() { host.eth_tx(&f); }
        // Drain the host network into the bridge once, then deliver only as many
        // frames as the RX ring can accept right now. Checking ring space before
        // popping means a frame is never lost to a full ring — it stays queued in
        // the bridge (governed by the bridge's flow-fair backlog cap) and is
        // delivered on a later pump once the SP frees a descriptor.
        host.eth_poll();
        while self.eth_rx_has_space() {
            match host.eth_rx() {
                Some(f) => { self.eth_rx_inject(&f); }
                None => break,
            }
        }
    }

    /// True if the next RX descriptor is owned by the DMA (free for the engine to
    /// write) — i.e. the ring can accept one more frame. Mirrors the OWN-bit check
    /// in `eth_rx_inject` so the pump can gate delivery without popping a frame.
    fn eth_rx_has_space(&mut self) -> bool {
        let base = self.eth_reg(DMACRXDLAR);
        if base == 0 { return false; }
        let d = base.wrapping_add(self.eth.rx_next.wrapping_mul(16));
        self.read32(d + 12) & (1 << 31) != 0 // RDES3.OWN set -> DMA owns it -> free
    }

    /// Inject a received frame into the next free RX descriptor and raise the
    /// ETH IRQ. Returns false (frame dropped) if the RX ring is full.
    pub fn eth_rx_inject(&mut self, frame: &[u8]) -> bool {
        let base = self.eth_reg(DMACRXDLAR);
        if base == 0 { return false; }
        let ring_len = (self.eth_reg(DMACRXRLR) & 0xFFFF) + 1;
        let ring_dbg = crate::dbg::rx();
        let d = base.wrapping_add(self.eth.rx_next.wrapping_mul(16));
        let rdes3 = self.read32(d + 12);
        if rdes3 & (1 << 31) == 0 { // driver still owns it -> ring full (DMA can't write)
            if ring_dbg {
                eprintln!("[rx-drop] ring full: rx_next={} ringlen={} d={:#x} rdes3={:#x} (OWN clear) cyc={}",
                    self.eth.rx_next, ring_len, d, rdes3, self.cur_cyc);
            }
            return false;
        }
        if ring_dbg {
            eprintln!("[rx-ok] rx_next={} ringlen={} d={:#x} cyc={}", self.eth.rx_next, ring_len, d, self.cur_cyc);
        }
        // The bridge supplies a tagged wire frame. The MAC strips the 802.1Q tag
        // and reports the VID in RDES0 (RS0V); net drops frames lacking a valid
        // VID, and reads the untagged frame from the buffer.
        let tagged = frame.len() >= 16 && u16::from_be_bytes([frame[12], frame[13]]) == VLAN_TPID;
        let vid = if tagged { u16::from_be_bytes([frame[14], frame[15]]) & 0xFFF } else { 0 };
        let untagged: Vec<u8>;
        let data: &[u8] = if tagged {
            untagged = [&frame[..12], &frame[16..]].concat();
            &untagged
        } else { frame };
        if crate::dbg::eth() {
            eprintln!("[eth-rx] inject {} bytes vid={:#x} cyc={} d={:#x} untagged={}",
                data.len(), vid, self.cur_cyc, d, data.iter().map(|b| format!("{:02x}", b)).collect::<String>());
        }
        let buf = self.read32(d);
        let len = (data.len() as u32).min(BUFSZ);
        for i in 0..len { self.write8(buf.wrapping_add(i), data[i as usize]); }
        // Write back: clear OWN, FD|LD, RS0V (RDES0 valid), length; RDES0 = VID.
        let wb = (1 << 29) | (1 << 28) | (1 << 25) | (len & 0x7FFF);
        self.write32(d, vid as u32);
        self.write32(d + 12, wb);
        self.eth.rx_next = (self.eth.rx_next + 1) % ring_len;
        let csr = self.eth_reg(DMACSR) | (1 << 6) | (1 << 15); // RI | NIS
        self.eth.regs.insert(DMACSR, csr);
        self.eth.regs.insert(DMAISR, self.eth_reg(DMAISR) | 1); // dc0is
        self.eth.irq = true;
        true
    }

    // ---- NVIC (interrupt controller) -------------------------------------
    // The NVIC register banks (0xE000E100..0xE000E4EF) live in the Bus rather
    // than the SCS device because interrupt delivery is a CPU-level concern: the
    // kernel writes ISER/ICER (enable) and IPR (priority) via sys_irq_control,
    // peripherals set pending, and the CPU's `maybe_interrupt` reads both.

    fn nvic_read(&self, addr: u32) -> Option<u32> {
        match addr {
            0xE000_E100..=0xE000_E11C => Some(self.nvic_en[((addr - 0xE000_E100) / 4) as usize]),
            0xE000_E180..=0xE000_E19C => Some(self.nvic_en[((addr - 0xE000_E180) / 4) as usize]),
            0xE000_E200..=0xE000_E21C => Some(self.nvic_pend[((addr - 0xE000_E200) / 4) as usize]),
            0xE000_E280..=0xE000_E29C => Some(self.nvic_pend[((addr - 0xE000_E280) / 4) as usize]),
            0xE000_E300..=0xE000_E31C => Some(0), // IABR (active): report none
            0xE000_E400..=0xE000_E4EF => {
                let i = (addr - 0xE000_E400) as usize;
                let mut v = 0;
                for b in 0..4 { v |= (self.nvic_prio[i + b] as u32) << (8 * b); }
                Some(v)
            }
            _ => None,
        }
    }

    fn nvic_write(&mut self, addr: u32, val: u32) -> bool {
        match addr {
            0xE000_E100..=0xE000_E11C => self.nvic_en[((addr - 0xE000_E100) / 4) as usize] |= val,
            0xE000_E180..=0xE000_E19C => self.nvic_en[((addr - 0xE000_E180) / 4) as usize] &= !val,
            0xE000_E200..=0xE000_E21C => self.nvic_pend[((addr - 0xE000_E200) / 4) as usize] |= val,
            0xE000_E280..=0xE000_E29C => self.nvic_pend[((addr - 0xE000_E280) / 4) as usize] &= !val,
            0xE000_E400..=0xE000_E4EF => {
                let i = (addr - 0xE000_E400) as usize;
                for b in 0..4 { self.nvic_prio[i + b] = (val >> (8 * b)) as u8; }
            }
            _ => return false,
        }
        true
    }

    /// Pend an external NVIC IRQ (used by the sprot bridge to wake the RoT's
    /// FLEXCOMM8 slave on a chip-select assert).
    pub fn pend_irq(&mut self, irq: u16) {
        self.nvic_pend[(irq / 32) as usize] |= 1 << (irq % 32);
    }


    /// Bridge the host-sp-comms UART (UART7) to the host: drain the SP's TX queue
    /// out to the host, then feed any host input into the SP's RX queue. Mirrors
    /// `pump_eth` and is called on the same cadence. The RX IRQ is raised in
    /// `collect_irqs` (level-triggered while `uart_rx` is non-empty).
    pub fn pump_uart(&mut self, host: &mut dyn crate::host::HostIo) {
        loop {
            let b = self.uart_tx.borrow_mut().pop_front();
            match b {
                Some(byte) => host.host_uart_tx(byte),
                None => break,
            }
        }
        while let Some(byte) = host.host_uart_rx() {
            self.uart_rx.borrow_mut().push_back(byte);
        }
    }

    pub fn collect_irqs(&mut self) {
        // Poll devices only if one was accessed since the last collect — no modeled
        // device raises an IRQ without an MMIO access, so this is exact, not lossy,
        // and skips ~15-20 take_irq() calls on every compute-only instruction.
        if self.dev_touched {
            self.dev_touched = false;
            let mut raised = [0u16; 8];
            let mut n = 0;
            for d in self.devs.iter_mut() {
                if let Some(irq) = d.dev.take_irq() {
                    if n < raised.len() { raised[n] = irq; n += 1; }
                }
            }
            for &irq in &raised[..n] {
                self.nvic_pend[(irq / 32) as usize] |= 1 << (irq % 32);
            }
        }
        // The Ethernet DMA (modeled in the Bus) raises its IRQ on TX/RX completion.
        if self.eth.irq {
            self.eth.irq = false;
            self.nvic_pend[(ETH_IRQ / 32) as usize] |= 1 << (ETH_IRQ % 32);
        }
        // UART7 (host-sp-comms) RX: like the eth DMA, host input is asynchronous
        // (not gated by the dev-touched poll). Keep IRQ 82 pending while a host
        // byte waits — level-triggered, matching the H7 FIFO RXFNE the task
        // enables; NVIC enable gates actual delivery, and the task's ISR drains
        // RDR (popping `uart_rx`), which clears it.
        if !self.uart_rx.borrow().is_empty() {
            self.nvic_pend[(UART7_IRQ / 32) as usize] |= 1 << (UART7_IRQ % 32);
        }
    }

    /// The lowest-numbered enabled+pending IRQ (priority-aware delivery is gated
    /// by the CPU via `irq_prio`). Returns the IRQ number, not the vector.
    pub fn next_irq(&self) -> Option<u16> {
        for i in 0..8 {
            let active = self.nvic_en[i] & self.nvic_pend[i];
            if active != 0 {
                return Some((i as u16) * 32 + active.trailing_zeros() as u16);
            }
        }
        None
    }

    pub fn irq_prio(&self, irq: u16) -> u8 { self.nvic_prio[irq as usize] }
    pub fn clear_pending(&mut self, irq: u16) {
        self.nvic_pend[(irq / 32) as usize] &= !(1 << (irq % 32));
    }

    pub fn add_ram(&mut self, base: u32, size: u32) {
        self.rams.push(Ram { base, data: vec![0u8; size as usize] });
    }

    pub fn add_device(&mut self, base: u32, size: u32, dev: Box<dyn Mmio>) {
        self.devs.push(Device { base, size, dev });
    }

    fn ram_for(&mut self, addr: u32, len: u32) -> Option<(&mut Ram, usize)> {
        for r in self.rams.iter_mut() {
            let end = r.base.wrapping_add(r.data.len() as u32);
            if addr >= r.base && addr.wrapping_add(len) <= end {
                let off = (addr - r.base) as usize;
                return Some((r, off));
            }
        }
        None
    }

    fn dev_for(&mut self, addr: u32) -> Option<(&mut Device, u32)> {
        for i in 0..self.devs.len() {
            let (base, size) = (self.devs[i].base, self.devs[i].size);
            if addr >= base && addr < base.wrapping_add(size) {
                self.mmio_hit = true;
                self.dev_touched = true;
                return Some((&mut self.devs[i], addr - base));
            }
        }
        None
    }

    /// Bulk-load bytes into a backing RAM/flash region (used by the image loader).
    pub fn load(&mut self, addr: u32, bytes: &[u8]) -> Result<()> {
        if let Some((r, off)) = self.ram_for(addr, bytes.len() as u32) {
            r.data[off..off + bytes.len()].copy_from_slice(bytes);
            Ok(())
        } else {
            bail!("load: no region covers {:#010x}..{:#010x}", addr, addr as usize + bytes.len());
        }
    }

    pub fn read32(&mut self, addr: u32) -> u32 {
        if (NVIC_LO..NVIC_HI).contains(&addr) {
            if let Some(v) = self.nvic_read(addr & !3) { self.mmio_hit = true; return v; }
        }
        if addr & !3 == SCB_ICSR {
            self.mmio_hit = true;
            return (self.pend_pendsv as u32) << 28; // ICSR.PENDSVSET
        }
        if (ETH_BASE..ETH_END).contains(&addr) {
            self.mmio_hit = true;
            return self.eth_read(addr - ETH_BASE);
        }
        if let Some((r, off)) = self.ram_for(addr, 4) {
            u32::from_le_bytes(r.data[off..off + 4].try_into().unwrap())
        } else if let Some((d, off)) = self.dev_for(addr) {
            d.dev.read(off & !3)
        } else {
            self.unmapped_reads += 1;
            if self.log_unmapped { eprintln!("[mem] unmapped read32  @ {:#010x}", addr); }
            0
        }
    }

    pub fn read16(&mut self, addr: u32) -> u16 {
        if let Some((r, off)) = self.ram_for(addr, 2) {
            u16::from_le_bytes(r.data[off..off + 2].try_into().unwrap())
        } else {
            (self.read32(addr & !3) >> (8 * (addr & 2))) as u16
        }
    }

    pub fn read8(&mut self, addr: u32) -> u8 {
        if let Some((r, off)) = self.ram_for(addr, 1) {
            r.data[off]
        } else {
            (self.read32(addr & !3) >> (8 * (addr & 3))) as u8
        }
    }

    pub fn write32(&mut self, addr: u32, val: u32) {
        if let Some(w) = self.watch { if addr == w { eprintln!("[watch] write32 {:#010x} = {:#010x} (pc={:#010x} cyc={})", addr, val, self.cur_pc, self.cur_cyc); } }
        if (NVIC_LO..NVIC_HI).contains(&addr)
            && self.nvic_write(addr & !3, val) { self.mmio_hit = true; return; }
        if addr & !3 == SCB_ICSR {
            if val & (1 << 28) != 0 { self.pend_pendsv = true; }  // PENDSVSET
            if val & (1 << 27) != 0 { self.pend_pendsv = false; } // PENDSVCLR
            self.mmio_hit = true;
            return;
        }
        if (ETH_BASE..ETH_END).contains(&addr) {
            self.mmio_hit = true;
            self.eth_write(addr - ETH_BASE, val);
            return;
        }
        // Record only writes that actually land in RAM (the reference model can't
        // see emu's device/unmapped writes, so replaying them would desync it).
        if self.rec && self.ram_for(addr, 4).is_some() { self.writes.push((addr, val, 4)); }
        if let Some((r, off)) = self.ram_for(addr, 4) {
            r.data[off..off + 4].copy_from_slice(&val.to_le_bytes());
        } else if let Some((d, off)) = self.dev_for(addr) {
            d.dev.write(off & !3, val);
        } else {
            self.unmapped_writes += 1;
            if self.log_unmapped { eprintln!("[mem] unmapped write32 @ {:#010x} = {:#010x}", addr, val); }
        }
    }

    pub fn write16(&mut self, addr: u32, val: u16) {
        if self.rec && self.ram_for(addr, 2).is_some() { self.writes.push((addr, val as u32, 2)); }
        if let Some((r, off)) = self.ram_for(addr, 2) {
            r.data[off..off + 2].copy_from_slice(&val.to_le_bytes());
        } else {
            let sh = 8 * (addr & 2);
            let w = self.read32(addr & !3);
            self.write32(addr & !3, (w & !(0xffffu32 << sh)) | ((val as u32) << sh));
        }
    }

    pub fn write8(&mut self, addr: u32, val: u8) {
        if self.rec && self.ram_for(addr, 1).is_some() { self.writes.push((addr, val as u32, 1)); }
        if let Some((r, off)) = self.ram_for(addr, 1) {
            r.data[off] = val;
        } else {
            let sh = 8 * (addr & 3);
            let w = self.read32(addr & !3);
            self.write32(addr & !3, (w & !(0xffu32 << sh)) | ((val as u32) << sh));
        }
    }
}
