//! LPC55 (Cortex-M33 / ARMv8-M) SoC for the emulated Root-of-Trust, running the
//! `oxide-rot-1` Hubris firmware. Memory map mirrors chips/lpc55/memory.toml.
//! Peripherals are modeled iteratively, driven by the firmware's accesses, the
//! same bring-up method used for the STM32H7 SP (see soc.rs).
use crate::mem::{Bus, Mmio};

/// Hubris RoT image slot A base (chips/lpc55/memory.toml `flash.a`). Skips the
/// LPC55 boot ROM + stage0/bootleby and loads the hubris image directly here.
pub const IMAGE_A_BASE: u32 = 0x0001_0000;

// Flash regions from chips/lpc55/memory.toml, used to synthesize caboose reads
// for the slots sp-emu doesn't populate (see `synthetic_stage0` / `byte_at`).
const IMAGE_B_BASE: u32 = 0x0005_0000; // hubris slot B
const IMAGE_B_END: u32 = 0x0009_0000;
const STAGE0_BASE: u32 = 0x0000_0000; // bootleby (slot 0)
const STAGE0_END: u32 = 0x0000_2000;
const STAGE0NEXT_BASE: u32 = 0x0000_2000; // stage0 update staging (slot 1)
const STAGE0NEXT_END: u32 = 0x0000_4000;

// sys/abi ImageHeader + caboose magics, matched by lpc55-update-server's
// `caboose_slice`: it reads the ImageHeader at region+0x130, then locates the
// caboose at the tail (last word = size, first word = magic).
const HEADER_MAGIC: u32 = 0x64CE_D6CA;
const CABOOSE_MAGIC: u32 = 0xCAB0_005E;
const HEADER_OFFSET: usize = 0x130;

/// Build a minimal stage0/bootleby image that `caboose_slice` accepts: a valid
/// `ImageHeader` at 0x130 plus a TLV-C caboose at the tail. sp-emu skips real
/// stage0, so without this every MGS `component/stage0/caboose` read returns
/// NoCaboose; the control plane's inventory retries the failures every poll,
/// pegging the emulated RoT. The keys are placeholders (read-only inventory
/// doesn't validate them). Serving it for both stage0 and stage0next mirrors a
/// device whose bootloader banks hold the same image.
fn synthetic_stage0() -> Vec<u8> {
    let mut tlvc = Vec::new();
    tlvc.extend_from_slice(&crate::soc::tlvc_chunk(b"BORD", b"oxide-rot-1"));
    tlvc.extend_from_slice(&crate::soc::tlvc_chunk(b"NAME", b"bootleby"));
    tlvc.extend_from_slice(&crate::soc::tlvc_chunk(b"GITC", b"0000000000000000000000000000000000000000"));
    tlvc.extend_from_slice(&crate::soc::tlvc_chunk(b"VERS", b"0.0.0-sp-emu"));
    // Caboose blob at the image tail: [MAGIC(4)] [tlvc] [size(4)]; `caboose_size`
    // is the whole blob length. Keep `image_end` well within the 0x2000 region.
    let caboose_size = (4 + tlvc.len() + 4) as u32;
    let image_end: u32 = 0x0800;
    let caboose_start = (image_end - caboose_size) as usize;
    let mut buf = vec![0xFFu8; image_end as usize];
    buf[HEADER_OFFSET..HEADER_OFFSET + 4].copy_from_slice(&HEADER_MAGIC.to_le_bytes());
    buf[HEADER_OFFSET + 4..HEADER_OFFSET + 8].copy_from_slice(&image_end.to_le_bytes()); // total_image_len
    buf[caboose_start..caboose_start + 4].copy_from_slice(&CABOOSE_MAGIC.to_le_bytes());
    buf[caboose_start + 4..caboose_start + 4 + tlvc.len()].copy_from_slice(&tlvc);
    buf[(image_end - 4) as usize..image_end as usize].copy_from_slice(&caboose_size.to_le_bytes());
    buf
}

pub fn install_memory(bus: &mut Bus) {
    bus.add_ram(0x0000_0000, 0x0010_0000); // Flash window (ROM alias + stage0 + image a/b)
    bus.add_ram(0x2000_0000, 0x0004_8000); // Main SRAM (ram + sram2)
    bus.add_ram(0x4010_0000, 0x0000_4000); // USB SRAM: DICE handoff (zeroed; real DICE = phase 2)
}

pub fn install_peripherals(bus: &mut Bus, image: &[u8]) {
    use crate::soc::{RegFile, Scs};
    // ARM System Control Space (SysTick/NVIC/SCB/CPACR/VTOR) — without it
    // maybe_tick reads CSR=0, SysTick never fires, and the kernel can't schedule.
    bus.add_device(0xE000_E000, 0x1000, Box::new(Scs::new()));
    // LPC55 flash controller (0x40034000): blank-check + read-word so
    // lpc55-rot-startup's FlashSlot::new can find the programmed image span.
    // Added before the catch-all so it owns 0x40034xxx.
    bus.add_device(0x4003_4000, 0x1000, Box::new(LpcFlash::new(image.to_vec(), IMAGE_A_BASE, synthetic_stage0())));
    // Sprot bridge endpoints on the RoT side: the FLEXCOMM8 SPI slave (0x4009F000,
    // chip.toml [flexcomm8]) and the GPIO block (0x4008C000, chip.toml [gpio]) that
    // carries ROT_IRQ (P0_18, RoT->SP) and CHIP_SELECT (P1_1, SP->RoT). These MUST
    // be added before the catch-all RegFiles below: dev_for() returns the first
    // device whose range covers an address, and the lpc55-periph-hi catch-all
    // (0x40035000..0x40100000) otherwise swallows both ranges, leaving the RoT's
    // SSA/SSD reads and rot-irq writes hitting a dead store/return stub, so it
    // never sees a request and never signals a reply. Gated on the bridge being
    // enabled so the standalone `sp-emu rot` mode (no link) is unaffected.
    if let Some(lk) = crate::sprot::link() {
        bus.add_device(
            0x4009_F000,
            0x1000,
            Box::new(crate::sprot::RotSpiSlave::new(lk.clone())),
        );
        bus.add_device(
            0x4008_C000,
            0x4000,
            Box::new(crate::sprot::LpcGpio::new(lk)),
        );
    }
    // FLEXCOMM5 SPI: the block the RoT clocks SWD through to drive the SP's debug
    // port. The granted address for this build (per `humility map`) is 0x40096000,
    // NOT the datasheet's 0x4009A000. Added before the catch-alls, like sprot.
    bus.add_device(
        0x4009_6000,
        0x1000,
        Box::new(crate::rotswd::RotSwdSpi::new()),
    );
    // Permissive catch-all for the rest of the peripheral block (SYSCON, IOCON,
    // GPIO, FLEXCOMM, HASHCRYPT...), split around the flash controller window.
    bus.add_device(
        0x4000_0000,
        0x0003_4000,
        Box::new(RegFile::new("lpc55-periph-lo")),
    );
    bus.add_device(
        0x4003_5000,
        0x000C_B000,
        Box::new(RegFile::new("lpc55-periph-hi")),
    );
    // SYSCON non-secure alias (0x50000000). lpc55-rot-startup reads SYSCON_DIEID at
    // 0x50000FFC and panics unless (val & 1) == ROM_VER (1). Without this the whole
    // region is unmapped, the read returns 0, and the RoT wedges in a panic spin
    // very early in boot (before any task runs). Model it and seed the DIEID.
    bus.add_device(
        0x5000_0000,
        0x1000,
        Box::new(RegFile::new("lpc55-syscon-ns")),
    );
    bus.write32(0x5000_0FFC, 0x0000_0001); // SYSCON_DIEID: bit0 = ROM version 1
    // get_clock_speed() asserts the FRO-96MHz config the ROM/stage0 leaves:
    // SYSCON MAINCLKSELA(0x280)=3, MAINCLKSELB(0x284)=0, AHBCLKDIV(0x380)=0.
    bus.write32(0x4000_0280, 3);
    bus.write32(0x4000_0284, 0);
    bus.write32(0x4000_0380, 0);
    // PUF (base 0x4003_B000): lpc55-rot-startup::puf_check panics unless KEY_INDEX
    // (1) is blocked and the register is locked in IDXBLK_L (offset 0x20C). The
    // boot ROM normally configures this; emulate the result. Value bits: bit2 =
    // index-1 disabled/blocked, bits[31:30]=0b01 = Locked (lpc55-puf is_index_blocked
    // checks bit index*2; is_locked checks idxblk >> 30 == 1).
    bus.write32(0x4003_B20C, 0x4000_0004);
}

/// Minimal LPC55 flash controller model. The flash content is the memory-mapped
/// RAM at 0x0; this models the command/status path so blank-check (programmed vs
/// erased detection) and single-word reads complete. Registers: CMD@0x00,
/// STARTA@0x10, STOPA@0x14, DATAW0..3@0x80, INT_STATUS@0xFE0 (FAIL=bit0, DONE=bit2),
/// INT_CLR_STATUS@0xFE8. Commands: ReadSingleWord=3, BlankCheck=5.
pub struct LpcFlash {
    img: Vec<u8>,
    base: u32,
    stage0: Vec<u8>, // synthetic stage0/bootleby image (caboose only)
    starta: u32,
    stopa: u32,
    status: u32,
    dataw: [u32; 4],
}
impl LpcFlash {
    pub fn new(img: Vec<u8>, base: u32, stage0: Vec<u8>) -> Self {
        LpcFlash { img, base, stage0, starta: 0, stopa: 0, status: 0, dataw: [0; 4] }
    }
    fn byte_at(&self, word: u32, i: u32) -> u8 {
        let addr = word.wrapping_mul(16).wrapping_add(i);
        // stage0 (slot 0) and stage0next (slot 1): the synthetic bootleby image,
        // served position-relative so its caboose is found in either region. Real
        // stage0 isn't loaded, so these would otherwise read erased and NoCaboose.
        if addr < STAGE0_END {
            return self.stage0.get(addr as usize).copied().unwrap_or(0xFF);
        }
        if addr >= STAGE0NEXT_BASE && addr < STAGE0NEXT_END {
            return self.stage0.get((addr - STAGE0NEXT_BASE) as usize).copied().unwrap_or(0xFF);
        }
        // Hubris slot A (the running image).
        if addr >= self.base {
            let o = (addr - self.base) as usize;
            if o < self.img.len() {
                return self.img[o];
            }
        }
        // Hubris slot B: mirror slot A so the inactive-bank caboose read succeeds
        // instead of NoCaboose-storming (sp-emu only programs one slot).
        if addr >= IMAGE_B_BASE && addr < IMAGE_B_END {
            let o = (addr - IMAGE_B_BASE) as usize;
            if o < self.img.len() { return self.img[o]; }
        }
        0xFF // outside the programmed image = erased flash
    }
    fn do_cmd(&mut self, cmd: u32) {
        const FAIL: u32 = 1 << 0;
        const DONE: u32 = 1 << 2;
        // Skip the BlankCheck (5) flood the startup does scanning for the image;
        // only ReadSingleWord (3) etc. on the update_server path matters.
        if crate::sprot::dbg() && cmd != 5 {
            eprintln!(
                "[flash] CMD={} starta={:#x} stopa={:#x}",
                cmd, self.starta, self.stopa
            );
        }
        match cmd {
            5 => {
                // BlankCheck: FAIL (not blank) at the first non-0xFF word.
                let mut hit = None;
                let mut w = self.starta;
                while w <= self.stopa {
                    if (0..16).any(|i| self.byte_at(w, i) != 0xFF) {
                        hit = Some(w);
                        break;
                    }
                    w = w.wrapping_add(1);
                    if w == 0 {
                        break;
                    }
                }
                if let Some(w) = hit {
                    self.dataw[0] = w;
                    self.status |= DONE | FAIL;
                } else {
                    self.status |= DONE;
                }
            }
            3 => {
                // ReadSingleWord: 16 bytes at STARTA into DATAW0..3.
                for k in 0..4 {
                    let mut v = 0u32;
                    for b in 0..4 {
                        v |= (self.byte_at(self.starta, (k * 4 + b) as u32) as u32) << (8 * b);
                    }
                    self.dataw[k] = v;
                }
                self.status |= DONE;
            }
            _ => {
                self.status |= DONE;
            } // erase/write/program: report done
        }
    }
}
impl Mmio for LpcFlash {
    fn name(&self) -> &str {
        "LPC55-FLASH"
    }
    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            0x10 => self.starta,
            0x14 => self.stopa,
            0x80 => self.dataw[0],
            0x84 => self.dataw[1],
            0x88 => self.dataw[2],
            0x8C => self.dataw[3],
            0xFE0 => self.status,
            _ => 0,
        }
    }
    fn write(&mut self, off: u32, val: u32) {
        match off & !3 {
            0x00 => self.do_cmd(val & 0xF),
            0x10 => self.starta = val,
            0x14 => self.stopa = val,
            0xFE8 => self.status &= !val, // INT_CLR_STATUS
            _ => {}
        }
    }
}
