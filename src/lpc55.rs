//! LPC55 (Cortex-M33 / ARMv8-M) SoC for the emulated Root-of-Trust, running the
//! `oxide-rot-1` Hubris firmware. Memory map mirrors chips/lpc55/memory.toml.
//! Peripherals are modeled iteratively, driven by the firmware's accesses, the
//! same bring-up method used for the STM32H7 SP (see soc.rs).
use crate::mem::Bus;

/// Hubris RoT image slot A base (chips/lpc55/memory.toml `flash.a`). Skips the
/// LPC55 boot ROM + stage0/bootleby and loads the hubris image directly here.
pub const IMAGE_A_BASE: u32 = crate::rot_flash::IMAGE_A_BASE;

// Bootloader regions from chips/lpc55/memory.toml, seeded with a synthetic
// caboose image when no real bootleby is loaded (see `synthetic_stage0`).
pub const STAGE0_BASE: u32 = 0x0000_0000; // bootleby (slot 0)
pub const STAGE0NEXT_BASE: u32 = 0x0000_2000; // stage0 update staging (slot 1)

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
pub fn synthetic_stage0() -> Vec<u8> {
    let mut tlvc = Vec::new();
    tlvc.extend_from_slice(&crate::soc::tlvc_chunk(b"BORD", b"oxide-rot-1"));
    tlvc.extend_from_slice(&crate::soc::tlvc_chunk(b"NAME", b"bootleby"));
    tlvc.extend_from_slice(&crate::soc::tlvc_chunk(b"GITC", b"0000000000000000000000000000000000000000"));
    tlvc.extend_from_slice(&crate::soc::tlvc_chunk(b"VERS", b"0.0.0-sp-emu"));
    // SIGN: the staging/devel RKTH sp-sim compiles in for its RoT/stage0
    // cabooses, so one update payload repo's artifact tags match both fleets.
    // The update planner string-compares it; nothing verifies it cryptographically.
    tlvc.extend_from_slice(&crate::soc::tlvc_chunk(
        b"SIGN",
        b"11594bb5548a757e918e6fe056e2ad9e084297c9555417a025d8788eacf55daf",
    ));
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
    // The 1 MB flash window (0x0: image slots + the protected flash region) is the
    // `RotFlash` model installed via bus.install_rot_flash() in build_rot_core,
    // and the flash controller at 0x4003_4000 is the same model, not flat RAM or a
    // device here.
    bus.add_ram(0x2000_0000, 0x0004_8000); // Main SRAM (ram + sram2)
    bus.add_ram(0x4010_0000, 0x0000_4000); // USB SRAM: DICE handoff
}

pub fn install_peripherals(bus: &mut Bus) {
    use crate::soc::{RegFile, Scs};
    // ARM System Control Space (SysTick/NVIC/SCB/CPACR/VTOR) — without it
    // maybe_tick reads CSR=0, SysTick never fires, and the kernel can't schedule.
    bus.add_device(0xE000_E000, 0x1000, Box::new(Scs::new()));
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
    if let Some(swd) = crate::rotswd::link() {
        bus.add_device(
            0x4009_6000,
            0x1000,
            Box::new(crate::rotswd::RotSwdSpi::new(swd)),
        );
    }
    // PUF key store (0x4003_B000): the dice-self startup drives GENERATEKEY/GETKEY
    // to derive the RoT's DICE seed, then blocks+locks index 1. Model it so the
    // image generates its own cert handoff (get-certs works). Added before the
    // catch-alls so it owns 0x4003_Bxxx.
    bus.add_device(0x4003_B000, 0x1000, Box::new(crate::puf::Puf::new()));
    // HASHCRYPT SHA-256 engine (0x400A_4000): real bootleby drives it to fold the
    // selected image's measurement into the DICE CDI (sha256::update_cdi). Added
    // before the catch-alls so it owns 0x400A_4xxx rather than reading back 0 from
    // the RegFile, which would spin bootleby forever on STATUS.DIGEST.
    bus.add_device(
        0x400A_4000,
        0x1000,
        Box::new(crate::hashcrypt::HashCrypt::new()),
    );
    // Permissive catch-all for the rest of the peripheral block (SYSCON, IOCON,
    // GPIO, FLEXCOMM, ...), split around the flash controller window.
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
    // DICE CDI: the boot ROM deposits the 256-bit Compound Device Identifier in
    // SYSCON registers at offset 0x900 (8 words). lib_dice::Cdi::from_reg returns
    // None (skipping ALL DICE cert generation) when these are zero. Seed the
    // per-instance CDI (crate::identity), so each sp-emu instance derives a
    // distinct DICE identity and self-signed cert (paired with the per-instance
    // PUF seed for the persistid key). Re-seeded each boot since from_reg zeroizes it.
    // The CDI is a device secret on silicon, but sp-emu's RoT is a deliberate open
    // book; see the "Secrets policy exception" in src/identity.rs.
    for (i, w) in crate::identity::dice_cdi_words().iter().enumerate() {
        bus.write32(0x4000_0900 + (i as u32) * 4, *w);
    }
    // (PUF at 0x4003_B000 is modeled by crate::puf::Puf, added above; the RoT's
    // dice-self startup blocks+locks index 1 itself, which puf_check then checks.)
}
