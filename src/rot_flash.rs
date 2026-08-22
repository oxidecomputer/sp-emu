// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! LPC55S69 RoT flash: the flash array plus a real flash-controller command
//! engine, so unmodified RoT Hubris can erase, program, and read its flash the
//! way it does on silicon.
//!
//! Key behaviors modeled (UM11126, lib/lpc55-flash):
//!
//!   * 512-byte pages, 16-byte words, 32 words/page. Command engine at
//!     0x4003_4000: CMD, STARTA/STOPA (word numbers, STOPA inclusive),
//!     DATAW0..3, INT_STATUS (FAIL/ERR/DONE/ECC_ERR).
//!   * Commands Read=3, EraseRange=4, BlankCheck=5, Write=8, Program=12. A page
//!     write is erase-page, then 32x Write (each loads DATAW0..3 into a page
//!     buffer row), then Program (commits the 512-byte page).
//!   * Per-512-byte-page erased tracking (a packed bitset). There is no ECC data;
//!     instead, a ReadSingleWord of a never-programmed page raises ECC_ERR, which
//!     is how the firmware detects erased flash (e.g. an empty scratch CFPA). So
//!     the RoT FWID, taken over the blank-check-programmed pages, is correct.
//!   * The protected flash region assembled from named field templates (real
//!     captured values): CMPA (kept verbatim, including the Bart keyset RKTH),
//!     CFPA ping (a fresh device: version 0, boot preference Slot A, with the
//!     self-hash recomputed from the fields) over a zeroed pong, so ping wins the
//!     version tie, and the captured NMPA pages at their real addresses.
//!
//!     Version 0 is the freshly seeded baseline; a restart loads the persisted
//!     flash and keeps the advanced counter. See demo/README.md for how that
//!     baseline supports rollback and key-revocation testing.
//!   * The active CFPA's persistent boot-preference bit. This is only one input
//!     bootleby uses; running the real bootleby (signature checks on both slots,
//!     the transient RAM boot preference, scratch-CFPA promotion) is future work.
//!
//! The flash contents and erased bitset persist to backing files, like the SP
//! flash. Memory-mapped stores into the flash window are ignored: on the LPC55
//! flash is written only through the command engine, never by a CPU store.

use anyhow::Result;
use std::path::Path;

pub const BASE: u32 = 0x0000_0000;
pub const SIZE: usize = 0x0010_0000; // 1 MB window: flash array plus the protected flash region
pub const PAGE: usize = 512;
pub const WORD: usize = 16;
const WORDS_PER_PAGE: usize = PAGE / WORD; // 32
const NPAGES: usize = SIZE / PAGE; // 2048
pub const ERASED: u8 = 0xFF;

/// RoT Hubris image slot A base (chips/lpc55/memory.toml `flash.a`).
pub const IMAGE_A_BASE: u32 = 0x0001_0000;
/// RoT Hubris image slot B base (chips/lpc55/memory.toml `flash.b`).
pub const IMAGE_B_BASE: u32 = 0x0005_0000;

// Protected flash region page byte-addresses (from the lpc55-pac peripheral
// bases: FLASH_CFPA0 = 0x9_E000, FLASH_CMPA = 0x9_E400, key store 0x9_E600).
const CFPA_SCRATCH: usize = 0x9_DE00;
const CFPA_PING: usize = 0x9_E000;
const CFPA_PONG: usize = 0x9_E200;
const CMPA: usize = 0x9_E400;
const NMPA: usize = 0x9_EC00;
/// NMPA spans ten 512-byte pages, 0x9_EC00..0x9_FFFF. Only pages 0, 1, 8 and 9
/// are programmed on a real part; the rest read as erased (and so fault).
const NMPA_PAGES: usize = 10;
/// 128-bit device UUID, at 0x9_FC70 per UM11126 (NMPA page 8 + 0x70).
const NMPA_UUID: usize = 0x9_FC70;

// Protected-flash pages captured from a real oxide-rot-1 (LPC55S69), with the
// device UUID zeroed and the lot/wafer trace codes replaced: the UUID is filled
// in per instance from `identity::rot_uuid()`, and nothing reads the trace codes.
// Pages 0 and 1 are boot-ROM patch code; 8 and 9 are the manufacturing data.
const NMPA_P0: &[u8] = include_bytes!("../data/lpc55-pfr/nmpa-0.bin");
const NMPA_P1: &[u8] = include_bytes!("../data/lpc55-pfr/nmpa-1.bin");
const NMPA_P8: &[u8] = include_bytes!("../data/lpc55-pfr/nmpa-8.bin");
const NMPA_P9: &[u8] = include_bytes!("../data/lpc55-pfr/nmpa-9.bin");
// include_bytes! takes whatever length the file happens to be, and write_pages
// would silently lay down a short or long region. A re-capture at the wrong size
// should fail the build, not corrupt the protected flash.
const _: () = assert!(NMPA_P0.len() == PAGE);
const _: () = assert!(NMPA_P1.len() == PAGE);
const _: () = assert!(NMPA_P8.len() == PAGE);
const _: () = assert!(NMPA_P9.len() == PAGE);
// The bundled page 8 must ship with its UUID field zeroed: the runtime value is
// written over it, and a re-capture that skipped the scrub would otherwise
// publish a real device's UUID.
const _: () = {
    let uuid_off = NMPA_UUID - (NMPA + 8 * PAGE);
    let mut i = 0;
    while i < 16 {
        assert!(NMPA_P8[uuid_off + i] == 0, "bundled NMPA page 8 has a UUID");
        i += 1;
    }
};

// Command-engine register offsets (from base 0x4003_4000).
const REG_CMD: u32 = 0x00;
const REG_STARTA: u32 = 0x10;
const REG_STOPA: u32 = 0x14;
const REG_DATAW: u32 = 0x80; // DATAW0..7 at 0x80..0x9C; the driver uses 0..3
const REG_INT_STATUS: u32 = 0xFE0;
const REG_INT_CLR_STATUS: u32 = 0xFE8;
const REG_INT_SET_STATUS: u32 = 0xFEC;

// Flash commands (lib/lpc55-flash FlashCmd; UM11126 table 171).
const CMD_READ: u32 = 3;
const CMD_ERASE: u32 = 4;
const CMD_BLANK: u32 = 5;
const CMD_WRITE: u32 = 8;
const CMD_PROGRAM: u32 = 12;

// INT_STATUS bits (lpc55-pac flash::int_status).
const ST_FAIL: u32 = 1 << 0;
const ST_DONE: u32 = 1 << 2;
const ST_ECC: u32 = 1 << 3;

// ---- protected flash region seed templates -----------------------------------
//
// The CMPA and CFPA page layouts come from `lpc55_areas` (the Oxide signing
// tooling's authoritative packed structs), so named fields are set rather than
// hand-rolled byte offsets. Field values are captured from a real grapefruit RoT.
// The CFPA carries a SHA-256 over its own first 480 bytes (byte 0x1E0); to_vec()
// packs but does not seal, so that digest is computed here (as lpc55_sign does).

/// The Bart keyset root-key hash (CMPA `rotkh`). A hash of the public root keys,
/// shared by every Bart-signed device (keyset identity, not device-unique).
const RKTH_BART: [u8; 32] = [
    0x84, 0x33, 0x2e, 0xf8, 0x27, 0x9d, 0xf8, 0x7f, 0xbb, 0x75, 0x9d, 0xc3, 0x86, 0x6c, 0xbc, 0x50,
    0xcd, 0x24, 0x6f, 0xbb, 0x5a, 0x64, 0x70, 0x5a, 0x7e, 0x60, 0xba, 0x86, 0xbf, 0x01, 0xc2, 0x7d,
];
/// DCFG_CC_SOCU debug-configuration word (pin and default hold the same value on
/// this debug-open board).
const DCFG_CC_SOCU: u32 = 0xfd00_02ff;
/// Byte offset of the CFPA's self-SHA-256 (over words 0..29 = bytes 0x000..0x1E0).
const CFPA_HASH_OFF: usize = 0x1E0;

fn to_page(bytes: Vec<u8>) -> [u8; PAGE] {
    bytes
        .try_into()
        .expect("a packed CMPA/CFPA page is exactly 512 bytes")
}

/// CMPA, kept verbatim from a real Bart-signed device: the secure-boot config and
/// the Bart RKTH. Not device-unique, so faithful across the fleet.
fn seed_cmpa() -> [u8; PAGE] {
    let mut cmpa = lpc55_areas::CMPAPage {
        boot_cfg: 0x7800_0080,
        cc_socu_pin: DCFG_CC_SOCU,
        cc_socu_dflt: DCFG_CC_SOCU,
        secure_boot_cfg: 0xc000_0004,
        ..Default::default()
    };
    cmpa.set_rotkh(&RKTH_BART);
    to_page(cmpa.to_vec().expect("pack CMPA"))
}

/// CFPA for a fresh device: version counter at 0, a persistent boot preference
/// (customer_defined0 bit0: clear = Slot A, set = Slot B; offset 0x100, matching
/// bootleby's `boot_flags`), and a valid self-hash recomputed from the fields.
fn seed_cfpa(pref_b: bool) -> [u8; PAGE] {
    use sha2::{Digest, Sha256};
    let mut cfpa = lpc55_areas::CFPAPage {
        version: 0,
        rkth_revoke: 1,
        dcfg_cc_socu_ns_pin: DCFG_CC_SOCU,
        dcfg_cc_socu_ns_dflt: DCFG_CC_SOCU,
        ..Default::default()
    };
    if pref_b {
        cfpa.customer_defined0[0] |= 1;
    }
    let mut page = to_page(cfpa.to_vec().expect("pack CFPA"));
    let digest = Sha256::digest(&page[..CFPA_HASH_OFF]);
    page[CFPA_HASH_OFF..PAGE].copy_from_slice(&digest);
    page
}

/// Load a real 512-byte CMPA/CFPA page from an override file (SP_EMU_ROT_CMPA/CFPA),
/// used to run real bootleby whose PFR validation is stricter than the synthesized
/// pages. Fails loudly on an unreadable file or a wrong-sized page.
fn load_page_override(path: Option<&str>) -> Result<Option<[u8; PAGE]>> {
    let Some(path) = path else {
        return Ok(None);
    };
    let bytes =
        std::fs::read(path).map_err(|e| anyhow::anyhow!("rot CMPA/CFPA override {path}: {e}"))?;
    let page: [u8; PAGE] = bytes.as_slice().try_into().map_err(|_| {
        anyhow::anyhow!(
            "rot CMPA/CFPA override {path}: {} bytes, need {PAGE}",
            bytes.len()
        )
    })?;
    Ok(Some(page))
}

pub struct RotFlash {
    mem: Vec<u8>,
    /// Packed per-page erased bitset (bit set = page never programmed). A read of
    /// an erased page faults (ECC) rather than returning 0xFF.
    erased: Vec<u8>,
    /// The 512-byte program buffer: Write fills a row, Program commits the page.
    page_buf: [u8; PAGE],
    starta: u32,
    stopa: u32,
    dataw: [u32; 8],
    status: u32,
    file: Option<std::fs::File>,
    path: String,
    dbg: bool,
    /// Command codes already reported as unimplemented, one bit per `cmd & 0xF`.
    unknown_cmds_logged: u16,
    /// Set after the first failed erased-bitset write, so a persistently
    /// failing disk logs once rather than per flash operation.
    bitset_write_failed: bool,
}

fn bitset_path(path: &str) -> String {
    format!("{path}.erased")
}

impl RotFlash {
    /// Build the model: load a persisted image + bitset if present, else seed a
    /// fresh device from `image` (into slot A) plus the protected flash region.
    /// `SP_EMU_ROT_FRESH` forces the seed path, ignoring any persisted state.
    pub fn new(path: &str, image: &[u8]) -> Result<RotFlash> {
        let cfg = crate::config::get();
        let mut f = RotFlash {
            mem: vec![ERASED; SIZE],
            erased: vec![0xFF; NPAGES / 8], // every page erased to start
            page_buf: [ERASED; PAGE],
            starta: 0,
            stopa: 0,
            dataw: [0; 8],
            status: 0,
            file: None,
            path: path.to_string(),
            dbg: cfg.rotflashdbg(),
            unknown_cmds_logged: 0,
            bitset_write_failed: false,
        };

        // Persisted state takes precedence over `image` and the CMPA/CFPA/bootleby
        // overrides, so be explicit about which path was taken; a stale backing
        // file can otherwise silently shadow them. Setting SP_EMU_ROT_FRESH
        // removes all doubt by forcing a re-seed.
        let persisted = Path::new(path).exists();
        let use_persisted = persisted && !cfg.rot_fresh();
        if use_persisted {
            // Warn loudly if provisioning overrides are being shadowed by the file.
            if cfg.rot_cmpa().is_some() || cfg.rot_cfpa().is_some() || cfg.rot_bootleby().is_some()
            {
                eprintln!(
                    "[rotflash] WARNING: persisted {path} shadows SP_EMU_ROT_CMPA/CFPA/BOOTLEBY \
                     (ignored). Set SP_EMU_ROT_FRESH=1 or delete the file to apply them."
                );
            }
            let data = std::fs::read(path)
                .map_err(|e| anyhow::anyhow!("reading persisted RoT flash {path}: {e}"))?;
            let n = data.len().min(SIZE);
            f.mem[..n].copy_from_slice(&data[..n]);
            // The erased bitset is load-bearing (blank-check and erased-read
            // ECC behavior); a persisted image without it is corrupt state.
            let bits = std::fs::read(bitset_path(path)).map_err(|e| {
                anyhow::anyhow!(
                    "reading erased bitset {}: {e}; delete {path} or set \
                     SP_EMU_ROT_FRESH=1 to re-seed",
                    bitset_path(path)
                )
            })?;
            let n = bits.len().min(f.erased.len());
            f.erased[..n].copy_from_slice(&bits[..n]);
            eprintln!(
                "[rotflash] loaded persisted RoT flash from {path} (ignoring the passed image)"
            );
        } else {
            if persisted {
                eprintln!("[rotflash] SP_EMU_ROT_FRESH: ignoring persisted {path}, re-seeding");
            }
            eprintln!(
                "[rotflash] seeded fresh RoT flash (slot A image + CMPA/CFPA/NMPA) to {path}"
            );
            f.seed_fresh(image)?;
        }

        // Open the backing file read-write for write-through and seed it.
        f.file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(path)
            .map_err(|e| eprintln!("[rotflash] open {path} failed: {e}"))
            .ok();
        if !use_persisted {
            f.persist_all();
        }
        Ok(f)
    }

    /// Seed a fresh device: the firmware image into slot A, and the protected
    /// flash region (CMPA / CFPA ping+pong / NMPA) from the captured pages. Each
    /// seeded page is marked programmed so its reads do not fault.
    fn seed_fresh(&mut self, image: &[u8]) -> Result<()> {
        let cfg = crate::config::get();
        // Slot A: the passed image, unless asked to leave it erased. An erased slot
        // is bootleby's "empty/invalid", used to drive the B-only / neither cases.
        if cfg.rot_erase_a() {
            eprintln!("[rotflash] SP_EMU_ROT_ERASE_A: leaving slot A erased");
        } else {
            self.write_pages(IMAGE_A_BASE as usize, image);
        }
        // Slot B: a second image if provided, so real bootleby can do genuine A/B
        // selection; absent, slot B stays erased (invalid).
        if let Some(path) = cfg.rot_image_b() {
            let img_b = crate::flash::load_image(path)?;
            eprintln!(
                "[rotflash] seeded slot B ({} bytes) from {path}",
                img_b.len()
            );
            self.write_pages(IMAGE_B_BASE as usize, &img_b);
        }
        // Real device CMPA/CFPA pages if provided (SP_EMU_ROT_CMPA/CFPA, for running
        // real bootleby), else the synthesized pages. The synthesized CFPA's
        // persistent boot preference follows SP_EMU_ROT_BOOT_PREF.
        let cmpa = load_page_override(cfg.rot_cmpa())?.unwrap_or_else(seed_cmpa);
        self.write_pages(CMPA, &cmpa);
        let pref_b = cfg.rot_boot_pref() == Some("b");
        let cfpa = load_page_override(cfg.rot_cfpa())?.unwrap_or_else(|| seed_cfpa(pref_b));
        self.write_pages(CFPA_PING, &cfpa);
        // Pong zeroed, not a copy: on a factory-fresh part the versions tie and
        // bootleby's read_cfpa takes the first page, so ping is unambiguously
        // active. A copy would make "which page won" untestable.
        self.write_pages(CFPA_PONG, &[0u8; PAGE]);
        // NMPA at its real addresses: pages 0/1 (ROM patch code) and 8/9
        // (manufacturing data) are programmed, the rest stay erased exactly as on
        // a real part, where reading them faults. SP_EMU_ROT_NMPA replaces the
        // whole region with a captured one.
        if let Some(path) = cfg.rot_nmpa() {
            let blob = crate::flash::load_image(path)?;
            let n = blob.len().min(NMPA_PAGES * PAGE);
            eprintln!("[rotflash] NMPA from {path} ({n} bytes)");
            self.write_pages(NMPA, &blob[..n]);
        } else {
            for (i, page) in [(0, NMPA_P0), (1, NMPA_P1), (8, NMPA_P8), (9, NMPA_P9)] {
                self.write_pages(NMPA + i * PAGE, page);
            }
        }
        // The per-instance device UUID, so two emulated RoTs are distinguishable
        // where real parts are. Only for the bundled pages, whose UUID field is
        // zeroed: a caller who supplies a capture is reproducing a specific part,
        // and silently substituting its UUID would defeat the point.
        if cfg.rot_nmpa().is_none() {
            let uuid = crate::identity::rot_uuid();
            self.mem[NMPA_UUID..NMPA_UUID + uuid.len()].copy_from_slice(&uuid);
            self.set_erased(NMPA_UUID / PAGE, false);
        }
        // CFPA scratch is left erased.
        // stage0 / stage0next: a synthetic bootleby image carrying just a caboose.
        // Without it every MGS `component/stage0/caboose` read returns NoCaboose and
        // the control plane's inventory retries it every poll. A real bootleby
        // (SP_EMU_ROT_BOOTLEBY) is loaded at 0x0 later and overwrites slot 0.
        let stage0 = crate::lpc55::synthetic_stage0();
        self.write_pages(crate::lpc55::STAGE0_BASE as usize, &stage0);
        self.write_pages(crate::lpc55::STAGE0NEXT_BASE as usize, &stage0);
        Ok(())
    }

    /// Copy `bytes` to `off` and mark every touched 512-byte page programmed.
    fn write_pages(&mut self, off: usize, bytes: &[u8]) {
        let n = bytes.len().min(self.mem.len().saturating_sub(off));
        if n == 0 {
            return;
        }
        self.mem[off..off + n].copy_from_slice(&bytes[..n]);
        for page in (off / PAGE)..=((off + n - 1) / PAGE) {
            self.set_erased(page, false);
        }
    }

    #[inline]
    fn is_erased(&self, page: usize) -> bool {
        debug_assert!(page < NPAGES);
        self.erased[page / 8] & (1 << (page % 8)) != 0
    }
    #[inline]
    fn set_erased(&mut self, page: usize, e: bool) {
        debug_assert!(page < NPAGES);
        let (i, bit) = (page / 8, 1u8 << (page % 8));
        if e {
            self.erased[i] |= bit;
        } else {
            self.erased[i] &= !bit;
        }
    }

    // ---- memory-mapped reads (instruction fetch + data loads) --------------

    #[inline]
    fn read_bytes<const N: usize>(&self, addr: u32) -> [u8; N] {
        let o = addr as usize;
        let mut b = [ERASED; N];
        let n = self.mem.len().saturating_sub(o).min(N);
        b[..n].copy_from_slice(&self.mem[o..o + n]);
        b
    }
    #[inline]
    pub fn read_mem32(&self, addr: u32) -> u32 {
        u32::from_le_bytes(self.read_bytes(addr))
    }
    #[inline]
    pub fn read_mem16(&self, addr: u32) -> u16 {
        u16::from_le_bytes(self.read_bytes(addr))
    }
    #[inline]
    pub fn read_mem8(&self, addr: u32) -> u8 {
        // A single byte cannot straddle the end: the Bus routes only
        // BASE..BASE+SIZE here, so addr < mem.len().
        self.mem[addr as usize]
    }

    /// A CPU store into the flash window. Ignored: LPC55 flash is programmed only
    /// through the command engine, never by a memory-mapped write.
    pub fn write_mem(&mut self, _addr: u32, _val: u32, _size: u8) {}

    /// Load an image span directly (the boot loader path). Marks pages programmed.
    pub fn load_image_at(&mut self, addr: u32, bytes: &[u8]) {
        self.write_pages(addr as usize, bytes);
        self.persist_all();
    }

    // ---- command-engine registers (0x4003_4000) ---------------------------

    pub fn reg_read(&self, off: u32) -> u32 {
        match off {
            REG_STARTA => self.starta,
            REG_STOPA => self.stopa,
            REG_INT_STATUS => self.status,
            o if (REG_DATAW..REG_DATAW + 32).contains(&o) => {
                self.dataw[((o - REG_DATAW) / 4) as usize]
            }
            _ => 0,
        }
    }

    pub fn reg_write(&mut self, off: u32, val: u32) {
        match off {
            REG_CMD => self.do_cmd(val),
            REG_STARTA => self.starta = val,
            REG_STOPA => self.stopa = val,
            REG_INT_CLR_STATUS => self.status &= !val,
            REG_INT_SET_STATUS => self.status |= val,
            o if (REG_DATAW..REG_DATAW + 32).contains(&o) => {
                self.dataw[((o - REG_DATAW) / 4) as usize] = val;
            }
            _ => {}
        }
    }

    fn do_cmd(&mut self, cmd: u32) {
        if self.dbg {
            eprintln!(
                "[rotflash] CMD={cmd} starta={:#x} stopa={:#x}",
                self.starta, self.stopa
            );
        }
        match cmd & 0xF {
            CMD_ERASE => self.cmd_erase(),
            CMD_WRITE => self.cmd_write(),
            CMD_PROGRAM => self.cmd_program(),
            CMD_BLANK => self.cmd_blank(),
            CMD_READ => self.cmd_read(),
            unknown => {
                // Unimplemented command: report failure rather than a fake
                // completion the firmware would take as success.
                if self.unknown_cmds_logged & (1 << unknown) == 0 {
                    self.unknown_cmds_logged |= 1 << unknown;
                    eprintln!("[rotflash] unimplemented CMD={unknown}; reporting FAIL");
                }
                self.status |= ST_DONE | ST_FAIL;
            }
        }
    }

    /// EraseRange: erase every 512-byte page the word range [STARTA, STOPA] spans.
    fn cmd_erase(&mut self) {
        let (lo, hi) = (self.starta as usize * WORD, self.stopa as usize * WORD);
        let (first, last) = (lo / PAGE, hi / PAGE);
        for page in first..=last.min(NPAGES - 1) {
            let base = page * PAGE;
            self.mem[base..base + PAGE].fill(ERASED);
            self.set_erased(page, true);
        }
        self.persist_pages(first, last);
        self.status |= ST_DONE;
    }

    /// Write: load one 16-byte word from DATAW0..3 into page-buffer row STARTA
    /// (0..31). Program then commits the buffer.
    fn cmd_write(&mut self) {
        let row = (self.starta as usize) % WORDS_PER_PAGE;
        for i in 0..4 {
            let b = self.dataw[i].to_le_bytes();
            self.page_buf[row * WORD + i * 4..row * WORD + i * 4 + 4].copy_from_slice(&b);
        }
        self.status |= ST_DONE;
    }

    /// Program: commit the 512-byte page buffer to the page containing STARTA
    /// (NOR: bits only clear), mark it programmed, then reset the buffer.
    fn cmd_program(&mut self) {
        let page = (self.starta as usize * WORD) / PAGE;
        if page < NPAGES {
            let base = page * PAGE;
            for i in 0..PAGE {
                self.mem[base + i] &= self.page_buf[i];
            }
            self.set_erased(page, false);
            self.persist_pages(page, page);
        }
        self.page_buf = [ERASED; PAGE];
        self.status |= ST_DONE;
    }

    /// BlankCheck: FAIL at the first non-0xFF word in [STARTA, STOPA], with its
    /// word number in DATAW0; otherwise the range is blank.
    fn cmd_blank(&mut self) {
        let mut w = self.starta;
        while w <= self.stopa {
            let base = w as usize * WORD;
            // Past the end of flash there is nothing to program, so stop rather
            // than scan on: STOPA is an 18-bit word number, and a range running
            // beyond the window would otherwise spin uselessly to STOPA.
            if base + WORD > self.mem.len() {
                break;
            }
            // "Blank" reflects a page's programmed state, not its byte content: on
            // real flash a programmed word carries ECC, so a page written with 0xFF
            // data still reads as not blank. A pure content check wrongly calls a
            // programmed-but-0xFF page blank (e.g. a firmware image with a 0xFF
            // run), which breaks bootleby's per-page is_programmed scan. sp-emu
            // tracks programmed-ness per page in the erased bitset.
            if !self.is_erased(base / PAGE) {
                self.dataw[0] = w;
                self.status |= ST_DONE | ST_FAIL; // not blank -> FAIL (== success, per UM)
                return;
            }
            w = w.wrapping_add(1);
        }
        self.status |= ST_DONE;
    }

    /// ReadSingleWord: a read of an erased page raises ECC_ERR (the firmware's cue
    /// that the page is unprogrammed); otherwise 16 bytes at STARTA into DATAW0..3.
    fn cmd_read(&mut self) {
        let base = self.starta as usize * WORD;
        let page = base / PAGE;
        if page >= NPAGES || self.is_erased(page) {
            self.status |= ST_DONE | ST_ECC;
            return;
        }
        for k in 0..4 {
            let mut v = 0u32;
            for b in 0..4 {
                v |= (self.mem[base + k * 4 + b] as u32) << (8 * b);
            }
            self.dataw[k] = v;
        }
        self.status |= ST_DONE;
    }

    // ---- persistent boot preference (one input to bootleby's selection) ----

    /// Parse the CFPA page at `page_addr` via the authoritative `lpc55_areas`
    /// layout. An unparseable (e.g. erased) page reads as all-zero fields.
    fn cfpa_at(&self, page_addr: usize) -> lpc55_areas::CFPAPage {
        let page: [u8; PAGE] = self.mem[page_addr..page_addr + PAGE].try_into().unwrap();
        lpc55_areas::CFPAPage::from_bytes(&page).unwrap_or_default()
    }

    /// The active CFPA page: the higher version of ping/pong, ping winning ties.
    fn active_cfpa(&self) -> usize {
        if self.cfpa_at(CFPA_PING).version >= self.cfpa_at(CFPA_PONG).version {
            CFPA_PING
        } else {
            CFPA_PONG
        }
    }

    /// The 512-byte CMPA page, for the boot-ROM signature verifier.
    pub fn cmpa_bytes(&self) -> [u8; PAGE] {
        self.mem[CMPA..CMPA + PAGE].try_into().unwrap()
    }

    /// The active CFPA page (higher ping/pong version) as 512 bytes.
    pub fn active_cfpa_bytes(&self) -> [u8; PAGE] {
        let a = self.active_cfpa();
        self.mem[a..a + PAGE].try_into().unwrap()
    }

    /// Borrow up to `len` bytes of flash starting at `addr` (bounded to the window),
    /// so the ROM shim can hand a slot image to the verifier without copying.
    pub fn slice(&self, addr: u32, len: usize) -> &[u8] {
        let o = (addr as usize).min(self.mem.len());
        let end = o.saturating_add(len).min(self.mem.len());
        &self.mem[o..end]
    }

    /// The persistent boot-preferred slot from the active CFPA (bit0 of the Oxide
    /// boot-preference word, `customer_defined0`: clear = Slot A, set = Slot B).
    /// This is only one of bootleby's inputs. Real bootleby also verifies the
    /// signature of each image (A and B), honors the transient boot preference held
    /// in RAM, and combines all three to pick the slot, or panics/loops if neither
    /// is valid. Running that requires the boot-ROM hash and signature routines and
    /// is separate future work.
    pub fn boot_pref_slot(&self) -> char {
        let cfpa = self.cfpa_at(self.active_cfpa());
        if cfpa.customer_defined0[0] & 1 == 0 {
            'a'
        } else {
            'b'
        }
    }

    // ---- persistence -------------------------------------------------------

    fn persist_pages(&mut self, first: usize, last: usize) {
        use std::io::{Seek, SeekFrom, Write};
        if let Some(f) = self.file.as_mut() {
            let off = first * PAGE;
            let end = ((last + 1) * PAGE).min(self.mem.len());
            let r = f
                .seek(SeekFrom::Start(off as u64))
                .and_then(|_| f.write_all(&self.mem[off..end]));
            if let Err(e) = r {
                eprintln!("[rotflash] write-through to {} failed: {e}", self.path);
                self.file = None;
            }
        }
        self.persist_bitset();
    }

    fn persist_all(&mut self) {
        use std::io::{Seek, SeekFrom, Write};
        if let Some(f) = self.file.as_mut() {
            let r = f
                .seek(SeekFrom::Start(0))
                .and_then(|_| f.write_all(&self.mem))
                .and_then(|_| f.set_len(SIZE as u64));
            if let Err(e) = r {
                eprintln!("[rotflash] seed {} failed: {e}", self.path);
                self.file = None;
            }
        }
        self.persist_bitset();
    }

    /// Persist the erased bitset. It is load-bearing state (blank-check and
    /// erased-read ECC), so a failure is reported, once.
    fn persist_bitset(&mut self) {
        if let Err(e) = std::fs::write(bitset_path(&self.path), &self.erased) {
            if !self.bitset_write_failed {
                self.bitset_write_failed = true;
                eprintln!(
                    "[rotflash] erased-bitset write to {} failed: {e}",
                    bitset_path(&self.path)
                );
            }
        }
    }

    pub fn flush(&mut self) {
        if let Some(f) = self.file.as_mut() {
            if let Err(e) = f.sync_all() {
                eprintln!("[rotflash] sync {} failed: {e}", self.path);
            }
        }
    }
}

/// Default backing-file path for the RoT flash, `$SP_EMU_ROT_NVM` or a default.
pub fn nvm_path() -> String {
    crate::config::instance_file("SP_EMU_ROT_NVM", crate::config::get().rot_nvm_path())
}

pub fn load_image(path: &str) -> Result<Vec<u8>> {
    crate::flash::load_image(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fresh() -> RotFlash {
        // A unique path per call: tests run in parallel, so a shared file would
        // race (one test's persisted image loaded by another).
        use std::sync::atomic::{AtomicU64, Ordering};
        static SEQ: AtomicU64 = AtomicU64::new(0);
        let path = std::env::temp_dir()
            .join(format!(
                "sp-emu-rotflash-{}-{}.bin",
                std::process::id(),
                SEQ.fetch_add(1, Ordering::Relaxed)
            ))
            .to_string_lossy()
            .into_owned();
        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_file(bitset_path(&path));
        // A minimal slot-A image: a plausible vector table so it looks programmed.
        let mut img = vec![0u8; PAGE];
        img[0..4].copy_from_slice(&0x2004_0000u32.to_le_bytes()); // SP into RAM
        img[4..8].copy_from_slice(&(IMAGE_A_BASE + 0x181).to_le_bytes()); // reset PC
        RotFlash::new(&path, &img).unwrap()
    }

    fn cleanup(f: &RotFlash) {
        let _ = std::fs::remove_file(&f.path);
        let _ = std::fs::remove_file(bitset_path(&f.path));
    }

    /// NMPA is seeded at the addresses a real part uses: the programmed pages
    /// carry their captured content, the gap between them stays erased (where a
    /// read faults), and the device UUID lands at the UM11126 address.
    #[test]
    fn nmpa_seeded_at_real_addresses_with_a_device_uuid() {
        let f = fresh();
        assert_eq!(&f.mem[NMPA..NMPA + PAGE], NMPA_P0, "page 0 (ROM patch)");
        assert_eq!(
            &f.mem[NMPA + PAGE..NMPA + 2 * PAGE],
            NMPA_P1,
            "page 1 (ROM patch)"
        );
        // Pages 2..7 are unprogrammed on the sampled part; a read of those faults.
        for page in 2..8 {
            let off = NMPA + page * PAGE;
            assert!(f.is_erased(off / PAGE), "NMPA page {page} must stay erased");
        }
        assert_eq!(
            f.mem[NMPA_UUID..NMPA_UUID + 16],
            crate::identity::rot_uuid(),
            "device UUID at 0x9_FC70 (UM11126)"
        );
        assert!(
            !f.is_erased(NMPA_UUID / PAGE),
            "UUID page reads as programmed"
        );
        cleanup(&f);
    }

    /// Drive the update-server page-write sequence and verify erase, NOR
    /// programming, blank-check, read-back, and the erased-read ECC fault.
    #[test]
    fn command_engine_write_read_erase_ecc() {
        let mut f = fresh();
        // Target an all-erased page well past the seeded image (page at 0x40000).
        let addr = 0x0004_0000usize;
        let word = (addr / WORD) as u32;
        let page = addr / PAGE;
        assert!(f.is_erased(page), "target page starts erased");

        // A ReadSingleWord of the erased page raises ECC_ERR, no data.
        f.reg_write(REG_STARTA, word);
        f.reg_write(REG_INT_CLR_STATUS, 0xF);
        f.reg_write(REG_CMD, CMD_READ);
        assert_ne!(f.reg_read(REG_INT_STATUS) & ST_ECC, 0, "erased read -> ECC");

        // Erase the page (already erased, but drives the path), then program it:
        // 32x Write filling the buffer, then Program.
        f.reg_write(REG_STARTA, word);
        f.reg_write(REG_STOPA, word + (WORDS_PER_PAGE as u32 - 1));
        f.reg_write(REG_INT_CLR_STATUS, 0xF);
        f.reg_write(REG_CMD, CMD_ERASE);
        assert_ne!(f.reg_read(REG_INT_STATUS) & ST_DONE, 0);

        for row in 0..WORDS_PER_PAGE as u32 {
            f.reg_write(REG_STARTA, row);
            for i in 0..4 {
                f.reg_write(REG_DATAW + i * 4, 0x1100_0000 | (row << 4) | i);
            }
            f.reg_write(REG_INT_CLR_STATUS, 0xF);
            f.reg_write(REG_CMD, CMD_WRITE);
        }
        f.reg_write(REG_STARTA, word);
        f.reg_write(REG_INT_CLR_STATUS, 0xF);
        f.reg_write(REG_CMD, CMD_PROGRAM);
        assert!(!f.is_erased(page), "page programmed");

        // ReadSingleWord now returns the programmed row 0 (no ECC).
        f.reg_write(REG_STARTA, word);
        f.reg_write(REG_INT_CLR_STATUS, 0xF);
        f.reg_write(REG_CMD, CMD_READ);
        let st = f.reg_read(REG_INT_STATUS);
        assert_eq!(st & ST_ECC, 0, "programmed read: no ECC");
        assert_eq!(f.reg_read(REG_DATAW), 0x1100_0000, "row 0 word 0");

        // BlankCheck over the page now FAILs at the first word.
        f.reg_write(REG_STARTA, word);
        f.reg_write(REG_STOPA, word + (WORDS_PER_PAGE as u32 - 1));
        f.reg_write(REG_INT_CLR_STATUS, 0xF);
        f.reg_write(REG_CMD, CMD_BLANK);
        assert_ne!(f.reg_read(REG_INT_STATUS) & ST_FAIL, 0, "not blank");
        assert_eq!(f.reg_read(REG_DATAW), word, "first non-blank word");

        // Memory-mapped read sees the programmed bytes too.
        assert_eq!(f.read_mem32(addr as u32), 0x1100_0000);
        cleanup(&f);
    }

    /// The seeded fresh device boots Slot A (CFPA version 0, boot-pref A), and the
    /// active-CFPA selection picks the higher version, ties to ping.
    #[test]
    fn cfpa_selection_and_boot_pref() {
        let mut f = fresh();
        assert_eq!(f.boot_pref_slot(), 'a', "fresh device boots slot A");

        // Make pong the higher version with boot-pref B; it should win.
        let mut c = lpc55_areas::CFPAPage::default();
        c.version = 9;
        c.customer_defined0[0] = 1; // boot preference = Slot B
        f.mem[CFPA_PONG..CFPA_PONG + PAGE].copy_from_slice(&c.to_vec().unwrap());
        assert_eq!(f.active_cfpa(), CFPA_PONG, "higher version wins");
        assert_eq!(f.boot_pref_slot(), 'b', "pong selects slot B");
        cleanup(&f);
    }

    /// CMPA is seeded verbatim (the Bart RKTH survives), and a read of the seeded
    /// CMPA page does not fault.
    #[test]
    fn cmpa_seeded_with_rkth() {
        let f = fresh();
        assert_eq!(
            &f.mem[CMPA..CMPA + 4],
            &0x7800_0080u32.to_le_bytes(),
            "BOOT_CFG"
        );
        assert_eq!(
            &f.mem[CMPA + 0x50..CMPA + 0x70],
            &RKTH_BART,
            "Bart RKTH kept verbatim"
        );
        assert!(!f.is_erased(CMPA / PAGE), "CMPA page programmed");
        cleanup(&f);
    }

    /// The CFPA template's self-SHA-256 (byte 0x1E0, over words 0..29) is valid, so
    /// editing a field and recompiling keeps the page verifiable.
    #[test]
    fn cfpa_self_hash_is_valid() {
        use sha2::{Digest, Sha256};
        let p = seed_cfpa(false);
        let want = Sha256::digest(&p[..CFPA_HASH_OFF]);
        assert_eq!(
            &p[CFPA_HASH_OFF..PAGE],
            &want[..],
            "CFPA self-hash recomputed"
        );
        let c = lpc55_areas::CFPAPage::from_bytes(&p).unwrap();
        assert_eq!(c.version, 0, "version 0");
        assert_eq!(c.customer_defined0[0] & 1, 0, "boot preference Slot A");
    }

    /// The lpc55_areas-built PFR pages are byte-for-byte the pages captured from a
    /// real grapefruit RoT (CMPA verbatim; CFPA reset to a fresh device). Guards
    /// against a field/layout drift in lpc55_areas silently changing the seed.
    #[test]
    fn pfr_pages_match_captured_device() {
        use sha2::{Digest, Sha256};
        assert_eq!(
            Sha256::digest(seed_cmpa())[..],
            hex32("ffb9169cce8f600a47e795a326ef97aa7872ef75bacb02a7ed2b93fdd5451db4")[..],
            "CMPA matches the captured device",
        );
        assert_eq!(
            Sha256::digest(seed_cfpa(false))[..],
            hex32("dc81d4c0429548776a9db89ffc2221725b04b2ed86c036f35e444270a4c7deb5")[..],
            "CFPA (fresh) matches the generated template",
        );
    }

    fn hex32(s: &str) -> [u8; 32] {
        let mut b = [0u8; 32];
        for (i, o) in b.iter_mut().enumerate() {
            *o = u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap();
        }
        b
    }

    /// The image, bitset, and PFR persist across a reload from the backing files.
    #[test]
    fn persists_across_reload() {
        let mut f = fresh();
        let addr = 0x0005_0000usize;
        let page = addr / PAGE;
        // Program one word so there is a durable change to reload.
        let word = (addr / WORD) as u32;
        f.reg_write(REG_STARTA, word);
        for i in 0..4 {
            f.reg_write(REG_DATAW + i * 4, 0xAABB_0000 | i);
        }
        f.reg_write(REG_INT_CLR_STATUS, 0xF);
        f.reg_write(REG_CMD, CMD_WRITE);
        f.reg_write(REG_STARTA, word);
        f.reg_write(REG_INT_CLR_STATUS, 0xF);
        f.reg_write(REG_CMD, CMD_PROGRAM);
        f.flush();

        let path = f.path.clone();
        let f2 = RotFlash::new(&path, &[]).unwrap();
        assert_eq!(
            f2.read_mem32(addr as u32),
            0xAABB_0000,
            "programmed word persists"
        );
        assert!(
            !f2.is_erased(page),
            "written page stays written across reload"
        );
        assert!(!f2.is_erased(CMPA / PAGE), "CMPA stays programmed");
        cleanup(&f2);
    }
}
