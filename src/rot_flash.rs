//! LPC55S69 RoT flash: the flash array plus a real flash-controller command
//! engine, so unmodified RoT Hubris can erase, program, and read its flash the
//! way it does on silicon. Replaces the flat `add_ram` window and the read-only
//! `LpcFlash` stub (which acknowledged erase/program but changed nothing).
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
//!     CFPA ping/pong (a fresh device: version 0, boot preference Slot A, with the
//!     self-hash recomputed from the fields), and an NMPA page.
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
pub const SIZE: usize = 0x0010_0000; // the 1 MB window the add_ram used to cover
pub const PAGE: usize = 512;
pub const WORD: usize = 16;
const WORDS_PER_PAGE: usize = PAGE / WORD; // 32
const NPAGES: usize = SIZE / PAGE; // 2048
pub const ERASED: u8 = 0xFF;

/// RoT Hubris image slot A base (chips/lpc55/memory.toml `flash.a`).
pub const IMAGE_A_BASE: u32 = 0x0001_0000;

// Protected flash region page byte-addresses (from the lpc55-pac peripheral
// bases: FLASH_CFPA0 = 0x9_E000, FLASH_CMPA = 0x9_E400, key store 0x9_E600).
const CFPA_SCRATCH: usize = 0x9_DE00;
const CFPA_PING: usize = 0x9_E000;
const CFPA_PONG: usize = 0x9_E200;
const CMPA: usize = 0x9_E400;
const NMPA: usize = 0x9_EC00;

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
// tooling's authoritative packed structs), so we set named fields rather than
// hand-rolling byte offsets. Field values are captured from a real grapefruit RoT.
// The CFPA carries a SHA-256 over its own first 480 bytes (byte 0x1E0); to_vec()
// packs but does not seal, so we compute that digest here (as lpc55_sign does).

/// The Bart keyset root-key hash (CMPA `rotkh`). A hash of the *public* root keys,
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

/// CFPA for a fresh device: version counter at 0, boot preference Slot A
/// (customer_defined0 bit0 = 0, the default), and a valid self-hash recomputed
/// from the fields.
fn seed_cfpa() -> [u8; PAGE] {
    use sha2::{Digest, Sha256};
    let mut cfpa = lpc55_areas::CFPAPage {
        version: 0,
        rkth_revoke: 1,
        dcfg_cc_socu_ns_pin: DCFG_CC_SOCU,
        dcfg_cc_socu_ns_dflt: DCFG_CC_SOCU,
        ..Default::default()
    };
    let mut page = to_page(cfpa.to_vec().expect("pack CFPA"));
    let digest = Sha256::digest(&page[..CFPA_HASH_OFF]);
    page[CFPA_HASH_OFF..PAGE].copy_from_slice(&digest);
    page
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
}

fn bitset_path(path: &str) -> String {
    format!("{path}.erased")
}

impl RotFlash {
    /// Build the model: load a persisted image + bitset if present, else seed a
    /// fresh device from `image` (into slot A) plus the protected flash region.
    pub fn new(path: &str, image: &[u8]) -> RotFlash {
        let dbg = std::env::var("SP_EMU_ROTFLASHDBG").is_ok();
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
            dbg,
        };

        // A persisted image takes precedence over `image`, so say which path was
        // taken: a stale backing file silently shadowing a freshly passed image is
        // otherwise a confusing surprise.
        let bin_exists = Path::new(path).exists();
        if bin_exists {
            eprintln!(
                "[rotflash] loaded persisted RoT flash from {path} (ignoring the passed image)"
            );
            if let Ok(data) = std::fs::read(path) {
                let n = data.len().min(SIZE);
                f.mem[..n].copy_from_slice(&data[..n]);
            }
            if let Ok(bits) = std::fs::read(bitset_path(path)) {
                let n = bits.len().min(f.erased.len());
                f.erased[..n].copy_from_slice(&bits[..n]);
            }
        } else {
            eprintln!(
                "[rotflash] seeded fresh RoT flash (slot A image + CMPA/CFPA/NMPA) to {path}"
            );
            f.seed_fresh(image);
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
        if !bin_exists {
            f.persist_all();
        }
        f
    }

    /// Seed a fresh device: the firmware image into slot A, and the protected
    /// flash region (CMPA / CFPA ping+pong / NMPA) from the captured pages. Each
    /// seeded page is marked programmed so its reads do not fault.
    fn seed_fresh(&mut self, image: &[u8]) {
        self.write_pages(IMAGE_A_BASE as usize, image);
        self.write_pages(CMPA, &seed_cmpa());
        let cfpa = seed_cfpa();
        self.write_pages(CFPA_PING, &cfpa);
        self.write_pages(CFPA_PONG, &cfpa);
        // NMPA: a placeholder programmed page so a read does not fault; the real
        // device UUID lives here (spemu-f27.3). CFPA scratch is left erased.
        self.write_pages(NMPA, &[0u8; PAGE]);
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
            _ => self.status |= ST_DONE,
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
            if self.mem[base..base + WORD].iter().any(|&b| b != ERASED) {
                self.dataw[0] = w;
                self.status |= ST_DONE | ST_FAIL;
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

    /// The PERSISTENT boot-preferred slot from the active CFPA (bit0 of the Oxide
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
        let _ = std::fs::write(bitset_path(&self.path), &self.erased);
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
        let _ = std::fs::write(bitset_path(&self.path), &self.erased);
    }

    pub fn flush(&mut self) {
        if let Some(f) = self.file.as_mut() {
            let _ = f.sync_all();
        }
    }
}

/// Default backing-file path for the RoT flash, `$SP_EMU_ROT_NVM` or a default.
pub fn nvm_path() -> String {
    std::env::var("SP_EMU_ROT_NVM").unwrap_or_else(|_| "sp-rot-flash.bin".to_string())
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
        RotFlash::new(&path, &img)
    }

    fn cleanup(f: &RotFlash) {
        let _ = std::fs::remove_file(&f.path);
        let _ = std::fs::remove_file(bitset_path(&f.path));
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
        let p = seed_cfpa();
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
            Sha256::digest(seed_cfpa())[..],
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
        let f2 = RotFlash::new(&path, &[]);
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
