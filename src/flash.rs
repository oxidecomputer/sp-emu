//! Flash banks (Slot A / Slot B) backed by a persistent host file: the
//! non-volatile memory of the emulated SP.
//!
//! The STM32H753 has 2 MB of flash in two 1 MB banks: bank 1 at 0x08000000
//! (Slot A) and bank 2 at 0x08100000 (Slot B). Both are modeled as a single 2 MB
//! image persisted to a host file so that, as on real silicon, flash contents
//! persist across runs. Program a slot once, then `run`.

use anyhow::{bail, Context, Result};
use std::collections::HashMap;
use std::path::Path;

pub const FLASH_BASE: u32 = 0x0800_0000;
pub const BANK_SIZE: usize = 0x10_0000; // 1 MB per bank
pub const TOTAL: usize = 2 * BANK_SIZE; // 2 MB across both banks
pub const ERASED: u8 = 0xFF; // erased flash reads as all-ones

/// Byte offset of a slot within the 2 MB image, and its absolute base address.
pub fn slot_offset(slot: char) -> Result<usize> {
    match slot.to_ascii_lowercase() {
        'a' => Ok(0),
        'b' => Ok(BANK_SIZE),
        _ => bail!("slot must be 'a' or 'b' (got {slot:?})"),
    }
}

pub fn slot_base(slot: char) -> Result<u32> {
    Ok(FLASH_BASE + slot_offset(slot)? as u32)
}

/// Load the persistent flash image, or a fully-erased image if none exists yet.
pub fn load_nvm(path: &str) -> Result<Vec<u8>> {
    if Path::new(path).exists() {
        let mut data = std::fs::read(path).with_context(|| format!("read {path}"))?;
        data.resize(TOTAL, ERASED);
        Ok(data)
    } else {
        Ok(vec![ERASED; TOTAL])
    }
}

pub fn save_nvm(path: &str, data: &[u8]) -> Result<()> {
    std::fs::write(path, data).with_context(|| format!("write {path}"))
}

/// Load a flashable image from either a raw flat binary or a Hubris build
/// archive (a zip containing `img/final.bin`, the same artifact `humility -a`
/// consumes, produced by `cargo xtask dist`). The archive's entries are bzip2-
/// compressed, so read via the `zip` crate rather than hand-extraction.
pub fn load_image(path: &str) -> Result<Vec<u8>> {
    let raw = std::fs::read(path).with_context(|| format!("read {path}"))?;
    if raw.starts_with(b"PK") {
        archive_entry(&raw, "img/final.bin")
            .context("no img/final.bin in archive — is this a Hubris build archive?")
    } else {
        Ok(raw)
    }
}

/// Extract one entry from a (possibly bzip2-compressed) zip archive.
fn archive_entry(zip_bytes: &[u8], name: &str) -> Result<Vec<u8>> {
    use std::io::Read;
    let mut zip = zip::ZipArchive::new(std::io::Cursor::new(zip_bytes))?;
    let mut f = zip.by_name(name)?;
    let mut buf = Vec::with_capacity(f.size() as usize);
    f.read_to_end(&mut buf)?;
    Ok(buf)
}

/// Best-effort read of `img/flash.ron` (the slot layout) for reporting; returns
/// None for a raw binary or if the entry is absent.
pub fn archive_flash_ron(path: &str) -> Option<String> {
    let raw = std::fs::read(path).ok()?;
    if !raw.starts_with(b"PK") {
        return None;
    }
    String::from_utf8(archive_entry(&raw, "img/flash.ron").ok()?).ok()
}

/// The image's `app.toml` from a Hubris build archive (root entry). Carries the
/// firmware's `[config.net.sockets.*]` table, which the well-known-port bridge
/// uses to bind exactly the SP UDP sockets this image declares.
pub fn archive_app_toml(path: &str) -> Option<String> {
    let raw = std::fs::read(path).ok()?;
    if !raw.starts_with(b"PK") {
        return None;
    }
    String::from_utf8(archive_entry(&raw, "app.toml").ok()?).ok()
}

/// Program an image into a slot: erase the bank, then write the image at its base.
pub fn program_slot(path: &str, slot: char, image: &[u8]) -> Result<()> {
    let off = slot_offset(slot)?;
    if image.len() > BANK_SIZE {
        bail!(
            "image is {} bytes, exceeds {} KB bank size",
            image.len(),
            BANK_SIZE / 1024
        );
    }
    let mut nvm = load_nvm(path)?;
    nvm[off..off + BANK_SIZE].fill(ERASED);
    nvm[off..off + image.len()].copy_from_slice(image);
    save_nvm(path, &nvm)?;
    Ok(())
}

/// If exactly one slot is programmed, mirror it into the other bank in-memory.
///
/// sp-emu programs a single slot, but the control-plane inventory (wicketd)
/// reads the caboose of BOTH banks every poll cycle (~10s per SP). An empty bank
/// returns NoCaboose, which wicketd never caches and re-fetches forever - and
/// each fetch is a ~38ms emulated MGS round-trip, so a real dual-banked SP's
/// harmless polling becomes a continuous CPU drain here. Presenting the same
/// image in both banks (as a real SP has) makes the inactive-slot caboose read
/// succeed and get cached, ending the retries. Only the caboose is read from the
/// inactive bank, so a byte copy suffices - that image is never executed there.
pub fn mirror_unprogrammed_slot(nvm: &mut [u8]) {
    let a = slot_programmed(nvm, 'a').unwrap_or(false);
    let b = slot_programmed(nvm, 'b').unwrap_or(false);
    match (a, b) {
        (true, false) => nvm.copy_within(0..BANK_SIZE, BANK_SIZE),
        (false, true) => nvm.copy_within(BANK_SIZE..TOTAL, 0),
        _ => {}
    }
}

pub fn erase_slot(path: &str, slot: char) -> Result<()> {
    let off = slot_offset(slot)?;
    let mut nvm = load_nvm(path)?;
    nvm[off..off + BANK_SIZE].fill(ERASED);
    save_nvm(path, &nvm)
}

/// Is a slot programmed (i.e. does it hold a plausible vector table, not just
/// erased 0xFF)? Checks the initial stack pointer points into RAM.
pub fn slot_programmed(nvm: &[u8], slot: char) -> Result<bool> {
    let off = slot_offset(slot)?;
    let sp = u32::from_le_bytes(nvm[off..off + 4].try_into().unwrap());
    // RAM lives at 0x20000000.. on the STM32H7; erased flash gives 0xFFFFFFFF.
    Ok(sp != 0xFFFF_FFFF && (0x2000_0000..0x4000_0000).contains(&sp))
}

// ---------------------------------------------------------------------------
// Non-volatile register state file
// ---------------------------------------------------------------------------
//
// The flash *contents* live in the raw `sp-flash.bin` image (physical layout:
// bank1 then bank2). A handful of option-byte bits that survive across a run are
// not flash data — chiefly the persisted bank-swap selection — so they live in a
// tiny plaintext state file (`sp-flash.bin.nv`, one `key = value` per line). It
// is deliberately human-inspectable and forward-extensible (Phase 2 adds RoT
// lines): deleting the state file resets the NV registers to their defaults.

/// Persisted non-volatile controller state (the STM32H7 option bytes we model).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NvState {
    /// `OPTSR_CUR.SWAP_BANK_OPT` — which physical bank boots at 0x0800_0000.
    pub swap_bank: bool,
}

pub fn nv_state_path(flash_path: &str) -> String {
    format!("{flash_path}.nv")
}

/// Load the NV state file, or defaults (all false) if missing / empty.
pub fn load_nv(path: &str) -> NvState {
    let mut nv = NvState::default();
    let Ok(text) = std::fs::read_to_string(path) else {
        return nv;
    };
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let Some((k, v)) = line.split_once('=') else {
            continue;
        };
        let (k, v) = (k.trim(), v.trim());
        if k == "swap_bank" {
            nv.swap_bank = matches!(v, "1" | "true");
        }
    }
    nv
}

pub fn save_nv(path: &str, nv: &NvState) -> Result<()> {
    let body = format!(
        "# sp-emu flash non-volatile registers (see src/flash.rs)\nswap_bank = {}\n",
        nv.swap_bank as u8
    );
    std::fs::write(path, body).with_context(|| format!("write {path}"))
}

// ---------------------------------------------------------------------------
// Runtime flash model (STM32H753 embedded flash + FLASH controller)
// ---------------------------------------------------------------------------
//
// Replaces the flat `add_ram` flash window and the store/return FLASH RegFile
// with a model that behaves like real silicon so unmodified Hubris firmware can
// perform an in-band MGS firmware update end to end: unlock, whole-bank erase
// (with the EOP interrupt the driver blocks on), 256-bit word programming with
// NOR semantics, the option-byte bank swap, and persistence to the host file.
//
// Owned directly by the `Bus` (like the Ethernet DMA), because the data stores
// that program flash target the *memory aperture* (0x0800_0000/0x0810_0000), not
// the register block, and must be gated by the controller's lock/PG state — only
// a single owner of both can coordinate that. Performance: the aperture is the
// hottest path in the emulator (every instruction fetch and constant load), so
// reads are a range check + one XOR (bank remap) + a slice read; write-back to
// disk is deferred (only on erase, reset, and exit), never per programmed word.

// FLASH controller register offsets (from base 0x5200_2000). The update server
// runs from bank1 and programs bank2, so it drives the bank2 register set plus
// the global option-byte registers.
const REG_OPTKEYR: u32 = 0x08;
const REG_OPTCR: u32 = 0x18;
const REG_OPTSR_CUR: u32 = 0x1C;
const REG_OPTSR_PRG: u32 = 0x20;
const REG_OPTCCR: u32 = 0x24;
const REG_KEYR2: u32 = 0x104;
const REG_CR2: u32 = 0x10C;
const REG_SR2: u32 = 0x110;
const REG_CCR2: u32 = 0x114;

// CR bits.
const CR_LOCK: u32 = 1 << 0;
const CR_PG: u32 = 1 << 1;
const CR_BER: u32 = 1 << 3;
const CR_START: u32 = 1 << 7;
const CR_EOPIE: u32 = 1 << 16;
// SR bits (BSY/QW read 0 — the model completes instantly; EOP latches).
const SR_EOP: u32 = 1 << 16;
// OPTCR bits.
const OPTCR_OPTLOCK: u32 = 1 << 0;
const OPTCR_OPTSTART: u32 = 1 << 1;
const OPT_SWAP_BANK: u32 = 1 << 31; // OPTCR.SWAP_BANK / OPTSR.SWAP_BANK_OPT

// Unlock key sequences (RM0433 §4.9.2/§4.9.3).
const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;
const OPTKEY1: u32 = 0x0819_2A3B;
const OPTKEY2: u32 = 0x4C5D_6E7F;

/// The FLASH NVIC line the STM32H7 update server blocks on for erase completion
/// (`chips/stm32h7/chip.toml`: flash_controller irq = 4).
pub const FLASH_IRQ: u16 = 4;

pub struct Flash {
    /// 2 MB physical: `[0..BANK_SIZE]` = bank1, `[BANK_SIZE..TOTAL]` = bank2.
    mem: Vec<u8>,
    /// Write-through handle to the backing file. Every program/erase writes just
    /// the changed bytes here (seek + write), so the flash image stays in sync
    /// even if the process is killed without a clean exit — matching real flash,
    /// where a program/erase is durable the moment it completes. `None` if the
    /// file could not be opened (the model still works in RAM).
    file: Option<std::fs::File>,
    path: String,
    nv_path: String,

    /// Effective bank swap (`OPTCR.SWAP_BANK`): drives the aperture; latched from
    /// `committed` only at reset, so the running image is never remapped mid-run.
    effective_swap: bool,
    /// Committed option-byte swap (`OPTSR_CUR.SWAP_BANK_OPT`): the NV value.
    committed_swap: bool,
    /// Staged option-byte swap (`OPTSR_PRG.SWAP_BANK_OPT`): what OPTSTART commits.
    staged_swap: bool,

    bank2_locked: bool,
    opt_locked: bool,
    key2_state: u8,          // progress through the 2-write KEYR2 unlock sequence
    optkey_state: u8,        // progress through the 2-write OPTKEYR unlock sequence
    cr2: u32,                // last CR2 write (PG/PSIZE/SNB/IE bits read back)
    sr2: u32,                // EOP + error bits (BSY/QW always read 0)
    erase_irq: bool,         // an erase completed with EOPIE armed -> pend FLASH_IRQ
    regs: HashMap<u32, u32>, // ACR, bank1 stubs, other store/return registers
    dbg: bool,               // $SP_EMU_FLASHDBG: trace controller register traffic
}

impl Flash {
    /// Build from a 2 MB image (`load_nvm`) and the NV state file (`load_nv`).
    pub fn new(path: &str, mut image: Vec<u8>, nv: NvState) -> Flash {
        image.resize(TOTAL, ERASED);
        // Open the backing file read-write for write-through, and seed it with the
        // full image so it holds the complete 2 MB (the file may not pre-exist, or
        // may be stale); subsequent program/erase only overwrite changed bytes.
        let mut file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(path)
            .map_err(|e| eprintln!("[flash] open {path} for write-through failed: {e}"))
            .ok();
        if let Some(f) = file.as_mut() {
            use std::io::{Seek, SeekFrom, Write};
            let r = f
                .seek(SeekFrom::Start(0))
                .and_then(|_| f.write_all(&image))
                .and_then(|_| f.set_len(TOTAL as u64));
            if let Err(e) = r {
                eprintln!("[flash] seed {path} failed: {e}");
                file = None;
            }
        }
        Flash {
            mem: image,
            file,
            path: path.to_string(),
            nv_path: nv_state_path(path),
            effective_swap: nv.swap_bank,
            committed_swap: nv.swap_bank,
            staged_swap: nv.swap_bank,
            bank2_locked: true,
            opt_locked: true,
            key2_state: 0,
            optkey_state: 0,
            cr2: 0,
            sr2: 0,
            erase_irq: false,
            regs: HashMap::new(),
            dbg: crate::config::get().flashdbg,
        }
    }

    /// Which physical byte an aperture address maps to, honoring the effective
    /// swap. `BANK_SIZE` is exactly bit 20, so swap is a single XOR over the whole
    /// 2 MB window: bank1<->bank2 at 0x0800_0000<->0x0810_0000.
    #[inline]
    fn phys_off(&self, addr: u32) -> usize {
        let rel = (addr - FLASH_BASE) as usize;
        if self.effective_swap {
            rel ^ BANK_SIZE
        } else {
            rel
        }
    }

    /// Read `N` bytes from the aperture, tolerating an access that straddles the
    /// top of the 2 MB window: bytes past the end read as erased flash (0xFF).
    /// The `mem.rs` dispatch only range-checks the base address, so a stray or
    /// unaligned access in the last few bytes must not panic — the emulator's
    /// contract is to keep the trace moving on out-of-range accesses, not fault.
    #[inline]
    fn read_bytes<const N: usize>(&self, addr: u32) -> [u8; N] {
        let o = self.phys_off(addr);
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
        // A single byte cannot straddle the end: the base range check guarantees
        // phys_off(addr) < mem.len().
        self.mem[self.phys_off(addr)]
    }

    /// A firmware store into the flash aperture. Honored only while bank2 is
    /// unlocked and CR2.PG is set (a real program cycle); NOR semantics — a
    /// program can only clear bits (`&=`), never set them. Anything else is
    /// dropped, matching hardware where a stray store to XIP flash faults / is
    /// ignored rather than mutating it.
    pub fn write_mem(&mut self, addr: u32, val: u32, size: u8) {
        if self.bank2_locked || (self.cr2 & CR_PG) == 0 {
            return;
        }
        let o = self.phys_off(addr);
        let mut n = 0;
        for i in 0..size as usize {
            // A program that runs off the end of flash writes nothing more,
            // rather than panicking on a boundary-straddling store.
            if o + i >= self.mem.len() {
                break;
            }
            self.mem[o + i] &= (val >> (8 * i)) as u8;
            n += 1;
        }
        self.write_through(o, n);
    }

    /// Raw image load at boot (bypasses lock/PG) — used by `Bus::load`. An
    /// oversized image is clamped to the window rather than panicking.
    pub fn load_image_at(&mut self, addr: u32, bytes: &[u8]) {
        let o = self.phys_off(addr);
        let n = self.mem.len().saturating_sub(o).min(bytes.len());
        self.mem[o..o + n].copy_from_slice(&bytes[..n]);
        self.write_through(o, n);
    }

    /// Write the `mem[off..off+len]` window straight through to the backing file
    /// at the same physical offset, so a program/erase is durable immediately
    /// (real flash is non-volatile the moment the operation completes). Only the
    /// changed bytes are written, so per-word programming stays cheap.
    fn write_through(&mut self, off: usize, len: usize) {
        use std::io::{Seek, SeekFrom, Write};
        if len == 0 {
            return;
        }
        if let Some(f) = self.file.as_mut() {
            let r = f
                .seek(SeekFrom::Start(off as u64))
                .and_then(|_| f.write_all(&self.mem[off..off + len]));
            if let Err(e) = r {
                eprintln!("[flash] write-through to {} failed: {e}", self.path);
                self.file = None; // stop retrying a broken handle
            }
        }
    }

    pub fn reg_read(&self, off: u32) -> u32 {
        match off {
            REG_OPTCR => {
                let mut v = 0;
                if self.opt_locked {
                    v |= OPTCR_OPTLOCK;
                }
                if self.effective_swap {
                    v |= OPT_SWAP_BANK;
                }
                if self.dbg {
                    eprintln!("[flashdbg] rd OPTCR={v:#010x} eff_swap={}", self.effective_swap);
                }
                v
            }
            REG_OPTSR_CUR => {
                // OPT_BUSY reads 0 (instant). SWAP_BANK_OPT = committed value.
                let v = if self.committed_swap { OPT_SWAP_BANK } else { 0 };
                if self.dbg {
                    eprintln!("[flashdbg] rd OPTSR_CUR={v:#010x} committed={}", self.committed_swap);
                }
                v
            }
            REG_OPTSR_PRG => {
                let v = if self.staged_swap { OPT_SWAP_BANK } else { 0 };
                if self.dbg {
                    eprintln!("[flashdbg] rd OPTSR_PRG={v:#010x} staged={}", self.staged_swap);
                }
                v
            }
            REG_CR2 => {
                let mut v = self.cr2;
                if self.bank2_locked {
                    v |= CR_LOCK;
                }
                v
            }
            REG_SR2 => self.sr2, // BSY/QW = 0; EOP + errors as latched
            _ => self.regs.get(&off).copied().unwrap_or(0),
        }
    }

    /// Write a FLASH controller register. Returns true if an erase just completed
    /// with its interrupt armed, so the Bus should pend FLASH_IRQ.
    pub fn reg_write(&mut self, off: u32, val: u32) {
        match off {
            REG_KEYR2 => {
                // Two-write unlock: KEY1 then KEY2 clears the bank2 lock.
                match (self.key2_state, val) {
                    (0, KEY1) => self.key2_state = 1,
                    (1, KEY2) => {
                        self.bank2_locked = false;
                        self.key2_state = 0;
                        if self.dbg {
                            eprintln!("[flashdbg] bank2 unlocked");
                        }
                    }
                    _ => self.key2_state = 0,
                }
            }
            REG_OPTKEYR => match (self.optkey_state, val) {
                (0, OPTKEY1) => self.optkey_state = 1,
                (1, OPTKEY2) => {
                    self.opt_locked = false;
                    self.optkey_state = 0;
                    if self.dbg {
                        eprintln!("[flashdbg] option bytes unlocked");
                    }
                }
                _ => self.optkey_state = 0,
            },
            REG_CR2 => {
                self.cr2 = val & !CR_LOCK; // LOCK is a command bit, not stored
                if val & CR_LOCK != 0 {
                    self.bank2_locked = true;
                }
                // Bank erase: BER + START, on an unlocked bank.
                if val & CR_BER != 0 && val & CR_START != 0 && !self.bank2_locked {
                    self.erase_bank2();
                    self.sr2 |= SR_EOP;
                    if val & CR_EOPIE != 0 {
                        self.erase_irq = true;
                    }
                }
            }
            REG_SR2 => self.sr2 = val,
            REG_CCR2 => self.sr2 &= !val, // W1C: clear EOP / error bits
            REG_OPTSR_PRG => {
                self.staged_swap = val & OPT_SWAP_BANK != 0;
                self.regs.insert(off, val);
                if self.dbg {
                    eprintln!("[flashdbg] wr OPTSR_PRG={val:#010x} -> staged={}", self.staged_swap);
                }
            }
            REG_OPTCR => {
                if self.dbg {
                    eprintln!(
                        "[flashdbg] wr OPTCR={val:#010x} (OPTSTART={} OPTLOCK={} opt_locked={})",
                        val & OPTCR_OPTSTART != 0,
                        val & OPTCR_OPTLOCK != 0,
                        self.opt_locked
                    );
                }
                if val & OPTCR_OPTLOCK != 0 {
                    self.opt_locked = true;
                }
                // OPTSTART commits the staged option bytes to the current/NV copy.
                // It does *not* change the effective mapping — only a reset latches
                // that — so the running image is not remapped underfoot.
                if val & OPTCR_OPTSTART != 0 && !self.opt_locked {
                    self.committed_swap = self.staged_swap;
                    if self.dbg {
                        eprintln!("[flashdbg] OPTSTART commit -> committed={}", self.committed_swap);
                    }
                    let nv = NvState {
                        swap_bank: self.committed_swap,
                    };
                    if let Err(e) = save_nv(&self.nv_path, &nv) {
                        // A failed persist means the committed swap is lost across
                        // the next run — surface it rather than silently dropping.
                        eprintln!("[flash] persist option bytes to {} failed: {e}", self.nv_path);
                    }
                }
            }
            REG_OPTCCR => {}
            _ => {
                self.regs.insert(off, val);
            }
        }
    }

    fn erase_bank2(&mut self) {
        // The update server always programs the *inactive* physical bank, reached
        // via the 0x0810_0000 aperture — which the same XOR maps to the right
        // physical half regardless of the effective swap.
        let base = self.phys_off(FLASH_BASE + BANK_SIZE as u32);
        self.mem[base..base + BANK_SIZE].fill(ERASED);
        self.write_through(base, BANK_SIZE);
    }

    /// Consume a pending erase-completion interrupt (pend FLASH_IRQ once).
    pub fn take_erase_irq(&mut self) -> bool {
        std::mem::take(&mut self.erase_irq)
    }

    pub fn effective_swap(&self) -> bool {
        self.effective_swap
    }

    /// Force the boot bank selection (a CLI slot override). Sets all three swap
    /// bits in memory but does not touch the NV state file — only a firmware OPTSTART
    /// commit persists a swap, so a `run a`/`run b` choice never clobbers the
    /// recorded option bytes.
    pub fn force_swap(&mut self, swap: bool) {
        self.effective_swap = swap;
        self.committed_swap = swap;
        self.staged_swap = swap;
    }

    /// Latch the effective mapping from the committed option byte. Called at every
    /// reset edge — this is where a committed bank swap actually takes effect.
    pub fn reset_latch(&mut self) {
        self.effective_swap = self.committed_swap;
    }

    /// Sync the backing file to disk. Program/erase already write through, so this
    /// only forces the OS to flush its buffers; called at reset and on exit.
    pub fn flush(&mut self) {
        if let Some(f) = self.file.as_mut() {
            let _ = f.sync_all();
        }
    }
}
