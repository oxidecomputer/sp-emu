//! Flash banks (Slot A / Slot B) backed by a persistent host file: the
//! non-volatile memory of the emulated SP.
//!
//! The STM32H753 has 2 MB of flash in two 1 MB banks: bank 1 at 0x08000000
//! (Slot A) and bank 2 at 0x08100000 (Slot B). Both are modeled as a single 2 MB
//! image persisted to a host file so that, as on real silicon, flash contents
//! persist across runs. Program a slot once, then `run`.

use anyhow::{bail, Context, Result};
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

/// SP UDP socket table (name, port) from the archive's app.toml
/// `[config.net.sockets.*]` sections. None for a raw binary.
pub fn archive_sockets(path: &str) -> Option<Vec<(String, u16)>> {
    let raw = std::fs::read(path).ok()?;
    if !raw.starts_with(b"PK") {
        return None;
    }
    let toml = String::from_utf8(archive_entry(&raw, "app.toml").ok()?).ok()?;
    Some(parse_sockets(&toml))
}

/// Minimal line scan of xtask-generated app.toml: section headers select the
/// socket name; a `port = N` key inside records it.
fn parse_sockets(toml: &str) -> Vec<(String, u16)> {
    let mut out = Vec::new();
    let mut cur: Option<String> = None;
    for line in toml.lines() {
        let t = line.trim();
        if let Some(rest) = t.strip_prefix("[config.net.sockets.") {
            cur = rest.strip_suffix(']').map(str::to_string);
        } else if t.starts_with('[') {
            cur = None;
        } else if let (Some(name), Some((k, v))) = (cur.as_ref(), t.split_once('=')) {
            if k.trim() == "port" {
                if let Ok(p) = v.trim().parse::<u16>() {
                    out.push((name.clone(), p));
                }
            }
        }
    }
    out
}

/// Path of the per-slot socket-table record next to the flash file.
pub fn sockets_path(nvm_path: &str, slot: char) -> String {
    format!("{nvm_path}.{}.sockets.json", slot.to_ascii_lowercase())
}

/// Record the socket table at flash time so well-known-port mode can bind
/// what the image actually serves.
pub fn save_sockets(nvm_path: &str, slot: char, sockets: &[(String, u16)]) -> Result<()> {
    let p = sockets_path(nvm_path, slot);
    let json = serde_json::to_string_pretty(sockets)?;
    std::fs::write(&p, json).with_context(|| format!("write {p}"))
}

pub fn load_sockets(nvm_path: &str, slot: char) -> Option<Vec<(String, u16)>> {
    let text = std::fs::read_to_string(sockets_path(nvm_path, slot)).ok()?;
    serde_json::from_str(&text).ok()
}

/// Program an image into a slot: erase the bank, then write the image at its base.
pub fn program_slot(path: &str, slot: char, image: &[u8]) -> Result<()> {
    let off = slot_offset(slot)?;
    if image.len() > BANK_SIZE {
        bail!("image is {} bytes, exceeds {} KB bank size", image.len(), BANK_SIZE / 1024);
    }
    let mut nvm = load_nvm(path)?;
    nvm[off..off + BANK_SIZE].fill(ERASED);
    nvm[off..off + image.len()].copy_from_slice(image);
    save_nvm(path, &nvm)?;
    Ok(())
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_socket_sections() {
        let toml = r#"
[kernel]
name = "gimlet"

[config.net.sockets.echo]
kind = "udp"
owner = {name = "udpecho", notification = "socket"}
port = 7
tx = { packets = 3, bytes = 1024 }

[config.net.sockets.control_plane_agent]
kind = "udp"
port = 11111

[config.net.sockets.ereport]
port = 57005

[tasks.net]
priority = 3
"#;
        let s = parse_sockets(toml);
        assert_eq!(s, vec![
            ("echo".to_string(), 7),
            ("control_plane_agent".to_string(), 11111),
            ("ereport".to_string(), 57005),
        ]);
    }

    #[test]
    fn non_socket_ports_ignored() {
        let s = parse_sockets("[tasks.net]\nport = 99\n");
        assert!(s.is_empty());
    }
}
