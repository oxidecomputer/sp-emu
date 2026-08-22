// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Per-instance device identity for an sp-emu instance.
//!
//! Every identity-bearing value the emulated SP and RoT expose is derived per
//! instance: the STM32H753 96-bit UID (`soc.rs`), the LPC55 DICE CDI
//! (`lpc55.rs`), and the PUF seed / UDS (`puf.rs`). Fixed constants would give
//! every instance the same self-signed DICE certificate and the same UID, which
//! breaks a fleet of emulators and any test that distinguishes instances or
//! discovers them like real hardware.
//!
//! This module derives all per-instance identity from a single 32-byte master
//! seed. Fields are domain-separated: `field = SHA3-256(master || tag)`, truncated
//! to the field width. The seed source (in precedence order) is:
//!
//!   1. an explicit `--seed <source>` on the command line,
//!   2. a persisted identity file (so an instance is stable across runs, and the
//!      separately-launched SP and RoT processes of one instance share it),
//!   3. otherwise a fresh random source, persisted once.
//!
//! A seed source is one of: the reserved word `legacy` (reproduces the previous
//! fixed constants exactly, so a legacy UID/CDI/cert can be recovered); a
//! `0x`-prefixed hex u64 (`--seed 0x1234`); or any other string (hashed). The
//! resolved source is written to the identity file so it round-trips.
//!
//! Not covered here: the SP's Ethernet MAC and serial, which come from the
//! emulated VPD EEPROM (`build_vpd_eeprom` in soc.rs), varied per instance by the
//! bridge-port index. Unifying that with this seed is a follow-up; the `mac`
//! field below is derived for that future use, not yet the active MAC.
//!
//! # Secrets policy exception (deliberate)
//!
//! Like the well-known Bart signing key used for development, sp-emu's RoT is
//! an explicit exception to the rule: "never read, display, or log secrets".
//! Any sp-emu "secrets" are an open book so that developers and tests can
//! inspect and reproduce behavior. The master seed used to derive the PUF/UDS
//! and DICE CDI, is persisted in plaintext and is logged on purpose.
//! There are no real secrets to protect here, and reproducibility/testability
//! take priority. Every place that treats this material as readable carries a
//! note pointing back here.

use anyhow::{Result, bail};
use sha3::{Digest, Sha3_256};
use std::sync::OnceLock;

/// Default identity file path, overridable via `$SP_EMU_IDENTITY`. Co-locating it
/// with a per-instance `$SP_EMU_FLASH` (each in its own directory) is the natural
/// way to run a fleet; two instances sharing one cwd without setting this would
/// share an identity, so a fleet must give each its own path.
fn identity_path() -> String {
    crate::config::instance_file(
        "SP_EMU_IDENTITY",
        crate::config::get().identity_path(),
    )
}

/// Domain tags. Changing a tag changes that field's value for a given seed, so
/// they are part of the stable identity contract. Do not edit casually.
const TAG_SP_UID: &[u8] = b"sp-emu/stm32h753-uid";
const TAG_ROT_UUID: &[u8] = b"sp-emu/lpc55-device-uuid";
const TAG_DICE_CDI: &[u8] = b"sp-emu/lpc55-dice-cdi";
const TAG_PUF_UDS: &[u8] = b"sp-emu/lpc55-puf-uds";
const TAG_MAC: &[u8] = b"sp-emu/vpd-mac-base";

// RFC 4122 version and variant fields (section 4.1.1/4.1.3), used to shape the
// derived RoT UUID as the version 3 UUID the factory programs: byte 6's high
// nibble is the version, byte 8's top two bits are the variant.
const UUID_VERSION_MASK: u8 = 0x0f; // clears byte 6's version nibble
const UUID_VERSION_3: u8 = 0x30; // name-based (v3)
const UUID_VARIANT_MASK: u8 = 0x3f; // clears byte 8's variant bits
const UUID_VARIANT_RFC4122: u8 = 0x80;

/// The reserved seed source that reproduces the previous fixed constants.
const LEGACY_SOURCE: &str = "legacy";

/// The historical fixed identity, kept so `--seed legacy` reproduces the exact
/// pre-identity constants: the same SP UID, DICE CDI, and PUF UDS (hence the same
/// self-signed cert). These were independent constants in soc.rs/lpc55.rs/puf.rs.
const LEGACY_SP_UID_WORDS: [u32; 3] = [0x5350_4D45, 0x2D45_4D55, 0x0000_0001];
const LEGACY_DICE_CDI_WORDS: [u32; 8] = [
    0xc0de_d1ce,
    0x0bad_f00d,
    0x1234_5678,
    0x9abc_def0,
    0x0f1e_2d3c,
    0x4b5a_6978,
    0x8796_a5b4,
    0xc3d2_e1f0,
];
const LEGACY_PUF_UDS: [u8; 32] = [
    0x53, 0x50, 0x2d, 0x45, 0x4d, 0x55, 0x2d, 0x50, 0x55, 0x46, 0x2d, 0x64,
    0x69, 0x63, 0x65, 0x2d, 0x73, 0x65, 0x65, 0x64, 0x2d, 0x76, 0x31, 0x2e,
    0x30, 0x2e, 0x30, 0x2d, 0x21, 0x21, 0x21, 0x21,
];

/// Master seed used for a legacy identity's derived fields (RoT UUID, VPD MAC),
/// which had no historical constant; the SP UID / DICE CDI / PUF UDS are then
/// overridden with the exact old values above.
const LEGACY_MASTER: [u8; 32] = *b"sp-emu-legacy-identity-master-v1";

/// The derived per-instance identity. All fields are little-endian byte strings as
/// the corresponding hardware register would expose them.
#[derive(Clone, Debug)]
pub struct Identity {
    /// STM32H753 96-bit unique device ID (`0x1FF1_E800`, 3 words).
    pub sp_uid: [u8; 12],
    /// LPC55 128-bit device UUID (NMPA / ROM `read_uid`).
    pub rot_uuid: [u8; 16],
    /// LPC55 DICE Compound Device Identifier (SYSCON `0x4000_0900`, 8 words).
    /// Open book; see the module "Secrets policy exception".
    pub dice_cdi: [u8; 32],
    /// PUF-derived device secret (UDS) streamed out of GETKEY. Open book; see
    /// the module "Secrets policy exception".
    pub puf_uds: [u8; 32],
    /// Base MAC address for the VPD EEPROM (locally-administered). VPD modeling is
    /// a follow-up; the value is derived now so it is stable when that lands.
    pub mac: [u8; 6],
}

/// `SHA3-256(master || tag)` truncated into `out` (all fields here are <= 32 B).
fn derive(master: &[u8; 32], tag: &[u8], out: &mut [u8]) {
    debug_assert!(out.len() <= 32);
    let mut h = Sha3_256::new();
    h.update(master);
    h.update(tag);
    let d = h.finalize();
    out.copy_from_slice(&d[..out.len()]);
}

fn words_to_le_bytes<const N: usize, const B: usize>(
    words: [u32; N],
) -> [u8; B] {
    debug_assert_eq!(B, N * 4);
    let mut b = [0u8; B];
    for (i, w) in words.iter().enumerate() {
        b[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
    }
    b
}

impl Identity {
    fn from_master(master: [u8; 32]) -> Identity {
        let mut id = Identity {
            sp_uid: [0; 12],
            rot_uuid: [0; 16],
            dice_cdi: [0; 32],
            puf_uds: [0; 32],
            mac: [0; 6],
        };
        derive(&master, TAG_SP_UID, &mut id.sp_uid);
        derive(&master, TAG_ROT_UUID, &mut id.rot_uuid);
        // Shape the derived bytes as an RFC 4122 version 3 (name-based) UUID,
        // the class NXP programs at the factory: a real oxide-rot-1 reads back
        // e.g. aece12b7-31d8-305c-8bab-5ebc24ac98f0. The derivation is sp-emu's
        // own (seed-based, not NXP's namespace), but anything that parses the
        // version and variant sees a UUID of the same class rather than 16
        // bytes that decode as no valid version at all.
        id.rot_uuid[6] = (id.rot_uuid[6] & UUID_VERSION_MASK) | UUID_VERSION_3;
        id.rot_uuid[8] =
            (id.rot_uuid[8] & UUID_VARIANT_MASK) | UUID_VARIANT_RFC4122;
        derive(&master, TAG_DICE_CDI, &mut id.dice_cdi);
        derive(&master, TAG_PUF_UDS, &mut id.puf_uds);
        derive(&master, TAG_MAC, &mut id.mac);
        // Locally-administered, unicast (bit1 set, bit0 clear); a synthesized
        // MAC, not an OUI-assigned one.
        id.mac[0] = (id.mac[0] | 0x02) & !0x01;
        id
    }

    /// The historical fixed identity: derived UUID/MAC (stable), with the SP UID,
    /// DICE CDI, and PUF UDS overridden to their exact pre-identity constants.
    fn legacy() -> Identity {
        let mut id = Identity::from_master(LEGACY_MASTER);
        id.sp_uid = words_to_le_bytes(LEGACY_SP_UID_WORDS);
        id.dice_cdi = words_to_le_bytes(LEGACY_DICE_CDI_WORDS);
        id.puf_uds = LEGACY_PUF_UDS;
        id
    }

    /// The word (little-endian) at byte offset `off` within the SP UID region.
    pub fn sp_uid_word(&self, off: u32) -> u32 {
        let o = off as usize;
        if o + 4 <= self.sp_uid.len() {
            u32::from_le_bytes(self.sp_uid[o..o + 4].try_into().unwrap())
        } else {
            0x0000_0001 // past the 96-bit UID: a stable non-zero filler
        }
    }

    /// The DICE CDI as 8 little-endian words for the SYSCON handoff registers.
    pub fn dice_cdi_words(&self) -> [u32; 8] {
        let mut w = [0u32; 8];
        for (i, wo) in w.iter_mut().enumerate() {
            *wo = u32::from_le_bytes(
                self.dice_cdi[i * 4..i * 4 + 4].try_into().unwrap(),
            );
        }
        w
    }
}

static IDENTITY: OnceLock<Identity> = OnceLock::new();

/// Build an identity from a seed source string (see the module docs): `legacy`,
/// a `0x`-prefixed hex u64, or any other string (hashed). Errors only on a
/// malformed `0x` seed, so a typo is reported rather than silently misinterpreted.
fn identity_from_source(source: &str) -> Result<Identity> {
    let s = source.trim();
    if s.eq_ignore_ascii_case(LEGACY_SOURCE) {
        eprintln!("[identity] seed = legacy (previous fixed constants)");
        return Ok(Identity::legacy());
    }
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        let n = parse_hex_u64(hex)?;
        // Open book: logging the resolved seed is intentional (see module docs).
        eprintln!("[identity] seed = {n:#018x} ({n})");
        return Ok(Identity::from_master(sha3_master(&n.to_le_bytes())));
    }
    Ok(Identity::from_master(sha3_master(s.as_bytes())))
}

/// Parse a `0x`-less hex string as a u64: 1..=16 hex digits. More than 16 digits
/// overflows a u64, and non-hex characters are invalid; both are hard errors.
fn parse_hex_u64(hex: &str) -> Result<u64> {
    if hex.is_empty() {
        bail!("--seed 0x… is missing its hex value");
    }
    if hex.len() > 16 {
        bail!(
            "--seed 0x{hex} exceeds 64 bits ({} hex digits, max 16)",
            hex.len()
        );
    }
    u64::from_str_radix(hex, 16)
        .map_err(|_| anyhow::anyhow!("--seed 0x{hex} is not valid hexadecimal"))
}

fn sha3_master(bytes: &[u8]) -> [u8; 32] {
    let mut m = [0u8; 32];
    m.copy_from_slice(&Sha3_256::digest(bytes));
    m
}

fn to_hex(bytes: &[u8]) -> String {
    use std::fmt::Write as _;
    let mut s = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        let _ = write!(s, "{b:02x}");
    }
    s
}

/// Read the persisted seed source from the identity file, if present.
fn load_source(path: &str) -> Option<String> {
    let text = std::fs::read_to_string(path).ok()?;
    for line in text.lines() {
        let line = line.trim();
        if let Some(v) = line.strip_prefix("seed") {
            let v = v.trim_start_matches([' ', '=']).trim();
            if !v.is_empty() {
                return Some(v.to_string());
            }
        }
    }
    None
}

fn save_source(path: &str, source: &str) {
    // The seed is persisted in plaintext on purpose; the emulated RoT is an open
    // book (see the module "Secrets policy exception").
    let body = format!(
        "# sp-emu per-instance identity (see src/identity.rs)\n\
         # Delete this file to mint a new identity, or pass --seed to set one.\n\
         # Sources: `legacy`, a 0x-prefixed hex u64, or any string.\n\
         seed = {source}\n",
    );
    if let Err(e) = std::fs::write(path, body) {
        eprintln!("[identity] persist to {path} failed: {e}");
    }
}

/// A fresh random seed source: 64 bits of OS randomness rendered as `0x…`, so it
/// persists and round-trips as an ordinary hex seed. Falls back to `legacy` if
/// `/dev/urandom` is unreadable (never panics at startup).
fn random_source() -> String {
    use std::io::Read;
    let mut b = [0u8; 8];
    match std::fs::File::open("/dev/urandom")
        .and_then(|mut f| f.read_exact(&mut b))
    {
        Ok(()) => format!("0x{:016x}", u64::from_le_bytes(b)),
        Err(e) => {
            eprintln!(
                "[identity] /dev/urandom read failed ({e}); using legacy identity"
            );
            LEGACY_SOURCE.to_string()
        }
    }
}

/// Initialize this process's identity. Call once, early in `main`, before any core
/// is built. Precedence: `--seed` > persisted file > fresh random (persisted).
pub fn init(seed_arg: Option<&str>) -> Result<()> {
    let path = identity_path();
    let source = match seed_arg {
        Some(s) => {
            save_source(&path, s);
            s.to_string()
        }
        None => match load_source(&path) {
            Some(s) => s,
            None => {
                let s = random_source();
                save_source(&path, &s);
                s
            }
        },
    };
    let id = identity_from_source(&source)?;
    // Open book: logging the derived identifiers is intentional (see module docs).
    eprintln!(
        "[identity] SP UID {}, RoT UUID {}",
        to_hex(&id.sp_uid),
        to_hex(&id.rot_uuid),
    );
    let _ = IDENTITY.set(id); // first call wins; ignore a redundant re-init
    Ok(())
}

/// The current process identity, lazily defaulting if [`init`] was never called
/// (unit tests / non-CLI paths) so accessors never panic.
pub fn current() -> &'static Identity {
    IDENTITY.get_or_init(|| Identity::from_master(LEGACY_MASTER))
}

pub fn sp_uid_word(off: u32) -> u32 {
    current().sp_uid_word(off)
}
pub fn dice_cdi_words() -> [u32; 8] {
    current().dice_cdi_words()
}
pub fn puf_uds() -> [u8; 32] {
    current().puf_uds
}
pub fn rot_uuid() -> [u8; 16] {
    current().rot_uuid
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(source: &str) -> Identity {
        identity_from_source(source).unwrap()
    }

    #[test]
    fn derivation_is_deterministic_and_seed_dependent() {
        let a = id("grapefruit-1");
        let a2 = id("grapefruit-1");
        let b = id("grapefruit-2");
        assert_eq!(a.sp_uid, a2.sp_uid);
        assert_eq!(a.dice_cdi, a2.dice_cdi);
        assert_ne!(a.sp_uid, b.sp_uid);
        assert_ne!(a.rot_uuid, b.rot_uuid);
        assert_ne!(
            a.dice_cdi, b.dice_cdi,
            "distinct CDI -> distinct self-signed cert"
        );
        assert_ne!(a.puf_uds, b.puf_uds);
    }

    #[test]
    fn fields_are_distinct_within_one_identity() {
        let id = id("anything");
        assert_ne!(&id.dice_cdi[..], &id.puf_uds[..]);
        assert_ne!(&id.rot_uuid[..], &id.sp_uid[..]);
        assert!(id.dice_cdi.iter().any(|&b| b != 0));
    }

    /// The RoT UUID must parse as the same class of UUID the factory programs
    /// (RFC 4122 version 3), not as arbitrary bytes.
    #[test]
    fn rot_uuid_is_rfc4122_v3() {
        for seed in ["a", "b", "0123456789abcdef"] {
            let id = id(seed);
            assert_eq!(id.rot_uuid[6] >> 4, 3, "version nibble");
            assert_eq!(id.rot_uuid[8] >> 6, 0b10, "RFC 4122 variant");
        }
    }

    #[test]
    fn hex_seed_parses_and_errors() {
        // `0x`-prefixed u64s of varying width resolve; case-insensitive prefix.
        assert!(identity_from_source("0x1234").is_ok());
        assert!(identity_from_source("0X0").is_ok());
        assert_eq!(id("0x1234").sp_uid, id("0x1234").sp_uid);
        assert_ne!(id("0x1").sp_uid, id("0x2").sp_uid);
        // Malformed 0x seeds are hard errors, not silently hashed.
        assert!(identity_from_source("0x").is_err(), "empty hex");
        assert!(identity_from_source("0xZZZ").is_err(), "invalid hex");
        assert!(
            identity_from_source("0x00000000000000000").is_err(),
            "17 digits > u64"
        );
    }

    #[test]
    fn legacy_reproduces_the_old_constants() {
        let l = Identity::legacy();
        // Exact pre-identity UID words.
        assert_eq!(l.sp_uid_word(0x0), 0x5350_4D45);
        assert_eq!(l.sp_uid_word(0x4), 0x2D45_4D55);
        assert_eq!(l.sp_uid_word(0x8), 0x0000_0001);
        // Exact pre-identity DICE CDI and PUF UDS.
        assert_eq!(l.dice_cdi_words()[0], 0xc0de_d1ce);
        assert_eq!(l.dice_cdi_words()[7], 0xc3d2_e1f0);
        assert_eq!(l.puf_uds, LEGACY_PUF_UDS);
        // The `legacy` source (any case) selects it.
        assert_eq!(id("legacy").sp_uid, l.sp_uid);
        assert_eq!(id("LEGACY").dice_cdi, l.dice_cdi);
    }

    #[test]
    fn identity_file_round_trips_the_source() {
        let path = std::env::temp_dir()
            .join(format!("sp-emu-idtest-{}", std::process::id()))
            .to_string_lossy()
            .into_owned();
        let _ = std::fs::remove_file(&path);
        assert!(load_source(&path).is_none(), "absent file -> None");
        for src in ["legacy", "0xdeadbeef", "some-string-seed"] {
            save_source(&path, src);
            assert_eq!(load_source(&path).as_deref(), Some(src));
            // The reloaded source produces the same identity as the original.
            assert_eq!(id(src).sp_uid, id(&load_source(&path).unwrap()).sp_uid);
        }
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn mac_is_locally_administered_unicast() {
        let id = Identity::from_master(LEGACY_MASTER);
        assert_eq!(id.mac[0] & 0x01, 0, "unicast");
        assert_eq!(id.mac[0] & 0x02, 0x02, "locally administered");
    }
}
