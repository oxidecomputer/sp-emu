//! The v0 bridge: the flat `SP_EMU_NAME = "value"` form mapped onto the typed v1
//! schema. This is the single source of the flat-name to typed-field
//! correspondence, used to migrate a legacy flat file forward and (in a later
//! phase) to read the `SP_EMU_*` environment into the same external form.
//!
//! Parsing here preserves the historical leniency: a value that silently coerced
//! to a default before still does (a malformed number is dropped to unset, a
//! board other than `sidecar` is left unset so it defaults to gimlet). Strict
//! checking happens later, in ingest, on the typed result.

use crate::error::ConfigError;
use crate::schema::v1::ConfigFileV1;

/// Every flat `SP_EMU_*` name this build understands, so a caller (the emulator's
/// env reader) can enumerate the variables to look up. Keep in sync with the
/// `set` match below; the emulator's backward-compatibility test guards the
/// historical subset.
pub const ENV_NAMES: &[&str] = &[
    "SP_EMU_FLASH",
    "SP_EMU_ROT_NVM",
    "SP_EMU_IDENTITY",
    "SP_EMU_STATE_DIR",
    "SP_EMU_ARCHIVE",
    "SP_EMU_SEED",
    "SP_EMU_MODE",
    "SP_EMU_SLOT",
    "SP_EMU_RUN_MAX",
    "SP_EMU_BOARD",
    "SP_EMU_IGNITION",
    "SP_EMU_BRIDGE",
    "SP_EMU_WELL_KNOWN_PORTS",
    "SP_EMU_ADDR0",
    "SP_EMU_ADDR1",
    "SP_EMU_VID0",
    "SP_EMU_VID1",
    "SP_EMU_ETH_QUANTUM",
    "SP_EMU_ETH_TXBREAK",
    "SP_EMU_IDLE_MS",
    "SP_EMU_HOST_UART",
    "SP_EMU_HOST_PTY",
    "SP_EMU_I2C_BRIDGE",
    "SP_EMU_I2C_DEVICE",
    "SP_EMU_ROT_ROM",
    "SP_EMU_ROT_FRESH",
    "SP_EMU_ROT_MEASURE",
    "SP_EMU_ROT_SERVICE",
    "SP_EMU_ROT_FLASH",
    "SP_EMU_ROT_BOOTLEBY",
    "SP_EMU_ROT_NO_BOOTLEBY",
    "SP_EMU_ROT_CMPA",
    "SP_EMU_ROT_CFPA",
    "SP_EMU_ROT_NMPA",
    "SP_EMU_ROT_IMAGE_B",
    "SP_EMU_ROT_ERASE_A",
    "SP_EMU_ROT_BOOT_PREF",
    "SP_EMU_ROT_DICE",
    "SP_EMU_ROT_PREBOOT",
    "SP_EMU_SPROT_FLOWCTL",
    "SP_EMU_SPROT_COUPLE",
    "SP_EMU_ENDOSCOPE_COUPLE",
    "SP_EMU_SP_CLOCK_KHZ",
    "SP_EMU_VPD_SERIAL",
    "SP_EMU_VPD_PART",
    "SP_EMU_VPD_REV",
    "SP_EMU_SENSORS",
    "SP_EMU_AMBIENT_C",
    "SP_EMU_DUMP_DIR",
    "SP_EMU_DUMP_ARCHIVE_ID",
    "SP_EMU_TRACE",
    "SP_EMU_TRACE_FROM",
    "SP_EMU_TRACE_TO",
    "SP_EMU_ROT_TRACE_FROM",
    "SP_EMU_ROT_TRACE_TO",
    "SP_EMU_ROTPC",
    "SP_EMU_ROTDUMP",
    "SP_EMU_WATCH",
    "SP_EMU_DIFF",
    "SP_EMU_PCPROF",
    "SP_EMU_RXSTATS",
    "SP_EMU_RTTSTATS",
    "SP_EMU_PUMPSTATS",
    "SP_EMU_PUMPSTATS_MS",
    "SP_EMU_NO_DEBUG",
    "SP_EMU_NO_ARCHIVE_WARN",
    "SP_EMU_SWD_TRIGGER",
    "SP_EMU_JTAG_TRIGGER",
    "SP_EMU_SWD_TRACE",
    "SP_EMU_ROTSVC",
    "SP_EMU_PINGTEST",
    "SP_EMU_FLASHDBG",
    "SP_EMU_ROTFLASHDBG",
    "SP_EMU_ETHDBG",
    "SP_EMU_UARTDBG",
    "SP_EMU_BRIDGEDBG",
    "SP_EMU_PUFDBG",
    "SP_EMU_VSCDBG",
    "SP_EMU_RXDBG",
    "SP_EMU_MDIODBG",
    "SP_EMU_VPDDBG",
    "SP_EMU_SPIDBG",
    "SP_EMU_PANICDBG",
    "SP_EMU_SVCDBG",
    "SP_EMU_EXCDBG",
    "SP_EMU_SPROTDBG",
    "SP_EMU_COUPLEDBG",
    "SP_EMU_ROMDBG",
    "SP_EMU_CONFIGDBG",
];

/// Turn a flat `SP_EMU_NAME = "value"` TOML document into the typed v1 schema.
pub fn flat_to_v1(text: &str) -> Result<ConfigFileV1, ConfigError> {
    let table: toml::Table = toml::from_str(text)?;
    Ok(flat_pairs_to_v1(
        table.iter().map(|(k, v)| (k.clone(), scalar_string(v))),
    ))
}

/// Turn flat `(SP_EMU_NAME, value)` pairs (from a file or the environment) into
/// the typed v1 schema. Unknown names are ignored; known ones are parsed with the
/// knob's historical leniency.
pub fn flat_pairs_to_v1(pairs: impl IntoIterator<Item = (String, String)>) -> ConfigFileV1 {
    let mut c = ConfigFileV1::default();
    for (name, value) in pairs {
        set(&mut c, &name, &value);
    }
    c
}

/// A flat value as a plain string, regardless of the TOML scalar type it parsed
/// as (historically every value was a quoted string, but a native number or bool
/// is accepted too).
fn scalar_string(v: &toml::Value) -> String {
    match v {
        toml::Value::String(s) => s.clone(),
        toml::Value::Integer(i) => i.to_string(),
        toml::Value::Boolean(b) => b.to_string(),
        toml::Value::Float(f) => f.to_string(),
        other => other.to_string(),
    }
}

/// Parse a hex value into `T`, dropping to `None` on a malformed or out-of-range
/// input, exactly as the historical `uN::from_str_radix` did (an overflowing
/// value fell back to the knob's default rather than truncating).
fn hex<T: TryFrom<u64>>(s: &str) -> Option<T> {
    let raw = u64::from_str_radix(s.trim().trim_start_matches("0x"), 16).ok()?;
    T::try_from(raw).ok()
}

/// Set the typed field a flat `SP_EMU_*` name maps to, parsing `s` the way the
/// knob historically did. An unknown name is ignored (forward compatibility with
/// a flat file that carries a knob this build has dropped).
fn set(c: &mut ConfigFileV1, name: &str, s: &str) {
    // Presence-only bool: the value never mattered, only that the key was set.
    let present = Some(true);
    // Ternary bool: on unless explicitly "0" (matches the three defaulted-true
    // couplings' `s != "0"` parse).
    let ne0 = Some(s != "0");

    match name {
        // paths
        "SP_EMU_FLASH" => c.paths.flash = Some(s.into()),
        "SP_EMU_ROT_NVM" => c.paths.rot_nvm = Some(s.into()),
        "SP_EMU_IDENTITY" => c.paths.identity = Some(s.into()),
        "SP_EMU_STATE_DIR" => c.paths.state_dir = Some(s.into()),
        "SP_EMU_ARCHIVE" => c.paths.archive = Some(s.into()),

        // operation
        "SP_EMU_SEED" => c.op.seed = Some(s.into()),
        "SP_EMU_MODE" => c.op.mode = Some(s.into()),
        "SP_EMU_SLOT" => c.op.slot = Some(s.into()),
        "SP_EMU_RUN_MAX" => c.op.run_max = s.parse().ok(),
        // Lenient board: only "sidecar" is meaningful; any other spelling stays
        // unset and ingest defaults it to gimlet, as the old resolver did.
        "SP_EMU_BOARD" => {
            if s == "sidecar" {
                c.op.board = Some("sidecar".into());
            }
        }
        "SP_EMU_IGNITION" => c.op.ignition = Some(s.into()),

        // host bridge + Ethernet
        "SP_EMU_BRIDGE" => c.net.bridge = Some(s.into()),
        "SP_EMU_WELL_KNOWN_PORTS" => c.net.well_known_ports = present,
        "SP_EMU_ADDR0" => c.net.addr0 = Some(s.into()),
        "SP_EMU_ADDR1" => c.net.addr1 = Some(s.into()),
        "SP_EMU_VID0" => c.net.vid0 = hex::<u16>(s),
        "SP_EMU_VID1" => c.net.vid1 = hex::<u16>(s),
        "SP_EMU_ETH_QUANTUM" => c.net.eth_quantum = s.parse().ok(),
        "SP_EMU_ETH_TXBREAK" => c.net.eth_txbreak = ne0,
        "SP_EMU_IDLE_MS" => c.net.idle_ms = s.parse().ok(),

        // host UART / IPCC
        "SP_EMU_HOST_UART" => c.host.uart = Some(s.into()),
        "SP_EMU_HOST_PTY" => c.host.pty = present,

        // companion I2C bridge
        "SP_EMU_I2C_BRIDGE" => c.i2c.bridge = Some(s.into()),
        "SP_EMU_I2C_DEVICE" => c.i2c.device = Some(s.into()),

        // RoT
        "SP_EMU_ROT_ROM" => c.rot.rom = present,
        "SP_EMU_ROT_FRESH" => c.rot.fresh = present,
        "SP_EMU_ROT_MEASURE" => c.rot.measure = present,
        "SP_EMU_ROT_SERVICE" => c.rot.service = Some(s.into()),
        "SP_EMU_ROT_FLASH" => c.rot.flash = Some(s.into()),
        "SP_EMU_ROT_BOOTLEBY" => c.rot.bootleby = Some(s.into()),
        "SP_EMU_ROT_NO_BOOTLEBY" => c.rot.no_bootleby = present,
        "SP_EMU_ROT_CMPA" => c.rot.cmpa = Some(s.into()),
        "SP_EMU_ROT_CFPA" => c.rot.cfpa = Some(s.into()),
        "SP_EMU_ROT_NMPA" => c.rot.nmpa = Some(s.into()),
        "SP_EMU_ROT_IMAGE_B" => c.rot.image_b = Some(s.into()),
        "SP_EMU_ROT_ERASE_A" => c.rot.erase_a = present,
        // Lenient: historically only "b" meant slot B and any other value fell
        // back to slot A without error, so a non-a/b value stays unset here.
        "SP_EMU_ROT_BOOT_PREF" => {
            if s == "a" || s == "b" {
                c.rot.boot_pref = Some(s.into());
            }
        }
        "SP_EMU_ROT_DICE" => c.rot.dice = Some(s.into()),
        "SP_EMU_ROT_PREBOOT" => c.rot.preboot = s.parse().ok(),

        // SP <-> RoT coupling
        "SP_EMU_SPROT_FLOWCTL" => c.sprot.flowctl = s.parse().ok(),
        "SP_EMU_SPROT_COUPLE" => c.sprot.couple = ne0,
        "SP_EMU_ENDOSCOPE_COUPLE" => c.sprot.endoscope_couple = ne0,
        "SP_EMU_SP_CLOCK_KHZ" => c.sprot.sp_clock_khz = s.parse().ok(),

        // VPD identity
        "SP_EMU_VPD_SERIAL" => c.vpd.serial = Some(s.into()),
        "SP_EMU_VPD_PART" => c.vpd.part = Some(s.into()),
        "SP_EMU_VPD_REV" => c.vpd.rev = Some(s.into()),

        // sensors
        "SP_EMU_SENSORS" => c.sensors.overrides = Some(s.into()),
        "SP_EMU_AMBIENT_C" => c.sensors.ambient_c = s.trim().parse().ok(),

        // hydrate RAM dump
        "SP_EMU_DUMP_DIR" => c.dump.dir = Some(s.into()),
        "SP_EMU_DUMP_ARCHIVE_ID" => c.dump.archive_id = Some(s.into()),

        // traces / windows / profiling
        "SP_EMU_TRACE" => c.trace.enabled = present,
        "SP_EMU_TRACE_FROM" => c.trace.from = s.parse().ok(),
        "SP_EMU_TRACE_TO" => c.trace.to = s.parse().ok(),
        "SP_EMU_ROT_TRACE_FROM" => c.trace.rot_from = hex::<u32>(s),
        "SP_EMU_ROT_TRACE_TO" => c.trace.rot_to = hex::<u32>(s),
        "SP_EMU_ROTPC" => c.trace.rotpc = s.parse().ok(),
        // Lenient: a malformed window was historically dropped to unset, not fatal.
        "SP_EMU_ROTDUMP" => {
            if crate::ingest::parse_rotdump(s).is_some() {
                c.trace.rotdump = Some(s.into());
            }
        }
        "SP_EMU_WATCH" => c.trace.watch = hex::<u32>(s),
        "SP_EMU_DIFF" => c.trace.diff = Some(s.into()),
        "SP_EMU_PCPROF" => c.trace.pcprof = present,

        // periodic stats
        "SP_EMU_RXSTATS" => c.stats.rx = present,
        "SP_EMU_RTTSTATS" => c.stats.rtt = present,
        "SP_EMU_PUMPSTATS" => c.stats.pump = present,
        "SP_EMU_PUMPSTATS_MS" => c.stats.pump_ms = s.parse().ok(),

        // per-subsystem log toggles + one-shots
        "SP_EMU_NO_DEBUG" => c.debug.no_debug = present,
        "SP_EMU_NO_ARCHIVE_WARN" => c.debug.no_archive_warn = present,
        "SP_EMU_SWD_TRIGGER" => c.debug.swd_trigger = present,
        "SP_EMU_JTAG_TRIGGER" => c.debug.jtag_trigger = present,
        "SP_EMU_SWD_TRACE" => c.debug.swd_trace = present,
        "SP_EMU_ROTSVC" => c.debug.rotsvc = present,
        "SP_EMU_PINGTEST" => c.debug.pingtest = present,
        "SP_EMU_FLASHDBG" => c.debug.flash = present,
        "SP_EMU_ROTFLASHDBG" => c.debug.rotflash = present,
        "SP_EMU_ETHDBG" => c.debug.eth = present,
        "SP_EMU_UARTDBG" => c.debug.uart = present,
        "SP_EMU_BRIDGEDBG" => c.debug.bridge = present,
        "SP_EMU_PUFDBG" => c.debug.puf = present,
        "SP_EMU_VSCDBG" => c.debug.vsc = present,
        "SP_EMU_RXDBG" => c.debug.rx = present,
        "SP_EMU_MDIODBG" => c.debug.mdio = present,
        "SP_EMU_VPDDBG" => c.debug.vpd = present,
        "SP_EMU_SPIDBG" => c.debug.spi = present,
        "SP_EMU_PANICDBG" => c.debug.panic = present,
        "SP_EMU_SVCDBG" => c.debug.svc = present,
        "SP_EMU_EXCDBG" => c.debug.exc = present,
        "SP_EMU_SPROTDBG" => c.debug.sprot = present,
        "SP_EMU_COUPLEDBG" => c.debug.couple = present,
        "SP_EMU_ROMDBG" => c.debug.rom = present,
        "SP_EMU_CONFIGDBG" => c.debug.config = present,

        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Board;
    use crate::ingest::ingest;

    #[test]
    fn a_flat_file_maps_onto_the_typed_sections() {
        let flat = "\
SP_EMU_FLASH = \"my-flash.bin\"
SP_EMU_BOARD = \"sidecar\"
SP_EMU_VID0 = \"0x301\"
SP_EMU_ROT_FLASH = \"rot.bin\"
SP_EMU_ETHDBG = \"1\"
SP_EMU_ROT_PREBOOT = \"40000000\"
";
        let v1 = flat_to_v1(flat).unwrap();
        assert_eq!(v1.paths.flash.as_deref(), Some("my-flash.bin"));
        assert_eq!(v1.op.board.as_deref(), Some("sidecar"));
        assert_eq!(v1.net.vid0, Some(0x301));
        assert_eq!(v1.rot.flash.as_deref(), Some("rot.bin"));
        assert_eq!(v1.debug.eth, Some(true));
        assert_eq!(v1.rot.preboot, Some(40_000_000));
    }

    #[test]
    fn a_board_typo_stays_lenient_through_ingest() {
        // The old resolver silently made a non-"sidecar" board gimlet; the flat
        // bridge preserves that, so ingest does not reject the typo.
        let c = ingest(flat_to_v1("SP_EMU_BOARD = \"typo\"\n").unwrap()).unwrap();
        assert_eq!(c.board(), Board::Gimlet);
    }

    #[test]
    fn txbreak_is_off_only_for_zero() {
        let on = ingest(flat_to_v1("SP_EMU_ETH_TXBREAK = \"1\"\n").unwrap()).unwrap();
        let off = ingest(flat_to_v1("SP_EMU_ETH_TXBREAK = \"0\"\n").unwrap()).unwrap();
        assert!(on.eth_txbreak());
        assert!(!off.eth_txbreak());
    }

    #[test]
    fn an_unknown_flat_name_is_ignored() {
        let v1 = flat_to_v1("SP_EMU_FROM_THE_FUTURE = \"x\"\n").unwrap();
        assert_eq!(v1, ConfigFileV1::default());
    }

    #[test]
    fn flat_pairs_take_the_same_path_as_a_flat_file() {
        let v1 = flat_pairs_to_v1([
            ("SP_EMU_ROT_FLASH".to_string(), "r.bin".to_string()),
            ("SP_EMU_ETHDBG".to_string(), "1".to_string()),
        ]);
        assert_eq!(v1.rot.flash.as_deref(), Some("r.bin"));
        assert_eq!(v1.debug.eth, Some(true));
    }

    #[test]
    fn a_lenient_boot_pref_and_rotdump_do_not_reach_strict_ingest() {
        // A non-a/b boot preference and a malformed rotdump were historically
        // tolerated; they stay unset so ingest (strict) does not reject them.
        let c = ingest(
            flat_pairs_to_v1([
                ("SP_EMU_ROT_BOOT_PREF".to_string(), "whatever".to_string()),
                ("SP_EMU_ROTDUMP".to_string(), "not-a-window".to_string()),
            ])
            .clone(),
        )
        .unwrap();
        assert_eq!(c.rot_boot_pref(), None);
        assert_eq!(c.rotdump(), None);
    }

    #[test]
    fn env_names_covers_the_backward_compat_knobs() {
        for name in [
            "SP_EMU_FLASH",
            "SP_EMU_BOARD",
            "SP_EMU_ETHDBG",
            "SP_EMU_VID0",
        ] {
            assert!(ENV_NAMES.contains(&name), "{name} missing from ENV_NAMES");
        }
    }

    /// A probe value that `set` accepts for `name`, so setting it moves the
    /// external form off its default. Most knobs take "1"; the validated-enum and
    /// compound knobs need a value they actually accept.
    fn probe(name: &str) -> &'static str {
        match name {
            "SP_EMU_BOARD" => "sidecar",
            "SP_EMU_MODE" => "run",
            "SP_EMU_SLOT" | "SP_EMU_ROT_BOOT_PREF" => "a",
            "SP_EMU_ROTDUMP" => "0x10:20",
            _ => "1",
        }
    }

    /// Guard against `ENV_NAMES` and the `set` match drifting apart: every listed
    /// name, given a value it accepts, must change the external form. A name that
    /// `set` does not handle would leave it at the default and fail here.
    #[test]
    fn every_env_name_is_handled_by_set() {
        for &name in ENV_NAMES {
            let v1 = flat_pairs_to_v1([(name.to_string(), probe(name).to_string())]);
            assert_ne!(
                v1,
                ConfigFileV1::default(),
                "{name} is in ENV_NAMES but `set` does not map it to a field"
            );
        }
        assert_eq!(
            ENV_NAMES.len(),
            ENV_NAMES
                .iter()
                .collect::<std::collections::BTreeSet<_>>()
                .len(),
            "ENV_NAMES has a duplicate"
        );
    }

    #[test]
    fn an_out_of_range_hex_falls_back_to_the_default() {
        // 0x10000 overflows u16: historically it dropped to unset (the knob's
        // default), not truncated to 0.
        let v1 = flat_pairs_to_v1([("SP_EMU_VID0".to_string(), "0x10000".to_string())]);
        assert_eq!(v1.net.vid0, None);
        // an in-range value still lands.
        let v1 = flat_pairs_to_v1([("SP_EMU_VID0".to_string(), "0x301".to_string())]);
        assert_eq!(v1.net.vid0, Some(0x301));
    }
}
