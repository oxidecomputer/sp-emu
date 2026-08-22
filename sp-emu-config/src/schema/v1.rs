// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! The v1 external schema: the raw, untrusted config file as written.
//!
//! Every field is `Option`, so "set" is `Some` and a dumped file round-trips
//! only the knobs that were set. Values are held as written (paths as strings,
//! compound knobs like `net.bridge` and `trace.rotdump` as strings); ingesting
//! into the validated `Config` is where they are checked and given effect.
//!
//! `deny_unknown_fields` on every struct turns a misspelled key or a stray
//! section into a parse error rather than a silently ignored line. This module
//! is frozen once released: a v2 adds a sibling and a `v1 -> v2` migration.

use serde::{Deserialize, Serialize};

/// A whole v1 config file. Sections default to all-unset when absent, so a file
/// may set as few knobs as it likes.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConfigFileV1 {
    /// The schema version. `Some(1)` in a well-formed v1 file; ingest checks it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub schema_version: Option<u32>,
    #[serde(default, skip_serializing_if = "Op::is_empty")]
    pub op: Op,
    #[serde(default, skip_serializing_if = "Paths::is_empty")]
    pub paths: Paths,
    #[serde(default, skip_serializing_if = "Net::is_empty")]
    pub net: Net,
    #[serde(default, skip_serializing_if = "Host::is_empty")]
    pub host: Host,
    #[serde(default, skip_serializing_if = "I2c::is_empty")]
    pub i2c: I2c,
    #[serde(default, skip_serializing_if = "Rot::is_empty")]
    pub rot: Rot,
    #[serde(default, skip_serializing_if = "Sprot::is_empty")]
    pub sprot: Sprot,
    #[serde(default, skip_serializing_if = "Vpd::is_empty")]
    pub vpd: Vpd,
    #[serde(default, skip_serializing_if = "Sensors::is_empty")]
    pub sensors: Sensors,
    #[serde(default, skip_serializing_if = "Dump::is_empty")]
    pub dump: Dump,
    #[serde(default, skip_serializing_if = "Trace::is_empty")]
    pub trace: Trace,
    #[serde(default, skip_serializing_if = "Stats::is_empty")]
    pub stats: Stats,
    #[serde(default, skip_serializing_if = "Debug::is_empty")]
    pub debug: Debug,
}

/// What this instance is and how it runs.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Op {
    /// Subcommand when the command line names none: `run` or `gdb`.
    pub mode: Option<String>,
    /// Boot slot (`a`/`b`) when no positional is given.
    pub slot: Option<String>,
    /// Instruction budget for `run`; 0 serves forever.
    pub run_max: Option<u64>,
    /// Board profile (`gimlet`/`sidecar`).
    pub board: Option<String>,
    /// Identity seed.
    pub seed: Option<String>,
    /// Ignition controller topology, e.g. `0:gimlet,1:sidecar`.
    pub ignition: Option<String>,
}

/// Instance state file paths.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Paths {
    pub flash: Option<String>,
    pub rot_nvm: Option<String>,
    pub identity: Option<String>,
    pub state_dir: Option<String>,
    /// The SP Hubris archive (.zip).
    pub archive: Option<String>,
}

/// Host bridge and Ethernet.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Net {
    pub bridge: Option<String>,
    pub well_known_ports: Option<bool>,
    pub addr0: Option<String>,
    pub addr1: Option<String>,
    pub vid0: Option<u16>,
    pub vid1: Option<u16>,
    pub eth_quantum: Option<u32>,
    pub eth_txbreak: Option<bool>,
    /// WFI idle-throttle period, milliseconds.
    pub idle_ms: Option<u64>,
}

/// Host UART / IPCC.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Host {
    /// Unix socket to connect the host UART to.
    pub uart: Option<String>,
    /// Serve the host UART on a pty sp-emu creates.
    pub pty: Option<bool>,
}

/// Companion I2C bridge.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct I2c {
    /// SNIFF: tee every transaction to this address.
    pub bridge: Option<String>,
    /// DELEGATE: serve reads from this device server.
    pub device: Option<String>,
}

/// Root of Trust.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Rot {
    pub rom: Option<bool>,
    pub fresh: Option<bool>,
    pub measure: Option<bool>,
    pub service: Option<String>,
    pub flash: Option<String>,
    pub bootleby: Option<String>,
    pub no_bootleby: Option<bool>,
    pub cmpa: Option<String>,
    pub cfpa: Option<String>,
    pub nmpa: Option<String>,
    pub image_b: Option<String>,
    pub erase_a: Option<bool>,
    pub boot_pref: Option<String>,
    pub dice: Option<String>,
    pub preboot: Option<u64>,
}

/// SP <-> RoT coupling.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sprot {
    pub flowctl: Option<u32>,
    pub couple: Option<bool>,
    pub endoscope_couple: Option<bool>,
    pub sp_clock_khz: Option<u32>,
}

/// VPD identity fields.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Vpd {
    pub serial: Option<String>,
    pub part: Option<String>,
    pub rev: Option<String>,
}

/// Sensor model.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sensors {
    /// Per-address temperature overrides, e.g. `0x48=30.0,0x49=31.5`.
    pub overrides: Option<String>,
    pub ambient_c: Option<f32>,
}

/// Hydrate RAM-dump trigger.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Dump {
    pub dir: Option<String>,
    pub archive_id: Option<String>,
}

/// Instruction traces, windows, and profiling.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Trace {
    pub enabled: Option<bool>,
    pub from: Option<u64>,
    pub to: Option<u64>,
    pub rot_from: Option<u32>,
    pub rot_to: Option<u32>,
    pub rotpc: Option<u64>,
    /// RoT RAM dump window, `0xADDR:LEN`.
    pub rotdump: Option<String>,
    pub watch: Option<u32>,
    /// Differential-trace output file.
    pub diff: Option<String>,
    pub pcprof: Option<bool>,
}

/// Periodic counters.
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Stats {
    pub rx: Option<bool>,
    pub rtt: Option<bool>,
    pub pump: Option<bool>,
    pub pump_ms: Option<u64>,
}

/// Per-subsystem log toggles and one-shot triggers. The bare names read in
/// section context (`debug.eth = true`).
#[derive(Debug, Default, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Debug {
    /// Disable the debug servers (serve the bridge only).
    pub no_debug: Option<bool>,
    pub no_archive_warn: Option<bool>,
    pub swd_trigger: Option<bool>,
    pub jtag_trigger: Option<bool>,
    pub swd_trace: Option<bool>,
    pub rotsvc: Option<bool>,
    pub pingtest: Option<bool>,
    pub flash: Option<bool>,
    pub rotflash: Option<bool>,
    pub eth: Option<bool>,
    pub uart: Option<bool>,
    pub bridge: Option<bool>,
    pub puf: Option<bool>,
    pub vsc: Option<bool>,
    pub rx: Option<bool>,
    pub mdio: Option<bool>,
    pub vpd: Option<bool>,
    pub spi: Option<bool>,
    pub panic: Option<bool>,
    pub svc: Option<bool>,
    pub exc: Option<bool>,
    pub sprot: Option<bool>,
    pub couple: Option<bool>,
    pub rom: Option<bool>,
    pub config: Option<bool>,
}

/// Each section reports whether it is entirely unset, so serializing a config
/// omits empty sections instead of emitting a wall of bare tables.
macro_rules! is_empty_impl {
    ($ty:ty { $($field:ident),* $(,)? }) => {
        impl $ty {
            fn is_empty(&self) -> bool {
                $( self.$field.is_none() )&&*
            }
        }
    };
}

is_empty_impl!(Op {
    mode,
    slot,
    run_max,
    board,
    seed,
    ignition
});
is_empty_impl!(Paths {
    flash,
    rot_nvm,
    identity,
    state_dir,
    archive
});
is_empty_impl!(Net {
    bridge,
    well_known_ports,
    addr0,
    addr1,
    vid0,
    vid1,
    eth_quantum,
    eth_txbreak,
    idle_ms
});
is_empty_impl!(Host { uart, pty });
is_empty_impl!(I2c { bridge, device });
is_empty_impl!(Rot {
    rom,
    fresh,
    measure,
    service,
    flash,
    bootleby,
    no_bootleby,
    cmpa,
    cfpa,
    nmpa,
    image_b,
    erase_a,
    boot_pref,
    dice,
    preboot
});
is_empty_impl!(Sprot {
    flowctl,
    couple,
    endoscope_couple,
    sp_clock_khz
});
is_empty_impl!(Vpd { serial, part, rev });
is_empty_impl!(Sensors {
    overrides,
    ambient_c
});
is_empty_impl!(Dump { dir, archive_id });
is_empty_impl!(Trace {
    enabled,
    from,
    to,
    rot_from,
    rot_to,
    rotpc,
    rotdump,
    watch,
    diff,
    pcprof
});
is_empty_impl!(Stats {
    rx,
    rtt,
    pump,
    pump_ms
});
is_empty_impl!(Debug {
    no_debug,
    no_archive_warn,
    swd_trigger,
    jtag_trigger,
    swd_trace,
    rotsvc,
    pingtest,
    flash,
    rotflash,
    eth,
    uart,
    bridge,
    puf,
    vsc,
    rx,
    mdio,
    vpd,
    spi,
    panic,
    svc,
    exc,
    sprot,
    couple,
    rom,
    config
});

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_file_parses_to_all_unset() {
        let cfg: ConfigFileV1 = toml::from_str("").unwrap();
        assert_eq!(cfg, ConfigFileV1::default());
    }

    #[test]
    fn a_typed_file_populates_the_right_sections() {
        let text = "\
schema_version = 1
[op]
mode = \"run\"
slot = \"a\"
[rot]
flash = \"rot.bin\"
preboot = 40000000
[net]
vid0 = 0x301
[debug]
eth = true
";
        let cfg: ConfigFileV1 = toml::from_str(text).unwrap();
        assert_eq!(cfg.schema_version, Some(1));
        assert_eq!(cfg.op.mode.as_deref(), Some("run"));
        assert_eq!(cfg.rot.flash.as_deref(), Some("rot.bin"));
        assert_eq!(cfg.rot.preboot, Some(40_000_000));
        assert_eq!(cfg.net.vid0, Some(0x301));
        assert_eq!(cfg.debug.eth, Some(true));
        // A section not mentioned stays entirely unset.
        assert!(cfg.vpd.serial.is_none());
    }

    #[test]
    fn set_knobs_round_trip_and_omit_empty_sections() {
        let mut cfg = ConfigFileV1 {
            schema_version: Some(1),
            ..Default::default()
        };
        cfg.op.mode = Some("gdb".into());
        cfg.rot.cmpa = Some("cmpa.bin".into());
        let text = toml::to_string(&cfg).unwrap();
        // Empty sections are omitted from the emitted file.
        assert!(!text.contains("[vpd]"));
        assert!(text.contains("[op]"));
        assert!(text.contains("[rot]"));
        let back: ConfigFileV1 = toml::from_str(&text).unwrap();
        assert_eq!(cfg, back);
    }

    #[test]
    fn an_unknown_key_is_rejected() {
        assert!(toml::from_str::<ConfigFileV1>("[op]\nmodee = \"run\"\n").is_err());
    }

    #[test]
    fn an_unknown_section_is_rejected() {
        assert!(toml::from_str::<ConfigFileV1>("[nope]\nx = 1\n").is_err());
    }
}
