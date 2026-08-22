// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Serialize configs back to the typed TOML schema.
//!
//! [`to_toml`] emits an external form as written, so a still-`None` field is
//! omitted (a minimal, only-set file). [`dump`] projects a resolved [`Config`]
//! back to the external schema with every knob set, for an effective-config
//! snapshot. [`template`] is that snapshot for the default config: every knob at
//! its default, a starting point to edit.

use crate::config::{Board, Config};
use crate::error::ConfigError;
use crate::ingest::ingest;
use crate::schema::v1::{self, ConfigFileV1};
use crate::version::CURRENT;

/// Serialize an external config to TOML. Unset (`None`) fields and empty sections
/// are omitted, so this round-trips exactly the knobs that were set.
pub fn to_toml(ext: &ConfigFileV1) -> Result<String, ConfigError> {
    Ok(toml::to_string_pretty(ext)?)
}

/// Serialize a resolved [`Config`] to TOML with every knob written out (defaults
/// included), tagged with the current schema version.
pub fn dump(config: &Config) -> Result<String, ConfigError> {
    to_toml(&config_to_external(config))
}

/// A documented template of the whole schema: every section, with the
/// defaulted knobs shown at their default (active) and the optional knobs shown
/// as commented example lines, so every configurable item is discoverable. Active
/// values are read from the default `Config`, so they cannot drift from `ingest`.
pub fn template() -> String {
    let d = ingest(ConfigFileV1::default()).expect("the empty config is always valid");
    let mut out = String::new();
    // Append one formatted line. An active knob is `key = value`; an optional one
    // is a commented `# key = value` example with a trailing hint. Sequential
    // calls, so the single mutable borrow of `out` is fine.
    macro_rules! line {
        ($($a:tt)*) => {{ out.push_str(&format!($($a)*)); out.push('\n'); }};
    }

    line!("# sp-emu configuration, schema v{CURRENT}.");
    line!("# Active lines are the defaults; commented lines are optional knobs with");
    line!("# example values. (Loaded today as the flat SP_EMU_* form; see `sp-emu config`.)");
    line!("schema_version = {CURRENT}");

    line!("\n# What this instance is and how it runs.");
    line!("[op]");
    line!("# mode = \"run\"          # run | gdb; unset prints usage");
    line!("# slot = \"a\"            # a | b; unset uses the persisted swap bank");
    line!("# run_max = 0            # instruction budget; 0 serves forever");
    line!("board = {:?}", board_str(d.board()));
    line!("# seed = \"0xC0FFEE\"     # per-instance identity seed");
    line!("ignition = {:?}", d.ignition());

    line!("\n# Instance state files (created on first use).");
    line!("[paths]");
    line!("flash = {:?}", d.flash_path());
    line!("rot_nvm = {:?}", d.rot_nvm_path());
    line!("identity = {:?}", d.identity_path());
    line!("# state_dir = \"/path/to/state\"   # default: $XDG_STATE_HOME/sp-emu");
    line!("# archive = \"gimlet.zip\"         # the SP Hubris archive");

    line!("\n# Host bridge and Ethernet.");
    line!("[net]");
    line!("# bridge = \"1\"          # MGS bind address, or \"1\" for the default");
    line!("well_known_ports = {}", d.well_known_ports());
    line!("# addr0 = \"::1\"");
    line!("# addr1 = \"::1\"");
    line!("# vid0 = 0x301");
    line!("# vid1 = 0x302");
    line!("eth_quantum = {}", d.eth_quantum());
    line!("eth_txbreak = {}", d.eth_txbreak());
    line!("idle_ms = {}", d.idle_ms());

    line!("\n# Host UART / IPCC.");
    line!("[host]");
    line!("# uart = \"/path/to/socket\"       # connect the host UART to a unix socket");
    line!("pty = {}", d.host_pty());

    line!("\n# Companion I2C bridge.");
    line!("[i2c]");
    line!("# bridge = \"127.0.0.1:9100\"      # SNIFF: tee transactions to this address");
    line!("# device = \"127.0.0.1:9100\"      # DELEGATE: serve reads from this device");

    line!("\n# Root of Trust.");
    line!("[rot]");
    line!("rom = {}", d.rot_rom());
    line!("fresh = {}", d.rot_fresh());
    line!("measure = {}", d.rot_measure());
    line!("# service = \"127.0.0.1:9200\"     # shared rot-service over IPC");
    line!("# flash = \"rot-a.bin\"            # in-process RoT slot-A image");
    line!("# bootleby = \"bootleby.zip\"");
    line!("no_bootleby = {}", d.rot_no_bootleby());
    line!("# cmpa = \"cmpa.bin\"");
    line!("# cfpa = \"cfpa.bin\"");
    line!("# nmpa = \"nmpa.bin\"");
    line!("# image_b = \"rot-b.bin\"          # slot-B image, for A/B selection");
    line!("erase_a = {}", d.rot_erase_a());
    line!("# boot_pref = \"a\"       # a | b; the synthesized CFPA boot preference");
    line!("# dice = \"dice-dir\"              # DICE handoff blob directory");
    line!("# preboot = 400000000            # RoT preboot instruction budget");

    line!("\n# SP <-> RoT coupling.");
    line!("[sprot]");
    line!("flowctl = {}", d.sprot_flowctl());
    line!("couple = {}", d.sprot_couple());
    line!("endoscope_couple = {}", d.endoscope_couple());
    line!("sp_clock_khz = {}", d.sp_clock_khz());

    line!("\n# VPD identity fields.");
    line!("[vpd]");
    line!("# serial = \"BRM42220001\"");
    line!("# part = \"913-0000019\"");
    line!("# rev = \"002\"");

    line!("\n# Sensor model.");
    line!("[sensors]");
    line!("# overrides = \"0x48=30.0,0x49=31.5\"   # per-address temperature overrides");
    line!("ambient_c = {}", d.ambient_c());

    line!("\n# Hydrate RAM-dump trigger.");
    line!("[dump]");
    line!("# dir = \"dump-dir\"");
    line!("archive_id = {:?}", d.dump_archive_id());

    line!("\n# Instruction traces, windows, and profiling.");
    line!("[trace]");
    line!("enabled = {}", d.trace());
    line!("# from = 0");
    line!("# to = 0");
    line!("# rot_from = 0");
    line!("# rot_to = 0");
    line!("# rotpc = 0");
    line!("# rotdump = \"0x20000000:256\"      # RoT RAM window, 0xADDR:LEN");
    line!("# watch = 0");
    line!("# diff = \"diff.log\"               # differential-trace output file");
    line!("pcprof = {}", d.pcprof());

    line!("\n# Periodic counters.");
    line!("[stats]");
    line!("rx = {}", d.rxstats());
    line!("rtt = {}", d.rttstats());
    line!("pump = {}", d.pumpstats());
    line!("pump_ms = {}", d.pumpstats_ms());

    line!("\n# Per-subsystem log toggles and one-shot triggers.");
    line!("[debug]");
    line!("no_debug = {}", d.no_debug());
    line!("no_archive_warn = {}", d.no_archive_warn());
    line!("swd_trigger = {}", d.swd_trigger());
    line!("jtag_trigger = {}", d.jtag_trigger());
    line!("swd_trace = {}", d.swd_trace());
    line!("rotsvc = {}", d.rotsvc());
    line!("pingtest = {}", d.pingtest());
    line!("flash = {}", d.flashdbg());
    line!("rotflash = {}", d.rotflashdbg());
    line!("eth = {}", d.ethdbg());
    line!("uart = {}", d.uartdbg());
    line!("bridge = {}", d.bridgedbg());
    line!("puf = {}", d.pufdbg());
    line!("vsc = {}", d.vscdbg());
    line!("rx = {}", d.rxdbg());
    line!("mdio = {}", d.mdiodbg());
    line!("vpd = {}", d.vpddbg());
    line!("spi = {}", d.spidbg());
    line!("panic = {}", d.panicdbg());
    line!("svc = {}", d.svcdbg());
    line!("exc = {}", d.excdbg());
    line!("sprot = {}", d.sprotdbg());
    line!("couple = {}", d.coupledbg());
    line!("rom = {}", d.romdbg());
    line!("config = {}", d.configdbg());

    out
}

/// Project a resolved [`Config`] back onto the external schema, every field set.
fn config_to_external(c: &Config) -> ConfigFileV1 {
    ConfigFileV1 {
        schema_version: Some(CURRENT),
        op: v1::Op {
            mode: c.mode.clone(),
            slot: c.boot_slot.clone(),
            run_max: c.run_max,
            board: Some(board_str(c.board).to_string()),
            seed: c.seed.clone(),
            ignition: Some(c.ignition.clone()),
        },
        paths: v1::Paths {
            flash: Some(c.flash_path.clone()),
            rot_nvm: Some(c.rot_nvm_path.clone()),
            identity: Some(c.identity_path.clone()),
            state_dir: c.state_dir.clone(),
            archive: c.archive.clone(),
        },
        net: v1::Net {
            bridge: c.bridge.clone(),
            well_known_ports: Some(c.well_known_ports),
            addr0: c.addr0.clone(),
            addr1: c.addr1.clone(),
            vid0: c.vid0,
            vid1: c.vid1,
            eth_quantum: Some(c.eth_quantum),
            eth_txbreak: Some(c.eth_txbreak),
            idle_ms: Some(c.idle_ms),
        },
        host: v1::Host {
            uart: c.host_uart.clone(),
            pty: Some(c.host_pty),
        },
        i2c: v1::I2c {
            bridge: c.i2c_bridge.clone(),
            device: c.i2c_device.clone(),
        },
        rot: v1::Rot {
            rom: Some(c.rot_rom),
            fresh: Some(c.rot_fresh),
            measure: Some(c.rot_measure),
            service: c.rot_service.clone(),
            flash: c.rot_flash.clone(),
            bootleby: c.rot_bootleby.clone(),
            no_bootleby: Some(c.rot_no_bootleby),
            cmpa: c.rot_cmpa.clone(),
            cfpa: c.rot_cfpa.clone(),
            nmpa: c.rot_nmpa.clone(),
            image_b: c.rot_image_b.clone(),
            erase_a: Some(c.rot_erase_a),
            boot_pref: c.rot_boot_pref.clone(),
            dice: c.rot_dice.clone(),
            preboot: c.rot_preboot,
        },
        sprot: v1::Sprot {
            flowctl: Some(c.sprot_flowctl),
            couple: Some(c.sprot_couple),
            endoscope_couple: Some(c.endoscope_couple),
            sp_clock_khz: Some(c.sp_clock_khz),
        },
        vpd: v1::Vpd {
            serial: c.vpd_serial.clone(),
            part: c.vpd_part.clone(),
            rev: c.vpd_rev.clone(),
        },
        sensors: v1::Sensors {
            overrides: c.sensors.clone(),
            ambient_c: Some(c.ambient_c),
        },
        dump: v1::Dump {
            dir: c.dump_dir.clone(),
            archive_id: Some(c.dump_archive_id.clone()),
        },
        trace: v1::Trace {
            enabled: Some(c.trace),
            from: c.trace_from,
            to: c.trace_to,
            rot_from: c.rot_trace_from,
            rot_to: c.rot_trace_to,
            rotpc: c.rotpc,
            rotdump: c.rotdump.map(|(a, l)| format!("{a:#x}:{l}")),
            watch: c.watch,
            diff: c.diff.clone(),
            pcprof: Some(c.pcprof),
        },
        stats: v1::Stats {
            rx: Some(c.rxstats),
            rtt: Some(c.rttstats),
            pump: Some(c.pumpstats),
            pump_ms: Some(c.pumpstats_ms),
        },
        debug: v1::Debug {
            no_debug: Some(c.no_debug),
            no_archive_warn: Some(c.no_archive_warn),
            swd_trigger: Some(c.swd_trigger),
            jtag_trigger: Some(c.jtag_trigger),
            swd_trace: Some(c.swd_trace),
            rotsvc: Some(c.rotsvc),
            pingtest: Some(c.pingtest),
            flash: Some(c.flashdbg),
            rotflash: Some(c.rotflashdbg),
            eth: Some(c.ethdbg),
            uart: Some(c.uartdbg),
            bridge: Some(c.bridgedbg),
            puf: Some(c.pufdbg),
            vsc: Some(c.vscdbg),
            rx: Some(c.rxdbg),
            mdio: Some(c.mdiodbg),
            vpd: Some(c.vpddbg),
            spi: Some(c.spidbg),
            panic: Some(c.panicdbg),
            svc: Some(c.svcdbg),
            exc: Some(c.excdbg),
            sprot: Some(c.sprotdbg),
            couple: Some(c.coupledbg),
            rom: Some(c.romdbg),
            config: Some(c.configdbg),
        },
    }
}

fn board_str(b: Board) -> &'static str {
    match b {
        Board::Gimlet => "gimlet",
        Board::Sidecar => "sidecar",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_set_knobs_serialize() {
        let mut ext = ConfigFileV1 {
            schema_version: Some(CURRENT),
            ..Default::default()
        };
        ext.rot.flash = Some("r.bin".into());
        let text = to_toml(&ext).unwrap();
        assert!(text.contains("[rot]"));
        assert!(text.contains("flash = \"r.bin\""));
        // Nothing was set in these, so they are omitted.
        assert!(!text.contains("[vpd]"));
        assert!(!text.contains("[debug]"));
    }

    #[test]
    fn dump_then_reload_is_idempotent() {
        // A resolved config, dumped and read back, ingests to the same values.
        let original = ingest(
            toml::from_str(
                "[op]\nboard = \"sidecar\"\nmode = \"gdb\"\n\
                 [rot]\nflash = \"r.bin\"\n[trace]\nrotdump = \"0x20000000:256\"\n",
            )
            .unwrap(),
        )
        .unwrap();
        let text = dump(&original).unwrap();
        let reloaded = crate::migrate::migrate(&text).and_then(ingest).unwrap();
        assert_eq!(reloaded.board(), original.board());
        assert_eq!(reloaded.mode(), original.mode());
        assert_eq!(reloaded.rot_flash(), original.rot_flash());
        assert_eq!(reloaded.rotdump(), original.rotdump());
        assert_eq!(reloaded.eth_quantum(), original.eth_quantum());
    }

    #[test]
    fn template_is_a_documented_valid_versioned_file_at_defaults() {
        let text = template();
        assert!(text.contains("schema_version = 1"));
        // The banner and a section comment document the file.
        assert!(text.contains("# sp-emu configuration"));
        assert!(text.contains("# Root of Trust."));
        // Optional knobs are surfaced as commented example lines, not omitted.
        assert!(text.contains("# mode = "));
        assert!(text.contains("# rot_from = "));
        // The template, comments included, migrates and ingests to the defaults.
        let cfg = crate::migrate::migrate(&text).and_then(ingest).unwrap();
        assert_eq!(cfg.flash_path(), "sp-flash.bin");
        assert_eq!(cfg.eth_quantum(), 4096);
        assert!(cfg.eth_txbreak());
    }

    /// The template must list every knob (active or commented), so a knob added to
    /// the schema cannot be silently absent from `config schema`. Counts the
    /// `key = ...` lines and compares against the knob count; section headers and
    /// prose comments do not match.
    #[test]
    fn template_lists_every_knob() {
        let knob_lines = template()
            .lines()
            .filter(|l| {
                let body = l.strip_prefix("# ").unwrap_or(l);
                body.split_once(" = ").is_some_and(|(k, _)| {
                    !k.is_empty()
                        && k.bytes()
                            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'_')
                })
            })
            .count();
        // One line per knob (the flat ENV_NAMES set), plus the schema_version line.
        assert_eq!(
            knob_lines,
            crate::ENV_NAMES.len() + 1,
            "template knob count drifted from the schema; add or remove a knob line"
        );
    }

    /// Uncommenting every optional example line must still parse as a valid v1
    /// file: it guards the commented lines' keys and example values from drifting
    /// away from the schema (a bad key or value would fail `deny_unknown_fields`
    /// or ingest).
    #[test]
    fn template_optional_examples_are_valid_when_enabled() {
        let enabled: String = template()
            .lines()
            .map(|l| {
                // Turn `# key = value  # hint` into `key = value`; leave section
                // comments (`# Foo.`) and already-active lines alone.
                let t = l.trim_start_matches("# ");
                if l.starts_with("# ") && t.contains(" = ") {
                    t.split("  #").next().unwrap_or(t).trim_end()
                } else if l.starts_with('#') {
                    "" // a prose comment line
                } else {
                    l
                }
                .to_string()
            })
            .collect::<Vec<_>>()
            .join("\n");
        crate::migrate::migrate(&enabled)
            .and_then(ingest)
            .expect("every optional example is a valid knob and value");
    }
}
