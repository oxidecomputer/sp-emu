//! Turn the external schema into the validated [`Config`].
//!
//! This is the "validate" half of parse-don't-validate: every value that needs
//! checking is checked here (enum spellings, the compound `trace.rotdump`
//! string, positive-only counts), and every always-present field is given its
//! default. The typed file is held to a strict standard: an unknown board or
//! mode is an error, not a silent fallback. Legacy leniency, where a stray value
//! coerced to a default, lives in the v0 to v1 migration, not here.

use crate::config::{Board, Config};
use crate::error::ConfigError;
use crate::schema::v1::ConfigFileV1;

/// Validate an external v1 config into the internal [`Config`].
pub fn ingest(ext: ConfigFileV1) -> Result<Config, ConfigError> {
    let ConfigFileV1 {
        schema_version: _,
        op,
        paths,
        net,
        host,
        i2c,
        rot,
        sprot,
        vpd,
        sensors,
        dump,
        trace,
        stats,
        debug,
    } = ext;

    Ok(Config {
        // paths
        flash_path: paths.flash.unwrap_or_else(|| "sp-flash.bin".into()),
        rot_nvm_path: paths.rot_nvm.unwrap_or_else(|| "sp-rot-flash.bin".into()),
        identity_path: paths.identity.unwrap_or_else(|| "sp-emu-identity".into()),
        state_dir: paths.state_dir,
        archive: paths.archive,

        // operation
        seed: op.seed,
        mode: one_of(op.mode, "op.mode", &["run", "gdb"])?,
        boot_slot: one_of(op.slot, "op.slot", &["a", "b"])?,
        run_max: op.run_max,
        board: board(op.board)?,
        ignition: op
            .ignition
            .unwrap_or_else(|| "0:gimlet,1:sidecar,2:gimlet,3:gimlet".into()),

        // host bridge + Ethernet
        bridge: net.bridge,
        well_known_ports: net.well_known_ports.unwrap_or(false),
        addr0: net.addr0,
        addr1: net.addr1,
        vid0: net.vid0,
        vid1: net.vid1,
        eth_quantum: net.eth_quantum.filter(|&q| q > 0).unwrap_or(4096),
        eth_txbreak: net.eth_txbreak.unwrap_or(true),
        idle_ms: net.idle_ms.unwrap_or(10),

        // host UART / IPCC
        host_uart: host.uart,
        host_pty: host.pty.unwrap_or(false),

        // companion I2C bridge (empty treated as unset)
        i2c_bridge: nonempty(i2c.bridge),
        i2c_device: nonempty(i2c.device),

        // RoT
        rot_rom: rot.rom.unwrap_or(false),
        rot_fresh: rot.fresh.unwrap_or(false),
        rot_measure: rot.measure.unwrap_or(false),
        rot_service: nonempty(rot.service),
        rot_flash: rot.flash,
        rot_bootleby: rot.bootleby,
        rot_no_bootleby: rot.no_bootleby.unwrap_or(false),
        rot_cmpa: rot.cmpa,
        rot_cfpa: rot.cfpa,
        rot_nmpa: rot.nmpa,
        rot_image_b: rot.image_b,
        rot_erase_a: rot.erase_a.unwrap_or(false),
        rot_boot_pref: one_of(rot.boot_pref, "rot.boot_pref", &["a", "b"])?,
        rot_dice: rot.dice,
        rot_preboot: rot.preboot,

        // SP <-> RoT coupling
        sprot_flowctl: sprot.flowctl.unwrap_or(16),
        sprot_couple: sprot.couple.unwrap_or(true),
        endoscope_couple: sprot.endoscope_couple.unwrap_or(true),
        sp_clock_khz: sprot.sp_clock_khz.filter(|&k| k > 0).unwrap_or(400_000),

        // VPD identity (empty treated as unset)
        vpd_serial: nonempty(vpd.serial),
        vpd_part: nonempty(vpd.part),
        vpd_rev: nonempty(vpd.rev),

        // sensors
        sensors: sensors.overrides,
        ambient_c: sensors.ambient_c.unwrap_or(30.0),

        // hydrate RAM dump
        dump_dir: dump.dir,
        dump_archive_id: dump.archive_id.unwrap_or_default(),

        // traces / windows / profiling
        trace: trace.enabled.unwrap_or(false),
        trace_from: trace.from,
        trace_to: trace.to,
        rot_trace_from: trace.rot_from,
        rot_trace_to: trace.rot_to,
        rotpc: trace.rotpc,
        rotdump: rotdump(trace.rotdump)?,
        watch: trace.watch,
        diff: trace.diff,
        pcprof: trace.pcprof.unwrap_or(false),

        // periodic stats
        rxstats: stats.rx.unwrap_or(false),
        rttstats: stats.rtt.unwrap_or(false),
        pumpstats: stats.pump.unwrap_or(false),
        pumpstats_ms: stats.pump_ms.unwrap_or(50),

        // per-subsystem log toggles + one-shots
        no_debug: debug.no_debug.unwrap_or(false),
        no_archive_warn: debug.no_archive_warn.unwrap_or(false),
        swd_trigger: debug.swd_trigger.unwrap_or(false),
        jtag_trigger: debug.jtag_trigger.unwrap_or(false),
        swd_trace: debug.swd_trace.unwrap_or(false),
        rotsvc: debug.rotsvc.unwrap_or(false),
        pingtest: debug.pingtest.unwrap_or(false),
        flashdbg: debug.flash.unwrap_or(false),
        rotflashdbg: debug.rotflash.unwrap_or(false),
        ethdbg: debug.eth.unwrap_or(false),
        uartdbg: debug.uart.unwrap_or(false),
        bridgedbg: debug.bridge.unwrap_or(false),
        pufdbg: debug.puf.unwrap_or(false),
        vscdbg: debug.vsc.unwrap_or(false),
        rxdbg: debug.rx.unwrap_or(false),
        mdiodbg: debug.mdio.unwrap_or(false),
        vpddbg: debug.vpd.unwrap_or(false),
        spidbg: debug.spi.unwrap_or(false),
        panicdbg: debug.panic.unwrap_or(false),
        svcdbg: debug.svc.unwrap_or(false),
        excdbg: debug.exc.unwrap_or(false),
        sprotdbg: debug.sprot.unwrap_or(false),
        coupledbg: debug.couple.unwrap_or(false),
        romdbg: debug.rom.unwrap_or(false),
        configdbg: debug.config.unwrap_or(false),
    })
}

/// Drop an empty string to `None`, matching the historical env leniency for the
/// knobs that treat "" as unset.
fn nonempty(o: Option<String>) -> Option<String> {
    o.filter(|s| !s.is_empty())
}

/// Accept only one of `allowed` (after dropping an empty string); anything else
/// is a validation error naming the offending value.
fn one_of(o: Option<String>, path: &str, allowed: &[&str]) -> Result<Option<String>, ConfigError> {
    match nonempty(o) {
        None => Ok(None),
        Some(s) if allowed.contains(&s.as_str()) => Ok(Some(s)),
        Some(s) => Err(ConfigError::invalid(
            path,
            format!("expected one of {allowed:?}, got {s:?}"),
        )),
    }
}

/// `Some("gimlet")`/`Some("sidecar")` map to the board; unset defaults to
/// gimlet; anything else is rejected (strict, unlike the lenient env path).
fn board(o: Option<String>) -> Result<Board, ConfigError> {
    match o.as_deref() {
        None | Some("gimlet") => Ok(Board::Gimlet),
        Some("sidecar") => Ok(Board::Sidecar),
        Some(other) => Err(ConfigError::invalid(
            "op.board",
            format!("expected \"gimlet\" or \"sidecar\", got {other:?}"),
        )),
    }
}

/// Parse the `rotdump` window `"0xADDR:LEN"` (hex address, decimal length). The
/// single source of the format, shared with the flat v0 bridge's leniency check.
pub(crate) fn parse_rotdump(s: &str) -> Option<(u32, u32)> {
    let (a, l) = s.split_once(':')?;
    let addr = u32::from_str_radix(a.trim().trim_start_matches("0x"), 16).ok()?;
    let len: u32 = l.trim().parse().ok()?;
    Some((addr, len))
}

/// Validate `trace.rotdump` strictly: a malformed window is an error.
fn rotdump(o: Option<String>) -> Result<Option<(u32, u32)>, ConfigError> {
    let Some(s) = o else {
        return Ok(None);
    };
    parse_rotdump(&s).map(Some).ok_or_else(|| {
        ConfigError::invalid(
            "trace.rotdump",
            format!("expected \"0xADDR:LEN\", got {s:?}"),
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_file_yields_the_documented_defaults() {
        let c = ingest(ConfigFileV1::default()).unwrap();
        assert_eq!(c.flash_path(), "sp-flash.bin");
        assert_eq!(c.rot_nvm_path(), "sp-rot-flash.bin");
        assert_eq!(c.identity_path(), "sp-emu-identity");
        assert_eq!(c.board(), Board::Gimlet);
        assert_eq!(c.ignition(), "0:gimlet,1:sidecar,2:gimlet,3:gimlet");
        assert_eq!(c.eth_quantum(), 4096);
        assert!(c.eth_txbreak());
        assert!(c.sprot_couple());
        assert!(c.endoscope_couple());
        assert_eq!(c.idle_ms(), 10);
        assert_eq!(c.sprot_flowctl(), 16);
        assert_eq!(c.sp_clock_khz(), 400_000);
        assert_eq!(c.pumpstats_ms(), 50);
        assert_eq!(c.ambient_c(), 30.0);
        assert_eq!(c.mode(), None);
        assert!(!c.ethdbg());
    }

    fn parse(text: &str) -> Result<Config, ConfigError> {
        ingest(toml::from_str(text).unwrap())
    }

    #[test]
    fn set_values_reach_the_getters() {
        let c = parse(
            "[op]\nmode = \"run\"\nslot = \"b\"\nboard = \"sidecar\"\n\
             [rot]\nflash = \"rot.bin\"\nboot_pref = \"b\"\n\
             [trace]\nrotdump = \"0x20000000:256\"\n\
             [debug]\neth = true\n",
        )
        .unwrap();
        assert_eq!(c.mode(), Some("run"));
        assert_eq!(c.boot_slot(), Some("b"));
        assert_eq!(c.board(), Board::Sidecar);
        assert_eq!(c.rot_flash(), Some("rot.bin"));
        assert_eq!(c.rot_boot_pref(), Some("b"));
        assert_eq!(c.rotdump(), Some((0x2000_0000, 256)));
        assert!(c.ethdbg());
    }

    #[test]
    fn a_bad_board_is_rejected() {
        assert!(parse("[op]\nboard = \"gymlet\"\n").is_err());
    }

    #[test]
    fn a_bad_mode_is_rejected() {
        assert!(parse("[op]\nmode = \"walk\"\n").is_err());
    }

    #[test]
    fn a_bad_slot_is_rejected() {
        assert!(parse("[op]\nslot = \"c\"\n").is_err());
    }

    #[test]
    fn a_malformed_rotdump_is_rejected() {
        assert!(parse("[trace]\nrotdump = \"not-an-address\"\n").is_err());
    }

    #[test]
    fn empty_strings_are_treated_as_unset() {
        let c = parse("[vpd]\nserial = \"\"\n[op]\nmode = \"\"\n").unwrap();
        assert_eq!(c.vpd_serial(), None);
        assert_eq!(c.mode(), None);
    }

    #[test]
    fn zero_quantum_falls_back_to_the_default() {
        let c = parse("[net]\neth_quantum = 0\n").unwrap();
        assert_eq!(c.eth_quantum(), 4096);
    }
}
