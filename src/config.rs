//! Central configuration: ingest every `SP_EMU_*` input (the environment, layered
//! over a config file) exactly once, resolve it, persist it, and store it. After
//! `init`, no other module reads the environment; they call `config::get()`.
//!
//! This is the ingestion half of the design in the tracker (spemu-dxw): a single
//! CLI wrapper sources and vets config; the rest of the program consumes a typed,
//! immutable `Config`.
//!
//! Every knob is declared once in the `config!` table below -- its field, type,
//! environment variable, default, and parser all live on a single line. The macro
//! generates the struct, the resolver, and the renderer from that one list, so they
//! can't drift. Adding a knob is one row; reading it is `config::get().<field>`. A
//! guard test (`env_reads_confined_to_config_module`) enforces that no other module
//! reads the environment directly.
//!
//! Sources and precedence: flag > (config file | environment) > default. The
//! environment and a config file are two *alternative* sources, never stacked:
//! `--load-config <path>` reads all `SP_EMU_*` settings from a flat
//! `SP_EMU_NAME = "value"` TOML table and ignores the environment, so a saved
//! configuration reproduces exactly; without it, the environment is read as usual.
//! `--dump-config <path>` writes the effective configuration back for re-loading, so a
//! run round-trips. See the `TODO` on `to_toml` for the eventual strongly-typed schema.
//!
//! Backward compatibility: this is a mechanical move of the existing reads. Every
//! variable keeps its name, default, and *leniency* -- a value (or typo) that
//! silently worked before still resolves the same way, so no existing environment
//! or flag usage changes.

use anyhow::Result;
use std::sync::OnceLock;

/// Which board the emulated SoC models.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Board {
    Gimlet,
    Sidecar,
}

impl Board {
    pub fn is_sidecar(self) -> bool {
        self == Board::Sidecar
    }
}

/// Declare the whole configuration as one table. Each row is
///
/// ```text
/// field: Type = "SP_EMU_VAR" => |raw| <expr resolving `raw: Option<String>` to Type>,
/// ```
///
/// `raw` is the resolved input value (`None` when the variable is set in neither the
/// environment nor the config file). The seed row may also read `seed_override` (the
/// `--seed` CLI flag), which is in scope in the resolver. From this list the macro
/// generates the `Config` struct, `resolve` (the single reader, parameterized over a
/// `get` closure so tests can inject a source), and `render` (one line per knob with
/// a set/default marker). A function would need the field list repeated in each of
/// those three places; the macro keeps it in exactly one.
macro_rules! config {
    (
        seed = $seed:ident,
        $(
            $field:ident : $ty:ty = $env:literal => |$raw:ident| $resolve:expr
        ),* $(,)?
    ) => {
        /// The resolved, vetted configuration for this process. Read-only after
        /// `init`; every field is resolved before any subsystem is built.
        #[derive(Clone, Debug)]
        pub struct Config {
            $( pub $field: $ty, )*
            /// The raw `(SP_EMU_NAME, value)` inputs that were explicitly present
            /// (from the environment or a config file), in declaration order. Drives
            /// the `set`/`default` marker and the round-trippable config file.
            inputs: Vec<(&'static str, String)>,
        }

        impl Config {
            /// The single input reader, parameterized over `get` so production reads
            /// the environment layered over a config file, while tests inject a map
            /// (hermetic). `get` returns the raw string for a variable, or `None`.
            fn resolve(
                get: &dyn Fn(&str) -> Option<String>,
                $seed: Option<String>,
            ) -> Config {
                let _ = &$seed; // read by the seed row; silence if that row is removed
                let mut inputs: Vec<(&'static str, String)> = Vec::new();
                $(
                    let $field: $ty = {
                        let $raw: Option<String> = get($env);
                        if let Some(ref val) = $raw {
                            inputs.push(($env, val.clone()));
                        }
                        $resolve
                    };
                )*
                Config { $( $field, )* inputs }
            }

            /// Every `SP_EMU_*` variable the table resolves, in declaration order. Used
            /// by the backward-compatibility test to assert no historical knob is dropped.
            #[cfg(test)]
            const ENV_VARS: &'static [&'static str] = &[$( $env, )*];

            /// Render every resolved knob (one `NAME = value (set|default)` line) for
            /// the opt-in stderr dump, so the full resolved state is visible on request.
            fn render(&self) -> String {
                let mut out = String::new();
                $(
                    out.push_str(&format!(
                        "{:<24} = {:?} ({})\n",
                        $env,
                        self.$field,
                        if self.is_set($env) { "set" } else { "default" },
                    ));
                )*
                out
            }
        }
    };
}

config! {
    seed = seed_override,

    // ---- state file paths (written to the working directory by default) ----
    flash_path: String = "SP_EMU_FLASH" => |v| v.unwrap_or_else(|| "sp-flash.bin".to_string()),
    rot_nvm_path: String = "SP_EMU_ROT_NVM" => |v| v.unwrap_or_else(|| "sp-rot-flash.bin".to_string()),
    identity_path: String = "SP_EMU_IDENTITY" => |v| v.unwrap_or_else(|| "sp-emu-identity".to_string()),

    // ---- instance identity (the --seed flag wins over $SP_EMU_SEED) ----
    seed: Option<String> = "SP_EMU_SEED" => |v| seed_override.clone().or(v),

    // ---- SoC selection: `sidecar` iff exactly "sidecar", else gimlet (lenient) ----
    board: Board = "SP_EMU_BOARD" => |v| {
        if v.as_deref() == Some("sidecar") { Board::Sidecar } else { Board::Gimlet }
    },

    // ---- behavior toggles (presence = on) ----
    // Emulate the LPC55 boot-ROM signature API (skboot_authenticate) so the RoT
    // pre-kernel's authenticate_image() runs for real. Off by default: keeps the
    // fast direct-boot path unchanged (spemu-z89).
    rot_rom: bool = "SP_EMU_ROT_ROM" => |v| v.is_some(),
    // Ignore any persisted RoT flash and re-seed from scratch this run (removes doubt
    // about whether persistent state is in use). Default off = backward compatible.
    rot_fresh: bool = "SP_EMU_ROT_FRESH" => |v| v.is_some(),
    rot_measure: bool = "SP_EMU_ROT_MEASURE" => |v| v.is_some(),
    well_known_ports: bool = "SP_EMU_WELL_KNOWN_PORTS" => |v| v.is_some(),
    no_debug: bool = "SP_EMU_NO_DEBUG" => |v| v.is_some(),
    host_pty: bool = "SP_EMU_HOST_PTY" => |v| v.is_some(),
    swd_trigger: bool = "SP_EMU_SWD_TRIGGER" => |v| v.is_some(),
    trace: bool = "SP_EMU_TRACE" => |v| v.is_some(),

    // ---- per-subsystem diagnostic tracing (presence = on) ----
    flashdbg: bool = "SP_EMU_FLASHDBG" => |v| v.is_some(),
    rotflashdbg: bool = "SP_EMU_ROTFLASHDBG" => |v| v.is_some(),
    ethdbg: bool = "SP_EMU_ETHDBG" => |v| v.is_some(),
    uartdbg: bool = "SP_EMU_UARTDBG" => |v| v.is_some(),
    bridgedbg: bool = "SP_EMU_BRIDGEDBG" => |v| v.is_some(),
    pufdbg: bool = "SP_EMU_PUFDBG" => |v| v.is_some(),
    vscdbg: bool = "SP_EMU_VSCDBG" => |v| v.is_some(),
    swd_trace: bool = "SP_EMU_SWD_TRACE" => |v| v.is_some(),
    rotsvc: bool = "SP_EMU_ROTSVC" => |v| v.is_some(),
    pingtest: bool = "SP_EMU_PINGTEST" => |v| v.is_some(),
    rxstats: bool = "SP_EMU_RXSTATS" => |v| v.is_some(),
    rttstats: bool = "SP_EMU_RTTSTATS" => |v| v.is_some(),
    pumpstats: bool = "SP_EMU_PUMPSTATS" => |v| v.is_some(),
    pcprof: bool = "SP_EMU_PCPROF" => |v| v.is_some(),
    // hot-path debug flags (exposed as thin accessors in dbg.rs)
    rxdbg: bool = "SP_EMU_RXDBG" => |v| v.is_some(),
    mdiodbg: bool = "SP_EMU_MDIODBG" => |v| v.is_some(),
    vpddbg: bool = "SP_EMU_VPDDBG" => |v| v.is_some(),
    spidbg: bool = "SP_EMU_SPIDBG" => |v| v.is_some(),
    panicdbg: bool = "SP_EMU_PANICDBG" => |v| v.is_some(),
    svcdbg: bool = "SP_EMU_SVCDBG" => |v| v.is_some(),
    excdbg: bool = "SP_EMU_EXCDBG" => |v| v.is_some(),
    sprotdbg: bool = "SP_EMU_SPROTDBG" => |v| v.is_some(),
    romdbg: bool = "SP_EMU_ROMDBG" => |v| v.is_some(), // boot-ROM API calls (skboot)
    // print the full resolved config table to stderr
    configdbg: bool = "SP_EMU_CONFIGDBG" => |v| v.is_some(),

    // ---- optional selectors / overrides (None when unset) ----
    host_uart: Option<String> = "SP_EMU_HOST_UART" => |v| v,
    // addr; empty treated as unset
    rot_service: Option<String> = "SP_EMU_ROT_SERVICE" => |v| v.filter(|s| !s.is_empty()),
    rot_flash: Option<String> = "SP_EMU_ROT_FLASH" => |v| v,
    // Path to a real bootleby image: load it at flash base 0x0 and boot IT (secure
    // aliases + boot-ROM API on), so bootleby does genuine A/B selection (spemu-kx3).
    rot_bootleby: Option<String> = "SP_EMU_ROT_BOOTLEBY" => |v| v,
    // Real device CMPA/CFPA pages (512 bytes each) to seed instead of the synthesized
    // ones, so real bootleby's PFR validation passes (spemu-kx3).
    rot_cmpa: Option<String> = "SP_EMU_ROT_CMPA" => |v| v,
    rot_cfpa: Option<String> = "SP_EMU_ROT_CFPA" => |v| v,
    // A slot-B image (flash.b, 0x50000) to seed alongside slot A, so real bootleby
    // can perform genuine A/B selection. Absent => slot B left erased/invalid (spemu-kzi).
    rot_image_b: Option<String> = "SP_EMU_ROT_IMAGE_B" => |v| v,
    // Leave slot A (flash.a, 0x10000) erased instead of seeding the passed image, to
    // drive bootleby's B-only and neither(panic) selection cases (spemu-kzi).
    rot_erase_a: bool = "SP_EMU_ROT_ERASE_A" => |v| v.is_some(),
    // Persistent CFPA boot preference for the synthesized CFPA: "b" prefers slot B,
    // otherwise slot A. Ignored when SP_EMU_ROT_CFPA overrides the page (spemu-kzi).
    rot_boot_pref: Option<String> = "SP_EMU_ROT_BOOT_PREF" => |v| v,
    rot_dice: Option<String> = "SP_EMU_ROT_DICE" => |v| v,
    archive: Option<String> = "SP_EMU_ARCHIVE" => |v| v,
    diff: Option<String> = "SP_EMU_DIFF" => |v| v,
    dump_dir: Option<String> = "SP_EMU_DUMP_DIR" => |v| v,
    sensors: Option<String> = "SP_EMU_SENSORS" => |v| v,
    watch: Option<u32> = "SP_EMU_WATCH" => |v| {
        v.and_then(|s| u32::from_str_radix(s.trim_start_matches("0x"), 16).ok())
    },
    // "0xADDR:LEN"
    rotdump: Option<(u32, u32)> = "SP_EMU_ROTDUMP" => |v| v.and_then(|s| {
        let (a, l) = s.split_once(':')?;
        Some((u32::from_str_radix(a.trim_start_matches("0x"), 16).ok()?, l.parse().ok()?))
    }),
    // the caller applies its own default (gdb 400M, rot-service 40M)
    rot_preboot: Option<u64> = "SP_EMU_ROT_PREBOOT" => |v| v.and_then(|s| s.parse().ok()),
    // raw: parsed three ways (MGS addr, gdb port offset, soc index)
    bridge: Option<String> = "SP_EMU_BRIDGE" => |v| v,

    // ---- well-known-port host bridge (each caller applies its own default) ----
    addr0: Option<String> = "SP_EMU_ADDR0" => |v| v,
    addr1: Option<String> = "SP_EMU_ADDR1" => |v| v,
    vid0: Option<u16> = "SP_EMU_VID0" => |v| {
        v.and_then(|s| u16::from_str_radix(s.trim_start_matches("0x"), 16).ok())
    },
    vid1: Option<u16> = "SP_EMU_VID1" => |v| {
        v.and_then(|s| u16::from_str_radix(s.trim_start_matches("0x"), 16).ok())
    },

    // ---- companion I2C bridge (empty treated as unset) ----
    i2c_bridge: Option<String> = "SP_EMU_I2C_BRIDGE" => |v| v.filter(|s| !s.is_empty()),
    i2c_device: Option<String> = "SP_EMU_I2C_DEVICE" => |v| v.filter(|s| !s.is_empty()),

    // ---- windowed instruction traces (diagnostics; None = off) ----
    trace_from: Option<u64> = "SP_EMU_TRACE_FROM" => |v| v.and_then(|s| s.parse().ok()),
    trace_to: Option<u64> = "SP_EMU_TRACE_TO" => |v| v.and_then(|s| s.parse().ok()),
    rot_trace_from: Option<u32> = "SP_EMU_ROT_TRACE_FROM" => |v| {
        v.and_then(|s| u32::from_str_radix(s.trim_start_matches("0x"), 16).ok())
    },
    rot_trace_to: Option<u32> = "SP_EMU_ROT_TRACE_TO" => |v| {
        v.and_then(|s| u32::from_str_radix(s.trim_start_matches("0x"), 16).ok())
    },
    rotpc: Option<u64> = "SP_EMU_ROTPC" => |v| v.and_then(|s| s.parse().ok()),

    // ---- values with a default ----
    idle_ms: u64 = "SP_EMU_IDLE_MS" => |v| v.and_then(|s| s.parse().ok()).unwrap_or(10),
    eth_quantum: u32 = "SP_EMU_ETH_QUANTUM" => |v| {
        v.and_then(|s| s.parse().ok()).filter(|&q| q > 0).unwrap_or(4096)
    },
    // on by default; "0" disables
    eth_txbreak: bool = "SP_EMU_ETH_TXBREAK" => |v| v.map(|s| s != "0").unwrap_or(true),
    pumpstats_ms: u64 = "SP_EMU_PUMPSTATS_MS" => |v| v.and_then(|s| s.parse().ok()).unwrap_or(50),
    // trim before parsing, matching the historical read (a padded value parsed)
    ambient_c: f32 = "SP_EMU_AMBIENT_C" => |v| v.and_then(|s| s.trim().parse().ok()).unwrap_or(30.0),
    ignition: String = "SP_EMU_IGNITION" => |v| {
        v.unwrap_or_else(|| "0:gimlet,1:sidecar,2:gimlet,3:gimlet".to_string())
    },
    dump_archive_id: String = "SP_EMU_DUMP_ARCHIVE_ID" => |v| v.unwrap_or_default(),
}

/// Meta variables `--dump-config` does not persist: `SP_EMU_CONFIGDBG` is a debug
/// toggle about config printing, not persistent state, so loading a dumped file must
/// not silently re-enable it.
const NOT_PERSISTED: &[&str] = &["SP_EMU_CONFIGDBG"];

impl Config {
    /// Resolve from the process environment only (no config file). Used by `get()`'s
    /// lazy default; `init` uses [`Config::resolve`] with a file layer when one is
    /// loaded. `seed_override` is the `--seed` flag, which wins over the environment.
    pub fn from_env(seed_override: Option<String>) -> Result<Config> {
        Ok(Self::resolve(&|k| std::env::var(k).ok(), seed_override))
    }

    /// Was this variable explicitly provided (by the environment or a loaded file)?
    fn is_set(&self, name: &str) -> bool {
        self.inputs.iter().any(|(n, _)| *n == name)
    }

    /// Serialize the explicitly-set variables as a round-trippable TOML config file:
    /// a flat table keyed by `SP_EMU_*` name, mirroring the environment. Loading it
    /// back (`--load-config`) reproduces the run; omitted variables take their default.
    ///
    /// TODO: move to a strongly-typed config schema (native TOML types, real booleans,
    /// nested tables) once the `SP_EMU_*` environment variables can be deprecated. The
    /// env-var-mirroring string map exists only to stay backward compatible with them;
    /// with the env layer gone, resolution can key off typed fields directly rather
    /// than presence-based strings. Tracked in spemu-dxw.
    fn to_toml(&self) -> String {
        let mut out = String::from(
            "# sp-emu configuration (mirrors $SP_EMU_*; the environment overrides it).\n\
             # Load with `--load-config <this file>`; omitted variables use their default.\n\
             # A flag is on by presence -- delete its line to disable it.\n\n",
        );
        for (name, val) in &self.inputs {
            if NOT_PERSISTED.contains(name) {
                continue;
            }
            out.push_str(&format!("{name} = \"{}\"\n", escape_toml(val)));
        }
        out
    }
}

/// Escape a value for a TOML basic string (`"..."`).
fn escape_toml(s: &str) -> String {
    s.replace('\\', "\\\\")
        .replace('"', "\\\"")
        .replace('\n', "\\n")
}

/// Load the config file as `(SP_EMU_NAME, value)` pairs, lenient: a missing file is
/// empty, a malformed file warns and is skipped, and scalar values are coerced to the
/// string the resolver expects (a boolean `false` means "unset" -- a flag left off).
fn load_config_file(path: &str) -> Vec<(String, String)> {
    let text = match std::fs::read_to_string(path) {
        Ok(t) => t,
        Err(_) => return Vec::new(), // absent file: no file layer
    };
    parse_config_toml(&text).unwrap_or_else(|e| {
        eprintln!("[config] ignoring malformed {path}: {e}");
        Vec::new()
    })
}

fn parse_config_toml(text: &str) -> std::result::Result<Vec<(String, String)>, toml::de::Error> {
    let table: toml::Table = text.parse()?;
    let mut out = Vec::new();
    for (k, v) in table {
        let s = match v {
            toml::Value::String(s) => s,
            toml::Value::Integer(i) => i.to_string(),
            toml::Value::Float(f) => f.to_string(),
            toml::Value::Boolean(true) => "1".to_string(),
            toml::Value::Boolean(false) => continue, // presence flag left off
            _ => continue,                           // arrays/tables/datetimes: not a knob
        };
        out.push((k, s));
    }
    Ok(out)
}

static CONFIG: OnceLock<Config> = OnceLock::new();

/// Resolve and store the process configuration. Call once, early in `main`, before
/// any subsystem is built.
///
/// sp-emu takes its `SP_EMU_*` settings from the environment -- or, with the
/// `--load-config <path>` flag, from a TOML config file *instead of* the environment,
/// so a saved configuration reproduces exactly regardless of the shell. The two
/// sources are never mixed; command-line flags always win. Precedence is
/// flag > (config file | environment) > default. `--dump-config <path>` writes the
/// effective configuration for later re-loading. The full resolved table is echoed to
/// stderr only under `$SP_EMU_CONFIGDBG`.
pub fn init(
    seed_override: Option<String>,
    load_config: Option<String>,
    dump_config: Option<String>,
) -> Result<()> {
    // Two alternative sources for SP_EMU_*, never stacked: a config file
    // (--load-config, which then ignores the environment entirely) or the environment.
    let cfg = match &load_config {
        Some(path) => {
            let file = load_config_file(path);
            eprintln!(
                "[config] loaded {} ({} vars); ignoring the SP_EMU_* environment",
                path,
                file.len()
            );
            let get = |k: &str| file.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
            Config::resolve(&get, seed_override)
        }
        None => Config::from_env(seed_override)?,
    };

    if let Some(path) = &dump_config {
        match std::fs::write(path, cfg.to_toml()) {
            Ok(()) => eprintln!("[config] wrote {} ({} set)", path, cfg.inputs.len()),
            Err(e) => eprintln!("[config] writing {path} failed: {e}"),
        }
    }
    if cfg.configdbg {
        eprint!("{}", cfg.render());
    }
    // Loud, not silent: if this fails, `get()` already resolved a config (without
    // the seed override) before `init` ran, so the process would run mis-seeded.
    CONFIG.set(cfg).map_err(|_| {
        anyhow::anyhow!("config::init called after config was already resolved by get()")
    })?;
    Ok(())
}

/// The process configuration. Lazily defaults from a clean environment if `init`
/// was never called (unit tests / non-CLI paths), so accessors never panic.
pub fn get() -> &'static Config {
    CONFIG.get_or_init(|| Config::from_env(None).expect("default config resolves"))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Resolve from an in-memory map instead of the process environment, so tests
    /// are hermetic regardless of the developer's / CI's ambient `SP_EMU_*`.
    fn from_map(vars: &[(&str, &str)], seed_override: Option<String>) -> Config {
        Config::resolve(
            &|k| {
                vars.iter()
                    .find(|(n, _)| *n == k)
                    .map(|(_, val)| val.to_string())
            },
            seed_override,
        )
    }

    #[test]
    fn defaults_resolve() {
        let c = from_map(&[], None);
        assert_eq!(c.flash_path, "sp-flash.bin");
        assert_eq!(c.rot_nvm_path, "sp-rot-flash.bin");
        assert_eq!(c.identity_path, "sp-emu-identity");
        assert_eq!(c.board, Board::Gimlet);
        assert!(!c.flashdbg);
        assert_eq!(c.idle_ms, 10);
        assert_eq!(c.eth_quantum, 4096);
        assert!(c.eth_txbreak);
        assert_eq!(c.ambient_c, 30.0);
    }

    #[test]
    fn seed_override_wins() {
        // --seed beats $SP_EMU_SEED.
        let c = from_map(&[("SP_EMU_SEED", "from-env")], Some("from-cli".into()));
        assert_eq!(c.seed.as_deref(), Some("from-cli"));
        // absent flag falls back to the environment.
        let c = from_map(&[("SP_EMU_SEED", "from-env")], None);
        assert_eq!(c.seed.as_deref(), Some("from-env"));
    }

    #[test]
    fn value_parsers_preserve_leniency() {
        // whitespace-padded ambient parses (historical .trim())
        assert_eq!(
            from_map(&[("SP_EMU_AMBIENT_C", " 42 ")], None).ambient_c,
            42.0
        );
        // a zero quantum is rejected in favor of the default
        assert_eq!(
            from_map(&[("SP_EMU_ETH_QUANTUM", "0")], None).eth_quantum,
            4096
        );
        assert_eq!(
            from_map(&[("SP_EMU_ETH_QUANTUM", "256")], None).eth_quantum,
            256
        );
        // txbreak: only "0" disables
        assert!(!from_map(&[("SP_EMU_ETH_TXBREAK", "0")], None).eth_txbreak);
        assert!(from_map(&[("SP_EMU_ETH_TXBREAK", "1")], None).eth_txbreak);
        // hex with or without 0x; empty selector treated as unset
        assert_eq!(
            from_map(&[("SP_EMU_WATCH", "0x2000")], None).watch,
            Some(0x2000)
        );
        assert_eq!(
            from_map(&[("SP_EMU_ROT_SERVICE", "")], None).rot_service,
            None
        );
        assert_eq!(
            from_map(&[("SP_EMU_BOARD", "sidecar")], None).board,
            Board::Sidecar
        );
        assert_eq!(
            from_map(&[("SP_EMU_BOARD", "typo")], None).board,
            Board::Gimlet
        );
    }

    #[test]
    fn config_file_round_trips() {
        // A run with several set variables serializes to TOML and, parsed back and
        // re-resolved, reproduces the same configuration.
        let orig = from_map(
            &[
                ("SP_EMU_BOARD", "sidecar"),
                ("SP_EMU_ETH_QUANTUM", "256"),
                ("SP_EMU_FLASHDBG", "1"),
                ("SP_EMU_AMBIENT_C", "42"),
                ("SP_EMU_IGNITION", "0:gimlet,1:sidecar"),
            ],
            None,
        );
        let toml = orig.to_toml();
        let loaded = parse_config_toml(&toml).expect("our own output parses");
        let pairs: Vec<(&str, &str)> = loaded
            .iter()
            .map(|(k, v)| (k.as_str(), v.as_str()))
            .collect();
        let round = from_map(&pairs, None);
        assert_eq!(round.board, Board::Sidecar);
        assert_eq!(round.eth_quantum, 256);
        assert!(round.flashdbg);
        assert_eq!(round.ambient_c, 42.0);
        assert_eq!(round.ignition, "0:gimlet,1:sidecar");
        // meta vars are not persisted to the file
        assert!(!toml.contains("SP_EMU_CONFIG "));
        assert!(!toml.contains("SP_EMU_CONFIGDBG"));
    }

    #[test]
    fn config_file_scalars_and_flags() {
        // Native TOML scalars are coerced to the strings the resolver expects, and a
        // boolean `false` means the flag is left off (unset), not "present".
        let m = parse_config_toml(
            "SP_EMU_ETH_QUANTUM = 256\nSP_EMU_AMBIENT_C = 42.5\n\
             SP_EMU_FLASHDBG = true\nSP_EMU_ETHDBG = false\n",
        )
        .unwrap();
        let pairs: Vec<(&str, &str)> = m.iter().map(|(k, v)| (k.as_str(), v.as_str())).collect();
        let c = from_map(&pairs, None);
        assert_eq!(c.eth_quantum, 256);
        assert_eq!(c.ambient_c, 42.5);
        assert!(c.flashdbg); // true -> on
        assert!(!c.ethdbg); // false -> absent -> off
                            // a malformed file is rejected (callers fall back to no file layer)
        assert!(parse_config_toml("this is not = = toml").is_err());
    }

    /// Backward-compatibility contract: every `SP_EMU_*` variable that configured
    /// sp-emu on the `master` branch must remain resolvable, with the same name. This
    /// is the exact set read on master (see git history); removing one silently would
    /// break existing environments. If a removal is ever intentional, delete it here
    /// too and call it out in the commit -- never let this list quietly shrink.
    #[test]
    fn supports_all_master_env_vars() {
        const MASTER_VARS: &[&str] = &[
            "SP_EMU_AMBIENT_C",
            "SP_EMU_BOARD",
            "SP_EMU_BRIDGE",
            "SP_EMU_BRIDGEDBG",
            "SP_EMU_DIFF",
            "SP_EMU_DUMP_ARCHIVE_ID",
            "SP_EMU_DUMP_DIR",
            "SP_EMU_ETHDBG",
            "SP_EMU_ETH_QUANTUM",
            "SP_EMU_ETH_TXBREAK",
            "SP_EMU_EXCDBG",
            "SP_EMU_FLASH",
            "SP_EMU_HOST_UART",
            "SP_EMU_I2C_BRIDGE",
            "SP_EMU_I2C_DEVICE",
            "SP_EMU_IDLE_MS",
            "SP_EMU_IGNITION",
            "SP_EMU_MDIODBG",
            "SP_EMU_NO_DEBUG",
            "SP_EMU_PANICDBG",
            "SP_EMU_PCPROF",
            "SP_EMU_PINGTEST",
            "SP_EMU_PUMPSTATS",
            "SP_EMU_PUMPSTATS_MS",
            "SP_EMU_ROTDUMP",
            "SP_EMU_ROT_FLASH",
            "SP_EMU_ROTPC",
            "SP_EMU_ROT_PREBOOT",
            "SP_EMU_ROT_SERVICE",
            "SP_EMU_ROTSVC",
            "SP_EMU_RTTSTATS",
            "SP_EMU_RXDBG",
            "SP_EMU_RXSTATS",
            "SP_EMU_SENSORS",
            "SP_EMU_SPIDBG",
            "SP_EMU_SPROTDBG",
            "SP_EMU_SVCDBG",
            "SP_EMU_TRACE",
            "SP_EMU_TRACE_FROM",
            "SP_EMU_TRACE_TO",
            "SP_EMU_UARTDBG",
            "SP_EMU_VID0",
            "SP_EMU_VID1",
            "SP_EMU_VPDDBG",
            "SP_EMU_VSCDBG",
            "SP_EMU_WATCH",
        ];
        let missing: Vec<&str> = MASTER_VARS
            .iter()
            .copied()
            .filter(|v| !Config::ENV_VARS.contains(v))
            .collect();
        assert!(
            missing.is_empty(),
            "config table no longer resolves master variables: {missing:?} -- \
             this breaks backward compatibility for existing environments"
        );
    }

    /// Guard: `config.rs` is the sole reader of the environment. No other module
    /// may call `env::var` -- everything routes through the table above.
    #[test]
    fn env_reads_confined_to_config_module() {
        let src = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
        for entry in std::fs::read_dir(&src).expect("read src/") {
            let path = entry.expect("dir entry").path();
            if path.extension().and_then(|e| e.to_str()) != Some("rs") {
                continue;
            }
            if path.file_name().and_then(|n| n.to_str()) == Some("config.rs") {
                continue;
            }
            let text = std::fs::read_to_string(&path).expect("read source file");
            assert!(
                !text.contains("env::var"),
                "{} reads the environment directly; route it through config.rs",
                path.display()
            );
        }
    }
}
