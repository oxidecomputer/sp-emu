// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Central configuration: ingest every `SP_EMU_*` input (the environment, or a
//! config file) exactly once, resolve it into a validated [`Config`], and store
//! it. After `init`, no other module reads the environment; they call
//! `config::get()`.
//!
//! This module is the emulator-side adapter over the `sp-emu-config` crate, which
//! owns the config format. The crate turns the raw inputs into the validated,
//! method-accessed `Config` (parse, don't validate). This module keeps what is
//! inherently the emulator's: the process-wide resolved config, the `SP_EMU_*`
//! environment glue that feeds the crate, the ambient reads (`state_dir`,
//! `rot_bootleby_path`) that a library must not do, and the flat
//! `SP_EMU_NAME = "value"` dump the bundle format still consumes.
//!
//! Sources and precedence: flag > (config file | environment) > default. The
//! environment and a config file are two alternative sources, never stacked:
//! `--load-config <path>` reads all `SP_EMU_*` settings from a flat
//! `SP_EMU_NAME = "value"` TOML table and ignores the environment, so a saved
//! configuration reproduces exactly; without it, the environment is read as usual.
//! `--dump-config <path>` writes the effective (only-set) configuration back for
//! re-loading. The full resolved table is echoed to stderr only under
//! `$SP_EMU_CONFIGDBG`.
//!
//! Backward compatibility: every `SP_EMU_*` variable keeps its name, default, and
//! leniency. The env/file path stays lenient (a typo or malformed value coerces to
//! a default); the strict validation the crate applies is reserved for
//! the typed config file.

use anyhow::{anyhow, bail, Context, Result};
use std::sync::OnceLock;

pub use sp_emu_config::Config;

/// Meta variables `--dump-config` does not persist: `SP_EMU_CONFIGDBG` is a debug
/// toggle about config printing, not persistent state, so loading a dumped file must
/// not silently re-enable it.
const NOT_PERSISTED: &[&str] = &["SP_EMU_CONFIGDBG"];

/// The resolved config plus the flat presence map: which `SP_EMU_*` were explicitly
/// set, and to what. The map drives `is_set` (and thus `instance_file`) and the flat
/// `--dump-config` / `pack` output; the validated `Config` itself carries no presence.
struct Resolved {
    config: Config,
    inputs: Vec<(&'static str, String)>,
}

/// Resolve a source (the environment or a loaded file, via `get`) into the presence
/// map and the validated `Config`. The `SP_EMU_*` variables feed the crate's flat
/// bridge, which parses them leniently; the `--seed` flag wins over `SP_EMU_SEED`.
fn resolve(
    get: &dyn Fn(&str) -> Option<String>,
    seed_override: Option<String>,
) -> Result<Resolved> {
    let mut inputs: Vec<(&'static str, String)> = Vec::new();
    for &name in sp_emu_config::ENV_NAMES {
        if let Some(v) = get(name) {
            inputs.push((name, v));
        }
    }
    let pairs = inputs.iter().map(|(n, v)| (n.to_string(), v.clone()));
    let mut external = sp_emu_config::flat_pairs_to_v1(pairs);
    if let Some(seed) = seed_override {
        external.op.seed = Some(seed); // --seed beats $SP_EMU_SEED
    }
    let config =
        sp_emu_config::ingest(external).map_err(|e| anyhow!("invalid configuration: {e}"))?;
    Ok(Resolved { config, inputs })
}

/// Was this variable explicitly provided (by the environment or a loaded file)?
fn is_set_in(inputs: &[(&'static str, String)], name: &str) -> bool {
    inputs.iter().any(|(n, _)| *n == name)
}

/// Escape a value for a TOML basic string (`"..."`).
fn escape_toml(s: &str) -> String {
    s.replace('\\', "\\\\")
        .replace('"', "\\\"")
        .replace('\n', "\\n")
}

/// Serialize the explicitly-set variables as a round-trippable flat TOML file, keyed
/// by `SP_EMU_*` name (mirroring the environment). Loading it back (`--load-config`)
/// reproduces the run; omitted variables take their default. The bundle format
/// (`pack`) consumes this flat form; the typed schema is emitted only by the
/// `sp-emu config` subcommands.
fn flat_toml(inputs: &[(&'static str, String)]) -> String {
    let mut out = String::from(
        "# sp-emu configuration (mirrors $SP_EMU_*; the environment overrides it).\n\
         # Load with `--load-config <this file>`; omitted variables use their default.\n\
         # A flag is on by presence -- delete its line to disable it.\n\n",
    );
    for (name, val) in inputs {
        if NOT_PERSISTED.contains(name) {
            continue;
        }
        out.push_str(&format!("{name} = \"{}\"\n", escape_toml(val)));
    }
    out
}

/// Render the resolved config for the opt-in stderr dump: the effective typed config
/// followed by which `SP_EMU_*` were explicitly set, so the full state is visible.
fn render(r: &Resolved) -> String {
    let mut out = sp_emu_config::dump(&r.config)
        .unwrap_or_else(|e| format!("# <effective-config dump failed: {e}>\n"));
    out.push_str("\n# explicitly set via SP_EMU_* (environment or config file):\n");
    for (name, val) in &r.inputs {
        out.push_str(&format!("#   {name} = {val:?}\n"));
    }
    out
}

/// Parse a flat config file into `(SP_EMU_NAME, value)` pairs, coercing scalar
/// values to the string the resolver expects (a boolean `false` means "unset",
/// a flag left off). A malformed file is a parse error the caller reports.
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

static CONFIG: OnceLock<Resolved> = OnceLock::new();

/// Resolve and store the process configuration. Call once, early in `main`, before
/// any subsystem is built.
///
/// sp-emu takes its `SP_EMU_*` settings from the environment, or, with the
/// `--load-config <path>` flag, from a TOML config file instead of the environment,
/// so a saved configuration reproduces exactly regardless of the shell. The two
/// sources are never mixed; command-line flags always win. Precedence is
/// flag > (config file | environment) > default. `--dump-config <path>` writes the
/// effective configuration for later re-loading.
pub fn init(
    seed_override: Option<String>,
    load_config: Option<String>,
    dump_config: Option<String>,
) -> Result<()> {
    // Two alternative sources for SP_EMU_*, never stacked: a config file
    // (--load-config, which then ignores the environment entirely) or the environment.
    let resolved = match &load_config {
        Some(path) => {
            // A path the caller named explicitly should be readable; a typo or a
            // permissions problem is an error, not a silent all-defaults run.
            let text = std::fs::read_to_string(path)
                .with_context(|| format!("reading --load-config {path}"))?;
            // A typed (versioned) file is not loadable this way yet: the flat reader
            // keeps only top-level scalars and silently drops its `[section]`s, which
            // would resolve to all defaults. Refuse it, pointing at the tools that do
            // read it, rather than run a misconfigured instance.
            if let Ok(v) = sp_emu_config::peek_version(&text) {
                let flat = matches!(
                    v,
                    sp_emu_config::SchemaVersion::LegacyFlat
                        | sp_emu_config::SchemaVersion::Known(0)
                );
                if !flat {
                    bail!(
                        "{path}: --load-config reads only the flat SP_EMU_* form, but this \
                         is a versioned config file. Check it with `sp-emu config validate \
                         {path}`, or convert it with `sp-emu config upgrade {path}`."
                    );
                }
            }
            let file = parse_config_toml(&text).unwrap_or_else(|e| {
                eprintln!("[config] ignoring malformed {path}: {e}");
                Vec::new()
            });
            eprintln!(
                "[config] loaded {} ({} vars); ignoring the SP_EMU_* environment",
                path,
                file.len()
            );
            let get = |k: &str| file.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
            resolve(&get, seed_override)?
        }
        None => resolve(&|k| std::env::var(k).ok(), seed_override)?,
    };

    if let Some(path) = &dump_config {
        match std::fs::write(path, flat_toml(&resolved.inputs)) {
            Ok(()) => eprintln!("[config] wrote {} ({} set)", path, resolved.inputs.len()),
            Err(e) => eprintln!("[config] writing {path} failed: {e}"),
        }
    }
    if resolved.config.configdbg() {
        eprint!("{}", render(&resolved));
    }
    // Loud, not silent: if this fails, `get()` already resolved a config (without
    // the seed override) before `init` ran, so the process would run mis-seeded.
    CONFIG
        .set(resolved)
        .map_err(|_| anyhow!("config::init called after config was already resolved by get()"))?;
    Ok(())
}

/// The resolved config plus presence map. Lazily defaults from a clean environment
/// if `init` was never called (unit tests / non-CLI paths), so accessors never panic.
fn resolved() -> &'static Resolved {
    CONFIG
        .get_or_init(|| resolve(&|k| std::env::var(k).ok(), None).expect("default config resolves"))
}

/// The process configuration.
pub fn get() -> &'static Config {
    &resolved().config
}

/// The effective configuration as a flat `SP_EMU_NAME = "value"` TOML table (only the
/// explicitly-set knobs). Used by `pack` to embed a reproducible config in a bundle.
pub fn to_toml() -> String {
    flat_toml(&resolved().inputs)
}

/// The instance state directory. `$SP_EMU_STATE_DIR` if set, otherwise a per-user
/// default under `$XDG_STATE_HOME` or `~/.local/state` (a temp dir if neither is
/// available). Default instance files (flash, RoT flash, identity) and the stowed
/// archives live under it, so a bare run does not litter the working directory.
pub fn state_dir() -> String {
    if let Some(d) = get().state_dir() {
        return d.to_string();
    }
    let base = std::env::var("XDG_STATE_HOME")
        .ok()
        .filter(|s| !s.is_empty())
        .or_else(|| {
            std::env::var("HOME")
                .ok()
                .filter(|s| !s.is_empty())
                .map(|h| format!("{h}/.local/state"))
        });
    match base {
        Some(b) => format!("{b}/sp-emu"),
        None => format!("{}/sp-emu", std::env::temp_dir().display()),
    }
}

/// The bootleby image to boot the RoT through, or None to jump straight to the
/// Hubris image.
///
/// Real bootleby is the default: it is what performs A/B slot selection and
/// honors the CFPA's persistent boot preference, so without it those behaviors
/// are not modeled at all. sp-emu does not ship the image (it is a signed binary
/// that lives in the hubris tree), so find it next to the RoT archive or in a
/// hubris checkout. `$SP_EMU_ROT_BOOTLEBY` names one explicitly;
/// `$SP_EMU_ROT_NO_BOOTLEBY` opts out.
pub fn rot_bootleby_path() -> &'static Option<String> {
    static RESOLVED: std::sync::OnceLock<Option<String>> = std::sync::OnceLock::new();
    RESOLVED.get_or_init(resolve_rot_bootleby)
}

/// The search behind `rot_bootleby_path`, run once. Callers must agree on the
/// answer even if the filesystem changes mid-run, and the "not found" warning is
/// worth printing once rather than once per caller.
fn resolve_rot_bootleby() -> Option<String> {
    let cfg = get();
    if cfg.rot_no_bootleby() {
        return None;
    }
    if let Some(p) = cfg.rot_bootleby() {
        return Some(p.to_string());
    }
    const NAME: &str = "bootleby-oxide-rot-1.zip";
    let mut candidates: Vec<String> = Vec::new();
    // Alongside the RoT archive: a curated image directory holds the matching set.
    if let Some(dir) = cfg
        .rot_flash()
        .and_then(|p| std::path::Path::new(p).parent())
        .and_then(|d| d.to_str())
    {
        candidates.push(format!("{dir}/{NAME}"));
    }
    // A hubris checkout, where bootleby is checked in.
    if let Ok(h) = std::env::var("HUBRIS") {
        candidates.push(format!("{h}/app/oxide-rot-1/{NAME}"));
    }
    let found = candidates
        .into_iter()
        .find(|p| std::path::Path::new(p).exists());
    if found.is_none() {
        eprintln!(
            "[rot] no {NAME} found next to the RoT archive or under $HUBRIS; \
             booting the image directly (no bootleby A/B selection). Set \
             SP_EMU_ROT_BOOTLEBY, or SP_EMU_ROT_NO_BOOTLEBY=1 to silence this."
        );
    }
    found
}

/// Whether `state_dir()` is the built-in default (no explicit `$SP_EMU_STATE_DIR`).
pub fn state_dir_is_default() -> bool {
    get().state_dir().is_none()
}

/// Whether an `SP_EMU_*` variable was explicitly set (environment or config file).
pub fn is_set(name: &str) -> bool {
    is_set_in(&resolved().inputs, name)
}

/// Resolve an instance file path: the explicit knob value when it was set (or already
/// absolute), otherwise `value` under the instance state directory (`state_dir`).
/// Debug: SP-core pc-window instruction trace ranges, from
/// `SP_EMU_PCWIN=lo-hi[,lo-hi...]` (hex, 0x prefix optional). An emulator
/// bring-up tracing aid, not instance configuration, so it bypasses the
/// config schema and is not recorded in bundles.
pub fn pcwin() -> Option<Vec<(u32, u32)>> {
    let s = std::env::var("SP_EMU_PCWIN").ok()?;
    let v: Vec<(u32, u32)> = s
        .split(',')
        .filter_map(|w| {
            let (lo, hi) = w.split_once('-')?;
            Some((
                u32::from_str_radix(lo.trim_start_matches("0x"), 16).ok()?,
                u32::from_str_radix(hi.trim_start_matches("0x"), 16).ok()?,
            ))
        })
        .collect();
    (!v.is_empty()).then_some(v)
}

pub fn instance_file(env_name: &str, value: &str) -> String {
    if is_set(env_name) || std::path::Path::new(value).is_absolute() {
        value.to_string()
    } else {
        format!("{}/{}", state_dir(), value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sp_emu_config::Board;

    /// Resolve from an in-memory map instead of the process environment, so tests
    /// are hermetic regardless of the developer's / CI's ambient `SP_EMU_*`.
    fn resolve_map(vars: &[(&str, &str)], seed_override: Option<String>) -> Resolved {
        resolve(
            &|k| {
                vars.iter()
                    .find(|(n, _)| *n == k)
                    .map(|(_, val)| val.to_string())
            },
            seed_override,
        )
        .expect("test config resolves")
    }

    /// The validated config for a set of `SP_EMU_*` values.
    fn from_map(vars: &[(&str, &str)], seed_override: Option<String>) -> Config {
        resolve_map(vars, seed_override).config
    }

    #[test]
    fn defaults_resolve() {
        let c = from_map(&[], None);
        assert_eq!(c.flash_path(), "sp-flash.bin");
        assert_eq!(c.rot_nvm_path(), "sp-rot-flash.bin");
        assert_eq!(c.identity_path(), "sp-emu-identity");
        assert_eq!(c.board(), Board::Gimlet);
        assert!(!c.flashdbg());
        assert_eq!(c.idle_ms(), 10);
        assert_eq!(c.eth_quantum(), 4096);
        assert!(c.eth_txbreak());
        assert_eq!(c.ambient_c(), 30.0);
        // No SP_EMU_STATE_DIR by default, so the built-in per-user default is used.
        assert_eq!(c.state_dir(), None);
    }

    /// `instance_file` passes absolute paths through and joins a relative default under
    /// the instance state directory, so a bare run does not write into the cwd.
    #[test]
    fn instance_file_absolute_passes_through_relative_joins_state_dir() {
        // Absolute value is used verbatim regardless of whether the knob was set.
        assert_eq!(
            instance_file("SP_EMU_UNSET_KNOB_XYZ", "/abs/sp-flash.bin"),
            "/abs/sp-flash.bin"
        );
        // A relative default (knob not set) lands under the namespaced state dir.
        let p = instance_file("SP_EMU_UNSET_KNOB_XYZ", "sp-flash.bin");
        assert!(p.ends_with("/sp-flash.bin"), "got {p}");
        assert!(p.contains("sp-emu"), "state dir should be namespaced: {p}");
    }

    #[test]
    fn seed_override_wins() {
        // --seed beats $SP_EMU_SEED.
        let c = from_map(&[("SP_EMU_SEED", "from-env")], Some("from-cli".into()));
        assert_eq!(c.seed(), Some("from-cli"));
        // absent flag falls back to the environment.
        let c = from_map(&[("SP_EMU_SEED", "from-env")], None);
        assert_eq!(c.seed(), Some("from-env"));
    }

    /// The operation knobs let a config file describe a whole instance: unset by
    /// default (so the command line drives), and parsed when present.
    #[test]
    fn operation_knobs_resolve() {
        let c = from_map(&[], None);
        assert_eq!(c.mode(), None);
        assert_eq!(c.boot_slot(), None);
        assert_eq!(c.run_max(), None);

        let c = from_map(
            &[
                ("SP_EMU_MODE", "run"),
                ("SP_EMU_SLOT", "b"),
                ("SP_EMU_RUN_MAX", "0"),
            ],
            None,
        );
        assert_eq!(c.mode(), Some("run"));
        assert_eq!(c.boot_slot(), Some("b"));
        assert_eq!(c.run_max(), Some(0));

        // Empty strings are treated as unset; a non-numeric budget is ignored.
        let c = from_map(&[("SP_EMU_MODE", ""), ("SP_EMU_RUN_MAX", "forever")], None);
        assert_eq!(c.mode(), None);
        assert_eq!(c.run_max(), None);
    }

    #[test]
    fn value_parsers_preserve_leniency() {
        // whitespace-padded ambient parses (historical .trim())
        assert_eq!(
            from_map(&[("SP_EMU_AMBIENT_C", " 42 ")], None).ambient_c(),
            42.0
        );
        // a zero quantum is rejected in favor of the default
        assert_eq!(
            from_map(&[("SP_EMU_ETH_QUANTUM", "0")], None).eth_quantum(),
            4096
        );
        assert_eq!(
            from_map(&[("SP_EMU_ETH_QUANTUM", "256")], None).eth_quantum(),
            256
        );
        // txbreak: only "0" disables
        assert!(!from_map(&[("SP_EMU_ETH_TXBREAK", "0")], None).eth_txbreak());
        assert!(from_map(&[("SP_EMU_ETH_TXBREAK", "1")], None).eth_txbreak());
        // hex with or without 0x; empty selector treated as unset
        assert_eq!(
            from_map(&[("SP_EMU_WATCH", "0x2000")], None).watch(),
            Some(0x2000)
        );
        assert_eq!(
            from_map(&[("SP_EMU_ROT_SERVICE", "")], None).rot_service(),
            None
        );
        assert_eq!(
            from_map(&[("SP_EMU_BOARD", "sidecar")], None).board(),
            Board::Sidecar
        );
        // a board typo is lenient: it falls back to gimlet.
        assert_eq!(
            from_map(&[("SP_EMU_BOARD", "typo")], None).board(),
            Board::Gimlet
        );
    }

    #[test]
    fn config_file_round_trips() {
        // A run with several set variables serializes to TOML and, parsed back and
        // re-resolved, reproduces the same configuration.
        let orig = resolve_map(
            &[
                ("SP_EMU_BOARD", "sidecar"),
                ("SP_EMU_ETH_QUANTUM", "256"),
                ("SP_EMU_FLASHDBG", "1"),
                ("SP_EMU_AMBIENT_C", "42"),
                ("SP_EMU_IGNITION", "0:gimlet,1:sidecar"),
            ],
            None,
        );
        let toml = flat_toml(&orig.inputs);
        let loaded = parse_config_toml(&toml).expect("our own output parses");
        let pairs: Vec<(&str, &str)> = loaded
            .iter()
            .map(|(k, v)| (k.as_str(), v.as_str()))
            .collect();
        let round = from_map(&pairs, None);
        assert_eq!(round.board(), Board::Sidecar);
        assert_eq!(round.eth_quantum(), 256);
        assert!(round.flashdbg());
        assert_eq!(round.ambient_c(), 42.0);
        assert_eq!(round.ignition(), "0:gimlet,1:sidecar");
        // meta vars are not persisted to the file
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
        assert_eq!(c.eth_quantum(), 256);
        assert_eq!(c.ambient_c(), 42.5);
        assert!(c.flashdbg()); // true -> on
        assert!(!c.ethdbg()); // false -> absent -> off
                              // a malformed file is rejected (callers fall back to no file layer)
        assert!(parse_config_toml("this is not = = toml").is_err());
    }

    /// Backward-compatibility contract: every `SP_EMU_*` variable in this list must
    /// remain resolvable, with the same name; removing one silently would break
    /// existing environments. If a removal is ever intentional, delete it here too
    /// and call it out in the commit; never let this list quietly shrink.
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
            .filter(|v| !sp_emu_config::ENV_NAMES.contains(v))
            .collect();
        assert!(
            missing.is_empty(),
            "config no longer resolves master variables: {missing:?} -- \
             this breaks backward compatibility for existing environments"
        );
    }

    /// Guard: `config.rs` is the sole reader of the environment. No other module
    /// may call `env::var`; everything routes through this adapter.
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
