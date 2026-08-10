# sp-emu-config

The versioned, validated configuration format for [sp-emu] instances. The crate
owns the config file schema so any program, not just the emulator, reads and
writes an sp-emu config through checked methods instead of hand-parsing TOML.

The design is parse-don't-validate: the TOML file is a transient external form,
and ingesting it produces a `Config` that is valid by construction and read only
through getters. A file declares a `schema_version`; an older version (the legacy
flat `SP_EMU_*` file is version 0) migrates forward, and a newer one is refused
with a clear error rather than misread. The crate is env-free and holds no global
state.

## Use

```rust
use sp_emu_config::{load_str, validate, ConfigError, SchemaVersion};

// Load any known version (flat or typed) into a validated Config.
let cfg = load_str(text)?;
let flash = cfg.flash_path();       // &str
let sidecar = cfg.board().is_sidecar();

// Check a file without keeping the result. A newer-than-known or invalid file is
// an error the caller decides what to do with; the library never exits.
match validate(text) {
    Ok(version) => println!("valid, read as {version:?}"),
    Err(ConfigError::NewerSchema { found, known }) => {
        eprintln!("schema v{found} is newer than this build (knows v{known})");
    }
    Err(e) => eprintln!("invalid: {e}"),
}
```

Other entry points: `load` (from a file path), `migrate` (fold any known version
to the current external schema), `ingest` (validate an external form), `to_toml`
and `dump` (serialize), and `template` (a documented default file). `flat_to_v1`
and `ENV_NAMES` bridge the legacy flat `SP_EMU_*` form.

[sp-emu]: ../
