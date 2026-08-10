//! Read a config into the validated [`Config`].
//!
//! [`load`] reads a file; [`load_str`] takes the text directly. Both fold any
//! known version forward ([`crate::migrate`]) and then validate
//! ([`crate::ingest`]), so a caller gets a ready-to-use `Config` or a single
//! error explaining what was wrong.

use std::path::Path;

use crate::config::Config;
use crate::error::ConfigError;
use crate::ingest::ingest;
use crate::migrate::migrate;

/// Read and validate a config file at `path`.
pub fn load(path: impl AsRef<Path>) -> Result<Config, ConfigError> {
    let path = path.as_ref();
    let text = std::fs::read_to_string(path).map_err(|source| ConfigError::Io {
        path: path.display().to_string(),
        source,
    })?;
    load_str(&text)
}

/// Migrate and validate config `text` (any known version) into a [`Config`].
pub fn load_str(text: &str) -> Result<Config, ConfigError> {
    ingest(migrate(text)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Board;
    use crate::error::ConfigError;

    #[test]
    fn load_str_reads_a_typed_file() {
        let c = load_str("schema_version = 1\n[op]\nboard = \"sidecar\"\n").unwrap();
        assert_eq!(c.board(), Board::Sidecar);
    }

    #[test]
    fn load_str_reads_a_legacy_flat_file() {
        let c = load_str("SP_EMU_ROT_FLASH = \"r.bin\"\n").unwrap();
        assert_eq!(c.rot_flash(), Some("r.bin"));
    }

    #[test]
    fn a_missing_file_is_an_io_error_naming_the_path() {
        let err = load("/no/such/sp-emu-config.toml").unwrap_err();
        match err {
            ConfigError::Io { path, .. } => assert!(path.contains("sp-emu-config.toml")),
            other => panic!("expected Io, got {other:?}"),
        }
    }
}
