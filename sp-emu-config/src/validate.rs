//! Check a config without keeping the result.
//!
//! [`validate`] runs the whole load path (version detection, migration,
//! validation) and reports the detected version on success or the first error.
//! The library never exits or prints; the caller (`sp-emu config validate`) turns
//! the outcome into a message and an exit status.

use crate::error::ConfigError;
use crate::ingest::ingest;
use crate::migrate::migrate;
use crate::version::{peek_version, SchemaVersion};

/// Validate config `text`. Returns the detected [`SchemaVersion`] when the config
/// is well-formed and every value passes ingest; otherwise the error describing
/// the first problem (unparseable, newer-than-known, or an invalid value).
pub fn validate(text: &str) -> Result<SchemaVersion, ConfigError> {
    let version = peek_version(text)?;
    ingest(migrate(text)?)?;
    Ok(version)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_good_typed_file_reports_its_version() {
        let v = validate("schema_version = 1\n[op]\nmode = \"run\"\n").unwrap();
        assert_eq!(v, SchemaVersion::Known(1));
    }

    #[test]
    fn a_good_flat_file_reports_legacy() {
        let v = validate("SP_EMU_ROT_FLASH = \"r.bin\"\n").unwrap();
        assert_eq!(v, SchemaVersion::LegacyFlat);
    }

    #[test]
    fn an_invalid_value_surfaces_as_an_error() {
        match validate("schema_version = 1\n[op]\nboard = \"gymlet\"\n") {
            Err(ConfigError::Validation { path, .. }) => assert_eq!(path, "op.board"),
            other => panic!("expected a validation error, got {other:?}"),
        }
    }

    #[test]
    fn a_newer_version_surfaces_as_an_error() {
        assert!(matches!(
            validate("schema_version = 999\n"),
            Err(ConfigError::NewerSchema { found: 999, .. })
        ));
    }
}
