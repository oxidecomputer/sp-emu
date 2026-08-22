// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Fold any known config version forward to the current external schema.
//!
//! Today there are two source versions: the legacy flat `SP_EMU_*` file (v0) and
//! the typed v1 schema. `migrate` classifies the input by its version and
//! produces a [`ConfigFileV1`], which [`crate::ingest`] then validates. A file
//! newer than this build is refused with [`ConfigError::NewerSchema`] rather than
//! guessed at.
//!
//! When a v2 lands, its parse and a `v1 -> v2` step slot in here; the version
//! dispatch is the one place that grows.

use crate::error::ConfigError;
use crate::legacy::flat_to_v1;
use crate::schema::v1::ConfigFileV1;
use crate::version::{peek_version, SchemaVersion, CURRENT};

/// Read `text` in whatever known version it declares and return it as the
/// current external schema.
pub fn migrate(text: &str) -> Result<ConfigFileV1, ConfigError> {
    match peek_version(text)? {
        // v0: the flat file, whether the version is absent or an explicit 0.
        SchemaVersion::LegacyFlat | SchemaVersion::Known(0) => flat_to_v1(text),
        // v1: the current typed schema; parse it directly.
        SchemaVersion::Known(1) => Ok(toml::from_str(text)?),
        // A known version between v1 and CURRENT with no step here yet. Unreachable
        // while CURRENT == 1; a real arm lands with the version that introduces it.
        SchemaVersion::Known(n) => Err(ConfigError::invalid(
            "schema_version",
            format!("no migration path for version {n}"),
        )),
        SchemaVersion::Newer(found) => Err(ConfigError::NewerSchema {
            found,
            known: CURRENT,
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Board;
    use crate::ingest::ingest;

    #[test]
    fn a_legacy_flat_file_migrates_and_ingests() {
        let flat = "SP_EMU_BOARD = \"sidecar\"\nSP_EMU_ROT_FLASH = \"rot.bin\"\n";
        let c = ingest(migrate(flat).unwrap()).unwrap();
        assert_eq!(c.board(), Board::Sidecar);
        assert_eq!(c.rot_flash(), Some("rot.bin"));
    }

    #[test]
    fn an_explicit_v0_is_still_flat() {
        let flat = "schema_version = 0\nSP_EMU_ETHDBG = \"1\"\n";
        let c = ingest(migrate(flat).unwrap()).unwrap();
        assert!(c.ethdbg());
    }

    #[test]
    fn a_v1_typed_file_parses_directly() {
        let typed = "schema_version = 1\n[op]\nmode = \"run\"\n[rot]\nflash = \"r.bin\"\n";
        let c = ingest(migrate(typed).unwrap()).unwrap();
        assert_eq!(c.mode(), Some("run"));
        assert_eq!(c.rot_flash(), Some("r.bin"));
    }

    #[test]
    fn a_v1_typo_is_rejected_by_the_typed_parse() {
        // A typed file gets deny_unknown_fields; an unknown key is an error, not a
        // silently ignored line as it would be in the flat form.
        let typed = "schema_version = 1\n[op]\nmoad = \"run\"\n";
        assert!(migrate(typed).is_err());
    }

    #[test]
    fn a_newer_version_is_refused_with_a_clear_error() {
        let text = format!("schema_version = {}\n", CURRENT + 1);
        match migrate(&text) {
            Err(ConfigError::NewerSchema { found, known }) => {
                assert_eq!(found, CURRENT + 1);
                assert_eq!(known, CURRENT);
            }
            other => panic!("expected NewerSchema, got {other:?}"),
        }
    }

    #[test]
    fn a_flat_and_equivalent_typed_file_yield_the_same_config() {
        let flat = "SP_EMU_ROT_FLASH = \"r.bin\"\nSP_EMU_SLOT = \"b\"\nSP_EMU_ETHDBG = \"1\"\n";
        let typed = "schema_version = 1\n[op]\nslot = \"b\"\n[rot]\nflash = \"r.bin\"\n[debug]\neth = true\n";
        let from_flat = ingest(migrate(flat).unwrap()).unwrap();
        let from_typed = ingest(migrate(typed).unwrap()).unwrap();
        assert_eq!(from_flat.rot_flash(), from_typed.rot_flash());
        assert_eq!(from_flat.boot_slot(), from_typed.boot_slot());
        assert_eq!(from_flat.ethdbg(), from_typed.ethdbg());
    }
}
