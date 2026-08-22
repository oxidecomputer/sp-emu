// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Config-file schema versioning.
//!
//! The version is read first, on its own, so a tool that meets a file it does
//! not understand fails with a clear message instead of misreading it. The probe
//! reads only the top-level `schema_version` scalar and tolerates every other
//! key, so even a newer file with unknown sections yields its version.

use crate::error::ConfigError;
use serde::Deserialize;

/// The schema version this build writes and understands. A file may declare an
/// older version (migrated forward) or none at all (a legacy flat file); a newer
/// version cannot be read.
pub const CURRENT: u32 = 1;

/// A config file classified by its `schema_version`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchemaVersion {
    /// A pre-versioning flat `SP_EMU_NAME = "value"` file: no `schema_version`
    /// key. Treated as version 0 and migrated forward.
    LegacyFlat,
    /// A versioned file this build understands (`<= CURRENT`).
    Known(u32),
    /// A versioned file newer than this build (`> CURRENT`). It cannot be read;
    /// the caller reports it and exits rather than guessing at the format.
    Newer(u32),
}

/// A tolerant probe that reads only `schema_version` and ignores every other
/// key, so a newer file with unknown sections still yields its version.
#[derive(Deserialize)]
struct VersionProbe {
    #[serde(default)]
    schema_version: Option<u32>,
}

/// Classify a config file by its `schema_version` without fully parsing it.
///
/// Returns [`ConfigError::Parse`] only when the input is not well-formed TOML;
/// an unknown-but-parseable file is reported as [`SchemaVersion::Newer`], not an
/// error, so the caller owns the policy for what to do about it.
pub fn peek_version(text: &str) -> Result<SchemaVersion, ConfigError> {
    let probe: VersionProbe = toml::from_str(text)?;
    Ok(match probe.schema_version {
        None => SchemaVersion::LegacyFlat,
        Some(n) if n <= CURRENT => SchemaVersion::Known(n),
        Some(n) => SchemaVersion::Newer(n),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_version_is_legacy_flat() {
        let flat = "SP_EMU_FLASH = \"sp-flash.bin\"\nSP_EMU_SLOT = \"a\"\n";
        assert_eq!(peek_version(flat).unwrap(), SchemaVersion::LegacyFlat);
    }

    #[test]
    fn current_version_is_known() {
        let text = format!("schema_version = {CURRENT}\n");
        assert_eq!(peek_version(&text).unwrap(), SchemaVersion::Known(CURRENT));
    }

    #[test]
    fn version_zero_is_known() {
        assert_eq!(
            peek_version("schema_version = 0\n").unwrap(),
            SchemaVersion::Known(0)
        );
    }

    #[test]
    fn newer_version_peeks_cleanly_past_unknown_sections() {
        // The whole point of the tolerant probe: a file one version ahead, with
        // sections this build has never seen, still reports its version.
        let text = format!(
            "schema_version = {}\n[unknown_future_section]\nx = 1\n",
            CURRENT + 1
        );
        assert_eq!(
            peek_version(&text).unwrap(),
            SchemaVersion::Newer(CURRENT + 1)
        );
    }

    #[test]
    fn malformed_toml_is_a_parse_error() {
        assert!(peek_version("this is = = not toml").is_err());
    }
}
