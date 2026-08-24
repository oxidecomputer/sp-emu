// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! The crate's error type.

use std::fmt;

/// An error from loading, migrating, or validating an sp-emu config.
///
/// The enum is `#[non_exhaustive]`: downstream matches must carry a wildcard arm
/// so a future variant does not break them.
#[derive(Debug)]
#[non_exhaustive]
pub enum ConfigError {
    /// A config file could not be read from disk.
    Io { path: String, source: std::io::Error },
    /// The input is not well-formed TOML.
    Parse(toml::de::Error),
    /// A config could not be serialized back to TOML.
    Serialize(toml::ser::Error),
    /// A value was well-formed TOML but not acceptable for its field (an
    /// out-of-range number, an unknown enum spelling, a malformed compound
    /// string). `path` is the dotted config path, e.g. `op.board`.
    Validation { path: String, problem: String },
    /// The file declares a schema version newer than this build understands. It
    /// cannot be read; the caller reports it and exits rather than guessing.
    NewerSchema { found: u32, known: u32 },
}

impl ConfigError {
    /// Build a [`ConfigError::Validation`] for `path` with a human-readable
    /// `problem`.
    pub(crate) fn invalid(
        path: impl Into<String>,
        problem: impl Into<String>,
    ) -> Self {
        ConfigError::Validation { path: path.into(), problem: problem.into() }
    }
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConfigError::Io { path, source } => {
                write!(f, "reading {path}: {source}")
            }
            ConfigError::Parse(e) => write!(f, "config is not valid TOML: {e}"),
            ConfigError::Serialize(e) => {
                write!(f, "config could not be serialized: {e}")
            }
            ConfigError::Validation { path, problem } => {
                write!(f, "config `{path}`: {problem}")
            }
            ConfigError::NewerSchema { found, known } => write!(
                f,
                "config schema version {found} is newer than this build understands \
                 (knows up to {known}); upgrade sp-emu"
            ),
        }
    }
}

impl std::error::Error for ConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ConfigError::Io { source, .. } => Some(source),
            ConfigError::Parse(e) => Some(e),
            ConfigError::Serialize(e) => Some(e),
            ConfigError::Validation { .. }
            | ConfigError::NewerSchema { .. } => None,
        }
    }
}

impl From<toml::de::Error> for ConfigError {
    fn from(e: toml::de::Error) -> Self {
        ConfigError::Parse(e)
    }
}

impl From<toml::ser::Error> for ConfigError {
    fn from(e: toml::ser::Error) -> Self {
        ConfigError::Serialize(e)
    }
}
