//! `sp-emu-config`: the versioned, validated configuration format for sp-emu
//! instances.
//!
//! The crate owns the config file format so any program (the emulator, sp-test,
//! fleet tooling) reads and writes an sp-emu config through the same checked
//! methods, rather than hand-parsing TOML.
//!
//! Two ideas shape the design:
//!
//! - Parse, don't validate. The raw, untrusted data deserialized from a file is
//!   a transient external form ([`ConfigFileV1`]); [`ingest`] checks every value
//!   into the validated [`Config`], so a `Config` is valid by construction and
//!   read only through its getters.
//! - Version first. A file declares a `schema_version`; a tool that meets a
//!   newer one fails with a clear message instead of misreading it. See
//!   [`peek_version`].
//!
//! The common entry points are [`load`]/[`load_str`] (read + migrate + validate
//! into a `Config`), [`validate`] (check without keeping the result), and
//! [`template`] (a documented default file). A legacy flat `SP_EMU_*` file is a
//! recognized older version and migrates forward automatically.
//!
//! The crate is env-free and holds no global state, which is the right shape for
//! an outside consumer. Reading the environment and owning the process-wide
//! resolved config stay in the emulator.

mod config;
mod dump;
mod error;
mod ingest;
mod legacy;
mod load;
mod migrate;
pub mod schema;
mod validate;
mod version;

pub use config::{Board, Config};
pub use dump::{dump, template, to_toml};
pub use error::ConfigError;
pub use ingest::ingest;
pub use legacy::{flat_pairs_to_v1, flat_to_v1, ENV_NAMES};
pub use load::{load, load_str};
pub use migrate::migrate;
pub use schema::{v1::ConfigFileV1, SchemaCurrent};
pub use validate::validate;
pub use version::{peek_version, SchemaVersion, CURRENT};
