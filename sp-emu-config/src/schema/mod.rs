// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! The external, versioned config schemas: the raw TOML forms.
//!
//! Each shipped schema version is a frozen module, never edited once released; a
//! new version adds a sibling module plus a migration into it. `SchemaCurrent`
//! names the version this build reads and writes.

pub mod v1;

/// The current external schema version.
pub use v1::ConfigFileV1 as SchemaCurrent;
