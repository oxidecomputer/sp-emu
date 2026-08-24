// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Cached debug-flag accessors.
//!
//! These gate `eprintln!` traces on the hot paths (per-instruction exceptions,
//! per-frame Ethernet, per-MMIO sensor reads, per-SPI-byte). The switches are
//! resolved once by `config` (the sole reader of the environment) and exposed
//! here as thin inline accessors; `config::get()` is a single atomic `OnceLock`
//! load, so steady-state cost is unchanged.

macro_rules! flag {
    ($name:ident, $field:ident) => {
        #[inline]
        pub fn $name() -> bool {
            crate::config::get().$field()
        }
    };
}

flag!(eth, ethdbg);
flag!(rx, rxdbg);
flag!(mdio, mdiodbg);
flag!(vpd, vpddbg);
flag!(spi, spidbg);
flag!(panic, panicdbg);
flag!(svc, svcdbg);
flag!(exc, excdbg);
flag!(sprot, sprotdbg);
