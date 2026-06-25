//! Cached debug-flag accessors.
//!
//! The `SP_EMU_*` debug switches are read from the environment ONCE and memoized.
//! They gate `eprintln!` traces sprinkled through the hot paths (per-instruction
//! exceptions, per-frame Ethernet, per-MMIO sensor reads, per-SPI-byte), where a
//! raw `std::env::var(..).is_ok()` — a getenv plus a `String` allocation on every
//! call — is pure overhead once and forever after the first read. Each accessor
//! caches into a `OnceLock`, so steady-state cost is a single atomic load.

macro_rules! flag {
    ($name:ident, $var:literal) => {
        #[inline]
        pub fn $name() -> bool {
            static V: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
            *V.get_or_init(|| std::env::var($var).is_ok())
        }
    };
}

flag!(eth, "SP_EMU_ETHDBG");
flag!(rx, "SP_EMU_RXDBG");
flag!(mdio, "SP_EMU_MDIODBG");
flag!(vpd, "SP_EMU_VPDDBG");
flag!(spi, "SP_EMU_SPIDBG");
flag!(panic, "SP_EMU_PANICDBG");
flag!(svc, "SP_EMU_SVCDBG");
flag!(exc, "SP_EMU_EXCDBG");
flag!(sprot, "SP_EMU_SPROTDBG");
