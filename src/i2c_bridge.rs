//! I2C bridge - tee (SNIFF) or DELEGATE the emulated SP's I2C transactions to a
//! local process. Mirrors sp-emu's socket-bridge idiom (`bridge.rs`,
//! `rot_service.rs`). Two modes, picked by env:
//!
//!  - **SNIFF** (`SP_EMU_I2C_BRIDGE=<addr>`) - one-way: every transaction is teed
//!    as an annotated human-readable line; the SP's built-in device model still
//!    answers. `sp-emu i2c-sniff <addr>` is the pretty-printing listener.
//!  - **DEVICE / DELEGATE** (`SP_EMU_I2C_DEVICE=<addr>`) - request/response: a
//!    local process answers reads as the device(s). sp-emu streams START/write
//!    observations and, per read, sends a query the server replies to with a byte
//!    (or `-` to defer to the built-in model, so the SP still boots). Injects
//!    arbitrary device behavior - a chosen sensor value, a custom/corrupt EEPROM
//!    image - to exercise how Hubris copes. `sp-emu i2c-device <addr> [spec...]`
//!    is a scriptable example server.
//!
//! DEVICE wire protocol (newline-delimited text - a device model may be written
//! in any language; one connection carries all four buses):
//! ```text
//!   sp-emu -> server:  S <bus> <addr> <R|W>         transaction start
//!                      W <bus> <addr> <byte>        write byte (1st = reg pointer)
//!                      R <bus> <addr> <reg> <idx>   READ query (response wanted)
//!   server -> sp-emu:  0xNN     the read value   |   -   defer to the built-in model
//! ```

use std::cell::RefCell;
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::rc::Rc;
use std::time::Duration;

enum Mode {
    Off,
    Sniff,
    Device,
}

struct Inner {
    mode: Mode,
    w: Option<TcpStream>,            // write side: observations (+ queries in device mode)
    r: Option<BufReader<TcpStream>>, // read side: device-mode query responses
}

/// Shared, cloned into each I2C controller so one connection carries every bus.
#[derive(Clone)]
pub struct I2cBridge(Rc<RefCell<Inner>>);

impl I2cBridge {
    /// DEVICE (`SP_EMU_I2C_DEVICE`) takes priority over SNIFF
    /// (`SP_EMU_I2C_BRIDGE`); neither set -> a disabled no-op bridge.
    pub fn from_env() -> Self {
        if let Some(addr) = env_addr("SP_EMU_I2C_DEVICE") {
            match TcpStream::connect(&addr) {
                Ok(s) => {
                    let _ = s.set_nodelay(true);
                    // Bounds on_read()'s blocking wait so a hung device server
                    // can't stall the emulator. Set before try_clone so the read
                    // half inherits it; warn if it didn't take.
                    if let Err(e) = s.set_read_timeout(Some(Duration::from_secs(2))) {
                        eprintln!("[i2c-device] WARNING: could not set read timeout ({e})");
                    }
                    let r = s.try_clone().ok().map(BufReader::new);
                    eprintln!("[i2c-device] delegating I2C reads to {addr}");
                    return Self::wrap(Mode::Device, Some(s), r);
                }
                Err(e) => eprintln!("[i2c-device] connect {addr} failed: {e}; using the built-in model"),
            }
        }
        if let Some(addr) = env_addr("SP_EMU_I2C_BRIDGE") {
            match TcpStream::connect(&addr) {
                Ok(s) => {
                    let _ = s.set_nodelay(true);
                    eprintln!("[i2c-sniff] connected to {addr}");
                    return Self::wrap(Mode::Sniff, Some(s), None);
                }
                Err(e) => eprintln!("[i2c-sniff] connect {addr} failed: {e}; I2C sniff disabled"),
            }
        }
        Self::wrap(Mode::Off, None, None)
    }

    fn wrap(mode: Mode, w: Option<TcpStream>, r: Option<BufReader<TcpStream>>) -> Self {
        I2cBridge(Rc::new(RefCell::new(Inner { mode, w, r })))
    }

    /// True when a transaction-start / write observation should be emitted (skips
    /// formatting on the hot MMIO path when no bridge is attached).
    fn active(&self) -> bool {
        let g = self.0.borrow();
        !matches!(g.mode, Mode::Off) && g.w.is_some()
    }

    /// One-way line write (observations + sniff trace). Drops the stream on a
    /// broken pipe so writes stop.
    fn send(&self, line: &str) {
        let mut g = self.0.borrow_mut();
        if let Some(s) = g.w.as_mut() {
            if writeln!(s, "{line}").is_err() {
                g.w = None;
            }
        }
    }

    /// A master transfer started (CR2 START): addr + direction.
    pub fn on_start(&self, bus: u8, addr: u8, is_read: bool, nbytes: u32) {
        if !self.active() {
            return;
        }
        let g = self.0.borrow();
        let line = match g.mode {
            Mode::Sniff => format!(
                "i2c{bus} START addr={addr:#04x} {} nbytes={nbytes}",
                if is_read { "RD" } else { "WR" }
            ),
            Mode::Device => format!("S {bus} {addr:#04x} {}", if is_read { "R" } else { "W" }),
            Mode::Off => return,
        };
        drop(g);
        self.send(&line);
    }

    /// A write data byte (TXDR); the first of a write phase is the register pointer.
    pub fn on_write(&self, bus: u8, addr: u8, byte: u8) {
        if !self.active() {
            return;
        }
        let g = self.0.borrow();
        let line = match g.mode {
            Mode::Sniff => format!("i2c{bus} WR addr={addr:#04x} <- {byte:#04x}"),
            Mode::Device => format!("W {bus} {addr:#04x} {byte:#04x}"),
            Mode::Off => return,
        };
        drop(g);
        self.send(&line);
    }

    /// DEVICE only: ask the server for a read byte. `None` = defer to the built-in
    /// model (server replied `-`, or any error/timeout - fail safe to internal).
    pub fn on_read(&self, bus: u8, addr: u8, reg: u8, idx: u16) -> Option<u8> {
        let mut g = self.0.borrow_mut();
        if !matches!(g.mode, Mode::Device) {
            return None;
        }
        let Inner { w, r, .. } = &mut *g;
        let (w, r) = (w.as_mut()?, r.as_mut()?);
        writeln!(w, "R {bus} {addr:#04x} {reg:#04x} {idx}").and_then(|_| w.flush()).ok()?;
        let mut line = String::new();
        r.read_line(&mut line).ok()?;
        let t = line.trim();
        if t.is_empty() || t == "-" {
            return None;
        }
        parse_byte(t)
    }

    /// SNIFF only: tee the value the built-in model served for a read (so the
    /// sniff trace shows reads). No-op in device/off mode.
    pub fn on_read_served(&self, bus: u8, addr: u8, reg: u8, idx: u16, byte: u8) {
        if !matches!(self.0.borrow().mode, Mode::Sniff) {
            return;
        }
        self.send(&format!("i2c{bus} RD addr={addr:#04x} reg={reg:#04x} idx={idx} -> {byte:#04x}"));
    }
}

fn env_addr(k: &str) -> Option<String> {
    std::env::var(k).ok().filter(|s| !s.is_empty())
}

/// Parse a delegate device's reply byte: `0xNN` hex, or (tolerated) decimal.
/// Kept distinct from [`parse_hex`] because of that decimal fallback.
fn parse_byte(s: &str) -> Option<u8> {
    let s = s.trim();
    match s.strip_prefix("0x") {
        Some(h) => u8::from_str_radix(h, 16).ok(),
        None => s.parse().ok(),
    }
}

/// Parse a hex number with an optional `0x` prefix (whitespace trimmed). Shared by
/// the `i2c-device` spec + reply-line parsers.
fn parse_hex(s: &str) -> Option<u32> {
    u32::from_str_radix(s.trim().trim_start_matches("0x"), 16).ok()
}

/// Annotate a 7-bit I2C address with the device behind it (mirrors `soc::I2c`).
fn device_name(addr: u8) -> &'static str {
    match addr {
        0x48..=0x4a => "TMP117 temp",
        0x18..=0x1f => "TSE2004av DIMM temp",
        0x4c => "TMP451 NIC temp",
        0x50..=0x53 => "AT24CSW080 VPD/FRUID",
        0x70..=0x77 => "PCA954x I2C mux",
        0x20..=0x27 => "PCA9538 GPIO expander",
        _ => "?",
    }
}

/// `sp-emu i2c-sniff <listen-addr>` - listen + pretty-print the SNIFF trace,
/// annotating known device addresses. Loops across emulator relaunches.
pub fn serve(addr: &str) -> anyhow::Result<()> {
    let l = TcpListener::bind(addr).map_err(|e| anyhow::anyhow!("bind {addr}: {e}"))?;
    eprintln!("[i2c-sniff] listening on {addr} - start the emulator with SP_EMU_I2C_BRIDGE={addr}");
    for conn in l.incoming() {
        let conn = match conn {
            Ok(c) => c,
            Err(e) => {
                eprintln!("[i2c-sniff] accept: {e}");
                continue;
            }
        };
        eprintln!("[i2c-sniff] emulator connected");
        for line in BufReader::new(conn).lines() {
            let line = match line {
                Ok(l) => l,
                Err(_) => break,
            };
            let note = line
                .split_whitespace()
                .find_map(|t| t.strip_prefix("addr="))
                .and_then(|h| u8::from_str_radix(h.trim_start_matches("0x"), 16).ok())
                .map(device_name)
                .filter(|n| *n != "?");
            match note {
                Some(n) => println!("{line}    # {n}"),
                None => println!("{line}"),
            }
        }
        eprintln!("[i2c-sniff] emulator disconnected; waiting for next");
    }
    Ok(())
}

/// A scriptable device-model override: by (addr, reg) -> 16-bit value (served
/// big-endian, hi byte at idx 0 - matching `soc::I2c::device_reg`), or by addr ->
/// a byte image (an EEPROM, served at `reg + idx`).
enum Override {
    Reg { addr: u8, reg: u8, val: u16 },
    Image { addr: u8, bytes: Vec<u8> },
}

/// Parse one `i2c-device` spec arg:
///   `<addr>/<reg>=<val>`   e.g. `0x48/0x00=0x2800`  (TMP117 temp register)
///   `<addr>@<file>`        e.g. `0x50@my-vpd.bin`   (serve a file as the EEPROM)
fn parse_override(s: &str) -> anyhow::Result<Override> {
    if let Some((addr, file)) = s.split_once('@') {
        let addr = parse_hex(addr).ok_or_else(|| anyhow::anyhow!("bad addr in {s:?}"))? as u8;
        let bytes = std::fs::read(file.trim()).map_err(|e| anyhow::anyhow!("read {file}: {e}"))?;
        return Ok(Override::Image { addr, bytes });
    }
    let (lhs, val) = s.split_once('=').ok_or_else(|| anyhow::anyhow!("spec {s:?} needs `addr/reg=val` or `addr@file`"))?;
    let (addr, reg) = lhs.split_once('/').ok_or_else(|| anyhow::anyhow!("spec {s:?} needs `addr/reg=val`"))?;
    Ok(Override::Reg {
        addr: parse_hex(addr).ok_or_else(|| anyhow::anyhow!("bad addr in {s:?}"))? as u8,
        reg: parse_hex(reg).ok_or_else(|| anyhow::anyhow!("bad reg in {s:?}"))? as u8,
        val: parse_hex(val).ok_or_else(|| anyhow::anyhow!("bad val in {s:?}"))? as u16,
    })
}

/// `sp-emu i2c-device <listen-addr> [spec...]` - an example DELEGATE device
/// server. Answers `R` queries from the given overrides (defers everything else
/// to the emulator's built-in model with `-`), logging each injected read so the
/// SP's consumption of the values is visible. Specs:
///   `<addr>/<reg>=<val>`   inject a 16-bit register value (hi byte first)
///   `<addr>@<file>`        serve a file as that device's read stream (EEPROM)
pub fn serve_device(addr: &str, specs: &[String]) -> anyhow::Result<()> {
    let overrides: Vec<Override> =
        specs.iter().map(|s| parse_override(s)).collect::<anyhow::Result<_>>()?;
    let l = TcpListener::bind(addr).map_err(|e| anyhow::anyhow!("bind {addr}: {e}"))?;
    eprintln!("[i2c-device] listening on {addr} - start the emulator with SP_EMU_I2C_DEVICE={addr}");
    for o in &overrides {
        match o {
            Override::Reg { addr, reg, val } => {
                eprintln!("[i2c-device]   inject {addr:#04x} reg {reg:#04x} = {val:#06x}  ({})", device_name(*addr))
            }
            Override::Image { addr, bytes } => {
                eprintln!("[i2c-device]   serve {} bytes as {addr:#04x}  ({})", bytes.len(), device_name(*addr))
            }
        }
    }
    for conn in l.incoming() {
        let conn = match conn {
            Ok(c) => c,
            Err(e) => {
                eprintln!("[i2c-device] accept: {e}");
                continue;
            }
        };
        eprintln!("[i2c-device] emulator connected");
        let mut w = match conn.try_clone() {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[i2c-device] clone: {e}");
                continue;
            }
        };
        for line in BufReader::new(conn).lines() {
            let line = match line {
                Ok(l) => l,
                Err(_) => break,
            };
            // Only `R <bus> <addr> <reg> <idx>` queries need a reply.
            let f: Vec<&str> = line.split_whitespace().collect();
            if f.first() != Some(&"R") || f.len() < 5 {
                continue;
            }
            let (addr, reg, idx) = match (parse_hex(f[2]), parse_hex(f[3]), f[4].parse::<usize>().ok()) {
                (Some(a), Some(r), Some(i)) => (a as u8, r as u8, i),
                _ => continue,
            };
            let ans: Option<u8> = overrides.iter().find_map(|o| match o {
                Override::Reg { addr: a, reg: r, val } if *a == addr && *r == reg => {
                    Some(if idx == 0 { (*val >> 8) as u8 } else { *val as u8 })
                }
                Override::Image { addr: a, bytes } if *a == addr => {
                    Some(*bytes.get((reg as usize).wrapping_add(idx) % bytes.len().max(1)).unwrap_or(&0))
                }
                _ => None,
            });
            let reply = match ans {
                Some(b) => {
                    eprintln!("[i2c-device] SP read {addr:#04x} reg {reg:#04x} idx {idx} -> injecting {b:#04x}");
                    format!("{b:#04x}")
                }
                None => "-".to_string(),
            };
            if writeln!(w, "{reply}").and_then(|_| w.flush()).is_err() {
                break;
            }
        }
        eprintln!("[i2c-device] emulator disconnected; waiting for next");
    }
    Ok(())
}
