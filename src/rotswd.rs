// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! RoT-side SWD: the emulated LPC55 RoT drives the SP's debug port (`SwDp`) over
//! an internal SWD link that it clocks through its FLEXCOMM5 SPI block.
//!
//! The real `drv-lpc55-swd` implements raw SWD on top of FLEXCOMM5 (granted at
//! 0x4009_6000 for this build, per `humility map`): MOSI/MISO tied together as
//! SWDIO, MSB-first, variable frame length set per word via FIFOWR.LEN. This
//! device decodes that raw SWD bit stream into ADIv5 register transactions,
//! hands them to the SP's `SwDp` (via a thread-local link the serve loop drains,
//! since a device can't reach the SP's cpu/bus itself), and clocks the ACK+data
//! back so the RoT can actually read/write the SP.

use crate::mem::Mmio;
use std::collections::VecDeque;
use std::sync::{Arc, Mutex, MutexGuard, OnceLock};

// FLEXCOMM5 SPI register offsets.
const STAT: u32 = 0x408; // MSTIDLE = bit8
const FIFOSTAT: u32 = 0xE04; // TXNOTFULL = bit5, RXNOTEMPTY = bit6
const FIFOWR: u32 = 0xE20; // TXDATA[15:0], LEN[27:24] = bits-1
const FIFORD: u32 = 0xE30; // RXDATA[15:0]

const STAT_MSTIDLE: u32 = 1 << 8;
const FIFOSTAT_TXNOTFULL: u32 = 1 << 5;
const FIFOSTAT_RXNOTEMPTY: u32 = 1 << 6;

// SWD ACK nibble the RoT reads (4-bit frame): trn(bit3), ACK0(bit2), ACK1(bit1),
// ACK2(bit0). OK = ACK0 set -> 0b0100.
const ACK_OK: u16 = 0x4;

/// One ADIv5 transaction the RoT wants run against the SP.
pub struct SwdReq {
    pub ap: bool,
    pub rnw: bool,
    pub a: u8, // register address bits [3:2]
    pub wdata: u32,
}

/// Shared link between the RoT-side FLEXCOMM5 device (RoT thread) and the SP
/// serve loop (SP thread), which owns the SP's cpu/bus and so can drive its
/// `SwDp`. SWD takes effect at SP instruction boundaries on hardware too, so
/// servicing transactions from the SP loop preserves halt semantics.
pub struct SwdLink {
    /// RoT -> serve loop: decoded transactions to run against the SP, in order
    /// (a burst may clock several writes before the serve loop drains them).
    pub req: VecDeque<SwdReq>,
    /// serve loop -> RoT: the read result for the outstanding read request (the
    /// RoT stalls on FIFOSTAT until this is set, so only one is ever in flight).
    pub resp: Option<u32>,
}

/// Mutex wrapper for the shared SWD link.
pub struct SwdCell(Mutex<SwdLink>);
impl SwdCell {
    // Poison policy: a panic on either core's thread is unrecoverable for the
    // pair, so propagating the poison panic (unwrap) is deliberate fail-fast.
    pub fn lock(&self) -> MutexGuard<'_, SwdLink> {
        self.0.lock().unwrap()
    }
}

static SWD: OnceLock<Arc<SwdCell>> = OnceLock::new();

/// Install the shared SWD link (once, before either bus is built).
pub fn enable() {
    let _ = SWD.set(Arc::new(SwdCell(Mutex::new(SwdLink {
        req: VecDeque::new(),
        resp: None,
    }))));
}

/// A clone of the shared SWD link handle, if enabled.
pub fn link() -> Option<Arc<SwdCell>> {
    SWD.get().cloned()
}

/// Decoder phase across the frames of one SWD transaction.
enum Phase {
    /// Between transactions: watching for a request byte (setup/idle ignored).
    Idle,
    /// A read request is in flight; RX (ACK then data) comes via FIFORD.
    ReadData,
    /// A write request is in flight; accumulate the 34 data bits (skip the ACK).
    WriteData { acc: u64, bits: u8, ap: bool, a: u8 },
}

/// FLEXCOMM5 SPI block: decodes the RoT's raw SWD and drives the SP's SwDp.
pub struct RotSwdSpi {
    link: Arc<SwdCell>,
    rx: VecDeque<u16>, // frames the RoT will pop via FIFORD
    phase: Phase,
    trace: bool,
    n_fwr: u32,
}

impl RotSwdSpi {
    pub fn new(link: Arc<SwdCell>) -> Self {
        RotSwdSpi {
            link,
            rx: VecDeque::new(),
            phase: Phase::Idle,
            trace: crate::config::get().swd_trace(),
            n_fwr: 0,
        }
    }

    /// If a read is in flight and the serve loop has supplied the result, turn it
    /// into the four read-data frames (8,8,8,9) the RoT expects: each data byte
    /// bit-reversed (SWD is LSB-first, the RoT un-reverses), last frame carries
    /// the even-parity bit as its LSB.
    fn maybe_supply_read_data(&mut self) {
        if !matches!(self.phase, Phase::ReadData) || !self.rx.is_empty() {
            return;
        }
        if let Some(data) = self.link.lock().resp.take() {
            self.rx.push_back((data as u8).reverse_bits() as u16);
            self.rx.push_back(((data >> 8) as u8).reverse_bits() as u16);
            self.rx.push_back(((data >> 16) as u8).reverse_bits() as u16);
            let b3 = ((data >> 24) as u8).reverse_bits() as u16;
            let parity = (data.count_ones() & 1) as u16;
            self.rx.push_back((b3 << 1) | parity);
            self.phase = Phase::Idle;
            if self.trace {
                eprintln!("[swd] read data supplied: {:#010x}", data);
            }
        }
    }

    fn fifowr(&mut self, val: u32) {
        let len = (((val >> 24) & 0xF) + 1) as u8; // FIFOWR.LEN is bits-1
        let data = (val & 0xFFFF) as u16;
        self.n_fwr += 1;
        if self.trace && self.n_fwr <= 60 {
            eprintln!("[swd] FIFOWR#{} raw={:#010x} len={} data={:#06x}", self.n_fwr, val, len, data);
        }
        match &mut self.phase {
            Phase::Idle => {
                // A request byte is 8 bits with Start(b7)=1, Stop(b1)=0, Park(b0)=1.
                // Line reset (0xFF), JTAG-to-SWD (0x79E7, 16-bit) and idle (0x00)
                // don't match and are ignored (no wire to model).
                if len == 8 && data & 0x83 == 0x81 {
                    let ap = (data >> 6) & 1 != 0;
                    let rnw = (data >> 5) & 1 != 0;
                    let a2 = (data >> 4) & 1;
                    let a3 = (data >> 3) & 1;
                    let a = ((a2 << 2) | (a3 << 3)) as u8;
                    // ACK is OK optimistically (the SP's SwDp never faults); the
                    // serve loop runs the actual transfer.
                    self.rx.push_back(ACK_OK);
                    if rnw {
                        self.link.lock().req.push_back(SwdReq { ap, rnw: true, a, wdata: 0 });
                        // The SP thread drains SWD requests; it may be parked idle.
                        if let Some(l) = crate::sprot::link() {
                            l.wake_sp();
                        }
                        self.phase = Phase::ReadData;
                    } else {
                        self.phase = Phase::WriteData { acc: 0, bits: 0, ap, a };
                    }
                    if self.trace {
                        eprintln!(
                            "[swd] request ap={} rnw={} a={:#x}",
                            ap as u8, rnw as u8, a
                        );
                    }
                }
            }
            Phase::ReadData => {} // dummy TX; data returns via FIFORD
            Phase::WriteData { acc, bits, ap, a } => {
                // Skip the 4-bit ACK-read dummy; accumulate the 34 data bits
                // (1 turnaround + 32 data + 1 parity), MSB-first.
                if len == 4 {
                    return;
                }
                *acc = (*acc << len) | (data as u64 & ((1u64 << len) - 1));
                *bits += len;
                if *bits >= 34 {
                    // bits[32:1] are the 32 data bits the RoT sent bit-reversed.
                    let rev = ((*acc >> 1) & 0xFFFF_FFFF) as u32;
                    let wdata = rev.reverse_bits();
                    let (ap, a) = (*ap, *a);
                    self.link.lock().req.push_back(SwdReq {
                        ap,
                        rnw: false,
                        a,
                        wdata,
                    });
                    if let Some(l) = crate::sprot::link() {
                        l.wake_sp();
                    }
                    if self.trace {
                        eprintln!("[swd] write ap={} a={:#x} data={:#010x}", ap as u8, a, wdata);
                    }
                    self.phase = Phase::Idle;
                }
            }
        }
    }
}

impl Mmio for RotSwdSpi {
    fn name(&self) -> &str {
        "FLEXCOMM5-swd"
    }

    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            STAT => STAT_MSTIDLE,
            FIFOSTAT => {
                self.maybe_supply_read_data();
                FIFOSTAT_TXNOTFULL
                    | if self.rx.is_empty() {
                        0
                    } else {
                        FIFOSTAT_RXNOTEMPTY
                    }
            }
            FIFORD => {
                self.maybe_supply_read_data();
                self.rx.pop_front().unwrap_or(0) as u32
            }
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, val: u32) {
        if off & !3 == FIFOWR {
            self.fifowr(val);
        }
    }
}
