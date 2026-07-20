//! Model of the LPC55 PUF (Physically Unclonable Function) key store, enough for
//! the RoT image's `dice-self` startup (`lib/lpc55-rot-startup/src/dice.rs`) to
//! derive its DICE identity and write the cert handoff -- so `faux-ipcc get-certs`
//! returns a real chain instead of `AttestNoCerts`.
//!
//! The `lpc55-puf` driver runs a small command engine: GENERATEKEY streams a
//! keycode out of CODEOUTPUT; GETKEY feeds that keycode back in through CODEINPUT
//! and streams the derived key out of KEYOUTPUT, polling STAT for the busy/avail
//! handshake. We model that handshake and return a *stable* seed from GETKEY, so
//! the DICE identity is deterministic across boots -- like a real device's fixed
//! UDS. This is not a real PUF: there is no unclonable secret, and the seed is a
//! fixed constant, so the identity is emulator-wide, not per-instance.
//!
//! Startup order matters (`lpc55-rot-startup::startup`): `dice::run` executes
//! GENERATEKEY -> GETKEY -> `block_index(1)` -> `lock_indices_low`, then
//! `puf_check` requires index 1 to be blocked+locked. So IDXBLK_L starts with
//! index 1 unblocked/unlocked and the driver's own read-modify-write blocks and
//! locks it -- unlike the old stub, which pre-blocked it and made GETKEY fail.

use crate::mem::Mmio;
use std::collections::{HashMap, VecDeque};

fn dbg() -> bool {
    std::env::var("SP_EMU_PUFDBG").is_ok()
}

// Register offsets from base 0x4003_B000 (lpc55-pac PUF RegisterBlock).
const CTRL: u32 = 0x00;
const KEYINDEX: u32 = 0x04;
const KEYSIZE: u32 = 0x08;
const STAT: u32 = 0x20;
const ALLOW: u32 = 0x28;
const KEYINPUT: u32 = 0x40;
const CODEINPUT: u32 = 0x44;
const CODEOUTPUT: u32 = 0x48;
const KEYOUTPUT: u32 = 0x64;
const IFSTAT: u32 = 0xDC;
const IDXBLK_L: u32 = 0x20C;

// CTRL command bits.
const CTRL_GENERATEKEY: u32 = 1 << 3;
const CTRL_GETKEY: u32 = 1 << 6;
// STAT flag bits.
const STAT_BUSY: u32 = 1 << 0;
const STAT_SUCCESS: u32 = 1 << 1;
const STAT_ERROR: u32 = 1 << 2;
const STAT_KEYOUTAVAIL: u32 = 1 << 5;
const STAT_CODEINREQ: u32 = 1 << 6;
const STAT_CODEOUTAVAIL: u32 = 1 << 7;
// ALLOW: enroll(0), start(1), setkey(2), getkey(3) -- report all permitted.
const ALLOW_ALL: u32 = 0x0F;

/// Stable device seed returned by GETKEY -- the root of the emulated RoT's DICE
/// identity. Any fixed non-zero value works.
const PUF_SEED: [u8; 32] = [
    0x53, 0x50, 0x2d, 0x45, 0x4d, 0x55, 0x2d, 0x50, 0x55, 0x46, 0x2d, 0x64, 0x69, 0x63, 0x65, 0x2d,
    0x73, 0x65, 0x65, 0x64, 0x2d, 0x76, 0x31, 0x2e, 0x30, 0x2e, 0x30, 0x2d, 0x21, 0x21, 0x21, 0x21,
];

pub struct Puf {
    keyindex: u32,
    keysize: u32,
    idxblk_l: u32,
    busy: bool,
    success: bool,
    error: bool,
    codeout: VecDeque<u32>,  // keycode words to emit (GENERATEKEY)
    codein_left: usize,      // keycode words still to consume (GETKEY)
    keyout: VecDeque<u32>,   // key/seed words to emit (GETKEY)
    regs: HashMap<u32, u32>, // catch-all for plain registers (idxblk_l_dp, cfg, ...)
}

impl Puf {
    pub fn new() -> Self {
        Puf {
            keyindex: 0,
            keysize: 0,
            idxblk_l: 0,
            busy: false,
            success: false,
            error: false,
            codeout: VecDeque::new(),
            codein_left: 0,
            keyout: VecDeque::new(),
            regs: HashMap::new(),
        }
    }

    /// KEYSIZE holds the key length in 64-bit units (`(bytes*8) >> 6`).
    fn key_bytes(&self) -> usize {
        (self.keysize as usize) * 8
    }

    /// Keycode length in u32 words, per `Puf::key_to_keycode_len` (UM11126).
    fn keycode_words(&self) -> usize {
        let kb = self.key_bytes();
        (20 + ((kb + 31) & !31)) / 4
    }

    fn stat(&self) -> u32 {
        let mut s = 0;
        if self.busy {
            s |= STAT_BUSY;
        }
        if self.success {
            s |= STAT_SUCCESS;
        }
        if self.error {
            s |= STAT_ERROR;
        }
        if !self.keyout.is_empty() {
            s |= STAT_KEYOUTAVAIL;
        }
        if self.codein_left > 0 {
            s |= STAT_CODEINREQ;
        }
        if !self.codeout.is_empty() {
            s |= STAT_CODEOUTAVAIL;
        }
        s
    }

    fn start_generate(&mut self) {
        self.success = false;
        self.error = false;
        let n = self.keycode_words();
        if dbg() {
            eprintln!(
                "[puf] GENERATEKEY index={} keysize={} ({} keycode words)",
                self.keyindex, self.keysize, n
            );
        }
        self.codeout.clear();
        // keycode[0] carries the key index in bits[11:8] (index_from_keycode).
        self.codeout.push_back((self.keyindex & 0xf) << 8);
        for i in 1..n {
            self.codeout.push_back(0xC0DE_0000 | i as u32);
        }
        self.busy = true;
    }

    fn start_getkey(&mut self) {
        self.success = false;
        self.error = false;
        if dbg() {
            eprintln!("[puf] GETKEY start");
        }
        self.codein_left = self.keycode_words();
        self.keyout.clear();
        self.busy = true;
    }

    fn pop_codeout(&mut self) -> u32 {
        let v = self.codeout.pop_front().unwrap_or(0);
        if self.codeout.is_empty() {
            self.busy = false;
            self.success = true;
            if dbg() {
                eprintln!("[puf] GENERATEKEY complete");
            }
        }
        v
    }

    fn consume_codein(&mut self) {
        if self.codein_left > 0 {
            self.codein_left -= 1;
            if self.codein_left == 0 {
                if dbg() {
                    eprintln!("[puf] GETKEY -> seed delivered");
                }
                // Keycode fully fed back: stream the seed out of KEYOUTPUT
                // (little-endian words, matching the driver's to_ne_bytes on the
                // little-endian Cortex-M33).
                let words = self.key_bytes() / 4;
                for i in 0..words {
                    let b: [u8; 4] = PUF_SEED[i * 4..i * 4 + 4].try_into().unwrap();
                    self.keyout.push_back(u32::from_le_bytes(b));
                }
            }
        }
    }

    fn pop_keyout(&mut self) -> u32 {
        let v = self.keyout.pop_front().unwrap_or(0);
        if self.keyout.is_empty() && self.codein_left == 0 {
            self.busy = false;
            self.success = true;
        }
        v
    }
}

impl Default for Puf {
    fn default() -> Self {
        Self::new()
    }
}

impl Mmio for Puf {
    fn name(&self) -> &str {
        "lpc55-puf"
    }

    fn read(&mut self, off: u32) -> u32 {
        if dbg() && matches!(off, ALLOW | IDXBLK_L | KEYINDEX | KEYSIZE) {
            eprintln!("[puf] read {off:#05x}");
        }
        match off {
            STAT => self.stat(),
            ALLOW => ALLOW_ALL,
            CODEOUTPUT => self.pop_codeout(),
            KEYOUTPUT => self.pop_keyout(),
            IFSTAT => 0, // no interface error
            IDXBLK_L => self.idxblk_l,
            KEYINDEX => self.keyindex,
            KEYSIZE => self.keysize,
            _ => self.regs.get(&off).copied().unwrap_or(0),
        }
    }

    fn write(&mut self, off: u32, val: u32) {
        if dbg() && matches!(off, CTRL | KEYINDEX | KEYSIZE | IDXBLK_L) {
            eprintln!("[puf] write {off:#05x} = {val:#010x}");
        }
        match off {
            CTRL => {
                if val & CTRL_GENERATEKEY != 0 {
                    self.start_generate();
                } else if val & CTRL_GETKEY != 0 {
                    self.start_getkey();
                }
            }
            KEYINDEX => self.keyindex = val,
            KEYSIZE => self.keysize = val,
            CODEINPUT => self.consume_codein(),
            IDXBLK_L => self.idxblk_l = val,
            KEYINPUT => {} // SETKEY path, unused
            _ => {
                self.regs.insert(off, val);
            }
        }
    }
}
