//! LPC55S69 HASHCRYPT SHA-256 engine (spemu-kx3).
//!
//! Real bootleby folds a measurement of the selected image into the DICE CDI with
//! `sha256::update_cdi`, which drives this hardware SHA-256 unit directly (UM11126
//! ch. 48): write CTRL to select SHA2-256 and start a new hash, feed the message as
//! 32-bit words through INDATA (polling STATUS.WAITING at each 512-bit block), then
//! poll STATUS.DIGEST and read the result from DIGEST0..7. The firmware does the
//! Merkle-Damgard padding in software, so the engine only ever sees whole 512-bit
//! blocks and runs the raw block compression.
//!
//! Without this, HASHCRYPT (base 0x400A_4000) falls into the SYSCON RegFile catch-all
//! and reads back 0, so bootleby spins forever waiting for STATUS.DIGEST.
//!
//! Only the polled-INDATA path bootleby uses is modeled (not the AHB-master mode or
//! the AES/crypto features). The compression is Oxide's `sha2::compress256`, so the
//! CDI is byte-for-byte what real hardware would produce.

use crate::mem::Mmio;
use sha2::digest::generic_array::GenericArray;

// Register offsets from the HASHCRYPT base (lpc55-pac hashcrypt::RegisterBlock).
const REG_CTRL: u32 = 0x00; // MODE[2:0] (2 = SHA2-256), NEW_HASH
const REG_STATUS: u32 = 0x04; // bit0 WAITING, bit1 DIGEST (ready)
const REG_INDATA: u32 = 0x20; // one input word
const REG_DIGEST0: u32 = 0x40; // DIGEST0..7 at 0x40..0x60

const ST_WAITING: u32 = 1 << 0;
const ST_DIGEST: u32 = 1 << 1;
const MODE_SHA2_256: u32 = 2; // CTRL.MODE

/// SHA-256 initial hash values (FIPS 180-4).
const SHA256_IV: [u32; 8] = [
    0x6a09_e667, 0xbb67_ae85, 0x3c6e_f372, 0xa54f_f53a, 0x510e_527f, 0x9b05_688c, 0x1f83_d9ab,
    0x5be0_cd19,
];

pub struct HashCrypt {
    /// Running SHA-256 state, compressed one 512-bit block at a time.
    state: [u32; 8],
    /// The partial block being filled by INDATA writes.
    block: [u32; 16],
    fill: usize,
    /// DIGEST0..7 register contents (the state, byte-swapped: firmware reads each
    /// DIGEST word and calls `.swap_bytes()`).
    digest: [u32; 8],
    status: u32,
}

impl Default for HashCrypt {
    fn default() -> Self {
        Self::new()
    }
}

impl HashCrypt {
    pub fn new() -> Self {
        HashCrypt {
            state: SHA256_IV,
            block: [0; 16],
            fill: 0,
            digest: [0; 8],
            status: 0,
        }
    }

    /// CTRL write selecting SHA2-256 + new hash: reset the state, ready for input.
    fn start(&mut self) {
        self.state = SHA256_IV;
        self.fill = 0;
        self.status = ST_WAITING;
    }

    /// One INDATA word. On completing a 512-bit block, compress it into the state
    /// and expose the digest. The engine consumes each INDATA word little-endian
    /// (see the block layout below), so a message streamed as LE words hashes as
    /// its original byte order.
    fn feed(&mut self, word: u32) {
        self.status &= !ST_DIGEST; // cleared whenever data is written
        self.block[self.fill] = word;
        self.fill += 1;
        if self.fill == 16 {
            // The engine consumes each INDATA word little-endian (the firmware's PAD
            // word 0x0000_0080 places 0x80 as the first message byte), so an image
            // read as LE words and streamed here hashes as its original byte order.
            let mut bytes = [0u8; 64];
            for (i, &w) in self.block.iter().enumerate() {
                bytes[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
            }
            sha2::compress256(&mut self.state, core::slice::from_ref(GenericArray::from_slice(&bytes)));
            self.fill = 0;
            for (d, &h) in self.digest.iter_mut().zip(self.state.iter()) {
                *d = h.swap_bytes();
            }
            self.status |= ST_DIGEST; // a block finished; a digest is ready
        }
        self.status |= ST_WAITING; // always ready for the next word
    }
}

impl Mmio for HashCrypt {
    fn name(&self) -> &str {
        "hashcrypt"
    }

    fn read(&mut self, off: u32) -> u32 {
        match off {
            REG_STATUS => self.status,
            o if (REG_DIGEST0..REG_DIGEST0 + 32).contains(&o) => {
                self.digest[((o - REG_DIGEST0) / 4) as usize]
            }
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, val: u32) {
        match off {
            // Starting a SHA2-256 hash (bootleby writes MODE + NEW_HASH together).
            REG_CTRL if val & 0x7 == MODE_SHA2_256 => self.start(),
            REG_INDATA => self.feed(val),
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sha2::{Digest, Sha256};

    /// Drive the engine the way bootleby's sha256 driver does (whole padded blocks,
    /// big-endian length) and confirm the DIGEST matches a reference `Sha256`.
    #[test]
    fn matches_reference_sha256() {
        // Message: 8 words (256 bits) -- like the HMAC key/CDI path.
        let msg: [u32; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut h = HashCrypt::new();
        h.write(REG_CTRL, MODE_SHA2_256); // begin

        // Feed the data words, then software MD padding (as the driver does).
        for &w in &msg {
            h.write(REG_INDATA, w);
        }
        h.write(REG_INDATA, 0x8000_0000u32.swap_bytes()); // PAD word
        // zero-fill to word 14 of the block
        while h.fill != 14 {
            h.write(REG_INDATA, 0);
        }
        let bits: u64 = (msg.len() as u64) * 32;
        h.write(REG_INDATA, u32::swap_bytes((bits >> 32) as u32));
        h.write(REG_INDATA, u32::swap_bytes(bits as u32));

        assert_eq!(h.read(REG_STATUS) & ST_DIGEST, ST_DIGEST, "digest ready");
        let got: Vec<u8> = (0..8)
            .flat_map(|i| h.read(REG_DIGEST0 + i * 4).swap_bytes().to_be_bytes())
            .collect();

        // Reference: SHA-256 over the same message bytes. The engine lays each
        // INDATA word out little-endian, so the message bytes are the words'
        // little-endian encodings (and the pre-swapped length words become the
        // standard big-endian length in the block).
        let mut r = Sha256::new();
        for &w in &msg {
            r.update(w.to_le_bytes());
        }
        assert_eq!(got.as_slice(), r.finalize().as_slice());
    }
}
