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
//! the AES/crypto features). The compression is Oxide's `sha2::compress256` driven
//! in the INDATA/DIGEST byte order the firmware expects (UM11126 ch. 48; see the
//! `feed` layout note), so the CDI matches what the firmware would compute on
//! silicon; the `matches_reference_sha256_*` tests pin that against a reference.

use crate::mem::Mmio;
use sha2::digest::generic_array::GenericArray;

// Register offsets from the HASHCRYPT base (lpc55-pac hashcrypt::RegisterBlock).
const REG_CTRL: u32 = 0x00; // MODE[2:0] (2 = SHA2-256), NEW_HASH
const REG_STATUS: u32 = 0x04; // bit0 WAITING, bit1 DIGEST (ready)
const REG_INDATA: u32 = 0x20; // one input word
const REG_DIGEST0: u32 = 0x40; // DIGEST0..7 at 0x40..0x60

const ST_WAITING: u32 = 1 << 0;
const ST_DIGEST: u32 = 1 << 1;
const MODE_MASK: u32 = 0x7; // CTRL.MODE field (bits 2:0)
const MODE_SHA2_256: u32 = 2; // CTRL.MODE value for SHA2-256
const NEW_HASH: u32 = 1 << 4; // CTRL.NEW_HASH: reset and start a fresh hash
const DIGEST_BYTES: u32 = 8 * 4; // DIGEST0..7: eight 32-bit words

/// SHA-256 initial hash values (FIPS 180-4).
const SHA256_IV: [u32; 8] = [
    0x6a09_e667,
    0xbb67_ae85,
    0x3c6e_f372,
    0xa54f_f53a,
    0x510e_527f,
    0x9b05_688c,
    0x1f83_d9ab,
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
            sha2::compress256(
                &mut self.state,
                core::slice::from_ref(GenericArray::from_slice(&bytes)),
            );
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
            o if (REG_DIGEST0..REG_DIGEST0 + DIGEST_BYTES).contains(&o) => {
                self.digest[((o - REG_DIGEST0) / 4) as usize]
            }
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, val: u32) {
        match off {
            // Start a fresh SHA2-256 hash: MODE=SHA2-256 AND NEW_HASH asserted
            // together (as bootleby writes). A MODE write without NEW_HASH selects
            // the algorithm without resetting -- we don't model that (the mode is
            // implicitly SHA-256), so such writes are ignored rather than restarting.
            REG_CTRL if val & MODE_MASK == MODE_SHA2_256 && val & NEW_HASH != 0 => self.start(),
            REG_INDATA => self.feed(val),
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sha2::{Digest, Sha256};

    /// Drive the engine the way bootleby's sha256 driver does: start with
    /// MODE+NEW_HASH, feed the message words, then the software Merkle-Damgard
    /// padding (pre-swapped PAD and 64-bit length words). Zero-fill carries into a
    /// second block when the length words don't fit the current one. Returns the
    /// byte-swapped DIGEST the firmware reads back.
    fn engine_digest(msg: &[u32]) -> Vec<u8> {
        let mut h = HashCrypt::new();
        h.write(REG_CTRL, MODE_SHA2_256 | NEW_HASH);
        for &w in msg {
            h.write(REG_INDATA, w);
        }
        h.write(REG_INDATA, 0x8000_0000u32.swap_bytes()); // PAD (0x80 first byte)
        while h.fill != 14 {
            h.write(REG_INDATA, 0);
        }
        let bits = (msg.len() as u64) * 32;
        h.write(REG_INDATA, u32::swap_bytes((bits >> 32) as u32));
        h.write(REG_INDATA, u32::swap_bytes(bits as u32));
        assert_eq!(h.read(REG_STATUS) & ST_DIGEST, ST_DIGEST, "digest ready");
        (0..8)
            .flat_map(|i| h.read(REG_DIGEST0 + i * 4).swap_bytes().to_be_bytes())
            .collect()
    }

    /// Reference SHA-256 over the words' little-endian encodings -- the byte order
    /// the engine consumes INDATA in.
    fn reference_digest(msg: &[u32]) -> Vec<u8> {
        let mut r = Sha256::new();
        for &w in msg {
            r.update(w.to_le_bytes());
        }
        r.finalize().to_vec()
    }

    /// Single padded block (256-bit message, like the CDI path).
    #[test]
    fn matches_reference_sha256_single_block() {
        let msg: [u32; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
        assert_eq!(engine_digest(&msg), reference_digest(&msg));
    }

    /// 640-bit message: data + padding spans two 512-bit blocks, exercising the
    /// `fill == 16` compress/reset and the cross-block state carry.
    #[test]
    fn matches_reference_sha256_multi_block() {
        let msg: [u32; 20] =
            core::array::from_fn(|i| (i as u32).wrapping_mul(0x0101_0101) ^ 0xdead_beef);
        assert_eq!(engine_digest(&msg), reference_digest(&msg));
    }

    /// A re-issued CTRL (MODE + NEW_HASH) mid-stream resets the running hash, so the
    /// digest reflects only the words fed after the restart.
    #[test]
    fn new_hash_restarts_cleanly() {
        let msg: [u32; 8] = [1, 2, 3, 4, 5, 6, 7, 8];
        let mut h = HashCrypt::new();
        h.write(REG_CTRL, MODE_SHA2_256 | NEW_HASH);
        for w in [0xaaaa_aaaau32, 0xbbbb_bbbb] {
            h.write(REG_INDATA, w); // garbage, discarded by the restart below
        }
        h.write(REG_CTRL, MODE_SHA2_256 | NEW_HASH); // NEW_HASH: reset
        for &w in &msg {
            h.write(REG_INDATA, w);
        }
        h.write(REG_INDATA, 0x8000_0000u32.swap_bytes());
        while h.fill != 14 {
            h.write(REG_INDATA, 0);
        }
        let bits = (msg.len() as u64) * 32;
        h.write(REG_INDATA, u32::swap_bytes((bits >> 32) as u32));
        h.write(REG_INDATA, u32::swap_bytes(bits as u32));
        let got: Vec<u8> = (0..8)
            .flat_map(|i| h.read(REG_DIGEST0 + i * 4).swap_bytes().to_be_bytes())
            .collect();
        assert_eq!(got, reference_digest(&msg));
    }

    /// A MODE write without NEW_HASH does not start/reset a hash.
    #[test]
    fn mode_without_new_hash_does_not_start() {
        let mut h = HashCrypt::new();
        h.write(REG_CTRL, MODE_SHA2_256); // no NEW_HASH bit
        assert_eq!(h.read(REG_STATUS) & ST_WAITING, 0, "not started");
    }
}
