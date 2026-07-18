//! RoT-side SWD: the emulated LPC55 RoT drives the SP's debug port (`SwDp`) over
//! an internal SWD link that it clocks through its FLEXCOMM5 SPI block.
//!
//! The real `drv-lpc55-swd` implements raw SWD on top of SPI5 (0x4009_A000):
//! MOSI/MISO tied together as SWDIO, MSB-first, variable frame length set per word
//! via FIFOWR.LEN. This device intercepts that FLEXCOMM5 SPI block. For now it is a
//! frame logger used to confirm the RoT actually clocks SWD; the raw-SWD decoder
//! (frame sequence -> ADIv5 transactions -> SwDp) is the next milestone.

use crate::mem::Mmio;

// FLEXCOMM5 SPI register offsets (same layout as the modeled FLEXCOMM8 sprot slave).
const STAT: u32 = 0x408; // MSTIDLE = bit8
const FIFOSTAT: u32 = 0xE04; // TXNOTFULL = bit5, RXNOTEMPTY = bit6
const FIFOWR: u32 = 0xE20; // TXDATA[15:0], LEN[27:24] = bits-1, EOT[20], RXIGNORE[22]
const FIFORD: u32 = 0xE30; // RXDATA[15:0]

const STAT_MSTIDLE: u32 = 1 << 8;
const FIFOSTAT_TXNOTFULL: u32 = 1 << 5;
const FIFOSTAT_RXNOTEMPTY: u32 = 1 << 6;

/// FLEXCOMM5 SPI block the RoT clocks SWD through. M2: log frames.
pub struct RotSwdSpi {
    n_fwr: u32,
}

impl Default for RotSwdSpi {
    fn default() -> Self {
        Self::new()
    }
}

impl RotSwdSpi {
    pub fn new() -> Self {
        RotSwdSpi { n_fwr: 0 }
    }
}

impl Mmio for RotSwdSpi {
    fn name(&self) -> &str {
        "FLEXCOMM5-swd"
    }

    fn read(&mut self, off: u32) -> u32 {
        match off & !3 {
            // Report the master perpetually idle and the FIFO always ready, so the
            // RoT's SPI driver never spins waiting on us.
            STAT => STAT_MSTIDLE,
            FIFOSTAT => FIFOSTAT_TXNOTFULL | FIFOSTAT_RXNOTEMPTY,
            FIFORD => 0, // M3 supplies real SWD read data here
            _ => 0,
        }
    }

    fn write(&mut self, off: u32, val: u32) {
        if off & !3 == FIFOWR {
            let len = ((val >> 24) & 0xF) + 1; // FIFOWR.LEN is bits-1
            let data = (val & 0xFFFF) as u16;
            self.n_fwr += 1;
            if std::env::var("SP_EMU_SWD_TRACE").is_ok() && self.n_fwr <= 120 {
                eprintln!("[swd] FIFOWR#{} len={} data={:#06x}", self.n_fwr, len, data);
            }
        }
    }
}
