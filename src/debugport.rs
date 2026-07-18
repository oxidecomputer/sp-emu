//! Emulated ARM debug port: an ADIv5 SW-DP + MEM-AP with a working debug core.
//!
//! This is the register-level model probe-rs drives over SWD. The Glasgow-applet
//! server (`glasgow.rs`) decodes `CMD_TRANSFER` into calls to [`SwDp::transfer`];
//! a later phase can drive the same model from the emulated RoT. Unlike the
//! read-only GDB stub and the OpenOCD-Tcl RPC in `gdb.rs`, this offers a *real*
//! debug core — halt/run/step plus register access via DCRSR/DCRDR — which is
//! what hiffy (halt, write program, run, read results) and endoscope need.
//!
//! sp-emu speaks the Glasgow *applet* protocol, not raw SWD line bits, so there
//! is no parity/turnaround/clocking here — just ADIv5 register semantics.

use crate::cpu::Cpu;
use crate::mem::Bus;

/// SW-DP IDCODE for a Cortex-M7 (as an ST-Link reports on an STM32H7).
const DPIDR: u32 = 0x6BA0_2477;
/// AHB-AP IDR (MEM-AP class), STM32H7 value. CLASS field (bits[16:13]) = 0b1000.
const AP_IDR: u32 = 0x8477_0001;

// CTRL/STAT power-up request/ack bits (the debug-port init handshake).
const CDBGPWRUPREQ: u32 = 1 << 28;
const CDBGPWRUPACK: u32 = 1 << 29;
const CSYSPWRUPREQ: u32 = 1 << 30;
const CSYSPWRUPACK: u32 = 1 << 31;

// CoreDebug block (ARMv7-M, architecturally fixed addresses).
const DFSR: u32 = 0xE000_ED30; // Debug Fault Status Register (write-1-to-clear)
const DHCSR: u32 = 0xE000_EDF0;
const DCRSR: u32 = 0xE000_EDF4;
const DCRDR: u32 = 0xE000_EDF8;
const DEMCR: u32 = 0xE000_EDFC;

// DFSR bit0 = HALTED: the halt was a debug request (C_HALT/C_STEP), not a
// breakpoint/watchpoint. Reporting this is what makes probe-rs classify the
// halt as HaltReason::Request; a stale non-zero DFSR reads as a breakpoint and
// sends probe-rs into an FPB retry loop.
const DFSR_HALTED: u32 = 1 << 0;

// DHCSR fields.
const DBGKEY: u32 = 0xA05F;
const C_DEBUGEN: u32 = 1 << 0;
const C_HALT: u32 = 1 << 1;
const C_STEP: u32 = 1 << 2;
const C_MASKINTS: u32 = 1 << 3;
const S_REGRDY: u32 = 1 << 16;
const S_HALT: u32 = 1 << 17;

/// One SWD transfer's result, mapped by the server to the applet's response byte.
pub enum Ack {
    /// Read data (`Some`) or a write acknowledgement (`None`).
    Ok(Option<u32>),
    Wait,
    Fault,
}

/// A SW-DP with a single MEM-AP, plus the CoreDebug semantics behind it.
pub struct SwDp {
    select: u32,     // DP SELECT: APSEL[31:24] / APBANKSEL[7:4] / DPBANKSEL[3:0]
    ctrl_stat: u32,  // last CTRL/STAT write (its power-req bits drive the ack echo)
    posted: u32,     // pipelined AP-read result (returned by the next AP read / RDBUFF)
    csw: u32,        // MEM-AP CSW (size + address-increment)
    tar: u32,        // MEM-AP TAR (transfer address)
    dcrdr: u32,      // DCRDR shadow for DCRSR register-file access
    demcr: u32,      // DEMCR (VC_CORERESET honored in phase 2)
    dhcsr_ctrl: u32, // C_DEBUGEN|C_HALT|C_MASKINTS echoed back on DHCSR read
    /// Set by a DHCSR resume-with-C_STEP; the server steps one instruction then
    /// re-halts. Free-run (no step) is driven by `cpu.halted` directly.
    pub step_request: bool,
}

impl Default for SwDp {
    fn default() -> Self {
        Self::new()
    }
}

impl SwDp {
    pub fn new() -> Self {
        SwDp {
            select: 0,
            ctrl_stat: 0,
            posted: 0,
            csw: 0,
            tar: 0,
            dcrdr: 0,
            demcr: 0,
            dhcsr_ctrl: 0,
            step_request: false,
        }
    }

    /// Process one SWD register transfer. `a` is the A[3:2] field (0/4/8/C).
    pub fn transfer(
        &mut self,
        cpu: &mut Cpu,
        bus: &mut Bus,
        ap: bool,
        rnw: bool,
        a: u8,
        wdata: u32,
    ) -> Ack {
        if ap {
            self.ap_transfer(cpu, bus, rnw, a, wdata)
        } else {
            self.dp_transfer(rnw, a, wdata)
        }
    }

    fn dp_transfer(&mut self, rnw: bool, a: u8, wdata: u32) -> Ack {
        match (rnw, a & 0x0C) {
            (true, 0x0) => Ack::Ok(Some(DPIDR)),
            (false, 0x0) => Ack::Ok(None), // ABORT: clear sticky errors (none modeled)
            (true, 0x4) => {
                // CTRL/STAT read: echo the power-up acks for the req bits so
                // probe-rs's debug_port_start handshake completes.
                let mut v = self.ctrl_stat;
                if v & CDBGPWRUPREQ != 0 {
                    v |= CDBGPWRUPACK;
                }
                if v & CSYSPWRUPREQ != 0 {
                    v |= CSYSPWRUPACK;
                }
                Ack::Ok(Some(v))
            }
            (false, 0x4) => {
                self.ctrl_stat = wdata;
                Ack::Ok(None)
            }
            (false, 0x8) => {
                self.select = wdata; // SELECT
                Ack::Ok(None)
            }
            (true, 0x8) => Ack::Ok(Some(0)), // RESEND (unused)
            (true, 0xC) => Ack::Ok(Some(self.posted)), // RDBUFF: last pipelined read
            (false, 0xC) => Ack::Ok(None),
            _ => Ack::Ok(Some(0)),
        }
    }

    fn ap_transfer(&mut self, cpu: &mut Cpu, bus: &mut Bus, rnw: bool, a: u8, wdata: u32) -> Ack {
        let apbanksel = (self.select >> 4) & 0xF;
        let reg = (apbanksel << 4) | (a as u32 & 0x0C); // AP register offset

        if rnw {
            // AP reads are pipelined: return the previously latched word and
            // latch the freshly read one (RDBUFF / the next AP read collects it).
            let out = self.posted;
            self.posted = match reg {
                0x00 => self.csw,
                0x04 => self.tar,
                0x0C => self.drw_read(cpu, bus), // DRW
                0xFC => AP_IDR,                  // IDR (bank 0xF)
                _ => 0,                          // BASE (no ROM table) and others
            };
            Ack::Ok(Some(out))
        } else {
            match reg {
                0x00 => self.csw = wdata,
                0x04 => self.tar = wdata,
                0x0C => self.drw_write(cpu, bus, wdata),
                _ => {}
            }
            Ack::Ok(None)
        }
    }

    // ---- MEM-AP data register, honoring CSW size + address-increment ---------

    /// CSW.Size in bytes (1/2/4); byte(0)/half(1)/word(2), clamped to word.
    fn access_size(&self) -> u32 {
        1u32 << (self.csw & 0x7).min(2)
    }

    /// Advance TAR after a DRW access when CSW.AddrInc == increment-single.
    fn auto_inc(&mut self) {
        if (self.csw >> 4) & 0x3 == 1 {
            self.tar = self.tar.wrapping_add(self.access_size());
        }
    }

    fn drw_read(&mut self, cpu: &mut Cpu, bus: &mut Bus) -> u32 {
        let tar = self.tar;
        let data = match self.access_size() {
            1 => (bus.read8(tar) as u32) << ((tar & 3) * 8),
            2 => (bus.read16(tar) as u32) << ((tar & 2) * 8),
            _ => self.mem_read_word(cpu, bus, tar),
        };
        self.auto_inc();
        data
    }

    fn drw_write(&mut self, cpu: &mut Cpu, bus: &mut Bus, wdata: u32) {
        let tar = self.tar;
        match self.access_size() {
            1 => bus.write8(tar, (wdata >> ((tar & 3) * 8)) as u8),
            2 => bus.write16(tar, (wdata >> ((tar & 2) * 8)) as u16),
            _ => self.mem_write_word(cpu, bus, tar, wdata),
        }
        self.auto_inc();
    }

    // ---- word-wide memory access with the CoreDebug intercept ----------------

    fn mem_read_word(&self, cpu: &Cpu, bus: &mut Bus, addr: u32) -> u32 {
        match addr {
            DHCSR => {
                let mut v = self.dhcsr_ctrl | S_REGRDY;
                if cpu.halted {
                    v |= S_HALT;
                }
                v
            }
            DCRDR => self.dcrdr,
            DEMCR => self.demcr,
            DCRSR => 0,
            // Synthesize the halt reason from live state. Real DFSR bits are
            // sticky/W1C; modeling it as plain storage let probe-rs's own
            // "clear DFSR" write (0x1F) read back as a breakpoint.
            DFSR => {
                if cpu.halted {
                    DFSR_HALTED
                } else {
                    0
                }
            }
            _ => bus.read32(addr),
        }
    }

    fn mem_write_word(&mut self, cpu: &mut Cpu, bus: &mut Bus, addr: u32, val: u32) {
        match addr {
            DHCSR => self.write_dhcsr(cpu, val),
            DCRSR => self.write_dcrsr(cpu, val),
            DCRDR => self.dcrdr = val,
            DEMCR => self.demcr = val,
            DFSR => {} // write-1-to-clear; the read reflects live halt state
            _ => bus.write32(addr, val),
        }
    }

    fn write_dhcsr(&mut self, cpu: &mut Cpu, val: u32) {
        // The write only takes effect with the debug key in the top half.
        if (val >> 16) != DBGKEY {
            return;
        }
        self.dhcsr_ctrl = val & (C_DEBUGEN | C_HALT | C_MASKINTS);
        if val & C_DEBUGEN == 0 {
            return;
        }
        if val & C_HALT != 0 {
            cpu.halted = true;
            self.step_request = false;
        } else {
            // Resume: free-run, or single-step one instruction then re-halt.
            cpu.halted = false;
            self.step_request = val & C_STEP != 0;
        }
    }

    fn write_dcrsr(&mut self, cpu: &mut Cpu, val: u32) {
        let regsel = (val & 0x7F) as u16; // DCRSR.REGSEL == humility ARMRegister numbering
        if val & (1 << 16) != 0 {
            cpu.set_gdb_reg(regsel, self.dcrdr); // REGWnR: write reg from DCRDR
        } else {
            self.dcrdr = cpu.gdb_reg(regsel); // read reg into DCRDR
        }
    }
}
