// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Emulated ARM debug port: an ADIv5 SW-DP + MEM-AP with a working debug core.
//!
//! This is the register-level model probe-rs drives over SWD. The Glasgow-applet
//! server (`glasgow.rs`) decodes `CMD_TRANSFER` into calls to [`SwDp::transfer`];
//! a later phase can drive the same model from the emulated RoT. This is a
//! working debug core, not a read-only memory stub: halt/run/step plus register
//! access via DCRSR/DCRDR, which is what hiffy (halt, write program, run, read
//! results) and endoscope need.
//!
//! sp-emu speaks the Glasgow applet protocol, not raw SWD line bits, so there
//! is no parity/turnaround/clocking here, just ADIv5 register semantics.

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
// DFSR bit1 = BKPT: the halt was a BKPT instruction (or BPU match). probe-rs maps
// this to HaltReason::Breakpoint. endoscope's terminal BKPT reports this.
const DFSR_BKPT: u32 = 1 << 1;
// DFSR bit3 = VCATCH: the halt was a vector catch (DEMCR.VC_CORERESET at the reset
// vector). The RoT's reset_into_debug_halt reads DFSR and confirms `is_vcatch()`
// before trusting the halt, so a plain HALTED report is rejected as "reset not
// caught".
const DFSR_VCATCH: u32 = 1 << 3;

// DHCSR fields.
const DBGKEY: u32 = 0xA05F;
const C_DEBUGEN: u32 = 1 << 0;
const C_HALT: u32 = 1 << 1;
const C_STEP: u32 = 1 << 2;
const C_MASKINTS: u32 = 1 << 3;
const S_REGRDY: u32 = 1 << 16;
const S_HALT: u32 = 1 << 17;
// DHCSR.S_RESET_ST (bit25): set when the core has reset since the last DHCSR
// read, and read-cleared. probe-rs's reset sequence polls this to know the
// reset completed.
const S_RESET_ST: u32 = 1 << 25;

// AIRCR (0xE000ED0C): a SYSRESETREQ with the correct write key triggers a
// system reset. This is the reset humility drives over the debug port.
const AIRCR: u32 = 0xE000_ED0C;
const AIRCR_VECTKEY: u32 = 0x05FA_0000;
const AIRCR_SYSRESETREQ: u32 = 1 << 2;
// DEMCR.VC_CORERESET (bit0): vector catch, halting at the reset vector instead
// of running. probe-rs sets it via reset_catch_set for reset-and-halt / endoscope.
const VC_CORERESET: u32 = 1 << 0;
// Slot-A boot vector table (XIP flash): [0]=initial SP, [1]=reset PC.
const VECTOR_TABLE: u32 = 0x0800_0000;

// RFD 568 SP measurement handoff word (SP RAM). After the RoT measures the SP over
// SWD it writes VALID here; the absent/other value makes the SP self-reset until it
// is measured. Logged so a measurement is observable without full SWD tracing.
const SP_MEASUREMENT_ADDR: u32 = 0x2000_0000;
const SP_MEASUREMENT_VALID: u32 = 0x0c88_7a12;
const SP_MEASUREMENT_SKIP: u32 = 0x9f38_bd71;

/// One SWD transfer's result, mapped by the server to the applet's response byte.
pub enum Ack {
    /// Read data (`Some`) or a write acknowledgement (`None`).
    Ok(Option<u32>),
    /// Protocol acks the applet response encoding defines; the modeled DP
    /// answers immediately and faults nothing, so nothing constructs them.
    #[allow(dead_code)]
    Wait,
    #[allow(dead_code)]
    Fault,
}

/// A SW-DP with a single MEM-AP, plus the CoreDebug semantics behind it.
pub struct SwDp {
    select: u32, // DP SELECT: APSEL[31:24] / APBANKSEL[7:4] / DPBANKSEL[3:0]
    ctrl_stat: u32, // last CTRL/STAT write (its power-req bits drive the ack echo)
    posted: u32, // pipelined AP-read result (returned by the next AP read / RDBUFF)
    csw: u32,    // MEM-AP CSW (size + address-increment)
    tar: u32,    // MEM-AP TAR (transfer address)
    dcrdr: u32,  // DCRDR shadow for DCRSR register-file access
    demcr: u32,  // DEMCR (VC_CORERESET drives halt-after-reset)
    dhcsr_ctrl: u32, // C_DEBUGEN|C_HALT|C_MASKINTS echoed back on DHCSR read
    reset_sticky: bool, // a reset happened; reported once as DHCSR.S_RESET_ST then cleared
    vcatch_halt: bool, // the current halt came from a reset vector catch (reported as DFSR.VCATCH)
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
            reset_sticky: false,
            vcatch_halt: false,
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

    fn ap_transfer(
        &mut self,
        cpu: &mut Cpu,
        bus: &mut Bus,
        rnw: bool,
        a: u8,
        wdata: u32,
    ) -> Ack {
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
                _ => 0, // BASE (no ROM table) and others
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
        // Injecting code (endoscope) into the SP's ITCM, which the core caches
        // while under debug: drop any cached decode this write invalidates so the
        // injected program is never run from a stale decode.
        if tar < 0x0001_0000 {
            cpu.invalidate_decode(tar);
        }
        self.auto_inc();
    }

    // ---- word-wide memory access with the CoreDebug intercept ----------------

    fn mem_read_word(&mut self, cpu: &Cpu, bus: &mut Bus, addr: u32) -> u32 {
        match addr {
            DHCSR => {
                let mut v = self.dhcsr_ctrl | S_REGRDY;
                if cpu.halted {
                    v |= S_HALT;
                }
                // S_RESET_ST is sticky-until-read: report a reset once, so
                // probe-rs's post-reset poll sees it set then cleared.
                if self.reset_sticky {
                    v |= S_RESET_ST;
                    self.reset_sticky = false;
                }
                v
            }
            DCRDR => self.dcrdr,
            DEMCR => self.demcr,
            DCRSR => 0,
            // Synthesize the halt reason from live state. Real DFSR bits are
            // sticky/W1C; plain storage would let probe-rs's own "clear DFSR"
            // write (0x1F) read back as a breakpoint hit.
            DFSR => {
                if cpu.bkpt_hit {
                    DFSR_BKPT
                } else if self.vcatch_halt && cpu.halted {
                    DFSR_VCATCH
                } else if cpu.halted {
                    DFSR_HALTED
                } else {
                    0
                }
            }
            _ => bus.read32(addr),
        }
    }

    fn mem_write_word(
        &mut self,
        cpu: &mut Cpu,
        bus: &mut Bus,
        addr: u32,
        val: u32,
    ) {
        match addr {
            DHCSR => self.write_dhcsr(cpu, val),
            DCRSR => self.write_dcrsr(cpu, val),
            DCRDR => self.dcrdr = val,
            DEMCR => self.demcr = val,
            DFSR => {} // write-1-to-clear; the read reflects live halt state
            AIRCR => {
                if (val & 0xFFFF_0000) == AIRCR_VECTKEY
                    && val & AIRCR_SYSRESETREQ != 0
                {
                    self.do_reset(cpu, bus);
                }
            }
            SP_MEASUREMENT_ADDR if val == SP_MEASUREMENT_VALID => {
                eprintln!(
                    "[rot] SP measurement recorded: VALID token deposited at {SP_MEASUREMENT_ADDR:#010x}"
                );
                bus.write32(addr, val);
            }
            SP_MEASUREMENT_ADDR if val == SP_MEASUREMENT_SKIP => {
                eprintln!(
                    "[rot] SP measurement skipped: SKIP token deposited at {SP_MEASUREMENT_ADDR:#010x}"
                );
                bus.write32(addr, val);
            }
            _ => bus.write32(addr, val),
        }
    }

    /// Reset the SP via its external reset pin (the RoT's ROT_TO_SP_RESET_L pulse).
    /// Same effect as a debug-port SYSRESETREQ: re-boot from the vector table, and
    /// if DEMCR.VC_CORERESET is armed, halt at the reset vector. This is how the
    /// emulated RoT lands the SP in reset-into-debug-halt for its endoscope
    /// measurement (it never writes AIRCR over SWD; it pulses the pin instead).
    pub fn pin_reset(&mut self, cpu: &mut Cpu, bus: &mut Bus) {
        self.do_reset(cpu, bus);
    }

    /// System reset via AIRCR.SYSRESETREQ: re-boot the core from the vector
    /// table. RAM/peripherals persist (a soft reset), matching real silicon; the
    /// firmware's startup re-inits them. If DEMCR.VC_CORERESET is armed, halt at
    /// the reset vector (reset-and-halt / endoscope) instead of running.
    fn do_reset(&mut self, cpu: &mut Cpu, bus: &mut Bus) {
        let sp = bus.read32(VECTOR_TABLE);
        let pc = bus.read32(VECTOR_TABLE + 4) & !1;
        cpu.reset_for_reboot(sp, pc);
        bus.reset_exception_sources(); // a system reset clears the NVIC
        self.reset_sticky = true;
        self.honor_vector_catch(cpu);
    }

    /// Apply an armed reset vector catch after a core reset from any source.
    ///
    /// On silicon `DEMCR.VC_CORERESET` halts the core at the reset vector for any
    /// reset, not only a pin/SWD-driven one. `do_reset` uses this for the RoT's
    /// pin/AIRCR resets; the two-core serve loop also calls it after the SP firmware
    /// drives its own `SYSRESETREQ`, so an armed RoT catches that self-reset too.
    /// Returns whether the catch fired: the core is now halted at the reset vector
    /// (reported as DFSR.VCATCH). A no-op returning `false` when not armed, so the
    /// core keeps running from its reset vector.
    ///
    /// When the catch fires it also marks the reset sticky, so `DHCSR.S_RESET_ST`
    /// reports the reset on the next read, the same as `do_reset` and as silicon does
    /// for a reset from any source. (`do_reset` marks it unconditionally, since a
    /// RoT-driven reset resets the core whether or not the catch is armed; a firmware
    /// self-reset only routes through here, so setting it on the caught path keeps the
    /// two paths consistent.)
    pub fn honor_vector_catch(&mut self, cpu: &mut Cpu) -> bool {
        if self.demcr & VC_CORERESET != 0 {
            cpu.halted = true;
            self.vcatch_halt = true;
            self.reset_sticky = true;
            true
        } else {
            false
        }
    }

    fn write_dhcsr(&mut self, cpu: &mut Cpu, val: u32) {
        // The write only takes effect with the debug key in the top half.
        if (val >> 16) != DBGKEY {
            return;
        }
        self.dhcsr_ctrl = val & (C_DEBUGEN | C_HALT | C_MASKINTS);
        // C_DEBUGEN gates whether a BKPT instruction halts into debug state.
        cpu.debug_en = val & C_DEBUGEN != 0;
        if !cpu.debug_en {
            // Leaving halting-debug (C_DEBUGEN cleared, e.g. the RoT's `end_debug()`
            // after depositing the measurement token): the core exits debug state
            // and runs on from where it was halted.
            cpu.halted = false;
            cpu.bkpt_hit = false;
            self.vcatch_halt = false;
            self.step_request = false;
            return;
        }
        if val & C_HALT != 0 {
            cpu.halted = true;
            self.step_request = false;
        } else {
            // Resume: free-run, or single-step one instruction then re-halt.
            // Clear the breakpoint-hit flag; the debugger is moving past it.
            cpu.halted = false;
            cpu.bkpt_hit = false;
            self.vcatch_halt = false;
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

#[cfg(test)]
mod tests {
    use super::*;

    // A firmware SYSRESETREQ self-reset with the RoT's vector catch armed must halt
    // the SP at the reset vector (0 instructions), and report DFSR.VCATCH rather than
    // a stale BKPT. This mirrors the serve loop's self-reset apply path: reboot the
    // core, then honor an armed DEMCR.VC_CORERESET.
    #[test]
    fn honor_vector_catch_halts_self_reset_and_reports_vcatch() {
        let mut cpu = Cpu::new();
        cpu.bkpt_hit = true; // stale state from an earlier BKPT halt
        cpu.reset_for_reboot(0x2000_1000, 0x0800_0100);
        assert!(
            !cpu.halted,
            "reset_for_reboot leaves the core running on its own"
        );

        let mut swdp = SwDp::new();
        swdp.demcr = VC_CORERESET; // the RoT armed reset-and-halt
        assert!(swdp.honor_vector_catch(&mut cpu), "armed catch fires");
        assert!(cpu.halted, "core halted at the reset vector");
        assert!(swdp.vcatch_halt);

        let mut bus = Bus::new();
        assert_eq!(
            swdp.mem_read_word(&cpu, &mut bus, DFSR),
            DFSR_VCATCH,
            "vector-catch halt reports VCATCH, not a stale BKPT/HALTED"
        );
        // The reset is sticky (S_RESET_ST) on the first DHCSR read, matching do_reset
        // and silicon, then read-clears.
        assert_ne!(
            swdp.mem_read_word(&cpu, &mut bus, DHCSR) & S_RESET_ST,
            0,
            "a caught self-reset reports S_RESET_ST once"
        );
        assert_eq!(
            swdp.mem_read_word(&cpu, &mut bus, DHCSR) & S_RESET_ST,
            0,
            "S_RESET_ST read-clears"
        );
    }

    // Without VC_CORERESET armed, a self-reset must leave the SP running (the
    // early-boot case, before the RoT's swd task configures SP_RESET).
    #[test]
    fn honor_vector_catch_noop_when_not_armed() {
        let mut cpu = Cpu::new();
        cpu.reset_for_reboot(0x2000_1000, 0x0800_0100);
        let mut swdp = SwDp::new(); // demcr == 0
        assert!(!swdp.honor_vector_catch(&mut cpu), "unarmed catch is a no-op");
        assert!(!cpu.halted, "core keeps running from the reset vector");
        assert!(!swdp.vcatch_halt);
        assert!(!swdp.reset_sticky, "an unarmed no-op does not mark a reset");
    }
}
