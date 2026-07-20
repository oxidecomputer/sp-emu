//! Cortex-M7 (ARMv7E-M) CPU core: state, fetch/decode/step, the Thumb-2
//! execution set Hubris uses, IT blocks, the M-profile system registers, and
//! the exception entry/return + SVC machinery to reach the first task.
//!
//! Decode is `yaxpeax-arm`. Execution semantics are hand-written.
//! yaxpeax decodes M-profile MRS/MSR with A-profile semantics, so those two are
//! re-decoded from the raw instruction word here.

use crate::host::HostIo;
use crate::mem::Bus;
use std::rc::Rc;
use yaxpeax_arch::{Decoder, LengthedInstruction, U8Reader};
use yaxpeax_arm::armv7::{ConditionCode, InstDecoder, Opcode, Operand, RegShiftStyle, ShiftStyle};

#[derive(Debug)]
pub enum Trap {
    Decode {
        pc: u32,
    },
    Unimplemented {
        pc: u32,
        bytes: [u8; 4],
        len: u32,
        disasm: String,
    },
    Halt {
        pc: u32,
        why: &'static str,
    },
}

impl Trap {
    /// The program counter at which the trap occurred (every variant carries one).
    pub fn pc(&self) -> u32 {
        match self {
            Trap::Decode { pc } | Trap::Unimplemented { pc, .. } | Trap::Halt { pc, .. } => *pc,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Mode {
    Thread,
    Handler,
}

pub struct Cpu {
    pub r: [u32; 16], // r0..r12, r13=active SP, r14=LR, r15 unused (pc is separate)
    pub pc: u32,
    pub n: bool,
    pub z: bool,
    pub c: bool,
    pub v: bool,
    pub q: bool,
    // M-profile system state
    pub mode: Mode,
    pub control: u32, // bit0 nPRIV, bit1 SPSEL, bit2 FPCA
    pub primask: bool,
    pub basepri: u32,
    pub faultmask: bool,
    pub ipsr: u32,
    pub msp: u32,
    pub psp: u32,
    sp_is_psp: bool,
    pub itstate: u8,
    pub s: [u32; 32], // VFP single-precision registers (s0..s31; d-regs are pairs)
    pub fpscr: u32,
    cur_insn: u32, // address of the instruction currently executing
    pub cycles: u64,
    pub entered_task: bool,
    pub bad_ret_dumps: u32, // crash detector: count of corrupt-return-PC dumps emitted
    pub last_vfp: bool,     // last step was a VFP instr (differential harness skip)
    pub last_it: bool,      // last step was an IT or IT-gated instr (harness skip)
    pub last_sys: bool,     // last step was MRS/MSR/CPS (Unicorn can't decode M-profile)
    cur_in_it: bool,        // currently executing inside an IT block
    cur_setflags: bool,     // effective flag-setting for the current instr (S, suppressed in IT)
    systick: u32,           // SysTick down-counter (driven per instruction)
    pub wfi_throttle: bool, // enable WFI idle-throttle (set by the gdb serve loop, post-preboot)
    pub idle_skip: u32,     // instrs skipped to the next tick on an idle WFI (loop sleeps instead)
    pub record_disasm: bool, // populate last_disasm per-instruction (only when tracing/diff is on)
    pub halted: bool,       // external debug halt (DHCSR C_HALT via the SWD debug port)
    pub debug_en: bool,     // DHCSR.C_DEBUGEN set: a BKPT halts into debug state (else it faults)
    pub bkpt_hit: bool,     // last halt was a BKPT instruction (reported as DFSR.BKPT)
    pub trace_svc: bool,    // log Hubris syscalls (Sysnum in r11) at each SVC — RoT IPC tracing
    pub last_disasm: String,
    decoder: InstDecoder,
    /// PC-keyed decode cache. Hubris executes in place from immutable flash, so
    /// the decode of an instruction at a given flash PC never changes; caching it
    /// removes the per-instruction fetch (two RAM reads) + yaxpeax decode, the
    /// dominant hot-loop cost. Only flash-window PCs are cached: the running
    /// image is never self-modified, and the flash-update path writes the other slot.
    dcache: std::collections::HashMap<u32, Rc<Decoded>, PcBuildHasher>,
    /// PC window whose decodes are cacheable: the immutable XIP flash of *this*
    /// core instance. sp-emu instantiates this one interpreter twice -- a separate
    /// `Cpu`+`Bus` for the STM32H7 SP and for the LPC55 RoT (independent registers,
    /// memory map, and decode cache); they only differ by where their image is
    /// mapped. So the cacheable window is per-instance: it defaults to the SP's
    /// flash window (`FLASH_LO..FLASH_HI`, 0x0800_0000) and the RoT instance sets
    /// its own (its image span at 0x0001_0000) via `set_flash_cache`. PCs outside
    /// the window (RAM, the ITCM-injected endoscope) are decoded fresh every time,
    /// so self-modified or injected code is never served a stale decode.
    flash_cache: std::ops::Range<u32>,
    syst_csr: u32, // cached SYST_CSR (refreshed periodically in maybe_tick)
    syst_rvr: u32, // cached SYST_RVR reload value (>=1)
}

/// A cached fetch+decode for one instruction at a fixed flash PC.
struct Decoded {
    raw: u32,     // the 32-bit little-endian instruction word (operands re-read from this)
    buf: [u8; 4], // the raw bytes (for Trap reporting)
    len: u32,     // encoded length (2 or 4)
    inst: Option<yaxpeax_arm::armv7::Instruction>, // None => yaxpeax Err (VFP / try_vfp on raw)
}

const SP: usize = 13;
const LR: usize = 14;
/// Flash window (2 MB). PCs here are XIP and immutable -> safe to cache decodes.
const FLASH_LO: u32 = 0x0800_0000;
const FLASH_HI: u32 = 0x0a00_0000;

/// Upper bound of the SP's ITCM (0x0000_0000..0x0001_0000). Injected code (the
/// RoT's endoscope measurement program) runs here. It is cacheable only while the
/// core is under debug control, and its entries are invalidated on debug-port
/// writes -- see `flash_cache` / `invalidate_decode` and `step`.
const ITCM_HI: u32 = 0x0001_0000;

/// Fast hasher for the decode cache's `u32` flash-PC keys. `HashMap`'s default
/// SipHash is DoS-resistant but far slower than needed, and the decode-cache
/// lookup is the interpreter's dominant per-instruction cost. PCs are dense and
/// 2-byte aligned, so a single Fibonacci multiply (2^64 / golden ratio) scatters
/// them across buckets without clustering; the keys are our own PCs, so SipHash's
/// DoS resistance buys nothing here. Behavior is identical -- only the bucket
/// mapping changes.
#[derive(Default)]
struct PcHasher(u64);
impl std::hash::Hasher for PcHasher {
    #[inline]
    fn write_u32(&mut self, n: u32) {
        self.0 = (n as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }
    #[inline]
    fn write(&mut self, _: &[u8]) {
        unreachable!("decode-cache keys are hashed via write_u32");
    }
    #[inline]
    fn finish(&self) -> u64 {
        self.0
    }
}
type PcBuildHasher = std::hash::BuildHasherDefault<PcHasher>;

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}

impl Cpu {
    pub fn new() -> Self {
        Cpu {
            r: [0; 16],
            pc: 0,
            n: false,
            z: false,
            c: false,
            v: false,
            q: false,
            mode: Mode::Thread,
            control: 0,
            primask: false,
            basepri: 0,
            faultmask: false,
            ipsr: 0,
            msp: 0,
            psp: 0,
            sp_is_psp: false,
            itstate: 0,
            s: [0; 32],
            fpscr: 0,
            cur_insn: 0,
            cycles: 0,
            entered_task: false,
            bad_ret_dumps: 0,
            last_vfp: false,
            last_it: false,
            last_sys: false,
            cur_in_it: false,
            cur_setflags: false,
            systick: 0,
            wfi_throttle: false,
            idle_skip: 0,
            record_disasm: false,
            halted: false,
            debug_en: false,
            bkpt_hit: false,
            trace_svc: false,
            last_disasm: String::new(),
            decoder: InstDecoder::default_thumb(),
            dcache: std::collections::HashMap::default(),
            flash_cache: FLASH_LO..FLASH_HI,
            syst_csr: 0,
            syst_rvr: 1,
        }
    }

    /// NZCVQ flags packed into the top bits of a word (APSR layout).
    pub fn apsr(&self) -> u32 {
        ((self.n as u32) << 31)
            | ((self.z as u32) << 30)
            | ((self.c as u32) << 29)
            | ((self.v as u32) << 28)
            | ((self.q as u32) << 27)
    }

    /// The two stack pointers, resolved to their banked values regardless of
    /// which one is currently active (`r[13]`).
    pub fn current_msp(&self) -> u32 {
        if self.sp_is_psp {
            self.msp
        } else {
            self.r[SP]
        }
    }
    pub fn current_psp(&self) -> u32 {
        if self.sp_is_psp {
            self.r[SP]
        } else {
            self.psp
        }
    }

    /// xPSR as a debugger sees it: APSR flags | IPSR exception number | Thumb.
    pub fn xpsr(&self) -> u32 {
        self.apsr() | (self.ipsr & 0x1ff) | (1 << 24)
    }

    /// Read a register by its GDB/ARM number (the operand of humility's `p`
    /// packet), per humility's `ARMRegister` encoding.
    pub fn gdb_reg(&self, n: u16) -> u32 {
        match n {
            0..=12 => self.r[n as usize],
            13 => self.r[SP],
            14 => self.r[LR],
            15 => self.pc,
            16 => self.xpsr(),        // PSR
            17 => self.current_msp(), // MSP
            18 => self.current_psp(), // PSP
            20 => {
                (self.primask as u32)               // SPR: {CONTROL,FAULTMASK,BASEPRI,PRIMASK}
                | (self.basepri << 8)
                | ((self.faultmask as u32) << 16)
                | (self.control << 24)
            }
            33 => self.fpscr,                     // FPSCR
            64..=95 => self.s[(n - 64) as usize], // S0..S31
            _ => 0,
        }
    }

    /// Write a register by its ARM DCRSR REGSEL, which shares humility's
    /// `ARMRegister` numbering used by [`gdb_reg`]. Inverse of `gdb_reg`; used by
    /// the SWD debug port's DCRSR/DCRDR register-file access (hiffy/endoscope
    /// load r0..r15 + xPSR + SP before running an injected program).
    pub fn set_gdb_reg(&mut self, n: u16, v: u32) {
        match n {
            0..=12 => self.r[n as usize] = v,
            13 => self.r[SP] = v,
            14 => self.r[LR] = v,
            15 => self.pc = v & !1,
            16 => {
                self.set_xpsr_flags(v);
                self.ipsr = v & 0x1ff;
            } // PSR
            17 => {
                self.msp = v;
                if !self.sp_is_psp {
                    self.r[SP] = v;
                }
            } // MSP
            18 => {
                self.psp = v;
                if self.sp_is_psp {
                    self.r[SP] = v;
                }
            } // PSP
            20 => {
                // SPR: {CONTROL,FAULTMASK,BASEPRI,PRIMASK}
                self.primask = v & 1 != 0;
                self.basepri = (v >> 8) & 0xff;
                self.faultmask = (v >> 16) & 1 != 0;
                self.set_control(v >> 24);
            }
            33 => self.fpscr = v,                     // FPSCR
            64..=95 => self.s[(n - 64) as usize] = v, // S0..S31
            _ => {}
        }
    }

    /// Set the PC window whose decodes are cached -- this instance's own immutable
    /// XIP flash. The SP and RoT are separate `Cpu` instances (see `flash_cache`)
    /// whose images sit at different flash bases (SP 0x0800_0000, RoT/LPC55
    /// 0x0001_0000), so each configures its own window; this is a memory-map
    /// setting, not a change of core architecture. The range must cover only
    /// immutable code -- RAM or injected/self-modified regions must stay outside
    /// it, or their stale decodes would be reused.
    pub fn set_flash_cache(&mut self, range: std::ops::Range<u32>) {
        self.flash_cache = range;
    }

    /// Drop cached decodes overlapping a write at `addr` (the write's word plus a
    /// possible 4-byte instruction straddling into it from `addr-2`). The debug
    /// port calls this when it writes to a region whose decodes may be cached
    /// (the ITCM the RoT injects endoscope into), so a re-injection is never
    /// served a stale decode. No-op for the common case where nothing there is
    /// cached.
    pub fn invalidate_decode(&mut self, addr: u32) {
        if self.dcache.is_empty() {
            return;
        }
        for a in [addr.wrapping_sub(2), addr, addr.wrapping_add(2)] {
            self.dcache.remove(&a);
        }
    }

    pub fn reset(&mut self, sp: u32, pc: u32) {
        self.msp = sp;
        self.r[SP] = sp;
        self.sp_is_psp = false;
        self.pc = pc;
        self.mode = Mode::Thread;
    }

    /// Full architectural reset to the boot vector, as a SYSRESETREQ (system
    /// reset) produces: SP/PC from the vector table plus all core execution
    /// state back to its reset values. RAM and peripherals are left as-is (a
    /// soft reset does not clear them; the firmware's startup re-inits them).
    /// Used by the debug port's AIRCR reset path.
    pub fn reset_for_reboot(&mut self, sp: u32, pc: u32) {
        self.reset(sp, pc);
        self.control = 0;
        self.primask = false;
        self.basepri = 0;
        self.faultmask = false;
        self.ipsr = 0;
        self.itstate = 0;
        self.psp = 0;
        self.n = false;
        self.z = false;
        self.c = false;
        self.v = false;
        self.q = false;
        self.systick = 0;
        self.halted = false;
        // A reset clears any "last halt was a BKPT" state, so a subsequent
        // vector-catch halt reports DFSR.VCATCH rather than a stale DFSR.BKPT.
        self.bkpt_hit = false;
    }

    #[inline]
    fn read_reg(&self, i: u8) -> u32 {
        if i == 15 {
            self.cur_insn.wrapping_add(4)
        } else {
            self.r[i as usize]
        }
    }
    #[inline]
    fn write_reg(&mut self, i: u8, v: u32) {
        if i == 15 {
            self.pc = v & !1;
        } else {
            self.r[i as usize] = v;
        }
    }

    pub fn step(&mut self, bus: &mut Bus, host: &mut dyn HostIo) -> Result<(), Trap> {
        let pc = self.pc;
        self.cur_insn = pc;
        bus.cur_pc = pc;
        bus.cur_cyc = self.cycles;
        self.last_vfp = false;
        self.last_sys = false;
        // Fetch + decode, via the PC-keyed cache for flash code (the common case).
        // Cacheable: this core's immutable flash, or -- only while the core is
        // under debug control -- its ITCM. The debug case lets the injected
        // endoscope program (which loops over the flash hash for ~140M
        // instructions) be decoded once instead of every instruction; the debug
        // port invalidates those entries on injection (`invalidate_decode`), and
        // gating on `debug_en` keeps normal execution's mutable ITCM uncached.
        let cacheable = self.flash_cache.contains(&pc) || (self.debug_en && pc < ITCM_HI);
        let dec: Rc<Decoded> = if cacheable {
            match self.dcache.get(&pc) {
                Some(d) => d.clone(), // Rc bump; decouples from the &mut self execute below
                None => {
                    let d = Rc::new(self.fetch_decode(pc, bus));
                    self.dcache.insert(pc, d.clone());
                    d
                }
            }
        } else {
            Rc::new(self.fetch_decode(pc, bus))
        };
        let raw = dec.raw;
        let buf = dec.buf;

        let is_it_insn = matches!(&dec.inst, Some(i) if i.opcode == Opcode::IT);
        let in_it = (self.itstate & 0xF) != 0;
        self.cur_in_it = in_it;
        self.last_it = in_it || is_it_insn;
        let cond_ok = if in_it && !is_it_insn {
            self.cond_holds(cond_from_bits((self.itstate >> 4) & 0xF))
        } else {
            true
        };
        let advance_it = in_it && !is_it_insn;

        self.cycles += 1;
        let res = match &dec.inst {
            Some(inst) => {
                let len = dec.len;
                // Formatting the disassembly is a heap alloc per instruction; only
                // do it when last_disasm is read (trace/diff). In production this is
                // the largest per-instruction cost removed.
                if self.record_disasm {
                    self.last_disasm = format!("{}", inst);
                }
                self.pc = pc.wrapping_add(len);
                if cond_ok {
                    self.execute(inst, pc, len, raw, bus, host)
                        .map_err(|_| Trap::Unimplemented {
                            pc,
                            bytes: buf,
                            len,
                            disasm: format!("{}", inst),
                        })
                } else {
                    Ok(()) // condition false: skip the instruction
                }
            }
            None => {
                // VFP (or genuinely unimplemented). Thumb VFP encodings are 4 bytes.
                if cond_ok {
                    if self.try_v8m(raw, pc, bus) || self.try_vfp(raw, pc, bus) {
                        Ok(())
                    } else {
                        Err(Trap::Decode { pc })
                    }
                } else {
                    self.pc = pc.wrapping_add(4); // skip the conditional VFP instr
                    Ok(())
                }
            }
        };

        if advance_it {
            self.it_advance();
        }

        res
    }

    /// ARMv8-M load-acquire / store-release (LDA/STL + B/H + LDAEX/STLEX), which
    /// yaxpeax's ARMv7 decoder rejects. The LPC55 RoT (Cortex-M33) uses these for
    /// atomics/sync. On a single-core emulator the acquire/release/exclusive
    /// semantics reduce to plain loads/stores (exclusives always succeed).
    /// hw1 = 1110_1000_110L_Rnnn (0xE8C0 store, 0xE8D0 load); hw2[11:8]=1111,
    /// hw2[7:4] = size/exclusive selector.
    fn try_v8m(&mut self, raw: u32, pc: u32, bus: &mut Bus) -> bool {
        let hw1 = (raw & 0xFFFF) as u16;
        let hw2 = ((raw >> 16) & 0xFFFF) as u16;
        if hw1 & 0xFFE0 != 0xE8C0 {
            return false;
        }
        if (hw2 >> 8) & 0xF != 0xF {
            return false;
        }
        let (size, exclusive) = match (hw2 >> 4) & 0xF {
            0x8 => (1u8, false),
            0x9 => (2, false),
            0xA => (4, false),
            0xC => (1, true),
            0xD => (2, true),
            0xE => (4, true),
            _ => return false,
        };
        let load = hw1 & 0x10 != 0;
        let rn = (hw1 & 0xF) as usize;
        let rt = ((hw2 >> 12) & 0xF) as usize;
        let addr = self.r[rn];
        if load {
            self.r[rt] = match size {
                1 => bus.read8(addr) as u32,
                2 => bus.read16(addr) as u32,
                _ => bus.read32(addr),
            };
        } else {
            let v = self.r[rt];
            match size {
                1 => bus.write8(addr, v as u8),
                2 => bus.write16(addr, v as u16),
                _ => bus.write32(addr, v),
            }
            if exclusive {
                self.r[(hw2 & 0xF) as usize] = 0;
            } // STLEX: report success
        }
        self.pc = pc.wrapping_add(4);
        true
    }

    fn fetch_decode(&self, pc: u32, bus: &mut Bus) -> Decoded {
        let mut buf = [0u8; 4];
        buf[0..2].copy_from_slice(&bus.read16(pc).to_le_bytes());
        buf[2..4].copy_from_slice(&bus.read16(pc.wrapping_add(2)).to_le_bytes());
        let raw = u32::from_le_bytes(buf);
        let mut reader = U8Reader::new(&buf);
        match self.decoder.decode(&mut reader) {
            Ok(inst) => {
                let len = inst.len().to_const();
                Decoded {
                    raw,
                    buf,
                    len,
                    inst: Some(inst),
                }
            }
            Err(_) => Decoded {
                raw,
                buf,
                len: 4,
                inst: None,
            },
        }
    }

    fn execute(
        &mut self,
        inst: &yaxpeax_arm::armv7::Instruction,
        pc: u32,
        len: u32,
        raw: u32,
        bus: &mut Bus,
        _host: &mut dyn HostIo,
    ) -> Result<(), ()> {
        let ops = &inst.operands;
        // ARM rule: 16-bit flag-setting data-processing instructions inside an
        // IT block do NOT update the flags (the implicit S is suppressed). yaxpeax
        // still reports them as `movs`/`adds` etc., so suppress here.
        let s = inst.s && !(self.cur_in_it && len == 2);
        self.cur_setflags = s; // so alu()/shift_op() honor the IT-block suppression too
        match inst.opcode {
            Opcode::NOP => Ok(()),
            Opcode::DSB | Opcode::DMB | Opcode::ISB | Opcode::YIELD | Opcode::WFE => Ok(()),
            Opcode::WFI => {
                // Wait-for-interrupt. When idle-throttling is enabled (the gdb
                // serve loop, post-preboot) and nothing is pending, skip the idle
                // spin: record the instructions otherwise burned down to the next
                // SysTick so the run loop can sleep the host instead of pegging a
                // core, and collapse the countdown so the tick fires promptly.
                // Preboot/run/diff keep wfi_throttle=false, so WFI is a plain nop
                // there (full-speed boot). The core still wakes immediately on a
                // real IRQ (e.g. eth-irq from an injected MGS packet).
                if self.wfi_throttle && self.systick > 1 && !bus.any_pending_irq() {
                    self.idle_skip = self.systick;
                    self.systick = 1;
                }
                Ok(())
            }

            Opcode::IT => {
                let firstcond = imm_val(&ops[0])? as u8 & 0xF;
                let mask = imm_val(&ops[1])? as u8 & 0xF;
                self.itstate = (firstcond << 4) | mask;
                Ok(())
            }

            Opcode::PUSH => self.stmdb_regs(bus, SP as u8, reglist(&ops[0])?, true),
            Opcode::POP => self.ldmia_regs(bus, SP as u8, reglist(&ops[0])?, true),

            // yaxpeax's LDM/STM (add, pre) tuple is unreliable (it reports 16-bit
            // STMIA as increment-before), so decode the addressing mode from raw.
            Opcode::STM(..) => {
                let (rn, wb) = regwback(&ops[0])?;
                let (add, pre) = ldm_stm_mode(raw, len);
                self.block_transfer(bus, rn, wb, reglist(&ops[1])?, add, pre, true)
            }
            Opcode::LDM(..) => {
                let (rn, wb) = regwback(&ops[0])?;
                let (add, pre) = ldm_stm_mode(raw, len);
                self.block_transfer(bus, rn, wb, reglist(&ops[1])?, add, pre, false)
            }

            Opcode::MOV => {
                let rd = reg(&ops[0])?;
                // yaxpeax mis-decodes two encodings here: the MOVW imm4 field
                // (shifts by 16 not 12), and MVN-modified-immediate (reports it
                // as MOV). Both surface as Opcode::MOV, so disambiguate via raw.
                let val = if raw & 0xFBF0 == 0xF240 {
                    movw_movt_imm16(raw) // MOVW: 16-bit immediate
                } else if raw & 0xFBEF == 0xF06F {
                    !self.opval(&ops[1])? // really MVN #imm
                } else {
                    self.opval(&ops[1])?
                };
                self.write_reg(rd, val);
                if s {
                    self.set_nz(val);
                }
                Ok(())
            }
            Opcode::MOVT => {
                let rd = reg(&ops[0])?;
                let imm16 = movw_movt_imm16(raw); // same yaxpeax quirk as MOVW
                let cur = self.read_reg(rd);
                self.write_reg(rd, (cur & 0xffff) | (imm16 << 16));
                Ok(())
            }
            Opcode::MVN => {
                let rd = reg(&ops[0])?;
                let val = !self.opval(&ops[1])?;
                self.write_reg(rd, val);
                if s {
                    self.set_nz(val);
                }
                Ok(())
            }

            Opcode::ADD => self.alu(ops, Alu::Add),
            Opcode::ADC => self.alu(ops, Alu::Adc),
            Opcode::SUB => self.alu(ops, Alu::Sub),
            Opcode::SBC => self.alu(ops, Alu::Sbc),
            Opcode::RSB => self.alu(ops, Alu::Rsb),
            Opcode::AND => self.alu(ops, Alu::And),
            Opcode::ORR => self.alu(ops, Alu::Orr),
            Opcode::ORN => self.alu(ops, Alu::Orn),
            Opcode::EOR => self.alu(ops, Alu::Eor),
            Opcode::BIC => self.alu(ops, Alu::Bic),

            // yaxpeax 0.4 mis-decodes the shift TYPE of the 32-bit register-form
            // shift (T2: `LSL/LSR/ASR/ROR.w Rd, Rn, Rm`); e.g. it reports `lsr.w`
            // as `lsl.w`. The type lives in raw hw1 bits[6:5]; decode it ourselves.
            Opcode::LSL | Opcode::LSR | Opcode::ASR | Opcode::ROR => {
                let dflt = match inst.opcode {
                    Opcode::LSL => ShiftStyle::LSL,
                    Opcode::LSR => ShiftStyle::LSR,
                    Opcode::ASR => ShiftStyle::ASR,
                    _ => ShiftStyle::ROR,
                };
                self.shift_op(ops, t2_reg_shift_style(raw, len).unwrap_or(dflt))
            }

            Opcode::CMP => {
                let a = self.read_reg(reg(&ops[0])?);
                let b = self.opval(&ops[1])?;
                self.flags_sub(a, b);
                Ok(())
            }
            Opcode::CMN => {
                let a = self.read_reg(reg(&ops[0])?);
                let b = self.opval(&ops[1])?;
                self.flags_add(a, b);
                Ok(())
            }
            Opcode::TST => {
                let a = self.read_reg(reg(&ops[0])?);
                let b = self.opval(&ops[1])?;
                self.set_nz(a & b);
                Ok(())
            }
            Opcode::TEQ => {
                let a = self.read_reg(reg(&ops[0])?);
                let b = self.opval(&ops[1])?;
                self.set_nz(a ^ b);
                Ok(())
            }

            // Extends (optionally with a rotate, and the "A" add forms).
            Opcode::UXTB => self.extend(ops, 8, false, false, raw, len),
            Opcode::UXTH => self.extend(ops, 16, false, false, raw, len),
            Opcode::SXTB => self.extend(ops, 8, true, false, raw, len),
            Opcode::SXTH => self.extend(ops, 16, true, false, raw, len),
            Opcode::UXTAB => self.extend(ops, 8, false, true, raw, len),
            Opcode::UXTAH => self.extend(ops, 16, false, true, raw, len),
            Opcode::SXTAB => self.extend(ops, 8, true, true, raw, len),
            Opcode::SXTAH => self.extend(ops, 16, true, true, raw, len),

            Opcode::MUL => {
                let a = self.read_reg(reg(&ops[1])?);
                let b = self.read_reg(reg(&ops[2])?);
                let r = a.wrapping_mul(b);
                self.write_reg(reg(&ops[0])?, r);
                if s {
                    self.set_nz(r);
                }
                Ok(())
            }
            Opcode::MLA => {
                let a = self.read_reg(reg(&ops[1])?);
                let b = self.read_reg(reg(&ops[2])?);
                let c = self.read_reg(reg(&ops[3])?);
                self.write_reg(reg(&ops[0])?, a.wrapping_mul(b).wrapping_add(c));
                Ok(())
            }
            Opcode::MLS => {
                let a = self.read_reg(reg(&ops[1])?);
                let b = self.read_reg(reg(&ops[2])?);
                let c = self.read_reg(reg(&ops[3])?);
                self.write_reg(reg(&ops[0])?, c.wrapping_sub(a.wrapping_mul(b)));
                Ok(())
            }
            Opcode::UMULL => {
                let a = self.read_reg(reg(&ops[2])?) as u64;
                let b = self.read_reg(reg(&ops[3])?) as u64;
                let p = a * b;
                self.write_reg(reg(&ops[0])?, p as u32);
                self.write_reg(reg(&ops[1])?, (p >> 32) as u32);
                Ok(())
            }
            Opcode::SMULL => {
                let a = self.read_reg(reg(&ops[2])?) as i32 as i64;
                let b = self.read_reg(reg(&ops[3])?) as i32 as i64;
                let p = a * b;
                self.write_reg(reg(&ops[0])?, p as u32);
                self.write_reg(reg(&ops[1])?, (p >> 32) as u32);
                Ok(())
            }
            // Multiply-accumulate long: {RdHi:RdLo} += Rn * Rm (un/signed 64-bit).
            Opcode::UMLAL => {
                let a = self.read_reg(reg(&ops[2])?) as u64;
                let b = self.read_reg(reg(&ops[3])?) as u64;
                let acc = ((self.read_reg(reg(&ops[1])?) as u64) << 32)
                    | self.read_reg(reg(&ops[0])?) as u64;
                let p = acc.wrapping_add(a * b);
                self.write_reg(reg(&ops[0])?, p as u32);
                self.write_reg(reg(&ops[1])?, (p >> 32) as u32);
                Ok(())
            }
            Opcode::SMLAL => {
                let a = self.read_reg(reg(&ops[2])?) as i32 as i64;
                let b = self.read_reg(reg(&ops[3])?) as i32 as i64;
                let acc = (((self.read_reg(reg(&ops[1])?) as u64) << 32)
                    | self.read_reg(reg(&ops[0])?) as u64) as i64;
                let p = acc.wrapping_add(a * b);
                self.write_reg(reg(&ops[0])?, p as u32);
                self.write_reg(reg(&ops[1])?, (p >> 32) as u32);
                Ok(())
            }
            // UMAAL: {RdHi:RdLo} = Rn*Rm + RdLo + RdHi (both accumulators added
            // as separate u32s, unlike UMLAL's 64-bit accumulator). Used heavily
            // by Ed25519 field arithmetic (salty), so the RoT's DICE key
            // derivation needs it. Cannot overflow u64: max is (2^32-1)^2 + 2*(2^32-1) = 2^64-1.
            Opcode::UMAAL => {
                let rn = self.read_reg(reg(&ops[2])?) as u64;
                let rm = self.read_reg(reg(&ops[3])?) as u64;
                let lo = self.read_reg(reg(&ops[0])?) as u64;
                let hi = self.read_reg(reg(&ops[1])?) as u64;
                let p = rn.wrapping_mul(rm).wrapping_add(lo).wrapping_add(hi);
                self.write_reg(reg(&ops[0])?, p as u32);
                self.write_reg(reg(&ops[1])?, (p >> 32) as u32);
                Ok(())
            }
            Opcode::UDIV => {
                let a = self.read_reg(reg(&ops[1])?);
                let b = self.read_reg(reg(&ops[2])?);
                self.write_reg(reg(&ops[0])?, if b == 0 { 0 } else { a / b });
                Ok(())
            }
            Opcode::SDIV => {
                let a = self.read_reg(reg(&ops[1])?) as i32;
                let b = self.read_reg(reg(&ops[2])?) as i32;
                self.write_reg(
                    reg(&ops[0])?,
                    if b == 0 {
                        0
                    } else {
                        (a.wrapping_div(b)) as u32
                    },
                );
                Ok(())
            }
            Opcode::SMLA(_, _) => {
                // signed multiply-accumulate (halfword forms): approximate with low halves
                let a = (self.read_reg(reg(&ops[1])?) as i16) as i32;
                let b = (self.read_reg(reg(&ops[2])?) as i16) as i32;
                let c = self.read_reg(reg(&ops[3])?);
                self.write_reg(reg(&ops[0])?, (a * b) as u32 + c);
                Ok(())
            }

            Opcode::CLZ => {
                let v = self.read_reg(reg(&ops[1])?);
                self.write_reg(reg(&ops[0])?, v.leading_zeros());
                Ok(())
            }
            Opcode::REV => {
                let v = self.read_reg(reg(&ops[1])?);
                self.write_reg(reg(&ops[0])?, v.swap_bytes());
                Ok(())
            }
            Opcode::REV16 => {
                let v = self.read_reg(reg(&ops[1])?);
                self.write_reg(
                    reg(&ops[0])?,
                    ((v & 0xff) << 8)
                        | ((v >> 8) & 0xff)
                        | ((v & 0xff0000) << 8)
                        | ((v >> 8) & 0xff0000),
                );
                Ok(())
            }
            Opcode::RBIT => {
                let v = self.read_reg(reg(&ops[1])?);
                self.write_reg(reg(&ops[0])?, v.reverse_bits());
                Ok(())
            }
            // REVSH: byte-swap the low halfword, then sign-extend bit 15 to 32 bits.
            // result<31:8>=SignExtend(Rm<7:0>), result<7:0>=Rm<15:8>.
            Opcode::REVSH => {
                let v = self.read_reg(reg(&ops[1])?);
                let half = (((v & 0xff) << 8) | ((v >> 8) & 0xff)) as u16;
                self.write_reg(reg(&ops[0])?, half as i16 as i32 as u32);
                Ok(())
            }

            // NB: yaxpeax gives the 4th operand of BFI/BFC as `msb`, not width
            // (its own source flags this as a known quirk), so derive width here.
            // yaxpeax 0.4 mis-decodes BFI/BFC's msb field (e.g. reports msb=15 for
            // a `& 0x7fff` clear that should have msb=31), so derive lsb/msb from
            // the raw T1 encoding: hw2 = (0)imm3 Rd imm2 (0) msb[4:0]; lsb=imm3:imm2.
            Opcode::BFI => {
                let rd = reg(&ops[0])?;
                let rn = self.read_reg(reg(&ops[1])?);
                let (lsb, msb) = bfx_lsb_msb(raw);
                let width = msb.saturating_sub(lsb) + 1;
                let mask = if width >= 32 {
                    u32::MAX
                } else {
                    ((1u32 << width) - 1) << lsb
                };
                let cur = self.read_reg(rd);
                self.write_reg(rd, (cur & !mask) | ((rn << lsb) & mask));
                Ok(())
            }
            Opcode::BFC => {
                let rd = reg(&ops[0])?;
                let (lsb, msb) = bfx_lsb_msb(raw);
                let width = msb.saturating_sub(lsb) + 1;
                let mask = if width >= 32 {
                    u32::MAX
                } else {
                    ((1u32 << width) - 1) << lsb
                };
                let cur = self.read_reg(rd);
                self.write_reg(rd, cur & !mask);
                Ok(())
            }
            Opcode::UBFX => {
                let rn = self.read_reg(reg(&ops[1])?);
                let lsb = imm_val(&ops[2])?;
                let width = imm_val(&ops[3])?;
                let mask = if width >= 32 {
                    u32::MAX
                } else {
                    (1u32 << width) - 1
                };
                self.write_reg(reg(&ops[0])?, (rn >> lsb) & mask);
                Ok(())
            }
            Opcode::SBFX => {
                let rn = self.read_reg(reg(&ops[1])?);
                let lsb = imm_val(&ops[2])?;
                let width = imm_val(&ops[3])?;
                let shifted = rn >> lsb;
                let sext = if width >= 32 {
                    shifted
                } else {
                    let m = 1u32 << (width - 1);
                    ((shifted & ((1 << width) - 1)) ^ m).wrapping_sub(m)
                };
                self.write_reg(reg(&ops[0])?, sext);
                Ok(())
            }

            // PKHBT/PKHTB (pack halfword). Decode T1 from raw: Rd=Rn[lo]|shifted
            // Rm[hi] for BT (LSL), Rn[hi]|shifted Rm[lo] for TB (ASR). net's smoltcp
            // uses this to assemble values; if skipped, a register keeps a stale
            // (garbage-pointer) value.
            Opcode::PKHBT | Opcode::PKHTB => {
                let (hw1, hw2) = ((raw & 0xFFFF) as u16, (raw >> 16) as u16);
                let rn = (hw1 & 0xF) as u8;
                let rd = ((hw2 >> 8) & 0xF) as u8;
                let rm = (hw2 & 0xF) as u8;
                let shift = ((((hw2 >> 12) & 0x7) << 2) | ((hw2 >> 6) & 0x3)) as u32;
                let tb = (hw2 >> 5) & 1; // 0 = PKHBT (LSL), 1 = PKHTB (ASR)
                let (vn, vm) = (self.read_reg(rn), self.read_reg(rm));
                let result = if tb == 0 {
                    let op2 = vm.wrapping_shl(shift & 31);
                    (vn & 0x0000_FFFF) | (op2 & 0xFFFF_0000)
                } else {
                    let amt = if shift == 0 { 32 } else { shift }; // imm 0 means ASR #32
                    let op2 = ((vm as i32) >> amt.min(31)) as u32;
                    (vn & 0xFFFF_0000) | (op2 & 0x0000_FFFF)
                };
                self.write_reg(rd, result);
                Ok(())
            }

            Opcode::LDR => self.load(ops, bus, 4, false, raw, len),
            Opcode::LDRB => self.load(ops, bus, 1, false, raw, len),
            Opcode::LDRH => self.load(ops, bus, 2, false, raw, len),
            Opcode::LDRSB => self.load(ops, bus, 1, true, raw, len),
            Opcode::LDRSH => self.load(ops, bus, 2, true, raw, len),
            Opcode::STR => self.store(ops, bus, 4, raw, len),
            Opcode::STRB => self.store(ops, bus, 1, raw, len),
            Opcode::STRH => self.store(ops, bus, 2, raw, len),
            Opcode::LDRD => self.load_double(ops, bus),
            Opcode::STRD => self.store_double(ops, bus),
            // Unprivileged load/store (LDR*T/STR*T). yaxpeax also mis-decodes some
            // 32-bit T2 imm12 byte/half/word loads-stores as these "T" variants
            // (e.g. `strb r4,[sp,#3612]` -> `strbt`), so handle them identically to
            // the privileged form: the emulator doesn't enforce MPU privilege, and
            // store()/load() recompute the address from raw bits via mem_addr32,
            // ignoring yaxpeax's wrong operands. Skipping these dropped a byte
            // store -> corrupted net's socket handle -> garbage waker -> spin.
            Opcode::STRBT => self.store(ops, bus, 1, raw, len),
            Opcode::STRHT => self.store(ops, bus, 2, raw, len),
            Opcode::STRT => self.store(ops, bus, 4, raw, len),
            Opcode::LDRBT => self.load(ops, bus, 1, false, raw, len),
            Opcode::LDRHT => self.load(ops, bus, 2, false, raw, len),
            Opcode::LDRT => self.load(ops, bus, 4, false, raw, len),
            Opcode::LDRSBT => self.load(ops, bus, 1, true, raw, len),
            Opcode::LDRSHT => self.load(ops, bus, 2, true, raw, len),

            // Exclusive accesses: model as plain accesses; the monitor always succeeds.
            Opcode::LDREX | Opcode::LDREXB | Opcode::LDREXH => {
                let sz = match inst.opcode {
                    Opcode::LDREXB => 1,
                    Opcode::LDREXH => 2,
                    _ => 4,
                };
                let addr = self.mem_addr(&ops[1])?;
                let v = match sz {
                    1 => bus.read8(addr) as u32,
                    2 => bus.read16(addr) as u32,
                    _ => bus.read32(addr),
                };
                self.write_reg(reg(&ops[0])?, v);
                Ok(())
            }
            Opcode::STREX | Opcode::STREXB | Opcode::STREXH => {
                let sz = match inst.opcode {
                    Opcode::STREXB => 1,
                    Opcode::STREXH => 2,
                    _ => 4,
                };
                let addr = self.mem_addr(&ops[2])?;
                let v = self.read_reg(reg(&ops[1])?);
                match sz {
                    1 => bus.write8(addr, v as u8),
                    2 => bus.write16(addr, v as u16),
                    _ => bus.write32(addr, v),
                };
                self.write_reg(reg(&ops[0])?, 0); // success
                Ok(())
            }

            Opcode::CBZ => {
                if self.read_reg(reg(&ops[0])?) == 0 {
                    self.pc = cbz_target(raw, pc);
                }
                Ok(())
            }
            Opcode::CBNZ => {
                if self.read_reg(reg(&ops[0])?) != 0 {
                    self.pc = cbz_target(raw, pc);
                }
                Ok(())
            }

            Opcode::B => {
                if self.cond_holds(inst.condition) {
                    self.pc = thumb_branch_target(raw, pc, len);
                }
                Ok(())
            }
            Opcode::BL => {
                self.r[LR] = pc.wrapping_add(len) | 1;
                self.pc = thumb_branch_target(raw, pc, len);
                Ok(())
            }
            Opcode::BLX => {
                let t = self.read_reg(reg(&ops[0])?);
                self.r[LR] = pc.wrapping_add(len) | 1;
                self.pc = t & !1;
                Ok(())
            }
            Opcode::BX => {
                let t = self.read_reg(reg(&ops[0])?);
                if t & 0xff00_0000 == 0xff00_0000 {
                    self.exception_return(t, bus);
                } else {
                    self.pc = t & !1;
                }
                Ok(())
            }

            Opcode::TBB | Opcode::TBH => {
                // yaxpeax mis-decodes the H bit and operand shape, so read the
                // encoding from raw: hw2 bit4 = H (0=byte table, 1=halfword).
                let half = (raw >> 16) & 0x10 != 0;
                let rn = (raw & 0xF) as u8;
                let rm = ((raw >> 16) & 0xF) as u8;
                let base = if rn == 15 {
                    self.cur_insn.wrapping_add(4)
                } else {
                    self.r[rn as usize]
                };
                let idx = self.r[rm as usize];
                let offset = if half {
                    bus.read16(base.wrapping_add(idx << 1)) as u32
                } else {
                    bus.read8(base.wrapping_add(idx)) as u32
                };
                self.pc = self.cur_insn.wrapping_add(4).wrapping_add(offset << 1);
                Ok(())
            }

            Opcode::MRS => {
                self.last_sys = true; // re-decode SYSm from raw word (yaxpeax uses A-profile here)
                let rd = ((raw >> 24) & 0xF) as u8; // hw2 bits [11:8]
                let sysm = ((raw >> 16) & 0xFF) as u8;
                let v = self.read_special(sysm);
                self.write_reg(rd, v);
                Ok(())
            }
            Opcode::MSR => {
                self.last_sys = true;
                let rn = (raw & 0xF) as u8; // hw1 bits [3:0]
                let sysm = ((raw >> 16) & 0xFF) as u8;
                let v = self.read_reg(rn);
                self.write_special(sysm, v);
                Ok(())
            }
            Opcode::CPS(disable) => {
                self.last_sys = true; // cpsid i / cpsie i -> PRIMASK
                self.primask = disable;
                Ok(())
            }

            Opcode::SVC => {
                if self.trace_svc {
                    // Hubris ABI: syscall number in r11 at the SVC. Log it + a couple
                    // args (Send target/op in r4/r5) to trace RoT IPC (sprot->update_server).
                    let sys = self.r[11];
                    let name = match sys {
                        0 => "Send",
                        1 => "Recv",
                        2 => "Reply",
                        3 => "SetTimer",
                        4 => "BorrowRead",
                        5 => "BorrowWrite",
                        6 => "BorrowInfo",
                        7 => "IrqControl",
                        8 => "PANIC",
                        9 => "GetTimer",
                        10 => "RefreshTaskId",
                        11 => "Post",
                        12 => "ReplyFault",
                        13 => "IrqStatus",
                        _ => "?",
                    };
                    eprintln!(
                        "[rotsvc] {} r4={:#x} r5={:#x} r6={:#x}",
                        name, self.r[4], self.r[5], self.r[6]
                    );
                }
                self.exception_entry(11, bus);
                Ok(())
            }
            Opcode::UDF => Err(()), // permanently undefined: surface as a stop

            Opcode::BKPT => {
                if self.debug_en {
                    // Debug enabled: halt into debug state AT the breakpoint (do
                    // not advance past it), the way a real core enters debug on
                    // BKPT when DHCSR.C_DEBUGEN is set. endoscope ends with a BKPT
                    // to signal completion to the RoT, which then reads the result.
                    self.pc = pc;
                    self.halted = true;
                    self.bkpt_hit = true;
                    Ok(())
                } else {
                    // No debugger attached: an unexpected BKPT is a fault.
                    Err(())
                }
            }

            _ => Err(()),
        }
    }

    // ---- ALU with shifted operands -----------------------------------------

    fn alu(&mut self, ops: &[Operand; 4], op: Alu) -> Result<(), ()> {
        let rd = reg(&ops[0])?;
        let (a, b) = if matches!(ops[2], Operand::Nothing) {
            (self.read_reg(rd), self.opval(&ops[1])?)
        } else {
            // ADD/SUB rd, pc, #imm is ADR: PC reads word-aligned (Align(PC,4)).
            let rn = reg(&ops[1])?;
            let a = if rn == 15 {
                self.read_reg(15) & !3
            } else {
                self.read_reg(rn)
            };
            (a, self.opval(&ops[2])?)
        };
        let cin = self.c as u32;
        let res: u32 = match op {
            Alu::Add => a.wrapping_add(b),
            Alu::Sub => a.wrapping_sub(b),
            Alu::Rsb => b.wrapping_sub(a),
            Alu::Adc => a.wrapping_add(b).wrapping_add(cin),
            Alu::Sbc => a.wrapping_sub(b).wrapping_sub(1 - cin),
            Alu::And => a & b,
            Alu::Orr => a | b,
            Alu::Orn => a | !b,
            Alu::Eor => a ^ b,
            Alu::Bic => a & !b,
        };
        self.write_reg(rd, res);
        if self.cur_setflags {
            match op {
                Alu::Add => self.flags_add(a, b),
                Alu::Adc => {
                    let (r1, c1) = a.overflowing_add(b);
                    let (r2, c2) = r1.overflowing_add(cin);
                    self.set_nz(r2);
                    self.c = c1 || c2;
                    self.v = (((a ^ !b) & (a ^ r2)) >> 31) & 1 != 0;
                }
                Alu::Sub => self.flags_sub(a, b),
                Alu::Rsb => self.flags_sub(b, a),
                // SBC = a + ~b + cin; flags from that addition (mirrors Adc).
                Alu::Sbc => {
                    let (r1, c1) = a.overflowing_add(!b);
                    let (r2, c2) = r1.overflowing_add(cin);
                    self.set_nz(r2);
                    self.c = c1 || c2;
                    self.v = (((a ^ b) & (a ^ r2)) >> 31) & 1 != 0;
                }
                _ => self.set_nz(res),
            }
        }
        Ok(())
    }

    fn shift_op(&mut self, ops: &[Operand; 4], style: ShiftStyle) -> Result<(), ()> {
        let rd = reg(&ops[0])?;
        let (rm, amt) = if matches!(ops[2], Operand::Nothing) {
            (self.read_reg(rd), self.opval(&ops[1])?)
        } else {
            (self.read_reg(reg(&ops[1])?), self.opval(&ops[2])?)
        };
        let (res, carry) = shift_c(rm, style, amt & 0xff, self.c);
        self.write_reg(rd, res);
        if self.cur_setflags {
            self.set_nz(res);
            self.c = carry;
        }
        Ok(())
    }

    fn extend(
        &mut self,
        ops: &[Operand; 4],
        bits: u32,
        signed: bool,
        add: bool,
        raw: u32,
        len: u32,
    ) -> Result<(), ()> {
        // forms: <ext> rd, rm[, rot]   /   <extA> rd, rn, rm[, rot]
        let rd = reg(&ops[0])?;
        let (acc, rm_op) = if add {
            (self.read_reg(reg(&ops[1])?), &ops[2])
        } else {
            (0u32, &ops[1])
        };
        // yaxpeax mis-reports the rotation operand; the ROR amount is the 2-bit
        // field in the 32-bit encoding times 8 (16-bit forms never rotate).
        let rot = if len == 2 { 0 } else { ((raw >> 20) & 3) * 8 };
        let mut v = self.opval(rm_op)?;
        if rot != 0 {
            v = v.rotate_right(rot);
        }
        let masked = v & ((1u64 << bits) - 1) as u32;
        let ext = if signed {
            let m = 1u32 << (bits - 1);
            (masked ^ m).wrapping_sub(m)
        } else {
            masked
        };
        self.write_reg(rd, acc.wrapping_add(ext));
        Ok(())
    }

    // ---- memory ------------------------------------------------------------

    fn load(
        &mut self,
        ops: &[Operand; 4],
        bus: &mut Bus,
        size: u32,
        signed: bool,
        raw: u32,
        len: u32,
    ) -> Result<(), ()> {
        let rt = reg(&ops[0])?;
        // For 32-bit encodings, decode the address from raw — yaxpeax mis-decodes
        // several load/store addressing forms (e.g. T3 imm as a register offset).
        let addr = if len == 4 {
            self.mem_addr32(raw)
        } else {
            self.mem_addr(&ops[1])?
        };
        let rv = match size {
            1 => bus.read8(addr) as u32,
            2 => bus.read16(addr) as u32,
            _ => bus.read32(addr),
        };
        let val = if signed {
            match size {
                1 => rv as u8 as i8 as i32 as u32,
                2 => rv as u16 as i16 as i32 as u32,
                _ => rv,
            }
        } else {
            rv
        };
        if rt == 15 {
            if val & 0xff00_0000 == 0xff00_0000 {
                self.exception_return(val, bus);
                return Ok(());
            }
            self.pc = val & !1;
        } else {
            self.write_reg(rt, val);
        }
        Ok(())
    }

    fn store(
        &mut self,
        ops: &[Operand; 4],
        bus: &mut Bus,
        size: u32,
        raw: u32,
        len: u32,
    ) -> Result<(), ()> {
        let rt = reg(&ops[0])?;
        let addr = if len == 4 {
            self.mem_addr32(raw)
        } else {
            self.mem_addr(&ops[1])?
        };
        let v = self.read_reg(rt);
        match size {
            1 => bus.write8(addr, v as u8),
            2 => bus.write16(addr, v as u16),
            _ => bus.write32(addr, v),
        };
        Ok(())
    }

    /// Effective address for a 32-bit single load/store, decoded from raw bits:
    /// T3 (positive imm12), T4 (imm8 with P/U/W index+writeback), or T2 (register
    /// offset Rm shifted by imm2). Rn==15 is the PC-relative literal form.
    fn mem_addr32(&mut self, raw: u32) -> u32 {
        let hw1 = raw & 0xFFFF;
        let hw2 = (raw >> 16) & 0xFFFF;
        let rn = (hw1 & 0xF) as usize;
        if rn == 15 {
            let base = self.cur_insn.wrapping_add(4) & !3;
            let imm12 = hw2 & 0xFFF;
            return if (hw1 >> 7) & 1 == 1 {
                base.wrapping_add(imm12)
            } else {
                base.wrapping_sub(imm12)
            };
        }
        let base = self.r[rn];
        if (hw1 >> 7) & 1 == 1 {
            base.wrapping_add(hw2 & 0xFFF) // T3: positive imm12
        } else if (hw2 >> 11) & 1 == 1 {
            // T4: imm8, P (pre-index), U (add), W (writeback)
            let imm8 = hw2 & 0xFF;
            let (p, u, w) = ((hw2 >> 10) & 1, (hw2 >> 9) & 1, (hw2 >> 8) & 1);
            let off_addr = if u == 1 {
                base.wrapping_add(imm8)
            } else {
                base.wrapping_sub(imm8)
            };
            let addr = if p == 1 { off_addr } else { base };
            if w == 1 || p == 0 {
                self.r[rn] = off_addr;
            }
            addr
        } else {
            // T2: register offset, Rm << imm2
            let rm = (hw2 & 0xF) as usize;
            base.wrapping_add(self.r[rm] << ((hw2 >> 4) & 3))
        }
    }

    fn load_double(&mut self, ops: &[Operand; 4], bus: &mut Bus) -> Result<(), ()> {
        let (rt, rt2) = (reg(&ops[0])?, reg(&ops[1])?);
        let addr = self.mem_addr(&ops[2])?;
        let a = bus.read32(addr);
        let b = bus.read32(addr.wrapping_add(4));
        self.write_reg(rt, a);
        self.write_reg(rt2, b);
        Ok(())
    }
    fn store_double(&mut self, ops: &[Operand; 4], bus: &mut Bus) -> Result<(), ()> {
        let (rt, rt2) = (reg(&ops[0])?, reg(&ops[1])?);
        let addr = self.mem_addr(&ops[2])?;
        bus.write32(addr, self.read_reg(rt));
        bus.write32(addr.wrapping_add(4), self.read_reg(rt2));
        Ok(())
    }

    /// Resolve a memory operand to an effective address, applying any base
    /// writeback (pre/post-index) as a side effect.
    fn mem_addr(&mut self, op: &Operand) -> Result<u32, ()> {
        match op {
            Operand::RegDeref(rn) => Ok(self.r[rn.number() as usize]),
            Operand::RegDerefPreindexOffset(rn, off, add, wback) => {
                let n = rn.number();
                let base = if n == 15 {
                    self.cur_insn.wrapping_add(4) & !3
                } else {
                    self.r[n as usize]
                };
                let ea = if *add {
                    base.wrapping_add(*off as u32)
                } else {
                    base.wrapping_sub(*off as u32)
                };
                if *wback && n != 15 {
                    self.r[n as usize] = ea;
                }
                Ok(ea)
            }
            Operand::RegDerefPostindexOffset(rn, off, add, _wback) => {
                let n = rn.number();
                let base = self.r[n as usize];
                let ea = base; // access at base, then writeback
                self.r[n as usize] = if *add {
                    base.wrapping_add(*off as u32)
                } else {
                    base.wrapping_sub(*off as u32)
                };
                Ok(ea)
            }
            Operand::RegDerefPreindexReg(rn, rm, add, wback) => {
                let n = rn.number();
                let base = if n == 15 {
                    self.cur_insn.wrapping_add(4) & !3
                } else {
                    self.r[n as usize]
                };
                let idx = self.r[rm.number() as usize];
                let ea = if *add {
                    base.wrapping_add(idx)
                } else {
                    base.wrapping_sub(idx)
                };
                if *wback && n != 15 {
                    self.r[n as usize] = ea;
                }
                Ok(ea)
            }
            Operand::RegDerefPreindexRegShift(rn, rs, add, wback) => {
                let n = rn.number();
                let base = if n == 15 {
                    self.cur_insn.wrapping_add(4) & !3
                } else {
                    self.r[n as usize]
                };
                let idx = match rs.into_shift() {
                    RegShiftStyle::RegImm(sh) => do_shift(
                        self.r[sh.shiftee().number() as usize],
                        sh.stype(),
                        sh.imm() as u32,
                    ),
                    RegShiftStyle::RegReg(sh) => do_shift(
                        self.r[sh.shiftee().number() as usize],
                        sh.stype(),
                        self.r[sh.shifter().number() as usize] & 0xff,
                    ),
                };
                let ea = if *add {
                    base.wrapping_add(idx)
                } else {
                    base.wrapping_sub(idx)
                };
                if *wback && n != 15 {
                    self.r[n as usize] = ea;
                }
                Ok(ea)
            }
            _ => Err(()),
        }
    }

    /// PUSH = STMDB sp! ; POP = LDMIA sp!. Generic block transfer otherwise.
    fn stmdb_regs(&mut self, bus: &mut Bus, rn: u8, mask: u16, wback: bool) -> Result<(), ()> {
        self.block_transfer(bus, rn, wback, mask, false, true, true)
    }
    fn ldmia_regs(&mut self, bus: &mut Bus, rn: u8, mask: u16, wback: bool) -> Result<(), ()> {
        self.block_transfer(bus, rn, wback, mask, true, false, false)
    }

    #[allow(clippy::too_many_arguments)] // load/store-multiple needs all the addressing-mode flags
    fn block_transfer(
        &mut self,
        bus: &mut Bus,
        rn: u8,
        wback: bool,
        mask: u16,
        add: bool,
        pre: bool,
        store: bool,
    ) -> Result<(), ()> {
        let count = mask.count_ones();
        let base = self.r[rn as usize];
        let mut addr = match (add, pre) {
            (true, false) => base,                                          // IA
            (true, true) => base.wrapping_add(4),                           // IB
            (false, true) => base.wrapping_sub(4 * count),                  // DB
            (false, false) => base.wrapping_sub(4 * count).wrapping_add(4), // DA
        };
        // Defer loading PC: a popped EXC_RETURN triggers an exception return that
        // changes the active stack pointer, and the writeback must be applied to
        // the current SP bank before that switch, otherwise it clobbers the SP
        // the exception return just set. Affects `pop {..,pc}` exception exits.
        let mut pc_val: Option<u32> = None;
        for i in 0..16u8 {
            if mask & (1 << i) != 0 {
                if store {
                    bus.write32(addr, self.read_reg(i));
                } else {
                    let v = bus.read32(addr);
                    if i == 15 {
                        pc_val = Some(v);
                    } else {
                        self.r[i as usize] = v;
                    }
                }
                addr = addr.wrapping_add(4);
            }
        }
        if wback {
            self.r[rn as usize] = if add {
                base.wrapping_add(4 * count)
            } else {
                base.wrapping_sub(4 * count)
            };
        }
        if let Some(v) = pc_val {
            if v & 0xff00_0000 == 0xff00_0000 {
                self.exception_return(v, bus);
            } else {
                self.pc = v & !1;
            }
        }
        Ok(())
    }

    // ---- M-profile special registers (MRS/MSR by SYSm) ---------------------

    fn read_special(&self, sysm: u8) -> u32 {
        match sysm {
            0..=3 => self.build_xpsr(), // (I)(E)APSR / xPSR
            5 => self.ipsr,             // IPSR
            8 => self.msp,
            9 => self.psp,
            16 => self.primask as u32,
            17 | 18 => self.basepri,
            19 => self.faultmask as u32,
            20 => self.control,
            _ => 0,
        }
    }

    fn write_special(&mut self, sysm: u8, v: u32) {
        match sysm {
            0..=3 => self.set_xpsr_flags(v),
            8 => {
                self.msp = v;
                if !self.sp_is_psp {
                    self.r[SP] = v;
                }
            }
            9 => {
                self.psp = v;
                if self.sp_is_psp {
                    self.r[SP] = v;
                }
            }
            16 => self.primask = v & 1 != 0,
            17 | 18 => self.basepri = v & 0xff,
            19 => self.faultmask = v & 1 != 0,
            20 => self.set_control(v),
            _ => {}
        }
    }

    fn set_control(&mut self, v: u32) {
        let was_psp = self.sp_is_psp;
        self.control = v & 0x7;
        let now_psp = self.mode == Mode::Thread && (self.control & 2) != 0;
        if was_psp != now_psp {
            if was_psp {
                self.psp = self.r[SP];
            } else {
                self.msp = self.r[SP];
            }
            self.r[SP] = if now_psp { self.psp } else { self.msp };
            self.sp_is_psp = now_psp;
        }
    }

    // ---- exceptions --------------------------------------------------------

    fn build_xpsr(&self) -> u32 {
        // ITSTATE is split across the EPSR field of xPSR: ITSTATE[7:2] -> bits
        // [15:10], ITSTATE[1:0] -> bits [26:25]. Must be included so an async
        // interrupt taken mid-IT-block stacks the IT state and the interrupted
        // task resumes its conditional run correctly on return.
        let it_hi = ((self.itstate >> 2) & 0x3F) as u32; // ITSTATE[7:2]
        let it_lo = (self.itstate & 0x3) as u32; // ITSTATE[1:0]
        ((self.n as u32) << 31)
            | ((self.z as u32) << 30)
            | ((self.c as u32) << 29)
            | ((self.v as u32) << 28)
            | ((self.q as u32) << 27)
            | (it_lo << 25)
            | (1 << 24)
            | (it_hi << 10)
            | (self.ipsr & 0x1ff)
    }

    /// Decode ITSTATE back out of a stacked/restored xPSR (inverse of build_xpsr).
    fn itstate_from_xpsr(x: u32) -> u8 {
        ((((x >> 10) & 0x3F) << 2) | ((x >> 25) & 0x3)) as u8
    }
    fn set_xpsr_flags(&mut self, x: u32) {
        self.n = x & (1 << 31) != 0;
        self.z = x & (1 << 30) != 0;
        self.c = x & (1 << 29) != 0;
        self.v = x & (1 << 28) != 0;
        self.q = x & (1 << 27) != 0;
    }

    /// Drive the SysTick timer one instruction. When it underflows and the
    /// timer interrupt is enabled, take the SysTick exception (vector 15), but
    /// only from thread mode with interrupts unmasked (the kernel runs SysTick
    /// at the lowest priority, so it never preempts a handler).
    pub fn maybe_tick(&mut self, bus: &mut Bus) {
        // SYST_CSR (enable/tickint) and SYST_RVR (reload) are configured once at
        // boot and ~never change, yet reading them through the full bus dispatch
        // (RAM-region scan + device scan to reach the SCS) every instruction was a
        // top per-instruction cost. Refresh the cached copies only periodically;
        // a sub-256-instruction lag in noticing a CSR/RVR change is immaterial
        // (the SysTick period is millions of instructions).
        if self.cycles & 0xFF == 0 {
            self.syst_csr = bus.read32(0xE000_E010);
            self.syst_rvr = bus.read32(0xE000_E014).max(1);
        }
        if self.syst_csr & 1 == 0 {
            return;
        } // counter disabled
        if self.systick == 0 {
            self.systick = self.syst_rvr;
        }
        self.systick -= 1;
        if self.systick == 0 {
            self.systick = self.syst_rvr;
            if self.syst_csr & 2 != 0 && self.mode == Mode::Thread && !self.primask {
                self.exception_entry(15, bus); // SysTick exception
            }
        }
    }

    /// Deliver a pending NVIC interrupt, if any is enabled and unmasked. Like
    /// `maybe_tick`, called once per instruction. Hubris runs all IRQs below the
    /// kernel's SVCall/PendSV/SysTick (priority 0), so only thread mode is
    /// preempted (never a running handler), honoring PRIMASK/FAULTMASK/BASEPRI.
    pub fn maybe_interrupt(&mut self, bus: &mut Bus) {
        bus.collect_irqs();
        if self.mode != Mode::Thread || self.primask || self.faultmask {
            return;
        }
        if let Some(irq) = bus.next_irq() {
            // BASEPRI masks interrupts whose priority is numerically >= basepri
            // (0 = disabled). Priorities live in the high bits of the byte.
            if self.basepri == 0 || (bus.irq_prio(irq) as u32) < self.basepri {
                if (irq == 61 || matches!(irq, 31 | 33 | 72 | 92 | 95)) && crate::dbg::eth() {
                    eprintln!("[irq] delivering IRQ {} at cyc {}", irq, self.cycles);
                }
                bus.clear_pending(irq);
                self.exception_entry(16 + irq as u32, bus); // exception number = 16 + IRQ
                return;
            }
        }
        // PendSV: the kernel's deferred context switch, lowest priority. Fires on
        // return to thread mode after the SysTick/interrupt handler that pended it.
        if bus.take_pendsv() {
            self.exception_entry(14, bus);
        }
    }

    fn exception_entry(&mut self, vecnum: u32, bus: &mut Bus) {
        // Catch task panics (SVC with Sysnum::Panic=8 in r11; msg ptr/len in r4/r5).
        if vecnum == 11 && self.r[11] == 8 && crate::dbg::panic() {
            let (ptr, len) = (self.r[4], self.r[5].min(120));
            let mut msg = String::new();
            for i in 0..len {
                msg.push(bus.read8(ptr.wrapping_add(i)) as char);
            }
            eprint!(
                "[task-panic] cyc={} psp={:#x} msg={:?} bt:",
                self.cycles, self.r[SP], msg
            );
            // Reliable backtrace by walking the r7 frame-pointer chain ([r7]=prev
            // frame, [r7+4]=return address).
            let mut fp = self.r[7];
            for _ in 0..16 {
                if !(0x2000_0000..0x3900_0000).contains(&fp) || fp & 3 != 0 {
                    break;
                }
                let ret = bus.read32(fp.wrapping_add(4));
                if (0x0800_0000..0x0806_0000).contains(&ret) {
                    eprint!(" {:#x}", ret & !1);
                }
                let next = bus.read32(fp);
                if next <= fp {
                    break;
                }
                fp = next;
            }
            eprintln!();
        }
        // Dump syscalls made from net's code range (flash 0x08008000-0x08017fff)
        // to expose the bogus buffer pointer it hands the kernel. r11=sysnum;
        // for Recv the args are r4=buf r5=len r6=notif r7=sender; for Send/Reply
        // they're in r4-r7 too. Gated by SP_EMU_SVCDBG.
        if vecnum == 11 && crate::dbg::svc() && (0x0800_8000..0x0801_8000).contains(&self.pc) {
            eprintln!(
                "[net-svc] cyc={} sysnum={} r0={:#x} r1={:#x} r2={:#x} r3={:#x} \
                r4={:#x} r5={:#x} r6={:#x} r7={:#x} psp={:#x} pc={:#x}",
                self.cycles,
                self.r[11],
                self.r[0],
                self.r[1],
                self.r[2],
                self.r[3],
                self.r[4],
                self.r[5],
                self.r[6],
                self.r[7],
                self.r[SP],
                self.pc
            );
            // BorrowRead (sysnum 4): r7 = dest ptr. Flag a slice base outside net's
            // RAM (0x24030000-0x2403ffff) / DMA (0x30000000-0x30047fff) as corrupt.
            let dest = self.r[7];
            if self.r[11] == 4
                && !(0x2403_0000..0x2404_0000).contains(&dest)
                && !(0x3000_0000..0x3004_8000).contains(&dest)
            {
                eprintln!(
                    "[net-BADdest] dest={:#x} dest_len={:#x} lr={:#x}",
                    dest, self.r[8], self.r[LR]
                );
            }
        }
        let return_addr = self.pc; // already advanced past the SVC
                                   // If FP context is active (CONTROL.FPCA), the hardware stacks an extended
                                   // frame (basic + S0-S15 + FPSCR + reserved). Entry and return must agree,
                                   // or the task's stack pointer drifts across syscalls.
        let fpca = self.control & 4 != 0;
        let words = if fpca { 26 } else { 8 };
        let frame_base = self.r[SP].wrapping_sub(4 * words);
        // Trace exceptions taken from a task's code (thread mode) to locate a
        // return that restores a corrupt PC. Gated by $SP_EMU_EXCDBG.
        if self.mode == Mode::Thread && fpca && crate::dbg::exc() {
            eprintln!("[exc-ent] vec={} from_pc={:#010x} fpca={} it={:#04x} sp={:#010x} frame={:#010x} cyc={}",
                vecnum, return_addr, fpca, self.itstate, self.r[SP], frame_base, self.cycles);
        }
        let basic = [
            self.r[0],
            self.r[1],
            self.r[2],
            self.r[3],
            self.r[12],
            self.r[LR],
            return_addr,
            self.build_xpsr(),
        ];
        let mut a = frame_base;
        for w in basic {
            bus.write32(a, w);
            a = a.wrapping_add(4);
        }
        if fpca {
            for i in 0..16 {
                bus.write32(a, self.s[i]);
                a = a.wrapping_add(4);
            }
            bus.write32(a, self.fpscr);
            a = a.wrapping_add(4);
            bus.write32(a, 0); // reserved
        }
        self.r[SP] = frame_base;

        // EXC_RETURN: bit4 clear means an extended (FP) frame.
        let exc_base: u32 = match (self.mode, self.sp_is_psp) {
            (Mode::Handler, _) => 0xFFFF_FFF1,
            (Mode::Thread, false) => 0xFFFF_FFF9,
            (Mode::Thread, true) => 0xFFFF_FFFD,
        };
        let exc = if fpca { exc_base & !0x10 } else { exc_base };
        // Switch to handler mode (always MSP).
        if self.sp_is_psp {
            self.psp = self.r[SP];
            self.r[SP] = self.msp;
            self.sp_is_psp = false;
        } else {
            self.msp = self.r[SP];
        }
        self.mode = Mode::Handler;
        self.ipsr = vecnum;
        self.r[LR] = exc;
        let vtor = bus.read32(0xE000_ED08) & !0x7f;
        let vtor = if vtor == 0 { 0x0800_0000 } else { vtor }; // boot-aliased table
        self.pc = bus.read32(vtor.wrapping_add(vecnum * 4)) & !1;
        self.itstate = 0;
    }

    fn exception_return(&mut self, exc: u32, bus: &mut Bus) {
        let return_psp = exc & 0x4 != 0;
        let to_thread = exc & 0x8 != 0;
        let extended = exc & 0x10 == 0;
        let base = if return_psp { self.psp } else { self.msp };
        let r0 = bus.read32(base);
        let r1 = bus.read32(base + 4);
        let r2 = bus.read32(base + 8);
        let r3 = bus.read32(base + 12);
        let r12 = bus.read32(base + 16);
        let lr = bus.read32(base + 20);
        let pc = bus.read32(base + 24);
        let xpsr = bus.read32(base + 28);
        // Crash detector: a stacked return PC must point into flash (code). If it
        // doesn't, either the exception frame was clobbered or the restored SP
        // is wrong; dump the frame + context to pinpoint it.
        if !(0x0800_0000..0x0820_0000).contains(&pc) && self.bad_ret_dumps < 6 {
            self.bad_ret_dumps += 1;
            eprintln!("[exc-ret BAD] return PC={:#010x} (not flash!) exc={:#x} from_psp={} base={:#010x} lr={:#010x} xpsr={:#010x} cyc={}",
                pc, exc, return_psp, base, lr, xpsr, self.cycles);
            eprintln!(
                "   frame: r0={:#x} r1={:#x} r2={:#x} r3={:#x} r12={:#x}",
                r0, r1, r2, r3, r12
            );
            eprintln!("   psp={:#010x} msp={:#010x}", self.psp, self.msp);
            for o in (0..0x48u32).step_by(4) {
                let a = base.wrapping_sub(8).wrapping_add(o);
                eprintln!(
                    "   [{:#010x}] = {:#010x}{}",
                    a,
                    bus.read32(a),
                    if a == base + 24 {
                        "  <-- stacked PC"
                    } else {
                        ""
                    }
                );
            }
        }
        let mut sp = base.wrapping_add(0x20);
        if extended {
            for i in 0..16 {
                self.s[i] = bus.read32(sp);
                sp = sp.wrapping_add(4);
            }
            self.fpscr = bus.read32(sp);
            sp = sp.wrapping_add(8); // fpscr + reserved
        }
        self.r[0] = r0;
        self.r[1] = r1;
        self.r[2] = r2;
        self.r[3] = r3;
        self.r[12] = r12;
        self.r[LR] = lr;
        self.pc = pc & !1;
        self.set_xpsr_flags(xpsr);
        // Restore the interrupted IT-block state (see build_xpsr) so a task
        // preempted mid-IT-block resumes its conditional execution correctly.
        self.itstate = Self::itstate_from_xpsr(xpsr);
        self.ipsr = xpsr & 0x1ff;
        // FPCA reflects whether the restored context had FP state active.
        if extended {
            self.control |= 4;
        } else {
            self.control &= !4;
        }
        self.mode = if to_thread {
            Mode::Thread
        } else {
            Mode::Handler
        };
        if to_thread {
            if return_psp {
                self.control |= 2;
            } else {
                self.control &= !2;
            }
        }
        if return_psp {
            self.psp = sp;
        } else {
            self.msp = sp;
        }
        self.sp_is_psp = self.mode == Mode::Thread && (self.control & 2) != 0;
        self.r[SP] = if self.sp_is_psp { self.psp } else { self.msp };
        let in_memmove = (0x0806_6c00..0x0806_6d00).contains(&self.pc);
        if to_thread && (extended || in_memmove) && crate::dbg::exc() {
            eprintln!(
                "[exc-ret]{} exc={:#x} -> pc={:#010x} extended={} base={:#010x} it={:#04x} cyc={}",
                if in_memmove { " *MEMMOVE*" } else { "" },
                exc,
                self.pc,
                extended,
                base,
                self.itstate,
                self.cycles
            );
        }
        if self.mode == Mode::Thread && (self.control & 1) != 0 && !self.entered_task {
            self.entered_task = true;
            eprintln!("\n*** ENTERED FIRST TASK: thread mode, unprivileged, PSP={:#010x}, PC={:#010x} ***\n", self.psp, self.pc);
        }
    }

    /// Thumb VFP load/store (yaxpeax can't decode these). Returns true if handled.
    /// Covers VLDR/VSTR (single reg) and VLDM/VSTM/VPUSH/VPOP (register list) for
    /// both single (S) and double (D) precision, enough for the kernel's
    /// FP context save/restore around syscalls.
    fn try_vfp(&mut self, raw: u32, pc: u32, bus: &mut Bus) -> bool {
        let hw1 = raw & 0xFFFF;
        let hw2 = (raw >> 16) & 0xFFFF;
        // Extension register load/store space: hw1[15:9] == 1110110.
        if hw1 >> 9 != 0b1110110 {
            // FP data-processing / VMOV / VMRS / VMSR: 1110 1110, coproc 101x.
            if (hw1 & 0xFF00) == 0xEE00 && (hw2 & 0x0E00) == 0x0A00 {
                self.last_vfp = true;
                self.control |= 4;
                self.exec_vfp_dp(hw1, hw2);
                self.pc = pc.wrapping_add(4);
                return true;
            }
            return false;
        }
        self.last_vfp = true;
        self.control |= 4; // executing FP marks the context FP-active (CONTROL.FPCA)
        let p = (hw1 >> 8) & 1;
        let u = (hw1 >> 7) & 1;
        let d = (hw1 >> 6) & 1;
        let w = (hw1 >> 5) & 1;
        let l = (hw1 >> 4) & 1;
        let rn = (hw1 & 0xF) as usize;
        let vd = (hw2 >> 12) & 0xF;
        let single = (hw2 >> 8) & 1 == 0;
        let imm8 = hw2 & 0xFF;

        let load_store_single = |s: &mut [u32; 32], bus: &mut Bus, reg: usize, addr: u32| {
            if l == 1 {
                s[reg & 31] = bus.read32(addr);
            } else {
                bus.write32(addr, s[reg & 31]);
            }
        };
        let load_store_double = |s: &mut [u32; 32], bus: &mut Bus, dreg: usize, addr: u32| {
            let r = (dreg & 15) * 2;
            if l == 1 {
                s[r] = bus.read32(addr);
                s[r + 1] = bus.read32(addr.wrapping_add(4));
            } else {
                bus.write32(addr, s[r]);
                bus.write32(addr.wrapping_add(4), s[r + 1]);
            }
        };

        if p == 1 && w == 0 {
            // VLDR / VSTR: single register, byte offset = imm8 << 2.
            let base = if rn == 15 {
                self.cur_insn.wrapping_add(4) & !3
            } else {
                self.r[rn]
            };
            let off = imm8 << 2;
            let addr = if u == 1 {
                base.wrapping_add(off)
            } else {
                base.wrapping_sub(off)
            };
            if single {
                load_store_single(&mut self.s, bus, ((vd << 1) | d) as usize, addr);
            } else {
                load_store_double(&mut self.s, bus, ((d << 4) | vd) as usize, addr);
            }
        } else {
            // VLDM / VSTM / VPUSH / VPOP: imm8 = number of transferred words.
            let count = if single { imm8 } else { imm8 / 2 };
            let first = if single { (vd << 1) | d } else { (d << 4) | vd };
            let stride = if single { 4 } else { 8 };
            let total = stride * count;
            let base = self.r[rn];
            let mut addr = if u == 1 {
                base
            } else {
                base.wrapping_sub(total)
            };
            for i in 0..count {
                if single {
                    load_store_single(&mut self.s, bus, (first + i) as usize, addr);
                } else {
                    load_store_double(&mut self.s, bus, (first + i) as usize, addr);
                }
                addr = addr.wrapping_add(stride);
            }
            if w == 1 {
                self.r[rn] = if u == 1 {
                    base.wrapping_add(total)
                } else {
                    base.wrapping_sub(total)
                };
            }
        }
        self.pc = pc.wrapping_add(4);
        true
    }

    /// Single-precision VFP data-processing + VMOV/VMRS/VMSR. Decoded from raw
    /// (yaxpeax can't decode VFP). Single precision (F32) is what this firmware
    /// uses; double (F64) arithmetic is handled for the common ops too.
    fn exec_vfp_dp(&mut self, hw1: u32, hw2: u32) {
        let coproc_fp = (hw2 & 0x0E00) == 0x0A00;
        // VMRS Rt, FPSCR  (hw1=0xEEF1)
        if hw1 == 0xEEF1 && coproc_fp && (hw2 & 0x00F0) == 0x0010 {
            let rt = ((hw2 >> 12) & 0xF) as usize;
            if rt == 15 {
                // VMRS APSR_nzcv, FPSCR — copy FP compare flags into APSR.
                self.n = self.fpscr & (1 << 31) != 0;
                self.z = self.fpscr & (1 << 30) != 0;
                self.c = self.fpscr & (1 << 29) != 0;
                self.v = self.fpscr & (1 << 28) != 0;
            } else {
                self.r[rt] = self.fpscr;
            }
            return;
        }
        // VMSR FPSCR, Rt  (hw1=0xEEE1)
        if hw1 == 0xEEE1 && coproc_fp {
            self.fpscr = self.r[((hw2 >> 12) & 0xF) as usize];
            return;
        }
        // VMOV core <-> single:  1110 1110 000o Vn | Rt 1010 N001 0000
        if (hw1 & 0xFFE0) == 0xEE00 && (hw2 & 0x0F7F) == 0x0A10 {
            let to_core = (hw1 >> 4) & 1 == 1;
            let sn = (((hw1 & 0xF) << 1) | ((hw2 >> 7) & 1)) as usize;
            let rt = ((hw2 >> 12) & 0xF) as usize;
            if to_core {
                self.r[rt] = self.s[sn];
            } else {
                self.s[sn] = self.r[rt];
            }
            return;
        }

        let d = (hw1 >> 6) & 1;
        let opc1 = (hw1 >> 4) & 3;
        let vn = hw1 & 0xF;
        let b23 = (hw1 >> 7) & 1;
        let vd = (hw2 >> 12) & 0xF;
        let sz = (hw2 >> 8) & 1;
        let n = (hw2 >> 7) & 1;
        let op = (hw2 >> 6) & 1;
        let m = (hw2 >> 5) & 1;
        let vm = hw2 & 0xF;

        if sz == 1 {
            // Double precision: register number = (bit<<4)|Vx; value spans 2 words.
            let dr = |x: u32, b: u32| (((b << 4) | x) as usize & 15) * 2;
            let (dd, dn, dm) = (dr(vd, d), dr(vn, n), dr(vm, m));
            let getd =
                |s: &[u32; 32], i: usize| f64::from_bits(s[i] as u64 | ((s[i + 1] as u64) << 32));
            let setd = |s: &mut [u32; 32], i: usize, v: f64| {
                let b = v.to_bits();
                s[i] = b as u32;
                s[i + 1] = (b >> 32) as u32;
            };
            let (a, b) = (getd(&self.s, dn), getd(&self.s, dm));
            if b23 == 0 {
                let r = match opc1 {
                    0b00 => {
                        let acc = getd(&self.s, dd);
                        if op == 0 {
                            acc + a * b
                        } else {
                            acc - a * b
                        }
                    }
                    0b10 => {
                        if op == 0 {
                            a * b
                        } else {
                            -(a * b)
                        }
                    }
                    0b11 => {
                        if op == 0 {
                            a + b
                        } else {
                            a - b
                        }
                    }
                    _ => a,
                };
                setd(&mut self.s, dd, r);
            } else if opc1 == 0b00 {
                setd(&mut self.s, dd, a / b);
            } else if opc1 == 0b11 && vn == 0 {
                let v = if op == 0 {
                    getd(&self.s, dm)
                } else {
                    getd(&self.s, dm).abs()
                };
                setd(&mut self.s, dd, v);
            }
            return;
        }

        // Single precision.
        let sd = (((vd << 1) | d) as usize) & 31;
        let sn = (((vn << 1) | n) as usize) & 31;
        let sm = (((vm << 1) | m) as usize) & 31;
        let f = |s: &[u32; 32], i: usize| f32::from_bits(s[i]);
        let (a, b) = (f(&self.s, sn), f(&self.s, sm));

        if b23 == 0 {
            let r = match opc1 {
                0b00 => {
                    let acc = f(&self.s, sd);
                    if op == 0 {
                        acc + a * b
                    } else {
                        acc - a * b
                    }
                } // VMLA/VMLS
                0b01 => {
                    let acc = f(&self.s, sd);
                    if op == 0 {
                        a * b - acc
                    } else {
                        -(a * b) - acc
                    }
                } // VNMLS/VNMLA
                0b10 => {
                    if op == 0 {
                        a * b
                    } else {
                        -(a * b)
                    }
                } // VMUL/VNMUL
                0b11 => {
                    if op == 0 {
                        a + b
                    } else {
                        a - b
                    }
                } // VADD/VSUB
                _ => a,
            };
            self.s[sd] = r.to_bits();
            return;
        }
        // b23 == 1
        if opc1 == 0b00 {
            self.s[sd] = (a / b).to_bits(); // VDIV
            return;
        }
        if opc1 == 0b11 {
            // VMOV (immediate): hw2[7:4] == 0000, imm = imm4H:imm4L via VFPExpandImm.
            if (hw2 & 0xF0) == 0 {
                let imm8 = ((hw1 & 0xF) << 4) | (hw2 & 0xF);
                self.s[sd] = vfp_expand_imm32(imm8);
                return;
            }
            // Extension ops, selected by opc2 (= vn) and op/sz bits.
            match vn {
                0b0000 => {
                    self.s[sd] = if op == 0 {
                        self.s[sm]
                    } else {
                        f(&self.s, sm).abs().to_bits()
                    };
                } // VMOV/VABS
                0b0001 => {
                    self.s[sd] = if op == 0 {
                        (-f(&self.s, sm)).to_bits()
                    } else {
                        f(&self.s, sm).sqrt().to_bits()
                    };
                } // VNEG/VSQRT
                0b0100 | 0b0101 => {
                    // VCMP / VCMPE
                    let rhs = if vn & 1 == 1 { 0.0 } else { f(&self.s, sm) };
                    self.fp_compare(f(&self.s, sd), rhs);
                }
                0b1000 => {
                    // VCVT.F32.<S32|U32>  (int -> float)
                    let i = self.s[sm];
                    self.s[sd] = if op == 1 {
                        (i as i32 as f32).to_bits()
                    } else {
                        (i as f32).to_bits()
                    };
                }
                0b1100 | 0b1101 => {
                    // VCVT.<S32|U32>.F32  (float -> int, round toward zero)
                    let v = f(&self.s, sm);
                    self.s[sd] = if vn & 1 == 1 {
                        (v as i32) as u32
                    } else {
                        v as u32
                    };
                }
                _ => {}
            }
        }
    }

    /// Set FPSCR N/Z/C/V from a single-precision compare (used by VCMP+VMRS).
    fn fp_compare(&mut self, a: f32, b: f32) {
        let (mut n, mut z, mut c, mut v) = (false, false, false, false);
        if a.is_nan() || b.is_nan() {
            c = true;
            v = true;
        } else if a == b {
            z = true;
            c = true;
        } else if a < b {
            n = true;
        } else {
            c = true;
        }
        self.fpscr = (self.fpscr & 0x0FFF_FFFF)
            | ((n as u32) << 31)
            | ((z as u32) << 30)
            | ((c as u32) << 29)
            | ((v as u32) << 28);
    }

    // ---- IT + condition + flags --------------------------------------------

    fn it_advance(&mut self) {
        if self.itstate & 0b111 == 0 {
            self.itstate = 0;
        } else {
            self.itstate = (self.itstate & 0b1110_0000) | ((self.itstate << 1) & 0b0001_1111);
        }
    }

    /// Resolve a register/immediate/shifted-register operand to its value.
    fn opval(&self, op: &Operand) -> Result<u32, ()> {
        match op {
            Operand::Reg(r) => Ok(self.read_reg(r.number())),
            Operand::Imm32(i) => Ok(*i),
            Operand::Imm12(i) => Ok(*i as u32),
            Operand::RegShift(rs) => Ok(match rs.into_shift() {
                RegShiftStyle::RegImm(s) => do_shift(
                    self.read_reg(s.shiftee().number()),
                    s.stype(),
                    s.imm() as u32,
                ),
                RegShiftStyle::RegReg(s) => do_shift(
                    self.read_reg(s.shiftee().number()),
                    s.stype(),
                    self.read_reg(s.shifter().number()) & 0xff,
                ),
            }),
            _ => Err(()),
        }
    }

    fn cond_holds(&self, c: ConditionCode) -> bool {
        match c {
            ConditionCode::AL => true,
            ConditionCode::EQ => self.z,
            ConditionCode::NE => !self.z,
            ConditionCode::HS => self.c,
            ConditionCode::LO => !self.c,
            ConditionCode::MI => self.n,
            ConditionCode::PL => !self.n,
            ConditionCode::VS => self.v,
            ConditionCode::VC => !self.v,
            ConditionCode::HI => self.c && !self.z,
            ConditionCode::LS => !self.c || self.z,
            ConditionCode::GE => self.n == self.v,
            ConditionCode::LT => self.n != self.v,
            ConditionCode::GT => !self.z && (self.n == self.v),
            ConditionCode::LE => self.z || (self.n != self.v),
        }
    }

    #[inline]
    fn set_nz(&mut self, v: u32) {
        self.n = (v as i32) < 0;
        self.z = v == 0;
    }
    fn flags_add(&mut self, a: u32, b: u32) {
        let (res, carry) = a.overflowing_add(b);
        self.set_nz(res);
        self.c = carry;
        self.v = (((a ^ !b) & (a ^ res)) >> 31) & 1 != 0;
    }
    fn flags_sub(&mut self, a: u32, b: u32) {
        let (res, borrow) = a.overflowing_sub(b);
        self.set_nz(res);
        self.c = !borrow;
        self.v = (((a ^ b) & (a ^ res)) >> 31) & 1 != 0;
    }
}

#[derive(Clone, Copy)]
enum Alu {
    Add,
    Adc,
    Sub,
    Sbc,
    Rsb,
    And,
    Orr,
    Orn,
    Eor,
    Bic,
}

fn do_shift(v: u32, style: ShiftStyle, amt: u32) -> u32 {
    shift_c(v, style, amt, false).0
}

/// ARM Shift_C: shift with carry-out (the bit last shifted out). `amt == 0`
/// leaves the value and carry unchanged.
fn shift_c(v: u32, style: ShiftStyle, amt: u32, cin: bool) -> (u32, bool) {
    if amt == 0 {
        return (v, cin);
    }
    match style {
        ShiftStyle::LSL => {
            if amt < 32 {
                (v << amt, (v >> (32 - amt)) & 1 != 0)
            } else if amt == 32 {
                (0, v & 1 != 0)
            } else {
                (0, false)
            }
        }
        ShiftStyle::LSR => {
            if amt < 32 {
                (v >> amt, (v >> (amt - 1)) & 1 != 0)
            } else if amt == 32 {
                (0, (v >> 31) & 1 != 0)
            } else {
                (0, false)
            }
        }
        ShiftStyle::ASR => {
            if amt < 32 {
                (((v as i32) >> amt) as u32, (v >> (amt - 1)) & 1 != 0)
            } else {
                (((v as i32) >> 31) as u32, (v >> 31) & 1 != 0)
            }
        }
        ShiftStyle::ROR => {
            let a = amt & 31;
            if a == 0 {
                (v, (v >> 31) & 1 != 0)
            } else {
                let r = v.rotate_right(a);
                (r, (r >> 31) & 1 != 0)
            }
        }
    }
}

/// Decode a Thumb B/BL/BLX branch target straight from the instruction word.
/// yaxpeax's BranchThumbOffset is inconsistent across encodings and is unused
/// here. All targets are PC-relative with PC = instruction address + 4.
fn thumb_branch_target(raw: u32, pc: u32, len: u32) -> u32 {
    let hw1 = raw & 0xFFFF;
    if len == 2 {
        if hw1 & 0xF000 == 0xD000 {
            // T1 conditional B: signed imm8 (in halfwords)
            let imm8 = ((hw1 & 0xFF) as u8 as i8 as i32) << 1;
            return pc.wrapping_add(4).wrapping_add(imm8 as u32);
        }
        // T2 unconditional B: signed imm11 (in halfwords)
        let imm11 = (((hw1 & 0x7FF) << 21) as i32 >> 21) << 1;
        pc.wrapping_add(4).wrapping_add(imm11 as u32)
    } else {
        // 32-bit T3 (cond B.W) / T4 (B.W) / BL share the S:J1:J2 layout.
        let hw2 = (raw >> 16) & 0xFFFF;
        let s = (hw1 >> 10) & 1;
        let j1 = (hw2 >> 13) & 1;
        let j2 = (hw2 >> 11) & 1;
        let imm11 = hw2 & 0x7FF;
        if (hw2 >> 12) & 1 == 1 {
            // T4 / BL: 25-bit signed offset
            let i1 = 1 - (j1 ^ s);
            let i2 = 1 - (j2 ^ s);
            let imm10 = hw1 & 0x3FF;
            let off = (s << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1);
            let off = ((off << 7) as i32 >> 7) as u32;
            pc.wrapping_add(4).wrapping_add(off)
        } else {
            // T3 conditional B.W: 21-bit signed offset
            let imm6 = hw1 & 0x3F;
            let off = (s << 20) | (j2 << 19) | (j1 << 18) | (imm6 << 12) | (imm11 << 1);
            let off = ((off << 11) as i32 >> 11) as u32;
            pc.wrapping_add(4).wrapping_add(off)
        }
    }
}

/// Decode a Thumb-2 MOVW/MOVT 16-bit immediate (imm4:i:imm3:imm8) from the raw
/// instruction word, yaxpeax places imm4 incorrectly for these encodings.
fn movw_movt_imm16(raw: u32) -> u32 {
    let hw1 = raw & 0xFFFF;
    let hw2 = (raw >> 16) & 0xFFFF;
    let imm4 = hw1 & 0xF;
    let i = (hw1 >> 10) & 1;
    let imm3 = (hw2 >> 12) & 0x7;
    let imm8 = hw2 & 0xFF;
    (imm4 << 12) | (i << 11) | (imm3 << 8) | imm8
}

/// CBZ/CBNZ: always a forward, zero-extended branch.
fn cbz_target(raw: u32, pc: u32) -> u32 {
    let hw1 = raw & 0xFFFF;
    let imm5 = (hw1 >> 3) & 0x1F;
    let i = (hw1 >> 9) & 1;
    pc.wrapping_add(4).wrapping_add(((i << 5) | imm5) << 1)
}

/// LDM/STM addressing mode from the raw word: 16-bit T1 is always increment-
/// after (IA); 32-bit T2 uses the U (add) and P (before) bits.
fn ldm_stm_mode(raw: u32, len: u32) -> (bool, bool) {
    if len == 2 {
        (true, false)
    } else {
        ((raw >> 7) & 1 != 0, (raw >> 8) & 1 != 0)
    }
}

/// VFPExpandImm for single precision (ARM ARM): turn an 8-bit VMOV immediate
/// into the f32 bit pattern.
fn vfp_expand_imm32(imm8: u32) -> u32 {
    let sign = (imm8 >> 7) & 1;
    let b6 = (imm8 >> 6) & 1;
    let exp = ((1 - b6) << 7) | ((if b6 == 1 { 0x1F } else { 0 }) << 2) | ((imm8 >> 4) & 3);
    let frac = (imm8 & 0xF) << 19;
    (sign << 31) | (exp << 23) | frac
}

fn cond_from_bits(b: u8) -> ConditionCode {
    use ConditionCode::*;
    match b & 0xF {
        0 => EQ,
        1 => NE,
        2 => HS,
        3 => LO,
        4 => MI,
        5 => PL,
        6 => VS,
        7 => VC,
        8 => HI,
        9 => LS,
        10 => GE,
        11 => LT,
        12 => GT,
        13 => LE,
        _ => AL,
    }
}

/// The 32-bit T2 register-controlled shift (`LSL/LSR/ASR/ROR.w Rd, Rn, Rm`):
/// hw1 = 1111_1010_0_tt_S_Rn, hw2 = 1111_Rd_0000_Rm. yaxpeax mis-reports `tt`,
/// so recover the true shift style from the raw encoding. Returns None when the
/// instruction isn't this form (immediate / 16-bit shifts decode fine).
fn t2_reg_shift_style(raw: u32, len: u32) -> Option<ShiftStyle> {
    if len != 4 {
        return None;
    }
    let (hw1, hw2) = ((raw & 0xFFFF) as u16, (raw >> 16) as u16);
    // T2 register-form shift (`LSL/LSR/ASR/ROR.w Rd, Rn, Rm`): type in hw1[6:5].
    if (hw1 >> 7) == 0x1F4 && (hw2 & 0xF0F0) == 0xF000 {
        return Some(match (hw1 >> 5) & 0x3 {
            0 => ShiftStyle::LSL,
            1 => ShiftStyle::LSR,
            2 => ShiftStyle::ASR,
            _ => ShiftStyle::ROR,
        });
    }
    // T3 MOV-immediate-shift (`MOV.w Rd, Rm, <type> #imm`, Rn=1111, S in bit4):
    // yaxpeax 0.4 also mis-decodes the type here; it lives in hw2[5:4]. Without
    // this, e.g. `mov.w rd, rm, ror #n` runs as ASR and silently corrupts values
    // (breaks the RoT's PlatformId validation, among others).
    if (hw1 & 0xFFEF) == 0xEA4F {
        return Some(match (hw2 >> 4) & 0x3 {
            0 => ShiftStyle::LSL,
            1 => ShiftStyle::LSR,
            2 => ShiftStyle::ASR,
            _ => ShiftStyle::ROR,
        });
    }
    None
}

/// Decode the (lsb, msb) bitfield bounds of a 32-bit T1 BFI/BFC from raw.
/// hw2[14:12]=imm3, hw2[7:6]=imm2 -> lsb=imm3:imm2; hw2[4:0]=msb.
fn bfx_lsb_msb(raw: u32) -> (u32, u32) {
    let hw2 = raw >> 16;
    let lsb = ((hw2 >> 12) & 0x7) << 2 | ((hw2 >> 6) & 0x3);
    (lsb, hw2 & 0x1F)
}

fn reg(op: &Operand) -> Result<u8, ()> {
    match op {
        Operand::Reg(r) => Ok(r.number()),
        _ => Err(()),
    }
}
fn imm_val(op: &Operand) -> Result<u32, ()> {
    match op {
        Operand::Imm32(i) => Ok(*i),
        Operand::Imm12(i) => Ok(*i as u32),
        _ => Err(()),
    }
}
fn reglist(op: &Operand) -> Result<u16, ()> {
    match op {
        Operand::RegList(m) => Ok(*m),
        _ => Err(()),
    }
}
fn regwback(op: &Operand) -> Result<(u8, bool), ()> {
    match op {
        Operand::RegWBack(r, wb) => Ok((r.number(), *wb)),
        _ => Err(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::host::StdoutHost;

    // Thumb `BKPT #0` = 0xBE00.
    const BKPT: u16 = 0xBE00;
    const RAM: u32 = 0x2000_0000;

    fn ram_bus() -> Bus {
        let mut bus = Bus::new();
        bus.log_unmapped = false;
        bus.add_ram(RAM, 0x1000);
        bus
    }

    #[test]
    fn bkpt_halts_when_debug_enabled() {
        let mut bus = ram_bus();
        bus.write16(RAM, BKPT);
        let mut cpu = Cpu::new();
        cpu.debug_en = true;
        cpu.pc = RAM;
        let mut host = StdoutHost;
        assert!(cpu.step(&mut bus, &mut host).is_ok());
        assert!(cpu.halted, "BKPT with C_DEBUGEN halts into debug state");
        assert!(cpu.bkpt_hit, "the halt is attributed to a breakpoint");
        assert_eq!(cpu.pc, RAM, "PC stays at the BKPT, not past it");
    }

    #[test]
    fn bkpt_faults_when_debug_disabled() {
        let mut bus = ram_bus();
        bus.write16(RAM, BKPT);
        let mut cpu = Cpu::new();
        cpu.debug_en = false;
        cpu.pc = RAM;
        let mut host = StdoutHost;
        assert!(
            cpu.step(&mut bus, &mut host).is_err(),
            "no debugger: BKPT is a fault, not a halt"
        );
        assert!(!cpu.halted);
    }
}
