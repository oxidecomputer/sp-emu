#!/usr/bin/env python3
"""Differential tester: lockstep sp-emu's instruction trace against Unicorn.

sp-emu dumps per-instruction state (SP_EMU_DIFF). For each instruction we set
Unicorn's registers to sp-emu's *pre* state, single-step one instruction, and
compare the *post* state. The first mismatch on a non-skipped line is the
earliest buggy instruction in sp-emu. Skip lines (MMIO / exception / VFP) are
re-synced but not compared, since Unicorn can't mirror those.

Usage: diff.py <flash.bin> <trace.txt>
"""
import sys
from unicorn import *
from unicorn.arm_const import *

REGS = [UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3,
        UC_ARM_REG_R4, UC_ARM_REG_R5, UC_ARM_REG_R6, UC_ARM_REG_R7,
        UC_ARM_REG_R8, UC_ARM_REG_R9, UC_ARM_REG_R10, UC_ARM_REG_R11,
        UC_ARM_REG_R12, UC_ARM_REG_SP, UC_ARM_REG_LR]  # r0..r14
import unicorn.arm_const as _ac
SREGS = [getattr(_ac, f'UC_ARM_REG_S{i}') for i in range(32)]

def main():
    flash_path, trace_path = sys.argv[1], sys.argv[2]
    flash = open(flash_path, 'rb').read()

    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    # Map flash + all RAM banks + peripheral space (4K-aligned, generous).
    uc.mem_map(0x08000000, 0x200000)            # flash (2 MB, both slots)
    uc.mem_write(0x08000000, flash)
    uc.mem_map(0x00000000, 0x10000)             # ITCM / boot alias
    uc.mem_map(0x20000000, 0x20000)             # DTCM
    uc.mem_map(0x24000000, 0x80000)             # AXI SRAM
    uc.mem_map(0x30000000, 0x50000)             # SRAM1/2/3
    uc.mem_map(0x38000000, 0x10000)             # SRAM4
    uc.mem_map(0x40000000, 0x20000000)          # peripherals (D2/D3 APB/AHB)
    uc.mem_map(0xE0000000, 0x100000)            # PPB / SCS

    # Enable the FPU so VFP-adjacent code doesn't fault (CPACR CP10/CP11).
    uc.reg_write(UC_ARM_REG_C1_C0_2, uc.reg_read(UC_ARM_REG_C1_C0_2) | (0xf << 20))
    uc.reg_write(UC_ARM_REG_FPEXC, 0x40000000)

    lines = []
    with open(trace_path) as f:
        for ln in f:
            t = ln.split()
            if 'S' not in t or 'W' not in t:
                continue
            si, w = t.index('S'), t.index('W')
            vals = [int(x, 16) for x in t[:18]]
            skip = int(t[18])
            sregs = [int(x, 16) for x in t[si + 1:w]]
            writes = []
            for wt in t[w + 1:]:
                a, v, sz = wt.split(':')
                writes.append((int(a, 16), int(v, 16), int(sz)))
            lines.append((vals[0], vals[1:16], vals[16], vals[17], skip, writes, sregs))
            #            instr_pc, r0..r14,    r15(next), apsr,   skip, writes, sregs

    def apply_writes(line):
        # Force Unicorn's memory to match sp-emu's by replaying sp-emu's recorded
        # writes (overwriting whatever Unicorn's own execution wrote). Applied
        # after every line so memory stays exactly synced across skipped instrs.
        for a, v, sz in line[5]:
            try:
                uc.mem_write(a, v.to_bytes(4, 'little')[:sz])
            except UcError:
                pass

    print(f"loaded {len(lines)} trace lines")
    apply_writes(lines[0])
    mism = 0
    for n in range(1, len(lines)):
        pre = lines[n - 1]
        cur = lines[n]
        instr_pc, exp_regs, exp_next_pc, exp_apsr, skip, _, exp_sregs = cur
        if skip:
            apply_writes(cur)
            continue
        # set Unicorn to sp-emu's pre-state (integer + FP registers)
        for r, v in zip(REGS, pre[1]):
            uc.reg_write(r, v)
        for r, v in zip(SREGS, pre[6]):
            uc.reg_write(r, v)
        uc.reg_write(UC_ARM_REG_CPSR, pre[3] | (1 << 5))  # +Thumb bit
        try:
            uc.emu_start(instr_pc | 1, instr_pc + 8, count=1)
        except UcError as e:
            print(f"[n={n}] uc error @ {instr_pc:#010x}: {e}")
            apply_writes(cur)
            mism += 1
            if mism > 5: break
            continue
        got_regs = [uc.reg_read(r) for r in REGS]
        got_pc = uc.reg_read(UC_ARM_REG_PC)
        got_apsr = uc.reg_read(UC_ARM_REG_CPSR) & 0xF8000000
        bad = []
        for i, (g, e) in enumerate(zip(got_regs, exp_regs)):
            if g != e:
                bad.append(f"r{i}: emu={e:#010x} uc={g:#010x}")
        if got_pc != exp_next_pc:
            bad.append(f"pc: emu={exp_next_pc:#010x} uc={got_pc:#010x}")
        if got_apsr != exp_apsr:
            bad.append(f"apsr: emu={exp_apsr:#010x} uc={got_apsr:#010x}")
        for i, e in enumerate(exp_sregs):
            g = uc.reg_read(SREGS[i])
            if g != e:
                bad.append(f"s{i}: emu={e:#010x} uc={g:#010x}")
        if bad:
            code = uc.mem_read(instr_pc, 4)
            print(f"\n*** MISMATCH at instruction #{n}, pc={instr_pc:#010x} "
                  f"bytes={' '.join(f'{b:02x}' for b in code)} ***")
            for b in bad:
                print("   ", b)
            mism += 1
            if mism > 12:
                print("...stopping after several mismatches")
                break
        apply_writes(cur)  # keep Unicorn's memory == sp-emu's after every step
    if mism == 0:
        print("no mismatches — sp-emu matches Unicorn across the whole trace")

if __name__ == '__main__':
    main()
