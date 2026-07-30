# Faithful two-core co-scheduler and shared timebase (design)

Status: design. Increment 1 (endoscope coupling) is implemented; the phase state
machine and self-caused-reset flag are not yet. Captures the model and an incremental
delivery plan for making the SP and RoT cores schedule as they do on hardware.

## Problem

sp-emu steps two independent cores (STM32H753 SP, LPC55S69 RoT) in one serve loop
(`gdb::serve`, `src/gdb.rs`). Faithful scheduling means the two cores' clocks advance at
a consistent real-time rate, and one core's timeouts count down at the other's true
rate, as on hardware. Today this is approximated by scattered, case-specific heuristics,
and the underlying timebases are inconsistent.

### Inconsistent timebases (measured)

The emulator has no wall clock; time is derived from retired instructions
(1 instruction = 1 cycle, `cpu.rs:405/411`; `bus.cur_cyc = cpu.cycles`). Three implied
instruction-to-time rates coexist and diverge:

| Timebase | Source | instructions per 1 ms |
|---|---|---|
| SP SysTick | firmware `SYST_RVR` (clock_khz minus 1), echoed by `Scs` (soc.rs:2020), cached in `cpu.syst_rvr` (cpu.rs:1633) | ~480,000 (480 MHz) |
| RoT SysTick | same path; LPC55 FRO 96 MHz (lpc55.rs:92) | ~96,000 |
| SP TIM5 | `TIM5_CNT = cur_cyc - tim5_base` (mem.rs:864), firmware treats it as 1 MHz | 1,000 |

The serve loop steps both cores at the same `quantum` (`eth_quantum`, default 4096,
`config.rs:256`), with `rot_budget` (256 or 1) as a throughput multiplier, not a clock
ratio. So per retired instruction the RoT's ms-clock advances about 5x faster than the
SP's, and TIM5-paced SP delays run about 480x faster than SysTick-paced SP code. Nothing
carries a per-core instructions-to-ms factor except the sprot coupling, which sidesteps
the issue by working in tick (ms) units.

### Scattered scheduling heuristics (the unification target)

From `gdb::serve`:
- SP burst per iteration: `sp_burst_for` (halted or reset-servicing yields 0; endoscope
  yields a 20M-instruction sprint; a phase-2 reply yields 48; otherwise the quantum),
  gdb.rs:33-52.
- RoT budget per iteration: 256 during an exchange, otherwise 1, gdb.rs:639-643.
- The sprot SysTick coupling (already landed): credit `cpu.sp_tick_credit` from the RoT's
  tick-event delta while a request is in flight, gdb.rs:773-807; drained in WFI,
  cpu.rs:594.
- The SP_RESET prompt-halt servicing (already landed): freeze the SP so the RoT halts it
  at the reset vector, gdb.rs:571-578, 811-817.
- Phase-2 flow control, `rot_busy`, the host idle-sleep gate, the SWD drain, and the
  RoT-wake logic. Plus a second, mutually exclusive shared-RoT-client path.

## The unifying idea

Almost all two-core interaction in this emulator is one core blocked, waiting on the
other's work: sprot (the RoT computes a reply while the SP waits) and the measurement
dance (the SP runs endoscope while the RoT polls; the RoT drives SWD while the SP is
halted). The cores are rarely both computing at once. So one rule covers the real cases:

> Blocked-on-working coupling: while core A is blocked on core B, advance A's clock by
> B's real progress.

B's progress has two forms, and this is where the timebase divisor matters:
- Tick-based, when B's kernel SysTick runs: credit A by B's tick-event delta (both ticks
  are 1 ms). This is the existing sprot coupling (RoT works, SP waits).
- Instruction-based, when B's SysTick is off (a bare or pre-kernel program): credit A by
  `d(B.cycles) / B_divisor` ms, with a fractional-remainder accumulator. This is the
  endoscope case (the SP hashes flash with SysTick disabled while the RoT polls), which
  is why a naive tick mirror produces no coupling at all. `B_divisor = B.syst_rvr + 1`,
  retained through a soft reset (`reset_for_reboot` clears only `systick`).

A blocked core idle in WFI does not advance its own SysTick, so it drains the credit to
fire its SysTick (cpu.rs:594-618, already generic across both cores).

### Why not a globally consistent clock (proportional stepping)?

The most correct model steps each core proportionally to its real clock (SP 5 : RoT 1)
so both clocks advance at one shared real-time rate even when both compute. It is
rejected as the primary model because:
1. The two cores are almost never both computing here, so it fixes a case that rarely
   occurs.
2. It changes the fundamental stepping ratio the whole battle-tested loop (sprot
   updates, prompt-halt, MGS bridge pacing) is tuned to, which is high risk for little
   practical gain.

Keep it on the table only if a genuine both-cores-compute workload appears. The
per-core-divisor coupling above gives the fidelity that matters without touching the
global stepping ratio.

The endoscope increment is the closest thing to a both-cores-compute case (the SP hashes
flash while the RoT actively polls the halt), and it was resolved without proportional
stepping: freeze the polling core's SysTick so its own execution does not advance its
clock, and let the blocked-on-working credit drive it at the working core's rate. The
freeze is what distinguishes an actively polling waiter from one asleep in WFI, whose
clock already does not advance while it waits. This keeps the divisor coupling sufficient
and leaves the global stepping ratio untouched.

### TIM5

TIM5's 1 microsecond per instruction is a deliberate boot-delay compression (the
firmware reads only deltas, mem.rs:174-184); it is SP-local and not part of cross-core
timing. Leave it as is, and document that it is intentionally not on the SysTick
timebase. The co-scheduler's divisors come from each core's `syst_rvr`, not TIM5.

## Architecture

### 1. Per-core timebase
A small accessor `Cpu::tick_divisor() -> u32` returning `syst_rvr + 1` (floored to a
sane minimum so an unconfigured core, `syst_rvr == 1`, does not couple). This is the only
new primitive the coupling rule needs.

### 2. Link phase state machine
Model the SP-to-RoT link as an explicit phase, derived from the existing signals
(`request_in_flight`, `cs`/`ssa`, `rot_irq`, `cpu.halted`, `cpu.debug_en`,
`reset_pending`, `sp_reset_release`, `jtag_detect`):

Idle, SprotRequest, SprotReply, ResetCaught, EndoscopeInject, EndoscopeRun, DigestRead,
LogUpdate, Release, then back to Idle.

Each phase declares which core works versus blocks, each core's per-iteration budget, and
the coupling rule (none, tick-based, or instruction-based). One `co_scheduler(phase, ...)`
returns `(sp_burst, rot_budget, coupling)` per iteration, replacing `sp_burst_for`,
`rot_budget`, the two coupling blocks, and the prompt-halt servicing with one component.
Keep the pure-helper and unit-test style the existing scheduling helpers already use.

### 3. Self-caused reset
The Release phase arms a one-shot flag meaning the next SP_RESET is RoT-initiated and
must not re-trigger measurement, so the RoT-driven SP release does not re-enter
prompt-halt or re-measure. This may already be implicitly true because a RoT-driven reset
goes through `do_reset` rather than `reset_pending`; the state machine makes it explicit
and verifiable.

## Incremental delivery (later rounds)

1. Foundation plus endoscope coupling (landed). `Cpu::tick_divisor()` plus the
   instruction-based endoscope coupling: cap the SP burst during `debug_en` so the RoT
   interleaves, freeze the RoT's SysTick (`Cpu::tick_frozen`) so its polling does not
   advance its own clock, and credit the RoT `d(cpu.cycles) / SP_divisor` ms with a
   fractional-remainder accumulator. The measurement runs pre-kernel, so `tick_divisor()`
   is `None` and the divisor comes from `SP_EMU_SP_CLOCK_KHZ` (default 400000). Gated by
   `SP_EMU_ENDOSCOPE_COUPLE` (default on); off restores the one-shot sprint. Does not
   require the state machine. Result: the measurement completes in one attempt with a
   bounded halt time proportional to the SP's endoscope instructions (about 330 ms for a
   133M-instruction hash) instead of timing out and retrying.
2. Phase state machine unification. Introduce the phase enum and `co_scheduler`; move
   `sp_burst_for`, `rot_budget`, both couplings, and prompt-halt behind it,
   behavior-preserving. Cover both the in-process-RoT and shared-client paths.
   Medium-to-large risk (it restructures the loop); mostly code-organization value.
3. The self-caused-reset flag (small), naturally part of the state machine.
4. Deferred: proportional cross-core stepping, only if a both-cores-compute case arises.

Already landed and unchanged: the SP_RESET prompt-halt, the sprot SysTick coupling,
JTAG_DETECT, and now the endoscope coupling (increment 1 above).

## Verification (for the implementing rounds)

Reuse the working self-reset reproduction (a scratch instance, gimlet-c SP plus
oxide-rot-1-selfsigned RoT, `SP_EMU_ROT_FLASH` plus `SP_EMU_ROT_ROM=1` plus
`SP_EMU_ROT_MEASURE=1` plus `SP_EMU_COUPLEDBG=1`, `run a 0`):
- The measurement still completes (`SP measurement recorded: VALID`, `sp-emu online`).
- With endoscope coupling on, the RoT records a bounded, non-zero halt time proportional
  to the SP's endoscope instructions (RoT ringbuf `Trace::Halted { delta_t }` greater
  than 0 and well under 500), versus about 0 with the sprint; `SP_EMU_ENDOSCOPE_COUPLE=0`
  restores the old behavior for an A/B contrast.
- The state-machine refactor must be a strict no-op on the sprot update path and on
  prompt-halt behavior: re-run an RoT or stage0 update (faux-mgs, generous budgets) and
  the self-reset reproduction, and diff behavior against the pre-refactor build.
  Unit-test the phase transitions and the per-phase `(sp_burst, rot_budget, coupling)`
  outputs as pure functions.

## Open questions to resolve before the state-machine round

- The exact phase set and the signal-to-phase mapping, especially the shared-RoT-client
  path, which has no in-process RoT core to couple against.
- Whether the emulator should model `SYST_RVR` being cleared by `SYSRESETREQ` (real
  hardware does; today it is retained). Increment 1 no longer depends on this: the
  endoscope divisor falls back to `SP_EMU_SP_CLOCK_KHZ` when `tick_divisor()` is `None`,
  so clearing `SYST_RVR` would not break endoscope coupling. The question remains only for
  other pre-kernel timing that might want the real reload.
- Whether `CREDIT_CAP` (5000) and the endoscope burst chunk need per-phase tuning.
