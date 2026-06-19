# sp-emu demo — an emulated gimlet SP you can talk to with MGS

This boots **real Oxide Hubris gimlet-c firmware** on a from-scratch emulated
STM32H753 (Cortex-M7), brings up its network stack, and lets the management
gateway (MGS) talk to it over UDP exactly like real hardware — `discover`,
`state`, `inventory`, etc.

## One-time check

- `~/oxide/sp-emu/target/release/sp-emu` exists (else `cargo build --release` in `~/oxide/sp-emu`)
- `~/oxide/sp-emu/sp-flash.bin` has gimlet-c in slot A (else:
  `sp-emu flash a ~/oxide/hubris/target/gimlet-c/dist/default/build-gimlet-c-image-default.zip`)
- `humility` on your PATH (for `./tasks`)

## The show

**Terminal 1 — boot the SP and wait for it to come online:**

```
cd ~/oxide/sp-emu/demo
./run-sp.sh
```

Wait ~60-90s for `[+] SP ONLINE` (it's really booting a kernel + 30+ tasks).

**Terminal 2 — talk to it with MGS:**

```
cd ~/oxide/sp-emu/demo

./mgs discover      # MGS discovers the SP, reports which switch port
./mgs state         # power state (A2), base MAC, firmware archive id, RoT status
./mgs inventory     # the whole gimlet component tree (CPU, flash, sensors, U.2, ...)

SP_PORT=33311 ./mgs state    # same SP, reached via the *other* switch uplink
```

**The money shot — it's really running firmware:**

```
./tasks             # live Hubris task table: jefe, net, gimlet_seq, hf,
                    # control_plane_agent, thermal, power, sprot, ... all live
./tasks -sl net     # source-line stack backtrace of a running task
```

## What to point out while showing off

- This is the **actual production gimlet firmware image**, unmodified — not a
  simulator that fakes responses. `humility` attaches and debugs it like a real
  board.
- MGS reaches it on **both switch uplinks** (ports 33310 / 33311) — the same
  dual-path wiring a real gimlet has to the two sidecars.
- `state` returns `rot: Err(...)` because there's no emulated root-of-trust yet
  — that's expected and matches what sp-sim does.

## Notes / caveats

- The emulator runs ~4M instr/s, so MGS uses a generous per-attempt timeout
  (baked into `./mgs`). Commands take a few seconds.
- `faux-mgs` here is built from the exact `gateway-messages` revision the
  gimlet image was compiled against — the wire protocol must match the firmware.
- Run `./tasks` when the SP is idle (not mid-`./mgs`-command): humility halts
  the CPU to read memory.
