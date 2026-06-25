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

## I2C bridge demos — pipe the SP's I2C traffic to a local process

Box-local (no rack). Each flashes a gimlet image, boots a standalone `sp-emu`,
and prints every command it runs (transcript style). Build first: `cargo build
--release`.

**`./i2c-sniff.sh [hubris.zip]` — SNIFF (observe).** Live-streams every I2C
transaction the SP makes, then a per-device access summary. Uses `sp-emu
i2c-sniff` + `SP_EMU_I2C_BRIDGE`. Watch the SP bring the board up over I2C:
muxes → GPIO expander → VPD EEPROM → temp sensors.

**`./i2c-device.sh [--read-sensor] [spec] ` — DELEGATE (be the device).** A local
socket *answers as* the I2C device(s); everything else defers to the built-in
model so the SP still boots. Uses `sp-emu i2c-device` + `SP_EMU_I2C_DEVICE`.

```
./i2c-device.sh                       # default: front TMP117 (0x48) reads ~128 C
./i2c-device.sh 0x48/0x00=0x4000      # inject a register value (hi byte at idx 0)
./i2c-device.sh 0x50@/path/vpd.bin    # serve a file as a VPD/FRUID EEPROM
./i2c-device.sh --read-sensor         # ALSO read it back via Hubris's sensor API (MGS)
```

`--read-sensor` shows the injected value propagating end-to-end: socket → SP I2C
→ Hubris sensor task → `faux-mgs read-sensor-value` (the injected sensor reports
your value; the others read their normal ~30 C).

Wire protocol (so you can write a device model in any language): `S <bus> <addr>
<R|W>`, `W <bus> <addr> <byte>`, `R <bus> <addr> <reg> <idx>` → server replies a
hex byte or `-` (defer). `faux-mgs`/the hubris image default to `/root/oxide`
sibling paths (override `$FAUX` / `$IMG`).
