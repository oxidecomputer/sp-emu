# sp-emu demo: an emulated gimlet SP you can talk to with MGS

This boots Oxide Hubris gimlet-c firmware on a from-scratch emulated
STM32H753 (Cortex-M7), brings up its network stack, and lets the management
gateway (MGS) talk to it over UDP exactly like real hardware: `discover`,
`state`, `inventory`, etc.

The scripts here resolve `sp-emu` and the flash file relative to the repo they
live in, so you can run them from anywhere. Override `SPEMU` / `SP_EMU_FLASH` to
point elsewhere.

## One-time check

- `target/release/sp-emu` exists (else `cargo build --release` from the repo root)
- a `sp-flash.bin` at the repo root has gimlet-c in slot A. If not:
  `sp-emu flash a <hubris>/target/gimlet-c/dist/default/build-gimlet-c-image-default.zip`
- `humility` on your PATH (for `./tasks`)
- `$FAUX_MGS` points at a `faux-mgs` from the management-gateway-service repo
  (the `mgs` helper uses it; it must match the firmware's gateway-messages revision)

## The show

Terminal 1, boot the SP and wait for it to come online:

```
cd demo
./run-sp.sh
```

Wait ~60-90s for `[+] SP ONLINE` (it is really booting a kernel plus 30+ tasks).

Terminal 2, talk to it with MGS:

```
cd demo

./mgs discover      # MGS discovers the SP, reports which switch port
./mgs state         # power state (A2), base MAC, firmware archive id, RoT status
./mgs inventory     # the whole gimlet component tree (CPU, flash, sensors, U.2, ...)

SP_PORT=33311 ./mgs state    # same SP, reached via the other switch uplink
```

Use the demos to run some actual commands:

```
./tasks             # live Hubris task table: jefe, net, gimlet_seq, hf,
                    # control_plane_agent, thermal, power, sprot, ... all live
./tasks -sl net     # source-line stack backtrace of a running task
```

## Notes and caveats

- This is the actual production gimlet firmware image, unmodified, not a
  simulator that fakes responses. `humility` attaches and debugs it like a real
  board.
- MGS reaches it on both switch uplinks (ports 33310 / 33311), the same
  dual-path wiring a real gimlet has to the two sidecars.
- `state` returns `rot: Err(...)` because there is no emulated root-of-trust in this demo.
  However, emulated RoT is available in voxel a4x2 environments.
- The emulator runs ~4M instr/s, so MGS uses a generous per-attempt timeout
  (baked into `./mgs`). Commands take a few seconds.
- The `mgs` helper needs `$FAUX_MGS` set to a `faux-mgs` from the
  management-gateway-service repo, built from the same `gateway-messages` revision
  as the gimlet image. The wire protocol must match the firmware, so the demo does
  not guess a path.
- Run `./tasks` when the SP is idle (not mid `./mgs` command): humility halts
  the CPU to read memory.

## I2C bridge demos: pipe the SP's I2C traffic to a local process

Box-local (no rack). Each flashes a gimlet image, boots a standalone `sp-emu`,
and prints every command it runs (transcript style). Build first: `cargo build
--release`.

`./i2c-sniff.sh [hubris.zip]`, SNIFF (observe). Live-streams every I2C
transaction the SP makes, then a per-device access summary. Uses `sp-emu
i2c-sniff` plus `SP_EMU_I2C_BRIDGE`. Watch the SP bring the board up over I2C:
muxes, GPIO expander, VPD EEPROM, temp sensors.

`./i2c-device.sh [--read-sensor] [spec]`, DELEGATE (be the device). A local
socket answers as the I2C device(s); everything else defers to the built-in
model so the SP still boots. Uses `sp-emu i2c-device` plus `SP_EMU_I2C_DEVICE`.

```
./i2c-device.sh                       # default: front TMP117 (0x48) reads ~128 C
./i2c-device.sh 0x48/0x00=0x4000      # inject a register value (hi byte at idx 0)
./i2c-device.sh 0x50@/path/vpd.bin    # serve a file as a VPD/FRUID EEPROM
./i2c-device.sh --read-sensor         # ALSO read it back via Hubris's sensor API (MGS)
```

`--read-sensor` shows the injected value propagating end-to-end: socket, SP I2C,
Hubris sensor task, `faux-mgs read-sensor-value` (the injected sensor reports
your value; the others read their normal ~30 C).

Wire protocol (so you can write a device model in any language): `S <bus> <addr>
<R|W>`, `W <bus> <addr> <byte>`, `R <bus> <addr> <reg> <idx>`, and the server
replies a hex byte or `-` (defer). Set `$FAUX` to your `faux-mgs`
(management-gateway-service repo; only needed for `--read-sensor`) and `$IMG` to
your hubris image.
</content>
