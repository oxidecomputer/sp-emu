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

Wait tens of seconds for the SP to come online (it is really booting a kernel
plus 30+ tasks); `run-sp.sh` prints `online:` with the ports when it is ready.

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
./hiffy             # call a Hubris IPC (default Jefe.get_state) over the SWD port
./hiffy -l          # list the interfaces hiffy can call
./tasks --swd       # same task table, but over the SWD debug port instead of ocd
```

`./tasks` uses the OpenOCD transport (`-p ocd`), which humility expects on
localhost:6666, so boot sp-emu with its MGS bridge on `[::1]:33300` for that
(`SP_BASE=33300 ./run-sp.sh`). `./hiffy` and `./tasks --swd` use the new **SWD
debug port**: a real halt/run/step debug core exposed as a Glasgow probe, so
`hiffy` (which injects and runs a program) actually works, where it hangs on
the fake-halt `-p ocd`/`-p ocdgdb` transports. The SWD port follows the bridge
(`4444 + (SP_BASE - 33300)`); `sp-emu gdb` prints the exact ports on startup.

## Watch the RoT measure the SP (attestation)

```
./run-sp-measure.sh <hubris>/.../oxide-rot-1-selfsigned/dist/a/build-oxide-rot-1-selfsigned-image-a.zip
```

Boots the SP with an **in-process** LPC55 RoT that drives the SP's debug port over
an internal SWD link and performs the RFD 568 attestation measurement, exactly as
real hardware does at boot: it resets the SP into debug halt, injects the
`endoscope` program, runs it to hash the SP flash, reads the digest back, and
deposits the VALID measurement token so the SP boots normally. The script waits for
`SP measurement recorded` and then leaves the SP running for `./tasks --swd` and
`./mgs`. (This is different from `run-sp-rot.sh`, which attaches an out-of-process
RoT over the sprot SPI link only for boot-info.)

## The full testbed: SP + RoT + bootleby, with SWD and IPCC

`run-testbed.sh` brings up the configuration closest to a real board and leaves
it serving until you stop it. Use it for humility, faux-mgs, faux-ipcc, and test
harnesses; the other scripts here are narrower demos.

```
./run-testbed.sh <gimlet SP archive> <oxide-rot-1 SELF-SIGNED archive>

# with real bootleby doing genuine A/B slot selection
BOOTLEBY=<bootleby-oxide-rot-1.zip> ./run-testbed.sh <sp archive> <rot archive>
```

It prints every endpoint it brings up:

| Interface | Endpoint | How to use it |
| --- | --- | --- |
| MGS | `[::1]:11111` | `faux-mgs --sp-sim-addr '[::1]:11111' state` |
| SP SWD probe | `tcp:127.0.0.1:4444` | `humility -a <sp archive> -p 20b7:9db1:tcp:127.0.0.1:4444 tasks` |
| RoT SWD probe | `tcp:127.0.0.1:4544` | `humility -a <rot archive> -p 20b7:9db1:tcp:127.0.0.1:4544 tasks` |
| IPCC | a pty (`/dev/pts/N`) | `faux-ipcc` with baud 0 (the pty path; no DTR ioctl) |

The RoT archive must be the **self-signed** `oxide-rot-1` build: it is Bart-signed
so the emulated boot ROM authenticates it, and it takes the `dice-self` identity
path. The production `dice-mfg` image wedges polling the unmodeled manufacturing
USART.

sp-emu boots the RoT through **real bootleby by default**: it looks for
`bootleby-oxide-rot-1.zip` next to the RoT archive, then under `$HUBRIS`.
Bootleby is what performs A/B slot selection and honors the CFPA's persistent
boot preference, so without it neither behavior is modeled. `BOOTLEBY=` names an
image explicitly; `SP_EMU_ROT_NO_BOOTLEBY=1` opts out, falling back to jumping
straight into the Hubris image, with the bootloader slots holding a synthetic
caboose-only image so `component/stage0/caboose` reads still answer.

The CMPA/CFPA that ship inside a bootleby archive are not seeded: sp-emu's
synthesized CMPA is byte-identical to a real oxide-rot-1's (Bart keyset,
debug-open `DCFG_CC_SOCU`, unsealed), and its CFPA is a factory-fresh version 0.

### Where to get the images

sp-emu needs a Hubris SP archive, a self-signed `oxide-rot-1` archive, and
bootleby. All three are published by the hubris release process, not built here:

- **SP images**: the `all-sp-v*` GitHub release of the hubris repo, published
  per board by its Release workflow as `dist-<runner>-hubris-<board>` artifacts,
  each holding `build-<board>-image-default.zip` (`dev` / `lab` / plain variants).
- **RoT Hubris images**: a separate `oxide-rot-1-v*` release on its own cadence,
  as release assets: `build-oxide-rot-1-selfsigned-image-{a,b}.zip` (use the
  self-signed one) and the production-signed `build-oxide-rot-1-image-{a,b}.zip`.
- **bootleby**: checked into the hubris tree at
  `app/oxide-rot-1/bootleby-oxide-rot-1.zip`, so a hubris checkout already has
  it (`app/{lpc55xpresso,rot-carrier}/` hold the other boards' builds).
- **Non-release boards** (grapefruit, gimletlet, nucleo): built by the CI
  workflow as `dist-ubuntu-latest-<board>`; the release commit has no CI run, so
  take the run at the release's merge-base on master.

Any Hubris build archive works. A local `hubris` `cargo xtask dist` tree is fine
for development. The releases matter when you want a known, reproducible set.

sp-test automates fetching all of the above with
`scripts/get-release-images.sh`, which resolves the tags and lays the archives
out in one directory. sp-emu deliberately does not carry its own copy: it
encodes GitHub artifact naming that changes, and a second copy would drift.

### faux-mgs against sp-emu

- **Use `--sp-sim-addr`, not `--interface`/`--discovery-addr`.** On loopback the
  interface-scoped form cannot resolve the peer's interface index, so the reply
  is discarded and discovery times out with `no interface name found for index 0`.
- **Match revisions.** faux-mgs must be built against the same
  `gateway-messages` revision as the SP image, or discovery fails outright.
- **Well-known vs offset ports.** `run-testbed.sh` uses
  `SP_EMU_WELL_KNOWN_PORTS`, binding the SP's real ports (11111 MGS, 57005
  ereport, ...) so tools need no port arithmetic. Only one instance can hold them
  at a time; run a fleet with the default offset mode (`SP_EMU_BRIDGE`) instead.
- **Attaching a SWD probe asserts JTAG_DETECT**, which invalidates the RoT's
  attestation log, the same thing a real probe does on real hardware. sp-emu
  asserts it on probe *connect*, not on SWD traffic, so a passively attached,
  idle probe still counts as connected.

### VPD: don't look like real hardware

By default an emulated SP reports an Oxide-style serial (`BRM4422000<n>`, part
`913-0000019`), which is indistinguishable from a shipped machine in inventory.
Override the VPD/FRUID barcode so it is obvious what it is:

```
SP_EMU_VPD_SERIAL=EMU00000001 \
SP_EMU_VPD_PART=EMULATED-SP \
SP_EMU_VPD_REV=001 \
  ./run-testbed.sh <sp archive> <rot archive>
```

`faux-mgs state` then reports that serial, model, and revision. Serial and part
are capped at the 11 characters the 0XV2 barcode allows and are truncated if
longer. Inventory keys SPs on serial, so give each instance in a fleet a
distinct one. `SP_EMU_VPDDBG=1` prints the barcode the SP is built with.

The default part number follows the board being modeled: gimlet-c is
`913-0000019`. sp-emu's sidecar model is sidecar-c, but no sidecar part number is
recorded in the sources sp-emu can see, so it reports a placeholder and says so
on startup. Set `SP_EMU_VPD_PART` to the real one when you know it.

### Boot time is host-dependent

The pair is two emulated cores booting real kernels: roughly ten seconds on a
fast laptop, several times that on a slower or loaded machine. `run-testbed.sh`
polls for readiness rather than sleeping. Anything driving sp-emu should do the
same and must not hard-code a delay; a fixed wait that passes here will fail on
someone else's machine or in CI.

### Where persistent state lives

Everything durable lives under `$STATE_DIR` (`SP_EMU_STATE_DIR`): the SP flash
image and its `.nv` sidecar, the RoT flash image plus its `.erased` page bitset,
the derived per-instance identity, and stowed Hubris archives. A persisted RoT
flash takes precedence over the image on the command line, so protected-flash
overrides (CMPA/CFPA) apply only when the flash is seeded fresh. `FRESH=1`
(or `SP_EMU_ROT_FRESH`) reseeds, and `rm -rf "$STATE_DIR"` is a factory reset.

This matters beyond convenience: CFPA carries the boot preference and the key
revocation state, so anything exercising RoT key rotation or invalidation
depends on those files surviving across sessions, and on knowing when they are
being reseeded instead.

The baseline is deliberate. A freshly seeded instance starts at CFPA version 0,
the factory-fresh state, not at the high counter a field part carries. A restart
keeps whatever value the firmware has advanced it to. So a rollback or
revocation test starts from a known floor, and only `FRESH=1` or removing
`$STATE_DIR` returns it there.

## Notes and caveats

- This is the actual production gimlet firmware image, unmodified, not a
  simulator that fakes responses. `humility` attaches and debugs it like a real
  board.
- MGS reaches it on both switch uplinks (ports 33310 / 33311), the same
  dual-path wiring a real gimlet has to the two sidecars.
- `state` returns `rot: Err(...)` because this standalone SP has no RoT. Attach
  one with `./run-sp-rot.sh` (an out-of-process RoT over sprot, for boot-info) or
  `./run-sp-measure.sh` (an in-process RoT that measures the SP over SWD).
- The emulator runs tens of millions of instructions per second; MGS still uses a
  generous per-attempt timeout (baked into `./mgs`), and commands take a few
  seconds of round-trips.
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
