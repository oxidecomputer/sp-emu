# sp-emu

A native-Rust emulator that boots unmodified Oxide Hubris firmware for a
service processor (SP) and root of trust (RoT), with no hardware or RTOS
underneath. It models enough of the STM32H7 (the SP) and LPC55 (the RoT) that the
production firmware images come up on their own, bring up their networks, and
answer the management gateway (MGS) over UDP.

## What it can do

- Boots the production gimlet-c and sidecar SP images on an emulated STM32H753
  (Cortex-M7), from the reset vector through the kernel and 30-plus Hubris tasks.
- Boots the oxide-rot-1 RoT image on an emulated LPC55 (Cortex-M33).
- Answers MGS over UDP on loopback, on both switch uplink ports, the same way Oxide
  hardware does: `discover`, `state`, `inventory`, `read-sensor-value`,
  `power-state`, `rot-boot-info`, caboose reads, dumps, and the rest of the
  faux-mgs surface.
- Lets `humility` attach to the running firmware over a real SWD debug port
  exposed as a Glasgow Interface Explorer probe that models actual halt/run/step,
  so halt-and-run commands like `hiffy` work, alongside the live task table,
  per-task stack backtraces, `readmem`/`writemem`, and ringbufs.
- Runs the SP and RoT together over an emulated sprot SPI link, so
  `drv-stm32h7-sprot-server` on the emulated SP talks to `drv-lpc55-sprot-server`
  on the emulated RoT. The RoT publishes boot-state measurements (sha3-256 of the
  flashed image), so `rot-boot-info` returns digests instead of zeros.
- Sniffs, or stands in for, the SP's I2C devices over a socket, so you can watch
  the bus traffic the firmware generates or inject your own sensor, VPD, and
  FRUID values from a process written in any language.

It is the emulated SP/RoT backend for the [voxel](https://github.com/oxidecomputer/voxel)
virtual rack, and it also runs standalone for firmware bring-up and debugging.

## How it works

- A from-scratch Cortex-M interpreter (M7 for the SP, M33 for the RoT) decodes and
  executes the Thumb-2 image against a flat memory and bus model.
- Each SoC peripheral the firmware touches has its own small MMIO device model: the
  Ethernet MAC and MDIO/PHY (so the `net` task's stack comes up), the SPI
  controllers and the devices behind them (the KSZ8463 management switch, the
  sequencer FPGA, the sidecar's VSC7448), I2C with its muxes, a GPIO expander, the
  AT24CSW VPD/FRUID EEPROM, and the TMP117 and TSE2004 temperature sensors, plus
  the UARTs, EXTI, and the hash block.
- Flash is a two-bank NVM image (slot A and slot B) kept in a file, the same A/B
  layout the real SP has. You program a slot from a Hubris build archive, then boot
  it.
- A small bridge presents the SP's MGS UDP surface on a loopback address, as two
  ports (one per switch view), which is all MGS needs to reach it.

Because it is an interpreter, timing is not real: an access to a peripheral takes
"one instruction", not the cycles it would on silicon. It runs tens of millions
of instructions per second (a PC-keyed decode cache does most of that), so a full
SP boot to `[sp-emu] online` takes tens of seconds. MGS and humility are given
generous timeouts anyway.

## Building

```
cargo build --release
```

The binary lands at `target/release/sp-emu`. It is an ordinary userland process
(developed on illumos / Helios).

To flash a slot you need a Hubris build archive, for example
`hubris/target/gimlet-c/dist/default/build-gimlet-c-image-default.zip`. To drive
and inspect the firmware you also want `faux-mgs` (from
management-gateway-service) and `humility`.

## Try it out

Program the gimlet SP image into slot A, boot it, and talk to it with MGS.

```
# 1. program slot A from a Hubris archive
sp-emu flash a ~/oxide/hubris/target/gimlet-c/dist/default/build-gimlet-c-image-default.zip

# 2. boot it, binding MGS on [::1]:33310 (switch0) and [::1]:33311 (switch1)
SP_EMU_BRIDGE='[::1]:33310' sp-emu gdb a 340000000
```

Wait about a minute for the network stack to come up, then point faux-mgs at it
from another shell:

```
faux-mgs --sp-sim-addr '[::1]:33310' state
faux-mgs --sp-sim-addr '[::1]:33310' inventory
faux-mgs --sp-sim-addr '[::1]:33310' read-sensor-value 0
```

Or attach humility to read and drive the live firmware, over the **SWD debug
port**: a real halt/run/step debug core exposed as a Glasgow Interface Explorer
probe, which a stock humility drives via probe-rs. Because it really halts and
runs the core, halt-and-run commands like `hiffy` work over it, not just memory
reads. The SWD port is `4444 + (bridge_port - 33300)`, so for `[::1]:33310` it is
4454:

```
humility -a <gimlet-archive.zip> -p 20b7:9db1:tcp:127.0.0.1:4454 tasks
humility -a <gimlet-archive.zip> -p 20b7:9db1:tcp:127.0.0.1:4454 hiffy -c Jefe.get_state
```

sp-emu prints the exact `swd` port and the attach line on startup (the
`ready (swd :...)` line), so you do not have to do the port arithmetic by hand.
(Older `-p ocdgdb`/`-p ocd` transports were removed once humility dropped those
probe backends; the SWD/Glasgow probe is the one a stock humility drives.)

The `demo/` directory wraps all of this in scripts:

- `demo/run-sp.sh` boots a gimlet SP and waits until it is reachable; `demo/mgs`
  and `demo/tasks` talk to it.
- `demo/run-sp-rot.sh` boots a gimlet SP together with an emulated RoT over the
  sprot link, so `mgs state` returns real RoT boot-info instead of `rot: Err(...)`.
- `demo/run-fleet.sh` brings up several SPs.
- `demo/i2c-sniff.sh` streams every I2C transaction the firmware makes.
- `demo/i2c-device.sh` answers as the SP's I2C devices, so you can inject a value
  (a temperature, say) and read it back through Hubris over MGS.

See `demo/README.md` for the walk-through.

## Commands

```
sp-emu flash <a|b> <image.bin | build-archive.zip>   program a flash slot
sp-emu erase <a|b>                                   erase a slot
sp-emu info                                          show each slot's reset vector
sp-emu run [a|b] [max_insns]                         boot from a slot and run (max_insns 0 = run forever)
sp-emu gdb [a|b] [preboot]                           legacy alias for `run <slot> 0` (serve SWD + MGS)
sp-emu rot <oxide-rot-1 image> [max]                 boot the LPC55 RoT firmware standalone
sp-emu rot-serve <listen-addr> <rot-image>           run a shared RoT for SPs to connect to
sp-emu pack [bundle.zip]                             bundle this instance (flash + archives) into one portable zip
sp-emu unpack <bundle.zip> [dir]                     extract an instance bundle, ready to run and inspect with humility
sp-emu i2c-sniff [listen-addr]                        observe I2C traffic from a running emulator
sp-emu i2c-device [addr] [spec ...]                   stand in as I2C devices for a running emulator
```

Global flags may appear anywhere on the line: `--seed <hex|string>` (see Instance
identity), and `--load-config <path>` / `--dump-config <path>` (see Configuration).

### How long `run` runs

`run` takes an optional instruction cap, and which you want depends on the task:

- A finite `max_insns` (default 5,000,000) boots to steady state and then stops.
  Use this for a quick smoke boot, an instruction trace, or CI, where a bounded,
  deterministic run is what you want.
- `max_insns` of `0` runs until the firmware traps or halts, i.e. indefinitely.
  Choose this when the SP must keep serving: answering MGS over the bridge, or a
  firmware-update test that must stay up across the reset that activates the new
  image (see below). This is the mode to reach for when you want a long-running
  emulated SP on the network.

### Firmware update and reboot into a new image

A firmware update is driven entirely by the real Hubris firmware over MGS; the
emulator just models the flash. The STM32H7 FLASH controller and the two banks
are modeled with real unlock/erase/program semantics, the option-byte bank swap,
and persistence, so an in-band MGS update programs the inactive bank, and the
option-byte swap plus the SP reset that follows reboots into the newly written
image, exactly as on silicon. Flash contents and the persisted swap survive
across runs (`$SP_EMU_FLASH` plus a small `.nv` state file beside it, which also
records the Hubris archive the slot was flashed from).

To exercise an update end to end the SP must run across that reset, so run it in
the run-forever mode (`run <slot> 0`) with the MGS bridge bound.

### Instance archives and portable bundles

Flash from a full **Hubris build archive**, not a bare `.bin`. sp-emu now relies on
archive content both to initialize the instance (the SP's Ethernet ports come from
the image's `app.toml`) and for humility tooling (`ringbuf`/`hiffy` verify the
archive's image id against the running image). Flashing a bare image still works but
prints a warning that a flash image alone is inadequate; set `SP_EMU_NO_ARCHIVE_WARN`
to silence it.

When you flash from an archive, sp-emu copies it into an `archives/` directory beside
the flash image and records the reference in the `.nv` companion file, so a
run-from-flash instance keeps its archive without re-supplying it. The SP flash, the
RoT flash, and their archives all anchor to one instance base (the SP flash's
directory), so the two cores travel together.

`sp-emu pack [bundle.zip]` bundles the whole instance (the flash images, their `.nv`
files, identity, a bundle-relative `config.toml`, the stowed Hubris archives, and a
`manifest.toml`) into a single portable zip. `sp-emu unpack <bundle.zip> [dir]`
extracts it; the unpacked instance re-runs with `sp-emu --load-config config.toml run
a 0` and is humility-attachable (`humility -a archives/<component>.zip ...`) without
the original archives. `pack` captures the `SP_EMU_*` knobs present in its own
environment, so pack with the same environment you run with.

## Running a testbed (SP + RoT + bootleby, SWD, IPCC)

`demo/run-testbed.sh` brings up the two-core instance with the MGS bridge, a SWD
probe per core over TCP, and the IPCC pty, and prints every endpoint. See
[demo/README.md](demo/README.md) for the walkthrough, the faux-mgs
considerations (loopback needs `--sp-sim-addr`), and where persistent state
lives.

## Environment variables

The ones you reach for most:

- `SP_EMU_STATE_DIR`: directory for this instance's persistent state (the flash
  images, their `.nv` companion files, the derived identity, and the stowed Hubris
  archives). When unset, sp-emu uses a per-user default under `$XDG_STATE_HOME` or
  `~/.local/state/sp-emu` and prints a warning, so a bare run never writes into the
  working directory. Give each instance in a fleet its own.
- `SP_EMU_FLASH`: path to the NVM (flash) image file. Defaults to `sp-flash.bin`
  under `SP_EMU_STATE_DIR`; set it to place the flash somewhere specific.
- `SP_EMU_BOARD`: `gimlet` (default) or `sidecar`. Selects the SoC model and identity.
- `SP_EMU_BRIDGE`: loopback address for the MGS UDP surface, for example
  `[::1]:33310`. The two switch ports are this one and the next.
- `SP_EMU_ROT_SERVICE`: address of a `rot-serve` RoT to attach over the sprot link
  (the shared, out-of-process RoT).
- `SP_EMU_ROT_FLASH`: instead of a service, run an in-process RoT core from this image.
  Unlike `SP_EMU_ROT_SERVICE`, this RoT drives the SP's debug port over an internal SWD
  link, so it can run the endoscope attestation measurement (see `demo/run-sp-measure.sh`).
- `SP_EMU_ROT_NVM`: path to the RoT flash backing file (defaults to `sp-rot-flash.bin`
  under `SP_EMU_STATE_DIR`), the RoT analog of `SP_EMU_FLASH`. A persisted file takes precedence over the image
  passed on the command line, so delete it to reseed (or set `SP_EMU_ROT_FRESH`); give
  each instance its own. A persisted file that would shadow the protected-flash
  overrides below warns rather than silently ignoring them.
- `SP_EMU_SWD_TRIGGER`: fire one synthetic SP-reset measurement request after boot, so the
  in-process RoT measures the SP even when the SP image does not gate its boot on the token.
- `SP_EMU_ROT_MEASURE`: let the SP self-reset until measured (drop the pre-seeded SKIP token)
  rather than short-circuiting the RFD 568 handoff.
- `SP_EMU_ROT_ROM`: emulate the LPC55 boot-ROM signature API (`skboot_authenticate`), so
  the RoT pre-kernel's image authentication runs the real secure-boot check, the cert
  chain to the CMPA RKTH via the host verifier (`lpc55_sign`), instead of being skipped.
  Off by default.
- `SP_EMU_ROT_FRESH`: ignore any persisted RoT flash and re-seed it from scratch this run,
  so there is no doubt about whether persistent state is in use.
- `SP_EMU_HOST_UART`: socket for the host-to-SP comms UART (IPCC).
- `SP_EMU_NO_DEBUG`: suppress the SWD debug listener (serve the MGS bridge only).
- `SP_EMU_SPROT_COUPLE`: while the SP is blocked on an in-flight sprot request, pace
  its SysTick by the RoT's elapsed ticks instead of the emulator's idle throttle, so
  the SP's sprot timeout tracks the RoT's real work (both kernels tick at 1 ms). On by
  default; set it to `0` to restore the old idle-throttle timing.
- `SP_EMU_NO_ARCHIVE_WARN`: silence the warning printed when the instance has no Hubris
  archive (flashed from a bare image); see "Instance archives and portable bundles".
- `SP_EMU_I2C_BRIDGE` / `SP_EMU_I2C_DEVICE`: socket for the I2C sniff and delegate bridges.
- `SP_EMU_IDENTITY`: path to the per-instance identity file (defaults to
  `sp-emu-identity` under `SP_EMU_STATE_DIR`). Give each instance in a fleet its own,
  like `SP_EMU_FLASH`.
- `SP_EMU_VPD_SERIAL` / `SP_EMU_VPD_PART` / `SP_EMU_VPD_REV`: VPD/FRUID barcode the
  SP reports as serial / model / revision. The defaults are Oxide-style and read as
  real hardware in inventory; override them so an emulated SP is not mistaken for a
  shipped one. Serial and part are capped at 11 characters by the barcode format.
- `SP_EMU_SEED`: same as the `--seed` flag below.

There are also `SP_EMU_*DBG` switches (`SP_EMU_SPROTDBG`, `SP_EMU_ETHDBG`,
`SP_EMU_SPIDBG`, `SP_EMU_FLASHDBG`, `SP_EMU_ROMDBG`, `SP_EMU_COUPLEDBG`, and so on)
that turn on per-subsystem tracing. `SP_EMU_FLASHDBG` traces the FLASH controller: unlock,
erase, program, and the option-byte bank swap; `SP_EMU_ROMDBG` traces boot-ROM
API calls (each `skboot_authenticate` and its verdict).

sp-emu boots the RoT through real bootleby by default, for genuine A/B (and
panic) image selection and to honor the CFPA's persistent boot preference: it
looks for `bootleby-oxide-rot-1.zip` next to the RoT archive, then under
`$HUBRIS`. `SP_EMU_ROT_BOOTLEBY=<image>` names one explicitly and
`SP_EMU_ROT_NO_BOOTLEBY=1` opts out. `SP_EMU_ROT_CMPA` / `SP_EMU_ROT_CFPA` /
`SP_EMU_ROT_NMPA` replace the seeded protected-flash pages, and
`SP_EMU_ROT_ROM=1` enables the boot-ROM `skboot_authenticate` shim.
bootleby verifies each slot's signature against the CMPA, selects and jumps, and the
real `lpc55-rot-startup` then rebuilds the boot-state handoff, so `rot-boot-info`
over MGS reports the actual active slot and real sha3-256 digests. Use self-signed
(dice-self) RoT images: they are secure-boot-signed so bootleby verifies them, and
take the PUF DICE path so they boot past the manufacturing USART step.

## Configuration

sp-emu takes its `SP_EMU_*` settings from the environment; or, with
`--load-config`, from a saved config file *instead of* the environment. The two are
alternatives, never mixed, and command-line flags always win, so precedence is
flag > (config file | environment) > default. Everything is read and vetted once at
startup; nothing consults the environment after that.

- `--load-config <path>`: read all `SP_EMU_*` settings from a TOML file (a flat table
  keyed by the variable names) and ignore the environment, so a saved run reproduces
  exactly regardless of your shell. Flags still override.
- `--dump-config <path>`: write the effective configuration as TOML, so a run can be
  captured and replayed with `--load-config`.
- `SP_EMU_CONFIGDBG`: print the full resolved configuration to stderr. It is an
  environment variable, so it has no effect under `--load-config`; use
  `--dump-config` (a flag) to inspect a loaded configuration.

## Instance identity (`--seed`)

Each instance has its own identity (the SP UID, the RoT device UUID, and the
RoT's DICE identity / self-signed certificate), so a fleet of emulators is
discoverable like real hardware. All of it derives from one seed:

- `--seed <source>` (or `$SP_EMU_SEED`): the seed source. It is one of
  - `legacy`: reproduces the previous fixed constants exactly (same UID, DICE
    CDI, and PUF seed, hence the same self-signed cert), for compatibility;
  - a `0x`-prefixed hex `u64`, e.g. `--seed 0x1234` (malformed or over-64-bit
    hex is an error);
  - any other string, which is hashed.
- With no `--seed`, the identity file is used if present, else a fresh random
  seed is minted and persisted, so each instance is unique yet stable across
  runs.

The SP's Ethernet MAC and serial come from its emulated VPD EEPROM
(`build_vpd_eeprom`), which is already varied per instance by the `SP_EMU_BRIDGE`
port index; folding the VPD identity into this seed is a follow-up.

Note: sp-emu's root of trust is deliberately an open book. The seed (which
derives the DICE/PUF secrets) is persisted in plaintext and logged on purpose --
there is no real secret to protect, and reproducibility wins. See the "Secrets
policy exception" note in `src/identity.rs`.

## Status and limits

- The SP path (gimlet-c and sidecar) and the combined SP-and-RoT sprot path both
  boot and serve MGS.
- The images are unmodified production builds, so `faux-mgs` and `humility` must be
  built from the `gateway-messages` and Hubris revisions the image was compiled
  against; the wire protocol has to match the firmware.
- It is an instruction interpreter, not a cycle-accurate model. Timing is not real,
  and only the peripherals the SP and RoT firmware actually touch are modeled. An
  access to an unmodeled register is logged, which is how we decide what to add next.
