# sp-emu

A native-Rust emulator that boots real, unmodified Oxide Hubris firmware for a
service processor (SP) and root of trust (RoT), with no hardware or RTOS
underneath. It models enough of the STM32H7 (the SP) and LPC55 (the RoT) that the
production firmware images come up on their own, bring up their networks, and
answer the management gateway (MGS) over UDP the way a real board does.

## What it can do

- Boots the production gimlet-c and sidecar SP images on an emulated STM32H753
  (Cortex-M7), from the reset vector through the kernel and 30-plus Hubris tasks.
- Boots the oxide-rot-1 RoT image on an emulated LPC55 (Cortex-M33).
- Answers MGS over UDP on loopback, on both switch uplink ports, the same way real
  hardware does: `discover`, `state`, `inventory`, `read-sensor-value`,
  `power-state`, `rot-boot-info`, caboose reads, dumps, and the rest of the
  faux-mgs surface.
- Lets `humility` attach to the running firmware over a GDB-RSP or OpenOCD
  listener: live task table, per-task stack backtraces, `readmem`, ringbufs.
- Runs the SP and RoT together over an emulated sprot SPI link, so the real
  `drv-stm32h7-sprot-server` on the SP talks to the real `drv-lpc55-sprot-server`
  on the RoT. The RoT publishes genuine boot-state measurements (sha3-256 of the
  flashed image), so `rot-boot-info` returns real digests instead of zeros.
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

Because it is an interpreter, it runs at a few million instructions per second, so
a full SP boot takes roughly 60 to 90 seconds. MGS and humility are given generous
timeouts to match.

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

Or attach humility to read the live Hubris task table. The GDB-RSP port is
`3333 + (bridge_port - 33300)`, so for `[::1]:33310` it is 3343:

```
HUMILITY_OCD_PORT=3343 humility -a <gimlet-archive.zip> -p ocdgdb tasks
```

The `demo/` directory wraps all of this in scripts:

- `demo/run-sp.sh` boots a gimlet SP and waits until it is reachable; `demo/mgs`
  and `demo/tasks` talk to it.
- `demo/run-fleet.sh` brings up several SPs plus a shared RoT over the sprot link.
- `demo/i2c-sniff.sh` streams every I2C transaction the firmware makes.
- `demo/i2c-device.sh` answers as the SP's I2C devices, so you can inject a value
  (a temperature, say) and read it back through Hubris over MGS.

See `demo/README.md` for the walk-through.

## Commands

```
sp-emu flash <a|b> <image.bin | build-archive.zip>   program a flash slot
sp-emu erase <a|b>                                   erase a slot
sp-emu info                                          show each slot's reset vector
sp-emu run [a|b] [max_insns]                         boot from a slot and run
sp-emu gdb [a|b] [preboot]                           boot a slot, then serve GDB/OpenOCD for humility
sp-emu rot <oxide-rot-1 image> [max]                 boot the LPC55 RoT firmware standalone
sp-emu rot-serve <listen-addr> <rot-image>           run a shared RoT for SPs to connect to
sp-emu i2c-sniff [listen-addr]                        observe I2C traffic from a running emulator
sp-emu i2c-device [addr] [spec ...]                   stand in as I2C devices for a running emulator
```

## Environment variables

The ones you reach for most:

- `SP_EMU_FLASH`: path to the NVM (flash) image file. Defaults to `sp-flash.bin`
  in the working directory.
- `SP_EMU_BOARD`: `gimlet` (default) or `sidecar`. Selects the SoC model and identity.
- `SP_EMU_BRIDGE`: loopback address for the MGS UDP surface, for example
  `[::1]:33310`. The two switch ports are this one and the next.
- `SP_EMU_ROT_SERVICE`: address of a `rot-serve` RoT to attach over the sprot link
  (the shared, out-of-process RoT).
- `SP_EMU_ROT_FLASH`: instead of a service, run an in-process RoT core from this image.
- `SP_EMU_HOST_UART`: socket for the host-to-SP comms UART (IPCC).
- `SP_EMU_NO_DEBUG`: suppress the humility debug listeners.
- `SP_EMU_I2C_BRIDGE` / `SP_EMU_I2C_DEVICE`: socket for the I2C sniff and delegate bridges.

There are also `SP_EMU_*DBG` switches (`SP_EMU_SPROTDBG`, `SP_EMU_ETHDBG`,
`SP_EMU_SPIDBG`, and so on) that turn on per-subsystem tracing.

## Status and limits

- The SP path (gimlet-c and sidecar) and the combined SP-and-RoT sprot path both
  boot and serve MGS.
- The images are unmodified production builds, so `faux-mgs` and `humility` must be
  built from the `gateway-messages` and Hubris revisions the image was compiled
  against; the wire protocol has to match the firmware.
- It is an instruction interpreter, not a cycle-accurate model. Timing is not real,
  and only the peripherals the SP and RoT firmware actually touch are modeled. An
  access to an unmodeled register is logged, which is how we decide what to add next.
