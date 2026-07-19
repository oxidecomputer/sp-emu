# Plan: archive-driven SP I2C emulation (sidecar bringup + transceivers)

Status: in progress on branch `sp-swd-i2c-emulation` (off `sp-swd`).
Done so far: the I2C topology parser (`src/i2c_topology.rs`, commit `479836b`).

## Goal

Give sp-emu a faithful, **archive-driven** SP I2C model so that:

1. The **sidecar** image boots to network-online (today it fault-loops in the
   sequencer), and
2. Removable hardware (front IO board, QSFP transceivers, DIMMs, ...) is a
   **configurable population**, not a blunt removal -- a bare board boots, and a
   populated board simulates the real modules. The QSFP **transceiver** emulation
   is a first-class target (network engineers want it).

Everything about the I2C topology comes from the flashed image's `app.toml`; the
only user configuration is *which removables are populated*.

## Why the sidecar doesn't boot today (root cause, verified)

Booting a sidecar image (`SP_EMU_BOARD=sidecar`) with a sidecar archive: the
kernel comes up and the mainboard FPGA IDENT/checksum pass (sp-emu already models
these via `Spi5`), but the `sequencer` task (`drv-sidecar-seq-server`)
fault-loops (~1/sec, gen 20-30+). Everything downstream (`net`, `monorail`,
`power`, `ignition`, `transceivers`) blocks `wait: send to sequencer`, so the SP
never transmits and never comes online.

Exact fault (via `SP_EMU_PANICDBG=1`):
`drv/sidecar-seq-server/src/main.rs:1087` -> `front_io_board.init().unwrap_lite()`
("explicit panic"). The chain:

- `front_io_board_preinit()` ends with `Ok(FrontIOBoard::present(i2c))`, which
  validates the front-IO FRUID EEPROM (`At24Csw080::validate` -> a security-
  register read).
- sp-emu's I2C model **always ACKs** (ISR reads TC, never NACKF), so the probe
  "succeeds" and the sequencer thinks a front IO board is present.
- It then calls `FrontIOBoard::init()`, which talks to the front-IO ECP5 FPGA
  (two controllers) -- unmodeled -- so `init()` fails and `unwrap_lite()` panics.

Next fault after that one (already observed): `task/power/src/bsp/sidecar_bcd.rs:45`
`"bad state"` -- `sequencer.tofino_seq_state()` must return `A0`/`A2`, which needs
modeling the mainboard FPGA's Tofino sequencer state machine.

Diagnosis tools that work: `SP_EMU_PANICDBG=1` (logs task panics + backtrace),
`humility -p 20b7:9db1:tcp:127.0.0.1:4454 tasks|ringbuf ...` over the SWD port.

## What the archive gives us (verified against the v1.76.0 sidecar-c image)

`app.toml [config.i2c]` fully documents the SP-direct I2C hardware:

- **controllers** (1-4), each with **ports** (named buses) and their GPIO pins.
- **muxes**: `muxes = [ { driver = "pca9545", address = 0x70 } ]` per port.
- **devices** (~40 on the sidecar): `bus = "<port>"`, optional `mux = N` +
  `segment = M` (behind a pca9545), `address`, `device = "<type>"`, and a
  **`removable`** flag (18 removable / 22 fixed on the sidecar).

Device location key = (controller, mux segment, address). Example: `local_vpd`
is `south2` (controller 4, no mux); the front-IO FRUID is `front_io`
(controller 2); transceiver FRUIDs are behind pca9545 muxes on controllers 1/3.
All AT24CSW080 share address `0x50`, so they are distinguished only by
controller + segment -- which is why the model must track mux segments.

Not in the SP-direct I2C: the **32 QSFP transceivers**. They have no `bus`/
`address`; they sit behind the front-IO ECP5 FPGA and are reached via the
transceivers task -> FPGA -> per-port module I2C. They are Stage 2.

## Design

### Stage 1 -- archive-driven SP I2C model + configurable population

1. **Topology (done):** `i2c_topology::I2cTopology::from_app_toml` parses the
   controller/port/mux/device inventory, with `device_at(controller, segment,
   address)`.
2. **Feed the topology to the `I2c` model.** In `soc.rs`, the four `I2c`
   controllers are built in the install loop (~soc.rs:102). Parse the flashed
   image's app.toml once (via `$SP_EMU_ARCHIVE`, same source the well-known-port
   work uses) and hand each `I2c` its controller number + the shared topology +
   the population config.
3. **Model the pca9545 mux.** The mux is an I2C device at (e.g.) `0x70` whose
   single control byte selects channels (bit per segment). When the driver writes
   it, record the active segment for that controller. Track it in the `I2c` state.
4. **Answer presence per (controller, segment, address).** Replace the blanket
   ACK with: look up `topology.device_at(controller, active_segment, address)`.
   - Device present and populated -> ACK; serve its registers (reuse the existing
     `device_reg` models keyed by `device.kind`: tmp117, at24csw080, etc.).
   - No device, or an **unpopulated removable** -> **NAK** (set ISR.NACKF) so the
     driver's probe fails and the firmware treats it as absent.
5. **Population config** (env now; fold into the fleet manifest later):
   - Fixed devices: always present.
   - Removable devices: absent by default (bare board boots), or populated per
     config, e.g. `SP_EMU_POPULATE=front_io,xcvr:0-7` (front IO board + QSFP
     ports 0-7). Keep it simple and documented.
   With front IO unpopulated, `FrontIOBoard::present()` reads a NAK -> returns
   false -> the sequencer skips front-IO init -> past the `main.rs:1087` panic.
6. **Then the Tofino state:** model the mainboard FPGA's `TOFINO_SEQ_STATE`
   (in `Spi5`'s register map) to report a coherent powered state (`A2`) so
   `power`/`net` get a valid `tofino_seq_state()` and stop panicking. Iterate the
   remaining fault chain (fan modules, power rails) the same way -- boot, read the
   panic via `SP_EMU_PANICDBG`, model the missing register/device -- until the SP
   comes online. This mirrors the original gimlet bringup.

Success: sidecar boots to `[sp-emu] online`, `faux-mgs state` works, ereports
work (the RNG fix is on the well-known-ports branch; the two lines will need to
be combined for a fully working sidecar in the workshop).

### Stage 2 -- front-IO ECP5 FPGA + QSFP transceiver emulation

The transceivers task drives the **front-IO ECP5** (a second FPGA, on a different
SPI than the mainboard's `Spi5`) for per-port QSFP presence/status and module I2C.
Model it the same way the mainboard FPGA is modeled:

- A front-IO FPGA SPI device (ident/checksum/ready so `FrontIOBoard::init()`
  succeeds when front IO is populated) -- `Spi5` (mainboard ECP5) is the template.
- The FPGA's QSFP register interface: per-port module presence, LOS/status, and
  the module I2C bridge, driven from the population config (which ports have
  modules, and what the modules report).

This is the network-engineer feature: a populated front IO board with a
configurable set of QSFP modules the transceivers task can enumerate and manage.

## Integration points in sp-emu

- `src/soc.rs`: the `I2c` model (`device_reg`, the ISR ACK at off `0x18`, the
  CR2/START address latch) and the controller install loop (~line 102). `Spi5`
  (~line 809, mainboard ECP5) is the template for the front-IO FPGA.
- `src/i2c_topology.rs`: the parser (done); extend with the population config.
- `src/flash.rs`: `archive_app_toml` (added on the well-known-ports branch) is the
  app.toml source; this branch will need its own copy or a rebase/merge.
- Archive path: `$SP_EMU_ARCHIVE` (same convention as the well-known-port work).

## Relationship to the well-known-ports branch

The ereport RNG fix and `archive_app_toml`/`toml` dependency live on
`sp-swd-well-known-ports`. This branch adds its own `toml` dependency and will
need `archive_app_toml` (copy it, or rebase once both land). A fully working
sidecar in the workshop needs both lines combined (well-known ports for
reachability + this branch for the sidecar hardware model).
