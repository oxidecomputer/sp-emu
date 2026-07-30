# hwci/

sp-emu's own [`sp-test`](https://github.com/oxidecomputer/sp-test) tests,
focused on the emulated LPC55S69 RoT boot path (bootleby A/B/panic slot
selection, transient boot-preference override, and RoT firmware / stage0
update). It mirrors the layout of `sp-test/hwci/` and is meant to be run as an
out-of-tree test root:

```
hwci/
├── README.md               # this file
├── testbed-sp-emu.toml     # testbed config pointing sp-test at a running sp-emu
├── suites/                 # named sets of tests
│   └── suite-rot-boot.toml
└── tests/                  # one directory per test
    └── rot-boot-state/     # read RoT boot state (active slot + FWIDs) over MGS
        ├── test-info.toml
        └── check-boot-state.sh
```

Tests are bash scripts driving `faux-mgs`/`humility` through the sp-test helper
library (`lib/sp-test.sh`, provided by the sp-test checkout via `SP_TEST_LIB`),
gated by `required_capabilities` so they SKIP rather than FAIL where they don't
apply. See `sp-test/hwci/README.md` for the framework walkthrough.

## Running against sp-emu

Start sp-emu in run-forever serve mode with the MGS bridge and an in-process
RoT, then point sp-test at it with this testbed config and the `--test-root`
here.

```bash
# 1. Build sp-emu (RUSTC_BOOTSTRAP=1 is required by the pinned toolchain).
RUSTC_BOOTSTRAP=1 cargo build --release --bin sp-emu

# 2. Flash a gimlet SP image into sp-emu's SP flash (once).
SP_EMU_FLASH=./sp-flash.bin ./target/release/sp-emu flash a <gimlet SP archive>

# 3. Run it forever: SP on slot A, RoT reachable over sprot, MGS on [::1]:33300.
#    `run <slot> 0` is the serve-forever mode; it wires the in-process RoT when
#    SP_EMU_ROT_FLASH is set and exposes a Glasgow SWD probe for BOTH cores:
#    the SP on :4444 and the RoT on :4544. Add SP_EMU_HOST_PTY=1 for the IPCC /
#    host-console channel (a pty for faux-ipcc). See "Ports and interfaces" below.
SP_EMU_FLASH=./sp-flash.bin \
SP_EMU_BRIDGE='[::1]:33300' \
SP_EMU_ROT_FLASH=<oxide-rot-1 self-signed archive.zip> \
SP_EMU_ROT_ROM=1 \
SP_EMU_HOST_PTY=1 \
  ./target/release/sp-emu run a 0 &
#  wait for: [sp-emu] online   (takes ~30s)

# 4. Run the RoT-boot suite. faux-mgs cannot reach sp-emu over loopback with
#    --interface/--discovery-addr ("no interface name found for index 0"); the
#    sp-test-workshop shim rewrites that to --sp-sim-addr. Put it ahead on PATH.
HWCI=~/Oxide/src/sp-emu/hwci
SHIM=~/Oxide/src/sp-test-workshop/stages/1-sp-emu/shim
PATH="$SHIM:$PATH" SP_EMU_SHIM=1 SP_EMU_ATTEMPT_MS=30000 \
sp-test suite run suite-rot-boot \
    --testbed   $HWCI/testbed-sp-emu.toml \
    --archive   <gimlet SP archive> \
    --rot-archive <oxide-rot-1 self-signed archive> \
    --test-root $HWCI/tests \
    -o ~/out/rot-boot
```

Two RoT-image requirements make the emulated RoT boot to its sprot server:

- **Use the SELF-SIGNED archive** (`build-oxide-rot-1-selfsigned-image-*.zip`). It
  is still Bart secure-boot-signed (so `skboot_authenticate` passes against the
  default CMPA) but uses the `dice-self` identity path (PUF, no USART). The
  production `dice-mfg` image wedges in DICE startup polling the unmodeled
  FLEXCOMM0 manufacturing USART. Pass the archive (not a bare image) so
  `humility -a` can attach to the RoT and the instance is self-contained.
- **`SP_EMU_ROT_ROM=1`** is required in serve mode: the RoT pre-kernel's
  `authenticate_image` reaches the boot-ROM `skboot_authenticate`; without the
  ROM shim it branches to an unmapped ROM pointer and faults.

`SP_EMU_ROT_FLASH` gives a real emulated RoT so `rot_state` returns real data
over the SP -> sprot -> RoT relay. For genuine bootleby A/B selection (rather than
the fabricated boot-state handoff), also set `SP_EMU_ROT_BOOTLEBY` plus
`SP_EMU_ROT_CMPA`/`SP_EMU_ROT_CFPA` (see the sp-emu README): bootleby now runs in
the two-core serve mode and reports the actual selected slot over MGS.

## Ports and interfaces (combined SP + RoT setup)

A single `run <slot> 0` instance exposes all of the interfaces below at once, so
one instance covers the MGS, humility-probe, and IPCC surfaces a test may need.
Everything is offset from the bridge port so several instances coexist in a zone:
`off = <bridge port> - 33300`. The table is for `SP_EMU_BRIDGE='[::1]:33300'`
(`off = 0`); startup logs the actual values (`[bridge] ...`, `[gdb] ready ...`,
`[bridge] host-uart ... pty ready`).

| Interface | Address (off 0) | Formula | Notes |
|---|---|---|---|
| MGS / faux-mgs (switch0) | `[::1]:33300` | `bridge + 0` | `faux-mgs --sp-sim-addr` target; discovery_addr in the testbed |
| MGS / faux-mgs (switch1) | `[::1]:33301` | `bridge + 1` | second switch view |
| ereport (switch0 / switch1) | `[::1]:44400` / `:44401` | `44400 + off` | `faux-mgs ereports` |
| SP SWD probe | `127.0.0.1:4444` | `4444 + off` | `humility -a <sp.zip> -p 20b7:9db1:tcp:127.0.0.1:4444` |
| RoT SWD probe | `127.0.0.1:4544` | `4544 + off` | `humility -a <rot.zip> -p 20b7:9db1:tcp:127.0.0.1:4544` |
| IPCC / host console (UART7) | pty or unix socket | n/a | see below |

Notes for planning a run that uses everything:

- **Both debug probes are live on the same run.** The SP probe drives the SP debug
  port; attaching to it asserts SP_TO_ROT_JTAG_DETECT_L, so the RoT invalidates its
  attestation log (as on hardware). The RoT probe is the RoT's own debug port;
  attaching halts the RoT with no such side effect. In the testbed these give
  `testbed:probe:sp` and `testbed:probe:rot`; a humility session on either freezes
  the whole instance for its duration, so don't hold one open while faux-mgs runs.
- **Ethernet.** MGS and ereport are UDP on loopback. faux-mgs on loopback needs
  `--sp-sim-addr <MGS addr>` (the `--interface`/`--discovery-addr` form fails with
  "no interface name found for index 0"); the sp-test shim rewrites that for you.
- **IPCC / host console.** The host-sp-comms link (UART7) is off by default. Set
  `SP_EMU_HOST_PTY=1` to expose it as a pty for serial tools
  (`faux-ipcc --port <pty> ...`; the pty path is logged at startup), or
  `SP_EMU_HOST_UART=<unix socket>` for a voxel/propolis IPCC COM port. This is the
  channel `faux-ipcc get-certs` and the host console use.

## Status and roadmap

Real bootleby boots the emulated RoT end-to-end in both standalone `sp-emu rot`
mode and the two-core serve mode (genuine `skboot_authenticate` + CDI measurement
via HASHCRYPT + `boot_into`), and all four A/B/panic selection outcomes are
validated via slot-seeding knobs (`SP_EMU_ROT_IMAGE_B`, `SP_EMU_ROT_ERASE_A`,
`SP_EMU_ROT_BOOT_PREF`). The sp-test tests here wrap that as MGS-driven regressions:

- `rot-boot-state`: read the RoT active slot + FWIDs over MGS. Passes today
  end-to-end against sp-emu (sp-test -> faux-mgs shim -> SP -> sprot -> RoT).
- `rot-ab-selection` (planned): assert the booted slot for A-only / B-only /
  both+preference / neither(panic) over MGS, using serve-mode bootleby.
- `rot-transient-override` (planned): set the transient RAM preference, reset,
  confirm it boots the transient slot once, then falls back to the persistent
  slot. Needs the serve-mode RoT-reset re-run.
- `rot-fw-update` / `rot-stage0-update` (planned): `builtin:update` against
  `--rot-archive` / `--rot-bootloader-archive`, then confirm the new slot boots
  after reset. The update itself completes via faux-mgs with generous retry/timeout
  budgets; driving it through the sp-test harness needs its gateway retry budget
  relaxed for the slow sim.
