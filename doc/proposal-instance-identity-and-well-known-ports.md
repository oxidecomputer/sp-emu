# Proposal: per-instance IPv6 identity and well-known SP ports

Status: draft for review by the sp-emu author.
Context: surfaced by the sp-test workshop

## Summary

Give each emulated SP its own IPv6 address (two, in fact — one per management
uplink) and expose the firmware's UDP sockets on their real, well-known port
numbers. A developer, faux-mgs, humility, and sp-test then reach the emulated SP
at exactly the addresses and ports they would use against real hardware —
`<sp-addr>:11111` for MGS, `:11115` for hiffy, `:998` for udprpc, `:11113` for
dump_agent, and so on — with no per-tool port arithmetic.

This is what unlocks humility's network transports (NetHiffy / NetUdpRpc) and
`dump_agent` against sp-emu, without emulating a debug probe or SWD. It replaces
the current scheme, where the bridge relays only two sockets and encodes each
SP's identity in its bind *port*.

Because moving to well-known ports removes the port as the per-instance
differentiator, the proposal also introduces an explicit instance identity: a
JSON fleet manifest, selected by `--index N` or `--name NAME`, that carries each
SP's board, addresses, base MAC, serial, and VLANs.

The workshop (a Docker/Podman container) is the driving consumer; voxel is the
other. The two want opposite addressing models — the workshop wants well-known
ports, voxel deliberately port-multiplexes a fleet onto one loopback to mirror
`sp-sim` — so the new path is strictly additive and off by default, and voxel is
undisturbed. See "Compatibility with voxel."

## Motivation

The workshop drove sp-test against sp-emu over the `simulator = true` loopback
MGS path — that works. What does not work is anything humility does over the
network:

- sp-emu's bridge (`src/bridge.rs`) relays only two SP sockets:
  `control_plane_agent` (11111) and the ereport snitch (57005). The firmware's
  other sockets — `hiffy`, `udprpc`/`rpc`, `dump_agent`, `echo`, `broadcast`,
  `inspector`, `transceivers` — are not reachable from the host, so any
  humility-over-network operation gets ECONNREFUSED.
- The debug transports sp-emu does offer — GDB-RSP (`humility -p ocdgdb`) and
  OpenOCD Tcl (`humility -p ocd`), both in `src/gdb.rs` — are network stubs for
  humility probe backends that humility removed on master (commit `e85b5d8e`,
  "Remove gdb support"). Current humility rejects `ocdgdb`/`ocd` and reaches a
  live target only through probe-rs over USB. There is no network transport left
  for those stubs to serve.

Humility's NetHiffy backend is the way out for the operations sp-test actually
needs (Idol calls: fault injection, `Jefe` task restart, `Jefe.read_fault_counts`).
It sends a hiffy RPC straight to an SP UDP socket — no probe, no gdb, no SWD. For
it to reach the emulated SP, three things must line up:

1. humility has the NetHiffy backend — yes, in the pre-removal window
   (v0.12.17, `cd161f63`).
2. the firmware image exposes a `hiffy`/`udprpc` socket — board- and
   profile-dependent (see the socket table below; gated on hubris#2466).
3. sp-emu bridges that socket — today it does not.

This proposal addresses (3), and does it in a way that also makes `dump_agent`
(task dumps), `udprpc` (NetUdpRpc fallback), and the rest reachable — by giving
the emulated SP the same address/port surface as real hardware.

What network hiffy does *not* cover: non-Idol memory access — raw `readmem`/
`writemem`, ringbuf dumps. Those still want a probe/ocd path or the already
-working `humility hydrate` snapshot (`Bus::write_hydrate_dump`, `src/mem.rs`).
So this shrinks the debug-transport gap; it does not eliminate the need for a
read path. It also runs target code, so it perturbs the SP (an observer effect
sp-test's own docs call out) — fine for functional tests, relevant for timing
ones.

## How it works today

Two mechanisms are entangled in the bridge's bind port.

Addressing (`src/bridge.rs`): `SP_EMU_BRIDGE` is a `host:port` bind address. The
bridge binds `base+0` and `base+1` for the two switch views (VLANs), and
`base+11100` for ereport, and injects tagged frames into the SP's `net` stack.
The SP already has its own MAC + link-local per VLAN, learned from its tagged TX
(`sp_by_vid`), and the bridge answers NDP for it. So the SP's two-address reality
is already modeled on the emulated link; only the host-facing bind is flat.

Identity (`src/soc.rs`, `build_vpd_eeprom`): the per-instance index is derived
from the same bind port:

```rust
let idx = (bridge_port - 33300) / 10;   // 33300 -> 0, 33310 -> 1, ...
let mac_last = 0x20 + idx;              // VPD MAC0 base_mac last byte
let serial   = format!("BRM4422000{}", idx);
```

VPD *is* emulated: `build_vpd_eeprom` writes a FRU0/`MAC0`
(`MacAddressBlock` = base_mac[6] + count(128) + stride(1)) + `BARC` barcode into
the AT24CSW080 model, and `net` derives its per-port MACs — and therefore its
link-local IPv6 addresses — from that block. The identity is real; it is just
keyed off the port.

So a developer runs a fleet today by varying the port (`demo/run-fleet.sh`,
`demo/run-sp.sh` `SP_BASE`): sidecar0 at 33300, gimlet0 at 33310, gimlet1 at
33320. The port picks the switch-view host ports, the ereport port, the gdb/ocd
debug ports, and the VPD MAC/serial — all at once.

## The problem this creates for well-known ports

If every instance binds the same well-known ports (`11111`, `11115`, ...), the
port can no longer encode which SP it is. The `idx` derivation collapses to 0,
every instance gets the same base MAC and serial, and we resurrect the L2
collision the current scheme exists to prevent (`src/soc.rs` comment: "a shared
MAC ... caused L2 collisions => intermittent 'no answer'"). Well-known ports and
per-instance identity cannot both come from the port. One of them has to move.

(Note: sidecars are already undifferentiated today — `build_vpd_eeprom` hardcodes
sidecar MAC/serial regardless of `idx`. It only doesn't bite because a4x2 runs a
single sidecar. An explicit identity fixes that latent assumption too.)

## Proposal

### 1. Per-instance distinction moves from port to IPv6 address

Each emulated SP owns two host IPv6 addresses, one per management uplink
(switch0/switch1) — faithful to hardware, where the SP has a distinct link-local
per uplink. On each address the bridge binds the firmware's real socket ports.
Host port always equals SP socket port; instances never collide because they live
on different addresses.

### 2. All archive-declared SP sockets are bridged

The complete set of SP UDP sockets across board configs (union from hubris
`app/*/*.toml`, cross-checked against the v1.76.0 gimlet-c archive; no port
collisions):

| Socket                | Port  | Boards                                   | Purpose                     |
|-----------------------|-------|------------------------------------------|-----------------------------|
| `echo`                | 7     | most                                     | connectivity test           |
| `broadcast`           | 997   | most                                     | net broadcast               |
| `rpc` (udprpc)        | 998   | most                                     | NetUdpRpc (non-Idol RPC)    |
| `control_plane_agent` | 11111 | all SP                                   | MGS (bridged today)         |
| `transceivers`        | 11112 | sidecar, medusa                          | QSFP management             |
| `dump_agent`          | 11113 | most                                     | task dumps                  |
| `fmc_test`            | 11114 | cosmo, grapefruit                        | FMC test                    |
| `hiffy`               | 11115 | cosmo, grapefruit, observer              | NetHiffy                    |
| `inspector`           | 23547 | gimlet                                   | gimlet inspector            |
| `ereport`             | 57005 | most                                     | ereports (bridged today)    |

The bridge binds the sockets a given image actually declares (from the flashed
archive's `[config.net.sockets.*]`), so there are no dead listeners and the
capability signal stays honest — bridging a socket the firmware does not serve
would over-promise, exactly the trap the capability model exists to prevent.

Implementation is small, because the relay is already parameterized by
`sp_port`:

- host -> SP (`poll_host`): each `BoundSock { sock, vid, sp_port }` already
  injects to `sp_ip:sp_port`; widening the socket set is mechanical.
- SP -> host (`handle_udp_tx`): the only hardcoding is one line —
  `if src_port != SP_PORT && src_port != EREPORT_PORT { return; }`. Open it to
  "any bound `sp_port` on this vid" and replies route back via the existing
  `find(s.vid == vid && s.sp_port == src_port)`.

The SP's two link-locals, NDP (Neighbor Solicit/Advert), and MLD/Router-Solicit
absorption are already handled, so unicast delivery to each address works with no
new multicast machinery. `echo`/`broadcast` become reachable simply by being
bound and un-gated.

### 3. Explicit instance identity via a JSON fleet manifest

A manifest describes the possible instances; each entry carries the identity and
topology that is implicit or env-scattered today:

```json
{
  "instances": [
    {
      "name": "sidecar0",
      "index": 0,
      "board": "sidecar",
      "address": ["fdb0::100", "fdb0::101"],
      "base_mac": "0e:1d:b7:fe:45:30",
      "mac_count": 128,
      "serial": "BRM42220001",
      "vids": ["0x130", "0x302"],
      "ignition": "0:gimlet,1:sidecar,2:gimlet,3:gimlet"
    },
    {
      "name": "gimlet0",
      "index": 1,
      "board": "gimlet",
      "address": ["fdb0::110", "fdb0::111"],
      "base_mac": "0e:1d:b7:fe:45:21",
      "mac_count": 128,
      "serial": "BRM44220001",
      "vids": ["0x301", "0x302"]
    }
  ]
}
```

- Selection: `sp-emu run --name gimlet0` or `--index 1`.
- Built-in default compiled into the binary: the a4x2 reference fleet that
  `run-fleet.sh` hardcodes today, so `--name gimlet0` works with no file.
- External override: `--config fleet.json` (or `SP_EMU_CONFIG=...`) replaces the
  built-in.
- The resolved entry drives the bridge addresses, the VPD `MAC0` block and
  barcode, the VLANs, and (for sidecar) the ignition topology — i.e. it replaces
  `SP_EMU_BOARD`, the `SP_EMU_BRIDGE`-port->idx derivation, `SP_EMU_VID0/1`, and
  `SP_EMU_IGNITION`.

### 4. What stays as env / flags

Only identity/topology moves into the manifest. Everything else stays:

- Debug/tracing: every `*DBG`, `SP_EMU_TRACE*`, `SP_EMU_WATCH`, `SP_EMU_DIFF`.
- Deployment/tuning: `SP_EMU_NO_DEBUG`, `SP_EMU_IDLE_MS`, `SP_EMU_ETH_*`,
  `SP_EMU_*STATS`, `SP_EMU_DUMP_*`.
- Runtime file paths (per-deployment, not fleet identity): `--flash` /
  `SP_EMU_FLASH`, the RoT pairing `SP_EMU_ROT_*`, `SP_EMU_HOST_UART`,
  `SP_EMU_I2C_*`. Individual env vars, if set, still override the resolved
  entry's field for one-off tweaks.

## Deployment models

The workshop is the driving consumer, and the base requirement is a Docker/Podman
container. Three models, in recommended order:

1. Single container on the student's laptop (default). One `docker run` /
   `podman run` with the whole toolchain — sp-test plus sp-emu — inside. Runs an
   instance of *both* a sidecar and a gimlet in the same container, which is the
   best demonstration of sp-test's capability-based testing: the same suite runs
   ignition/transceiver tests on the sidecar and skips them on the gimlet, sensor
   coverage differs, and so on. Self-contained, works on any laptop OS because the
   container runs on Linux under the runtime.

2. A "butler" server (fallback + real-hardware act). Hosts many sp-emu instances
   (each its own container or zone) alongside real SPs, exposed as testbeds. It
   rescues a student whose laptop setup breaks, and it is where the same sp-test
   is shown driving emulated and real hardware identically — the capability
   model's payoff.

3. Per-student Nucleo board, build-from-source (opt-in). Highest friction —
   building hubris/humility/sp-test on a student laptop is exactly the setup
   yak-shave the container model exists to avoid. Offer it to the keen; do not
   gate the workshop on it.

The same manifest serves all three unchanged (see Host addressing below): it
always carries identity, and carries `address` only when instances share a
namespace.

## Host addressing and portability

The SP's own link-local addressing does not change. The SP derives its `fe80::`
link-locals from its VPD MAC and speaks them to the bridge inside the frame model,
exactly as today. "Each instance has its own IPv6 address" refers to the *host
rendezvous* plane — the address the bridge binds and a tool dials — which is
already a simulator abstraction (`--sp-sim-addr [::1]:...`), not the SP's real
address. Keep the rendezvous address ULA/global, not link-local, to avoid the
`%scope` zone-id that link-local binds require across OSes.

A container is a network namespace, which is exactly the isolation that makes
well-known ports work with no host-address juggling:

- One container per SP (butler server): each is its own namespace, binds the
  well-known ports on its own container IP with no collision. sp-emu manages no
  addresses; the manifest reduces to identity, and `address` is omitted (bind
  wildcard `[::]:port` inside the namespace).
- Fleet in one container (workshop option 1): the two SPs share one namespace, so
  they need distinct addresses inside it. Trivial here because it is all Linux
  inside the container — the entrypoint adds loopback IPv6 aliases
  (`ip -6 addr add ... dev lo`), one or two per instance, and each sp-emu binds
  the well-known ports on its own address. The manifest carries `address`.

No macvlan/vnic/tap in any of this. The bridge is a userland UDP relay
(`UdpSocket` only); it never creates an L2 interface. macvlan would enter only if
the rendezvous were made the SP's real link-local on a dedicated per-instance
interface — which is Linux/illumos-only and unnecessary, since tools reach the SP
via the sim rendezvous, not a real link-local.

Cross-platform falls out of the container target: Docker Desktop, Podman machine,
and Windows/WSL2 all run the container on a Linux kernel, so sp-emu's Unix APIs
(`std::os::unix::net::UnixStream` for the host UART, `bridge.rs:19`) build and run
everywhere, and the host OS never provisions addresses. Windows students use
WSL2; native Windows stays out of scope. The one container-network config item is
enabling IPv6 on the network (Docker requires it explicitly; Podman/netavark does
it readily) when the rendezvous is over IPv6.

## Compatibility with voxel

voxel is the other consumer of sp-emu and must not be disturbed. Its model is the
opposite of the workshop's, so the two coexist cleanly only if the new path is
strictly additive.

How voxel uses sp-emu today (`voxel-config/src/sp.rs`): the `SpBackend::Emu`
runs sp-emu "in the switch zone on loopback exactly like sp-sim - so MGS reaches
it at the same `[::1]:333xx` unicast surface", with "the emulator's VLAN/trust/
location logic internal to its bridge and never seen by MGS." It is a drop-in for
`sp-sim`:

- The whole fleet is packed onto one switch-zone loopback, differentiated by
  port: `base_port + 0/1` for the two switch views, `ereport_base + 0/1`. The
  sidecar is at 33300, gimlet `i` at `33300 + 10*(i+1)` (`SP_PORT_BASE`,
  `PORT_STRIDE`).
- MGS's `[[simulated_sps]]` config is backend-agnostic (`bind_addr =
  "[::]:{base_port+inst}"`), so sp-emu must present the identical port surface
  sp-sim does.
- Per-SP identity is port-derived: sp-emu's VPD `idx = (base_port - 33300)/10`
  feeds the base MAC and serial.

So voxel cannot move to well-known ports without giving each SP its own address —
it deliberately multiplexes the fleet onto one loopback to mirror sp-sim, and
well-known ports would collide there. It also does not need to: MGS reaches the
SP fine over the port-multiplexed surface via `--sp-sim-addr`. Well-known ports
matter only for the *other* sockets (hiffy, udprpc, dump_agent) that humility's
network backends dial at fixed ports — which is the workshop's need, not voxel's.

Requirements for not disrupting voxel:

1. The default, env-driven path stays bit-for-bit: `SP_EMU_BRIDGE=[::1]:base_port`
   binds `base+0/1`, ereport `base+11100`, and derives VPD identity from the
   port, exactly as today. The manifest/well-known-ports path is reached only via
   the new `--index`/`--name`/`--config` selectors and is off unless asked for.
2. The VPD identity derivation must remain available as the default: the manifest
   becomes an alternative source, not a replacement, with the port->idx path
   retained as fallback. voxel's expected identities depend on it.
3. Verify the identity contract before landing: voxel expects the sidecar serial
   `SIDECAR_SERIAL` and gimlet serials `2{i:07}` (`sp.rs` `for_gimlets`), while
   sp-emu's VPD currently derives `BRM4422000{idx}`. Confirm how these reconcile
   over the wire (config vs. VPD-reported serial) so the identity refactor does
   not silently change what MGS sees. This is a pre-existing coupling the refactor
   must preserve, whatever its current resolution.

Future option (not required): if voxel ever gives each in-zone SP its own address
(a per-SP loopback alias in the switch zone, or a per-SP zone), it could opt into
well-known ports via the same manifest. The design leaves that door open without
forcing it.

## Migration of the demo

`demo/run-sp.sh`, `demo/run-fleet.sh`, `demo/mgs`, and the README gain the new
`--name`/`--index` path and a container-first quickstart, while the existing
`SP_EMU_BRIDGE=[::1]:port` invocations keep working. The built-in manifest
reproduces today's a4x2 fleet, so nothing that works now stops working.

## Open decisions

1. `address` per entry is optional (omit for one-container-per-SP; present when a
   fleet shares one namespace). When present, an explicit pair
   `["...:110", "...:111"]` for the two switch views (unambiguous) vs. one base
   with switch1 derived. Leaning explicit.
2. Expose `mac_count`/`stride` per entry (faithful to real VPD, default 128/1)
   vs. keep the constant. Leaning expose.
3. Manifest scope: SP-only for now, or leave room for RoT pairing and per-SP
   flash paths later. Leaning SP identity now, keep the schema open.

## What this resolves

- The bridge relaying only `control_plane_agent` (+ ereport): all
  archive-declared sockets are bridged.
- The stale humility debug path: the durable answer is network hiffy over
  well-known ports plus `humility hydrate` for reads, rather than pinning
  humility for the removed `ocdgdb`/`ocd` stubs. README/banner updated
  accordingly.
- The lack of a documented sp-emu/sp-test link: the well-known-port surface is
  what a `simulator = true` loopback testbed would target.

## Out of scope (other repos)

- humility: needs the NetHiffy backend (pinned pre-`e85b5d8e` window today).
- hubris: a `hiffy`/`udprpc` socket must be enabled in the SP image for net
  hiffy to have an endpoint (hubris#2466). The current gimlet-c release image
  exposes neither.
- sp-test: making NetHiffy a first-class, auto-selected transport is planned
  work there.
