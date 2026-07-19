# Running sp-emu with well-known SP ports

By default sp-emu multiplexes a fleet onto one loopback address and encodes each
instance's identity in its bind *port* (`SP_EMU_BRIDGE=[::1]:33310` -> mgmt at
33310/33311, ereport at 44410/44411, ...). That is what voxel wants, and it is
unchanged.

The **well-known-port mode** is an opt-in alternative: each emulated SP binds the
firmware's real UDP socket ports on its own IPv6 address, so faux-mgs, humility,
and sp-test reach it at exactly the addresses and ports they would use against
real hardware -- `<addr>:11111` for MGS, `<addr>:57005` for ereports, and so on,
with no per-tool port arithmetic. This is what lets sp-test collect ereports from
the emulated SP (the client dials the fixed port 57005; the offset-mode bridge
never bound it).

## Enabling it

```
SP_EMU_WELL_KNOWN_PORTS=1 \
SP_EMU_ADDR0=::1 \
  sp-emu gdb a
```

- `SP_EMU_WELL_KNOWN_PORTS=1` selects the mode.
- `SP_EMU_ADDR0` is the switch0 (management uplink 0) host address. Default `::1`.
- `SP_EMU_ADDR1` is the switch1 address. Omit for a single uplink. It must be a
  **different** address from `SP_EMU_ADDR0`, because both uplinks bind the same
  real ports -- that is the whole point of per-instance addresses.
- `SP_EMU_VID0` / `SP_EMU_VID1` and `SP_EMU_BOARD` set the VLAN ids / board as in
  the default mode.

The sockets bound (per address) are the union the board declares: `echo` (7),
`broadcast` (997), `rpc`/udprpc (998), `control_plane_agent` (11111),
`dump_agent` (11113), `ereport` (57005), plus `inspector` (23547) on gimlet or
`transceivers` (11112) on sidecar.

Point tools at the address directly:

```
faux-mgs --sp-sim-addr [::1]:11111 state
faux-mgs --sp-sim-addr [::1]:11111 ereports
```

## Privileged ports (echo 7, broadcast 997, rpc 998)

Three of the SP sockets are below 1024, which most Linux systems reserve for
privileged processes. If sp-emu cannot bind one, it prints

```
[bridge] skip SP port 7 on [::1]:7 (vid 0x301): Permission denied (os error 13) (socket not bridged)
```

and continues -- mgmt (11111) and ereport (57005) still work, only the low
sockets are missing. `echo`/`broadcast`/`rpc` are rarely needed for sp-test, so
this fallback is fine for a quick run. To bind them, pick **one** of the options
below (in rough order of preference).

### Option A -- grant the binary the capability (host, persistent)

Give just the sp-emu binary permission to bind low ports, without running it as
root:

```
sudo setcap 'cap_net_bind_service=+ep' target/release/sp-emu
getcap target/release/sp-emu            # verify: cap_net_bind_service=ep
```

Re-run `setcap` after each rebuild (a new binary drops the capability). To remove
it: `sudo setcap -r target/release/sp-emu`.

### Option B -- lower the unprivileged-port floor (host or container, whole system)

Allow *any* process to bind from port 0 up:

```
sudo sysctl -w net.ipv4.ip_unprivileged_port_start=0
```

This also covers IPv6 (the setting is shared). Persist it across reboots with a
drop-in:

```
echo 'net.ipv4.ip_unprivileged_port_start=0' | sudo tee /etc/sysctl.d/50-sp-emu.conf
sudo sysctl --system
```

Undo by restoring the default (`1024`).

### Option C -- container capability (workshop / CI)

When sp-emu runs in a container (the workshop's stage 1), grant the capability to
the container rather than the host:

```
docker run --cap-add=NET_BIND_SERVICE ...
# or
podman run --cap-add=NET_BIND_SERVICE ...
```

Equivalently, set the sysctl inside the container:

```
docker run --sysctl net.ipv4.ip_unprivileged_port_start=0 ...
```

The workshop's stage-1 image does this for you (see the stage-1 container docs),
so students never hit the privileged-port wall.

### Option D -- run as root

Works, but is the least preferred: prefer the capability (Option A/C) so the
emulator runs unprivileged.

## Multiple instances / two uplinks: loopback aliases

Because instances are distinguished by address, two uplinks (or two SPs) on one
host need distinct addresses. On Linux, add loopback aliases (needs
`CAP_NET_ADMIN` / root):

```
sudo ip -6 addr add fdb0::110/128 dev lo     # gimlet0 switch0
sudo ip -6 addr add fdb0::111/128 dev lo     # gimlet0 switch1

SP_EMU_WELL_KNOWN_PORTS=1 SP_EMU_ADDR0=fdb0::110 SP_EMU_ADDR1=fdb0::111 sp-emu gdb a
```

Inside a container each SP can instead get its own container IP (its own network
namespace), and bind the well-known ports on the wildcard `[::]` with no aliases.

Remove aliases with `sudo ip -6 addr del fdb0::110/128 dev lo`.
