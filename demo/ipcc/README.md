# IPCC against the emulated SP (host_sp_comms)

sp-emu can present the host-SP UART -- the `host_sp_comms` / IPCC channel the host
CPU uses to talk to the SP -- so host-side tools like `faux-ipcc` can drive it.

## Boot sp-emu with a pty for the host UART

```sh
SP_EMU_HOST_PTY=1 SP_EMU_WELL_KNOWN_PORTS=1 SP_EMU_ADDR0=::1 \
SP_EMU_ARCHIVE=<gimlet-c archive.zip> \
    sp-emu gdb a 340000000
```

On startup it prints the pty to attach to:

```
[bridge] host-uart (UART7/IPCC) pty ready: /dev/pts/N  (attach: faux-ipcc --port /dev/pts/N ...)
```

(The `SP_EMU_HOST_UART=<unix socket>` back-end -- used by voxel/propolis -- is
unchanged; `SP_EMU_HOST_PTY` is the pty alternative for serial tools.)

## Talk to it with faux-ipcc

Two accommodations are needed because faux-ipcc targets real serial hardware:

1. **Read timeout.** The emulator is far slower per request than silicon (a local
   IPCC call is ~400 ms; RoT-backed calls are seconds). faux-ipcc's default read
   timeout is 200 ms, so raise it with `--read-timeout-ms` (a flag on the
   `faux-ipcc-timeouts` branch of `ipcc-rs`).
2. **Modem control.** faux-ipcc (via serial2) asserts the DTR line on open, which
   a pty does not have, so the open fails with ENOTTY. The `nomodem.so` shim here
   stubs the modem-control ioctls. Build it once:

   ```sh
   cc -shared -fPIC -o nomodem.so nomodem.c -ldl
   ```

Then:

```sh
LD_PRELOAD=./nomodem.so faux-ipcc --port /dev/pts/N --read-timeout-ms 5000 status
# INFO got status Status { status: Status(1), startup: HostStartupOptions(0) }
```

## What the emulated SP can service

Measured on gimlet-c with the in-process RoT attached
(`SP_EMU_ROT_FLASH=<oxide-rot-1 image-a>`):

- **Local calls** (`status`, and the firmware's GetIdentity / GetMacAddresses /
  GetBootStorageUnit -- packrat-local, no RoT): serviced fast. The first call after
  attaching the pty pays a one-time ~2 s channel-sync cost (COBS resync on the freshly
  opened pipe); every call after that is ~8 ms. faux-ipcc only exposes `status` of
  this group.
- **RoT-backed calls** (`get-certs`, `get-log`): **not serviceable yet.** They round-
  trip to the RoT attest task over sprot, and the SP's own sprot layer times out on
  the RoT -- faux-ipcc reports `SprotError(ProtocolTimeout)` in ~0.14 s. This is an
  SP-side deadline, so raising faux-ipcc's `--read-timeout-ms` does not help. The
  emulated RoT core boots, but attestation over the emulated sprot/SWD link
  (RFD 568 measurement + attest replies) is not wired up end to end. Tracked with the
  phase-2 RoT-over-SWD work in `doc/` -- see the sp-swd debug-port plan.
</content>
