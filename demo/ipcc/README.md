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

- **Local calls** (`status`, and the firmware's GetIdentity / GetMacAddresses /
  GetBootStorageUnit): serviced in the ~400 ms per-request range, comparable to an
  MGS `state` query -- the interpreter's inherent per-request cost, not an IPCC
  problem.
- **RoT-backed calls** (`get-certs`, `get-log`): these round-trip to the RoT over
  sprot. The standalone emulated SP has no RoT, so they fail; attach the in-process
  RoT with `SP_EMU_ROT_FLASH=<oxide-rot-1 archive.zip>` to service them, at
  seconds-scale latency.
</content>
