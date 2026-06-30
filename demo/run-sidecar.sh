#!/usr/bin/env bash
# Boot the emulated sidecar Service Processor (running Oxide Hubris firmware) and
# wait until its network stack is up and reachable by MGS.
#
#   ./run-sidecar.sh            # boot the sidecar on ports 33300 (switch0) / 33301 (switch1)
#
# Boots the `sidecar-c-emu` image: production sidecar firmware built with the
# `emulator` feature on drv-sidecar-seq-server, which skips hardware bring-up the
# emulator can't drive (VPD EEPROM, idt8a34003 clock gen, front-IO board, Tofino
# power sequencer) and injects a canned MAC/identity so the sequencer reaches its
# dispatch loop reporting A2. a4x2's Tofino data plane is SoftNPU (software P4),
# so the SP only needs to answer MGS.

set -euo pipefail

# Resolve paths relative to this script's directory; works regardless of checkout
# location or launch directory. Override SPEMU / SP_EMU_FLASH in the environment.
HERE="$(cd -- "$(dirname "${BASH_SOURCE[0]:-$0}")" >/dev/null 2>&1 && pwd -P)"
ROOT="$(cd -- "$HERE/.." >/dev/null 2>&1 && pwd -P)"

SPEMU="${SPEMU:-$ROOT/target/release/sp-emu}"
SP_BASE="${SP_BASE:-33300}"
LOG="${LOG:-/tmp/sp-emu-sidecar.log}"
export SP_EMU_BOARD=sidecar
# Flash image (sidecar-c-emu in slot A); located at the repo root by convention.
export SP_EMU_FLASH="${SP_EMU_FLASH:-$ROOT/sidecar-flash.bin}"

if [ ! -s "$SP_EMU_FLASH" ]; then
  echo "[!] Flash image $SP_EMU_FLASH is missing/empty. Flash sidecar-c-emu into slot A first:"
  echo "    $SPEMU flash a <hubris>/target/sidecar-c-emu/dist/default/build-sidecar-c-emu-image-default.zip"
  exit 1
fi

echo "[*] Booting emulated sidecar SP: Hubris sidecar-c-emu firmware on an"
echo "    emulated STM32H753 (Cortex-M7). Binding MGS ports ${SP_BASE} (switch0)"
echo "    and $((SP_BASE+1)) (switch1). Trusted mgmt VLANs 0x130 / 0x302."
: > "$LOG"
SP_EMU_BRIDGEDBG=1 SP_EMU_BRIDGE="[::1]:${SP_BASE}" \
  "$SPEMU" gdb a 340000000 > "$LOG" 2>&1 &
PID=$!
echo "    pid ${PID}   (full log: ${LOG})"
echo "[*] Booting the kernel + ~28 tasks and bringing up the network."
echo "    Estimated bring up time is ~60-90s. Watching for the SP to appear..."
printf "    "
# 0x130 = local_sidecar, the trusted switch0 management VLAN.
until grep -q "learned SP vid 0x130" "$LOG" 2>/dev/null; do
  if ! kill -0 "$PID" 2>/dev/null; then echo; echo "[!] SP exited early. See ${LOG}"; exit 1; fi
  printf "."
  sleep 2
done
echo
echo "[+] SP ONLINE. The emulated sidecar is on the (virtual) management network."
echo "    Reach it with MGS at  [::1]:${SP_BASE}  (switch0)  or  [::1]:$((SP_BASE+1))  (switch1)."
echo
echo "    Now run, from this demo dir:"
echo "        SP_PORT=${SP_BASE} ./mgs discover     # MGS finds the SP (port: One = switch0)"
echo "        SP_PORT=$((SP_BASE+1)) ./mgs discover  # switch1 view (port: Two)"
echo "        SP_PORT=${SP_BASE} ./mgs state         # power state A2, MAC, identity, RoT"
echo "        SP_PORT=${SP_BASE} ./mgs inventory      # full sidecar component tree"
echo
echo "    Stop the SP with:  kill ${PID}"
