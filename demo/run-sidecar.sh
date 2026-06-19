#!/usr/bin/env bash
# Boot the emulated sidecar Service Processor (real Oxide Hubris firmware) and
# wait until its network stack is up and reachable by MGS.
#
#   ./run-sidecar.sh            # boot the sidecar on ports 33300 (switch0) / 33301 (switch1)
#
# This boots the `sidecar-c-emu` image: the production sidecar firmware built
# with the `emulator` feature on drv-sidecar-seq-server, which skips the
# hardware bring-up the emulator can't drive (VPD EEPROM, idt8a34003 clock gen,
# front-IO board, Tofino power sequencer) and injects a canned MAC/identity so
# the sequencer reaches its dispatch loop reporting A2. a4x2's Tofino data plane
# is SoftNPU (software P4), so the SP only needs to answer MGS.
set -euo pipefail

SPEMU="${SPEMU:-$HOME/oxide/sp-emu/target/release/sp-emu}"
SP_BASE="${SP_BASE:-33300}"
LOG="${LOG:-/tmp/sp-emu-sidecar.log}"
export SP_EMU_BOARD=sidecar
export SP_EMU_FLASH="${SP_EMU_FLASH:-$HOME/oxide/sp-emu/sidecar-flash.bin}"

if [ ! -s "$SP_EMU_FLASH" ]; then
  echo "[!] Flash image $SP_EMU_FLASH is missing/empty. Flash sidecar-c-emu into slot A first:"
  echo "    $SPEMU flash a ~/oxide/hubris/target/sidecar-c-emu/dist/default/build-sidecar-c-emu-image-default.zip"
  exit 1
fi

echo "[*] Booting emulated sidecar SP — REAL Hubris sidecar-c-emu firmware on an"
echo "    emulated STM32H753 (Cortex-M7). Binding MGS ports ${SP_BASE} (switch0)"
echo "    and $((SP_BASE+1)) (switch1). Trusted mgmt VLANs 0x130 / 0x302."
: > "$LOG"
SP_EMU_BRIDGEDBG=1 SP_EMU_BRIDGE="[::1]:${SP_BASE}" \
  "$SPEMU" gdb a 340000000 > "$LOG" 2>&1 &
PID=$!
echo "    pid ${PID}   (full log: ${LOG})"
echo "[*] Booting the kernel + ~28 tasks and bringing up the network — this is"
echo "    real firmware, so it takes ~60-90s. Watching for the SP to appear..."
printf "    "
# 0x130 = local_sidecar, the trusted switch0 management VLAN.
until grep -q "learned SP vid 0x130" "$LOG" 2>/dev/null; do
  if ! kill -0 "$PID" 2>/dev/null; then echo; echo "[!] SP exited early — see ${LOG}"; exit 1; fi
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
