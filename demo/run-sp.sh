#!/usr/bin/env bash
# Boot the emulated gimlet Service Processor (running Oxide Hubris firmware) and
# wait until its network stack is up and reachable by MGS.
#
#   ./run-sp.sh            # boot gimlet0 on ports 33310 (switch0) / 33311 (switch1)
#   SP_BASE=33320 ./run-sp.sh   # boot as gimlet1

set -euo pipefail

# Resolve everything relative to this repo (the dir this script lives in), so the
# demo works no matter where the checkout is or where it's launched from. Override
# any of SPEMU / SP_EMU_FLASH in the environment to point elsewhere.

HERE="$(cd -- "$(dirname "${BASH_SOURCE[0]:-$0}")" >/dev/null 2>&1 && pwd -P)"
ROOT="$(cd -- "$HERE/.." >/dev/null 2>&1 && pwd -P)"

SPEMU="${SPEMU:-$ROOT/target/release/sp-emu}"
SP_BASE="${SP_BASE:-33310}"
LOG="${LOG:-/tmp/sp-emu-demo.log}"
# The flash image (gimlet-c in slot A) lives at the repo root by convention.
export SP_EMU_FLASH="${SP_EMU_FLASH:-$ROOT/sp-flash.bin}"

if [ ! -s "$SP_EMU_FLASH" ]; then
  echo "[!] Flash image $SP_EMU_FLASH is missing/empty. Flash gimlet-c into slot A first:"
  echo "    $SPEMU flash a <hubris>/target/gimlet-c/dist/default/build-gimlet-c-image-default.zip"
  exit 1
fi

echo "[*] Booting emulated gimlet SP: Hubris gimlet-c firmware on an"
echo "    emulated STM32H753 (Cortex-M7). Binding MGS ports ${SP_BASE} (switch0)"
echo "    and $((SP_BASE+1)) (switch1)."
: > "$LOG"
SP_EMU_BRIDGEDBG=1 SP_EMU_BRIDGE="[::1]:${SP_BASE}" \
  "$SPEMU" gdb a 340000000 > "$LOG" 2>&1 &
PID=$!
echo "    pid ${PID}   (full log: ${LOG})"
echo "[*] Booting the kernel + 30+ tasks and bringing up the network — this is"
echo "    real firmware, so it takes ~60-90s. Watching for the SP to appear..."
printf "    "
until grep -q "learned SP vid 0x301" "$LOG" 2>/dev/null; do
  if ! kill -0 "$PID" 2>/dev/null; then echo; echo "[!] SP exited early — see ${LOG}"; exit 1; fi
  printf "."
  sleep 2
done
echo
echo "[+] SP ONLINE. The emulated gimlet is on the (virtual) management network."
echo "    Reach it with MGS at  [::1]:${SP_BASE}  (switch0)  or  [::1]:$((SP_BASE+1))  (switch1)."
echo
echo "    Now run, from this demo dir:"
echo "        ./mgs discover     # MGS finds the SP"
echo "        ./mgs state        # power state, MAC, firmware id, RoT status"
echo "        ./mgs inventory    # full gimlet component tree"
echo "        ./tasks            # live Hubris task table (humility)"
echo
echo "    Stop the SP with:  kill ${PID}"
