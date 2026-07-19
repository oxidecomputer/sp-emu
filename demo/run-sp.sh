#!/usr/bin/env bash
# Boot the emulated gimlet Service Processor (running Oxide Hubris firmware) and
# wait until its network stack is up and reachable by MGS.
#
#   ./run-sp.sh            # boot gimlet0 on ports 33310 (switch0) / 33311 (switch1)
#   SP_BASE=33320 ./run-sp.sh   # boot as gimlet1

set -euo pipefail

# Resolve paths relative to this script's directory; works regardless of checkout
# location or launch directory. Override SPEMU / SP_EMU_FLASH in the environment.

HERE="$(cd -- "$(dirname "${BASH_SOURCE[0]:-$0}")" >/dev/null 2>&1 && pwd -P)"
ROOT="$(cd -- "$HERE/.." >/dev/null 2>&1 && pwd -P)"

SPEMU="${SPEMU:-$ROOT/target/release/sp-emu}"
SP_BASE="${SP_BASE:-33310}"
LOG="${LOG:-/tmp/sp-emu-demo.log}"
# Flash image (gimlet-c in slot A); located at the repo root by convention.
export SP_EMU_FLASH="${SP_EMU_FLASH:-$ROOT/sp-flash.bin}"

run() { printf '$ %s\n' "$*"; }   # echo a command before running it

if [ ! -s "$SP_EMU_FLASH" ]; then
  echo "[!] Flash image $SP_EMU_FLASH is missing/empty. Flash gimlet-c into slot A first:"
  echo "    $SPEMU flash a <hubris>/target/gimlet-c/dist/default/build-gimlet-c-image-default.zip"
  exit 1
fi

: > "$LOG"
run "SP_EMU_BRIDGE=[::1]:${SP_BASE} ${SPEMU} gdb a 340000000 &"
SP_EMU_BRIDGEDBG=1 SP_EMU_BRIDGE="[::1]:${SP_BASE}" \
  "$SPEMU" gdb a 340000000 > "$LOG" 2>&1 &
PID=$!
echo "pid ${PID}, log ${LOG}"
printf "waiting for the SP (tens of seconds) "
until grep -qE '\[sp-emu\] online|learned SP vid 0x301' "$LOG" 2>/dev/null; do
  if ! kill -0 "$PID" 2>/dev/null; then echo; echo "[!] SP exited early — see ${LOG}"; exit 1; fi
  printf "."
  sleep 2
done
echo
echo "online: [::1]:${SP_BASE} (switch0)  [::1]:$((SP_BASE+1)) (switch1)"
echo
echo "    ./mgs discover"
echo "    ./mgs state"
echo "    ./mgs inventory"
echo "    ./tasks"
echo "    kill ${PID}"
