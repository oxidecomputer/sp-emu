#!/usr/bin/env bash
# Boot the emulated gimlet SP together with an emulated LPC55 RoT, wired over the
# sprot SPI link, and wait until the SP's network stack is up and reachable by MGS.
#
# run-sp.sh plus a root-of-trust: the SP's drv-stm32h7-sprot-server talks to the
# RoT's drv-lpc55-sprot-server, avoiding the "RoT failed to deassert" hang; `./mgs
# state` then reports real RoT boot-info digests instead of rot: Err(...).
#
#   ./run-sp-rot.sh <oxide-rot-1 image>     # boot gimlet0 + RoT
#   ROT_IMAGE=<image> ./run-sp-rot.sh       # same, image via env
#   SP_BASE=33320 ./run-sp-rot.sh <image>   # boot as gimlet1
#
# The RoT image is an oxide-rot-1 build (raw final.bin or a build archive), e.g.
#   <hubris>/target/oxide-rot-1/dist/a/final.bin

set -euo pipefail

# Resolve paths relative to this script's directory; works regardless of checkout
# location or launch directory. Override SPEMU / SP_EMU_FLASH / ROT_IMAGE / ROT_PORT
# in the environment.

HERE="$(cd -- "$(dirname "${BASH_SOURCE[0]:-$0}")" >/dev/null 2>&1 && pwd -P)"
ROOT="$(cd -- "$HERE/.." >/dev/null 2>&1 && pwd -P)"

SPEMU="${SPEMU:-$ROOT/target/release/sp-emu}"
SP_BASE="${SP_BASE:-33310}"
# RoT service port: SP_BASE - 14000 (the derivation voxel uses); stable and
# paired with the SP.
ROT_PORT="${ROT_PORT:-$((SP_BASE - 14000))}"
ROT_ADDR="[::1]:${ROT_PORT}"
SP_LOG="${LOG:-/tmp/sp-emu-demo.log}"
ROT_LOG="${ROT_LOG:-/tmp/rot-emu-demo.log}"
# Flash image (gimlet-c in slot A); located at the repo root by convention.
export SP_EMU_FLASH="${SP_EMU_FLASH:-$ROOT/sp-flash.bin}"
# RoT image: first positional arg, else ROT_IMAGE.
ROT_IMAGE="${1:-${ROT_IMAGE:-}}"

if [ ! -s "$SP_EMU_FLASH" ]; then
  echo "[!] Flash image $SP_EMU_FLASH is missing/empty. Flash gimlet-c into slot A first:"
  echo "    $SPEMU flash a <hubris>/target/gimlet-c/dist/default/build-gimlet-c-image-default.zip"
  exit 1
fi
if [ -z "$ROT_IMAGE" ] || [ ! -s "$ROT_IMAGE" ]; then
  echo "[!] RoT image missing. Pass an oxide-rot-1 image (raw final.bin or build archive):"
  echo "    ./run-sp-rot.sh <hubris>/target/oxide-rot-1/dist/a/final.bin"
  echo "    (or set ROT_IMAGE=...)"
  exit 1
fi
# Reject a non-image (e.g. a board/app .toml): accept a "PK" build archive, or a
# raw image whose first word is an LPC55 RAM stack pointer (high byte 0x20/0x30).
MAGIC2="$(od -An -tx1 -N2 "$ROT_IMAGE" | tr -d ' ')"
SP_HI="$(od -An -tx1 -j3 -N1 "$ROT_IMAGE" | tr -d ' ')"
if [ "$MAGIC2" != "504b" ] && [ "$SP_HI" != "20" ] && [ "$SP_HI" != "30" ]; then
  echo "[!] $ROT_IMAGE does not look like an oxide-rot-1 image (no build-archive or"
  echo "    Cortex-M vector table). Pass the firmware, e.g. target/oxide-rot-1/dist/a/final.bin"
  echo "    — not a board/app .toml. None is built? Build the oxide-rot-1 app in hubris first."
  exit 1
fi

# Start the shared RoT first so its socket is listening before the SP connects.
echo "[*] Starting emulated LPC55 RoT (oxide-rot-1 firmware) on ${ROT_ADDR}."
: > "$ROT_LOG"
"$SPEMU" rot-serve "$ROT_ADDR" "$ROT_IMAGE" > "$ROT_LOG" 2>&1 &
ROT_PID=$!
echo "    pid ${ROT_PID}   (full log: ${ROT_LOG})"
printf "    waiting for the RoT to bind its socket"
until grep -q "rotsvc] listening on" "$ROT_LOG" 2>/dev/null; do
  if ! kill -0 "$ROT_PID" 2>/dev/null; then echo; echo "[!] RoT exited early — see ${ROT_LOG}"; exit 1; fi
  printf "."
  sleep 1
done
echo " ok"

# Boot the SP, pointed at the RoT over the sprot link.
echo "[*] Booting emulated gimlet SP: Hubris gimlet-c firmware on an"
echo "    emulated STM32H753 (Cortex-M7). Binding MGS ports ${SP_BASE} (switch0)"
echo "    and $((SP_BASE+1)) (switch1), with the RoT attached at ${ROT_ADDR}."
: > "$SP_LOG"
SP_EMU_BRIDGEDBG=1 SP_EMU_BRIDGE="[::1]:${SP_BASE}" SP_EMU_ROT_SERVICE="$ROT_ADDR" \
  "$SPEMU" gdb a 340000000 > "$SP_LOG" 2>&1 &
PID=$!
echo "    pid ${PID}   (full log: ${SP_LOG})"
echo "[*] Booting the kernel + 30+ tasks and bringing up the network — this is"
echo "    real firmware, so it takes ~60-90s. Watching for the SP to appear..."
printf "    "
until grep -q "learned SP vid 0x301" "$SP_LOG" 2>/dev/null; do
  if ! kill -0 "$PID" 2>/dev/null; then
    echo; echo "[!] SP exited early — see ${SP_LOG}"; kill "$ROT_PID" 2>/dev/null || true; exit 1
  fi
  if ! kill -0 "$ROT_PID" 2>/dev/null; then
    echo; echo "[!] RoT exited — see ${ROT_LOG}"; kill "$PID" 2>/dev/null || true; exit 1
  fi
  printf "."
  sleep 2
done
echo
echo "[+] SP + RoT ONLINE. The emulated gimlet is on the (virtual) management network,"
echo "    with a live root-of-trust over the sprot link."
echo "    Reach it with MGS at  [::1]:${SP_BASE}  (switch0)  or  [::1]:$((SP_BASE+1))  (switch1)."
echo
echo "    Now run, from this demo dir:"
echo "        ./mgs discover     # MGS finds the SP"
echo "        ./mgs state        # power state, MAC, firmware id — RoT status now OK, not Err"
echo "        ./mgs inventory    # full gimlet component tree"
echo "        ./tasks            # live Hubris task table (humility)"
echo
echo "    Stop both with:  kill ${PID} ${ROT_PID}"
