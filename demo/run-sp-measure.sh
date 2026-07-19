#!/usr/bin/env bash
# Boot the emulated gimlet SP together with an in-process emulated LPC55 RoT, and
# watch the RoT measure the SP over an internal SWD link and record the result --
# the RFD 568 attestation handoff, exactly as real Oxide hardware does at boot.
#
# Unlike run-sp-rot.sh (which attaches an out-of-process rot-serve over the sprot
# SPI link, just for boot-info), this runs the RoT core *in process* via
# SP_EMU_ROT_FLASH, so the RoT actually drives the SP's debug port: it resets the
# SP into debug halt, injects the `endoscope` measurement program, runs it to hash
# the SP flash, reads the digest back, and deposits the VALID measurement token so
# the SP boots normally.
#
#   ./run-sp-measure.sh <oxide-rot-1 image>   # boot gimlet0 + in-process RoT, measure
#   ROT_IMAGE=<image> ./run-sp-measure.sh
#   SP_BASE=33320 ./run-sp-measure.sh <image> # boot as gimlet1
#
# The RoT image must be an oxide-rot-1 build whose drv-lpc55-swd does the SWD
# measurement (e.g. a self-signed sp-reset-testing build):
#   <hubris>/.../oxide-rot-1-selfsigned/dist/a/final.bin

set -euo pipefail

HERE="$(cd -- "$(dirname "${BASH_SOURCE[0]:-$0}")" >/dev/null 2>&1 && pwd -P)"
ROOT="$(cd -- "$HERE/.." >/dev/null 2>&1 && pwd -P)"

SPEMU="${SPEMU:-$ROOT/target/release/sp-emu}"
SP_BASE="${SP_BASE:-33310}"
SP_LOG="${LOG:-/tmp/sp-emu-measure.log}"
export SP_EMU_FLASH="${SP_EMU_FLASH:-$ROOT/sp-flash.bin}"
ROT_IMAGE="${1:-${ROT_IMAGE:-}}"

run() { printf '$ %s\n' "$*"; }

if [ ! -s "$SP_EMU_FLASH" ]; then
  echo "[!] Flash image $SP_EMU_FLASH is missing/empty. Flash gimlet-c into slot A first:"
  echo "    $SPEMU flash a <hubris>/target/gimlet-c/dist/default/build-gimlet-c-image-default.zip"
  exit 1
fi
if [ -z "$ROT_IMAGE" ] || [ ! -s "$ROT_IMAGE" ]; then
  echo "[!] RoT image missing. Pass a measurement-capable oxide-rot-1 image:"
  echo "    ./run-sp-measure.sh <hubris>/.../oxide-rot-1-selfsigned/dist/a/final.bin"
  echo "    (or set ROT_IMAGE=...)"
  exit 1
fi
# Same shape check as run-sp-rot.sh: a "PK" build archive or a raw LPC55 image.
MAGIC2="$(od -An -tx1 -N2 "$ROT_IMAGE" | tr -d ' ')"
SP_HI="$(od -An -tx1 -j3 -N1 "$ROT_IMAGE" | tr -d ' ')"
if [ "$MAGIC2" != "504b" ] && [ "$SP_HI" != "20" ] && [ "$SP_HI" != "30" ]; then
  echo "[!] $ROT_IMAGE does not look like an oxide-rot-1 image. Pass the firmware,"
  echo "    e.g. target/oxide-rot-1-selfsigned/dist/a/final.bin — not a board/app .toml."
  exit 1
fi

: > "$SP_LOG"
# SP_EMU_SWD_TRIGGER fires one synthetic SP-reset measurement request after boot,
# so the RoT measures even though this gimlet image does not gate its own boot on
# the token. The RoT deposits the token itself when the measurement succeeds.
run "SP_EMU_BRIDGE=[::1]:${SP_BASE} SP_EMU_ROT_FLASH=${ROT_IMAGE} SP_EMU_SWD_TRIGGER=1 ${SPEMU} gdb a 340000000 &"
SP_EMU_BRIDGE="[::1]:${SP_BASE}" SP_EMU_ROT_FLASH="$ROT_IMAGE" SP_EMU_SWD_TRIGGER=1 \
  "$SPEMU" gdb a 340000000 > "$SP_LOG" 2>&1 &
PID=$!
echo "pid ${PID}, log ${SP_LOG}"

printf "booting and measuring the SP "
DEADLINE=$(( SECONDS + 180 ))
until grep -q 'SP measurement recorded' "$SP_LOG" 2>/dev/null; do
  if ! kill -0 "$PID" 2>/dev/null; then echo; echo "[!] emulator exited early — see ${SP_LOG}"; exit 1; fi
  if grep -q 'SP measurement skipped' "$SP_LOG" 2>/dev/null; then
    echo; echo "[!] RoT deposited SKIP, not VALID — measurement did not succeed. See ${SP_LOG}"; kill "$PID" 2>/dev/null || true; exit 1
  fi
  if [ "$SECONDS" -ge "$DEADLINE" ]; then
    echo; echo "[!] no measurement within 180s — see ${SP_LOG}"; kill "$PID" 2>/dev/null || true; exit 1
  fi
  printf "."
  sleep 2
done
echo
echo "measured:"
grep 'SP measurement recorded' "$SP_LOG" | tail -1 | sed 's/^/    /'
echo
echo "The RoT reset the SP into debug halt, ran endoscope over internal SWD, and"
echo "deposited the VALID token so the SP boots normally. The SP keeps running:"
echo
echo "    ./tasks --swd          # read the live task table over the SWD debug port"
echo "    ./mgs state            # (after tens of seconds, once the network stack is up)"
echo "    kill ${PID}"
