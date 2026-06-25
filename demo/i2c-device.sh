#!/usr/bin/bash
#
# demo-i2c-device.sh - a local socket ACTS AS an I2C device for the emulated SP.
# Box-local, no rack: a `sp-emu i2c-device` server injects a chosen response for
# one device/register (defers the rest to the built-in model so the SP boots),
# the SP delegates its I2C reads to it, and we watch the SP consume the value.
# Prints each fully-expanded command it runs, transcript style.
#
# Usage:
#   demo-i2c-device.sh [inject-spec]                 show the SP reading the injection
#   demo-i2c-device.sh --read-sensor [inject-spec]   ALSO read the value back through
#                                                    Hubris's sensor API over MGS
# Default inject-spec: 0x48/0x00=0x4000  (front TMP117 TempResult = ~128 C).
#
set -u
# Resolve the sp-emu binary relative to this script (demos/ -> repo root), so the
# demo works wherever the repo lives. faux-mgs + the hubris image are in sibling
# repos (box-specific paths; override with $FAUX / $IMG).
HERE="$(cd -- "$(dirname "$0")" >/dev/null 2>&1 && pwd -P)"
SP="${SPEMU:-$(cd "$HERE/.." >/dev/null 2>&1 && pwd -P)/target/release/sp-emu}"
FAUX="${FAUX:-/root/oxide/management-gateway-service/target/release/faux-mgs}"
IMG="${IMG:-/root/oxide/hubris/target/gimlet-c-emu/dist/default/build-gimlet-c-emu-image-default.zip}"
BRIDGE="[::1]:33360"
DEVSOCK="[::1]:9100"
LOG=/tmp/i2c-device.log
FLASH=/tmp/i2c-dev.flash

READ_SENSOR=0
if [ "${1:-}" = "--read-sensor" ]; then READ_SENSOR=1; shift; fi
INJECT="${1:-0x48/0x00=0x4000}"

cleanup() { kill ${EMU:-0} ${DEV:-0} 2>/dev/null; rm -f "$FLASH"; }
trap cleanup EXIT
rm -f "$LOG" "$FLASH"
[ -x "$SP" ] || { echo "build sp-emu first"; exit 1; }
# Box-local hygiene (silent): reap leftover demo procs so the socket is free.
pkill -f "$SP i2c-device" 2>/dev/null; pkill -f "$SP i2c-sniff" 2>/dev/null
pkill -f "$SP gdb a 340000000" 2>/dev/null; sleep 1

run() { echo "\$ $*"; }   # print the fully-expanded command, then we run it

run "SP_EMU_FLASH=$FLASH $SP flash a $IMG"
SP_EMU_FLASH="$FLASH" "$SP" flash a "$IMG" 2>&1 || { echo flash failed; exit 1; }

echo
run "$SP i2c-device $DEVSOCK $INJECT &"
"$SP" i2c-device "$DEVSOCK" "$INJECT" >"$LOG" 2>&1 & DEV=$!
sleep 1

echo
run "SP_EMU_BOARD=gimlet SP_EMU_FLASH=$FLASH SP_EMU_BRIDGE=$BRIDGE SP_EMU_NO_DEBUG=1 SP_EMU_I2C_DEVICE=$DEVSOCK $SP gdb a 340000000 &"
SP_EMU_BOARD=gimlet SP_EMU_FLASH="$FLASH" SP_EMU_BRIDGE="$BRIDGE" SP_EMU_NO_DEBUG=1 \
  SP_EMU_I2C_DEVICE="$DEVSOCK" "$SP" gdb a 340000000 >/tmp/i2c-dev-emu.log 2>&1 & EMU=$!
echo "  # ~45s preboot, then the SP polls its I2C devices ..."
for i in $(seq 1 50); do grep -q injecting "$LOG" 2>/dev/null && break; sleep 1; done
sleep 6

echo
run "grep -E 'inject|listening|connected' $LOG"
grep -E "inject|listening|connected" "$LOG" | head -20

if [ "$READ_SENSOR" = 1 ]; then
  # Wait (silently, with retries) for the sensor task to publish a reading.
  for i in $(seq 1 30); do
    [ -n "$("$FAUX" --json --sp-sim-addr "$BRIDGE" --max-attempts 3 --per-attempt-timeout-millis 6000 read-sensor-value 0 2>/dev/null | sed -n 's/.*"value":"\([^"]*\)".*/\1/p')" ] && break
    sleep 2
  done
  for id in 0 1 2; do
    echo
    run "$FAUX --json --sp-sim-addr $BRIDGE read-sensor-value $id"
    "$FAUX" --json --sp-sim-addr "$BRIDGE" read-sensor-value "$id" 2>/dev/null
  done
fi
