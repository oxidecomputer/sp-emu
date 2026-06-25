#!/usr/bin/bash
#
# demo-i2c-sniff.sh - watch the emulated SP's I2C bus live, via the sniff bridge.
# Box-local, no rack: flashes a hubris image, boots a standalone sp-emu with
# SP_EMU_I2C_BRIDGE pointed at `sp-emu i2c-sniff`, and live-streams one line per
# bus transaction as the SP brings the board up. Ends with a per-device summary.
# Prints each fully-expanded command it runs, transcript style.
# Usage: demo-i2c-sniff.sh [hubris-image.zip]
#
set -u
# Resolve the sp-emu binary relative to this script (demos/ -> repo root).
HERE="$(cd -- "$(dirname "$0")" >/dev/null 2>&1 && pwd -P)"
SP="${SPEMU:-$(cd "$HERE/.." >/dev/null 2>&1 && pwd -P)/target/release/sp-emu}"
IMG="${1:-/root/oxide/hubris/target/gimlet-c-emu/dist/default/build-gimlet-c-emu-image-default.zip}"
SNIFFSOCK="[::1]:9100"
TRACE=/tmp/i2c-demo-trace.log
FLASH=/tmp/i2c-demo.flash
WATCH=45

cleanup() { kill ${EMU:-0} ${SNIFF:-0} ${TAIL:-0} 2>/dev/null; rm -f "$FLASH"; }
trap cleanup EXIT
rm -f "$TRACE" "$FLASH"
[ -x "$SP" ] || { echo "build sp-emu first: (cd /root/oxide/sp-emu && cargo build --release)"; exit 1; }
[ -f "$IMG" ] || { echo "hubris image not found: $IMG"; exit 1; }
# Box-local hygiene (silent): reap leftover demo procs so the socket is free.
pkill -f "$SP i2c-sniff" 2>/dev/null; pkill -f "$SP i2c-device" 2>/dev/null
pkill -f "$SP gdb a 340000000" 2>/dev/null; sleep 1

run() { echo "\$ $*"; }

run "SP_EMU_FLASH=$FLASH $SP flash a $IMG"
SP_EMU_FLASH="$FLASH" "$SP" flash a "$IMG" 2>&1 || { echo flash failed; exit 1; }

echo
run "$SP i2c-sniff $SNIFFSOCK &"
"$SP" i2c-sniff "$SNIFFSOCK" >"$TRACE" 2>&1 & SNIFF=$!
sleep 1

echo
run "SP_EMU_BOARD=gimlet SP_EMU_FLASH=$FLASH SP_EMU_BRIDGE=[::1]:33360 SP_EMU_NO_DEBUG=1 SP_EMU_I2C_BRIDGE=$SNIFFSOCK $SP gdb a 340000000 &"
SP_EMU_BOARD=gimlet SP_EMU_FLASH="$FLASH" SP_EMU_BRIDGE="[::1]:33360" SP_EMU_NO_DEBUG=1 \
  SP_EMU_I2C_BRIDGE="$SNIFFSOCK" "$SP" gdb a 340000000 >/tmp/i2c-demo-emu.log 2>&1 & EMU=$!

echo
run "tail -f $TRACE | grep START      # live for ~${WATCH}s; one line per bus transaction"
tail -f "$TRACE" 2>/dev/null | grep START & TAIL=$!
sleep "$WATCH"
kill $TAIL 2>/dev/null

echo
run "grep -oE '#.*' $TRACE | sort | uniq -c | sort -rn      # device access summary"
grep -oE "#.*" "$TRACE" 2>/dev/null | sort | uniq -c | sort -rn
