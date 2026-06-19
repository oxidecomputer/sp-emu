#!/usr/bin/env bash
# run-fleet.sh — launch + gate + supervise a fleet of emulated SPs for a4x2/voxel.
#
# This is the reference voxel-init's launch+gate is meant to mirror in Rust:
#   (1) canonical per-SP launch line, (2) the readiness gate, (3) supervision.
#
# Each emulated SP is ONE process running ONE Hubris image, binding a switch0/
# switch1 port pair on loopback (in oxz_switch, MGS reaches it on [::1] exactly
# like sp-sim). Edit FLEET below (or translate this logic) for your topology.
set -uo pipefail

SPEMU="${SPEMU:-$HOME/oxide/sp-emu/target/release/sp-emu}"
HUBRIS="${HUBRIS:-$HOME/oxide/hubris}"
RUNDIR="${RUNDIR:-/var/tmp/sp-emu}"          # flash files + logs live here
IDLE_MS="${SP_EMU_IDLE_MS:-20}"              # idle-CPU vs latency knob; see notes below
PREBOOT="${PREBOOT:-340000000}"              # instructions to steady state (~17s now)

# Fleet topology: "name  board  switch0_port"  (switch1 = port+1).
#   board = sidecar | gimlet ;  ports per the a4x2 map (sidecar 33300, gimlet i 333{i+1}0)
FLEET=(
  "sidecar0  sidecar  33300"
  "gimlet0   gimlet   33310"
)

image_for() {  # board -> image .zip (built by `cargo xtask dist` against v25 hubris)
  case "$1" in
    sidecar) echo "$HUBRIS/target/sidecar-c-emu/dist/default/build-sidecar-c-emu-image-default.zip" ;;
    gimlet)  echo "$HUBRIS/target/gimlet-c/dist/default/build-gimlet-c-image-default.zip" ;;
    *) echo "" ;;
  esac
}

declare -A PIDS
mkdir -p "$RUNDIR"

# (1) Canonical per-SP launch. This exact env is what voxel-init must set:
#     SP_EMU_BOARD, SP_EMU_FLASH (per-SP file), SP_EMU_BRIDGE=[::1]:<base>,
#     SP_EMU_NO_DEBUG=1 (no gdb/ocd listeners in production), SP_EMU_IDLE_MS.
launch() {
  local name="$1" board="$2" base="$3"
  local flash="$RUNDIR/$name-flash.bin" log="$RUNDIR/$name.log" img
  img="$(image_for "$board")"
  [ -s "$img" ] || { echo "[!] $name: image not found: $img"; return 1; }
  # Flash slot A once (persistent); reflash only if absent.
  [ -s "$flash" ] || SP_EMU_FLASH="$flash" "$SPEMU" flash a "$img" >/dev/null 2>&1
  # nohup so each SP survives independently of this supervisor (and of any ssh
  # session launching it). In production each SP is its own SMF instance anyway.
  SP_EMU_BOARD="$board" SP_EMU_FLASH="$flash" SP_EMU_BRIDGE="[::1]:$base" \
    SP_EMU_NO_DEBUG=1 SP_EMU_IDLE_MS="$IDLE_MS" \
    nohup "$SPEMU" gdb a "$PREBOOT" > "$log" 2>&1 < /dev/null &
  PIDS[$name]=$!
}

# (2) Readiness gate. Block until the SP prints its single quiet "online" marker
#     on stderr — no SP_EMU_BRIDGEDBG firehose needed. This is the signal
#     voxel-init blocks on before letting rack bring-up / MGS proceed.
wait_online() {
  local name="$1"
  local log="$RUNDIR/$name.log"
  for _ in $(seq 1 180); do
    grep -q '^\[sp-emu\] online' "$log" 2>/dev/null && return 0
    kill -0 "${PIDS[$name]}" 2>/dev/null || return 1   # process died during boot
    sleep 1
  done
  return 1   # timed out
}

# (3) Supervision/teardown. Restart any SP that dies (mirror SMF restart-on-fault),
#     and kill the whole fleet on exit.
cleanup() { for n in "${!PIDS[@]}"; do kill "${PIDS[$n]}" 2>/dev/null; done; }
trap cleanup EXIT INT TERM

for spec in "${FLEET[@]}"; do set -- $spec
  launch "$1" "$2" "$3" && echo "launching $1 ($2) on [::1]:$3 (switch0) / $(( $3 + 1 )) (switch1) → pid ${PIDS[$1]:-?}"
done
for spec in "${FLEET[@]}"; do set -- $spec
  if wait_online "$1"; then echo "[+] $1 ONLINE"; else echo "[!] $1 FAILED to come online — see $RUNDIR/$1.log"; exit 1; fi
done
echo "[+] fleet up: ${!PIDS[*]} — MGS can now reach each SP at [::1]:<base>/<base+1>"

while true; do
  for spec in "${FLEET[@]}"; do set -- $spec
    if ! kill -0 "${PIDS[$1]:-0}" 2>/dev/null; then
      echo "[!] $1 died — restarting"; launch "$1" "$2" "$3" && wait_online "$1" && echo "[+] $1 back online"
    fi
  done
  sleep 5
done
