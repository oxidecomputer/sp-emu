#!/usr/bin/env bash
# run-fleet.sh — launch + gate + supervise a fleet of emulated SPs for a4x2/voxel.
#
# Reference for what voxel-init's launch+gate mirrors in Rust:
#   (1) canonical per-SP launch line, (2) the readiness gate, (3) supervision.
#
# Each emulated SP is one process running one Hubris image, binding a switch0/
# switch1 port pair on loopback (in oxz_switch, MGS reaches it on [::1] as with
# sp-sim). Edit FLEET below (or translate this logic) for other topologies.

set -uo pipefail

# Resolve the sp-emu binary relative to this script's directory; works regardless
# of checkout location. Override SPEMU / HUBRIS in the environment.
HERE="$(cd -- "$(dirname "${BASH_SOURCE[0]:-$0}")" >/dev/null 2>&1 && pwd -P)"
ROOT="$(cd -- "$HERE/.." >/dev/null 2>&1 && pwd -P)"

SPEMU="${SPEMU:-$ROOT/target/release/sp-emu}"
# Hubris images come from the separate hubris repo; set $HUBRIS to that checkout.
HUBRIS="${HUBRIS:-}"
[ -n "$HUBRIS" ] || { echo "[!] Set \$HUBRIS to your hubris checkout (for the per-board images)" >&2; exit 1; }
RUNDIR="${RUNDIR:-/var/tmp/sp-emu}"          # flash files + logs live here
IDLE_MS="${SP_EMU_IDLE_MS:-20}"              # idle-CPU vs latency knob
PREBOOT="${PREBOOT:-340000000}"              # instructions to steady state (~17s)

# Optional MGS-readiness probe. The bridge "online" marker means the SP's network
# is up; it does NOT mean control_plane_agent is answering MGS yet. If a faux-mgs
# binary is given, the readiness gate also blocks until a `discover` succeeds, which
# is true MGS-readiness and eliminates the first-contact request drop (the SP
# ignoring MGS's first packet) that otherwise shows up as a slow/failed first GET
# /ignition during rack inventory.
# faux-mgs is from the separate management-gateway-service repo; set $FAUX_MGS to
# that build to enable the MGS-readiness gate, else it's skipped (network-up only).

FAUX_MGS="${FAUX_MGS:-}"

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

# (1) Canonical per-SP launch. 
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
  # session launching it). In production each SP is its own SMF instance.
  SP_EMU_BOARD="$board" SP_EMU_FLASH="$flash" SP_EMU_BRIDGE="[::1]:$base" \
    SP_EMU_NO_DEBUG=1 SP_EMU_IDLE_MS="$IDLE_MS" \
    nohup "$SPEMU" gdb a "$PREBOOT" > "$log" 2>&1 < /dev/null &
  PIDS[$name]=$!
}

# (2) Readiness gate. Two stages: (a) block until the SP prints its single quiet
#     "online" marker on stderr (network up), then (b) if faux-mgs is available,
#     block until a `discover` succeeds (control_plane_agent answering MGS). 

wait_online() {
  local name="$1" base="$2"
  local log="$RUNDIR/$name.log"
  # (a) network up
  local up=""
  for _ in $(seq 1 180); do
    grep -q '^\[sp-emu\] online' "$log" 2>/dev/null && { up=1; break; }
    kill -0 "${PIDS[$name]}" 2>/dev/null || return 1   # process died during boot
    sleep 1
  done
  [ -n "$up" ] || return 1
  # (b) MGS-ready: discover succeeds. Skipped if no faux-mgs binary is present.
  [ -x "$FAUX_MGS" ] || { echo "    ($name: network up; \$FAUX_MGS unset, skipping discover gate)"; return 0; }
  for _ in $(seq 1 60); do
    "$FAUX_MGS" --sp-sim-addr "[::1]:$base" --max-attempts 2 --per-attempt-timeout-millis 5000 \
      discover >/dev/null 2>&1 && return 0
    kill -0 "${PIDS[$name]}" 2>/dev/null || return 1
    sleep 1
  done
  return 1   # network came up but MGS never answered
}

# (3) Supervision/teardown. Restart any SP that dies (mirror SMF restart-on-fault),
#     and kill the whole fleet on exit.
cleanup() { for n in "${!PIDS[@]}"; do kill "${PIDS[$n]}" 2>/dev/null; done; }
trap cleanup EXIT INT TERM

for spec in "${FLEET[@]}"; do set -- $spec
  launch "$1" "$2" "$3" && echo "launching $1 ($2) on [::1]:$3 (switch0) / $(( $3 + 1 )) (switch1) → pid ${PIDS[$1]:-?}"
done
for spec in "${FLEET[@]}"; do set -- $spec
  if wait_online "$1" "$3"; then echo "[+] $1 READY (MGS-reachable)"; else echo "[!] $1 FAILED to come ready — see $RUNDIR/$1.log"; exit 1; fi
done
echo "[+] fleet up: ${!PIDS[*]} — MGS can now reach each SP at [::1]:<base>/<base+1>"

while true; do
  for spec in "${FLEET[@]}"; do set -- $spec
    if ! kill -0 "${PIDS[$1]:-0}" 2>/dev/null; then
      echo "[!] $1 died — restarting"; launch "$1" "$2" "$3" && wait_online "$1" "$3" && echo "[+] $1 back online"
    fi
  done
  sleep 5
done
