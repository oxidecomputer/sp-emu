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
# Optional MGS-readiness probe. The bridge "online" marker means the SP's network
# is up; it does NOT mean control_plane_agent is answering MGS yet. If a faux-mgs
# binary is given, the readiness gate also blocks until a `discover` succeeds —
# that's true MGS-readiness, and it eliminates the first-contact request drop
# (the SP ignoring MGS's first packet) that otherwise shows up as a slow/failed
# first GET /ignition during rack inventory. voxel-init should mirror this.
FAUX_MGS="${FAUX_MGS:-$HOME/oxide/management-gateway-service/target/release/faux-mgs}"

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

# (2) Readiness gate. Two stages: (a) block until the SP prints its single quiet
#     "online" marker on stderr (network up), then (b) if faux-mgs is available,
#     block until a `discover` succeeds (control_plane_agent answering MGS). Stage
#     (b) is the real readiness signal — it's what voxel-init must block on before
#     letting RSS / MGS inventory proceed, and it removes the first-contact drop.
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
  [ -x "$FAUX_MGS" ] || { echo "    ($name: network up; no faux-mgs at \$FAUX_MGS, skipping discover gate)"; return 0; }
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
