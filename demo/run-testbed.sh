#!/usr/bin/env bash
# Boot the canonical sp-emu testbed: a two-core SP+RoT instance serving forever,
# with the MGS bridge, an IPCC pty, and a SWD debug probe per core over TCP.
#
# This is the configuration that looks most like a real board, and the one to
# use for humility, faux-mgs, faux-ipcc, and test harnesses:
#
#   * SP Hubris on an emulated STM32H753, RoT Hubris on an emulated LPC55S69,
#     wired over the sprot SPI link in one process (SP_EMU_ROT_FLASH), so
#     rot_state answers with real digests instead of timing out.
#   * Optional real bootleby at RoT flash 0x0 (BOOTLEBY=...), which does genuine
#     A/B slot selection. Without it the bootloader slots hold sp-emu's synthetic
#     caboose-only image: stage0 caboose reads answer, but nothing boots them.
#   * SWD probes speaking the Glasgow protocol over TCP: SP on :4444, RoT on
#     :4544. Attach humility with `-p 20b7:9db1:tcp:127.0.0.1:<port>`.
#   * The host/SP IPCC UART as a pty, for faux-ipcc.
#
# Usage:
#   ./run-testbed.sh <sp-archive.zip> <rot-selfsigned-archive.zip>
#   BOOTLEBY=<bootleby.zip> ./run-testbed.sh <sp-archive> <rot-archive>
#
# Environment:
#   SPEMU        sp-emu binary (default: ../target/release/sp-emu)
#   STATE_DIR    instance state (default: /tmp/sp-emu-testbed)
#   ADDR0        host address the MGS view binds (default: ::1)
#   BOOTLEBY     bootleby build archive to boot instead of jumping to the image
#   FRESH=1      discard persisted RoT flash and reseed this run
#
# The RoT archive must be a SELF-SIGNED oxide-rot-1 build
# (build-oxide-rot-1-selfsigned-image-a.zip): it is Bart-signed so the emulated
# boot ROM authenticates it, and uses the dice-self identity path. The production
# dice-mfg image wedges polling the unmodeled manufacturing USART.

set -euo pipefail

die() { echo "run-testbed.sh: $*" >&2; exit 1; }

HERE="$(cd -- "$(dirname "${BASH_SOURCE[0]:-$0}")" >/dev/null 2>&1 && pwd -P)"
ROOT="$(cd -- "$HERE/.." >/dev/null 2>&1 && pwd -P)"

SPEMU="${SPEMU:-$ROOT/target/release/sp-emu}"
STATE_DIR="${STATE_DIR:-/tmp/sp-emu-testbed}"
ADDR0="${ADDR0:-::1}"

sp_archive="${1:-}"
rot_archive="${2:-}"
[ -n "$sp_archive" ] && [ -n "$rot_archive" ] \
    || die "usage: run-testbed.sh <sp-archive.zip> <rot-selfsigned-archive.zip>"
[ -f "$sp_archive" ]  || die "SP archive not found: $sp_archive"
[ -f "$rot_archive" ] || die "RoT archive not found: $rot_archive"
[ -x "$SPEMU" ] || die "sp-emu not built at $SPEMU (RUSTC_BOOTSTRAP=1 cargo build --release)"

log="$STATE_DIR/sp-emu.log"
mkdir -p "$STATE_DIR"

# All persistent state (SP flash, RoT flash + its erased bitset, the .nv
# sidecars, the derived identity, stowed archives) lives under here, so a run is
# reproducible and `rm -rf "$STATE_DIR"` is a full factory reset.
export SP_EMU_STATE_DIR="$STATE_DIR"
export SP_EMU_FLASH="$STATE_DIR/sp-flash.bin"

echo "run-testbed.sh: flashing SP $(basename "$sp_archive")" >&2
"$SPEMU" flash a "$sp_archive" >"$STATE_DIR/flash.log" 2>&1 \
    || { cat "$STATE_DIR/flash.log" >&2; die "SP flash failed"; }

env=(
    SP_EMU_HOST_PTY=1              # IPCC UART as a pty, for faux-ipcc
    SP_EMU_WELL_KNOWN_PORTS=1      # bind the SP's real ports (11111 MGS, ...)
    SP_EMU_ADDR0="$ADDR0"
    SP_EMU_ROT_FLASH="$rot_archive"
    SP_EMU_ROT_ROM=1               # boot-ROM skboot_authenticate; required in serve mode
)
[ -n "${FRESH:-}" ] && env+=(SP_EMU_ROT_FRESH=1)

# stage0: sp-emu boots the RoT through real bootleby by default, finding
# bootleby-oxide-rot-1.zip next to the RoT archive or under $HUBRIS; that is what
# performs A/B slot selection and honors the CFPA persistent boot preference.
# BOOTLEBY names one explicitly. The CMPA/CFPA shipped in a bootleby archive are
# not seeded: sp-emu's synthesized CMPA is byte-identical to a real oxide-rot-1's
# (Bart keyset, debug-open DCFG_CC_SOCU, unsealed).
if [ -n "${BOOTLEBY:-}" ]; then
    [ -f "$BOOTLEBY" ] || die "bootleby archive not found: $BOOTLEBY"
    env+=(SP_EMU_ROT_BOOTLEBY="$BOOTLEBY")
fi

# `run a 0` is the serve-forever SWD mode (not the gdb stub): it wires the
# in-process RoT and exposes both probes. nohup (not setsid) keeps $! as sp-emu's
# own pid so the printed stop command works.
: >"$log"
env "${env[@]}" nohup "$SPEMU" run a 0 >"$log" 2>&1 &
pid=$!
disown "$pid" 2>/dev/null || true

# Boot time is host-dependent: the pair is two emulated cores booting real
# kernels, roughly ten seconds on a fast laptop and several times that on a
# loaded or slower machine. Poll for readiness rather than sleeping a fixed
# amount, and do not bake a specific number into anything downstream.
pty=""
for _ in $(seq 1 240); do
    if ! kill -0 "$pid" 2>/dev/null; then
        cat "$log" >&2
        die "sp-emu exited during boot (see $log)"
    fi
    if grep -q 'online:' "$log" && grep -q 'pty ready:' "$log"; then
        pty=$(grep -oE '/dev/pts/[0-9]+' "$log" | head -1)
        break
    fi
    sleep 0.5
done
# Stop the emulator we started before giving up. It was nohup'd and disowned, so
# it otherwise survives holding the well-known MGS and both SWD ports, and the
# next run fails the port check below blaming a stale instance -- manufacturing
# the very condition that check exists to report.
if [ -z "$pty" ]; then
    tail -20 "$log" >&2
    kill "$pid" 2>/dev/null || true
    die "sp-emu did not become ready (see $log)"
fi

# In well-known-port mode a stale instance holding the ports does not stop the
# boot: sp-emu skips the ports it cannot bind and still reports online, leaving
# the SP unreachable. Say so here instead of letting every faux-mgs call hang.
if grep -qE 'skip SP port 11111.*Address already in use|well-known-port mode: 0 socket' "$log"; then
    kill "$pid" 2>/dev/null || true
    die "no MGS socket bound on 11111; another instance probably holds it (check 'pgrep -a sp-emu')"
fi

cat <<EOF
run-testbed.sh: online (pid $pid)

  MGS         [$ADDR0]:11111        faux-mgs --sp-sim-addr '[$ADDR0]:11111' state
  SP probe    tcp:127.0.0.1:4444    humility -a $(basename "$sp_archive") -p 20b7:9db1:tcp:127.0.0.1:4444 tasks
  RoT probe   tcp:127.0.0.1:4544    humility -a $(basename "$rot_archive") -p 20b7:9db1:tcp:127.0.0.1:4544 tasks
  IPCC pty    $pty
  state       $SP_EMU_STATE_DIR     (rm -rf to factory-reset)
  log         $log
  stop        kill $pid

faux-mgs on loopback must use --sp-sim-addr: the --interface/--discovery-addr
form cannot resolve a loopback peer's interface index and silently discards the
reply. Its gateway-messages revision must also match the SP image's, or
discovery fails with a version mismatch.

Attaching a SWD probe asserts JTAG_DETECT on the RoT, which invalidates the
attestation log. That is expected, and true of a real probe on real hardware.
EOF
