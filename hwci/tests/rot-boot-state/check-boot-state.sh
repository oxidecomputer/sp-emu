#!/bin/bash
# Query RoT boot state through the SP's control-plane-agent and verify the RoT
# reports an active image slot. This is the read path a bootleby A/B-selection
# assertion builds on; here we only check reachability + a plausible active slot.
set -euo pipefail
source "${SP_TEST_LIB:?}/sp-test.sh"

sp_step "Querying SP state (includes RoT boot state)"
state=$(sp_faux_mgs state)
sp_assert_json_field "$state" '.[].Ok' "SP state response Ok"

sp_step "Extracting RoT boot state"
# V2 and V3 state formats carry the RoT boot state under .rot.
rot_state=$(echo "$state" | jq '.[].Ok | .V2.rot // .V3.rot // empty' 2>/dev/null)
if [ -z "$rot_state" ] || [ "$rot_state" = "null" ]; then
    # No RoT boot state: sp-emu was started without SP_EMU_ROT_FLASH, or the SP
    # reports V1 state. Not applicable rather than a failure.
    sp_skip "SP response carries no RoT boot state (no emulated RoT, or V1 format)"
fi
sp_log INFO "RoT boot state" raw="$(echo "$rot_state" | jq -c .)"

sp_step "Checking RoT active slot"
active=$(echo "$rot_state" | jq -r '.Ok.active // empty' 2>/dev/null)
if [ -z "$active" ] || [ "$active" = "null" ]; then
    sp_fail "RoT boot state present but reports no active slot"
fi
sp_log INFO "RoT active slot" slot="$active"

# The active slot must name one of the two image banks.
case "$active" in
    A|B|0|1|SlotA|SlotB) : ;;
    *) sp_fail "RoT active slot is not a recognized bank" slot="$active" ;;
esac

sp_pass "RoT boot state accessible via SP; active slot=$active"
