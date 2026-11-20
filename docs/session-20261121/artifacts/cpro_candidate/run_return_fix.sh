#!/bin/bash
set -euo pipefail
base=/root/as2w_deploy
candidate=/root/as2w_return_fix.MkQ7CU
bin="$candidate/build/deploy/robots/as2w/armd/as2w_armd_lp5w_publish"
sdk=(env LD_LIBRARY_PATH=/root/unitree_sdk2_install/lib "$base/build-sdk2/as2w_sdk2")
test -x "$bin"
mode=$("${sdk[@]}" motion-mode eth0)
printf '%s\n' "$mode"
grep -Eq 'CheckMode rc=0 .*name=[[:space:]]*$' <<< "$mode"
cd /root/field_sessions
session=$(mktemp -d "$(date +%Y%m%d-%H%M%S)_lp5w-return-fix.XXXXXX")
cd "$session"
echo "SESSION=$PWD"
"${sdk[@]}" low-capture eth0 rt/lowstate 2 200 > preflight.log 2>&1
python3 - <<'PY'
import csv
from pathlib import Path
rows = [r for r in csv.reader(Path("preflight.log").open()) if r and r[0] == "LOW_SAMPLE" and len(r) >= 224]
assert rows, "No valid LowState"
last = rows[-1]
assert int(last[2]) >= 1500, "Degraded LowState"
assert int(last[6]) == 2, "Unexpected mode_machine"
assert all(int(last[14 + i*6 + 4]) == 0 for i in range(16)), "Axis mode nonzero"
assert all(-3.0 < float(last[14 + i*6]) < -2.4 for i in (2,5,8,11)), "Robot not prone"
print("PRONE_AND_LOWSTATE_PASS", last[2], flush=True)
PY
if [[ ${1:-} == --preflight-only ]]; then exit 0; fi
nice -n 10 "${sdk[@]}" low-capture eth0 rt/lowstate 145 2 > low_capture_fast.log 2>&1 &
capture=$!
nice -n 10 "${sdk[@]}" low-protection eth0 145 > low_protection.log 2>&1 &
protection=$!
finish_monitors() {
  kill "$capture" "$protection" 2>/dev/null || true
  wait "$capture" "$protection" 2>/dev/null || true
}
trap finish_monitors EXIT
sleep 1
kill -0 "$capture" "$protection"
echo ZERO_B_SESSION_START
set +e
printf 'GO\n' | stdbuf -oL -eL "$bin" "$base/models/armd-lp5/deploy.yaml" "$base/config/r152_measured.map" eth0 \
  --i-confirm-lowcmd-publish --zero-b-trial --stand t2 --interp-sec 3 --squat-sec 4 \
  --stand-hold-seconds 15 --stand-kp 85 --stand-kd 3.2 --policy-lowpass-hz 5 --policy-lowpass-wheels --startup-wait-sec 30 --debug 2>&1 | tee publisher.log
result=${PIPESTATUS[1]}
echo "ZERO_B_TRIAL_REMOTE_RC=$result"
sleep 2
exit "$result"
