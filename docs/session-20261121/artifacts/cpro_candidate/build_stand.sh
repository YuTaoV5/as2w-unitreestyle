#!/bin/bash
set -euo pipefail
cd /root/as2w_stand_fix.hAOR9Y
file=deploy/robots/as2w/armd/CMakeLists.txt
if ! grep -q 'add_executable(as2w_armd_lp5w_stand_publish' "$file"; then
cat >> "$file" <<'CMAKE'

if(AS2W_ARMD_LP5W_JOYSTICK_CANDIDATE)
  add_executable(as2w_armd_lp5w_stand_publish ../common/as2w_publish.cpp)
  target_compile_definitions(as2w_armd_lp5w_stand_publish PRIVATE
    AS2W_POLICY_ARMD=1 AS2W_ENABLE_POLICY_STAND_TRIAL=1
    AS2W_ENABLE_POLICY_LOWPASS=1 AS2W_ARMD_LP5_CANDIDATE=1
    AS2W_ARMD_LP5W_CANDIDATE=1)
  target_link_libraries(as2w_armd_lp5w_stand_publish PRIVATE
    as2w_armd_lp5w_joystick unitree_sdk2 Threads::Threads)
endif()
CMAKE
fi
cmake --build build --target as2w_armd_lp5w_stand_publish -j 2 || {
  cmake -S . -B build
  cmake --build build --target as2w_armd_lp5w_stand_publish -j 2
}
bin=build/deploy/robots/as2w/armd/as2w_armd_lp5w_stand_publish
args=(models/armd-lp5/deploy.yaml config/r152_measured.map)
"$bin" "${args[@]}" --offline-audit --stand-trial --stand t2 --stand-hold-seconds 5 --stand-kp 85 --stand-kd 3.2 --policy-lowpass-hz 5 --policy-lowpass-wheels > build/stand-audit.log 2>&1
grep 'AUDIT PASS' build/stand-audit.log
sha256sum "$bin"
echo STAND_BUILD_VERIFIED
