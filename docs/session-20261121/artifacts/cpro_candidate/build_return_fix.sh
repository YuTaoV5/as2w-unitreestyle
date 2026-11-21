#!/bin/bash
set -euo pipefail
base=/root/as2w_stand_fix.hAOR9Y
candidate=$(mktemp -d /root/as2w_return_fix.XXXXXX)
echo "CANDIDATE=$candidate"
cd "$base"
cp -a CMakeLists.txt deploy src "$candidate/"
ln -s /root/as2w_deploy/models /root/as2w_deploy/config "$candidate/"
cd "$candidate"
python3 - <<'PY'
from pathlib import Path
p = Path("deploy/robots/as2w/common/as2w_publish.cpp")
s = p.read_text()
start = s.index("                        // The last zero-command R214 target")
end = s.index("                        return_stand_started", start)
block = s[start:end]
assert "fixed_pose[i] = return_stand_start.motors[i].q;" in block
p.write_text(s[:start] + """#if defined(AS2W_ARMD_LP5W_CANDIDATE) && !defined(AS2W_ARMD_LP5W_JOYSTICK_CANDIDATE)
                        // Blend back to the fixed stance used before B.
#else
""" + block + "#endif\n" + s[end:])
PY
args=()
while IFS= read -r line; do
  case "$line" in
    ONNXRUNTIME_ROOT:*|unitree_sdk2_DIR:*|yaml-cpp_DIR:*|CMAKE_PREFIX_PATH:*|CMAKE_BUILD_TYPE:*) args+=("-D$line");;
  esac
done < "$base/build/CMakeCache.txt"
cmake -S . -B build "${args[@]}" -DAS2W_ARMD=ON -DAS2W_ARMD_PUBLISH=ON -DAS2W_ARMD_LP5W_CANDIDATE=ON
cmake --build build -j 3
ctest --test-dir build --output-on-failure
mkdir audit
cd audit
../build/deploy/robots/as2w/armd/as2w_armd_lp5w_publish ../models/armd-lp5/deploy.yaml ../config/r152_measured.map --offline-audit --zero-b-trial --stand t2 --interp-sec 3 --squat-sec 4 --stand-hold-seconds 15 --stand-kp 85 --stand-kd 3.2 --policy-lowpass-hz 5 --policy-lowpass-wheels --startup-wait-sec 30 --debug > audit.log 2>&1
grep -e AUDIT.PASS -e DEBUG_CAPTURE_AUDIT audit.log
sha256sum ../build/deploy/robots/as2w/armd/as2w_armd_lp5w_publish
echo "CANDIDATE_VERIFIED=$candidate"
