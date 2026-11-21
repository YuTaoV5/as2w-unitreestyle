#!/bin/bash
set -euo pipefail
base=/root/as2w_deploy
source_file=deploy/robots/as2w/common/as2w_publish.cpp
staged=/tmp/as2w-stand-cap-v1.patch
candidate=$(mktemp -d /root/as2w_stand_fix.XXXXXX)
echo "CANDIDATE=$candidate"
cd "$base"
echo "cfe15d11984d606f346acc1699d39bad54e12bf5c6e9eefe9c71ccb541a69df1  $source_file" | sha256sum -c
cp -a CMakeLists.txt deploy src "$candidate/"
ln -s "$base/models" "$base/config" "$candidate/"
cd "$candidate"
patch --batch --forward --fuzz=0 -p1 < "$staged"
sha256sum "$source_file"
old="$base/build-armd-lp5w-joystick/CMakeCache.txt"
args=()
while IFS= read -r line; do
  case "$line" in
    ONNXRUNTIME_ROOT:*|unitree_sdk2_DIR:*|yaml-cpp_DIR:*|CMAKE_PREFIX_PATH:*|CMAKE_BUILD_TYPE:*) args+=("-D$line");;
  esac
done < "$old"
cmake -S . -B build "${args[@]}" -DAS2W_ARMD=ON -DAS2W_ARMD_PUBLISH=ON -DAS2W_ARMD_LP5W_JOYSTICK_CANDIDATE=ON
cmake --build build -j 3
ctest --test-dir "$candidate/build" --output-on-failure
bin="$candidate/build/deploy/robots/as2w/armd/as2w_armd_lp5w_joystick_publish"
"$bin" models/armd-lp5/deploy.yaml config/r152_measured.map --offline-audit --policy-lowpass-hz 5 --policy-lowpass-wheels --stand-kp 85 --stand-kd 3.2
sha256sum "$bin" models/armd-lp5/policy.onnx
echo "CANDIDATE_VERIFIED=$candidate"
