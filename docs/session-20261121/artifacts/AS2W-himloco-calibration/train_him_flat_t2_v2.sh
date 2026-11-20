#!/usr/bin/env bash
set -euo pipefail
cd /root/as2w
asset="$PWD/calibration/candidates/t2-v2-20261119/as2w-t2-v2.urdf"
seed=/root/gpufree-data/as2w-runs/him-stairs-20261119-013122/train/model_7836.pt
test -r "$asset"
test -r "$seed"
args=(--mode himloco --stage flat --num-envs 4096 --rollout-steps 48 --iterations 2000 --save-interval 100 --asset "$asset" --resume "$seed" --resume-update 7836)
if [[ "${1:-}" == --check ]]; then
  bash -n "$0"
  sha256sum "$asset" "$seed"
  printf '%q ' ./preflight/isaac-python him_arc/run_quality.py "${args[@]}"
  printf '\n'
  exit
fi
[[ "${1:-}" == --start ]] || { echo 'Use --check or --start'; exit 2; }
r=$(mktemp -d /root/gpufree-data/as2w-runs/him-flat-t2-v2-20261119-XXXXXX)
cp "$asset" "$r/"
nohup ./preflight/isaac-python him_arc/run_quality.py "${args[@]}" --out "$r/train" >"$r/train.log" 2>&1 </dev/null &
echo $! >"$r/pid"
echo "$r"
