#!/usr/bin/env bash
set -euo pipefail
cd /root/as2w
r=$1
base=$(cat "$r/seed.txt")
./preflight/isaac-python him_stairs/run_quality.py --mode himloco --num-envs 4096 --rollout-steps 48 --iterations 2000 --save-interval 100 --resume "$base" --resume-update 5836 --asset calibration/candidates/load-v0/as2w-load-v0.urdf --out "$r/train" >"$r/train.log" 2>&1
grep -q AS2W_TRAIN_SMOKE_OK "$r/train.log"
for which in best endpoint; do
 d="$r/$which-eval";mkdir "$d"
 src="$r/train/best.pt"; [[ "$which" == endpoint ]] && src="$r/train/model_7836.pt"
 cp "$src" "$d/best.pt";cp "$r/train/run.json" "$d/"
 ./preflight/isaac-python him_stairs/evaluate.py --mode himloco --terrain flat --snapshot "$d" >"$d/eval.log" 2>&1
done
