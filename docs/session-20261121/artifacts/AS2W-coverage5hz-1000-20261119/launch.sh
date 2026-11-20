#!/usr/bin/env bash
set -euo pipefail
export PYTHONUNBUFFERED=1
cd /root/as2w/coverage5hz_20261119-103318
/root/as2w/preflight/isaac-python /root/as2w/coverage5hz_20261119-103318/run.py --out /root/gpufree-data/as2w-runs/unitree-coverage5hz-20261119-103318/train --asset /root/as2w/calibration/candidates/load-v0/as2w-load-v0.urdf --num-envs 4096 --iterations 1000 --rollout-steps 48 --seed 42 --dr-level 0 --resume /root/gpufree-data/as2w-runs/unitree-unified-reward-20261119-010208/train/model_1000.pt --resume-update 0
