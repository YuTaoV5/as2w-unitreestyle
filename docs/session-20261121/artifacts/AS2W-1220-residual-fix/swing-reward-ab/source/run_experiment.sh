#!/usr/bin/env bash
set -euo pipefail
cd /root/as2w
r=$1
source_run=$2
baseline=$3
py=/opt/isaaclab-venv/bin/python
export PYTHONUNBUFFERED=1
wait_gpu() {
    local min=$1
    while (( $(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1) < min )); do
        sleep 15
    done
}
for i in $(seq 1 60); do
    [[ -s "$r/smoke/endpoint.pt" ]] && break
    if [[ -s "$r/smoke.pid" ]] && ! kill -0 "$(cat "$r/smoke.pid")" 2>/dev/null; then
        if [[ ! -s "$r/smoke/endpoint.pt" ]]; then tail -40 "$r/smoke.log"; exit 1; fi
    fi
    sleep 5
done
test -s "$r/smoke/endpoint.pt"
for arm in A B C; do
    wait_gpu 11000
    echo "START_TRAIN $arm $(date -Is)"
    ./preflight/isaac-python swing_ab_mesh/train.py --arm "$arm" --root "$source_run" --out "$r/$arm" \
        --iterations 50 --envs 4096 --calibration "$r/calibration.json" >"$r/train-$arm.log" 2>&1
    test -s "$r/$arm/endpoint.pt"
    cmp "$r/A/initial.sha256" "$r/$arm/initial.sha256"
    echo "DONE_TRAIN $arm $(date -Is)"
    for n in 25 50; do
        d="$r/eval-$n/$arm"
        mkdir -p "$d"
        ln "$r/$arm/update-$n.pt" "$d/best.pt"
        cp "$r/$arm/run.json" "$d/"
        wait_gpu 8200
        ./preflight/isaac-python swing_ab_mesh/evaluate_zero_warmup.py --mode unitree_style --terrain flat --snapshot "$d" >"$d/replay.log" 2>&1
        grep -q AS2W_EVAL_OK "$d/replay.log"
        env -u LD_LIBRARY_PATH "$py" quality/summarize_contact.py "$d" >"$d/summary.log"
        python3 continuation1220/gate.py "$baseline" "$d" >"$d/gate.log"
        PYTHONPATH=swing_ab_mesh python3 - "$baseline" "$d" <<'PY'
import sys
from pathlib import Path
from paired import compare
a,b=map(Path,sys.argv[1:])
g=compare(a,b)
print(b.parent.name,b.name,"PASS" if g["gait_preserved"] else "FAIL",g["failures"],flush=True)
PY
        env -u LD_LIBRARY_PATH "$py" swing_ab_mesh/measure.py report "$d/flat.npz" "$d/swing-report.json" "$r/calibration.json"
        echo "DONE_EVAL $arm $n $(date -Is)"
    done
done
echo "EXPERIMENT_COMPLETE $(date -Is)"
