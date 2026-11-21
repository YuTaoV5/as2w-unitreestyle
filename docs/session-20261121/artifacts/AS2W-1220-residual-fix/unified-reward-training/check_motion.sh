#!/usr/bin/env bash
set -euo pipefail
cd /root/as2w
run=$1
base=$2
score() {
  local p=$1
  python3 continuation1220/gate.py "$base" "$p" >"$p/gate.log"
  PYTHONPATH=residual_original_2k python3 - "$base" "$p" <<'PY'
import json,sys
from pathlib import Path
from paired import compare
base,d=map(Path,sys.argv[1:])
g=compare(base,d)
reports=json.loads((d/"flat.json").read_text())
for c in ("left","right","yaw_left","yaw_right"):
    if reports[c]["wheel_target"]>1e-4:
        g["failures"].append(c+": nonzero wheel target")
g["gait_preserved"]=not g["failures"]
(d/"gate.json").write_text(json.dumps(g,indent=2)+"\n")
print(d.name, "PASS" if g["gait_preserved"] else "FAIL", g["failures"], flush=True)
PY
}
for label in 500 1000 $(seq 2000 1000 9000) 9999; do
  cp="$run/train/model_$label.pt"
  while ! test -s "$cp"; do
    if test -f "$run/exit-code"; then exit 0; fi
    sleep 30
  done
  sleep 4
  p="$run/intent-audit-u$((label+1))"
  mkdir -p "$p"
  ln "$cp" "$p/best.pt"
  cp "$run/train/run.json" "$p/run.json"
  if ./preflight/isaac-python evaluation/record_fixed_command_zero_warmup.py --mode unitree_style --terrain flat --snapshot "$p" >"$p/replay.log" 2>&1 &&
    grep -q AS2W_EVAL_OK "$p/replay.log"; then
    env -u LD_LIBRARY_PATH /opt/isaaclab-venv/bin/python quality/summarize_contact.py "$p" >"$p/summary.log"
    score "$p"
  else
    printf '%s\n' "REPLAY_ERROR $p"
  fi
done
