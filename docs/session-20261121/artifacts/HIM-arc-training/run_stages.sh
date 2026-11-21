#!/usr/bin/env bash
set -euo pipefail
cd /root/as2w
root=$1
resume=()
stage=flat
offset=0
for end in $(seq 2000 2000 40000); do
 out="$root/u$end"
 ./preflight/isaac-python him_arc/run_quality.py --mode himloco --stage "$stage" --num-envs 4096 --rollout-steps 48 --iterations 2000 --save-interval 100 --asset calibration/candidates/load-v0/as2w-load-v0.urdf --out "$out" "${resume[@]}" >"$root/u$end.log" 2>&1
 for which in best endpoint; do
   src="$out/best.pt"
   [[ "$which" == endpoint ]] && src="$out/model_$end.pt"
   dest="$out/$which-eval"
   mkdir "$dest"
   cp "$src" "$dest/best.pt"; cp "$out/run.json" "$dest/"
   ./preflight/isaac-python him_arc/evaluate.py --mode himloco --snapshot "$dest" --terrain flat >"$dest/eval.log" 2>&1
   grep -q AS2W_EVAL_OK "$dest/eval.log"
   python3 him_arc/check.py "$dest"
 done
 chosen=$(python3 - "$out" "$root" <<'PY'
import sys,json
from pathlib import Path
out,root=map(Path,sys.argv[1:])
good=[p.parent for p in out.glob("*-eval/arc-acceptance.json") if json.loads(p.read_text())["passed"]]
if good:
 p=good[0]
 (root/"validated-best.json").write_text(json.dumps({"checkpoint":str(p/"best.pt"),"stage":json.loads((out/"run.json").read_text())["stage"]}))
 print(p/"best.pt")
PY
)
 if [[ -n "$chosen" ]]; then
   [[ "$stage" == randomized ]] && stage=rough
   [[ "$stage" == flat ]] && stage=randomized
 fi
 # Continue the endpoint optimizer and iteration. Gate only controls curriculum promotion.
 resume=(--resume "$out/model_$end.pt" --resume-update "$end")
done
