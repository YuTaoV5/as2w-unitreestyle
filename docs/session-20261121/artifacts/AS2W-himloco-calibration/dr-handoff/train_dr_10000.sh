#!/usr/bin/env bash
set -euo pipefail
cd /root/as2w
stage="${1:-}"
mode="${2:-}"
[[ "$mode" == --check || "$mode" == --start ]] || exit 2
dir="$PWD/him_arc/dr-handoff"
asset="$PWD/calibration/candidates/t2-v2-20261119/as2w-t2-v2.urdf"
seed=/root/gpufree-data/as2w-runs/him-flat-t2-v2-20261119-G6IW9O/train/model_9836.pt
update=9836
updates=$(python3 - "$dir/plan-10000.json" "$stage" <<'PY'
import json,sys
stages=json.load(open(sys.argv[1]))["stages"]
assert sum(x["updates"] for x in stages)==10000
print(next(x["updates"] for x in stages if x["name"]==sys.argv[2]))
PY
)
case "$stage" in
 field_reset) [[ $# == 2 ]] || exit 2 ;;
 load)
   [[ $# == 3 ]] || { echo 'Provide previous-stage acceptance manifest' >&2; exit 2; }
   relay=$(python3 - "$3" <<'PY'
import hashlib,json,sys
from pathlib import Path
d=json.load(open(sys.argv[1]))
assert d["stage"]=="field_reset" and d["passed"] is True
p=Path(d["checkpoint"]).resolve(strict=True)
assert hashlib.file_digest(p.open("rb"),"sha256").hexdigest()==d["sha256"]
assert type(d["completed_updates"]) is int and 9836<d["completed_updates"]<=10836
print(p)
print(d["completed_updates"])
PY
)
   mapfile -t fields <<<"$relay"
   seed="${fields[0]}"
   update="${fields[1]}"
   ;;
 *) echo "$stage awaits validated contact/timing integration" >&2; exit 3 ;;
esac
test -r "$asset"
test -r "$seed"
args=(--mode himloco --stage flat --dr-stage "$stage" --num-envs 4096 --rollout-steps 48 --iterations "$updates"
      --save-interval 100 --asset "$asset" --dr-dir "$dir" --hip-sign 1 --resume "$seed" --resume-update "$update")
if [[ "$mode" == --check ]]; then
 sha256sum "$asset" "$seed" "$dir/plan-10000.json"
 printf '%q ' ./preflight/isaac-python him_arc/run_dr.py "${args[@]}"
 printf '\n'
 exit
fi
r=$(mktemp -d "/root/gpufree-data/as2w-runs/him-dr10000-$stage-XXXXXX")
cp "$dir/plan-10000.json" "$dir/stages.json" "$dir/fixtures.json" "$r/"
archive_interval=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["curves_every"])' "$dir/plan-10000.json")
nohup bash -c '
set -euo pipefail
run=$1; tools=$2; asset=$3; start=$4; updates=$5; interval=$6
shift 6
./preflight/isaac-python him_arc/run_dr.py "$@" --out "$run/train" >"$run/train.log" 2>&1
python3 "$tools/select_milestones.py" "$run/train" "$start" "$updates" "$interval" >"$run/milestones.tsv"
mkdir -p "$run/evaluations"
while IFS=$'"'"'\t'"'"' read -r milestone checkpoint; do
    name=$(basename "$checkpoint" .pt)
    output="$run/evaluations/${milestone}_$name"
    bash "$tools/archive_checkpoint.sh" "$checkpoint" "$asset" "$output" >>"$run/evaluations/archive.log" 2>&1
done <"$run/milestones.tsv"
' bash "$r" "$dir" "$asset" "$update" "$updates" "$archive_interval" "${args[@]}" >"$r/worker.log" 2>&1 </dev/null &
echo $! >"$r/pid"
echo "$r"

