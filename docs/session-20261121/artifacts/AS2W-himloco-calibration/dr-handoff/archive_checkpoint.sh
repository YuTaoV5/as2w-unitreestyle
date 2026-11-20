#!/usr/bin/env bash
# Usage: bash archive_checkpoint.sh CHECKPOINT ASSET OUTPUT
set -euo pipefail
[[ $# == 3 ]] || exit 2
checkpoint=$(realpath -e "$1")
asset=$(realpath -e "$2")
out=$(realpath -m "$3")
mkdir "$out"
cd /root/as2w
tools="$PWD/him_arc/dr-handoff"
py=(./preflight/isaac-python)
for subset in nominal holdout; do
    mkdir "$out/$subset"
    ln -s "$checkpoint" "$out/$subset/best.pt"
    python3 - "$asset" "$out/$subset/run.json" <<'PY'
import json,sys
from pathlib import Path
Path(sys.argv[2]).write_text(json.dumps({"asset":sys.argv[1]}))
PY
    flags=()
    if [[ "$subset" == holdout ]]; then flags=(--holdout-handoff); fi
    "${py[@]}" him_arc/evaluate_archive.py --mode himloco --terrain flat --seed 319 \
      --snapshot "$out/$subset" "${flags[@]}" >"$out/$subset/eval.log" 2>&1
    "${py[@]}" "$tools/archive_curves.py" "$out/$subset/flat.npz" --representative
done
"${py[@]}" "$tools/score_holdout.py" "$out/holdout/flat.npz" "$out/holdout-metrics.json"
python3 - "$checkpoint" "$asset" "$out" <<'PY'
import hashlib,json,sys
from pathlib import Path
checkpoint,asset,out=map(Path,sys.argv[1:])
def digest(p):
    with p.open("rb") as stream:
        return hashlib.file_digest(stream,"sha256").hexdigest()
manifest={"checkpoint":str(checkpoint),"sha256":digest(checkpoint),
          "asset":str(asset),"asset_sha256":digest(asset),
          "seed":319,"dt_s":.02,"evaluation_complete":True,
          "acceptance_status":"pending_review"}
(out/"archive.json").write_text(json.dumps(manifest,indent=2)+"\n")
PY
