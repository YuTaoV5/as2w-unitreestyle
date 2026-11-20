"""Publish the two matched A/B/C reports after all replays for each stage finish."""
import json
from pathlib import Path
from statistics import mean
import sys
import time

import numpy as np
from measure import MODES,WHEELS

def load(path):
    return json.loads(path.read_text())

def score(base, reference, path):
    g=load(path/"gate.json")
    swing=load(path/"swing-report.json")
    with np.load(base/"flat.npz") as ref, np.load(path/"flat.npz") as test:
        assert np.array_equal(ref["labels"],test["labels"])
        straight=np.isin(ref["labels"],["stand","forward","reverse"])
        rolling_exact=all(np.array_equal(ref[k][:,straight],test[k][:,straight])
            for k in ("q","wheel_data","target_trace","vel"))
    rows=[swing[m][w] for m in MODES for w in WHEELS if reference[m][w]["eligible"]]
    phases=[g["case_metrics"][m]["phase_candidate"]["measured_residual_rms_deg"]/
            g["case_metrics"][m]["phase_baseline"]["measured_residual_rms_deg"]
            for m in MODES if g["case_metrics"][m].get("phase_candidate") and
            g["case_metrics"][m].get("phase_baseline")]
    failures=list(g["failures"])
    if not rolling_exact:
        failures.append("rolling traces differ from frozen baseline")
    return {"passes":not failures,"failures":failures,"rolling_exact":rolling_exact,
            "calibrated_events":sum(r["calibrated_events"] for r in rows),
            "mean_high_frequency_ratio":mean(phases) if len(phases)==4 else None,
            "per_mode":swing}

def main(root,base):
    meta=load(root/"calibration.json")
    baseline=load(root/"baseline-report.json")
    counts=sum(baseline[m][w]["calibrated_events"] for m in MODES for w in WHEELS if meta[m][w]["eligible"])
    for n in (25,50):
        stage=root/f"eval-{n}"
        paths={k:stage/k for k in "ABC"}
        ready=[p/"swing-report.json" for p in paths.values()]
        while not all(p.exists() and p.stat().st_size>0 for p in ready):
            time.sleep(15)
        out={"baseline_calibrated_events":counts,"baseline":baseline,
             "arms":{k:score(base,meta,p) for k,p in paths.items()}}
        tmp=stage/"comparison.tmp"
        tmp.write_text(json.dumps(out,indent=2)+"\n")
        tmp.replace(stage/"comparison.json")
        print("STAGE",n,[(k,row["passes"],row["calibrated_events"]) for k,row in out["arms"].items()],
              flush=True)

if __name__=="__main__":
    main(*map(Path,sys.argv[1:]))
