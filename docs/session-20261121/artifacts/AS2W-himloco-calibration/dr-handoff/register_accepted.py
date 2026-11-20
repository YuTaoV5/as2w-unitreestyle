"""Publish a reviewed acceptance manifest without changing candidate weights."""
import argparse
import hashlib
import json
import os
from pathlib import Path

def digest(path):
    with path.open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

def main():
    p = argparse.ArgumentParser()
    p.add_argument("archive", type=Path)
    p.add_argument("review", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    archived = json.loads((a.archive / "archive.json").read_text())
    review = json.loads(a.review.read_text())
    if archived.get("evaluation_complete") is not True or review.get("passed") is not True:
        p.error("Complete evaluation and passed review required")
    checkpoint = Path(archived["checkpoint"]).resolve(strict=True)
    if digest(checkpoint) != archived["sha256"] or review.get("sha256") != archived["sha256"]:
        p.error("Checkpoint differs from archive or review")
    gates = ("joint_angles","wheel_support","pitch","tracking","stopping","no_nonwheel_support")
    if any(review.get("gates", {}).get(gate) is not True for gate in gates):
        p.error("Review must pass every gate")
    for fn in ("nominal/flat.npz","holdout/flat.npz","holdout-metrics.json"):
        if not (a.archive/fn).is_file():
            p.error(f"Missing archive evidence: {fn}")
    if type(review.get("completed_updates")) is not int or review["completed_updates"] <= 9836:
        p.error("A trained checkpoint update is required")
    result = {**review,"checkpoint":str(checkpoint),"archive":str(a.archive.resolve()),
              "asset_sha256":archived["asset_sha256"]}
    temp = a.output.with_name(f".{a.output.name}.{os.getpid()}.tmp")
    with temp.open("x",encoding="utf-8") as f:
        json.dump(result,f,indent=2)
        f.write("\n")
    os.replace(temp,a.output)

if __name__ == "__main__":
    main()
