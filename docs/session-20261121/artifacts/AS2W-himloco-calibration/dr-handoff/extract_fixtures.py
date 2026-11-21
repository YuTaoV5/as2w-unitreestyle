"""Generate whole-frame reset fixtures in raw DDS contract order."""
import argparse
import csv
import json
from pathlib import Path

SLOTS = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8, 13, 12, 15, 14]

def extract(path, start, end, stride=100):
    frames = []
    with path.open() as stream:
        for row in csv.reader(stream):
            if len(row) < 110 or row[0] != "LOW_SAMPLE":
                continue
            elapsed = int(row[1])
            if not start <= elapsed < end or elapsed % stride >= 2:
                continue
            frames.append({
                "tick": int(row[4]), "elapsed_ms": elapsed,
                "quat_wxyz": list(map(float, row[7:11])),
                "gyro_body": list(map(float, row[11:14])),
                "q_dds": [float(row[14+6*i]) for i in SLOTS],
                "dq_dds": [float(row[15+6*i]) for i in SLOTS],
            })
    if not frames:
        raise ValueError(f"No valid frames: {path}")
    return frames

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", type=Path, help="Workspace calibration folder containing retrieved logs")
    parser.add_argument("v2", type=Path, help="V2 data package root")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    sessions = []
    for name, lo, hi, split in [
        ("182802", 10000, 30000, "train"),
        ("185930", 7000, 20000, "train"),
        ("190012", 8000, 20000, "holdout"),
    ]:
        path = next((args.v2/"recordings/field_sessions").glob(f"*{name}*"))/"low_capture_fast.log"
        sessions.append(dict(name=name, split=split, frames=extract(path,lo,hi)))
    for stamp, split in [("20261119-114540","train"), ("20261119-190857","holdout")]:
        policy_root = Path("X:/Items/as2w/himloco_field_20261119")
        name = "vx015_walked" if split=="train" else "vx010_X_abort_handoff"
        with (policy_root/name/"debug_policy.csv").open() as stream:
            tick = next(int(r[3]) for r in csv.reader(stream) if r and r[0]=="DEBUG_POLICY" and r[2]=="HANDOFF")
        path = args.logs/f"{stamp}-low_capture_fast.log"
        with path.open() as stream:
            elapsed = next(int(r[1]) for r in csv.reader(stream) if len(r)>=110 and r[0]=="LOW_SAMPLE" and int(r[4])>=tick)
        sessions.append(dict(name=name,split=split,frames=extract(path,max(0,elapsed-1000),elapsed)))
    args.output.write_text(json.dumps({"dds_slots":SLOTS,"sessions":sessions},indent=2),encoding="utf-8")

if __name__ == "__main__":
    main()

