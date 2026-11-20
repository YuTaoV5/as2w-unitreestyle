"""Select retained checkpoints for milestone evaluation after a stage ends."""
import argparse
from pathlib import Path

def select(directory, start, updates, interval):
    end = start + updates
    snapshots = {}
    for path in directory.glob("model_*.pt"):
        suffix = path.stem.split("_",1)[1]
        if suffix.isdigit() and start < int(suffix) <= end and path.stat().st_size:
            snapshots[int(suffix)] = path
    if end not in snapshots:
        raise ValueError(f"Stage endpoint model_{end}.pt is missing")
    targets = sorted(set(range(start+interval, end+1, interval)) | {end})
    return [(target, snapshots[min(i for i in snapshots if i>=target)]) for target in targets]

if __name__ == "__main__":
    p=argparse.ArgumentParser()
    p.add_argument("directory",type=Path)
    p.add_argument("start",type=int)
    p.add_argument("updates",type=int)
    p.add_argument("interval",type=int)
    a=p.parse_args()
    for target,path in select(a.directory,a.start,a.updates,a.interval):
        print(f"{target}\t{path.resolve()}")
