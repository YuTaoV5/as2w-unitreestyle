"""Score fixed 100/150/50-step HIMLoco holdout trajectories."""
import argparse
import json
from pathlib import Path
import numpy as np

def longest(mask):
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    return int((np.flatnonzero(edges == -1) - np.flatnonzero(edges == 1)).max(initial=0))

def score(path):
    with np.load(path, allow_pickle=False) as data:
        pose, q, forces, cmd, vel = (data[k] for k in
           ("pose", "q", "wheel_ground_normal_n", "cmd", "vel"))
        assert pose.shape[0] == 300 and forces.shape == (300, pose.shape[1], 4)
        assert all(np.isfinite(x).all() for x in (pose,q,forces,cmd,vel))
        assert not np.any(cmd[:100]) and not np.any(cmd[250:])
        x,y,z,w = np.moveaxis(pose[...,3:7], -1, 0)
        pitch = np.degrees(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
        rows = []
        for idx,label in enumerate(data["labels"].tolist()):
            stop = pose[250:,idx,:2]-pose[249,idx,:2]
            unsupported = forces[125:250,idx] < 3.
            rows.append({
                "trial": idx, "scenario": label,
                "terminated": bool(data["fall"][:,idx].any()),
                "handoff_pitch_min_deg": float(pitch[:100,idx].min()),
                "handoff_pitch_change_deg": float(pitch[99,idx]-pitch[0,idx]),
                "max_continuous_unload_s": [longest(m)*.02 for m in unsupported.T],
                "max_continuous_fewer_than_four_s": longest(unsupported.any(axis=1))*.02,
                "mean_abs_velocity_error": np.abs(vel[125:250,idx]-cmd[125:250,idx]).mean(0).tolist(),
                "stop_drift_max_m": float(np.linalg.norm(stop,axis=1).max()),
                "joint_min_rad": q[:,idx,:12].min(0).tolist(),
                "joint_max_rad": q[:,idx,:12].max(0).tolist(),
                "nonwheel_contact_frames": int(np.count_nonzero(data["nonwheel"][:,idx])),
            })
        return {"schema":1, "dt_s":.02,"wheel_contact_threshold_n":3.,
                "wheel_order":data["joints"][12:].tolist(), "trials":rows}

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("trace",type=Path)
    parser.add_argument("output",type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(score(args.trace),indent=2)+"\n",encoding="utf-8")
