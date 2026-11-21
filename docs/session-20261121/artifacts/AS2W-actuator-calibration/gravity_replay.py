"""Replay measured poses through the T2 rigid-body gravity model."""
import csv
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ASSET = HERE.parent / "AS2W-himloco-calibration/asset-candidate-t2-v2"
CONTRACT = json.loads((ASSET / "contract.json").read_text())
NAMES = [f"{leg}_{joint}" for leg in ("FL", "FR", "RL", "RR")
         for joint in ("hip", "thigh", "calf", "foot")]
ORDER = np.array(CONTRACT["dds_to_mujoco_slots"])[[0,1,2,12,3,4,5,13,6,7,8,14,9,10,11,15]]
ROOT = ET.parse(ASSET / "as2w.xml").find("worldbody/body")


def gravity(state):
    quat = state[:, 6:10]
    w, x, y, z = (quat / np.linalg.norm(quat, axis=1)[:, None]).T
    g = -9.81 * np.stack((2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)), axis=1)
    q = state[:, 13:].reshape(-1,16,6)[:, ORDER, 0]
    result = np.zeros_like(q)

    def visit(body, pos, rot):
        pos = pos + rot @ np.fromstring(body.get("pos", "0 0 0"), sep=" ")
        joint = body.find("joint")
        index = None
        if joint is not None and joint.get("name") in NAMES:
            index = NAMES.index(joint.get("name"))
            axis = np.fromstring(joint.get("axis"), sep=" ")
            skew = np.cross(axis, np.eye(3)).T
            angle = q[:, index, None, None]
            rot = rot @ (np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew@skew))
            world_axis = rot @ axis
        inertia = body.find("inertial")
        mass = float(inertia.get("mass"))
        moment = mass * (pos + rot @ np.fromstring(inertia.get("pos"), sep=" "))
        for child in body.findall("body"):
            child_mass, child_moment = visit(child, pos, rot)
            mass += child_mass
            moment = moment + child_moment
        if index is not None:
            result[:, index] = -np.sum(world_axis*np.cross(moment-mass*pos, g), axis=1)
        return mass, moment

    visit(ROOT, np.zeros(3), np.eye(3))
    return result


CASES = [
    ("P55-S", "field_sessions/20261119-194344_p55-hold", 21,110),
    ("P85D-S", "field_sessions/20261119-194837_p85d-hold", 42,122),
    ("O-S", "20261119-165542_official-os", 0,60),
    ("P55-G1", "field_sessions/20261119-182802_p55-hold", 10,30),
    ("P55-G2", "field_sessions/20261119-185930_p55-hold", 7,20),
    ("P55-G3", "field_sessions/20261119-190012_p55-hold", 7,20),
]


def main(logs):
    rows = []
    for label, folder, start, end in CASES:
        with (logs / folder / "low_capture_fast.log").open() as source:
            sample = [line.split(",", 110)[1:110] for line in source if line.startswith("LOW_SAMPLE,")]
        data = np.array([row for row in sample if len(row)==109 and start <= float(row[0])/1000 <= end], dtype=float)
        reported = data[:, 13:].reshape(-1,16,6)[:, ORDER, 3]
        modeled = gravity(data)
        for i,name in enumerate(NAMES):
            if name.endswith("_foot"):
                continue
            residual = reported[:,i] - modeled[:,i]
            rows.append([label,name,len(data),reported[:,i].mean(),modeled[:,i].mean(),
                         residual.mean(),*np.percentile(residual,[5,95])])
    with (HERE/"gravity-residual.csv").open("w", newline="") as out:
        writer = csv.writer(out)
        writer.writerow(["case","joint","frames","tau_est_nm","gravity_hold_nm",
                         "residual_nm","residual_p05_nm","residual_p95_nm"])
        writer.writerows(rows)
    for case, name, n, observed, expected, residual, *_ in rows:
        if name.endswith("_hip"):
            print(f"{case:7} {name} {n:6} {observed:+6.2f} {expected:+6.2f} {residual:+6.2f}")


if __name__ == "__main__":
    main(Path(sys.argv[1]))

