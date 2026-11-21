"""Rank the paired static screen against same-condition real data."""
import argparse
import json
import math
from pathlib import Path

REF = {
    "t2": (-6.13, [-11.81, 11.70, -15.15, 15.19]),

}
_official = json.loads(Path(__file__).with_name('physx-official-reference-fixture.json').read_text())
REF['official'] = (_official['ref_pitch'], _official['ref_hip_tau'])
HIP = [0, 3, 6, 9]

def metrics(path, scene):
    data = json.loads(path.read_text())
    pitch, torque = REF[scene]
    sim = [data["pd_nm"][i] for i in HIP]
    return {
        "pitch_abs_error_deg": abs(data["pitch"] - pitch),
        "hip_rmse_nm": math.sqrt(sum((a-b)**2 for a,b in zip(sim,torque))/4),
        "hip_sign_matches": sum(a*b > 0 for a,b in zip(sim,torque)),
        "max_contact_force_closure_n": max(
            abs(total-normal-friction)
            for totals,normals,frictions in zip(data["forces"],data["ground_normal"],data["ground_friction"])
            for total,normal,friction in zip(totals,normals,frictions)),
        "wheel_normal_n": [f[2] for f in data["ground_normal"]],
    }

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    parser.add_argument('--baseline',type=Path)
    parser.add_argument('--settings',nargs='+',default=['.1','.2','.25','.3','.6'])
    args = parser.parse_args()
    cases = {}
    for setting in ["native", *args.settings]:
        root = args.baseline if setting == "native" and args.baseline else args.run
        files = {scene:root/f"{scene}-{setting}.json" for scene in REF}
        cases[setting] = {scene:metrics(path,scene) for scene,path in files.items() if path.is_file()}
    accepted = [
        setting for setting, scenes in cases.items()
        if setting != "native" and len(scenes)==len(REF) and len(cases["native"])==len(REF)
        and all(
            m["pitch_abs_error_deg"] < cases["native"][scene]["pitch_abs_error_deg"]
            and m["hip_rmse_nm"] < cases["native"][scene]["hip_rmse_nm"]
            and m["max_contact_force_closure_n"] < .01
            for scene,m in scenes.items()
        )
    ]
    print(json.dumps({"cases":cases,"joint_improvements":accepted},indent=2))

if __name__ == "__main__":
    main()



