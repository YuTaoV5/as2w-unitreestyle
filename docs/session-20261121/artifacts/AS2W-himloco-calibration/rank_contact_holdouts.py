"""Score independent standing sessions against their own telemetry."""
import argparse
import json
import math
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("run", type=Path)
parser.add_argument('--candidate', default='.3')
args = parser.parse_args()
fixtures = Path(__file__).parent
output = {}
for scene in ("t2-repeat1", "t2-repeat3", "official-repeat"):
    ref = json.loads((fixtures / f"physx-{scene}-fixture.json").read_text())
    cases = {}
    for mode in ("native", args.candidate):
        path = args.run / f"{scene}-{mode}.json"
        if not path.is_file():
            continue
        data = json.loads(path.read_text())
        torque = data["pd_nm"][:12:3]
        closure = max(abs(f-n-t)
                      for full, normal, tangent in zip(data["forces"], data["ground_normal"], data["ground_friction"])
                      for f,n,t in zip(full,normal,tangent))
        cases[mode] = {
            "pitch_error_deg": abs(data["pitch"] - ref["ref_pitch"]),
            "hip_rmse_nm": math.sqrt(sum((t-r)**2 for t,r in zip(torque,ref["ref_hip_tau"]))/4),
            "signed_hip_torque_nm": torque,
            "wheel_normal_n": [f[2] for f in data["ground_normal"]],
            "contact_closure_n": closure,
        }
    if len(cases) == 2:
        cases["both_improved"] = all(cases[args.candidate][m] < cases["native"][m] for m in ("pitch_error_deg","hip_rmse_nm"))
    output[scene] = cases
print(json.dumps(output,indent=2))

