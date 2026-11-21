"""Produce stricter diagnostic replay without overwriting field evidence."""
from pathlib import Path
import json
p=Path(r'X:/Items/as2w/himloco_field_20261119/scripts/p9_mujoco_himloco.py')
s=p.read_text(encoding='utf-8')
s=s.replace('import math','import json\nimport hashlib\nfrom pathlib import Path\nimport xml.etree.ElementTree as ET\nimport math',1)
s=s.replace('else: raise SystemExit("bad arg " + argv[i])','''elif argv[i] == "--urdf": o["urdf"] = argv[i+1]; i += 2
        elif argv[i] == "--initial-state": o["initial_state"] = argv[i+1]; i += 2
        else: raise SystemExit("bad arg " + argv[i])''')
s=s.replace('    m = mujoco.MjModel.from_xml_path(o["xml"])','''    if "urdf" not in o or o["xml"] == XML:
        raise SystemExit("Explicit --xml and --urdf required; old r11 default disabled")
    m = mujoco.MjModel.from_xml_path(o["xml"])
    if abs(m.opt.timestep * DECIM - DT) > 1e-9:
        raise SystemExit("XML timestep * decimation != 20 ms; correct asset/time contract first")
    if m.nu:
        raise SystemExit("Remove XML actuators for this explicit qfrc_applied PD replay")
    urdf = ET.parse(o["urdf"]).getroot()
    limits = np.array([float(urdf.find("joint[@name='%s']/limit" % n).get("effort")) for n in LEGS+WHEELS])
    print("DIAGNOSTIC ONLY: URDF limits are not verified hardware limits")
    print("assets", {k:hashlib.sha256(Path(o[k]).read_bytes()).hexdigest() for k in ("xml","urdf","onnx")})
    initial = json.loads(Path(o["initial_state"]).read_text()) if "initial_state" in o else None''')
s=s.replace('        mujoco.mj_forward(m, d)','''        if initial:
            d.qpos[3:7] = initial["quat_wxyz"]
            d.qpos[adr["q"]] = initial["q"][:12]
            d.qvel[adr["v"]] = initial["dq"][:12]
            d.qvel[adr["wv"]] = initial["dq"][12:]
        mujoco.mj_forward(m, d)''',1)
s=s.replace('-35, 35)', '-limits[:12], limits[:12])').replace('-8, 8)', '-limits[12:], limits[12:])')
out=Path(__file__).resolve().parent
(out/'p9_checked.py').write_text(s,encoding='utf-8')
import csv
for name in ['vx015_walked','vx010_X_abort_handoff']:
    r=next(r for r in csv.reader((p.parent.parent/name/'debug_policy.csv').open()) if r and r[0]=='DEBUG_POLICY')
    state={'q':list(map(float,r[25:41])),'dq':list(map(float,r[41:57])),'quat_wxyz':list(map(float,r[18:22])),
           'height':'not measured; explicit --z is a diagnostic assumption','root_linear_velocity':'not measured; defaults zero',
           'scope':'initial-state diagnostic, NOT complete publisher handoff replay'}
    (out/(name+'-initial.json')).write_text(json.dumps(state,indent=2))
print('Checked diagnostic replay and measured initial-state fixtures generated')
