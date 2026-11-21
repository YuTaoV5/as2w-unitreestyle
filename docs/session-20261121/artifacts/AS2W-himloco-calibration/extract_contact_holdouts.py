"""Extract matched initial pose and observed static baseline for independent sessions."""
import csv
import json
import math
from pathlib import Path

ROOT = Path(r"X:/Items/as2w/AS2-W_CPRO/部署测试/04_调试记录/logs")
WORK = Path(__file__).resolve().parent
SLOTS = [3,4,5,0,1,2,9,10,11,6,7,8,13,12,15,14]
sources = [
    ("official-reference", ROOT/"AS2W官方运控基线数据包_20261119", "03_*", 2000,15000),
    ("t2-repeat1", ROOT/"AS2W_悬空负载诊断V2_全部数据与分析_20261119/recordings/field_sessions",
     "*182802*", 10000, 30000),
    ("t2-repeat3", ROOT/"AS2W_悬空负载诊断V2_全部数据与分析_20261119/recordings/field_sessions",
     "*190012*", 8000, 19000),
    ("official-repeat", ROOT/"AS2W官方运控基线数据包_20261119", "01_*", 25000, 40000),
]
for name, base, pattern, start, stop in sources:
    path = next(base.glob(pattern))/"low_capture_fast.log"
    samples = []
    with path.open() as stream:
        for r in csv.reader(stream):
            if len(r) >= 110 and r[0] == "LOW_SAMPLE" and start <= int(r[1]) < stop and int(r[1]) % 20 < 2:
                samples.append(r)
    row = samples[len(samples)//2]
    def pitch(r):
        w,x,y,z = map(float,r[7:11])
        return math.degrees(math.asin(max(-1., min(1.,2*(w*y-z*x)))))
    out = dict(session=path.parent.name, tick=int(row[4]), elapsed_ms=int(row[1]),
               quat_wxyz=list(map(float,row[7:11])),
               q_dds=[float(row[14+6*i]) for i in SLOTS],
               ref_pitch=sum(pitch(r) for r in samples)/len(samples),
               ref_hip_tau=[sum(float(r[17+6*i]) for r in samples)/len(samples) for i in SLOTS[:12:3]],
               n=len(samples))
    (WORK/f"physx-{name}-fixture.json").write_text(json.dumps(out,indent=2))
    print(name, out["ref_pitch"], out["ref_hip_tau"], out["n"])

