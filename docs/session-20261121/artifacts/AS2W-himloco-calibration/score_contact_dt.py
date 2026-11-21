import json,math
from pathlib import Path
r=Path('AS2W-himloco-calibration')
d=r/'contact-screen-dt'
refs={'t2':(-6.13,[-11.81,11.70,-15.15,15.19])}
for name,fixture in [('official','official-reference'),('official-repeat','official-repeat')]:
 x=json.loads((r/f'physx-{fixture}-fixture.json').read_text())
 refs[name]=(x['ref_pitch'],x['ref_hip_tau'])
out={}
for name,(pitch,tau) in refs.items():
 path=d/f'{name}.json'
 if path.exists():
  x=json.loads(path.read_text()); out[name]=dict(pitch_error_deg=abs(x['pitch']-pitch),hip_rmse_nm=math.sqrt(sum((a-b)**2 for a,b in zip(x['pd_nm'][:12:3],tau))/4),wheel_normal_n=[f[2] for f in x['ground_normal']])
(d/'summary.json').write_text(json.dumps(out,indent=2))
print(out)
