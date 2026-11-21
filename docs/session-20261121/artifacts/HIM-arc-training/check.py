import json,sys
from pathlib import Path
import numpy as np
p=Path(sys.argv[1]); a=np.load(p/'flat.npz')
rows=[]
def longest(mask):
 e=np.diff(np.r_[False,mask,False].astype(int))
 return float(np.max(np.flatnonzero(e==-1)-np.flatnonzero(e==1),initial=0)*.02)
for j,c in enumerate(a['labels']):
 q=a['q'][50:,j,:12].reshape(-1,4,3)
 f=a['wheel_data'][50:,j,:,15]; v=a['vel'][50:,j]; cmd=a['cmd'][50:,j]
 mae=np.abs(v-cmd).mean(0)
 tilt=float(np.quantile(a['tilt'][50:,j],.95))
 bad=(abs(q[:,:,0])>.35)|(q[:,:,1]<.3)|(q[:,:,1]>1.3)|(q[:,:,2]<-2.15)|(q[:,:,2]>-1.15)
 support=float((f>=3).all(1).mean())
 flags=[]
 if a['fall'][:,j].any(): flags.append('fall')
 if longest(bad.any(1))>.16: flags.append('joint_pose')
 if tilt>6.: flags.append('tilt')
 if support<(.95 if c in ('stand','forward','reverse') else .85): flags.append('wheel_support')
 if max(longest(f[:,k]<3) for k in range(4))>.3: flags.append('sustained_unload')
 if mae[0]>.10 or mae[2]>.15 or mae[1]>.07: flags.append('tracking')
 rows.append(dict(case=str(c),trial=j,flags=flags,mae=mae.tolist(),tilt_p95=tilt,four_support=support,
                  joint_median=np.median(q,axis=0).tolist(),wheel_mean_N=f.mean(0).tolist()))
report=dict(passed=all(not r['flags'] for r in rows),failed=sum(bool(r['flags']) for r in rows),trials=rows)
(p/'arc-acceptance.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k!='trials'}))
