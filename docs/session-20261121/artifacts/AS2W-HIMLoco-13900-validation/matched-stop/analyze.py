from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
p=Path(__file__).parent
target=.45
tolerance=.03
cases=('9836-min','9836-max','13900-min','13900-max','9836-max-refine')
report={}
for case in cases:
 a=np.load(p/f'{case}.npz')
 result={}
 for direction, sign in [('forward',1),('reverse',-1)]:
  ids=np.flatnonzero(np.char.startswith(a['labels'],direction+'_'))
  speed=sign*a['vel'][235:250,ids,0].mean(0)
  drift=np.linalg.norm(a['pose'][250:,ids,:2]-a['pose'][249,ids,:2],axis=-1)
  support=(a['wheel_ground_normal_n'][:,ids]>=3).all(-1)
  good=(abs(speed-target)<=tolerance)&~a['fall'][:,ids].any(0)&~a['nonwheel'][:,ids].astype(bool).any(0)&support[235:250].all(0)
  rows=[]
  for j,i in enumerate(ids):
   rows.append(dict(trial=int(i),command=float(a['cmd'][249,i,0]),entry_speed=float(speed[j]),stop_max_cm=float(100*drift[:,j].max()),stop_four_support=float(support[250:,j].mean()),eligible=bool(good[j])))
  g=[r for r in rows if r['eligible']]
  result[direction]=dict(count=len(g),entry_avg=float(speed[good].mean()) if g else None,stop_mean_cm=float((drift[:,good].max(0)*100).mean()) if g else None,stop_max_cm=float(drift[:,good].max()*100) if g else None,trials=rows)
 report[case]=result
(p/'report.json').write_text(json.dumps(report,indent=2))
fig,axes=plt.subplots(1,2,figsize=(11,4.5),sharey=True)
for ax,m in zip(axes,('forward','reverse')):
 for case in cases:
  rows=report[case][m]['trials']
  ax.scatter([z['entry_speed'] for z in rows],[z['stop_max_cm'] for z in rows],label=case,s=22,alpha=.7)
 ax.axvspan(target-tolerance,target+tolerance,color='green',alpha=.10)
 ax.set_title(m);ax.set_xlabel('Actual speed before release (m/s)');ax.grid(alpha=.2)
axes[0].set_ylabel('Maximum planar displacement during first second (cm)')
axes[1].legend(fontsize=8)
fig.tight_layout();fig.savefig(p/'matched-stop.png',dpi=150)
for k,v in report.items():
 print(k, {m:[r[x] for x in ('count','entry_avg','stop_mean_cm','stop_max_cm')] for m,r in v.items()})

