from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
p=Path(__file__).parent
real=np.load(p.parent/'physx-input.npz')
names=real['names']
cases={k:json.loads((p/f'{k}.json').read_text()) for k in ('A','A1','B')}
summary={}
for k,record in cases.items():
 rows=record['rows']
 t=np.array([r['t'] for r in rows])
 i=np.searchsorted(real['real_time'],t).clip(0,len(real['real_time'])-1)
 record.update(t=t,q=np.array([r['q'] for r in rows]),effort=np.array([r['effort'] for r in rows]),normal=np.array([r['normal'] for r in rows]),tangent=np.array([r['tangent'] for r in rows]))
 summary[k]={}
 for lo,hi in [(68,70),(71,73),(73,76),(77,81)]:
  w=(t>=lo)&(t<hi)
  q=record['q'][w]-real['real_q'][i[w]]
  tau=record['effort'][w]-real['real_tau'][i[w]]
  summary[k][f'{lo}-{hi}']=dict(q_mae=np.abs(q[:,:12]).mean(),tau_mae=np.abs(tau[:,:12]).mean(),hip_mae=np.abs(tau[:,[0,3,6,9]]).mean(),hip_mean=record['effort'][w][:,[0,3,6,9]].mean(0).tolist(),real_hip_mean=real['real_tau'][i[w]][:,[0,3,6,9]].mean(0).tolist(),wheel_normal=record['normal'][w,:,2].mean(0).tolist(),four_support=np.mean(np.all(record['normal'][w,:,2]>=3,axis=1)))
(p/'summary.json').write_text(json.dumps(summary,indent=2))
for field,ref,unit in [('q','real_q','rad'),('effort','real_tau','Nm')]:
 fig,ax=plt.subplots(4,3,figsize=(15,9),sharex=True)
 for j,a in enumerate(ax.flat):
  mask=(real['real_time']>=68)&(real['real_time']<81)
  a.plot(real['real_time'][mask][::5],real[ref][mask,j][::5],label='Real')
  for k,c in cases.items():a.plot(c['t'],c[field][:,j],label=k,alpha=.7)
  a.set_title(names[j]);a.set_ylabel(unit);a.grid(alpha=.2)
 ax[0,0].legend(ncol=2)
 fig.suptitle('Official rolling to stand | A original / A1 mass+CoM / B mu025 USD')
 fig.tight_layout()
 fig.savefig(p/f'{field}.png',dpi=140)
 plt.close(fig)
fig,ax=plt.subplots(4,2,figsize=(13,9),sharex=True)
for j in range(4):
 for k,c in cases.items():
  ax[j,0].plot(c['t'],c['normal'][:,j,2],label=k)
  ax[j,1].plot(c['t'],c['tangent'][:,j,1],label=k)
 ax[j,0].set_ylabel(str(names[12+j])+' (N)')
 for a in ax[j]:a.grid(alpha=.2)
ax[0,0].set_title('Normal force');ax[0,1].set_title('World lateral force');ax[0,0].legend()
fig.tight_layout();fig.savefig(p/'forces.png',dpi=140)
print(json.dumps(summary,indent=2))
