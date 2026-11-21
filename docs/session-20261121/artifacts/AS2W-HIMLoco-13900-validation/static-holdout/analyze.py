from pathlib import Path
import numpy as np
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
d=Path(__file__).parent
out={}
cases=('native','min','max')
names=('t2-repeat1','t2-repeat3','official-repeat')
fig,axes=plt.subplots(3,2,figsize=(12,9))
for i,name in enumerate(names):
 ref=json.loads((d/f'{name}-fixture.json').read_text())
 real=np.array(ref['ref_hip_tau'])
 out[name]={}
 labels=['FL','FR','RL','RR'];x=np.arange(4)
 axes[i,0].bar(x-.3,real,.19,label='Real')
 axes[i,1].bar(-.3,ref['ref_pitch'],.19,label='Real')
 for j,c in enumerate(cases):
  a=json.loads((d/f'{name}-{c}.json').read_text())
  tau=np.array(a['hip_torque_nm'])
  pd=a['contract']['kp']
  kd=a['contract']['kd']
  goal=np.array([-.15,.15,-.15,.15]) if pd==200 else np.zeros(4)
  reconstructed=pd*(goal-np.array(a['q'])[::3])-kd*np.array(a['dq'])[::3]
  out[name][c]=dict(hip_rmse=float(np.sqrt(np.mean((reconstructed-real)**2))),pitch_abs_err=abs(a['pitch_deg']-ref['ref_pitch']),hip_pd=reconstructed.tolist(),hip_logged=tau.tolist(),real_hip=real.tolist(),pitch=a['pitch_deg'],real_pitch=ref['ref_pitch'],normal=np.array(a['wheel_normal_n'])[:,2].tolist(),all_wheels_loaded=a['all_wheels_loaded'])
  axes[i,0].bar(x-.1+.2*j,reconstructed,.19,label=c)
  axes[i,1].bar(-.1+.2*j,a['pitch_deg'],.19,label=c)
 axes[i,0].set_xticks(x,labels);axes[i,0].set_title(name+' signed hip torque (Nm)')
 axes[i,1].set_title(name+' pitch (deg)');axes[i,1].set_xticks([])
for ax in axes.flat:ax.grid(axis='y',alpha=.2)
axes[0,0].legend(ncol=4)
fig.tight_layout();fig.savefig(d/'holdout-static.png',dpi=145)
(d/'summary.json').write_text(json.dumps(out,indent=2))
for n,v in out.items():
 for c,z in v.items():print(n,c,round(z['hip_rmse'],2),round(z['pitch_abs_err'],2),z['all_wheels_loaded'],np.round(z['hip_pd'],2),np.round(z['normal'],1))
