import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
p=Path(__file__).parent
files={k:np.load(p/f'{k}.npz') for k in ('9836-min','9836-max','13900-min','13900-max')}
ref=files['9836-min']
for a in files.values():
 np.testing.assert_array_equal(a['labels'],ref['labels'])
 np.testing.assert_array_equal(a['cmd'],ref['cmd'])
 np.testing.assert_array_equal(a['joints'],ref['joints'])
t=np.arange(300)*.02
out={}
colors=['#246ca8','#1da08a','#c5762b','#ad4175']
for mode in ('forward','reverse'):
 ids=ref['labels']==mode
 summary={}
 fig,axes=plt.subplots(2,2,figsize=(12,7),sharex=True)
 for (label,a),color in zip(files.items(),colors):
  vel=a['vel'][:,ids,0]
  wheel=a['wheel_target'][:,ids]
  pose=a['pose'][:,ids,:2]
  drift=np.linalg.norm(pose[250:]-pose[249],axis=-1)
  normal=a['wheel_ground_normal_n'][:,ids]
  err=np.abs(vel-a['cmd'][:,ids,0])
  summary[label]=dict(steady_vx=float(vel[150:250].mean()),steady_mae=float(err[150:250].mean()),stop_max_m=float(drift.max()),stop_median_m=float(np.median(drift.max(0))),command_support_pct=float(100*np.mean((normal[100:250]>=3).all(-1))),stop_support_pct=float(100*np.mean((normal[250:]>=3).all(-1))),terminated=int(a['fall'][:,ids].any(0).sum()),steady_leg_tau_abs_mean=np.abs(a['leg_torque_nm'][150:250,ids]).mean((0,1)).tolist(),stop_wheel_load_N=normal[250:].mean((0,1)).tolist())
  axes[0,0].plot(t,vel.mean(1),color=color,label=label)
  axes[0,1].plot(t,wheel.mean(1),color=color)
  axes[1,0].plot(t[250:],100*drift.max(1),color=color)
  axes[1,1].plot(t,100*(normal>=3).all(-1).mean(1),color=color)
 axes[0,0].plot(t,ref['cmd'][:,ids,0].mean(1),'k:',label='command')
 for ax, title in zip(axes.flat,('Body vx (m/s)','Wheel target mean magnitude (rad/s)','Worst planar drift since release (cm)','All four wheels loaded (% of trials)')):
  ax.set_title(title);ax.grid(alpha=.2)
  for x in (2,5):ax.axvline(x,color='gray',alpha=.3)
 axes[0,0].legend(ncol=2)
 fig.suptitle(mode+' | six matched field-reset trials per condition')
 fig.tight_layout();fig.savefig(p/f'{mode}-speed-stop.png',dpi=150);plt.close(fig)
 for key,title in [('q','joint angle (rad)'),('leg_torque_nm','joint torque estimate (Nm)'),('wheel_ground_normal_n','ground normal load (N)')]:
  iswheel=key.startswith('wheel')
  fig,axes=plt.subplots(4,1 if iswheel else 3,figsize=(13,8),sharex=True)
  for j,ax in enumerate(axes.flat):
   for (label,a),color in zip(files.items(),colors):
    v=a[key][:,ids,j]
    ax.plot(t,np.median(v,axis=1),color=color,label=label)
    if key=='q':ax.plot(t,np.median(a['target_trace'][:,ids,2,j],axis=1),':',color=color,alpha=.6)
   ax.set_title(str(ref['joints'][j+12 if iswheel else j]));ax.grid(alpha=.2)
   for x in (2,5):ax.axvline(x,color='gray',alpha=.3)
  axes.flat[0].legend(fontsize=7,ncol=2)
  fig.suptitle(f'{mode} | {title} | median of six trials'+(' | dotted = applied target' if key=='q' else ''))
  fig.tight_layout();fig.savefig(p/f'{mode}-{key}.png',dpi=140);plt.close(fig)
 out[mode]=summary
(p/'summary.json').write_text(json.dumps(out,indent=2))
