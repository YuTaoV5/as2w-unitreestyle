from pathlib import Path
import json, sys
import numpy as np

root = Path(sys.argv[1])
def score(path):
    with np.load(path) as z:
        pose, force, vel, cmd, q, torque, fall = (z[k] for k in (
            'pose','wheel_ground_normal_n','vel','cmd','q','leg_torque_nm','fall'))
        x,y,zz,w = np.moveaxis(pose[...,3:7], -1, 0)
        pitch = np.degrees(np.arcsin(np.clip(2*(w*y-zz*x),-1,1)))
        filtered = pitch.copy()
        alpha = -np.expm1(-2*np.pi*5*.02)
        for i in range(1,300):
            filtered[i] = filtered[i-1]+alpha*(pitch[i]-filtered[i-1])
        def timer(sign):
            angle = filtered*sign
            run = np.zeros_like(angle, dtype=int)
            for i in range(300):
                prev = run[i-1] if i else np.zeros(angle.shape[1],dtype=int)
                run[i] = np.where((angle[i]>4)|((prev>0)&(angle[i]>3)),prev+1,0)
            return run
        f,b = timer(1),timer(-1)
        active = slice(125,250)
        result = {}
        for label in ['all',*dict.fromkeys(z['labels'].tolist())]:
            idx = z['labels']==label if label!='all' else np.ones(pose.shape[1],bool)
            rows = {'trials':int(idx.sum()), 'falls':int(fall[:,idx].any(axis=0).sum())}
            for name,sl in [('handoff',slice(0,100)),('move',slice(100,250)),('stop',slice(250,300))]:
                rows[name] = dict(fwd=int((f[sl,idx]>30).any(axis=0).sum()),
                    back=int((b[sl,idx]>30).any(axis=0).sum()),
                    abs_p95=float(np.percentile(abs(pitch[sl,idx]),95)),
                    severe=int((abs(pitch[sl,idx])>10).any(axis=0).sum()))
            rows.update(track_mae=np.abs(vel[active,idx]-cmd[active,idx]).mean(axis=(0,1)).tolist(),
                stop_p95=float(np.percentile(np.linalg.norm(pose[250:,idx,:2]-pose[249,idx,:2],axis=-1).max(axis=0),95)),
                four_support=float((force[active,idx]>=3).all(axis=-1).mean()),
                force_mean=force[active,idx].mean(axis=(0,1)).tolist(),
                q_mean=q[active,idx,:12].mean(axis=(0,1)).tolist(),
                torque_abs_mean=np.abs(torque[active,idx]).mean(axis=(0,1)).tolist())
            result[label]=rows
        return result
out={}
for a in ('A','B'):
    for tag in ('10136','10436'):
        path=root/a/('eval-'+tag)/'holdout/flat.npz'
        if path.exists(): out[f'{tag}-{a}']=score(path)
for path in sorted((root/'pressure').glob('*/flat.npz')):
    out[path.parent.name]=score(path)
for name, data in out.items():
    a=data['all']
    print(name,[(k,a[k]['fwd'],a[k]['back'],round(a[k]['abs_p95'],2)) for k in ('handoff','move','stop')],a['falls'],round(a['four_support'],3),round(a['stop_p95'],3), list(data)[1:])
(root/'comparison.json').write_text(json.dumps(out,indent=2))
