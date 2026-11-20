from pathlib import Path
import numpy as np,csv,json
# Reuse the exact model construction used in the command replay.
p=Path(__file__).resolve().parent
exec((p/'open_loop.py').read_text().split('results={}')[0])
f=Path(__import__('sys').argv[1])
rows=read(f,'LOW_SAMPLE',110)
rows=rows[(rows[:,0]>=float(__import__('sys').argv[2])*1000)&(rows[:,0]<float(__import__('sys').argv[3])*1000)]
slots=[3,4,5,0,1,2,9,10,11,6,7,8,13,12,15,14]
q,_,_,tau,*_=rows[:,13:].reshape(-1,16,6)[:,slots].transpose(2,0,1)*sign
d=mj.MjData(m)
d.qpos[2]=2
quat=rows[:,6:10].copy()
quat[np.sum(quat*quat[0],axis=1)<0]*=-1
avg=quat.mean(0)
d.qpos[3:7]=avg/np.linalg.norm(avg)
d.qpos[qa]=q.mean(0)
mj.mj_forward(m,d)
contacts=[(m.geom(c.geom1).name,m.geom(c.geom2).name,m.body(m.geom_bodyid[c.geom1]).name,m.body(m.geom_bodyid[c.geom2]).name,float(c.dist)) for c in d.contact]
m.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_CONTACT)
mj.mj_forward(m,d)
assert d.ncon==0
# Root acceleration is zero and the support takes the root wrench.
sim=d.qfrc_bias[va][:12].copy()
real=tau.mean(0)[:12]
result={'frames':len(rows),'pose_wxyz':d.qpos[3:7].tolist(),'contact_count':d.ncon,'candidate_self_contacts_before_disable':contacts,
        'mae_nm':float(abs(sim-real).mean()),'axes':{n:dict(real_nm=float(r),sim_nm=float(s),error_nm=float(s-r)) for n,r,s in zip(names,real,sim)}}
hip=np.arange(0,12,3)
d.qpos[qa[hip]] *= -1
mj.mj_forward(m,d)
alt=d.qfrc_bias[va[hip]].copy()
result['hip_sign_sensitivity']={'real_nm':(-real[hip]).tolist(),'sim_nm':alt.tolist(),'mae_nm':float(abs(alt+real[hip]).mean())}
(p/(__import__('sys').argv[4]+'.json')).write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))



