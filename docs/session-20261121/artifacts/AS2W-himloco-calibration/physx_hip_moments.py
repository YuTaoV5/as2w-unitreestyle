"""Estimate hip torque contributions using the lowest wheel-mesh contact patch."""
from pathlib import Path
import json
import numpy as np
p=Path(__file__).resolve().parent
exec((p/'open_loop.py').read_text().split('results={}')[0])
roots=(p/'contact-screen-load-v0',p/'contact-screen-v2-mu02',p/'contact-screen-dt')
out={}
for folder in roots:
 for f in sorted(folder.glob('*.json')):
  r=json.loads(f.read_text())
  if not {'gravity','ground_normal','ground_friction','q','pd_nm'}<=r.keys():
   continue
  d=mj.MjData(m)
  d.qpos[2]=2
  g=np.asarray(r['gravity'])
  roll=np.arctan2(-g[1],-g[2]); pitch=np.arcsin(np.clip(g[0],-1,1))
  cr,sr=np.cos(roll/2),np.sin(roll/2)
  cp,sp=np.cos(pitch/2),np.sin(pitch/2)
  d.qpos[3:7]=[cr*cp,sr*cp,cr*sp,-sr*sp]
  d.qpos[qa[:12]]=r['q']; mj.mj_forward(m,d)
  axes={}
  for k,name in enumerate(names[12:]):
   b=m.body(name).id; geom=m.body_geomadr[b]; mesh=m.geom_dataid[geom]
   v=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
   xyz=v@d.geom_xmat[geom].reshape(3,3).T+d.geom_xpos[geom]
   contact=xyz[xyz[:,2]<=xyz[:,2].min()+.001].mean(0)
   joint=ids[3*k]; arm=contact-d.xanchor[joint]
   moment=np.cross(arm,np.eye(3))@d.xaxis[joint]
   normal=moment@r['ground_normal'][k]
   fx,fy,fz=moment*np.asarray(r['ground_friction'][k])
   grav=float(d.qfrc_bias[va[3*k]])
   applied=r['pd_nm'][3*k]
   axes[name]={'normal_nm':float(normal),'longitudinal_nm':float(fx),'lateral_nm':float(fy),
               'other_nm':float(fz),'hip_gravity_nm':grav,'pd_nm':applied,
               'closure_nm':float(applied+normal+fx+fy+fz-grav)}
  out[f'{folder.name}/{f.name}']=axes
(p/'physx-hip-moment-estimate.json').write_text(json.dumps(out,indent=2))
for key in ('contact-screen-load-v0/t2-native.json','contact-screen-v2-mu02/t2-.2.json'):
 print(key,out.get(key))
