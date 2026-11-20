from pathlib import Path
import numpy as np, csv, json, xml.etree.ElementTree as ET
import mujoco as mj
import os
p=Path(__file__).parent
a=Path('AS2W-github-publish/unified_dr/simulation/assets/load-v0').resolve()
x=ET.parse('AS2W-github-publish/review/u3220/model/as2w_flat.xml').getroot()
u=ET.parse(a/'as2w-load-v0.urdf').getroot()
for mesh in x.findall('asset/mesh'):
 mesh.set('file',str(a/'meshes'/Path(mesh.get('file')).name))
for b in x.findall('.//body'):
 i=b.find('inertial'); r=u.find(f"link[@name='{b.get('name')}']/inertial")
 if i is None or r is None: continue
 o=r.find('origin'); assert np.allclose(np.fromstring(o.get('rpy','0 0 0'),sep=' '),0)
 i.attrib=dict(pos=o.get('xyz'),mass=r.find('mass').get('value'),fullinertia=' '.join(r.find('inertia').get(k) for k in ('ixx','iyy','izz','ixy','ixz','iyz')))
base=x.find(".//body[@name='base_link']/inertial")
scale=float(os.getenv('BASE_SCALE','1'))
dx=float(os.getenv('BASE_DX','0'))
base.set('mass',str(float(base.get('mass'))*scale))
base.set('fullinertia',' '.join(str(float(v)*scale) for v in base.get('fullinertia').split()))
xyz=np.fromstring(base.get('pos'),sep=' '); xyz[0]+=dx
base.set('pos',' '.join(map(str,xyz)))
x.remove(x.find('actuator')); x.find('option').set('timestep','.001')
if os.getenv('EXCLUDE_FRONT_THIGHS'):
 contact=x.find('contact')
 if contact is None: contact=ET.SubElement(x,'contact')
 for thigh in ('FL_thigh','FR_thigh'):
  ET.SubElement(contact,'exclude',body1='base_link',body2=thigh)
if os.getenv('GROUND_MU'):
 x.find('.//geom[@name="ground"]').set('friction',os.environ['GROUND_MU']+' .005 .0001')
m=mj.MjModel.from_xml_string(ET.tostring(x,encoding='unicode'))
names=[f'{leg}_{joint}' for leg in ('FL','FR','RL','RR') for joint in ('hip','thigh','calf')]+[f'{leg}_foot' for leg in ('FL','FR','RL','RR')]
ids=[m.joint(n).id for n in names]; qa=m.jnt_qposadr[ids]; va=m.jnt_dofadr[ids]
limit=np.array([float(u.find(f"joint[@name='{n}']/limit").get('effort')) for n in names])
sign=np.array([1 if os.getenv('ALT_HIP') else -1,1,1]*4+[1]*4)
def read(f,tag,n):
 with f.open() as stream:
  return np.array([r[1:n] for r in csv.reader(stream) if len(r)>=n and r[0]==tag],float)
def pitch(q):
 w,x,y,z=q.T
 return np.degrees(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
results={}
for s,label in [('20261119-114540','vx015_walked'),('20261119-190857','vx010_X_abort_handoff')]:
 c=read(p/f'{s}-debug_lowcmd.csv','DEBUG_LOWCMD',117)
 l=read(p/f'{s}-low_capture_fast.log','LOW_SAMPLE',110)
 with (Path(r'X:/Items/as2w/himloco_field_20261119')/label/'debug_policy.csv').open() as f:
  ticks=[int(r[3]) for r in csv.reader(f) if r and r[0]=='DEBUG_POLICY' and r[2]=='HANDOFF']
 c=c[(c[:,1]>=ticks[0])&(c[:,1]<=ticks[-1])]
 t=(c[:,0]-c[0,0])*1e-9
 inds=np.unique(l[:,3],return_index=True)[1]; l=l[inds]
 j=np.searchsorted(l[:,3],c[:,1]).clip(1,len(l)-1)
 j-=abs(l[j-1,3]-c[:,1])<abs(l[j,3]-c[:,1])
 cmds=c[:,4:].reshape(-1,16,7); slots=cmds[0,:,0].astype(int)
 q,dq,_,tau,*_=l[j,13:].reshape(-1,16,6)[:,slots].transpose(2,0,1)*sign
 quat=l[j,6:10]; d=mj.MjData(m)
 d.qpos[2]=.4; d.qpos[3:7]=quat[0]; d.qpos[qa]=q[0]
 d.qvel[va]=dq[0]; d.qvel[3:6]=l[j[0],10:13]
 mj.mj_forward(m,d)
 bottoms=[]
 for g in range(m.ngeom):
  if m.body(m.geom_bodyid[g]).name not in names[12:]:continue
  mesh=m.geom_dataid[g]
  verts=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
  bottoms.append(min(verts@d.geom_xmat[g].reshape(3,3)[2]+d.geom_xpos[g,2]))
 d.qpos[2]-=min(bottoms)
 st=np.arange(0,t[-1]+.00001,.001); sim=[]
 for ti in st:
  k=np.searchsorted(t,ti,side='right')-1
  _,_,g,v,kp,kd,ff=cmds[k].T
  applied=np.clip(kp*(g*sign-d.qpos[qa])+kd*(v*sign-d.qvel[va])+ff*sign,-limit,limit)
  sim.append(np.r_[pitch(d.qpos[3:7]),d.qpos[qa],applied])
  d.qfrc_applied[va]=applied; mj.mj_step(m,d)
 sim=np.array(sim); real_pitch=pitch(quat)
 match=np.searchsorted(st,t).clip(0,len(st)-1)
 results[label]=dict(real_pitch=[float(real_pitch[0]),float(real_pitch[-1])],sim_pitch=[float(sim[0,0]),float(sim[-1,0])],leg_q_mae=float(abs(sim[match,1:13]-q[:,:12]).mean()),leg_tau_mae=float(abs(sim[match,17:29]-tau[:,:12]).mean()))
 if not os.getenv("SCAN_ONLY"): np.savez_compressed(p/f'{label}-open-loop.npz',time=t,real_pitch=real_pitch,real_q=q,real_tau=tau,sim_time=st,sim_pitch=sim[:,0],sim_q=sim[:,1:17],sim_tau=sim[:,17:])
(p/('scan-case.json' if os.getenv('SCAN_ONLY') else 'open-loop.json')).write_text(json.dumps(results,indent=2))
print(results)




