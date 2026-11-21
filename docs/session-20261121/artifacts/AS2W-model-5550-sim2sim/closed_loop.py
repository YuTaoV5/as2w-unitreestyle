import json, sys, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import mujoco as mj
out, candidate, runtime=map(Path,sys.argv[1:])
sys.path.insert(0,str(runtime))
from runtime import observation, wrap, load_session, DEFAULT
session=load_session(out/'policy.onnx')
template=Path('/root/gpufree-data/Rob2Nav/studio/as2w_eval/local-step-diagnostic/as2w_flat.xml')
urdf=ET.parse('/root/as2w/calibration/candidates/load-v0/as2w-load-v0.urdf').getroot()
xml=ET.parse(template).getroot()
for mesh in xml.findall('asset/mesh'):
 mesh.set('file',str(Path('/root/as2w/assets/as2w/meshes')/Path(mesh.get('file')).name))
for body in xml.findall('.//body'):
 name=body.get('name')
 link=urdf.find(f"link[@name='{name}']/inertial")
 inertial=body.find('inertial')
 if link is None or inertial is None: continue
 origin=link.find('origin')
 assert np.allclose(np.fromstring(origin.get('rpy','0 0 0'),sep=' '),0)
 tensor=link.find('inertia')
 inertial.attrib.clear()
 inertial.set('pos',origin.get('xyz'))
 inertial.set('mass',link.find('mass').get('value'))
 inertial.set('fullinertia',' '.join(tensor.get(k) for k in ('ixx','iyy','izz','ixy','ixz','iyz')))
xml.find('option').set('timestep','.005')
act=xml.find('actuator')
if act is not None: xml.remove(act)
ground=xml.find("worldbody/geom[@name='ground']")
ground.set('friction','0.65 .005 .0001')
for terrain in ('flat','slope'):
 p=candidate/f'{terrain}-dr0'
 a=np.load(p/f'{terrain}.npz')
 tape=np.load(p/f'{terrain}-sensor-tape.npz')
 ctx=json.loads((p/'replay-context.json').read_text())
 shape=a['q'].shape[:2]
 keys=('pose','q','vel','cmd','fall','tilt','normal_load','wheel_target','wheel_data','clearance','nonwheel','target_trace')
 rec={k:np.zeros_like(a[k]) for k in keys}
 summary={}
 for i,label in enumerate(a['labels']):
  grade=0 if terrain=='flat' else ctx['slope_spawn_grade_deg'][i]
  angle=np.deg2rad(grade)
  ground.set('quat',f'{np.cos(angle/2)} 0 {np.sin(angle/2)} 0')
  x0=a['pose'][0,i,0]
  # Approximate the local slope as a plane anchored at the sampled wheel footprint.
  center=a['wheel_data'][0,i,:,:3]
  z0=0 if terrain=='flat' else float(np.median(center[:,2]+np.tan(angle)*(center[:,0]-x0)-.086/np.cos(angle)))
  ground.set('pos',f'{x0} 0 {z0}')
  m=mj.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'))
  d=mj.MjData(m)
  names=a['joints'].tolist()
  j=[m.joint(n).id for n in names]
  q=m.jnt_qposadr[j]; v=m.jnt_dofadr[j]
  wheels=[m.body(n).id for n in names[12:]]
  floor=m.geom('ground').id
  lim=np.array([float(urdf.find(f"joint[@name='{n}']/limit").get('effort')) for n in names])
  d.qpos[:3]=a['pose'][0,i,:3]; d.qpos[3:7]=tape['quat_wxyz'][0,i]
  d.qpos[q]=tape['q_rad'][0,i]; d.qvel[v]=tape['dq_rad_s'][0,i]
  d.qvel[3:6]=tape['gyro_rad_s'][0,i]
  rot=np.empty(9); mj.mju_quat2Mat(rot,d.qpos[3:7]); d.qvel[:3]=rot.reshape(3,3)@np.r_[a['vel'][0,i,:2],0.]
  prior=tape['prior_target'][0,i].copy()
  mj.mj_forward(m,d)
  fallen=False
  for k in range(shape[0]):
   cmd=a['cmd'][k,i]
   velocity=np.zeros(6)
   mj.mj_objectVelocity(m,d,mj.mjtObj.mjOBJ_BODY,m.body('base_link').id,velocity,1)
   obs=observation(d.qpos[q],d.qvel[v],velocity[:3],d.qpos[3:7],cmd,prior)
   raw=session.run(None,{'observation':obs[None]})[0][0]
   prior=wrap(prior,raw,cmd)
   for _ in range(4):
    torque=np.r_[55*(DEFAULT[:12]+prior[:12]-d.qpos[q[:12]])-2*d.qvel[v[:12]],2*(prior[12:]-d.qvel[v[12:]])]
    d.qfrc_applied[v]=np.clip(torque,-lim,lim)
    mj.mj_step(m,d)
   mj.mj_forward(m,d)
   forces=np.zeros(4); nonwheel=False
   for c in range(d.ncon):
    g1,g2=d.contact[c].geom1,d.contact[c].geom2
    if floor not in (g1,g2):continue
    body=m.geom_bodyid[g2 if g1==floor else g1]
    force=np.zeros(6);mj.mj_contactForce(m,d,c,force)
    if body in wheels:forces[wheels.index(body)]+=force[0]
    elif force[0]>=8:nonwheel=True
   mj.mj_objectVelocity(m,d,mj.mjtObj.mjOBJ_BODY,m.body('base_link').id,velocity,1)
   tilt=np.degrees(np.arccos(np.clip(d.xmat[m.body('base_link').id,8],-1,1)))
   fallen=fallen or tilt>60 or d.qpos[2]-(z0-np.tan(angle)*(d.qpos[0]-x0))<.15
   rec['pose'][k,i]=d.qpos[[0,1,2,4,5,6,3]]
   rec['q'][k,i]=d.qpos[q]
   rec['vel'][k,i]=velocity[[3,4,2]]
   rec['cmd'][k,i]=cmd
   rec['fall'][k,i]=fallen
   rec['tilt'][k,i]=tilt
   rec['normal_load'][k,i]=forces
   rec['wheel_target'][k,i]=np.mean(abs(prior[12:]))
   rec['wheel_data'][k,i,:,:3]=d.xpos[wheels]
   normal=np.array([np.sin(angle),0,np.cos(angle)])
   rec['clearance'][k,i]=(d.xpos[wheels]-[x0,0,z0])@normal-.086
   rec['nonwheel'][k,i]=nonwheel
   rec['target_trace'][k,i]=[DEFAULT[:12]+np.clip(raw,-1,1)*.25,DEFAULT[:12]+prior[:12]]
 for label in dict.fromkeys(a['labels']):
  ids=np.flatnonzero(a['labels']==label)
  def measure(data):
   ok=~np.maximum.accumulate(data['fall'][:,ids],axis=0)[50:]
   f=data['normal_load'][50:,ids]
   active=(data['clearance'][50:,ids]>=.01)&(f<3)&ok[...,None]
   lifts=np.zeros(4,int)
   for j in range(len(ids)):
    for w in range(4):
     x=np.diff(np.r_[False,active[:,j,w],False].astype(int))
     lifts[w]+=np.sum(np.flatnonzero(x==-1)-np.flatnonzero(x==1)>=3)
   return dict(survived=int(ok[-1].sum()),trials=len(ids),
     support=float((f>=3).all(-1)[ok].mean()) if ok.any() else None,
     lifts=lifts.tolist(),mae=np.abs(data['vel'][50:,ids]-data['cmd'][50:,ids])[ok].mean(0).tolist() if ok.any() else None)
  summary[str(label)]=dict(isaac=measure(a),mujoco=measure(rec))
 dest=out/terrain;dest.mkdir(exist_ok=True)
 np.savez_compressed(dest/f'{terrain}.npz',labels=a['labels'],joints=a['joints'],**rec)
 (dest/'comparison.json').write_text(json.dumps(summary,indent=2))
 (dest/'replay-context.json').write_text(json.dumps(ctx))
 print(terrain,json.dumps(summary),flush=True)
