from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco as mj
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path.cwd()
out=root/'AS2W-official-stop-replay/v2-sign'
p=Path(r'X:/Items/as2w/AS2-W_CPRO/部署测试/04_调试记录/logs/AS2W官方运控基线数据包_20261119/02_walk_155953')
asset=root/'AS2W-github-publish/unified_dr/simulation/assets/load-v0'
template=root/'AS2W-github-publish/review/u3220/model/as2w_flat.xml'
slots=[3,4,5,0,1,2,9,10,11,6,7,8,13,12,15,14]
sign=np.ones(16)
names=[f'{l}_{j}' for l in ('FL','FR','RL','RR') for j in ('hip','thigh','calf')]+[f'{l}_foot' for l in ('FL','FR','RL','RR')]
def read(file,tag,start,end):
 rows=[]
 with (p/file).open() as f:
  for l in f:
   if not l.startswith(tag+','):continue
   a=l.split(',')
   if len(a)<110 or not start<=float(a[1])/1000<=end:continue
   if tag=='LOWCMD_SAMPLE' and a[9]!='1':continue
   rows.append([float(x) for x in a[1:110]])
 return np.array(rows)
st=read('low_capture_fast.log','LOW_SAMPLE',66,90)
cm=read('lowcmd_capture.log','LOWCMD_SAMPLE',66,90)
print('rows',len(st),len(cm))
tick,idx=np.unique(st[:,3],return_index=True)
assert np.all(np.diff(st[:,3])>=0)
ct=np.interp(cm[:,5],tick,st[idx,0])*1e-3
print('cmd-state alignment offset ms',np.quantile(ct*1000-cm[:,0],[0,.5,1]))
data=cm[:,13:].reshape(-1,16,6)[:,slots]
real=st[:,13:].reshape(-1,16,6)[:,slots]
urdf=ET.parse(asset/'as2w-load-v0.urdf').getroot()
xml=ET.parse(template).getroot()
for mesh in xml.findall('asset/mesh'):
 mesh.set('file',str(asset/'meshes'/Path(mesh.get('file')).name))
for b in xml.findall('.//body'):
 inertial=b.find('inertial')
 ref=urdf.find(f"link[@name='{b.get('name')}']/inertial")
 if inertial is None or ref is None:continue
 origin=ref.find('origin')
 assert np.allclose(np.fromstring(origin.get('rpy','0 0 0'),sep=' '),0)
 inertial.attrib.clear()
 for k,v in dict(pos=origin.get('xyz'),mass=ref.find('mass').get('value'),fullinertia=' '.join(ref.find('inertia').get(k) for k in ('ixx','iyy','izz','ixy','ixz','iyz'))).items():inertial.set(k,v)
xml.remove(xml.find('actuator'))
xml.find('option').set('timestep','.00125')
m=mj.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'))
j=[m.joint(name).id for name in names]
qa=m.jnt_qposadr[j]; va=m.jnt_dofadr[j]
limit=np.array([float(urdf.find(f"joint[@name='{name}']/limit").get('effort')) for name in names])
wheelb=[m.body(name).id for name in names[12:]]
ground=m.geom('ground').id
report={}
for key,start,end in [('transition',68,81),('static',77,85)]:
 d=mj.MjData(m)
 i=np.argmin(abs(st[:,0]/1000-start))
 d.qpos[:3]=[0,0,.4]
 d.qpos[3:7]=st[i,6:10]
 d.qpos[qa]=real[i,:,0]*sign
 d.qvel[va]=real[i,:,1]*sign
 d.qvel[3:6]=st[i,10:13]
 mj.mj_forward(m,d)
 bottom=[]
 for g in range(m.ngeom):
  if m.geom_bodyid[g] not in wheelb:continue
  mesh=m.geom_dataid[g]
  verts=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
  bottom.append(np.min(verts@d.geom_xmat[g].reshape(3,3)[2]+d.geom_xpos[g,2]))
 d.qpos[2]-=min(bottom)
 mj.mj_forward(m,d)
 rows=[]
 for k in range(round((end-start)/m.opt.timestep)):
  t=start+k*m.opt.timestep
  c=data[np.clip(np.searchsorted(ct,t,side='right')-1,0,len(ct)-1)]
  goal=c[:,1]*sign
  vel=c[:,2]*sign
  def pd(q,dq):
   return c[:,3]*(goal-q)+c[:,4]*(vel-dq)+c[:,5]*sign
  simulated=pd(d.qpos[qa],d.qvel[va])
  applied=np.clip(simulated,-limit,limit)
  d.qfrc_applied[va]=applied
  if k%8==0:
   ii=np.clip(np.searchsorted(st[:,0],t*1000),0,len(st)-1)
   sample=real[ii]
   nforce=np.zeros(4)
   for cc in range(d.ncon):
    g1,g2=d.contact[cc].geom1,d.contact[cc].geom2
    if ground not in (g1,g2):continue
    b=m.geom_bodyid[g2 if g1==ground else g1]
    if b not in wheelb:continue
    f=np.empty(6);mj.mj_contactForce(m,d,cc,f)
    nforce[wheelb.index(b)]+=f[0]
   rows.append((t,goal.copy(),vel.copy(),sample[:,0]*sign,d.qpos[qa].copy(),sample[:,1]*sign,d.qvel[va].copy(),sample[:,3]*sign,pd(sample[:,0]*sign,sample[:,1]*sign),applied.copy(),nforce,d.qpos[:7].copy()))
  mj.mj_step(m,d)
 t,goal,vel,rq,sq,rdq,sdq,rtau,rpd,stau,load,pose=map(np.asarray,zip(*rows))
 np.savez_compressed(out/(key+'.npz'),time=t,goal=goal,wheel_target=vel,real_q=rq,sim_q=sq,real_dq=rdq,sim_dq=sdq,real_tau=rtau,real_pd=rpd,sim_tau=stau,sim_wheel_load=load,sim_pose=pose,joints=names)
 for a,b in ([(68,70),(71,73),(73,76),(77,81)] if key=='transition' else [(80,85)]):
  mask=(t>=a)&(t<b)
  report[f'{key} {a}-{b}']={n:np.round(v,3).tolist() for n,v in dict(q_mae=np.mean(abs(rq[mask]-sq[mask]),axis=0),tau_mae=np.mean(abs(rtau[mask]-stau[mask]),axis=0),real_pd_mae=np.mean(abs(rtau[mask]-rpd[mask]),axis=0),real_tau=rtau[mask].mean(0),sim_tau=stau[mask].mean(0),real_eq=(goal-rq)[mask].mean(0),sim_eq=(goal-sq)[mask].mean(0),sim_load=load[mask].mean(0)).items()}
 for title,a,b,c in [('angle',rq,sq,goal),('torque',rtau,stau,rpd)]:
  fig,axes=plt.subplots(4,3,figsize=(13,8),sharex=True)
  for j,ax in enumerate(axes.flat):
   ax.plot(t,a[:,j],label='real measured')
   ax.plot(t,b[:,j],label='MuJoCo')
   ax.plot(t,c[:,j],':',label='command' if title=='angle' else 'PD from real state')
   ax.set_title(names[j]); ax.grid(alpha=.2)
  axes[0,0].legend(fontsize=7)
  fig.suptitle(f'Official LowCmd replay | {key} | {title}')
  fig.tight_layout();fig.savefig(out/f'{key}-{title}.png',dpi=130);plt.close(fig)
(out/'metrics.json').write_text(json.dumps(dict(joints=names,windows=report),indent=2))

