from pathlib import Path
p=Path(__file__).resolve().parent
exec((p/'open_loop.py').read_text().split('results={}')[0])
folder=Path(r'X:/Items/as2w/AS2-W_CPRO/部署测试/04_调试记录/logs/AS2W_悬空负载诊断V2_全部数据与分析_20261119/recordings/field_sessions')
results={}
for pattern,lo,hi in [('*182802*',10,30),('*185930*',7,20),('*190012*',8,20)]:
 rows=read(next(folder.glob(pattern))/'low_capture_fast.log','LOW_SAMPLE',110)
 rows=rows[(rows[:,0]>=lo*1000)&(rows[:,0]<hi*1000)]
 slots=[3,4,5,0,1,2,9,10,11,6,7,8,13,12,15,14]
 q,_,_,tau,*_=rows[:,13:].reshape(-1,16,6)[:,slots].transpose(2,0,1)*sign
 target=np.r_[np.tile([0.,.8,-1.6],4),np.zeros(4)]
 d=mj.MjData(m);d.qpos[2]=1;d.qpos[3:7]=rows[0,6:10];d.qpos[qa]=q.mean(0)
 mj.mj_forward(m,d)
 wheels=[m.body(n).id for n in names[12:]]
 bottoms=[]
 for b in wheels:
  g=int(m.body_geomadr[b]);mesh=m.geom_dataid[g]
  verts=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
  bottoms.append(min(verts@d.geom_xmat[g].reshape(3,3)[2]+d.geom_xpos[g,2]))
 d.qpos[2]-=min(bottoms)
 ground=m.geom('ground').id
 samples=[]
 for k in range(5000):
  effort=np.r_[55*(target[:12]-d.qpos[qa[:12]])-2*d.qvel[va[:12]],-2*d.qvel[va[12:]]]
  effort=np.clip(effort,-limit,limit);d.qfrc_applied[va]=effort
  mj.mj_step(m,d)
  if k<4000:continue
  loads=np.zeros(4);other=0.
  for i,c in enumerate(d.contact):
   if ground not in (c.geom1,c.geom2):continue
   b=m.geom_bodyid[c.geom2 if c.geom1==ground else c.geom1]
   force=np.zeros(6);mj.mj_contactForce(m,d,i,force)
   if b in wheels:loads[wheels.index(b)]+=force[0]
   else:other+=force[0]
  samples.append(np.r_[d.qpos[qa[:12]],effort[:12],loads,pitch(d.qpos[3:7]),other])
 s=np.mean(samples,axis=0)
 results[pattern]=dict(q_mae_rad=float(abs(s[:12]-q.mean(0)[:12]).mean()),tau_mae_nm=float(abs(s[12:24]-tau.mean(0)[:12]).mean()),wheel_normal_n=s[24:28].tolist(),pitch_sim=float(s[28]),pitch_real=float(pitch(rows[:,6:10]).mean()),other_ground_n=float(s[29]),rear_calf_sim=(target[[8,11]]-s[[8,11]]).tolist(),rear_calf_real=(target[[8,11]]-q.mean(0)[[8,11]]).tolist(),sim_tau_nm=s[12:24].tolist(),real_tau_nm=tau.mean(0)[:12].tolist())
case='exclude' if os.getenv('EXCLUDE_FRONT_THIGHS') else 'native'
case+=os.getenv('CASE_SUFFIX','')
(p/f't2-ground-{case}.json').write_text(json.dumps(results,indent=2))
for key,r in results.items(): print(case,key,{k:v for k,v in r.items() if not k.endswith('_tau_nm')})


