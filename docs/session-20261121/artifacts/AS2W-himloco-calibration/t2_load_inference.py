from pathlib import Path
p=Path(__file__).resolve().parent
exec((p/'open_loop.py').read_text().split('results={}')[0])
folder=Path(r'X:/Items/as2w/AS2-W_CPRO/部署测试/04_调试记录/logs/AS2W_悬空负载诊断V2_全部数据与分析_20261119/recordings/field_sessions')
out={}
for key,lo,hi in [('182802',10,30),('185930',7,20),('190012',8,20)]:
 rows=read(next(folder.glob(f'*{key}*'))/'low_capture_fast.log','LOW_SAMPLE',110)
 rows=rows[(rows[:,0]>=lo*1000)&(rows[:,0]<hi*1000)]
 q,_,_,tau,*_=rows[:,13:].reshape(-1,16,6)[:,[3,4,5,0,1,2,9,10,11,6,7,8,13,12,15,14]].transpose(2,0,1)*sign
 r=json.loads((p/'t2-ground-exclude.json').read_text())[f'*{key}*']
 d=mj.MjData(m); d.qpos[2]=1
 def calculate(quat,angles):
  d.qpos[3:7]=quat; d.qpos[qa]=angles; mj.mj_forward(m,d)
  result={}; columns=[]; gs=d.qfrc_bias[va[:12]].copy()
  for k,n in enumerate(names[12:]):
   b=m.body(n).id; g=m.body_geomadr[b]; mesh=m.geom_dataid[g]
   v=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
   w=v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
   points=w[w[:,2]<=w[:,2].min()+.001]
   jac=np.zeros((3,m.nv))
   mj.mj_jac(m,d,jac,None,points.mean(0),b)
   columns.append(jac[2,va[:12]])
   j=ids[3*k]; axis=d.xaxis[j]; arms=np.cross(points-d.xanchor[j], [0,0,1])@axis
   arm=float(arms.mean()); grav=float(d.qfrc_bias[va[3*k]])
   F=r['wheel_normal_n'][k]; real=float(tau[:,3*k].mean())
   result[n]=dict(arm_m=arm,arm_range_m=[float(min(arms)),float(max(arms))],gravity_nm=grav,normal_n=F,predicted_nm=grav-arm*F,real_nm=real,required_normal_n=(grav-real)/arm)
  A=np.array(columns).T
  target=gs-tau.mean(0)[:12]
  load=np.linalg.lstsq(A,target,rcond=None)[0]
  residual=A@load-target
  result['fit']=dict(load_n=load.tolist(),mass_kg=float(load.sum()/9.81),rear_share=float(load[2:].sum()/load.sum()),mae_nm=float(abs(residual).mean()),residual_nm=residual.tolist())
  return result
 quat=rows[:,6:10].mean(0); quat/=np.linalg.norm(quat)
 out[key]=calculate(quat,q.mean(0))
(p/'t2-load-inference.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))

