from pathlib import Path
p=Path(__file__).resolve().parent
exec((p/'open_loop.py').read_text().split('results={}')[0])
root=Path(r'X:/Items/as2w/AS2-W_CPRO/部署测试/04_调试记录/logs')
official=root/'AS2W官方运控基线数据包_20261119'
v2=root/'AS2W_悬空负载诊断V2_全部数据与分析_20261119/recordings/field_sessions'
slots=[3,4,5,0,1,2,9,10,11,6,7,8,13,12,15,14]
hip=np.arange(0,12,3)
def state(rows,hs,zero_wheels=False):
 d=mj.MjData(m);d.qpos[2]=2
 quat=rows[:,6:10].copy()
 quat[(quat@quat[0])<0]*=-1
 quat=quat.mean(0);d.qpos[3:7]=quat/np.linalg.norm(quat)
 s=np.ones(16);s[hip]=hs
 data=rows[:,13:].reshape(-1,16,6)[:,slots].mean(0)
 d.qpos[qa]=data[:,0]*s
 if zero_wheels:d.qpos[qa[12:]]=0
 mj.mj_forward(m,d)
 return d,data,s
rows=read(next(official.glob('03_*'))/'low_capture_fast.log','LOW_SAMPLE',110)
row=rows[(rows[:,0]==8500)&(rows[:,3]==8254106)]
assert len(row)==1
out={'frame':{'elapsed_ms':8500,'tick':8254106,'quat_wxyz':row[0,6:10].tolist()},'width':{},'gravity':{}}
for hs in (1,-1):
 d,_,_=state(row,hs,True)
 edges=[]
 for name in names[12:14]:
  b=m.body(name).id; g=m.body_geomadr[b]; mesh=m.geom_dataid[g]
  v=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
  f=m.mesh_face[m.mesh_faceadr[mesh]:m.mesh_faceadr[mesh]+m.mesh_facenum[mesh]]
  w=v@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
  tr=w[f]; a1=tr.reshape(-1,3); a2=np.roll(tr,-1,axis=1).reshape(-1,3)
  z=d.xpos[b,2]
  mask=((a1[:,2]-z)*(a2[:,2]-z)<=0)&(a1[:,2]!=a2[:,2])
  a1,a2=a1[mask],a2[mask]
  hit=a1+(a2-a1)*((z-a1[:,2])/(a2[:,2]-a1[:,2]))[:,None]
  edges.extend(hit[:,1].tolist())
 out['width'][str(hs)]=max(edges)-min(edges)
m.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_CONTACT)
for name,folder,pattern,lo,hi in [('official',official,'04_*',2,55),('T2_55',v2,'*194344*',21,110),('T2_85',v2,'*194837*',42,122)]:
 rows=read(next(folder.glob(pattern))/'low_capture_fast.log','LOW_SAMPLE',110)
 rows=rows[(rows[:,0]>=lo*1000)&(rows[:,0]<hi*1000)]
 out['gravity'][name]={}
 for hs in (1,-1):
  d,data,s=state(rows,hs)
  real=data[hip,3]; predicted=d.qfrc_bias[va[hip]]*s[hip]
  out['gravity'][name][str(hs)]={'real_dds_nm':real.tolist(),'sim_dds_nm':predicted.tolist(),'mae_nm':float(abs(predicted-real).mean())}
(p/'dr-hip-sign-audit.json').write_text(json.dumps(out,indent=2))
print(out)
