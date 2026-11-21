from pathlib import Path
p=Path(__file__).resolve().parent
exec((p/'open_loop.py').read_text().split('results={}')[0])
import hashlib
alternative={}
for n in names:
 j=u.find(f"joint[@name='{n}']")
 b=x.find(f".//body[@name='{n}']")
 assert np.allclose(np.fromstring(j.find('axis').get('xyz'),sep=' '),m.jnt_axis[m.joint(n).id]),n
 assert np.allclose(np.fromstring(j.find('origin').get('xyz'),sep=' '),m.body_pos[m.body(n).id]),n
for n in ('base_link','FL_thigh','FR_thigh','FL_foot'):
 link=u.find(f"link[@name='{n}']")
 alternative[n]=[ET.tostring(c.find('geometry'),encoding='unicode').strip() for c in link.findall('collision')]
folder=Path(r'X:/Items/as2w/AS2-W_CPRO/部署测试/04_调试记录/logs/AS2W官方运控基线数据包_20261119')
reports={}
for label,pattern in [('ground','03_*'),('suspended','04_*')]:
 rows=read(next(folder.glob(pattern))/'low_capture_fast.log','LOW_SAMPLE',110)
 rows=rows[(rows[:,0]>=2000)&(rows[:,0]<15000)]
 raw=rows[:,13:].reshape(-1,16,6)[:,[3,4,5,0,1,2,9,10,11,6,7,8,13,12,15,14],0].mean(0)
 reports[label]={}
 for hs in (-1,1):
  d=mj.MjData(m); d.qpos[2]=1
  d.qpos[3:7]=rows[0,6:10]; d.qpos[qa]=raw*np.array([hs,1,1]*4+[1]*4)
  mj.mj_forward(m,d)
  gaps={}; centers={}
  for name in names[12:]:
   g=int(m.body(m.body(name).id).geomadr[0]); mesh=m.geom_dataid[g]
   verts=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
   gaps[name]=float(min(verts@d.geom_xmat[g].reshape(3,3)[2]+d.geom_xpos[g,2]))
   centers[name]=d.xpos[m.body(name).id].tolist()
  reports[label][str(hs)]={'wheel_gap_mm':{n:round(1000*(v-min(gaps.values())),2) for n,v in gaps.items()},'front_track_m':float(d.xmat[m.body('base_link').id].reshape(3,3)[:,1] @ (np.array(centers['FL_foot'])-centers['FR_foot'])),'self_contacts':[(m.body(m.geom_bodyid[c.geom1]).name,m.body(m.geom_bodyid[c.geom2]).name,round(float(c.dist)*1000,2)) for c in d.contact]}
out={'joint_axes_and_origins_matched':16,'urdf_collision':alternative,'cases':reports}
(p/'geometry-audit.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))

