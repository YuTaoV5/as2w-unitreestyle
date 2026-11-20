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
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axs=plt.subplots(1,2,figsize=(11,5),sharex=True,sharey=True)
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
  if label=='ground':
   ax=axs[int(hs==1)]
   points=[]
   for name in names[:6]+[names[12],names[13]]:
    body=m.body(name)
    g=int(body.geomadr[0]); mesh=m.geom_dataid[g]
    verts=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]]
    world=verts@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
    local=(world-d.xpos[m.body('base_link').id])@d.xmat[m.body('base_link').id].reshape(3,3)
    ax.scatter(local[:,1]*100,local[:,2]*100,s=2,alpha=.32,color='#175f83' if name.endswith('foot') else '#86919b')
    if name.endswith('foot'): points.append(local)
   pts=np.vstack(points)
   lo,hi=pts[:,1].min()*100,pts[:,1].max()*100
   z=pts[:,2].min()*100-4
   ax.annotate('',(lo,z),(hi,z),arrowprops=dict(arrowstyle='<->',color='#a34018',lw=2))
   ax.vlines([lo,hi],z,pts[:,2].mean()*100,colors='#a34018',linestyles='dashed')
   ax.text((lo+hi)/2,z-2,f'Outer envelope: {hi-lo:.1f} cm',ha='center')
   ax.set(title='Current hip mapping' if hs==-1 else 'Reversed hip mapping',xlabel='Lateral position (cm)',aspect='equal',ylim=(z-6,8))
   ax.grid(alpha=.2)
  reports[label][str(hs)]={'wheel_gap_mm':{n:round(1000*(v-min(gaps.values())),2) for n,v in gaps.items()},'front_track_m':float(d.xmat[m.body('base_link').id].reshape(3,3)[:,1] @ (np.array(centers['FL_foot'])-centers['FR_foot'])),'self_contacts':[(m.body(m.geom_bodyid[c.geom1]).name,m.body(m.geom_bodyid[c.geom2]).name,round(float(c.dist)*1000,2)) for c in d.contact]}
out={'joint_axes_and_origins_matched':16,'urdf_collision':alternative,'cases':reports}
(p/'geometry-audit.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))


axs[0].set_ylabel('Height relative to base (cm)')
fig.suptitle('Official standing pose | front wheel mesh projection')
fig.tight_layout()
fig.savefig(p/'front-wheel-measurement.png',dpi=180)
