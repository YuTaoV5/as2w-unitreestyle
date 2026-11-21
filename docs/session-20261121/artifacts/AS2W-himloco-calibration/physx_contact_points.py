"""No-policy PhysX static contact sweep using the training articulation."""
import argparse,json,math
from pathlib import Path
from isaaclab.app import AppLauncher
p=argparse.ArgumentParser()
p.add_argument('--mu',type=float,required=True)
p.add_argument('--asset',required=True)
p.add_argument('--fixture',required=True)
p.add_argument('--out',required=True)
p.add_argument('--official',action='store_true')
p.add_argument('--native',action='store_true')
p.add_argument("--physics-dt",type=float,default=.005)
a=p.parse_args()
kp,kd=(200.,4.) if a.official else (55.,2.)
goal_values=[-.15,.6,-1.2,.15,.6,-1.2]*2 if a.official else [0,.8,-1.6]*4
app=AppLauncher(headless=True).app
import torch
from arc_environment import QualityCfg,ArcEnv
from environment import tensor
cfg=QualityCfg(him=True,rough=False,randomize=False,seed=42)
cfg.scene.num_envs=1
cfg.sim.dt=a.physics_dt
cfg.decimation=round(.020/a.physics_dt)
cfg.scene.robot.spawn.asset_path=str(Path(a.asset).resolve())
cfg.configure_terrain()
mat=cfg.scene.terrain.physics_material
mat.static_friction=a.mu
mat.dynamic_friction=a.mu
mat.friction_combine_mode='min'
from isaaclab.sensors import ContactSensorCfg
import omni.usd
from pxr import Usd, UsdPhysics, UsdShade, PhysxSchema
wheel_names=('FL_foot','FR_foot','RL_foot','RR_foot')
for name in wheel_names:
 setattr(cfg.scene,'ground_'+name,ContactSensorCfg(
  prim_path='{ENV_REGEX_NS}/Robot/Geometry/.*/'+name,
  filter_prim_paths_expr=['/World/ground/terrain/GroundPlane/CollisionPlane'],
  track_friction_forces=True, track_contact_points=True, max_contact_data_count_per_prim=32))
cfg.scene.robot.actuators['legs'].stiffness=kp
cfg.scene.robot.actuators['legs'].damping=kd
cfg.scene.robot.actuators['wheels'].damping=.5 if a.official else 2.
env=ArcEnv(cfg)
stage=omni.usd.get_context().get_stage()
if not a.native:
 for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
  if prim.HasAPI(UsdPhysics.CollisionAPI) and str(prim.GetPath()).endswith('/CollisionPlane'):
   material,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
   api=PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
   assert api
   api.CreateFrictionCombineModeAttr().Set('min')
bindings=[]
for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
 path=str(prim.GetPath())
 if not prim.HasAPI(UsdPhysics.CollisionAPI) or not ('/ground' in path or any(n in path for n in wheel_names)):
  continue
 material,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
 entry={'shape':path,'material':str(material.GetPath()) if material else None}
 if material:
  mp=material.GetPrim()
  entry.update({at.GetName():at.Get() for at in mp.GetAttributes()
                if 'friction' in at.GetName().lower() or 'combine' in at.GetName().lower()})
  api=PhysxSchema.PhysxMaterialAPI(mp)
  entry['combine']=api.GetFrictionCombineModeAttr().Get() if api else None
 bindings.append(entry)
print('MATERIAL_BINDINGS',json.dumps(bindings),flush=True)
try:
 env.reset()
 fx=json.loads(Path(a.fixture).read_text())
 rob=env.robot
 make=lambda x:torch.tensor(x,device=env.device,dtype=torch.float32)
 q=make(fx['q_dds'])[None]; q[:,12:]=0
 rob.write_joint_state_to_sim(q,torch.zeros_like(q),joint_ids=env.ids)
 root=tensor(rob.data.root_pose_w).clone()
 root[:,3:]=make(fx['quat_wxyz'])[[1,2,3,0]]
 root[:,2]=.37
 rob.write_root_pose_to_sim(root)
 rob.write_root_velocity_to_sim(torch.zeros(1,6,device=env.device))
 goal=make(goal_values)[None]
 result=[]
 for k in range(round(6/a.physics_dt)):
  rob.set_joint_position_target(goal,joint_ids=env.leg_ids)
  rob.set_joint_velocity_target(torch.zeros(1,4,device=env.device),joint_ids=env.wheel_ids)
  env.scene.write_data_to_sim()
  env.sim.step(render=False)
  env.scene.update(env.physics_dt)
  if k>=round(5/a.physics_dt) and k%cfg.decimation==0:
   quat=tensor(rob.data.root_quat_w)[0]
   x,y,z,w=map(float,quat)
   pitch=math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x)))))
   contact_points=[]
   ground_normal=[]
   ground_friction=[]
   for n in wheel_names:
    data=env.scene['ground_'+n].data
    contact_points.append(tensor(data.contact_pos_w)[0,0,0].tolist())
    ground_normal.append(tensor(data.normal_force_matrix_w)[0,0,0].tolist())
    ground_friction.append(tensor(data.friction_force_matrix_w)[0,0,0].tolist())
   result.append(dict(contact_points=contact_points,ground_normal=ground_normal,ground_friction=ground_friction,pitch=pitch,forces=tensor(env.sensor.data.net_forces_w)[0,env.foot_sensors].tolist(),
                      gravity=tensor(rob.data.projected_gravity_b)[0].tolist(),
                      q=tensor(rob.data.joint_pos)[0,env.leg_ids].tolist(),
                      dq=tensor(rob.data.joint_vel)[0,env.leg_ids].tolist()))
 import numpy as np
 summary={key:np.mean([r[key] for r in result],axis=0).tolist() for key in result[0]}
 summary['mu']=a.mu
 summary['bindings']=bindings
 summary['pd_nm']=(kp*(np.array(goal_values)-summary['q'])-kd*np.array(summary['dq'])).tolist()
 Path(a.out).write_text(json.dumps(summary,indent=2))
 print('STATIC_RESULT',json.dumps(summary))
finally:
 env.close()
 app.close()






