"""No-policy PhysX static contact sweep using the training articulation."""
import argparse,json,math
from pathlib import Path
from isaaclab.app import AppLauncher
p=argparse.ArgumentParser()
p.add_argument('--mu',type=float,required=True)
p.add_argument('--asset',required=True)
p.add_argument('--fixture',required=True)
p.add_argument('--out',required=True)
a=p.parse_args()
app=AppLauncher(headless=True).app
import torch
from arc_environment import QualityCfg,ArcEnv
from environment import tensor
cfg=QualityCfg(him=True,rough=False,randomize=False,seed=42)
cfg.scene.num_envs=1
cfg.scene.robot.spawn.asset_path=str(Path(a.asset).resolve())
cfg.configure_terrain()
mat=cfg.scene.terrain.physics_material
mat.static_friction=a.mu
mat.dynamic_friction=a.mu
mat.friction_combine_mode='min'
env=ArcEnv(cfg)
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
 goal=make([0,.8,-1.6]*4)[None]
 result=[]
 for k in range(1200):
  rob.set_joint_position_target(goal,joint_ids=env.leg_ids)
  rob.set_joint_velocity_target(torch.zeros(1,4,device=env.device),joint_ids=env.wheel_ids)
  env.scene.write_data_to_sim()
  env.sim.step(render=False)
  env.scene.update(env.physics_dt)
  if k>=1000 and k%4==0:
   quat=tensor(rob.data.root_quat_w)[0]
   x,y,z,w=map(float,quat)
   pitch=math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x)))))
   result.append(dict(pitch=pitch,forces=tensor(env.sensor.data.net_forces_w)[0,env.foot_sensors].tolist(),
                      gravity=tensor(rob.data.projected_gravity_b)[0].tolist(),
                      q=tensor(rob.data.joint_pos)[0,env.leg_ids].tolist(),
                      dq=tensor(rob.data.joint_vel)[0,env.leg_ids].tolist()))
 import numpy as np
 summary={key:np.mean([r[key] for r in result],axis=0).tolist() for key in result[0]}
 summary['mu']=a.mu
 summary['pd_nm']=(55*(np.array([0,.8,-1.6]*4)-summary['q'])-2*np.array(summary['dq'])).tolist()
 Path(a.out).write_text(json.dumps(summary,indent=2))
 print('STATIC_RESULT',json.dumps(summary))
finally:
 env.close()
 app.close()
