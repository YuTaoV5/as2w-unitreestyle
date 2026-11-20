"""No-policy static stand acceptance for the HIMLoco contact contract.

Runs the training articulation against a fixed static pose fixture, holds the
PD targets used by the field log, and reports:
  * pitch from the root quaternion
  * signed applied torque on the four hips (FL, FR, RL, RR)
  * per-wheel ground-only normal force and tangential (friction) force vector
  * joint position/velocity and per-wheel loading

Two contact contracts:
  --native  keep the original URDF contact setup untouched
  --usd     load the reviewed mu025 binary USD (wheels 0.25, ground 0.25, min)

Output is JSON so the contrast can be reviewed per trial.
"""
import os
import argparse
import json
import math
from pathlib import Path

from isaaclab.app import AppLauncher

p = argparse.ArgumentParser()
p.add_argument("--asset", required=True)
p.add_argument("--fixture", required=True)
p.add_argument("--pd", choices=("55", "200"), default="55")
p.add_argument("--native", action="store_true")
p.add_argument("--usd", action="store_true")
p.add_argument("--mu", type=float, default=None,
               help="override ground friction for the native URDF comparison")
p.add_argument("--base-mass-delta", type=float, default=0.,
               help="single-variable mass contrast on base_link (kg)")
p.add_argument("--com-delta-x", type=float, default=0.,
               help="single-variable CoM contrast on base_link (m)")
p.add_argument("--hold-fixture", action="store_true",
               help="drive the legs to the fixture pose itself instead of the standard stand goal")
p.add_argument("--out", required=True)
a = p.parse_args()
assert a.native != a.usd, "choose exactly one contact contract"

kp, kd = (200., 4.) if a.pd == "200" else (55., 2.)
goal_values = [-.15, .6, -1.2, .15, .6, -1.2] * 2 if a.pd == "200" else [0., .8, -1.6] * 4

app = AppLauncher(headless=True).app
import torch
from arc_environment import QualityCfg, ArcEnv
from environment import tensor

cfg = QualityCfg(him=True, rough=False, randomize=False, seed=42)
asset = Path(a.asset).resolve(strict=True)
cfg.scene.num_envs = 1
cfg.configure_terrain()  # terrain material must exist before the USD contract binds it
if a.usd:
    from asset_contract import configure_asset
    configure_asset(cfg, asset, asset.parent / "usd_cache")
else:
    cfg.scene.robot.spawn.asset_path = str(asset)
if not a.usd and a.mu is not None:
    mat = cfg.scene.terrain.physics_material
    mat.static_friction = a.mu
    mat.dynamic_friction = a.mu
    mat.friction_combine_mode = "min"

from isaaclab.sensors import ContactSensorCfg
import omni.usd
from pxr import Usd, UsdPhysics, UsdShade, PhysxSchema

wheel_names = ("FL_foot", "FR_foot", "RL_foot", "RR_foot")
for name in wheel_names:
    setattr(cfg.scene, "ground_" + name, ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/Geometry/.*/" + name,
        filter_prim_paths_expr=["/World/ground/terrain/GroundPlane/CollisionPlane"],
        track_friction_forces=True))
cfg.scene.robot.actuators["legs"].stiffness = 0.
cfg.scene.robot.actuators["legs"].damping = 0.
cfg.scene.robot.actuators["wheels"].damping = 0.

env = ArcEnv(cfg)
stage = omni.usd.get_context().get_stage()

# The flat scene has a single shared plane. For the 0.25 contract its combine
# mode must be explicit; the native URDF comparison keeps whatever it had.
if a.usd or a.mu is not None:
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI) and str(prim.GetPath()).endswith("/CollisionPlane"):
            material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
            api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
            api.CreateFrictionCombineModeAttr().Set("min")

bindings = []
for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
    path = str(prim.GetPath())
    if not prim.HasAPI(UsdPhysics.CollisionAPI) or not ("/ground" in path or any(n in path for n in wheel_names)):
        continue
    if "/envs/" in path and not path.startswith("/World/envs/env_0/"):
        continue
    material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
    entry = {"shape": path, "material": str(material.GetPath()) if material else None}
    if material:
        mp = material.GetPrim()
        entry.update({at.GetName(): at.Get()
                      for at in mp.GetAttributes()
                      if "friction" in at.GetName().lower() or "combine" in at.GetName().lower()})
        api = PhysxSchema.PhysxMaterialAPI(mp)
        entry["combine"] = api.GetFrictionCombineModeAttr().Get() if api else None
    bindings.append(entry)

leg_ids = list(env.leg_ids)
hip_ids = leg_ids[:4]
mass_contrast = None
result = []
try:
    env.reset()
    if a.base_mass_delta or a.com_delta_x:
        import warp as wp
        view = env.robot.root_physx_view
        body_ids = [env.robot.body_names.index("base_link")]
        masses = torch.as_tensor(view.get_masses(), device=env.device, dtype=torch.float32).clone()
        coms = torch.as_tensor(view.get_coms(), device=env.device, dtype=torch.float32).clone()
        masses[0, body_ids[0]] += a.base_mass_delta
        coms[0, body_ids[0], 0] += a.com_delta_x
        env.robot.set_masses_index(masses=wp.from_torch(masses[:, :1].contiguous()),
                                   body_ids=body_ids, env_ids=[0])
        env.robot.set_coms_index(coms=wp.from_torch(coms[:, :1].contiguous(), dtype=wp.transformf),
                                 body_ids=body_ids, env_ids=[0])
        got_m = float(torch.as_tensor(view.get_masses())[0, body_ids[0]])
        got_c = float(torch.as_tensor(view.get_coms())[0, body_ids[0], 0])
        assert abs(got_m - masses[0, body_ids[0]]) < 1e-5, (got_m, masses[0, body_ids[0]])
        assert abs(got_c - coms[0, body_ids[0], 0]) < 1e-5, (got_c, coms[0, body_ids[0], 0])
        mass_contrast = {"base_mass_kg": got_m, "base_com_x_m": got_c,
                         "delta_mass_kg": a.base_mass_delta, "delta_com_x_m": a.com_delta_x}
        print("MASS_CONTRAST", json.dumps(mass_contrast), flush=True)

    import numpy as np
    trace=np.load(a.fixture)
    rob=env.robot
    device=env.device
    make=lambda x:torch.as_tensor(x,device=device,dtype=torch.float32)
    names=trace["names"].tolist()
    ids=[rob.joint_names.index(n) for n in names]
    limit=tensor(rob.data.joint_effort_limits)[0,ids] if hasattr(rob.data,"joint_effort_limits") else make([60,60,90]*4+[15]*4)
    root=tensor(rob.data.root_pose_w).clone()
    root[:,2]=float(os.environ["REPLAY_HEIGHT"])
    root[:,3:]=make(trace["quat"][0])[[1,2,3,0]]
    rob.write_root_pose_to_sim(root)
    rob.write_root_velocity_to_sim(torch.zeros(1,6,device=device))
    rob.write_joint_state_to_sim(make(trace["q"][0])[None],make(trace["dq"][0])[None],joint_ids=ids)
    commands=make(trace["commands"])
    times=trace["ct"]
    dt=env.physics_dt
    rows=[]
    for k in range(round(13/dt)):
        t=68+k*dt
        goal,dq_des,kp,kd,ff=commands[max(0,np.searchsorted(times,t,side="right")-1)].T
        q=tensor(rob.data.joint_pos)[0,ids]
        dq=tensor(rob.data.joint_vel)[0,ids]
        effort=torch.clamp(kp*(goal-q)+kd*(dq_des-dq)+ff,-limit,limit)
        rob.set_joint_effort_target(effort[None],joint_ids=ids)
        env.scene.write_data_to_sim()
        env.sim.step(render=False)
        env.scene.update(dt)
        if k%4==0:
            normal=[]; tangent=[]
            for n in wheel_names:
                data=env.scene["ground_"+n].data
                normal.append(tensor(data.normal_force_matrix_w)[0,0,0].tolist())
                tangent.append(tensor(data.friction_force_matrix_w)[0,0,0].tolist())
            rows.append(dict(t=t,q=q.tolist(),dq=dq.tolist(),effort=effort.tolist(),target=goal.tolist(),
                velocity_target=dq_des.tolist(),normal=normal,tangent=tangent,
                root=tensor(rob.data.root_pose_w)[0].tolist()))
    Path(a.out).write_text(json.dumps(dict(bindings=bindings,mass_contrast=mass_contrast,names=names,rows=rows),allow_nan=False))
    print("REPLAY_OK",a.out,flush=True)
finally:
    env.close()
    app.close()
