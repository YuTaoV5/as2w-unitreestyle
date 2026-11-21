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
import argparse, os
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
cfg.scene.robot.actuators["legs"].stiffness = kp
cfg.scene.robot.actuators["legs"].damping = kd
cfg.scene.robot.actuators["wheels"].damping = .5 if a.pd == "200" else 2.

env = ArcEnv(cfg)
stage = omni.usd.get_context().get_stage()

# The flat scene has a single shared plane. For the 0.25 contract its combine
# mode must be explicit; the native URDF comparison keeps whatever it had.
if a.usd or a.mu is not None:
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI) and str(prim.GetPath()).endswith("/CollisionPlane"):
            material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
            api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
            mode=os.environ.get("STAND_COMBINE","min")
            api.CreateFrictionCombineModeAttr().Set(mode)
            if "STAND_MU" in os.environ:
                mu=float(os.environ["STAND_MU"])
                mat=UsdPhysics.MaterialAPI(material.GetPrim())
                mat.CreateStaticFrictionAttr().Set(mu)
                mat.CreateDynamicFrictionAttr().Set(mu)

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

names=[f"{l}_{j}" for l in ("FL","FR","RL","RR") for j in ("hip","thigh","calf")]
leg_ids=[env.robot.joint_names.index(n) for n in names]
wheel_ids=[env.robot.joint_names.index(n) for n in wheel_names]
hip_ids=leg_ids[::3]
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
    fx = json.loads(Path(a.fixture).read_text())
    rob = env.robot

    def make(x):
        return torch.tensor(x, device=env.device, dtype=torch.float32)

    q = make(fx["q_dds"])[None]
    q[:, 12:] = 0
    rob.write_joint_state_to_sim(q, torch.zeros_like(q), joint_ids=leg_ids+wheel_ids)
    root = tensor(rob.data.root_pose_w).clone()
    root[:, 3:] = make(fx["quat_wxyz"])[[1, 2, 3, 0]]
    root[:, 2] = .37
    rob.write_root_pose_to_sim(root)
    rob.write_root_velocity_to_sim(torch.zeros(1, 6, device=env.device))
    goal = make(goal_values)[None]
    if a.hold_fixture:
        goal = q[:, :12].clone()
    for k in range(1200):
        rob.set_joint_position_target(goal, joint_ids=leg_ids)
        rob.set_joint_velocity_target(torch.zeros(1, 4, device=env.device), joint_ids=wheel_ids)
        env.scene.write_data_to_sim()
        env.sim.step(render=False)
        env.scene.update(env.physics_dt)
        if k >= 1000 and k % 4 == 0:
            quat = tensor(rob.data.root_quat_w)[0]
            x, y, z, w = map(float, quat)
            pitch = math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))
            applied = tensor(rob.data.applied_torque)[0]
            normal, friction = [], []
            for n in wheel_names:
                data = env.scene["ground_" + n].data
                normal.append(tensor(data.normal_force_matrix_w)[0, 0, 0].tolist())
                friction.append(tensor(data.friction_force_matrix_w)[0, 0, 0].tolist())
            result.append(dict(
                pitch_deg=pitch,
                hip_torque_nm=[float(applied[i]) for i in hip_ids],
                leg_torque_nm=[float(applied[i]) for i in leg_ids],
                wheel_normal_n=normal,
                wheel_tangent_n=friction,
                q=[float(v) for v in tensor(rob.data.joint_pos)[0, leg_ids]],
                dq=[float(v) for v in tensor(rob.data.joint_vel)[0, leg_ids]],
                gravity=[float(v) for v in tensor(rob.data.projected_gravity_b)[0]]))
    import numpy as np
    summary = {key: np.mean([r[key] for r in result], axis=0).tolist() for key in result[0]}
    summary["pitch_abs_max_deg"] = float(np.max(np.abs([r["pitch_deg"] for r in result])))
    summary["contract"] = {"asset": str(asset), "mode": "usd" if a.usd else "native",
                           "mu_override": a.mu, "pd": a.pd, "kp": kp, "kd": kd,
                           "hold_fixture": bool(a.hold_fixture)}
    summary["mass_contrast"] = mass_contrast
    summary["bindings"] = bindings
    summary["leg_names"] = names
    normals = np.array(summary["wheel_normal_n"])
    summary["wheel_loaded"] = [bool(v[2] > 3.) for v in normals]
    summary["all_wheels_loaded"] = bool(all(summary["wheel_loaded"]))
    Path(a.out).write_text(json.dumps(summary, indent=2) + "\n")
    print("STATIC_ACCEPTANCE_OK", a.out, flush=True)
    print(json.dumps({k: summary[k] for k in ("pitch_deg", "pitch_abs_max_deg", "hip_torque_nm",
                                              "wheel_normal_n", "wheel_tangent_n", "all_wheels_loaded")}, indent=2), flush=True)
finally:
    env.close()
    app.close()
