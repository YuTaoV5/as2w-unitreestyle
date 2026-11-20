"""HIMLoco flat-ground domain randomisation environment (ArcEnv subclass).

Implements the integration points listed in
dr-handoff/README.md 鎺ュ叆椤哄簭 1-3, WITHOUT touching the training baseline:

  1. per-env reset from a real field fixture (root pose + joint state), whole
     session held out of training by the sampler's split;
  2. a 100-step zero-velocity handoff that blends the applied target from the
     FixedStand contract target (0 offset from default) to the policy target,
     keeping leg 55/2, wheel 0/2, leg 10 Hz / wheel 5 Hz, and keeping the filter
     state (`self.target`) separate from the applied feedback (`self.applied`)
     exactly like the deployment fix requires;
  3. per-episode mass / CoM deltas on top of the v2 asset (never accumulated),
     plus friction and one-step latency for the last stage.

Deliberate omissions / guards:
  * friction needs a per-env ground collision body; the flat terrain here is a
    single shared plane, so the friction stage refuses to run until that exists
    (`_check_friction_supported`). Better a hard stop than randomising a
    material shared by 4096 environments.
  * the original `--stage randomized` events stay OFF (cfg.randomize=False):
    this module is the only source of randomness, per the handoff note.
  * root height comes from ground-contact geometry per frame, precomputed once
    (`_precompute_ground_offsets`): a single fixed height does not fit 471
    frames and a penetrating spawn gets launched by PhysX (observed).

The DDS hip sign is NOT assumed: it is a parameter, and run_dr.py --verify
prints the wheel-splay check that decides it (+1 on this URDF; -1 puts the
wheels 6.3 cm inward).
"""
from arc_environment import ArcEnv
from quality_environment import QualityEnv
from environment import RADIUS, tensor

import torch


class DREnv(ArcEnv):
    def __init__(self, cfg, sampler, stage, fixture_z=.362, handoff_steps=100,
                 leg_gain=.6, stand_leg_offset=None, stand_wheel=0., hip_sign=1.,
                 ground_margin=.002, log=None):
        self.sampler = sampler
        self.dr_stage = stage
        super().__init__(cfg)
        self.handoff_steps = int(handoff_steps)
        self.fixture_z = float(fixture_z)
        self.dr_leg_gain = float(leg_gain)
        self.dr_ground_margin = float(ground_margin)
        self.dr_log = log or (lambda msg: print(msg, flush=True))
        self.dr_sign = torch.tensor([hip_sign, 1., 1.] * 4 + [1.] * 4, device=self.device)
        self.dr_episode = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.dr_handoff_step = torch.full((self.num_envs,), self.handoff_steps,
                                          dtype=torch.long, device=self.device)
        self.dr_fixture_active = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.dr_stand_leg = (torch.zeros(self.num_envs, 12, device=self.device)
                             if stand_leg_offset is None
                             else torch.as_tensor(stand_leg_offset, device=self.device,
                                                  dtype=torch.float32).expand(self.num_envs, 12).clone())
        self.dr_stand_wheel = torch.full((self.num_envs, 4), float(stand_wheel), device=self.device)
        self.dr_masses = None
        self.dr_coms = None
        self.dr_last_sample = {}
        self.dr_last_fixture = {}
        self.dr_ground = {}
        self.dr_cmd_seen = 0.
        self.dr_written = {"mass": [], "com": [], "friction": [], "delay": []}
        self.dr_verify = {"fixture_episodes": 0, "nominal_episodes": 0}
        self._prepare_physics_baseline()
        self._check_friction_supported()
        self._precompute_ground_offsets()
        cfg_stage = sampler.stage(stage)
        self.dr_log("DR stage=%s handoff=%d leg_gain=%.2f hip_sign=%+.0f fixture_z=%.3f "
                    "field_probability=%.2f mass_delta=%s com_delta_x=%s friction=%s delay=%s"
                    % (stage, self.handoff_steps, self.dr_leg_gain, float(self.dr_sign[0]),
                       self.fixture_z, cfg_stage["field_probability"],
                       cfg_stage["base_mass_delta_kg"], cfg_stage["com_delta_x_m"],
                       cfg_stage["friction_scale"], cfg_stage["delay_steps"]))

    # 鈹€鈹€ physics baseline: capture v2 values so deltas never accumulate 鈹€鈹€鈹€鈹€鈹€鈹€
    def _prepare_physics_baseline(self):
        view = self.robot.root_physx_view
        bodies = self.robot.num_bodies

        def to_torch(value):
            return torch.as_tensor(value, device=self.device, dtype=torch.float32).clone()

        try:
            masses = view.get_masses()
        except Exception:
            masses = view.get_mass_matrix() if hasattr(view, "get_mass_matrix") else None
        if masses is None:
            self.dr_log("DR_WARN mass API unavailable; mass stage disabled")
            return
        masses = to_torch(masses)
        if masses.dim() == 1:
            masses = masses[None, :]
        self.dr_masses = masses
        try:
            coms = to_torch(view.get_coms())
        except Exception:
            coms = None
        if coms is not None and coms.dim() == 2:
            coms = coms[None, :, :]
        self.dr_coms = coms
        self.dr_log("DR_BASELINE bodies=%d total_mass=%.4f kg base_mass=%.4f kg base_com_x=%.4f"
                    % (bodies, float(self.dr_masses[0].sum()), float(self.dr_masses[0, 0]),
                       float(self.dr_coms[0, 0, 0]) if self.dr_coms is not None else float("nan")))

    def _precompute_ground_offsets(self):
        """Fixture spawn height.

        The fixture poses are all T2 stands inside a narrow band (thigh 0.70-0.83,
        calf -1.94..-1.72 rad), so their leg extension differs by only ~2-3 cm.
        Spawning at the height the baseline already proves safe (0.390 nominal,
        +1 cm clearance) means the worst case is a few cm of FLOAT, which the
        2 s handoff absorbs; penetration - the failure mode that makes PhysX
        launch the robot - cannot happen.  Per-frame FK refinement is deliberately
        left out: reading body_link_pos_w during __init__ proved unreliable on
        this build (it reported a wheel bottom 13 cm above the root).
        """
        frames = [(s["name"], f) for s in self.sampler.sessions for f in s["frames"]]
        self.dr_ground_uniform = .40
        self.dr_log("DR_GROUND frames=%d uniform_z=%.3f (nominal-safe 0.390 + clearance)" %
                    (len(frames), self.dr_ground_uniform))
    def _setup_scene(self):
        super()._setup_scene()
        lo, hi = self.sampler.stage(self.dr_stage)["friction_scale"]
        if lo == hi == 1.:
            return
        from isaaclab import sim
        from pxr import UsdPhysics, UsdShade, PhysxSchema
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        plane = stage.GetPrimAtPath("/World/ground/terrain/GroundPlane/CollisionPlane")
        assert plane and plane.HasAPI(UsdPhysics.CollisionAPI)
        UsdPhysics.CollisionAPI(plane).GetCollisionEnabledAttr().Set(False)
        self.dr_tile_materials = []
        spacing = self.cfg.scene.env_spacing
        for i, origin in enumerate(self.terrain.env_origins.cpu().tolist()):
            # Each environment has its own static tile and material. Scales stay
            # fixed within an environment across resets.
            scale = self.sampler.sample(self.dr_stage, i, 0)["friction_scale"]
            mu = .25 * scale
            path = f"/World/DRGround/tile_{i}"
            tile = sim.CuboidCfg(size=(spacing-.02, spacing-.02, .10),
                collision_props=sim.CollisionPropertiesCfg(),
                physics_material=sim.RigidBodyMaterialCfg(static_friction=mu,
                    dynamic_friction=mu, friction_combine_mode="min"))
            tile.func(path, tile, translation=(origin[0],origin[1],origin[2]-.05))
            material = stage.GetPrimAtPath(path+"/geometry/material")
            assert material and material.HasAPI(UsdPhysics.MaterialAPI)
            api = PhysxSchema.PhysxMaterialAPI.Apply(material)
            api.CreateFrictionCombineModeAttr().Set("min")
            actual = UsdPhysics.MaterialAPI(material).GetStaticFrictionAttr().Get()
            assert abs(actual-mu)<1e-6
            if i < 2:
                self.dr_tile_materials.append(str(material.GetPath()))
        assert len(self.dr_tile_materials)==2 and len(set(self.dr_tile_materials))==2
        print("DR_ISOLATED_TILES",self.num_envs,self.dr_tile_materials,flush=True)

    def _check_friction_supported(self):
        lo, hi = self.sampler.stage(self.dr_stage)["friction_scale"]
        if (lo != 1. or hi != 1.) and not getattr(self,"dr_tile_materials",None):
            raise RuntimeError("Isolated tile setup did not complete")

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = self.rows
        super()._reset_idx(env_ids)
        env_list = env_ids.tolist()
        self.dr_episode[env_ids] += 1
        frames = [self.sampler.sample(self.dr_stage, env, int(self.dr_episode[env]))
                  for env in env_list]
        self._apply_physics(env_ids, frames)
        fixture_ids, fixture_frames = [], []
        for env, sample in zip(env_list, frames):
            if sample["fixture"] is not None:
                fixture_ids.append(env)
                fixture_frames.append(sample["fixture"])
        self.dr_fixture_active[env_ids] = False
        self.dr_verify["nominal_episodes"] += len(env_list) - len(fixture_ids)
        for env_id in env_list:
            self.dr_last_fixture.pop(env_id, None)
        if fixture_ids:
            self._write_fixture(fixture_ids, fixture_frames)
            ids = torch.as_tensor(fixture_ids, device=self.device)
            self.dr_fixture_active[ids] = True
            self.dr_verify["fixture_episodes"] += len(fixture_ids)
        # handoff starts either way: the deployment's B press follows a FixedStand hold
        self.dr_handoff_step[env_ids] = 0
        # fixed-stand target is the contract pose (0 offset); queues, filter state and
        # applied feedback start there so the obs prev-target block matches what the
        # motors were actually commanded
        self.target[env_ids] = 0.
        self.dr_stand_leg[env_ids] = 0.
        self.target_queue[:, env_ids] = 0.
        self.applied[env_ids] = 0.
        self.raw_previous[env_ids] = 0.
        self.raw_delta[env_ids] = 0.
        self.raw_acc[env_ids] = 0.
        self.noise[env_ids] = 0.
        self.supported[env_ids] = False
        self.dr_last_sample.update({env: s for env, s in zip(env_list, frames)})
        for env, s in zip(env_list, frames):
            if s["fixture"] is not None:
                self.dr_last_fixture[env] = s["fixture"]

    def _apply_physics(self, env_ids, frames):
        """Per-episode mass / CoM / latency (never accumulated across episodes)."""
        needs_mass = any(abs(f["base_mass_delta_kg"]) > 1e-9 or abs(f["com_delta_x_m"]) > 1e-9
                         for f in frames)
        self.latency[env_ids] = torch.as_tensor(
            [f["delay_steps"] for f in frames], device=self.device, dtype=self.latency.dtype)
        if not needs_mass:
            return
        if self.dr_masses is None:
            raise SystemExit("DR_BLOCKED: mass API unavailable but stage wants mass deltas")
        masses = self.dr_masses[env_ids].clone()
        base = masses[:, 0].clone()
        for i, f in enumerate(frames):
            masses[i, 0] = base[i] + f["base_mass_delta_kg"]
        coms = None
        if self.dr_coms is not None:
            coms = self.dr_coms[env_ids].clone()
            for i, f in enumerate(frames):
                coms[i, 0, 0] += f["com_delta_x_m"]
        import warp as wp
        body_ids = [self.robot.body_names.index("base_link")]
        self.robot.set_masses_index(masses=wp.from_torch(masses[:, :1].contiguous()), body_ids=body_ids, env_ids=env_ids)
        if coms is not None:
            self.robot.set_coms_index(coms=wp.from_torch(coms[:, :1].contiguous(), dtype=wp.transformf), body_ids=body_ids, env_ids=env_ids)
        if getattr(self, "dr_check_readback", False):
            view = self.robot.root_physx_view
            idx_cpu = env_ids.cpu()
            got_m = torch.as_tensor(view.get_masses()).cpu()[idx_cpu, body_ids[0]]
            got_c = torch.as_tensor(view.get_coms()).cpu()[idx_cpu, body_ids[0], 0]
            m_err = float((got_m - masses[:, 0].cpu()).abs().max())
            c_err = float((got_c - coms[:, 0, 0].cpu()).abs().max())
            assert m_err < 1e-5 and c_err < 1e-5, (m_err, c_err)
            self.dr_log(f"DR_VERIFY PHYSX_READBACK mass_err={m_err} com_x_err={c_err}")
        self.dr_written["mass"].append(float(masses[0, 0]))
        self.dr_written["com"].append(float(coms[0, 0, 0]) if coms is not None else float("nan"))

    def _write_fixture(self, env_ids, fixtures):
        n = len(env_ids)
        idx = torch.as_tensor(env_ids, device=self.device)
        origin = self.terrain.env_origins[idx]
        quat = torch.tensor([f["quat_wxyz"] for f in fixtures], device=self.device, dtype=torch.float32)
        gyro = torch.tensor([f["gyro_body"] for f in fixtures], device=self.device, dtype=torch.float32)
        q = torch.tensor([f["q_dds"] for f in fixtures], device=self.device, dtype=torch.float32)
        dq = torch.tensor([f["dq_dds"] for f in fixtures], device=self.device, dtype=torch.float32)
        q = q * self.dr_sign
        dq = dq * self.dr_sign
        # wheel POSITIONS are unobservable in this contract (the policy zeroes them and
        # drives velocity only) and the field logs carry multi-turn accumulated angles
        # (e.g. -19.41 rad); writing those serves no purpose and invites solver trouble
        q[:, 12:] = 0.
        # per-frame root height from the precomputed ground-contact estimate
        z = torch.full((n,), self.dr_ground_uniform, device=self.device, dtype=torch.float32)
        root = torch.zeros(n, 7, device=self.device)
        root[:, :3] = origin
        root[:, 2] = origin[:, 2] + z
        # Isaac builds report/accept the root quaternion as (x,y,z,w); the field
        # logs carry (w,x,y,z).  Reorder exactly at this write boundary.  The
        # rotation matrix below still uses the w,x,y,z components it unpacked.
        root[:, 3:] = quat[:, [1, 2, 3, 0]]
        w, x, y, zq = quat.unbind(-1)
        rot = torch.stack((
            torch.stack((1 - 2 * (y * y + zq * zq), 2 * (x * y - w * zq), 2 * (x * zq + w * y)), -1),
            torch.stack((2 * (x * y + w * zq), 1 - 2 * (x * x + zq * zq), 2 * (y * zq - w * x)), -1),
            torch.stack((2 * (x * zq - w * y), 2 * (y * zq + w * x), 1 - 2 * (x * x + y * y)), -1),
        ), 1)
        vel = torch.zeros(n, 6, device=self.device)
        vel[:, 3:] = torch.bmm(rot, gyro[..., None])[..., 0]
        joint_pos = tensor(self.robot.data.default_joint_pos)[idx].clone()
        joint_vel = torch.zeros_like(joint_pos)
        joint_pos[:, self.ids] = q
        joint_vel[:, self.ids] = dq
        self.robot.write_root_pose_to_sim(root, idx)
        self.robot.write_root_velocity_to_sim(vel, idx)
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=idx)

    # 鈹€鈹€ integration point 2: zero-velocity handoff 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    def _pre_physics_step(self, latent):
        self.latent.copy_(latent)
        interval = round(2.5 / self.step_dt)
        ids = (self.episode_length_buf.remainder(interval) == 0).nonzero(as_tuple=True)[0]
        ArcEnv._sample_commands(self, ids)
        if self.exercise is not None:
            self.command[:] = self.exercise
        active = self.dr_handoff_step < self.handoff_steps
        if active.any():
            self.command[active] = 0.            # zero velocity for the whole handoff
            self.dr_cmd_seen = max(self.dr_cmd_seen, float(self.command[active].abs().max()))
        bounded = latent.tanh()
        side = self.command.new_tensor([.1654, -.1654, .1654, -.1654])
        wheel = (self.command[:, :1] - self.command[:, 2:3] * side) / RADIUS
        wheel = (wheel + 2. * bounded[:, 12:]).clamp(-12., 12.)
        self.physical.copy_(torch.cat((self.dr_leg_gain * bounded[:, :12], wheel / 12.), 1))
        sample = self._sample_commands
        self._sample_commands = lambda ids: None
        try:
            QualityEnv._pre_physics_step(self, self.physical)
        finally:
            self._sample_commands = sample
        if active.any():
            w = ((self.dr_handoff_step[active] + 1).float() / self.handoff_steps)[:, None]
            self.applied[active, :12] = (1. - w) * self.dr_stand_leg[active] + w * self.applied[active, :12]
            self.applied[active, 12:] = (1. - w) * self.dr_stand_wheel[active] + w * self.applied[active, 12:]
            self.dr_handoff_step[active] += 1

    # 鈹€鈹€ diagnostics 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    def dr_report(self):
        return {
            "stage": self.dr_stage,
            "fixture_episodes": self.dr_verify["fixture_episodes"],
            "nominal_episodes": self.dr_verify["nominal_episodes"],
            "handoff_active": int((self.dr_handoff_step < self.handoff_steps).sum()),
            "handoff_cmd_max_seen": self.dr_cmd_seen,
            "mass_written": self.dr_written["mass"][-4:],
            "com_written": self.dr_written["com"][-4:],
        }







