"""Phase-conditioned AS2W flat and rough-terrain training."""
import math
from pathlib import Path
import sys

import torch
from isaaclab.managers import EventTermCfg, SceneEntityCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab.envs import mdp

from wheel_clearance import tire_bottom_z, wheel_support_table

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "common"))
from environment import AS2WCfg, AS2WEnv, AS2WSceneCfg, DomainEvents, FRAME, HISTORY, RADIUS, WHEELS, tensor

QUALITY_FRAME = FRAME + 2

# load-v0 nominal wheel centers in the body plane, in FL/FR/RL/RR order.
NEUTRAL_WHEEL_XY = ((.2119, .1654), (.2119, -.1654),
                    (-.2119, .1654), (-.2119, -.1654))


def gait_frequency(command):
    speed = command[:, 1].abs() + .25 * command[:, 2].abs()
    return (1.25 + .95 * speed).clamp(1.25, 1.75)


def quintic(phase):
    return phase ** 3 * (10. + phase * (-15. + 6. * phase))


def swing_wheel_xy(command, swing_phase, neutral):
    """Return the horizontal wheel-center target in the body-heading frame."""
    vy, wz = command[:, 1:2], command[:, 2:3]
    stride = (.62 / gait_frequency(command)[:, None, None]) * torch.stack(
        (-wz * neutral[:, 1], vy + wz * neutral[:, 0]), dim=-1)
    stride = stride * (.09 / stride.norm(dim=-1, keepdim=True).clamp_min(.09))
    blend = quintic(swing_phase.clamp(0., 1.)) - .5
    return neutral + stride * blend[..., None]


def wheel_xy_in_heading(position, root_position, root_quaternion):
    x, y, z, w = root_quaternion.unbind(-1)
    yaw = torch.atan2(2. * (w * z + x * y), 1. - 2. * (y.square() + z.square()))
    dx, dy = (position[..., :2] - root_position[:, None, :2]).unbind(-1)
    c, s = yaw.cos()[:, None], yaw.sin()[:, None]
    return torch.stack((c * dx + s * dy, -s * dx + c * dy), dim=-1)


def rolling_stability_cost(command, gravity, loads):
    """Track body tilt and minimum wheel load in standing and straight rolling."""
    step_command = command[:, 1].abs() + .28 * command[:, 2].abs()
    rolling = (1. - (step_command / .04).clamp(0., 1.))
    tilt_excess = ((gravity[:, :2].norm(dim=1) - math.sin(math.radians(5.))) / .20).clamp(0., 2.)
    load_deficit = ((12. - loads.amin(dim=1)) / 12.).clamp(0., 1.)
    return rolling * (tilt_excess.square() + load_deficit.square())


def lateral_progress(vy, command_vy):
    """Reward reducing lateral error relative to holding still."""
    magnitude = command_vy.abs()
    active = ((magnitude - .04) / .12).clamp(0., 1.)
    improvement = (magnitude - (vy - command_vy).abs()) / magnitude.clamp_min(.10)
    return active * improvement.clamp(-1., 1.)

@configclass
class QualityEvents(DomainEvents):
    push = EventTermCfg(
        func=mdp.push_by_setting_velocity, mode="interval",
        interval_range_s=(8., 14.), params={"velocity_range": {"x": (-.16, .16), "y": (-.12, .12)}},
    )
    payload = EventTermCfg(
        func=mdp.randomize_rigid_body_mass, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names="base_link"),
                "mass_distribution_params": (.90, 1.16), "operation": "scale",
                "distribution": "uniform", "recompute_inertia": True},
    )

    def __post_init__(self):
        self.mass.params["asset_cfg"] = SceneEntityCfg("robot", body_names="^(?!base_link$).*")
        self.mass.params["mass_distribution_params"] = (.92, 1.08)
        self.material.params.update(static_friction_range=(.65, 1.2), dynamic_friction_range=(.42, .65))
        self.com.params["com_range"] = {"x": (-.015, .015), "y": (-.01, .01), "z": (-.008, .008)}
        self.drive_gains.params["stiffness_distribution_params"] = (.88, 1.12)
        self.drive_gains.params["damping_distribution_params"] = (.85, 1.15)

@configclass
class QualityScene(AS2WSceneCfg):
    contacts: ContactSensorCfg | None = None

@configclass
class QualityCfg(AS2WCfg):
    scene = QualityScene(num_envs=64, env_spacing=2.5, replicate_physics=True)
    swing_load_weight: float = 0.
    action_overrun_weight: float = 0.
    lateral_progress_weight: float = 0.
    swing_placement_weight: float = 0.
    rolling_stability_weight: float = 0.
    leg_action_center_rad: tuple[float, ...] | None = None
    tire_clearance: bool = False
    lateral_curriculum_updates: int = 0

    def __post_init__(self):
        super().__post_init__()
        self.observation_space = QUALITY_FRAME * (HISTORY if self.him else 1)
        self.state_space = QUALITY_FRAME + 3
        self.events = QualityEvents() if self.randomize else None
        self.scene.robot.spawn.activate_contact_sensors = True
        self.scene.contacts = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/Geometry/.*")

class QualityEnv(AS2WEnv):
    frame_width = QUALITY_FRAME

    def __init__(self, cfg, **kwargs):
        super().__init__(cfg, **kwargs)
        n,dev=self.num_envs,self.device
        self.history=torch.zeros(n,HISTORY,QUALITY_FRAME,device=dev)
        self.terminal_critic=torch.zeros(n,QUALITY_FRAME+3,device=dev)
        self.phase=torch.zeros(n,device=dev)
        self.phase_offset=torch.tensor([0.,.5,.5,0.],device=dev)
        self.neutral_wheel_xy=torch.tensor(NEUTRAL_WHEEL_XY,device=dev)
        self.sensor=self.scene["contacts"]
        self.wheel_bodies=[self.robot.body_names.index(name) for name in WHEELS]
        self.foot_sensors=[self.sensor.body_names.index(name) for name in WHEELS]
        self.nonwheel_sensors=[i for i in range(len(self.sensor.body_names)) if i not in self.foot_sensors]
        self.latency=torch.zeros(n,dtype=torch.long,device=dev)
        self.rows=torch.arange(n,device=dev)
        self.target_queue=torch.zeros(3,n,16,device=dev)
        self.applied=torch.zeros_like(self.target)
        self.leg_action_center=tensor(self.robot.data.default_joint_pos)[:,self.leg_ids].clone()
        if cfg.leg_action_center_rad is not None:
            center=self.leg_action_center.new_tensor(cfg.leg_action_center_rad)
            limits=tensor(self.robot.data.soft_joint_pos_limits)[:,self.leg_ids]
            if center.shape!=(12,) or not torch.isfinite(center).all():
                raise ValueError("Expected 12 finite leg action centers")
            if ((center-.25<limits[...,0]) | (center+.25>limits[...,1])).any():
                raise ValueError("Action center and range exceed leg position limits")
            self.leg_action_center[:]=center
        self.raw_previous=torch.zeros(n,12,device=dev)
        self.raw_delta=torch.zeros_like(self.raw_previous)
        self.raw_acc=torch.zeros_like(self.raw_previous)
        self.action_overrun=torch.zeros(n,device=dev)
        self.supported=torch.zeros(n,4,dtype=torch.bool,device=dev)
        self.foot_reference=torch.zeros(n,4,device=dev)
        self.gyro_bias=torch.zeros(n,3,device=dev)
        self.joint_bias=torch.zeros(n,12,device=dev)
        self.noise=torch.zeros(n,QUALITY_FRAME,device=dev)
        self.metrics={}
        self.lateral_curriculum_progress=0.
        self.tire_support=None
        if cfg.tire_clearance:
            self.tire_support=torch.as_tensor(
                wheel_support_table(cfg.scene.robot.spawn.asset_path,WHEELS),
                device=self.device,
            )

    def _sample_commands(self, ids):
        super()._sample_commands(ids)
        if not self.cfg.lateral_curriculum_updates or self.lateral_curriculum_progress>=1.:
            return
        command=self.command[ids]
        pure_lateral=(command[:,0]==0.) & (command[:,2]==0.) & (command[:,1]!=0.)
        low_speed=pure_lateral & (torch.rand(len(ids),device=self.device)<.75)
        command[low_speed,1]*=.4+.6*self.lateral_curriculum_progress
        self.command[ids]=command

    def _step_strength(self):
        return ((self.command[:,1].abs()+.28*self.command[:,2].abs()-.04)/.16).clamp(0.,1.)

    def _pre_physics_step(self, actions):
        super()._pre_physics_step(actions)
        self.phase.add_(self.step_dt*gait_frequency(self.command)).remainder_(1.)
        self.target_queue[2].copy_(self.target_queue[1])
        self.target_queue[1].copy_(self.target_queue[0])
        self.target_queue[0].copy_(self.target)
        self.applied.copy_(self.target_queue[self.latency,self.rows])
        requested=actions[:,:12]
        target=requested.clamp(-1.,1.)*.25
        if self.cfg.action_overrun_weight:
            excess=(requested.abs()-1.).clamp(0.,2.)
            self.action_overrun.copy_(excess.square().mean(1))
        delta=target-self.raw_previous
        self.raw_acc.copy_(delta-self.raw_delta)
        self.raw_previous.copy_(target)
        self.raw_delta.copy_(delta)
        if self.cfg.randomize:
            self.noise[:,3:6].normal_(std=.018)
            self.noise[:,9:21].normal_(std=.004)
            self.noise[:,25:41].normal_(std=.006)

    def _apply_action(self):
        pos=self.leg_action_center+self.applied[:,:12]
        self.robot.set_joint_position_target(pos,joint_ids=self.leg_ids)
        self.robot.set_joint_velocity_target(self.applied[:,12:],joint_ids=self.wheel_ids)

    def _frame(self):
        frame=super()._frame()
        frame[:,41:53]=self.applied[:,:12]/.25
        frame[:,53:57]=self.applied[:,12:]/12.
        frame[:,3:6]+=self.gyro_bias
        frame[:,9:21]+=self.joint_bias
        phi=self.phase*math.tau
        strength=self._step_strength()
        return torch.cat((frame,(phi.sin()*strength)[:,None],(phi.cos()*strength)[:,None]),dim=1)+self.noise

    def _swing_reference(self):
        swing_phase=((self.phase[:,None]+self.phase_offset)%1.-.62)/.38
        shape=(1.-(2.*swing_phase.clamp(0.,1.)-1.).abs()).clamp(0.,1.)
        height=.030 if self.cfg.swing_placement_weight else .035
        lift=height*(10*shape**3-15*shape**4+6*shape**5)
        return swing_phase,lift

    def _get_rewards(self):
        reward=super()._get_rewards()
        gravity=tensor(self.robot.data.projected_gravity_b)
        gyro=tensor(self.robot.data.root_ang_vel_b)
        normals=tensor(self.sensor.data.net_normal_forces_w)
        loads=normals[:,self.foot_sensors,2].clamp_min(0.)
        support=loads>=5.
        position=tensor(self.robot.data.body_link_pos_w)[:,self.wheel_bodies]
        lin=tensor(self.robot.data.body_link_lin_vel_w)[:,self.wheel_bodies]
        ang=tensor(self.robot.data.body_link_ang_vel_w)[:,self.wheel_bodies]
        x,y,z,w=tensor(self.robot.data.body_link_quat_w)[:,self.wheel_bodies].unbind(-1)
        axle=torch.stack((2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w)),-1)
        down=torch.zeros_like(axle)
        down[...,2]=-1.
        radial=down-(down*axle).sum(-1,keepdim=True)*axle
        radial=radial/radial.norm(dim=-1,keepdim=True).clamp_min(.05)
        contact_velocity=lin+torch.cross(ang,RADIUS*radial,dim=-1)
        sideways=(contact_velocity[...,:2]*axle[...,:2]).sum(-1)
        drop=(contact_velocity[...,2].neg()-.12).clamp(0.,.8)
        just_landed=support & ~self.supported & (self.episode_length_buf[:,None]>2)
        self.supported.copy_(support)
        contact_height=position[...,2]
        if self.tire_support is not None:
            contact_height=tire_bottom_z(contact_height,axle[...,2],self.tire_support)
        self.foot_reference.copy_(torch.where(support,contact_height,self.foot_reference))
        swing_phase,lift=self._swing_reference()
        clearance=contact_height-self.foot_reference
        gait=self._step_strength()
        if self.cfg.rough:
            plane=(self.terrain.terrain_types < math.ceil(self.cfg.scene.terrain.terrain_generator.num_cols*.375))
            phase_weight=torch.where(plane,1.,.25)
        else:
            phase_weight=1.
        swing=(swing_phase>0)&(lift>.009)
        stance=(swing_phase<-.08)
        phase_loss=((swing*support+stance*~support).float().mean(1)
                    +1.5*(swing*((clearance-lift)/.05).square().clamp(max=3.)).mean(1))
        slip=(sideways.square().clamp(max=1.)*support).mean(1)
        impact=(just_landed*drop.square()).mean(1)
        nonwheel=(normals[:,self.nonwheel_sensors].norm(dim=-1)>8.).any(1)
        rolling=(1.-gait)*((support.sum(1)<4).float())
        delta=self.raw_delta.square().mean(1)
        acceleration=self.raw_acc.square().mean(1)
        cost=(.48*phase_weight*gait*phase_loss + .9*slip + .35*impact
              +.35*gyro[:,:2].square().sum(1)+.5*gravity[:,:2].square().sum(1)
              +.22*rolling+.6*nonwheel.float()+.45*delta+.8*acceleration)
        if self.cfg.swing_load_weight:
            # Emphasize the middle of swing and give credit for partial unloading.
            swing_weight = (lift / .035) * swing
            load_fraction = (loads / 35.).clamp(max=1.)
            clearance_shortfall = ((lift - clearance) / .035).clamp(0., 1.)
            swing_cost = (swing_weight * (load_fraction + clearance_shortfall.square())).mean(1)
            cost += self.cfg.swing_load_weight * phase_weight * gait * swing_cost
        self.metrics={"slip":slip.mean(),"body_xy_rate":gyro[:,:2].norm(dim=1).mean(),
                      "raw_target_curvature":acceleration.mean(),"phase":phase_loss.mean(),
                      "nonwheel":nonwheel.float().mean(),"four_support":(support.sum(1)==4).float().mean()}
        if self.cfg.action_overrun_weight:
            cost += self.cfg.action_overrun_weight*self.action_overrun
            self.metrics["action_overrun_cost"]=self.action_overrun.mean()
        if self.cfg.lateral_progress_weight:
            vy = tensor(self.robot.data.root_lin_vel_b)[:, 1]
            progress = lateral_progress(vy, self.command[:, 1])
            reward = reward + self.step_dt*self.cfg.lateral_progress_weight*progress
            self.metrics["lateral_progress"]=progress.mean()
        if self.cfg.swing_placement_weight:
            data = self.robot.data
            actual_xy = wheel_xy_in_heading(position,tensor(data.root_pos_w),tensor(data.root_quat_w))
            target_xy = swing_wheel_xy(self.command,swing_phase,self.neutral_wheel_xy)
            distance_squared = (actual_xy-target_xy).square().sum(-1)
            placement_cost = (swing * (distance_squared / .045**2).clamp(max=4.)).mean(1)
            cost += self.cfg.swing_placement_weight * phase_weight * gait * placement_cost
            self.metrics["swing_placement_cost"]=placement_cost.mean()
        if self.cfg.rolling_stability_weight:
            stability=rolling_stability_cost(self.command,gravity,loads)
            cost += self.cfg.rolling_stability_weight*phase_weight*stability
            self.metrics["rolling_stability_cost"]=stability.mean()
        return reward-self.step_dt*cost

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids=self.rows
        super()._reset_idx(env_ids)
        n=len(env_ids)
        self.phase[env_ids]=torch.rand(n,device=self.device)
        self.target_queue[:,env_ids]=self.target[env_ids]
        self.applied[env_ids]=self.target[env_ids]
        self.raw_previous[env_ids]=self.target[env_ids,:12]
        self.raw_delta[env_ids]=0.
        self.raw_acc[env_ids]=0.
        self.noise[env_ids]=0.
        self.supported[env_ids]=False
        offset=RADIUS if self.tire_support is None else 0.
        self.foot_reference[env_ids]=self.terrain.env_origins[env_ids,2,None]+offset
        if self.cfg.randomize:
            self.latency[env_ids]=torch.randint(0,3,(n,),device=self.device)
            self.gyro_bias[env_ids]=(torch.rand(n,3,device=self.device)*2.-1.)*.018
            self.joint_bias[env_ids]=(torch.rand(n,12,device=self.device)*2.-1.)*.009
