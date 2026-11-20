"""Compare complete-swing and tire-slip rewards on the frozen 1220 policy."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--root",type=Path,required=True)
parser.add_argument("--out",type=Path,required=True)
parser.add_argument("--iterations",type=int,required=True)
parser.add_argument("--offset",type=int,default=1220)
parser.add_argument("--resume",type=Path)
parser.add_argument("--envs",type=int,default=4096)
parser.add_argument("--arm",choices=("A","B","C"),required=True)
parser.add_argument("--calibration",type=Path,required=True)
args=parser.parse_args()
args.swing_weight=.08 if args.arm in ("B","C") else 0.
args.gap_weight=.18 if args.arm in ("B","C") else 0.
args.slip_weight=.20 if args.arm=="C" else 0.
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"common"))
app=AppLauncher(headless=True).app
env=None
try:
    import torch
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab.utils import configclass
    from environment import AS2WCfg, AS2WEnv, AS2WSceneCfg, RADIUS, WHEELS, tensor
    from learner import train
    from swing_reward import SwingReward
    from policy import read_actor, compose, gate

    @configclass
    class ContactScene(AS2WSceneCfg):
        contacts: ContactSensorCfg | None = None

    @configclass
    class ContinueCfg(AS2WCfg):
        scene=ContactScene(num_envs=args.envs,env_spacing=2.5,replicate_physics=True)

        def configure_terrain(self):
            super().configure_terrain()
            generator=self.scene.terrain.terrain_generator
            generator.sub_terrains={k:generator.sub_terrains[k] for k in ("plane","slope")}
            generator.sub_terrains["plane"].proportion=.60
            generator.sub_terrains["slope"].proportion=.40

    class ContinueEnv(AS2WEnv):
        def __init__(self,cfg):
            super().__init__(cfg)
            self.base_ckpt=torch.load(args.root/"source/best.pt",map_location="cpu",weights_only=False)
            self.base_actor=read_actor(self.base_ckpt["actor_state_dict"],self.device)
            self.sensor=self.scene["contacts"]
            self.sensor_ids=[self.sensor.body_names.index(w) for w in WHEELS]
            self.body_ids=[self.robot.body_names.index(w) for w in WHEELS]
            self.swing=SwingReward(self.num_envs,self.device,json.loads(args.calibration.read_text()))
            self.switched=torch.zeros(self.num_envs,device=self.device,dtype=torch.bool)
            self.metrics=torch.zeros(16,7,device=self.device)
            self.metrics_names=("count","base_reward_per_sec","credited_events_per_sec",
                                "overdue_fraction","loaded_slip_cost","valid_events_per_sec",
                                "applied_residual_mean_rad")

        def _pre_physics_step(self,actions):
            frame=self._frame()
            frozen=self.base_actor(frame)
            old_gate=gate(frame[:,:3])
            interval = round(2.5 / self.step_dt)
            env_ids = (self.episode_length_buf.remainder(interval) == 0).nonzero(as_tuple=True)[0]
            self.switched.zero_()
            self.switched[env_ids]=True
            self._sample_commands(env_ids)
            if self.exercise is not None:
                self.command[:] = self.exercise
            actions = compose(frozen, actions, torch.minimum(old_gate,gate(self.command)))
            self.applied_delta=(actions-frozen.clamp(-1.,1.)).abs()*.25
            wheel = actions[:, 12:] * 12. if self.cfg.him else self.command[:, :1].expand(-1, 4) / RADIUS
            self.target[:, :12].lerp_(actions[:, :12] * .25, self.alpha_leg)
            self.target[:, 12:].lerp_(wheel.clamp(-12., 12.), self.alpha_wheel)
            if not self.cfg.him:
                zero = (wheel == 0) & (self.target[:, 12:].abs() < .02)
                self.target[:, 12:].masked_fill_(zero, 0.)
    

        def _reset_idx(self,env_ids):
            super()._reset_idx(env_ids)
            if hasattr(self,"swing"):
                self.swing.reset(slice(None) if env_ids is None else env_ids)

        def _get_rewards(self):
            legacy=super()._get_rewards()
            vx,vy,wz=self.command.unbind(1)
            nx,ny,nw=(self.command.abs()>1e-6).unbind(1)
            side=~nx&~nw&(vy.abs()>=.15)
            turn=~nx&~ny&(wz.abs()>=.35)
            num_cols=self.cfg.scene.terrain.terrain_generator.num_cols
            plane=self.terrain.terrain_types/num_cols+.001 < .60
            case=torch.where(side,(vy<0).long(),torch.where(turn,2+(wz<0).long(),-1))
            case=torch.where(plane,case,-1)
            d=self.robot.data
            ids=self.body_ids
            events,overdue,slip,valid=self.swing(
                case,self.switched,tensor(self.sensor.data.net_normal_forces_w)[:,self.sensor_ids,2],
                tensor(d.body_link_pos_w)[:,ids],tensor(d.body_link_quat_w)[:,ids],
                tensor(d.body_link_lin_vel_w)[:,ids],tensor(d.body_link_ang_vel_w)[:,ids],
                self.terrain.env_origins[:,2],self.step_dt)
            group=torch.zeros_like(case)
            group=torch.where(nx,1+(vx<0).long(),group)
            group=torch.where(ny,3+(vy<0).long(),group)
            group=torch.where(nw,5+(wz<0).long(),group)
            group=torch.where(nx.long()+ny.long()+nw.long()>1,7,group)
            group=group+8*(~plane).long()
            values=torch.stack((torch.ones_like(events),legacy/self.step_dt,4*events/self.step_dt,
                                overdue,slip,valid/self.step_dt,self.applied_delta.mean(1)),1)
            self.metrics.index_add_(0,group,values)
            return legacy+args.swing_weight*events-self.step_dt*(args.gap_weight*overdue+args.slip_weight*slip)

    input_manifest=json.loads((args.root/"source/run.json").read_text())
    asset=Path(input_manifest["asset"])
    assert hashlib.sha256(asset.read_bytes()).hexdigest()==input_manifest["asset_sha256"]
    cfg=ContinueCfg(rough=True,randomize=True,seed=42)
    cfg.scene.robot.spawn.asset_path=str(asset)
    cfg.scene.robot.spawn.usd_dir=str(args.out.parent/"usd"/args.out.name)
    cfg.scene.robot.spawn.activate_contact_sensors=True
    cfg.scene.contacts=ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/Geometry/.*")
    cfg.configure_terrain()
    env=ContinueEnv(cfg)
    assert cfg.observation_space==58 and abs(env.step_dt-.020)<1e-8
    train(env,args,args.out)
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    if env is not None:
        env.close()
    app.close()
