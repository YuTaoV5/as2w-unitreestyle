import argparse
import hashlib
import sys
from pathlib import Path
import traceback

from isaaclab.app import AppLauncher

from training_options import add_training_options, options_from_namespace, validate_options

parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=("unitree_style","himloco"), required=True)
parser.add_argument("--num-envs", type=int, required=True)
parser.add_argument("--rollout-steps", type=int, required=True)
parser.add_argument("--iterations", type=int, required=True)
parser.add_argument("--save-interval", type=int, default=100)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--asset", type=Path, required=True)
parser.add_argument("--out", type=Path, required=True)
parser.add_argument("--resume", type=Path)
parser.add_argument("--resume-update", type=int)
parser.add_argument("--previous-best", type=Path)
add_training_options(parser)
parser.add_argument("--stage",choices=("flat","randomized","rough"),default="flat")
parser.add_argument("--smoke", action="store_true")
args = parser.parse_args()
try:
    validate_options(options_from_namespace(args),args.mode)
except ValueError as error:
    parser.error(str(error))
app=AppLauncher(headless=True).app
env=None
code=0
try:
    import torch
    from stair_environment import QualityCfg, StairEnv as QualityEnv
    from quality_training import train
    cfg=QualityCfg(him=args.mode=="himloco", rough=False, randomize=False, seed=args.seed,
                   swing_load_weight=args.swing_load_weight, action_overrun_weight=args.action_overrun_weight,
                   lateral_progress_weight=args.lateral_progress_weight,
                   swing_placement_weight=args.swing_placement_weight, tire_clearance=args.tire_clearance,
                   rolling_stability_weight=args.rolling_stability_weight,
                   lateral_curriculum_updates=args.lateral_curriculum_updates,
                   leg_action_center_rad=tuple(args.leg_action_center_rad) if args.leg_action_center_rad is not None else None)
    cfg.scene.num_envs=args.num_envs
    cfg.scene.env_spacing=6.
    cfg.scene.robot.spawn.asset_path=str(args.asset.resolve(strict=True))
    digest=hashlib.sha256(args.asset.read_bytes()).hexdigest()
    cfg.scene.robot.spawn.usd_dir=str(Path(__file__).parent/"usd"/digest/args.mode)
    cfg.configure_terrain()
    env=QualityEnv(cfg)
    import json
    reference=json.loads((Path(__file__).parent/"static_reference.json").read_text())
    env.reference_q=torch.tensor(reference["reference_q"],device=env.device)
    env.reference_height=reference["reference_height"]
    if args.smoke:
        obs,_=env.reset()
        a=torch.zeros(env.num_envs,cfg.action_space,device=env.device)
        for command in [(0.,.3,0.),(.6,0.,0.),(0.,0.,.6)]:
            env.exercise=torch.tensor(command,device=env.device)
            for _ in range(60):
                obs,reward,terminated,timeout,_=env.step(a)
                assert torch.isfinite(reward).all()
                assert all(torch.isfinite(v).all() for v in obs.values())
        assert obs["policy"].shape[-1]==cfg.observation_space
        if args.action_overrun_weight:
            obs,reward,terminated,timeout,_=env.step(torch.full_like(a,2.))
            assert torch.allclose(env.action_overrun,torch.ones_like(env.action_overrun))
            assert env.raw_previous.abs().max()<=.25
            assert torch.isfinite(reward).all()
            print("AS2W_ACTION_OVERRUN_SMOKE_OK",flush=True)
        env.episode_length_buf.fill_(env.max_episode_length-1)
        obs,reward,terminated,timeout,_=env.step(a)
        assert timeout.all()
        if cfg.him:
            histories=obs["policy"].reshape(env.num_envs,6,env.frame_width)
            assert torch.allclose(histories,histories[:,:1].expand_as(histories))
        print("AS2W_QUALITY_SMOKE_OK",flush=True)
    train(env,args,args.out)
except Exception:
    traceback.print_exc()
    code=1
finally:
    if env is not None:
        env.close()
    app.close()
sys.exit(code)
