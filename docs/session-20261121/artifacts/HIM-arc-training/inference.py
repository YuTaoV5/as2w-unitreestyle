from pathlib import Path
import sys

import torch

ROOT = Path(__file__).resolve().parents[1]

def load_policy(raw, weight):
    cfg = raw.cfg
    root = ROOT
    weight=weight
    if cfg.him:
        sys.path.insert(0,str(root/"upstream/HIMLoco/rsl_rl"))
        from rsl_rl.modules import HIMActorCritic
        model=HIMActorCritic(raw.cfg.observation_space,raw.cfg.state_space,raw.frame_width,16).to(raw.device)
        model.load_state_dict(torch.load(weight,map_location=raw.device,weights_only=True)["model_state_dict"])
        model.eval()
        def policy(obs):
            return model.act_inference(obs["policy"])
    else:
        from tensordict import TensorDict
        from isaaclab_rl.rsl_rl import (RslRlVecEnvWrapper,create_rsl_rl_runner,
                                       check_rsl_rl_version,handle_deprecated_rsl_rl_cfg)
        from isaaclab_tasks.core.velocity.config.go2.agents.rsl_rl_ppo_cfg import UnitreeGo2RoughPPORunnerCfg
        config=UnitreeGo2RoughPPORunnerCfg()
        config.device=raw.device
        config.num_steps_per_env=1
        config.obs_groups={"actor":["policy"],"critic":["critic"]}
        config=handle_deprecated_rsl_rl_cfg(config,check_rsl_rl_version())
        runner=create_rsl_rl_runner(RslRlVecEnvWrapper(raw,clip_actions=1.),config,log_dir=None)
        runner.load(str(weight))
        model=runner.get_inference_policy()
        def policy(obs):
            return model(TensorDict(obs,batch_size=[raw.num_envs],device=raw.device))
    return policy
