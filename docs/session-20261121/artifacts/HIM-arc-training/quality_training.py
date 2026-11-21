import hashlib
import json
from pathlib import Path
import sys

import torch

from best_checkpoint import BestCheckpoint
from environment import ROOT


class HIMEnvironmentAdapter:
    """Adapt Isaac Lab transitions to the upstream HIM runner."""

    num_actions = 16

    def __init__(self, env):
        self.env = env
        self.num_envs = env.num_envs
        self.num_obs = env.cfg.observation_space
        self.num_one_step_obs = env.frame_width
        self.num_privileged_obs = env.cfg.state_space
        self.max_episode_length = env.max_episode_length

    def reset(self):
        self.obs, _ = self.env.reset()
        return self.get_observations(), self.get_privileged_observations()

    def get_observations(self):
        return self.obs["policy"]

    def get_privileged_observations(self):
        return self.obs["critic"]

    def step(self, actions):
        self.obs, reward, terminated, timeout, extras = self.env.step(actions)
        dones = terminated | timeout
        ids = dones.nonzero(as_tuple=True)[0]
        extras = {**extras, "time_outs": timeout & ~terminated}
        return (self.get_observations(), self.get_privileged_observations(),
                reward, dones, extras, ids, self.env.terminal_critic[ids])


def train(env, args, out):
    env.exercise = None
    out = out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    asset = Path(env.cfg.scene.robot.spawn.asset_path).resolve()
    (out / "run.json").write_text(json.dumps({
        "asset": str(asset), "asset_sha256": hashlib.sha256(asset.read_bytes()).hexdigest(),
        "num_envs": env.num_envs, "rollout_steps": args.rollout_steps or (100 if env.cfg.him else 24),
        "policy_dt_s": env.step_dt, "seed": args.seed, "experiment": "him-arc-v1", "stage":args.stage, "reference_q": env.reference_q.tolist(), "reference_height":env.reference_height, "frame_width": env.frame_width,
        "resume": str(args.resume) if args.resume else None,
        "swing_load_weight": env.cfg.swing_load_weight,
        **({"mirror_loss_weight": args.mirror_loss_weight} if args.mirror_loss_weight else {}),
        **({"action_overrun_weight": args.action_overrun_weight} if args.action_overrun_weight else {}),
        **({"lateral_progress_weight": args.lateral_progress_weight} if args.lateral_progress_weight else {}),
        **({"swing_placement_weight": args.swing_placement_weight} if args.swing_placement_weight else {}),
        **({"tire_clearance":True} if args.tire_clearance else {}),
        **({"lateral_curriculum_updates": args.lateral_curriculum_updates} if args.lateral_curriculum_updates else {}),
        **({"rolling_stability_weight": args.rolling_stability_weight} if args.rolling_stability_weight else {}),
        **({"leg_action_center_rad": args.leg_action_center_rad} if args.leg_action_center_rad is not None else {}),
    }, indent=2) + "\n")
    steps = args.rollout_steps or (100 if env.cfg.him else 24)
    print(f"AS2W_CONFIG envs={env.num_envs} rollout_steps={steps} iterations={args.iterations}", flush=True)
    if env.cfg.him:
        # This interpreter is dedicated to the HIM task. Its legacy rsl_rl
        # contains custom PPO, rollout storage, and successor-state training.
        sys.path.insert(0, str(ROOT / "upstream/HIMLoco/rsl_rl"))
        from rsl_rl.runners import HIMOnPolicyRunner

        cfg = {
            "policy": {"init_noise_std": .3, "activation": "elu",
                       "actor_hidden_dims": [512, 256, 128],
                       "critic_hidden_dims": [512, 256, 128]},
            "algorithm": {"value_loss_coef": 1., "use_clipped_value_loss": True,
                          "clip_param": .2, "entropy_coef": 0., "num_learning_epochs": 5,
                          "num_mini_batches": 4, "learning_rate": .0003, "schedule": "adaptive",
                          "gamma": .99, "lam": .95, "desired_kl": .01, "max_grad_norm": 1.},
            "runner": {"policy_class_name": "HIMActorCritic", "algorithm_class_name": "HIMPPO",
                       "num_steps_per_env": steps, "save_interval": args.save_interval},
        }
        runner = HIMOnPolicyRunner(HIMEnvironmentAdapter(env), cfg, str(out), device=env.device)
    else:
        from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, create_rsl_rl_runner, handle_deprecated_rsl_rl_cfg, check_rsl_rl_version
        from isaaclab_tasks.core.velocity.config.go2.agents.rsl_rl_ppo_cfg import UnitreeGo2RoughPPORunnerCfg

        cfg = UnitreeGo2RoughPPORunnerCfg()
        cfg.device = env.device
        cfg.seed = args.seed
        cfg.num_steps_per_env = steps
        cfg.save_interval = args.save_interval
        cfg.obs_groups = {"actor": ["policy"], "critic": ["critic"]}
        if args.mirror_loss_weight:
            from isaaclab_rl.rsl_rl.symmetry_cfg import RslRlSymmetryCfg
            from mirror_policy import augment_left_right

            cfg.algorithm.symmetry_cfg = RslRlSymmetryCfg(
                data_augmentation_func=augment_left_right, use_mirror_loss=True,
                mirror_loss_coeff=args.mirror_loss_weight,
            )
        cfg = handle_deprecated_rsl_rl_cfg(cfg, check_rsl_rl_version())
        # QualityEnv clips the physical command. Pass through the sampled action
        # when the loss needs the excess that would be discarded by the wrapper.
        clip_actions = None if env.cfg.action_overrun_weight else 1.
        runner = create_rsl_rl_runner(RslRlVecEnvWrapper(env, clip_actions=clip_actions), cfg, log_dir=str(out))
    with torch.no_grad():
        runner.alg.actor_critic.actor[-1].weight.zero_()
        runner.alg.actor_critic.actor[-1].bias.zero_()
    runner.alg.actor_critic.std.requires_grad_(False)
    if args.resume:
        runner.load(str(args.resume.resolve()))
        if args.resume_update is not None:
            runner.current_learning_iteration = args.resume_update
        print(f"AS2W_RESUME {args.resume} runner_iteration={runner.current_learning_iteration}", flush=True)
    BestCheckpoint(env, runner, out, interval=3 if env.cfg.him else 10, previous=args.previous_best)
    record_update = runner.alg.update
    iteration = runner.current_learning_iteration
    start_iteration = iteration
    def update_and_record(*args, **kwargs):
        nonlocal iteration
        result = record_update(*args, **kwargs)
        iteration += 1
        env.progress=min(1.,iteration/2000.)
        if env.cfg.lateral_curriculum_updates:
            env.lateral_curriculum_progress=min(1.,(iteration-start_iteration)/env.cfg.lateral_curriculum_updates)
        if iteration == 1 or iteration % 10 == 0:
            metrics = {k: float(v) for k,v in env.metrics.items()}
            if env.cfg.lateral_curriculum_updates:
                metrics["lateral_curriculum_max_vy_m_s"]=.45*(.4+.6*env.lateral_curriculum_progress)
            with (out / "motion_train.jsonl").open("a") as stream:
                stream.write(json.dumps({"update": iteration, **metrics}) + "\n")
        return result
    runner.alg.update = update_and_record
    runner.learn(num_learning_iterations=args.iterations, init_at_random_ep_len=False)
    print("AS2W_TRAIN_SMOKE_OK", flush=True)
