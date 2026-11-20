"""Entry point for the HIMLoco flat-ground domain-randomisation runs.

Three modes:
  --verify           build 8 envs, force the fixture path, and print the checks
                     the handoff README demands (bidirectional mapping, wheel
                     splay sign, root height, zero-command handoff, NaN scan).
  --smoke            one short training pass to prove the loop runs and saves.
  --stage <name>     the real staged run (field_reset / load / contact_timing).

Baseline env/contract is untouched: it reuses him_arc's QualityCfg + ArcEnv and
imports the DR pieces from the handed-over package. The original
`--stage randomized` events stay disabled (randomize=False) by design.
"""
import argparse
import hashlib
import json
import sys
import traceback
from pathlib import Path

from isaaclab.app import AppLauncher

from training_options import add_training_options, options_from_namespace, validate_options

DR_DIR = Path(__file__).resolve().parent / "dr-handoff"

parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=("himloco",), default="himloco")
parser.add_argument("--stage", choices=("flat",), default="flat")
parser.add_argument("--num-envs", type=int, default=4096)
parser.add_argument("--rollout-steps", type=int, default=48)
parser.add_argument("--iterations", type=int, default=0)
parser.add_argument("--save-interval", type=int, default=100)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--asset", type=Path, required=True)
parser.add_argument("--out", type=Path)
parser.add_argument("--resume", type=Path)
parser.add_argument("--resume-update", type=int)
parser.add_argument("--previous-best", type=Path)
parser.add_argument("--dr-dir", type=Path, default=DR_DIR)
parser.add_argument("--fixture-z", type=float, default=.362)
parser.add_argument("--handoff-steps", type=int, default=100)
parser.add_argument("--hip-sign", type=float, default=1.)
parser.add_argument("--ground-margin", type=float, default=.002)
parser.add_argument("--verify", action="store_true")
parser.add_argument("--smoke", action="store_true")
add_training_options(parser)
parser.add_argument("--dr-stage", required=True,
                help="field_reset | load | contact_timing (from stages.json)")
args = parser.parse_args()
try:
    validate_options(options_from_namespace(args), args.mode)
except ValueError as error:
    parser.error(str(error))
app = AppLauncher(headless=True).app
env = None
code = 0
try:
    import torch
    from quality_environment import QualityCfg
    from dr_sampler import Sampler
    from dr_environment import DREnv
    from environment import tensor

    log = lambda msg: print(msg, flush=True)
    sampler = Sampler(args.dr_dir, seed=args.seed)
    cfg = QualityCfg(him=True, rough=False, randomize=False, seed=args.seed,
                     swing_load_weight=args.swing_load_weight,
                     action_overrun_weight=args.action_overrun_weight,
                     lateral_progress_weight=args.lateral_progress_weight,
                     swing_placement_weight=args.swing_placement_weight,
                     tire_clearance=args.tire_clearance,
                     rolling_stability_weight=args.rolling_stability_weight,
                     lateral_curriculum_updates=args.lateral_curriculum_updates)
    cfg.scene.num_envs = args.num_envs
    digest = hashlib.sha256(args.asset.read_bytes()).hexdigest()
    cfg.configure_terrain()
    from asset_contract import configure_asset
    configure_asset(cfg, args.asset, Path(__file__).parent / "usd" / digest / args.mode)
    log("DR_CONFIG asset=%s sha256=%s stage=%s envs=%d handoff=%d"
        % (args.asset, digest, args.dr_stage, args.num_envs, args.handoff_steps))
    env = DREnv(cfg, sampler, args.dr_stage, fixture_z=args.fixture_z,
                handoff_steps=args.handoff_steps, hip_sign=args.hip_sign, ground_margin=args.ground_margin, log=log)
    from asset_contract import apply_shared_plane_contract
    apply_shared_plane_contract(args.asset)
    import json as _json
    reference = _json.loads((Path(__file__).parent / "static_reference.json").read_text())
    env.reference_q = torch.tensor(reference["reference_q"], device=env.device)
    env.reference_height = reference["reference_height"]

    if args.verify:
        env.dr_check_readback = True
        # quaternion convention probe: write a known 90 deg yaw and read it back
        probe = tensor(env.robot.data.root_pose_w)[:1].clone()
        probe[0, 2] = env.terrain.env_origins[0, 2] + 0.45
        probe[0, 3:] = torch.tensor([0.7071068, 0., 0., 0.7071068], device=env.device)
        idx1 = torch.zeros(1, dtype=torch.long, device=env.device)
        env.robot.write_root_pose_to_sim(probe, idx1)
        env.scene.write_data_to_sim()
        env.sim.step(render=False)
        env.robot.update(env.physics_dt)
        back = tensor(env.robot.data.root_quat_w)[0]
        log("DR_QUAT sent=(w,x,y,z)=(0.707,0,0,0.707) read_back_wxyz=(%.3f,%.3f,%.3f,%.3f) "
            "read_back_xyzw=(%.3f,%.3f,%.3f,%.3f)"
            % (float(back[0]), float(back[1]), float(back[2]), float(back[3]),
               float(back[3]), float(back[0]), float(back[1]), float(back[2])))
        log("DR_ENV max_episode_length=%d step_dt=%.4f episode_length_s=%.2f decimation=%d physics_dt=%.4f"
            % (env.max_episode_length, env.step_dt, env.cfg.episode_length_s,
               env.cfg.decimation, env.physics_dt))
        stage_cfg = sampler.stage(args.dr_stage)
        wheel_y = env.wheel_bodies
        origin = env.terrain.env_origins
        zero = torch.zeros(env.num_envs, cfg.action_space, device=env.device)

        def pose_metrics(tag):
            env.scene.write_data_to_sim()
            env.sim.forward()
            env.robot.update(env.physics_dt)
            jp = tensor(env.robot.data.joint_pos)[:, env.ids]
            bp = tensor(env.robot.data.body_link_pos_w)[:, wheel_y]
            y = (bp[..., 1] - origin[:, None, 1]).abs().mean()
            z = float(tensor(env.robot.data.root_pos_w)[:, 2].mean())
            log("DR_VERIFY %-9s mean|wheel_y|=%.4f root_z=%.4f hip_q=%s calf_q=%s"
                % (tag, float(y), z,
                   " ".join("%+.3f" % v for v in jp[0, [0, 3, 6, 9]]),
                   " ".join("%+.3f" % v for v in jp[0, [2, 5, 8, 11]])))
            return float(y), jp.clone()

        # 1) nominal reset (no fixture) -> neutral wheel offset
        saved = stage_cfg["field_probability"]
        stage_cfg["field_probability"] = 0.
        obs, _ = env.reset()
        y_neutral, _ = pose_metrics("nominal")
        # 2) forced fixture reset -> same pose, must be splayed OUTWARD
        stage_cfg["field_probability"] = 1.
        obs, _ = env.reset()
        y_fixture, jp_fixture = pose_metrics("fixture")
        wp = tensor(env.robot.data.body_link_pos_w)[:, env.wheel_bodies][..., 2]
        log("DR_VERIFY wheel_bottom_min=%.4f m max=%.4f m (0 = touching, <0 penetrating, fixture envs only near this)"
            % (float((wp - 0.086).min()), float((wp - 0.086).max())))
        log("DR_VERIFY splay_sign %s (|y| fixture %.4f vs neutral %.4f)"
            % ("OK" if y_fixture > y_neutral else "SUSPECT(wrong hip sign?)", y_fixture, y_neutral))
        # 3) joint round-trip against the raw fixture
        fx = env.dr_last_fixture.get(0)
        if fx:
            want = torch.tensor(fx["q_dds"], device=env.device) * env.dr_sign
            got = jp_fixture[0]
            log("DR_VERIFY roundtrip max|dq|=%.5f  want_hip=%+.3f got_hip=%+.3f  want_calf=%+.3f got_calf=%+.3f"
                % (float((want - got).abs().max()), float(want[0]), float(got[0]),
                   float(want[2]), float(got[2])))
        stage_cfg["field_probability"] = saved
        # 4) handoff: 100 steps of zero action, command must stay zero
        obs, _ = env.reset()
        probe_action = zero.clone()
        probe_action[:, 1] = .25
        episodes_before = env.dr_episode.clone()
        cmd_max = 0.
        applied_ramp = []
        for i in range(args.handoff_steps + 5):
            obs, rew, term, timeout, info = env.step(probe_action)
            if i < 3:
                g = tensor(env.robot.data.projected_gravity_b)
                gz = " ".join("%+.2f" % v for v in g[:8, 2])
                fx = " ".join("F" if e in env.dr_last_fixture else "." for e in range(8))
                q = tensor(env.robot.data.root_quat_w)
                z = tensor(env.robot.data.root_pos_w)[:, 2]
                log("DR_PROBE step=%d term=%d failed_frac=%.3f grav_z_mean=%+.3f quat0=%s root_z=%.3f ep_len=%s"
                    % (i, int(term.sum()), float(term.float().mean()), float(g[:, 2].mean()),
                       " ".join("%+.3f" % v for v in q[0]), float(z.mean()),
                       str(env.episode_length_buf[:4].tolist())))
                log("DR_PROBE term_mask=[%s]" % " ".join("T" if t else "." for t in term[:8]))
                log("DR_PROBE per_env graz=[%s] fixture=[%s] wheelz=[%s]"
                    % (gz, fx, " ".join("%+.3f" % v for v in tensor(env.robot.data.body_link_pos_w)[:8, env.wheel_bodies[0], 2])))
            assert not (term | timeout).any(), f"unexpected termination at step {i}"
            live = env.dr_handoff_step < env.handoff_steps
            if live.any():
                cmd_max = max(cmd_max, float(env.command[live].abs().max()))
            if i in (0, 1, args.handoff_steps // 2, args.handoff_steps - 1, args.handoff_steps + 4):
                applied_ramp.append((i, float(env.applied[:, :12].abs().max()),
                                     int((env.dr_handoff_step < env.handoff_steps).sum())))
            assert torch.isfinite(obs["policy"]).all(), "obs NaN at step %d" % i
        log("DR_VERIFY handoff cmd_max_in_step=%.3e (must be 0) cmd_max_after_step=%.3e ramp=%s"
            % (env.dr_cmd_seen, cmd_max, " ".join("s%d:%.4f/%d" % a for a in applied_ramp)))
        assert torch.equal(env.dr_episode, episodes_before), "unexpected reset during handoff probe"
        assert env.dr_cmd_seen == 0., "nonzero command inside handoff"
        assert applied_ramp[2][1] > applied_ramp[0][1], "nonzero action did not ramp in"
        log("DR_VERIFY report %s" % json.dumps(env.dr_report()))
        log("DR_VERIFY_OK")
    else:
        from quality_training import train
        env.dr_log = log
        train(env, args, args.out)
except Exception:
    traceback.print_exc()
    code = 1
finally:
    if env is not None:
        env.close()
    app.close()
sys.exit(code)












