#!/usr/bin/env python3
"""P9: HIMLoco (360 obs / 16 act) MuJoCo flat closed-loop replay.

Mirrors the deployed controller exactly (himloco_controller.cpp):
  obs  = 6 x 60 history, newest first; first step fills all six slots
  frame60 = cmd(3) gyro(3) gravity(3) (q-default)(16, wheels zeroed)
            dq*0.05(16) appliedLeg/0.25(12) appliedWheel/12(4) HEIGHT(1)
            sin(phi)*strength(1) cos(phi)*strength(1)   [strength == 0 here]
  act  = tanh(latent); leg_phys = [0.6,1,1]x4 * tanh[:12]
         wheel_phys = clamp((vx - wz*side)/R + 2*tanh[12:], +-12)
  target_leg   = lerp(target_leg,   0.25*leg_phys, a_leg)
  target_wheel = lerp(target_wheel, wheel_phys,    a_wheel)
  motors: legs  tau = clip(55*(default+target_leg - q) - 2*dq, +-35)
          wheels tau = clip(2*(target_wheel - dq), +-8)

Purpose: the policy has NO flat-ground replay evidence (its 31/32 acceptance is a
5 cm step task), so before touching hardware parameters we need to know the pitch
it settles at. Body-frame angular velocity is a quaternion finite difference,
because MuJoCo's free-joint qvel frame conventions proved unreliable here.

usage: p9_mujoco_himloco.py <policy.onnx> [--xml ...] [--z 0.36]
                            [--vx 0.15 0.25 0.0] [--sec 6]
"""
import json
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
import math
import sys

import numpy as np
import onnxruntime as ort
import mujoco

XML = "/root/as2w-runs/as2w-rolling-goal/work/armd_lp5/mj/r11_mjcf_local.xml"
FRAME, HIST, ACT = 60, 6, 16
DEFAULT = np.array([0.0, 0.8, -1.6] * 4 + [0.0] * 4)
LEGS = [f"{l}_{p}" for l in ("FL", "FR", "RL", "RR") for p in ("hip", "thigh", "calf")]
WHEELS = [f"{l}_foot" for l in ("FL", "FR", "RL", "RR")]
LEG_GAIN = np.array([0.6, 1.0, 1.0] * 4)
SCALE, WGAIN, CLAMP, R, SIDE, HEIGHT = 0.25, 2.0, 12.0, 0.086, 0.1654, 0.37
DT, DECIM = 0.02, 4
A_LEG = -math.expm1(-2 * math.pi * 10.0 * DT)
A_WHEEL = -math.expm1(-2 * math.pi * 5.0 * DT)


def parse(argv):
    o = {"xml": XML, "z": 0.36, "vx": [0.15, 0.25, 0.0], "sec": 6.0, "onnx": argv[0]}
    i = 1
    while i < len(argv):
        if argv[i] == "--xml": o["xml"] = argv[i + 1]; i += 2
        elif argv[i] == "--z": o["z"] = float(argv[i + 1]); i += 2
        elif argv[i] == "--sec": o["sec"] = float(argv[i + 1]); i += 2
        elif argv[i] == "--vx":
            o["vx"] = []
            i += 1
            while i < len(argv) and not argv[i].startswith("--"):
                o["vx"].append(float(argv[i])); i += 1
        elif argv[i] == "--urdf": o["urdf"] = argv[i+1]; i += 2
        elif argv[i] == "--initial-state": o["initial_state"] = argv[i+1]; i += 2
        else: raise SystemExit("bad arg " + argv[i])
    return o


def rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def ang_from_quats(prev, cur, dt):
    if prev is None:
        return np.zeros(3)
    w1, x1, y1, z1 = prev
    w2, x2, y2, z2 = cur
    cw, cx, cy, cz = w1, -x1, -y1, -z1
    w = cw * w2 - cx * x2 - cy * y2 - cz * z2
    x = cw * x2 + cx * w2 + cy * z2 - cz * y2
    y = cw * y2 - cx * z2 + cy * w2 + cz * x2
    z = cw * z2 + cx * y2 - cy * x2 + cz * w2
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    v = np.array([x, y, z]); n = np.linalg.norm(v)
    if n < 1e-12:
        return np.zeros(3)
    return v * (2.0 * math.atan2(n, w) / (n * dt))


def main(argv):
    o = parse(argv)
    sess = ort.InferenceSession(o["onnx"], providers=["CPUExecutionProvider"])
    inp, out = sess.get_inputs()[0], sess.get_outputs()[0]
    print("onnx %r %s -> %r %s" % (inp.name, inp.shape, out.name, out.shape))
    if "urdf" not in o or o["xml"] == XML:
        raise SystemExit("Explicit --xml and --urdf required; old r11 default disabled")
    m = mujoco.MjModel.from_xml_path(o["xml"])
    if abs(m.opt.timestep * DECIM - DT) > 1e-9:
        raise SystemExit("XML timestep * decimation != 20 ms; correct asset/time contract first")
    if m.nu:
        raise SystemExit("Remove XML actuators for this explicit qfrc_applied PD replay")
    urdf = ET.parse(o["urdf"]).getroot()
    limits = np.array([float(urdf.find("joint[@name='%s']/limit" % n).get("effort")) for n in LEGS+WHEELS])
    print("DIAGNOSTIC ONLY: URDF limits are not verified hardware limits")
    print("assets", {k:hashlib.sha256(Path(o[k]).read_bytes()).hexdigest() for k in ("xml","urdf","onnx")})
    initial = json.loads(Path(o["initial_state"]).read_text()) if "initial_state" in o else None
    jid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in LEGS}
    wid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in WHEELS}
    adr = {"q": np.array([m.jnt_qposadr[jid[n]] for n in LEGS]),
           "v": np.array([m.jnt_dofadr[jid[n]] for n in LEGS]),
           "wq": np.array([m.jnt_qposadr[wid[n]] for n in WHEELS]),
           "wv": np.array([m.jnt_dofadr[wid[n]] for n in WHEELS])}
    adr["all"] = np.concatenate([adr["v"], adr["wv"]])

    for vx in o["vx"]:
        d = mujoco.MjData(m)
        d.qpos[0:3] = [0, 0, o["z"]]
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qpos[adr["q"]] = DEFAULT[:12]
        d.qpos[adr["wq"]] = 0.0
        if initial:
            d.qpos[3:7] = initial["quat_wxyz"]
            d.qpos[adr["q"]] = initial["q"][:12]
            d.qvel[adr["v"]] = initial["dq"][:12]
            d.qvel[adr["wv"]] = initial["dq"][12:]
        mujoco.mj_forward(m, d)
        target_leg = np.zeros(12); target_wheel = np.zeros(4)
        history = None; q_prev = None
        pitch = []; zs = []; wdq = []; ldq = []; nw = []
        for f in range(int(o["sec"] / DT)):
            quat = d.qpos[3:7].copy()
            gyro = ang_from_quats(q_prev, quat, DT)
            q_prev = quat.copy()
            grav = rot(quat).T @ np.array([0.0, 0.0, -1.0])
            frame = np.zeros(FRAME)
            frame[0:3] = [vx, 0.0, 0.0]
            frame[3:6] = gyro
            frame[6:9] = grav
            frame[9:25] = np.concatenate([d.qpos[adr["q"]], d.qpos[adr["wq"]]]) - DEFAULT
            frame[21:25] = 0.0
            frame[25:41] = np.concatenate([d.qvel[adr["v"]], d.qvel[adr["wv"]]]) * 0.05
            frame[41:53] = target_leg / SCALE
            frame[53:57] = target_wheel / CLAMP
            frame[57] = HEIGHT
            if history is None:
                history = np.tile(frame, (HIST, 1))
            else:
                history = np.vstack((frame[None, :], history[:-1, :]))
            obs = history.reshape(-1).astype(np.float32)
            latent = sess.run(None, {inp.name: obs[None]})[0][0]
            b = np.tanh(latent)
            leg_phys = LEG_GAIN * b[:12]
            side = np.array([SIDE, -SIDE, SIDE, -SIDE])
            wheel_phys = np.clip((vx - 0.0 * side) / R + WGAIN * b[12:], -CLAMP, CLAMP)
            target_leg = target_leg + (SCALE * leg_phys - target_leg) * A_LEG
            target_wheel = target_wheel + (wheel_phys - target_wheel) * A_WHEEL
            for _ in range(DECIM):
                tau = np.concatenate([
                    np.clip(55.0 * (DEFAULT[:12] + target_leg - d.qpos[adr["q"]])
                            - 2.0 * d.qvel[adr["v"]], -limits[:12], limits[:12]),
                    np.clip(2.0 * (target_wheel - d.qvel[adr["wv"]]), -limits[12:], limits[12:])])
                d.qfrc_applied[adr["all"]] = tau
                mujoco.mj_step(m, d)
            w2, x2, y2, z2 = d.qpos[3:7]
            pitch.append(math.degrees(math.asin(max(-1, min(1, 2 * (w2 * y2 - z2 * x2))))))
            zs.append(float(d.qpos[2]))
            wdq.append(float(np.abs(d.qvel[adr["wv"]]).mean()))
            ldq.append(float(np.abs(d.qvel[adr["v"]]).max()))
            n = 0.0
            for c in range(d.ncon):
                b1 = m.geom_bodyid[d.contact[c].geom1]; b2 = m.geom_bodyid[d.contact[c].geom2]
                n1 = m.body(b1).name if b1 >= 0 else ""
                n2 = m.body(b2).name if b2 >= 0 else ""
                if n1 in WHEELS or n2 in WHEELS:
                    continue
                fc = np.zeros(6); mujoco.mj_contactForce(m, d, c, fc)
                n = max(n, abs(fc[0]))
            nw.append(n)
        p = np.array(pitch); zz = np.array(zs); nn = np.array(nw)
        print("vx=%+.2f  pitch mean %+6.2f (min %+6.2f max %+6.2f)  z %.3f (min %.3f)  "
              "wheel|dq| %.2f  leg|dq|max %.2f  nonwheel %.1f N (%d frames)"
              % (vx, p.mean(), p.min(), p.max(), zz.mean(), zz.min(),
                 max(wdq), max(ldq), nn.max(), int((nn > 5).sum())))
        # settle window: last third
        k = len(p) // 3
        print("        last-third pitch mean %+6.2f (min %+6.2f max %+6.2f)  z %.3f"
              % (p[-k:].mean(), p[-k:].min(), p[-k:].max(), zz[-k:].mean()))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
