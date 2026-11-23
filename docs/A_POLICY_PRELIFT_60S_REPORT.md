# A_POLICY_PRELIFT_60S_REPORT — 新起身序列（先抬膝→收髋→伸腿）+ C1_s42_200 的 60 s 视频

> 任务：把 HimLoco 18200 纳入候选；起身改为"先抬膝避碰 → 再收髋 → 再伸腿"；选姿态最佳权重重做 60 s 视频。
> 范围：仅 MuJoCo；未接实机/publisher/未改包。Run UTC **2026-11-23T10:53–11:07Z（18:53–19:07+08）**，渲染 wall 792 s。
> 模型：`as2w_stand200_grid.xml`（26.000 kg，μ=0.35，effort 60/60/90 & 15 Nm，checker 地面），200 Hz 物理 / 30 fps 渲染。

## 0. 视频（ffprobe 已验证 60 s / 1800 帧）

- **路径**：`/root/as2w/mujoco-sim2sim-20261121/videos/a_policy_60s_prelift_best/a_policy_60s_prelift_best_grid.mp4`
- **SHA256**：`96434e33867d987d5031103cdfd44d9e23b9a412b3db191782c3b475065597c5`
- **ffprobe**：`h264`，`960×540`，`30/1 fps`，**1800 帧 / 60.000000 s**，size **8,395,650 B**，yuv420p、faststart。
- 抽帧（t=0.3 PRE_LIFT / 1.0 HIP / 3.0 EXTEND / 30 / 50 / 58 s）全部为机器人+格子画面（sat 5.1–6.7）；t=0.3 s 帧显示 `phase=PRE_LIFT`、kneeZ=0.032 m、wheelbase=0.424/trackF=0.480/trackR=0.477/minPair=0.424。
- 叠加：phase/时间/segment、腿增益、cmd vs ACTUAL vx,vy,wz、odom、z/tilt、support4/nonwheel/fall/cmd_viol、12 关节 q/tau 数值+bar（>0.8×effort 或 q 超软限位标红）、**wb/trackF/trackR/minPair + kneeZ**、world-map PiP、top-down 轨迹。

## 1. 新起身序列（200/4；8.0 s 后 55/2）

| 阶段 | 时间 | 内容 |
|---|---|---|
| **PRE_LIFT** | 0.00–0.60 s | hip 保持趴姿；thigh +1.019→+1.219、calf −2.762→−2.562（+0.20/+0.20 rad），抬膝离地 |
| **HIP** | 0.60–2.00 s | hip 趴姿 → 官方 ±0.15（1.4 s）；thigh/calf 保持 pre-lift |
| **EXTEND** | 2.00–5.00 s | thigh/calf → +0.60/−1.20（3 s）；hip 保持 |
| **HOLD_200_4** | 5.00–8.00 s | 官方 t2 |
| **HOLD_55_2** | 8.00–10.00 s | 官方 t2，55/2 |
| A_policy | 10.00–60.00 s | T15 同场景（C1_s42_200） |

pre-lift 增量取自网格搜索（`t17_prelift_probe.py`）：dth=+0.20/dca=+0.20 → 膝离地 0.0745 m、4 轮全触地、无身触地，为 knee≥0.03 解中最小改动且留余量。

## 2. 旧序列 vs 新序列：膝碰地量化（重点）

| 阶段 | 旧（无 pre-lift）膝最小离地 | 新（pre-lift）膝最小离地 | 变化 |
|---|---|---|---|
| 趴姿 | 0.0099 m | 0.0099 m | — |
| **收髋段** | **0.0250 m** | **0.0693 m** | **+4.43 cm（2.8×）** |
| 伸腿段 | 0.0791 m | 0.1249 m | +4.58 cm |
| HOLD t2 | 0.2405 m | 0.2577 m | +1.7 cm |
| 身-地接触帧（0–10 s） | **0 / 2000** | **0 / 2000** | 均无 |

**如实结论**：本 MuJoCo 资产中**旧序列本身也没有膝/小腿-地接触**（2000 帧 0 帧），所以 pre-lift 不是"消除已发生碰撞"，而是把收髋段最小膝离地从 2.5 cm 提到 6.9 cm（+4.4 cm）、给实机膝部网格/裙板/装配公差留安全余量；若实机实测有膝碰，该序列是有效手段。

## 3. 权重选择

**C1_s42_200**：`/root/gpufree-data/as2w-runs/retrain-dr3c-20261122/c1_s42/train/model_199.pt`，sha `572898f0e9897f5d03d32febb1bed60952a727662a074269bcc36f370550b3fb`。
20 s 新起身筛选（7 个 12-act 候选）：C1 的 **trackF 保持 0.582、minPair 0.223、tilt95 4.4° 均为 12-act 候选最好**；support4 0.746（DR3a 0.787 但 tilt95 24.3°/z min 0.326 → 低趴姿态差，不选）。详见 `WEIGHT_PRELIFT_COMPARISON.md`。

## 4. 60 s 结果（C1_s42_200）

| 验收项 | 结果 |
|---|---|
| 60 s fall | **0 fall**（0 帧；tilt95 段最大 8.8°，z 0.429–0.470） |
| 实际位移 | 净位移 **(2.74, +2.41) m**；yaw 净 −3.63° |
| vx/vy/wz | fwd+0.8→**0.711**（+3.13 m/5 s）；rev−0.8→**−0.697**（−3.07 m）；vy+0.45→**+0.410** / −0.45→**−0.313**；wz+0.8→**+0.732**（yaw +185.2°）/ wz−0.8→**−0.769**（yaw −194.1°）；arc/mixed_safe 正常 |
| 支撑 | support4 6787/10000（67.9%）；高速横移/急转瞬时轮载下降（打滑），无 fall |
| 越限 | τ>0.8×effort **0 帧**（τ 峰 27.16 Nm < 48）；q 软限位无越界；非轮触地 0；命令越界 0 |
| 几何（名义 stance wb 0.4236 / trackF 0.382 / trackR 0.3828 / minPair 0.382） | 60 s 最小：**wb 0.3169（74.8%）**、**trackF 0.1478（38.7%）**、**trackR 0.2099（54.8%）**、**minPair 0.1491（39.0%）**；policy 段 knee 最小 0.2054 m |
| 55/2 handoff | z 0.4426→0.4321（−1.05 cm）、tilt 0.01→0.55°、support4 1.0、无 fall |

## 5. HimLoco 18200：合同不兼容（独立栈可跑，非被比下去）

- 合同：**16 act**（12 腿 + 4 轮速度）、obs 需 **6×60 history + 19 latent estimator**（actor_in 79）、gait phase sin/cos、默认站姿 **(0,0.8,−1.6)**、腿 kp55/kd2（10 Hz LPF×0.15）、轮 kp0/**kd2**（5 Hz LPF）、100 cycle handoff。与 A/C1 的 58 obs/12 act/轮合同/官方 t2 默认站姿不同 → **不能同表公平比较**。
- 用其自带栈在 MuJoCo 中（新 pre-lift 起身 + 1 s 过渡到其默认站姿）**可跑**：20 s 全命令无 fall，z 0.340–0.365（更矮）、tilt95 3.94°、knee min 0.156 m、support4 0.691、非轮触地 0、几何保持 wb 87.6%/trackF 105%/trackR 104%/minPair 92.6%（其自身站姿更宽）。
- 判定：**合同不兼容，需独立部署栈**；若要用它，publisher 必须支持 16 维动作 + history/estimator。不是"参与后被比下去"。

## 6. 产物（/root/as2w/mujoco-sim2sim-20261121/）

| 文件 | 大小 (B) | SHA256 |
|---|---|---|
| `videos/a_policy_60s_prelift_best/a_policy_60s_prelift_best_grid.mp4` | 8,395,650 | `96434e33867d987d5031103cdfd44d9e23b9a412b3db191782c3b475065597c5` |
| `results/a_policy_60s_prelift_best/a_policy_60s_prelift_best_200hz.csv` | 14,250,866 | `00b3cabf5729d725a0f6746761542916bf238ae6daba063f48450aaca0088849` |
| `results/a_policy_60s_prelift_best/summary.json` | 7,327 | `53dad26973023f766458fd4929ed6ae0a0a233476fb450fe97f5558d2a83347d` |
| `a_policy_prelift_best.py` | 24,929 | `e4b1271d11baa8867d88778ab5a63fa2036ab32edc953c7a0211aa01e04eb44c` |
| `WEIGHT_PRELIFT_COMPARISON.md` | 7,480 | `65e6592a324764aa2800bb4936eed0535c04345a19375b822d28d38472c5949b` |
| `t17_screen.py` | 22,518 | `be3d06cea5e2831f5d16eec9b2ac1f55248d074e1db45cd36160f153e991654f` |
| `t17_prelift_probe.py` | 6,410 | `be8aa7db3caf5166baf74ad33f2a2bcfd8c2d766717229a87e887f9582a566a1` |
| `t17_himloco_metrics.py` | 6,523 | `320cd1710ee141c91d9506cdfa5e0a46c8ef580a5f13d12eb0c5a8cd397dbb36` |
| `results/t17_prelift/screen.json` | 5,842 | `9f9e7f348aa588841257d61166045eeebab1e4ef4d3e2771cf0b9f48be2e92cf` |
| `results/t17_prelift/standup_compare.csv` | 234,636 | `07a46d55962c07e21c7078258f254e570ec9db85a41bb1180091f685b815ccea` |
| `results/t17_prelift/himloco_stats.json` | 707 | `14ce046dc2a587c36f0a5799858a5538a366127c405bba9f2b4176c8e42acd0b` |

CSV 列（200 Hz，12000 行）：`t, phase, seg, kp, kv, cmd_vx/vy/wz, q_12, q_tgt_12, tau_12, wdq_4, w_tgt_4, x, y, yaw, z, roll, pitch, tilt, pose_err, vx, vy, wz, vx_world, vy_world, wheelbase, track_front, track_rear, min_pair, knee_z, fn_4, support4, nonwheel, cmd_viol, fall`。
复现：`OMP_NUM_THREADS=2 /root/as2w/preflight/isaac-python a_policy_prelift_best.py --ckpt /root/gpufree-data/as2w-runs/retrain-dr3c-20261122/c1_s42/train/model_199.pt --tag C1_s42_200_prelift`（约 13 min）。
