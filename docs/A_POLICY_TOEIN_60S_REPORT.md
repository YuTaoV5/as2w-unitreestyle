# A_POLICY_TOEIN_60S_REPORT — 内八最贴合官方 ai 站姿的权重（A_dr2b_s42_400）60 s 视频

> 任务（修正版）：用户更倾向**内八步态符合官方 ai 站姿**（不是"外八角最小化"）。本报告按新评分重做：先测官方内八参考，再筛选候选，选 `|toe_in − official|` 最小且稳定者做 60 s 视频。
> 范围：仅 MuJoCo；未接实机/publisher/未改包。Run UTC **2026-11-23T11:18–11:32Z（19:18–19:32+08）**，渲染 wall 786 s。
> 模型：`as2w_stand200_grid.xml`（26.000 kg，μ=0.35，effort 60/60/90 & 15 Nm，checker 地面）；200 Hz；起身用 T17 **PRE_LIFT→HIP→EXTEND→HOLD_200_4→55/2** 序列。

## 0. 视频（ffprobe 已验证 60 s / 1800 帧）

- **路径**：`/root/as2w/mujoco-sim2sim-20261121/videos/a_policy_60s_toein_best/a_policy_60s_toein_best_grid.mp4`
- **SHA256**：`35c2015ea3e3628b831a0e237c79908877c08246b75c8da2283b4a8d382495ac`
- **ffprobe**：`h264`，`960×540`，`30/1 fps`，**1800 帧 / 60.000000 s**，size **8,613,359 B**，yuv420p、faststart。
- 叠加：phase（PRE_LIFT/HIP/EXTEND/HOLD_200_4/HOLD_55_2/A_policy）/时间/segment、cmd vs ACTUAL vx,vy,wz、odom x,y,yaw、z/tilt/support4/nonwheel/fall/cmd_viol、**wb/trackF/trackR/minPair + kneeZ**、**toeIn（L/R）+ hipOut**、12 关节 q/tau 数值+bar（红色越限：0 帧）、world-map PiP、轨迹图。
- 抽帧（t=0.3/3.0/20/30/50/58 s）全为机器人+格子画面（sat 4.9–6.7）；t=30 s 帧显示 `toeIn=+12.7deg (L +12.8/R +12.5) hipOut=12.7deg`。

## 1. 官方内八参考（同一模型 / 同一起身序列）

定义：轮轴 = 轮几何体 local x，`cam=asin(axle_z)`；**左腿 toe_in=−cam，右腿 toe_in=+cam**（向内侧为正）。轮子内八在本机器人表现为"轮平面内倾 + 轮心内收"（髋外展驱动；髋 q=0/±0.15/±0.30 对应前轮距 0.341/0.254/0.124 m，数值验证）。

| 参考 | toe_mean | L / R | 对称 | 每腿 FL/FR/RL/RR | hip_out |
|---|---|---|---|---|---|
| REF-A（官方 t2 目标 200/4 稳定） | **4.81°** | 4.82/4.79 | 0.03° | 4.83/4.81/4.81/4.78 | 4.81° |
| **REF-B（官方 ai 实测 hip，主参考）** | **5.54°** | 5.55/5.54 | 0.006° | **6.51/6.37/4.59/4.72** | 5.55° |

→ `official_toe_in_deg = 5.54°`。

## 2. 选中的权重：A_dr2b_s42_400

- 路径：`/root/gpufree-data/as2w-runs/retrain-dr2b-20261122/dr2b_s1_seed42/train/model_400.pt`，SHA256 `88cff1a264ad3ffe5e7b9fc61ddee4998d6101a1f30fa68d0218696e5bbf4802`（DR2b seed42，**400 步**，58 obs/12 act，腿 55/2，轮合同）。
- 20 s 筛选中 **|toe_mean−5.54| = 0.98°（全候选最小）**，且：无 fall/膝碰/非轮触地、support4 **0.89（最高）**、轮距 **tf_rel 0.670 / tr_rel 0.703（不窄）**、tilt95 6.4°、z 0.429–0.470。
- 相比 A_799（当前）：toe 8.71°→6.53°（更贴官方）、hipOut 9.79→7.36°、support4 0.73→0.89、轮距更宽；C1_200 的 toe 9.10°（|Δ|3.56）在此评分下不如 A_400。

## 3. 60 s 结果（A_dr2b_s42_400）

| 验收项 | 结果 |
|---|---|
| 60 s fall | **0 fall**（0 帧） |
| 实际位移 | 净位移 **(−2.65, +3.76) m**；yaw 净 **+124.8°** |
| vx/vy/wz | fwd+0.8→**0.727**（+3.21 m/5 s）；rev−0.8→**−0.719**（−3.13 m）；vy+0.45→**+0.400** / −0.45→**−0.253**；wz+0.8→**+0.776**（yaw +195.5°）/ wz−0.8→**−0.504**（yaw −127.4°）；arc/mixed_safe 正常 |
| **内八指标（policy 段 10–60 s）** | toe_in 均值 **8.39°**（min 3.91 / max 16.56；左 10.81 / 右 5.98）、hip_out 均值 9.48°；相对官方 5.54°：静态/保持段约 6.5°，激进机动时会内收到 8–17° |
| 轮距/轴距（名义 wb 0.4236 / trackF 0.382 / trackR 0.3828 / minPair 0.382） | 60 s 最小 wb 0.3321（78.4%）、trackF 0.1946（50.9%）、trackR 0.2256（58.9%）、minPair 0.1949（51.0%） |
| 支撑/越限 | support4 6854/10000（68.5%）；τ 峰 **26.18 Nm**、τ>0.8×effort **0 帧**；非轮触地 **0**；命令越界 **0** |
| 55/2 handoff | z 0.4426→0.4321（−1.05 cm）、tilt 0.01→0.55°、support4 1.0 |

> 注：60 s 平均 toe_in 8.39° 高于官方 5.54°（激进机动时髋内收更多），但**这是"内八"方向偏大而非外八**；20 s 筛选中 A_400 已是最接近官方者。视频中 toe_in 字段可逐帧查看。

## 4. HimLoco 18200（单表，合同不同，不混表、未选）

- 合同：16 act（12 腿 + 4 轮速度）+ history 6×60 + estimator latent 19（actor_in 79）+ gait phase + 默认站姿 (0,0.8,−1.6) + 腿 kp55/kd2(10Hz LPF×0.15) + 轮 kp0/kd2(5Hz) + 100 cycle handoff。
- 自带栈 MuJoCo 20 s：无 fall，**toe_mean −1.65°**（略外八）、|Δofficial|=**7.19°**、hip_out 4.19°、support4 0.652、z 0.340–0.365、tilt95 3.95°、几何 wb 87.6%/trackF 105%/trackR 104%。
- 结论：合同不同不能与 12-act 候选同表公平比较；其 toe 方向偏外八且离官方远 → **最终视频用 12-act 最佳 A_dr2b_s42_400，不用 HimLoco**。
- 另：v2step 系列在 A 合同下 8/9 失稳跌倒（合同/默认站姿不同），稳定者 toe 13–14°、support4 仅 0.24–0.27，同样未选。

## 5. 限制

- MuJoCo 理想 PD/轮伺服、无电流环/齿隙/强 stiction/机体自碰撞；toe_in 由轮轴倾角/髋角计算（该模型轮子 yaw≈0，内八表现为 camber+轮心内收）。
- 60 s 为策略开环执行（obs 由仿真状态构造，无感知/通信延迟）；exact mixed 仍不稳定 → 用 mixed_safe(0.4,0.10,−0.3) 并已标注。
- 渲染：X 共享 GL 读回污染 → 每帧新建 Renderer + 饱和度校验。

## 6. 产物（/root/as2w/mujoco-sim2sim-20261121/）

| 文件 | 大小 (B) | SHA256 |
|---|---|---|
| `videos/a_policy_60s_toein_best/a_policy_60s_toein_best_grid.mp4` | 8,613,359 | `35c2015ea3e3628b831a0e237c79908877c08246b75c8da2283b4a8d382495ac` |
| `results/a_policy_60s_toein_best/a_policy_60s_toein_best_200hz.csv` | 15,357,102 | `68657ea34aaabec0dc7cf25d5bb4d7ceef5527def10bc9ba929c96a8a67a501e` |
| `results/a_policy_60s_toein_best/summary.json` | 7,495 | `6ba2499dfcb193426e4062299719941561f05f272f9916bf8a88f4cf059adff9` |
| `a_policy_toein_best.py` | 26,803 | `6cdb59888849cf702b226c59ab712990f4f11f1d9259c06dcbdf34a907dfb605` |
| `WEIGHT_TOEIN_COMPARISON.md` | 8,717 | `b58a3ad834873bb66554394b63f54821e50e8e39c384f464c5c17025af16148f` |
| `results/t18_toein/official_reference.json` | — | `95d16a0fc2bc047c71dbec52946266cae379b0ae6b6ad38ccda208f70df74461` |
| `results/t18_toein/screen.json` | — | `17c578accb7371d865410436ad3225da49b6c641f496a3a46b89de3548c8e942` |
| `results/t18_toein/screen.csv` | — | `dfa6e59d9e4a0db416ca9927052e8708528121c8ff1eba50c13a24b7d70a7c13` |
| `results/t18_toein/himloco_toein.json` | — | `cce61bc74b3afdf70fdfc9540cbd5d276f14d8863930f681baa49a6e006e22a9` |
| `t18_toein_screen.py` | 13,459 | `9c7521f2149fa87e310a42017490fe8c8202635f87bee40bea38109165295049` |
| `t18_himloco_toein.py` | 4,870 | `927799d323079cdd8b8b60a74b6d5a5b50da0bef85f0d834ba25337db36b4bc7` |

CSV 列（200 Hz，12000 行）：`t, phase, seg, kp, kv, cmd_vx/vy/wz, q_12, q_tgt_12, tau_12, wdq_4, w_tgt_4, x, y, yaw, z, roll, pitch, tilt, pose_err, vx, vy, wz, vx_world, vy_world, wheelbase, track_front, track_rear, min_pair, knee_z, toe_in, toe_left, toe_right, toe_sym, hip_out, fn_4, support4, nonwheel, cmd_viol, fall`。
复现：`OMP_NUM_THREADS=2 /root/as2w/preflight/isaac-python a_policy_toein_best.py --ckpt /root/gpufree-data/as2w-runs/retrain-dr2b-20261122/dr2b_s1_seed42/train/model_400.pt --tag A_dr2b_s42_400`（约 13 min）。
