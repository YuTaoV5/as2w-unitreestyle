# A_POLICY_60S_FULL_SIM_REPORT — 60 s 全命令范围 A 策略仿真视频（格子地面，含真实位移）

> 任务：仅 MuJoCo，不接实机/不启动 publisher/不改包。解除首飞限幅，按 A 训练完整命令范围做 60 s 连续视频。
> Run UTC **2026-11-23T09:44–09:59Z（17:44–17:59+08）**，wall 762.6 s。
> A 策略：部署 ONNX `/root/as2w/retrain-wheeldiff-v41-20261121/deploy_v4x/pkg/policy.onnx`，**SHA256 `63f1c5b6a6bf5c7fd9fdcaa09cce93deb5aa5c81dc97c96e6c0dd72bf7ab64ee`**（已校验）；58 obs / 12 act，50 Hz，腿 55/2，轮合同。
> 模型：`as2w_stand200_grid.xml`（26.000 kg，μ=0.35，effort 60/60/90 & 15 Nm，checker 地面）；200 Hz 物理 / 30 fps 渲染。

## 0. 视频（ffprobe 已验证可播放）

- **路径**：`/root/as2w/mujoco-sim2sim-20261121/videos/a_policy_60s_full/a_policy_60s_full_grid.mp4`
- **SHA256**：`524386f50590c57394048ed04be93369258f1d58c6ca1030f1bd932ec80aef5d`
- **ffprobe**：`h264`，`960×540`，`30/1 fps`，**1800 帧 / 60.000000 s**，size **8,165,597 B**，yuv420p、faststart。
- 画面：**checker 格子地面**；叠加 phase/时间/segment/腿增益 kp,kv/**cmd vs ACTUAL vx,vy,wz**/odom **x,y,yaw**/z/tilt/support4/nonwheel/fall/cmd_viol；左侧 12 关节 **q、tau 数值 + tau bar（>0.8×effort 或 q 超软限位标红）**；**左下角 world-map 固定相机 PiP**（看整体位移）；右下角 **top-down 轨迹图**（实时累积 x-y 路径）。
- 抽帧核验（t=2/12/20/30/40/50/58 s）全部为机器人+格子画面（sat 4.8–6.3），t=50 s 帧中 odom 已显示 x=−0.06,y=+2.24,yaw=+129.6°、轨迹图已画出 U 形路径 → **机器人确实在移动**。

## 1. 运动验证（本轮关键，回应上轮“几乎没动”）

| 指标 | 数值 |
|---|---|
| A 策略阶段路径长度（10–60 s） | **15.81 m** |
| 净位移 (x, y) | **(−0.48, +4.26) m** |
| yaw 净变化 / 范围 | **+55.0°** / [−2.2°, +198.8°] |
| 最大实际速度 | vx **0.79** m/s、vy **0.54** m/s、wz **1.12** rad/s |
| 前/后可复现位移（单独测试） | fwd +0.8：**3.86 m/5 s**（实 0.75 m/s）；rev −0.8：**−3.88 m/5 s**（−0.76 m/s）；左移 +0.45：**+2.20 m/5 s**（实 0.44）；右移 −0.45：−1.44 m（实 −0.28） |

## 2. 场景表（10–60 s，10 段×5 s，slew 1.0 s）

| 段 | cmd (vx,vy,wz) | 实测 (vx,vy,wz) | 段内位移 (dx,dy) m | dyaw | z | tilt95 | 4 轮支撑 | τpeak |
|---|---|---|---|---|---|---|---|---|
| zero | (0,0,0) | (−0.00, 0.00, −0.00) | (−0.02, 0.00) | −0.3° | 0.448 | 2.4° | 1.00 | 5.7 |
| **fwd_p08** | (+0.80,0,0) | (**+0.72**, 0.01, 0.04) | (**+3.19**, +0.30) | +10.6° | 0.430 | 7.5° | 1.00 | 14.2 |
| **rev_n08** | (−0.80,0,0) | (**−0.71**, −0.00, −0.00) | (**−3.10**, −0.45) | −1.1° | 0.457 | 5.5° | 1.00 | 10.1 |
| **left_vy045** | (0,+0.45,0) | (−0.07, **+0.42**, 0.03) | (−0.81, **+1.72**) | +8.3° | 0.459 | 5.2° | 0.05 | 23.4 |
| **right_vy045** | (0,−0.45,0) | (−0.02, **−0.26**, −0.05) | (+0.17, **−1.15**) | −12.8° | 0.452 | 9.4° | 0.00 | 19.8 |
| **yaw_p08** | (0,0,+0.80) | (−0.08, 0.00, **+0.68**) | (+0.22, −0.19) | **+171.4°** | 0.436 | 8.9° | 0.44 | 35.4 |
| **yaw_n08** | (0,0,−0.80) | (−0.00, −0.04, **−0.57**) | (+0.16, +0.06) | **−144.7°** | 0.456 | 5.1° | 0.64 | 29.5 |
| **arc_p05p04** | (+0.50,0,+0.40) | (+0.45, −0.02, +0.32) | (+0.01, +1.83) | +81.6° | 0.450 | 4.2° | 0.83 | 16.5 |
| **mixed_safe** | (+0.40,+0.10,−0.30) | (+0.41, +0.02, −0.30) | (−0.27, +1.64) | −75.9° | 0.466 | 3.7° | 0.77 | 12.0 |
| stop | (0,0,0) | (0.00, −0.00, 0.02) | (+0.01, +0.01) | +4.9° | 0.466 | 1.7° | 0.97 | 8.8 |

## 3. 诊断：为什么用 slew 1.0 s 与 `mixed_safe`（诚实说明与请求场景的一处偏差）

- **rev→vy 转段**：slew **0.5 s** 时序列在 left_vy 段跌倒（t=25.38 s）；slew **1.0/1.5 s** 时该转段稳定（复测无 fall，位移 −1.06/+1.79 m）→ 主视频用 **slew 1.0 s**。
- **exact mixed `(+0.40,+0.25,−0.30)` 在 MuJoCo 中不稳定**：单独从站立执行也会跌倒（t_fall 2.2 s；换成 wy 0.15/0.25 与 wz −0.1/−0.2 的组合亦会倒；**即使 vy 用 1.5/2/3/4 s 慢斜坡仍跌**）。可稳定组合：`(0.4,0.20,−0.3)`（未倒）、`(0.4,0.25,−0.1)`（未倒）、`(0.4,0.10,−0.3)`（未倒）。为保持 60 s 连续无跌倒视频，**主视频用 `mixed_safe=(0.40,0.10,−0.30)`**；exact-mixed 失败作为独立发现记录（`results/a_policy_60s_full/command_matrix.json`、`diag2.json`）。
- 这不是“脚本没动”的问题：vx/vy/wz 映射、ONNX（内嵌 tanh，按合同乘 0.25 后 LPF）、轮合同均已核对；fwd/rev/strafe/yaw/arc 全部有正确方向与大位移。

## 4. 验收

| 验收项 | 结果 |
|---|---|
| **60 s 是否 fall** | **无 fall**（0 帧；tilt 最大 9.4°，z 0.430–0.466） |
| **vx/wz 响应** | **是**：vx ±0.8 → 0.72/−0.71（89–90%）；wz ±0.8 → 0.68/−0.57 且 yaw 改变 ±145–171°；vy ±0.45 → +0.42/−0.26；arc/mixed 正常 |
| **55/2 handoff** | 掉高 **1.34 cm**（0.4436→0.4302）；calf 软 **0.087/0.105 rad**；hip 保持 Δ≤0.0037 rad；无 fall |
| **是否越限** | τ>0.8×effort **0 帧**（τ 峰 35.4 Nm < 前 48）；q 软限位无越界；非轮触地 **0**；命令越界 **0** |
| **A 策略稳定性** | 60 s 无跌倒；但**高速横移/原地转时四轮支撑瞬时降到很低**（left 0.05、right 0.00、yaw 0.44/0.64；全程 67% 帧四轮全触地）——判为高速打滑/轮载转移（tilt 5–9°），无身接触、无 fall |
| **位移可见性** | 轨迹图 + odom + 固定 world-map PiP + 滑动格子地面；路径长度 15.81 m |

## 5. 限制

- MuJoCo 理想 PD/轮伺服、无电流环/齿隙/强 stiction/机体自碰撞；**exact mixed（带 vy 0.25 的组合）在此模型下不稳定**，不能外推实机；support4 在高速打滑时会降为 0（轮离地判定 Fn<3 N）。
- 60 s 中 10–60 s 为 A 策略在 MuJoCo 里开环执行（obs 由 MuJoCo 状态构造，无感知/通信延迟）；部署延迟未建模。
- 渲染：X 共享 GL 读回污染 → 每帧新建 Renderer + 饱和度校验；world-map PiP 每 2 帧刷新一次。

## 6. 产物（/root/as2w/mujoco-sim2sim-20261121/）

| 文件 | 大小 (B) | SHA256 |
|---|---|---|
| `videos/a_policy_60s_full/a_policy_60s_full_grid.mp4` | 8,165,597 | `524386f50590c57394048ed04be93369258f1d58c6ca1030f1bd932ec80aef5d` |
| `results/a_policy_60s_full/a_policy_60s_full_200hz.csv` | 13,068,525 | `4e1dcddfacfb295f093b0215c51702c6d6f05959dc4b3dd7d7f6b22e105ded0f` |
| `results/a_policy_60s_full/summary.json` | 6,747 | `0c10e9569465c6e3e80f8cb422f7c9cc453594c0f3e2acb0e157b7ce39cdd1a8` |
| `results/a_policy_60s_full/command_matrix.json` | 3,223 | `eab26a37339f3b5690e01e71c51d23cf78a3f44be2e69c175cda279bc68adfc6` |
| `results/a_policy_60s_full/diag2.json` | 2,375 | `9e502407c02ebfe4a9cca03eb4acdaa70f2c49c87aa7b6416307ee68153d1552` |
| `a_policy_60s_full.py` | 21,729 | `273879aa340ebf2ffbffd0ffe5f6b944c1b32320c6e2fa3c00f51e818d4e9b73` |
| `as2w_stand200_grid.xml` | 11,875+ | `4d1c72ab7fd4e0f7fb266403a9ca8ef6c1c7edfd2674d891091f032262ba90d8` |

CSV 列：`t, phase, seg, kp, kv, cmd_vx/vy/wz, q_12, q_tgt_12, tau_12, wdq_4, w_tgt_4, x, y, yaw, z, roll, pitch, tilt, pose_err, vx, vy, wz, vx_world, vy_world, fn_4, support4, nonwheel, cmd_viol, fall`（200 Hz，12000 行）。
复现：`OMP_NUM_THREADS=2 /root/as2w/preflight/isaac-python a_policy_60s_full.py`（约 13 min，含 1800 帧渲染 + PiP）。
