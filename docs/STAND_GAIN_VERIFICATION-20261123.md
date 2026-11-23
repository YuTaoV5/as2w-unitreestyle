# FixedStand 200/4 验证记录（2026-11-23）

> 结论：按原厂 `ai_stand_up` 的 **leg kp200/kd4 + wheel kp0/kd0.5** 后，FixedStand 稳定姿态 `pose_error` 从 55/2 的 **0.389** 降到 **0.1305**，低于 B gate **0.35**。本次只按 A，**未按 B**，30 s 后自动 FixSquat→Passive，随后恢复 `ai`。

## 1. 测试条件
- CPRO ARM64：`/root/as2w_deploy_v4x`，publish SHA `74180701…`
- policy A `63f1c5b6…`，deploy.yaml `531f07d0…`
- `--stand t2 --stand-kp 200 --stand-kd 4 --stand-hold-seconds 30`
- 同时只读记录 `rt/lowstate`（120 s）
- **未按 B**；无 PolicyWalk

## 2. FixedStand 稳定段（LowState 复算）
按 map `unitree58_measured.map` 与 deploy.yaml 目标 q 计算逐轴误差；稳定段（LowState 回放第 17–46 行）最大/逐轴：

| 关节 | max |err| (rad) |
|---|---:|
| FL_hip | 0.115 |
| FL_thigh | 0.041 |
| FL_calf | 0.108 |
| FR_hip | 0.120 |
| FR_thigh | 0.011 |
| FR_calf | 0.097 |
| RL_hip | 0.091 |
| RL_thigh | 0.023 |
| RL_calf | 0.124 |
| RR_hip | 0.076 |
| RR_thigh | 0.041 |
| RR_calf | 0.131 |
| **max_pose_error** | **0.1305** |
| 55/2 旧值 | 0.389（B reject） |
| B gate | 0.35 |

**判定：pose_error 0.1305 < 0.35，FixedStand 已不再后塌到 B 门以下。**

## 3. 后续
- 下一步才是 B 零速 PolicyWalk；必须在 FixedStand 稳定后 **30 s 内按 B**。
- 本次 LowState 稳定段已证明姿态；B gate 的其他门（wheel_dq/upright/quiet）仍需 B 按下时日志确认。
- 未启动 B、未进 PolicyWalk、未切包。

## 4. 文件
- `lowstate.log`：只读 LowState，120 s
- `publisher.log`：FSM/publisher
- `release.log`、`restore_ai.log`
- 本地/交付路径：`AS2W-交付整理-20261123/01_部署包_A_v4x/logs/zero_b_200_4_A_only/`
