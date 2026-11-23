# AS2-W v4.x zero-B 实机尝试记录（2026-11-23 11:01）

> 结果：**T0/T1 未通过**。A 触发 FixStand/FixedStand，但未到原厂站姿；B 被 B-gate 拒绝；现场 X 进入 EmergencyDamping；**未进入 PolicyWalk、未执行零速策略**。已恢复原厂 `ai` 控制。

## 1. 现场状态与动作
- CPRO→机器人 DDS 在线：LowState ~1kHz，初始 `mode_machine=2`，`motion-mode=ai`。
- 执行 `ReleaseMode rc=0`；publisher 启动：
  - `--zero-b-trial --stand t2 --interp-sec 3 --squat-sec 4 --stand-hold-seconds 30`
  - `--vx-limit 0.10 --vy-limit 0 --wz-limit 0.10`
- 操作员 A：`[Passive] → FixStand (A)` → `[FixStand] → FixedStand (PD hold)`。
- **未到原厂 t2 站姿**：
  - `pose_error=0.389112`（B gate 0.35）
  - `upright=0.909642`（gate 0.9）
  - 逐轴误差：RL_hip 0.1889、RL_thigh 0.3796、RL_calf 0.3891、RR_hip 0.1801、RR_thigh 0.3645、RR_calf 0.3744（rad）；前腿约 0.03–0.04。
- B 被拒：`[FixedStand] B rejected`。
- 现场 X：`[FixedStand] → EmergencyDamping`；`ZERO-B TRIAL: emergency passive hold complete, stop`；publisher exit=0。
- 恢复：`MotionSwitcher SelectMode("ai") rc=0`，CheckMode=`ai`；sport-mode 2/3。

## 2. 数据/SHA
目录 `/root/as2w_deploy_v4x/field_zero_b_20261123/`
- `publisher.log` SHA `ddc0ce2f…`（3373 B）
- `debug_lowcmd.csv` SHA `06422338…`（6,039,436 B，14,566 行）
- `debug_policy.csv` SHA `fac79aea…`（295 B，**0 行策略输入/目标**）
- `release.log` SHA `919a9f81…`（ReleaseMode rc=0）
- 本地镜像：`X:\Items\as2w\.tmp_cpro_deploy\zero_b_20261123\`

## 3. 判定与原因
- **T0/T1 失败**：A 后进入 FixedStand，但 PD 固定站未把机器人拉到 t2；RL 后腿误差 0.36–0.39 rad，超过 B gate。
- 这解释了“按 A 后没切到原厂固定站姿”：**A 键生效，但 FixedStand 的 PD 目标/实际姿态未收敛**；不是 A 键没有触发。
- 可能因素：
  1. 从 released/低姿态开始，`kp55/kd2` 固定 PD 无重力前馈，后腿负载下无法抬到 t2；
  2. 初始姿态与 t2 差距较大，`--interp-sec 3` 不足；
  3. 现场 t2 站姿 sag 已知 0.29–0.40，本机 RL 达到 0.389。
- **不能按 B**（B gate 正常 fail-closed）；不能放宽 `--b-gate-pose 0.40` 做首飞，除非按阶段2条件单独签字。

## 4. 当前状态
- 机器人已恢复原厂 `ai` 控制；未继续发布 LowCmd。
- 本次 **未进入零速 PolicyWalk**，因此没有零速策略跟踪/停车数据；M0、DDS 50Hz 仍未测。
- 未切包、未修改包/权重。

## 5. 下一步建议
1. 先解决“FixedStand 到 t2 收敛”问题（同增益标定/重力前馈/更长插值/初始姿态），不要直接 B。
2. 若必须用 B gate 0.40，只能按阶段2条件（稳定 pose_error 0.35–0.40、逐轴≤0.30、其他门全过）单独签字；本次 RL 轴 >0.30，不满足。
3. 恢复到可重复的 FixedStand 后，再重跑 T0/T1；仍不进入 T2/T3。
