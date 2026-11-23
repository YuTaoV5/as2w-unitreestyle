# 髋关节符号修正（2026-11-23）

## 现象
官方 `ai_stand_up` 从正面看前后腿向内；我们按 A 后腿向外。

## 根因
官方 `lowcmd_capture.log`（01_prone_to_stand_153922，elapsed=20000 ms，站立段）原始 16 槽：

| slot | 关节 | q | kp | kd |
|---:|---|---:|---:|---:|
| 0 | FR_hip | **+0.15** | 200 | 4 |
| 3 | FL_hip | **−0.15** | 200 | 4 |
| 6 | RR_hip | **+0.15** | 200 | 4 |
| 9 | RL_hip | **−0.15** | 200 | 4 |
| 1/4/7/10 | thigh | +0.60 | 200 | 4 |
| 2/5/8/11 | calf | −1.20 | 200 | 4 |
| 12–15 | wheels | 0 | 0 | 0.5 |

我们的 `config/unitree58_measured.map` 四个 hip 原来都是 `sign=-1`，导致 policy 目标 `FL/RL=-0.15、FR/RR=+0.15` 映射到 raw 后反号，腿向外。

## 修正
- 四个 hip 的 map sign：**−1 → +1**
- 目标 raw q 变为：FR/RR **+0.15**、FL/RL **−0.15**，与官方一致。
- 新 map SHA256：`f33fcae4d72e353efdb3d9b727b131fa72965890b9c5f97be14b1d58c3de598c`
- 已重跑 CTest **9/9 PASS**、offline-audit **PASS**。

## 当前状态
- 修正已写入 CPRO `/root/as2w_deploy_v4x/config/unitree58_measured.map` 与 manifest。
- 尚未完成修正后的 A→FixedStand 实机复测：等待操作员 A 时窗口内未按下，已安全停止并保持/恢复 `ai`。
- 下一步：A→FixedStand（200/4，不按 B）确认腿向内、pose_error 过门，再讨论 B 零速。

## 注意
- 之前 200/4 的 `pose_error=0.1305` 是在 **旧 hip 符号** 下对“向外目标”的测量，不能作为官方站姿验证；修正后必须重测。
