# 髋符号只读核对（2026-11-23）

> 不启动 publisher、不发 LowCmd；机器人由原厂 `ai` 控住站立，CPRO 仅订阅 `rt/lowstate`。

## 当前 raw q（LowState，mode=ai，站立）
| 槽 | 关节 | 实测 raw q | 官方 ai_stand_up 目标 | 误差 |
|---:|---|---:|---:|---:|
| 0 | FR_hip | **+0.1070** | **+0.15** | 0.043 |
| 3 | FL_hip | **−0.1157** | **−0.15** | 0.034 |
| 6 | RR_hip | **+0.1227** | **+0.15** | 0.027 |
| 9 | RL_hip | **−0.1156** | **−0.15** | 0.034 |
| 1/4/7/10 | thigh | 0.594–0.599 | +0.60 | ≤0.006 |
| 2/5/8/11 | calf | −1.220…−1.242 | −1.20 | ≤0.042 |

- 符号完全一致：**FR/RR hip = +，FL/RL hip = −**
- 最大目标误差约 **0.043 rad**（官方站姿、原厂 200/4 控住）
- `mode=ai`，LowState ~1 kHz，无 publisher/LowCmd

## 结论
- `unitree58_measured.map` 的 hip `sign=+1` 与官方 raw 合同一致；此前 `sign=-1` 会把目标反号，导致腿向外。
- 我们 `deploy.yaml` 的站姿目标（policy order FL −0.15、FR +0.15、RL −0.15、RR +0.15）经 map +1 后，raw 与官方逐槽一致。
- 这次是**只读**验证；未验证我们 publisher 的 FixedStand 动态过程。下一步仍需 A→FixedStand（200/4，不按 B）确认动态姿态和 pose_error。
