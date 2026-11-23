# AS2-W 分析过程与决策链（迁移快照 2026-11-23）

> 目的：新服务器/新 agent 在不依赖旧会话上下文的情况下，理解为什么走到 A_400、哪些结论已证、哪些仍未证。

## 0. 结论先行
- **A_799**：原部署候选；固定站/符号/stand 增益已修，但实机 zero-B 仍未过 B gate，不能发布。
- **A_400**：T18 内八筛选最接近官方；已生成自包含部署包；**CPRO audit/实机/owner release 未完成**。
- **C1_s42_200**：上一轮姿态候选，保留作对照/回退。
- **v2step / HimLoco 18200**：合同不同或稳定性差，不能与 A 合同直接混表。
- **发布状态**：所有候选 release_authorized=false；不得把仿真结论当实机放行。

## 1. 官方数据反析（2026-11-19 数据，11-21 分析）
证据：`docs/`、`artifacts/AS2W_DELIVERY_20261123.zip` 内报告。
- 官方 `ai_stand_up` 原始 LowCmd：
  - 腿 **kp200/kd4**，轮 **kp0/kd0.5**；q 目标走官方 t2（hip ±0.15、thigh +0.60、calf −1.20），dq=0，tau=0。
  - 原始槽：FR/RR hip **+0.15**，FL/RL hip **−0.15**。
- 官方起身顺序：14.25 s 起 200/4；hip 约 15.69 s 到位；calf 16.03→16.82 s；thigh 16.54→16.83 s；18.2 s 稳定。
- 早期我们的 publisher 使用 55/2（策略合同）做 FixedStand，后腿静差大；B gate `pose_error=0.389>0.35`，被拒。
- 将 FixedStand 改为官方 200/4、轮 0/0.5 后，A-only 实测 pose_error 降到约 0.13–0.17，但实机 hip 仍未完全到官方内收值。

## 2. 符号/零位/代码修正
- map 原四个 hip `sign=-1`，导致目标 raw 反号、腿外八；改为 `+1`，raw 目标与官方 FR/RR +0.15、FL/RL −0.15 一致。
- FixedStand/ReturnStand 原调用 `fixed_stand_targets(fixed_pose)` 未传 200/4，保持阶段回落 55/2；源码已修为显式传 `stand_kp/stand_kd`。
- 只读 ai 站姿实测：raw hip FR/RR≈+0.107/+0.123、FL/RL≈−0.116，符号正确；但 publisher 的 hip 实际只到 ~0.004–0.048，说明还有未建模路径/接触/摩擦问题。

## 3. 门限与 T8（A_799）
- Owner 签署 **Scheme 2**：固定输入 raw action ≤1e-6、最终 LowCmd ≤1e-6；闭环 raw action 仅诊断。
- T8：固定输入 12 场景 max **9.318e-7 PASS**；最终 LowCmd q 4.8e-7、dq 9.6e-7；闭环 raw action max 3.427e-6（诊断）。
- 未完成：完整 FSM/DDS LowCmd 逐帧、DDS/publisher 50 Hz、M0、实际加载文件核验、owner 最终放行。
- 2e-6 已撤回；不能用 `--b-gate-pose 0.40` 绕过首飞。

## 4. 实机 zero-B 事实
证据：`docs/ZERO_B_FIELD_ATTEMPT-20261123.md`、现场 logs。
- 55/2 版：A→FixedStand，B reject `pose_error=0.389`，X EmergencyDamping。
- 200/4 版：A→FixedStand，但 B 未在 30 s 内按下，自动 squat；另一次在 Passive 发生 FOC timeout。
- 机器人已恢复 `ai`；未进入 PolicyWalk；未完成 T0/T1。
- 目前没有 active publisher、没有持续 LowCmd。

## 5. 仿真阶段（T10–T18）
全部仅 MuJoCo，不接实机。
- **T10 高增益起身**：高增益可把 hip 拉到目标，但“高增益后降回 200/4”会回到静差；400/8 过冲。
- **T11 200/4→55/2 切换**：仿真里 hip 基本不变，膝/高度软塌 1.4–2.2 cm；手交保持目标优于冻结。
- **T12 官方起身顺序**：先髋后腿可改善后腿内收；S2 高增益无收益。
- **T13 视频**：趴→收髋→3s 起身→55/2；膝软/掉高 1.46 cm，hip 变化≤0.002。
- **T14/T15 60s**：A 策略全范围/多场景；速度能跟踪；support4 动态下降，exact mixed 会倒。
- **T16**：13 个候选姿态筛选，C1_s42_200 综合轮距/hip 较好。
- **T17**：加 pre-lift 抬膝避碰；旧仿真本无膝碰，pre-lift 把收髋段膝离地 2.5→6.9 cm；HimLoco 18200 合同不同（16 act/estimator），自身栈可跑但偏外八。
- **T18 内八筛选**：官方 toe-in 5.54°；A_400（model_400，DR2b s42）6.53°，Δ0.98°（候选最小）、support4 0.89、轮距未收窄；选 A_400。60s 动态 toe-in 均值 8.39°，左右不对称，硬段轮距仍收缩 → **A_400 是最接近官方但不是完全复刻**。

## 6. 为什么选 A_400
- 目标从“外八最小”修正为“内八贴合官方”；评分 = |toe_in − official 5.54°|；
- A_400 最小；A_600/C1_100/A_200/A_799/C1_200 偏差更大；
- 不靠收窄轮距/膝碰提高评分；
- 但 A_400 只是中间 checkpoint，不是最终 A_799；部署包必须标注 UNRELEASED。

## 7. 当前迁移状态与下一步
- 迁移仓库保存：A/C1/A400 部署包、关键训练资产、报告、视频、脚本、server manifest、SHA。
- 需要在新服务器/CPRO 完成：
  1. A_400 C++ offline audit + CTest（包内安装脚本会 audit）；
  2. zero-B/B gate 实机（需 owner、现场安全）；
  3. DDS/publisher 50 Hz、完整 FSM/DDS LowCmd、M0；
  4. 实际加载文件 SHA 核验；
  5. owner 最终放行。
- 服务器若关闭：按 `server_manifest/MIGRATION_MANIFEST.json` 的路径/SHA 与 `README_MIGRATION.md` 恢复；缺失 raw logs 需从仍在线副本补拉。
