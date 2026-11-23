# AS2-W v4.x 跨架构验收门（OWNER-SIGNED · 方案 2 · 2026-11-23）

> **状态：OWNER-SIGNED（方案 2），发布仍锁定。**
> Owner 于 2026-11-23 10:54（Asia/Shanghai）批准方案 2：
> - **固定输入 raw action ≤1e-6**
> - **最终 LowCmd ≤1e-6**
> - **完整闭环 raw action 仅作诊断，不作为放行门**
>
> 原 `2e-6` 提审值已被扩展回放否决并撤回；不再使用。部署发布仍需满足：完整 FSM/DDS LowCmd 覆盖、DDS/publisher 50 Hz、M0 实测、实际加载文件/哈希核验、owner 最终放行。在此之前 **不启动 publisher、不发 LowCmd、不切包**。

## 0. 签字记录
- 批准人：owner（会话内确认）
- 批准时间：2026-11-23 10:54 Asia/Shanghai
- 批准范围：方案 2（固定输入 1e-6 + 最终 LowCmd 1e-6；闭环 raw action 诊断）
- 当前状态：**门限已签；发布未放行**
- 唯一 canonical：本文件；SHA256 见同目录 `GATE_V4X_CROSSARCH_DRAFT.sha256`
- 撤回：`2e-6` 不再作为任何验收门限

## 1. 已签门限

| 验收项 | 要求 | 当前证据 | 结论 |
|---|---|---|---|
| 固定输入 raw action | 同一 ONNX、逐字节相同 float32 输入；全部帧/维度 max_abs ≤ **1e-6** | T8 12 场景最大 **9.318e-7**（05_limit_wz_over） | **PASS** |
| 最终 LowCmd | 发包前浮点字段逐项 ≤ **1e-6**；关节映射、mode、FSM、限幅、守卫逐帧一致 | 12 fixture 控制器目标 q **4.8e-7**、dq **9.6e-7**、kp/kd/mode 0；离线 audit 2 帧 slot/16-axis 0.0 | **部分 PASS**；完整 FSM/DDS LowCmd 覆盖仍不足 |
| 完整闭环 raw action | **仅诊断，不作放行门** | 12 场景最大 **3.427e-6**（05_limit_wz_over）；03/04vx/08 2.034e-6 | 记录，不判放行 |
| DDS/publisher 50 Hz | p50/p95/max、>20 ms 次数 | controller-only 两次 run：19.995/20.055/20.94 ms，235/600；原始 CSV run 19.968/20.028/21.009 ms，83/600；DDS 未测 | **缺口，不能判 PASS** |
| M0 / 加载文件 | 观测年龄、指令延迟、ARM64 binary/config/map/ONNX SHA | 未测/部分核验 | **发布阻断** |

## 2. 关键证据

### 2.1 固定输入
- 12 个扩展场景固定输入 raw action max **9.318e-7**（05_limit_wz_over），全部 ≤1e-6。
- fixture 初始 300 帧固定输入 max **4.17e-7**。

### 2.2 闭环诊断（不作放行门）
- max **3.427e-6**（05_limit_wz_over）；05_limit_combo 3.160e-6；03_stop/04_zero_cross_vx/08_remote 2.034e-6。
- 原因：frame0 的 8.9e-8 级 action 差异经 5 Hz target 滤波回填 `prev_target`，闭环累积；不是推理/观测构造 bug。
- 首次分歧：frame0 obs 相同；frame1 obs idx52（上一拍 leg target）。

### 2.3 最终 LowCmd
- 12 场景控制器目标：q max **4.8e-7**、dq max **9.6e-7**、kp/kd/mode max 0、first>1e-6 None。
- 离线 audit 2 帧：max slot/16-axis error **0.0**。
- **缺口**：完整 16 轴 FSM/DDS LowCmd 逐帧覆盖尚未完成；2 帧 audit 不能代表全 FSM。

### 2.4 50 Hz
- `Controller::step`：p50 287.5 µs、p95 360.5 µs、max 1130.6 µs。
- controller-only wall-clock 两次 run 不一致，必须都保留：
  - run/lead：p50 19.995 ms、p95 20.055 ms、max 20.94 ms、>20 ms **235/600**（39.2%）；
  - 原始 `wallclock_frames.csv`（SHA `0B9B0CC5…`，600 帧）：step p50 283.75 µs/p95 352.70/max 590.47；period p50 19.968 ms/p95 20.028/max 21.009 ms、>20 ms **83/600**（13.8%）。
- 两次 run 差异说明 controller-only 时序波动大；**DDS/publisher 50 Hz 未测**；controller-only 不能当作发布 50 Hz 通过。

## 3. 发布前必须补齐
1. **完整 FSM/DDS LowCmd 逐帧**：16 轴 q/dq/kp/kd、mode、映射、启动/交接/停车/过零/限幅/guard/remote timeout；覆盖率与 SHA。
2. **DDS/publisher 50 Hz**：p50/p95/max、>20 ms 次数、最坏帧；与 controller-only 分开记录。
3. **M0**：策略实际消费的观测年龄 vs 指令生效延迟。
4. **实际加载文件**：CPRO ARM64 binary/config/map/ONNX SHA，不能只用 x86 参考。
5. **owner 最终放行**。

## 4. 当前放行状态
- **未发布、未启动 publisher、未发 LowCmd、未切周一包。**
- CPRO 离线部署目录 `/root/as2w_deploy_v4x` 可复现 CTest 9/9 + offline-audit PASS。
- 原门 `TARGET ≤1e-6` 仍 FAIL；方案 2 已签但发布未放行。

## 5. 证据路径
- T8：`/root/gpufree-data/as2w-runs/retrain-dr3c-20261122/T8/EXTENDED_REPLAY_REPORT.md`
- CPRO 报告：`CPRO_V4X_DEPLOY_ATTEMPT-20261123.md`
- CPRO 离线部署：`/root/as2w_deploy_v4x/`
- fixed-input/closed-loop 数据：`fixtures_ext/compare_summary.json`、`fixed_input_arm/`、`lowcmd_*.csv`
- 50 Hz：`timing/` + CPRO controller-only wall-clock CSV
