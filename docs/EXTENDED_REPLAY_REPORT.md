# T8 扩展回放 / LowCmd / CPRO 时延证据报告

生成：2026-11-23 10:56 +08（Asia/Shanghai）

> **当前原门仍 FAIL（完整闭环 TARGET 1.341e-6 > 1e-6）；三层门框架已签但发布未生效；2e-6/4e-6 均只是提审值，不得作为放行依据。**
> 未启动 publisher、未发 LowCmd、未部署、未切周一包。

## 1. 证据来源与 SHA（来源标签明确）

| 项 | 值 |
|---|---|
| ARM64 来源 | **CPRO 设备 `0123456789ABCDEF`，Windows HDC USB 通道，CPRO 原生 ARM64 构建/运行** |
| CPRO canonical 目录 | `/root/as2w_deploy_v4x`（`V4X_DEPLOY_MANIFEST.json`） |
| ARM64 replay | `6da02f9fc82018a775d2447bd9cf5e7e0166f792c77f19781159b770892745ea` |
| ARM64 publish/audit | `22ed313587d5a1e3cc1b0822554b9bd679a5dcf6c47006545cbe63adc8708bb9` |
| ONNX A | `63f1c5b6a6bf5c7fd9fdcaa09cce93deb5aa5c81dc97c96e6c0dd72bf7ab64ee` |
| deploy.yaml | `531f07d0dbbf9ccc36d932db90e866c4db101d63e6e221b94dae0d4f358db123` |
| joint map | `a2f2963462bb566aec5eb016756d0e67591a5440e31c19cea627d518e01fe1bc` |
| x86 来源 | **x86 训练服务器 staging 重建**（仅参考，不冒充 ARM64） |
| x86 replay | `5c1848470619ff055745e81ecd25b29700ea614d9047ad9559af002d0253c599` |
| x86 publish/audit | `772260deb1da818dc69a7945ca478dc2a465ef5371fbebdf85b74e0e06b97058` |
| fixtures | 12 个，逐文件 SHA 见 `compare/compare_summary.json → provenance.fixture_sha256` 与 `fixtures/fixtures.sha256` |
| 整包证据 SHA | `T8_EVIDENCE_SHA256.txt` |

staging replay `4d090df6…` 与 canonical replay `6da02f9f…` 对全部 12 fixture 输出逐字节相同；本报告采用 canonical ARM64 输出。

## 2. 扩展场景逐帧结果（ARM64 vs x86）

每场景保存了逐帧原始输出：`arm64_out/<fixture>.out`、`x86_out/<fixture>.out`；`compare/compare_table.md` 为汇总。

| fixture | 类别 | 帧 | obs max (帧,维) | obs 首个>1e-6 | action max (帧,维) | action 首个>1e-6 | target max (帧,维) | leg_q max | wheel_dq max | wheelcmd max |
|---|---|---|---|---|---|---|---|---|---|---|
| 01_startup_zero | 严格首飞包络（vx≤.10,wz≤.10,vy=0） | 50 | 6.85e-07 (0.72,44) | 无 | 1.78e-06 (0.7,action3) | [0.08, 3, 'action3'] | 1.78e-06 (0.7,3) | 1.78e-07 | 0 | 0 |
| 02_handoff_endpoint | 严格首飞包络（vx≤.10,wz≤.10,vy=0） | 60 | 6.85e-07 (0.72,44) | 无 | 1.93e-06 (1.06,action3) | [0.08, 3, 'action3'] | 1.93e-06 (1.06,3) | 1.78e-07 | 0 | 0 |
| 03_stop_to_zero | 严格首飞包络（vx≤.10,wz≤.10,vy=0） | 100 | 6.71e-07 (1.42,44) | 无 | 2.03e-06 (0.32,action3) | [0.26, 0, 'action0'] | 2.03e-06 (0.32,3) | 1.79e-07 | 0 | 0 |
| 04_zero_cross_vx | 严格首飞包络（vx≤.10,wz≤.10,vy=0） | 80 | 6.63e-07 (1.48,44) | 无 | 2.03e-06 (0.32,action3) | [0.26, 0, 'action0'] | 2.03e-06 (0.32,3) | 1.64e-07 | 2.98e-08 | 0 |
| 04_zero_cross_wz | yaml 包络/摆角诊断 | 80 | 4.17e-07 (0.86,41) | 无 | 1.2e-06 (1.34,action4) | [0.84, 0, 'action0'] | 1.2e-06 (1.34,4) | 1.2e-07 | 1.2e-07 | 0 |
| 05_limit_combo_over | 超包络诊断（1.0/5.0，非部署输入） | 40 | 1.76e-06 (0.56,52) | [0.28, 50, 'obs50'] | 3.16e-06 (0.26,action9) | [0.04, 9, 'action9'] | 3.16e-06 (0.26,9) | 4.8e-07 | 0 | 0 |
| 05_limit_vx_over | 超包络诊断（1.0/5.0，非部署输入） | 40 | 3.58e-07 (0.14,43) | 无 | 8.94e-07 (0.56,action3) | 无 | 9.6e-07 (0.04,24) | 1.2e-07 | 9.6e-07 | 0 |
| 05_limit_wz_over | 超包络诊断（1.0/5.0，非部署输入） | 40 | 1.3e-06 (0.42,52) | [0.42, 52, 'obs52'] | 3.43e-06 (0.4,action11) | [0.08, 11, 'action11'] | 3.43e-06 (0.4,11) | 3.5e-07 | 0 | 0 |
| 06_single_frame_step | 严格首飞包络（vx≤.10,wz≤.10,vy=0） | 41 | 6.56e-07 (0.26,41) | 无 | 1.83e-06 (0.48,action3) | [0.08, 3, 'action3'] | 1.83e-06 (0.48,3) | 1.79e-07 | 0 | 0 |
| 06_single_frame_wz | yaml 包络/摆角诊断 | 41 | 6.56e-07 (0.26,41) | 无 | 1.37e-06 (0.2,action3) | [0.08, 3, 'action3'] | 1.37e-06 (0.2,3) | 1.49e-07 | 0 | 0 |
| 07_guard_leg_excursion | yaml 包络/摆角诊断 | 30 | 6.55e-07 (0.58,52) | 无 | 1.16e-06 (0.56,action11) | [0.56, 11, 'action11'] | 1.16e-06 (0.56,11) | 2.4e-07 | 0 | 0 |
| 08_remote_timeout_zero | 严格首飞包络（vx≤.10,wz≤.10,vy=0） | 60 | 6.48e-07 (0.34,44) | 无 | 2.03e-06 (0.32,action3) | [0.26, 0, 'action0'] | 2.03e-06 (0.32,3) | 1.79e-07 | 0 | 0 |

### 2.1 首个 >2e-6 位置（闭环 raw action）

| fixture | closed action max | 首个>2e-6 (frame,field,value) |
|---|---|---|
| 01_startup_zero | 1.78e-06 | 无 |
| 02_handoff_endpoint | 1.93e-06 | 无 |
| 03_stop_to_zero | 2.03e-06 | [16, 3, 'action3', 2.033999999997982e-06] |
| 04_zero_cross_vx | 2.03e-06 | [16, 3, 'action3', 2.033999999997982e-06] |
| 04_zero_cross_wz | 1.2e-06 | 无 |
| 05_limit_combo_over | 3.16e-06 | [12, 9, 'action9', 2.2790000000216803e-06] |
| 05_limit_vx_over | 8.94e-07 | 无 |
| 05_limit_wz_over | 3.43e-06 | [8, 10, 'action10', 2.049840000000608e-06] |
| 06_single_frame_step | 1.83e-06 | 无 |
| 06_single_frame_wz | 1.37e-06 | 无 |
| 07_guard_leg_excursion | 1.16e-06 | 无 |
| 08_remote_timeout_zero | 2.03e-06 | [16, 3, 'action3', 2.033999999997982e-06] |

严格首飞包络中 `03_stop_to_zero`、`04_zero_cross_vx`、`08_remote_timeout_zero` 的 action max 均 2.034e-6，超提审 2e-6 0.034e-6；超包络诊断最高 3.427e-6（`05_limit_wz_over`）。

## 3. >2e-6 场景的 fixed-input 归因

对每个场景，把 x86 replay 的 58 维 obs 逐帧序列**固定输入**到同一 ONNX（x86 参考 + CPRO ARM64），再与闭环 raw action 对比：

| fixture | closed action max | fixed-input max | fixed 首个>1e-6 | fixed_input 门 1e-6 | 归因 |
|---|---|---|---|---|---|
| 01_startup_zero | 1.78e-06 | 4.1e-07 | 无 | PASS | fixed_input PASS<=1e-6 -> closed-loop state accumulation |
| 02_handoff_endpoint | 1.93e-06 | 4.1e-07 | 无 | PASS | fixed_input PASS<=1e-6 -> closed-loop state accumulation |
| 03_stop_to_zero | 2.03e-06 | 3.65e-07 | 无 | PASS | fixed_input PASS<=1e-6 -> closed-loop state accumulation |
| 04_zero_cross_vx | 2.03e-06 | 3.65e-07 | 无 | PASS | fixed_input PASS<=1e-6 -> closed-loop state accumulation |
| 04_zero_cross_wz | 1.2e-06 | 3.58e-07 | 无 | PASS | fixed_input PASS<=1e-6 -> closed-loop state accumulation |
| 05_limit_combo_over | 3.16e-06 | 7.75e-07 | 无 | PASS | fixed_input PASS<=1e-6 -> closed-loop state accumulation |
| 05_limit_vx_over | 8.94e-07 | 3.2e-07 | 无 | PASS | fixed_input PASS<=1e-6 -> closed-loop state accumulation |
| 05_limit_wz_over | 3.43e-06 | 9.32e-07 | 无 | PASS | fixed_input PASS<=1e-6 -> closed-loop state accumulation |
| 06_single_frame_step | 1.83e-06 | 4.1e-07 | 无 | PASS | fixed_input PASS<=1e-6 -> closed-loop state accumulation |
| 06_single_frame_wz | 1.37e-06 | 4.1e-07 | 无 | PASS | fixed_input PASS<=1e-6 -> closed-loop state accumulation |
| 07_guard_leg_excursion | 1.16e-06 | 3.58e-07 | 无 | PASS | fixed_input PASS<=1e-6 -> closed-loop state accumulation |
| 08_remote_timeout_zero | 2.03e-06 | 4.25e-07 | 无 | PASS | fixed_input PASS<=1e-6 -> closed-loop state accumulation |

**结论：所有 12 个场景 fixed-input max = 9.318e-7 < 1e-6（PASS；多数 ≤4.25e-7）；闭环 >2e-6 来自 `prev_target`/5Hz 滤波状态在闭环中的累积，不是单帧推理或限幅本身。**
固定输入复跑状态：`fixed_input/*.f32`（obs 序列）、`fixed_input_x86/*.f32`、`fixed_input_arm/*.f32` 及各自 SHA。

## 4. LowCmd / FSM / 守卫 / 远程超时

### 4.1 offline audit（ARM64 canonical vs x86）

- audit frames: x86=['neutral_zero_torque', 'passive_kd3'] / ARM64=['neutral_zero_torque', 'passive_kd3']（`neutral_zero_torque`、`passive_kd3`）
- **16 轴 q/dq/kp/kd/mode max|Δ| = 0.0**；全 slot max|Δ|=0.0；first mismatch=None；raw_hex 完全相同帧=2
- ARM64 audit 头部：`UNITREE58V4_MODEL input=obs[N,58] output=act[N,12] float32` / `FSM CONTRACT A=FixedStand B=PolicyHandoff Y=FixSquat X=EmergencyDamping` / `GUARD INJECTION PASS: safe-envelope-200f warn@5->Brake hard@3->Emergency motorstate=hard flicker-reset remote-timeout@200ms no-auto-rewalk slew-0.5s-in/0.25s-out applied==policy-command`
- x86 audit 头部：`UNITREE58V4_MODEL input=obs[N,58] output=act[N,12] float32`
- audit 断言覆盖：FixedStand→PolicyHandoff→PolicyWalk 端点、FixSquat、EmergencyDamping、remote 映射/slew、guard warn/hard、motorstate hard、remote-timeout@200ms no-auto-rewalk、applied==policy-command、wheel/leg bypass/jitter-step；两侧 PASS。

### 4.2 全部扩展 fixture 的逐帧 LowCmd 重建比较

按 deploy.yaml（default/kp/kd）+ map（slot/sign/zero）把每帧 replay TARGET 重建为 16 轴 LowCmd（policy-walk 状态）：q=default+leg target，dq=wheel target，kp=55/0，kd=2/2，mode=1；逐帧 x86 vs ARM64。

| fixture | 帧 | q max|Δ| | dq max|Δ| | kp max|Δ| | kd max|Δ| | mode max|Δ| | 首个>1e-6 |
|---|---|---|---|---|---|---|---|
| 01_startup_zero | 50 | 1.78e-07 | 0 | 0 | 0 | 0 | 无 |
| 02_handoff_endpoint | 60 | 1.78e-07 | 0 | 0 | 0 | 0 | 无 |
| 03_stop_to_zero | 100 | 1.79e-07 | 0 | 0 | 0 | 0 | 无 |
| 04_zero_cross_vx | 80 | 1.64e-07 | 2.98e-08 | 0 | 0 | 0 | 无 |
| 04_zero_cross_wz | 80 | 1.2e-07 | 1.2e-07 | 0 | 0 | 0 | 无 |
| 05_limit_combo_over | 40 | 4.8e-07 | 0 | 0 | 0 | 0 | 无 |
| 05_limit_vx_over | 40 | 1.2e-07 | 9.6e-07 | 0 | 0 | 0 | 无 |
| 05_limit_wz_over | 40 | 3.5e-07 | 0 | 0 | 0 | 0 | 无 |
| 06_single_frame_step | 41 | 1.79e-07 | 0 | 0 | 0 | 0 | 无 |
| 06_single_frame_wz | 41 | 1.49e-07 | 0 | 0 | 0 | 0 | 无 |
| 07_guard_leg_excursion | 30 | 2.4e-07 | 0 | 0 | 0 | 0 | 无 |
| 08_remote_timeout_zero | 60 | 1.79e-07 | 0 | 0 | 0 | 0 | 无 |

**LowCmd 汇总：12 fixture 逐帧 policy-walk 16 轴 q max|Δ| = 4.8e-7、dq max|Δ| = 9.6e-7、mode/kp/kd max|Δ| = 0 → PASS（≤1e-6）。** 逐帧原始比较见 `attribution/lowcmd_<fixture>.csv`（每帧 max q/dq diff）。

**缺口（必须明确）**：offline audit 只导出 `neutral_zero_torque`、`passive_kd3` 两帧；FixedStand / PolicyHandoff / Brake / ReturnStand 的逐帧完整 LowCmd 尚未由 publisher 离线 dump（需要另建诊断 build），因此“全 FSM 状态逐帧 LowCmd”未完成。未启动 publisher、未发 LowCmd。

## 5. CPRO 50Hz / 时延（三层严格区分 + 原始件）

必须区分三类量，**不得把前两者当发布 50Hz 证据**：

| 层级 | 测量 | step p50/p95/max | period p50/p95/max | >20ms | 来源/原始件 |
|---|---|---|---|---|---|
| A. step compute（本任务 harness） | `Controller::step` 含 ORT，不含 sleep/IO/DDS | 23–34µs / 45–103µs / 588µs | — | 0 | CPRO ARM64；`timing/timing_summary.json`、`timing/*.err` 原始 TIMING_US |
| B. wall-clock loop（**原始件 run_wallclock3**） | 600 帧 controller loop，含本机 sleep/调度，不含 DDS/publisher | **283.75µs / 352.70 / 590.47** | **19.968ms / 20.028 / 21.009** | **83/600 (13.8%)** | CPRO ARM64；原始件 `timing/wallclock_frames.csv`（SHA 见下）、`timing/wallclock_loop.cpp`、`timing/run_wallclock3.sh` |
| C. lead 外部补充 | 另一次测量（自称 lead message） | 287.5µs / 360.5µs / 1130.6µs | 19.995ms / 20.055ms / 20.94ms | 235/600 | **无原始件；仅 lead 外部补充，不作独立证据**；见 `timing/lead_measurements.json` |
| D. **DDS/publisher 真实 50Hz** | **未测** | — | — | — | **缺口**：需启动 publisher 实测，本任务禁止 |

原始 `wallclock_frames.csv` 复算（numpy，600 帧）：step p50 283.75µs/p95 352.70/max 590.47µs；period p50 19.968ms/p95 20.028/max 21.009ms；**83/600 帧 >20ms（13.8%）**。
原始件 `timing/wallclock_frames.csv` sha256 **`0b9b0cc53aa7a87a318953db4ab14525d0ba38a726f554152e94696733ae30e7`**；本地镜像 `X:\Items\as2w\T8_EVIDENCE_20261123\timing\wallclock_frames.csv`。
B 与 C 是**两次 run**（83 vs 235 帧 >20ms），存在明显 run-to-run 波动；两者都表明 controller-only 已贴 20ms 边，均**不能判 50Hz PASS**，也不能替代 DDS/publisher 实测。

A 的逐 fixture 明细：

| fixture | 帧 | p50 µs | p95 µs | max µs | >20ms 次数 | 最坏帧（全帧） |
|---|---|---|---|---|---|---|
| 02_handoff_endpoint | 60 | 34.0 | 103.3 | 418.0 | 0 | f0 (418.0µs) |
| 03_stop_to_zero | 100 | 34.0 | 54.2 | 327.0 | 0 | f0 (327.0µs) |
| 08_remote_timeout_zero | 60 | 23.0 | 90.5 | 588.0 | 0 | f19 (588.0µs) |
| fixture | 300 | 24.0 | 59.1 | 303.0 | 0 | f0 (303.0µs) |

结论：A 为 step compute；**B 是 run_wallclock3 的原始 CSV 复算（83/600 >20ms）**，**C 是 lead 外部补充的另一次 run（235/600，无原始件）**；两次 run 存在明显 run-to-run 波动，均说明 controller-only 贴 20ms 边；**D（DDS/publisher 真实 50Hz）未完成，是缺口**，不以 A/B/C 替代，也不能判 50Hz PASS。

## 6. M0 / 加载（离线部分）

- ARM64 replay/publish/ONNX/yaml/map SHA 见 §1；`file`=ELF64 aarch64、`ldd` 显示 ORT 1.22.0；canonical manifest 哈希与实测一致；CTest 9/9 与 offline audit PASS 为 CPRO 侧记录。
- **未做**：真实 M0 加载/启动/发 LowCmd 核验（禁止）；publisher 未启动。

## 7. 结论 / 建议（owner 决定，不生效）

1. 固定输入 raw action：ARM64 vs x86 **4.17e-7 ≤1e-6 PASS**；12 场景 fixed-input 最大 **9.318e-7 PASS**；obs 序列 6.56e-7 PASS。
2. 完整闭环原门 TARGET：**1.341e-6 >1e-6 FAIL**（CPRO 原始 fixture），扩展场景 max **3.427e-6**（`05_limit_wz_over`）；2e-6 提审值不通过（03/04vx/08 均 2.034e-6，超包络 3.16/3.43e-6）；**原门仍 FAIL**。
3. >2e-6 归因：fixed-input 全部 ≤9.318e-7，**闭环状态累积**（`prev_target`/5Hz 滤波）；不是单帧推理，也不是限幅/守卫本身；首次 >2e-6 位置见 §2.1。
4. LowCmd：audit 2 帧 16 轴 0 差异；**12 fixture 逐帧 policy-walk LowCmd q=4.8e-7/dq=9.6e-7/mode/kp/kd=0 → PASS**；**全 FSM（FixedStand/Handoff/Brake/ReturnStand）逐帧 LowCmd 缺口**（offline audit 未 dump）。
5. 时延：A 为 step compute；B 为 **run_wallclock3 原始 CSV**（step p50 283.75µs/p95 352.70/max 590.47µs；period p50 19.968ms/p95 20.028/max 21.009ms；**83/600 >20ms=13.8%**）；C 为 lead 外部补充（235/600 >20ms，**无原始件**）；两次 run 波动 83 vs 235，均说明 controller-only 贴边，**不能判 50Hz PASS**；**DDS/publisher 真实 50Hz = 未测缺口**（禁止启动 publisher）。
6. 建议（仅提审记录，不生效）：① owner 可选 **“固定输入 1e-6 + final LowCmd 1e-6，闭环 raw action 仅作诊断”**（不改变原门 FAIL 事实）；②或由 owner 书面抬门（如 4e-6 提审），但不得覆盖 LowCmd/50Hz 缺口；③控制器状态确定性更新/量化可降低闭环累积，但改控制器版本需重新全量验收；④仅 fixed-input+final LowCmd 门不覆盖完整闭环风险。**不自行放宽，由 owner 决定。**

证据目录：`/root/gpufree-data/as2w-runs/retrain-dr3c-20261122/T8/`；SHA 清单：`T8_EVIDENCE_SHA256.txt`。

