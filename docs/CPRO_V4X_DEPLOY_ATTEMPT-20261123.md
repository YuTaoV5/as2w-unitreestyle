# CPRO v4x 部署尝试记录（2026-11-23 10:30）

> 结论：**CPRO 连接、取包、ARM64 构建与 CTest 均通过；ARM64↔x86 replay 对拍 FAIL（1.341e-6 > 1e-6），按门槛停止，未发布 LowCmd、未让机器人动作。**

## 1. 包确认
- 来源：训练服务器 `/root/as2w/retrain-wheeldiff-v41-20261121/deploy_v4x/pkg/`
- 主 policy：`pkg/policy.onnx` SHA **`63f1c5b6…`** = **方案 A（DR2b seed42 final，model_799）**
- deploy.yaml：`531f07d0…`
- 包 tar：`as2w_deploy_v4x_pkg_20261123.tar.gz` SHA `8e156a66…`（5.0 MB）
- 已传 CPRO：`/data/ubuntu_in_hm/ubuntu_rootfs/root/as2w_v4x_20261123_pkg.tar.gz`

## 2. CPRO 连接与环境
- HDC USB target `0123456789ABCDEF`，HarmonyOS root
- Ubuntu 22.04.5 rootfs：`/data/ubuntu_in_hm/ubuntu_rootfs`
- g++ 11.4.0、CMake 3.22.1、aarch64 原生
- CPRO 网口：`eth0 192.168.123.100/24` 已 up；`wlan0 192.168.10.77/24`

## 3. staging 构建
- 目录：`/root/as2w_deploy_v4x_staging_20261123`
- 用旧源码 `/root/as2w_deploy` + 包内 `cpp_patch`：
  - 新增 `deploy/robots/as2w/unitree58v4`（controller/replay/CMake）
  - 顶层 CMake 改为该子目录
  - common publisher include/namespace 改为 `unitree58v4`
  - `models/unitree58-v4x/{policy.onnx,deploy.yaml}` 放 A
- 构建：**rc=0**
- CTest：**9/9 PASS**（含 6 fixtures、controller、unitree58-startup-audit、unitree58-v4x-startup-audit）
- 日志：`/root/as2w_deploy_v4x_build_20261123.log` SHA `68d2501f…`

| ARM64 产物 | SHA256 |
|---|---|
| `as2w_unitree58v4_lp5_publish` | `22ed3135…` |
| `as2w_unitree58v4_lp5_replay` | `6da02f9f…` |
| `models/unitree58-v4x/policy.onnx` | `63f1c5b6…` |
| `models/unitree58-v4x/deploy.yaml` | `531f07d0…` |

## 4. ARM64↔x86 replay 对拍
- fixture：`/root/as2w/retrain-wheeldiff-v41-20261121/dr2_pipeline/results/fixture.txt`，300 帧
- x86 参考：包内 `as2w_unitree58v4_lp5_replay`（服务器 x86，ORT 1.22.0）
- ARM64：CPRO staging 重建的 `as2w_unitree58v4_lp5_replay`
- 两侧相同 `deploy.yaml` + `policy.onnx` + fixture

| 输出 | max_abs_err | 门限 | 结果 |
|---|---:|---:|---|
| `UNITREE58_OBS` | 6.55e-7 | ≤1e-6 | ✅ |
| `UNITREE58_TARGET` | **1.341e-6** | ≤1e-6 | ❌ |
| `UNITREE58V4_WHEELCMD` | 0.0 | ≤1e-6 | ✅ |

- `TARGET` 最大差异在 action 第 3 列、frame 69：ARM64 `0.203427181` / x86 `0.20342584`；leg_q 2.4e-7、wheel_dq 4.8e-7。
- 两侧 `LEG_ALPHA=0.466512 WHEEL_ALPHA=0.466512`，`MAX_LEG_STEP` 0.078097 vs 0.0780969。
- **判据要求 TARGET ≤1e-6；当前超 0.341e-6，不能判 PASS。**

### 4b. 同源 x86 复建（排除“x86 包陈旧”）
- 在服务器上用**同一 staging 源码/同一补丁**重建 x86：`/root/as2w_deploy_v4x_x86_staging_20261123`
- CTest 9/9 PASS；replay 输出 SHA `86012ac9…`，与包内 x86 二进制输出**逐字节一致**。
- 因此 1.341e-6 不是“x86 包陈旧”，而是 **ARM64 与 x86 的 ORT/浮点实现差异**；按硬门槛仍 **FAIL**。
- 未放宽阈值、未 `--i-confirm-lowcmd-publish`、未发 LowCmd、机器人未动作。

### 4d. 输入差异 vs 推理差异（2026-11-23 10:40，已拆开）
- 诊断回放新增 frame69/275 的 **ORT 前 58×float32 obs 原始字节**与 **12×float32 raw action**：
  - frame69：obs max_abs **2.68e-7**（idx50 = 上一拍 leg target），raw action max_abs **1.341e-6**；
  - frame275：obs max_abs **3.58e-7**，raw action max_abs **1.311e-6**。
- 固定输入（同一个 ONNX、两端各跑两份 obs）：
  - x86 输入：ARM64 vs x86 raw action max **3.87e-7**；
  - ARM64 输入：ARM64 vs x86 raw action max **4.17e-7**；
  - 两侧都 **<1e-6** → **推理本身不是超门原因**。
- 同 runtime 不同输入：
  - x86 runtime：arm-input vs x86-input raw action max **1.326e-6**；
  - ARM64 runtime：max **1.401e-6**；
  - → **差异主要随输入走**。
- 完整 fixture 固定输入（300 帧，x86 obs 序列喂两端）：raw action max **4.17e-7**，也 <1e-6。
- 首次分歧：
  - frame0 obs **完全相同**；frame0 target max 8.9e-8；
  - frame1 obs max 4.5e-8，idx52 = 上一拍 leg target（idx 41+11）；frame1 target max 4.8e-7；
  - frame2 obs idx45，之后逐步放大；
  - 结论：**frame0 的 8.9e-8 级 action 差异，经 5 Hz target 滤波回填到 `prev_target` obs，闭环累积成 1.3e-6**；不是观测构造 bug，也不是 ORT 推理超门。

### 4e. 验收门建议
- **raw action 固定输入/开环**：实测 ≤4.17e-7，可保留 **≤1e-6** 门。
- **完整闭环 fixture 的 raw action**：1.341e-6，超 1e-6；属于闭环状态累积，非推理差异。
- **最终控制输出**：leg_q max **2.4e-7**、wheel_dq max **4.8e-7**，均 <1e-6。
- 建议申请**分开门限**：
  - 固定输入 raw action ≤1e-6；
  - 完整闭环 raw action 暂按 **≤2e-6 提审**（最终值由扩展回放/签字定）；
  - 最终 LowCmd 字段仍 ≤1e-6。
- 补测项：最终 LowCmd 字段、限幅、FSM 启动/停车/过零/边界分支；CPRO 50 Hz 时延。当前尚未发布 LowCmd。
### 4c. 编译 flags / ORT 图优化（无改善）
- 两侧 ORT 均 **1.22.0**；g++ 均 **11.4.0**；默认 flags 均 `-O3 -DNDEBUG`。
- ARM64 编译变体（同 fixture、同 policy）：
  - `-O2`：输出与 `-O3` 逐字节相同，TARGET max **1.341e-6**；
  - `-O3/-O2 -ffp-contract=off -fno-fast-math`：TARGET max 反而 **1.639e-6**（更差）；
  - `OrtRunner` graph optimization `ORT_ENABLE_ALL / BASIC / DISABLE_ALL`：输出全部与默认 **逐字节相同**（SHA `f34f309c…`）。
- 同源 x86 重建输出与包内 x86 输出逐字节相同（SHA `86012ac9…`）。
- 结论：编译 flags 和图优化无法解释/消除差异；真正的超门原因是 **闭环输入累积**（见 4d），不是固定输入推理。

## 5. 止损
- **未发布 LowCmd**；未使用 `--i-confirm-lowcmd-publish`；未运行 publisher；机器人未动作。
- 未启用 publisher/未发 LowCmd；未切换周一包。
- **已按 AS2-W_CPRO 规范建立独立离线部署目录** `/root/as2w_deploy_v4x`（旧 `/root/as2w_deploy` 未动）；该目录是离线验收包，不是发布版本。
- **已定位**：固定输入 raw action ≤4.17e-7（推理过门）；完整 fixture 超门来自闭环 `prev_target`/5 Hz 滤波状态累积（frame0 8.9e-8 → frame69 1.3e-6）。
- 当前仍不能发布：需 owner 书面决定“固定输入门 ≤1e-6 + 完整闭环 raw action 另行门限（2e-6 提审）”，或继续做参考推理/状态量化实验。**不得自行放宽 1e-6，也不得用 x86 代替 ARM64。**

## 7. CPRO 独立离线部署布局（参考 AS2-W_CPRO）
- 目录：`/root/as2w_deploy_v4x/`（独立，不覆盖旧 `/root/as2w_deploy`）
- 结构：
  - `models/unitree58-v4x/{policy.onnx,deploy.yaml}` — A `63f1c5b6…` / `531f07d0…`
  - `config/unitree58_measured.map` — `a2f29634…`
  - `build-v4x/deploy/robots/as2w/unitree58v4/` — ARM64 publish/replay
  - `tools/v4x_field.sh` — `status|audit|replay|sha`，**无 publisher 启动入口**
  - `deploy_logs/` — CTest、offline audit、build、SHA 日志
  - `V4X_DEPLOY_MANIFEST.json` + `.sha256`
- 验收结果：
  - CTest **9/9 PASS**（最终目录重配后自包含）
  - `--offline-audit` **PASS**
  - publish `22ed3135…`、replay `6da02f9f…`
  - **未启动 publisher、未发 LowCmd**
- Manifest 明确记录：`cross_arch_original_gate=FAIL 1.341e-6 > 1e-6`、`gate_draft=framework signed, release NOT effective`、`lowcmd_published=false`。

## 6. 证据
- ARM64 replay：`/root/v4x_arm64_replay.out` SHA `f34f309c…`、`.err` SHA `8a87f926…`
- x86 replay：`/tmp/v4x_x86_replay.out` SHA `86012ac9…`、`.err` SHA `1cd1c008…`
- 构建日志 SHA `68d2501f…`
- CPRO staging：`/root/as2w_deploy_v4x_staging_20261123/`
