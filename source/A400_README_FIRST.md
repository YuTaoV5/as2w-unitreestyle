# AS2-W A_400（DR2b seed42 model_400）— 完整部署包

> 本包是 **A_400 专属自包含部署包**：A_400 ONNX + A/v4x ARM64 部署骨架 + 训练资产。
> **不是已放行版本**：A_400 仅通过 MuJoCo 内八姿态筛选；CPRO C++ audit、实机、B gate、owner 放行均未完成。**release_authorized=false**。

## 0. 先读
- 目标机：CPRO（HarmonyOS host + Ubuntu 22.04 rootfs），**ARM64**。
- 候选：`A_400` = DR2b seed42，`model_400.pt`，400 步。
- 选择理由：在 T18 内八筛选中，与官方 ai 站姿 `toe_in=5.54°` 偏差最小（A_400 6.53°，Δ0.98°），且不是靠收窄轮距实现。
- 策略合同：58 obs / 12 act，内嵌 tanh，腿 **55/2**；轮 vx/wz 差速、track 0.44、eta 0.415、clamp 12。
- 起身/站立：腿 **200/4**、轮 0/0.5；**禁止把 200/4 带进策略阶段**。
- **禁止**在 `motion-mode name=ai` 时启动 publisher；**禁止**同时运行两个 publisher。

## 1. 三步安装（CPRO rootfs）
```bash
# 1) 传包/解压/校验
hdc file send AS2W_A400_DEPLOY_20261123.zip /data/local/tmp/
cd /data/local/tmp
unzip -o AS2W_A400_DEPLOY_20261123.zip
cd AS2W_A400_DEPLOY_20261123
sha256sum -c SHA256SUMS

# 2) 安装到独立目录并离线 audit（不启动 publisher、不发 LowCmd）
cd /data/local/tmp
bash AS2W_A400_DEPLOY_20261123/deployment/scripts/install_a400_cpro.sh /root/as2w_deploy_a400_s42_400

# 3) 只有现场安全确认后才做 zero-B
#    先确认 eth0 UP、LowState 有包、motion-mode name=ai、无 active publisher
#    现场操作员执行一次 ReleaseMode；motion-mode name= 为空
cd /root/as2w_deploy_a400_s42_400
bash scripts/run_zero_b_guarded_a400.sh
```
安装脚本会校验 SHA、复制到独立目录、执行 `--offline-audit`，**不会**启动 publisher。

## 2. 目录
```text
AS2W_A400_DEPLOY_20261123/
├── README_FIRST.md
├── VERSION
├── SHA256SUMS
├── deployment/
│   ├── bin/                 ARM64 publish/replay
│   ├── models/unitree58-v4x/A400 policy.onnx + deploy.yaml + export.json
│   ├── config/              measured map（hip sign=+1）
│   ├── scripts/             install / guarded zero-B / safe exit / rollback A
│   ├── tools/               v4x_field.sh
│   ├── source/              C++ 源码快照（含 FixedStand 200/4/hip 修正）
│   ├── manifest/            A400 manifest
│   ├── logs/                参考 A 基线 audit/CTest
│   └── rollback_A/          A_799 回滚 policy
├── training/
│   ├── checkpoints/         a400_s42_model_400.pt
│   ├── configs/             resolved/run/best/domain/reward
│   ├── source/              environment/task/control/run script
│   ├── export/              export_onnx.py/parity_v4x.py
│   ├── eval400/             flat/gait gate/run/eval
│   └── evidence/            T18 内八筛选/60s 报告
└── docs/
    ├── DEPLOY_CPRO.md
    ├── A400_SELECTION_AND_LIMITS.md
    ├── TRAINING_PROVENANCE.md
    └── BUILD_ARM64.md
```

## 3. 关键 SHA
| 对象 | SHA256 |
|---|---|
| A400 policy.onnx | `acccdedba62312adc15732c8048ff49b2a5e9e12ceb91df730f2d8add3a1b715` |
| A400 checkpoint | `88cff1a264ad3ffe5e7b9fc61ddee4998d6101a1f30fa68d0218696e5bbf4802` |
| deploy.yaml | `531f07d0dbbf9ccc36d932db90e866c4db101d63e6e221b94dae0d4f358db123` |
| measured map | `f33fcae4d72e353efdb3d9b727b131fa72965890b9c5f97be14b1d58c3de598c` |
| ARM64 publish | `7418070199700fa7ffc503301c41db7c79c4abf992edde4b601fa632dcf175fc` |
| ARM64 replay | `6da02f9fc82018a775d2447bd9cf5e7e0166f792c77f19781159b770892745ea` |
| fallback A_799 policy | `63f1c5b6a6bf5c7fd9fdcaa09cce93deb5aa5c81dc97c96e6c0dd72bf7ab64ee` |

## 4. 验证状态（必须如实）
- **ONNX export parity：PASS**，torch↔ORT max abs err **4.172e-6**（limit 2e-5）。
- **C++ offline audit on A_400：PENDING**。打包时 CPRO/HDC 不可用；安装脚本会在目标机执行，只有看到 `AUDIT PASS` 才能继续。
- **参考 CTest 9/9** 来自 A 基线（A/v4x 同骨架）；A_400 没有在 CPRO 重新跑 CTest。
- **实机：未测试**；zero-B、B gate、handoff 未在 A_400 上验证。
- **binary/source 一致性**：`bin/` 是 A/v4x 参考 ARM64 构建；`source/` 已含 FixedStand 200/4 参数修正，但打包环境无法重编译。严格部署请按 `docs/BUILD_ARM64.md` 在 CPRO 重编译并替换二进制，再 audit。
- **release_authorized=false**。

## 5. 训练资产
- checkpoint：`training/checkpoints/a400_s42_model_400.pt`（400 步）
- 配置：resolved_config、run、best、domain、reward、control.toml
- 源码：environment.py、task.py、run_wheeldiff_v41.sh
- 导出：export_onnx.py、parity_v4x.py、policy.export.json
- 证据：T18 toe-in 对比、60 s 报告

## 6. 安全注意
- guarded 脚本拒绝在 `name=ai` 下启动，也拒绝重复 publisher。
- X/EmergencyDamping 后必须等 publisher 停止，再 `motion-select ai`；`safe_exit_publisher.sh` 可辅助。
- A_400 在 MuJoCo 高速横移/转向仍会轮距收缩、support4 下降；仿真不是实机安全证明。
- exact mixed `(0.4,0.25,-0.3)` 在 MuJoCo 会倒；包内无 bypass。
- 本包含 zero-B publish/replay，不含 ARM64 `joy_publish`；T2/T3 遥控行走需另补 JOY target。
- 不做任何上机发布，除非 owner 明确放行。
