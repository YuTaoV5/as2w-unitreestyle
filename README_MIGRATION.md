# AS2-W 迁移仓库（2026-11-23）

本仓库用于在服务器可能关闭的情况下，保留 **可复现的部署/训练/分析资产**，让新服务器或新 agent 能继续任务。

> **状态：迁移快照，不是已放行产品。**
> A_400 / C1 / A_799 均未完成 owner 最终放行；实机 zero-B/B gate 未全部通过。
> 仓库应设为 **private**（训练数据/内部报告不应公开）。

## 1. 仓库内容

| 路径 | 内容 | 大小/SHA |
|---|---|---|
| `artifacts/AS2W_DELIVERY_20261123.zip` | 完整交付包：A/v4x 部署、全部 60s 视频、训练资产、证据、脚本 | 78,480,543 B / sha256 `见 MIGRATION_MANIFEST.json` |
| `artifacts/AS2W_A400_DEPLOY_20261123.zip` | A_400 自包含部署包（A400 ONNX + ARM64 + 训练资产） | 6,714,675 B |
| `artifacts/AS2W_C1_s42_200_DEPLOY_20261123.zip` | C1_s42_200 部署包（对照/回退候选） | 6,738,949 B |
| `docs/` | 门限、现场、符号、stand、T8、姿态/内八/起身对比报告 | 见根 `SHA256SUMS` |
| `source/` | A 训练配方、A_400 README | — |
| `server_manifest/MIGRATION_MANIFEST.json` | 服务器路径/关键权重/SHA、迁移说明、已省略数据 | — |
| `ANALYSIS_PROCESS.md` | 从官方数据反析到 A_400 的分析决策链 | — |
| `scripts/` | 校验/解包/新服务器恢复脚本 | — |

## 2. 新服务器恢复步骤

```bash
git clone <private-repo-url> AS2W_MIGRATION_20261123
cd AS2W_MIGRATION_20261123

# 1) 完整性
sha256sum -c SHA256SUMS
python3 scripts/verify_manifest.py

# 2) 解包交付物（不会覆盖系统；默认到 ./restored）
bash scripts/restore_artifacts.sh

# 3) 看当前任务状态与分析链
less ANALYSIS_PROCESS.md
less docs/GATE_V4X_CROSSARCH_DRAFT.md
less docs/ZERO_B_FIELD_ATTEMPT-20261123.md
less docs/A_POLICY_TOEIN_60S_REPORT.md

# 4) 部署 A_400（CPRO rootfs，按 deployment/README_FIRST.md）
#    unzip -o restored/AS2W_A400_DEPLOY_20261123.zip
#    cd AS2W_A400_DEPLOY_20261123
#    sha256sum -c SHA256SUMS
#    bash deployment/scripts/install_a400_cpro.sh /root/as2w_deploy_a400_s42_400
```

`restore_artifacts.sh` 只解包到 `restored/`，不启动 publisher、不发 LowCmd。

## 3. 当前关键状态（搬迁前快照）
- 机器人：已恢复 `ai`；无 active publisher；无持续 LowCmd。
- A_799：MuJoCo/固定站/符号已修；zero-B 未过 B gate。
- A_400：T18 内八最接近官方（toe 6.53° vs official 5.54°）；仅在 MuJoCo 验证，CPRO audit 仍 PENDING；release_authorized=false。
- C1_s42_200：上一轮姿态候选；包在 `artifacts/` 供对照/回退。
- 服务器：训练 run、checkpoint、eval 存放在 `/root/gpufree-data/as2w-runs/...`；本仓库保存**关键可迁移资产**，不是整机全盘备份。

## 4. 不含/不公开
- 不含 API token、SSH key、密码、`.env`、服务器凭据。
- 不包含全部原始 200Hz/500Hz bag（体积大）；保留报告、摘要、关键 CSV/视频与路径清单。
- 训练大 run 的完整 raw logs 未全量上传；`server_manifest` 记录路径与 SHA，服务器若仍在线可按清单补拉。

## 5. 上传方式（需要 GitHub 凭据）
本目录已可 `git init/commit`。上传到 private repo 需要：
- repo URL（建议新建 private `as2w-migration`）
- 有 `repo` 权限的 PAT 或已配置的 SSH remote
后续执行：
```bash
git remote add origin <url>
git branch -M main
git push -u origin main
```

> 迁移目标：新服务器 clone 后能复现部署包、训练候选选择、仿真证据和当前风险边界；不是把旧服务器所有 raw telemetry 镜像。
