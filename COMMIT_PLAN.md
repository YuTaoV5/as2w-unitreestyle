# 分 commit 迁移计划

分支 `migration-split` 按依赖顺序逐个提交、逐个 push；每个 commit 都可单独审查。

| # | commit 内容 | 文件 |
|---|---|---|
| 1 | 迁移说明 + 分析链 + 安全边界 | `README_MIGRATION.md`, `ANALYSIS_PROCESS.md`, `.gitignore`, `COMMIT_PLAN.md` |
| 2 | 证据报告 | `docs/*.md` |
| 3 | 训练配方/来源 | `source/*` |
| 4 | 恢复脚本 + 服务器清单 | `scripts/*`, `server_manifest/*` |
| 5 | A_400 部署包 | `artifacts/AS2W_A400_DEPLOY_20261123.zip` |
| 6 | C1 部署包 | `artifacts/AS2W_C1_s42_200_DEPLOY_20261123.zip` |
| 7 | 完整交付包 | `artifacts/AS2W_DELIVERY_20261123.zip` |
| 8 | 最终完整性清单 | `SHA256SUMS` |

注意：
- `SHA256SUMS` 最后提交，之前 commit 不声称完整校验。
- 大文件在 commit 7；GitHub 会告警 >50MB，但 <100MB 上限。
- 每个 commit 后立即 `git push origin migration-split`，确认 remote HEAD。
- 不要把这个仓库当 public 存放内部训练资产；见 README 安全提醒。
