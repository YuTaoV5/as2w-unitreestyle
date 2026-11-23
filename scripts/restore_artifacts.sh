#!/bin/bash
# Unpack migration artifacts into ./restored without touching system paths.
# Read-only with respect to the robot; no publisher, no LowCmd.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p restored
for z in artifacts/*.zip; do
  echo "== unzip $z =="
  unzip -o "$z" -d restored
done
cat > restored/READ_ME_FIRST.txt <<'EOF'
restored/ contains:
- AS2W_DELIVERY_20261123/ : full delivery ZIP contents (A/v4x, videos, docs, training assets)
- AS2W_A400_DEPLOY_20261123/ : A_400 self-contained deployment package
- AS2W_C1_s42_200_DEPLOY_20261123/ : C1 comparison/fallback deployment package

Next:
1) cd AS2W_A400_DEPLOY_20261123
2) sha256sum -c SHA256SUMS
3) read README_FIRST.md; CPRO install script runs offline audit only.
Do not start publisher while motion-mode name=ai.
EOF
echo "RESTORE_OK restored/"
