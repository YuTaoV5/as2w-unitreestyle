#!/bin/bash
# New-server sanity check: no hardware interaction.
set -euo pipefail
cd "$(dirname "$0")/.."
echo "=== repo ==="
pwd
echo "=== artifacts ==="
ls -lh artifacts
echo "=== SHA256SUMS ==="
sha256sum -c SHA256SUMS
echo "=== manifest ==="
python3 scripts/verify_manifest.py
echo "=== current state ==="
python3 - <<'PY'
import json
m=json.load(open("server_manifest/MIGRATION_MANIFEST.json",encoding="utf-8"))
print(json.dumps(m["current_state"],indent=2,ensure_ascii=False))
PY
echo "NEW_SERVER_CHECK_OK"
