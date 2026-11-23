#!/usr/bin/env python3
"""Verify migration artifact/docs hashes recorded in server_manifest/MIGRATION_MANIFEST.json."""
import hashlib, json, sys
from pathlib import Path

root = Path(__file__).resolve().parent.parent
manifest = json.loads((root / "server_manifest" / "MIGRATION_MANIFEST.json").read_text())
bad = []
for section in ("artifacts", "docs"):
    for item in manifest[section]:
        p = root / item["path"]
        if not p.is_file():
            bad.append(("MISSING", item["path"])); continue
        h = hashlib.sha256(p.read_bytes()).hexdigest()
        if h != item["sha256"]:
            bad.append(("MISMATCH", item["path"], h, item["sha256"]))
        else:
            print("OK", item["path"], item["bytes"], h)
if bad:
    print("FAIL", len(bad), bad[:5]); sys.exit(1)
print("MANIFEST_OK")
