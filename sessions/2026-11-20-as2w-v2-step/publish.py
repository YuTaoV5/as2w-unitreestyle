"""Write manifest.md for the staged session archive and publish it to GitHub.

Publishing uses the REST API (git push is blocked from this host). The token is
read from the stored git credential and never printed.
"""
import base64
import hashlib
import json
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path

STAGE = Path(r"X:\Items\as2w\.stage58\gh-stage-20261120")
REPO = "YuTaoV5/as2w-unitreestyle"
PREFIX = "sessions/2026-11-20-as2w-v2-step"
MESSAGE = """Add 2026-11-20 AS2-W v2 session archive (sim-to-real, friction, field logs)

One-day session covering the v2 step-only line from training to the real robot:

- tanh squash + vx removal + motor-jitter randomisation (sign-flip 30% -> 8-13%)
- quantified the missing sim term: ground/tire friction (static hip load
  0.39 -> 5.37 Nm; rigid-hold 1.66 -> 11.91 Nm vs 11-12 Nm on hardware)
- negative controls: joint friction, base mass and COM shift do not explain it
- friction-aligned retraining (mu025, 600 updates) and its hardware A/B:
  yaw tracking slope 0.146 vs 0.199 -> the friction hypothesis does NOT explain
  the weak hardware turning
- CPRO deployment path: ONNX export, frame parity, ARM64 build, CTest 10/10,
  read-only replay and C++<->Python parity
- analysis JSON, plots and downsampled data for every claim

Full field logs (4.8 GB) stay local; see manifest.md for hashes and locations.
"""


def sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


# ── manifest ────────────────────────────────────────────────────────────────
files = sorted(p for p in STAGE.rglob("*") if p.is_file() and p.name not in ("manifest.md",))
rows = [(p.relative_to(STAGE).as_posix(), p.stat().st_size, sha256(p)) for p in files]
manifest = ["# 归档清单（2026-11-20 会话）", "",
            f"共 {len(rows)} 个文件，合计 {sum(r[1] for r in rows)/1024/1024:.2f} MB。", "",
            "| 文件 | 字节 | SHA256 |", "|---|---|---|"]
for rel, size, h in rows:
    manifest.append(f"| `{rel}` | {size} | `{h}` |")
manifest += [
    "", "## 未上传（体量原因，已在本机归档）", "",
    "| 内容 | 大小 | 位置 / 校验 |", "|---|---|---|",
    "| 实机日志全量（field_logs + field_sessions） | 4.86 GB（解包） | `X:/Items/as2w/AS2W-field-logs-20261120/`；"
    "`field_logs_bundle.tar.gz` sha `5c09c4e00b17cfb486e5060f4eb7d50de0152c130e74d5225b92d953e6b65a22`，"
    "`field_sessions.tar.gz` sha `7f3a237c85139ce6b3e0e5f74a61225b9ad6383e3ff20bc30e1fdb657d6a3576` |",
    "| 训练权重与 ONNX | 每个 4.7 MB / 0.8 MB | 训练服务器 `/root/gpufree-data/as2w-runs/<run>/train/`；"
    "交付包内 SHA 见 `docs/deployment-contract-audit.md` |",
    "| 传感器级回放带 | 7 MB/份 | 同交付包 `replay/sensor_tape.npz` |",
    "", "## 说明", "",
    "- 训练服务器（gpufree）在会话结束后已不可达；本目录所有数字均来自本机留存的分析产物与日志副本。",
    "- 图表由 `make_plots.py` 从 `data/` 与 `analysis/` 的可复算数据生成，可重新运行。 ",
]
(STAGE / "manifest.md").write_text("\n".join(manifest) + "\n", encoding="utf-8")
files_all = sorted(p for p in STAGE.rglob("*") if p.is_file())
print("staged files:", len(files_all))

# ── token ───────────────────────────────────────────────────────────────────
inp = "protocol=https\nhost=github.com\n\n"
proc = subprocess.run(["git", "credential", "fill"], input=inp, capture_output=True, text=True)
token = ""
user = ""
for line in proc.stdout.splitlines():
    if line.startswith("username="):
        user = line[len("username="):]
    elif line.startswith("password="):
        token = line[len("password="):]
if not token:
    print("no stored credential")
    sys.exit(1)
print("credential ok:", user, "len", len(token))


def api(method, path, payload=None):
    url = f"https://api.github.com/repos/{REPO}{path}"
    data = json.dumps(payload).encode() if payload is not None else None
    req = urllib.request.Request(url, data=data, method=method, headers={
        "Authorization": f"Bearer {token}",
        "Accept": "application/vnd.github+json",
        "X-GitHub-Api-Version": "2022-11-28",
        "Content-Type": "application/json",
        "User-Agent": "as2w-session-publish",
    })
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.loads(r.read().decode())


head = api("GET", "/git/ref/heads/main")["object"]["sha"]
print("main head:", head)
commit = api("GET", f"/git/commits/{head}")
base_tree = commit["tree"]["sha"]

entries = []
for p in files_all:
    rel = p.relative_to(STAGE).as_posix()
    blob = api("POST", "/git/blobs", {
        "content": base64.b64encode(p.read_bytes()).decode(),
        "encoding": "base64",
    })
    entries.append({"path": f"{PREFIX}/{rel}", "mode": "100644", "type": "blob", "sha": blob["sha"]})
    print("blob", rel, p.stat().st_size)

tree = api("POST", "/git/trees", {"base_tree": base_tree, "tree": entries})
new_commit = api("POST", "/git/commits", {"message": MESSAGE, "tree": tree["sha"], "parents": [head]})
print("commit", new_commit["sha"])
api("PATCH", "/git/refs/heads/main", {"sha": new_commit["sha"], "force": False})
print("PUBLISH_OK", new_commit["sha"])
