"""Paired counterfactual inference on recorded sensors and target histories."""
import csv
import json
import hashlib
from pathlib import Path
import numpy as np
import onnxruntime as ort

base = Path(r"X:/Items/as2w/himloco_field_20261119")
policy = base / "offline_artifacts/policy.onnx"
assert hashlib.sha256(policy.read_bytes()).hexdigest() == "a74abbf77ca09ec021a8f306281f730a8a5f114b35fca8de2e84f0870acf8063"
runtime = ort.InferenceSession(str(policy), providers=["CPUExecutionProvider"])
default = np.tile([0, .8, -1.6], 4)
report = {}
for session in ("vx015_walked", "vx010_X_abort_handoff"):
    with (base / session / "debug_policy.csv").open() as stream:
        rows = [r for r in csv.reader(stream) if r and r[0] == "DEBUG_POLICY" and r[2] == "HANDOFF"]
    original = np.array([r[58:418] for r in rows], dtype=np.float32).reshape(-1, 6, 60)
    corrected = original.copy()
    actual = np.array([r[-64:] for r in rows], float).reshape(-1, 16, 4)
    feedback = np.column_stack(((actual[:, :12, 0] - default) / .25, actual[:, 12:, 1] / 12))
    for frame in range(len(rows)):
        for slot in range(min(frame, 6)):
            corrected[frame, slot, 41:57] = feedback[frame-slot-1]
    def predict(obs):
        return runtime.run(None, {"obs": obs.reshape(-1, 360)})[0]
    old, new = predict(original), predict(corrected)
    expected = np.array([r[419+int(r[418]):435+int(r[418])] for r in rows], float)
    parity = float(abs(old-expected).max())
    assert parity < 1e-4, parity
    delta = (np.tanh(new)-np.tanh(old)) * np.array([.15,.25,.25]*4+[2]*4)
    report[session] = {"frames":len(rows), "baseline_raw_parity_max":parity,
                       "pre_filter_leg_delta_max_rad":float(abs(delta[:,:12]).max()),
                       "pre_filter_wheel_delta_max_rad_s":float(abs(delta[:,12:]).max()),
                       "pre_filter_leg_delta_p95_rad":float(np.quantile(abs(delta[:,:12]),.95))}
result = json.dumps(report, indent=2)
Path(__file__).with_name("feedback-ab.json").write_text(result, encoding="utf-8")
print(result)
