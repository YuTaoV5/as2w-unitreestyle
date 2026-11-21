"""Archive one plot per evaluation trial from an Isaac flat replay."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

p = argparse.ArgumentParser()
p.add_argument("trace", type=Path)
p.add_argument("--dt", type=float, default=.02)
p.add_argument("--representative", action="store_true")
a = p.parse_args()
out = a.trace.parent / "curves"
out.mkdir(exist_ok=True)
with np.load(a.trace, allow_pickle=False) as data:
    q, targets, names = data["q"], data["target_trace"], data["joints"]
    labels, fall = data["labels"], data["fall"]
    has_ground = "wheel_ground_normal_n" in data
    load = data["wheel_ground_normal_n"] if has_ground else data["wheel_data"][..., 15]
    assert load.shape == (*q.shape[:2], 4)
    groups = {}
    for trial, label in enumerate(labels.tolist()):
        groups.setdefault(label, []).append(trial)
    selected = [ids[len(ids)//2] for ids in groups.values()] if a.representative else list(range(len(labels)))
    for trial in selected:
        ends = np.flatnonzero(fall[:, trial])
        end = int(ends[0])+1 if len(ends) else len(q)
        t = np.arange(end)*a.dt
        fig, axes = plt.subplots(5, 3, figsize=(12,10), sharex=True)
        for j, ax in enumerate(axes[:4].flat):
            ax.plot(t,q[:end,trial,j], label="measured")
            ax.plot(t,targets[:end,trial,2,j], "--", label="applied")
            ax.set_title(f"{names[j]} (rad)")
        axes[0,0].legend()
        ax = axes[4,0]
        for wheel,name in enumerate(names[12:16]):
            ax.plot(t,load[:end,trial,wheel],label=name)
        ax.legend(fontsize=7)
        ax.set_title("Ground normal (N)" if has_ground else "Wheel net normal z (N)")
        axes[4,1].plot(t,data["tilt"][:end,trial])
        axes[4,1].set_title("Tilt (deg)")
        axes[4,2].plot(t,data["vel"][:end,trial,0],label="actual")
        axes[4,2].plot(t,data["cmd"][:end,trial,0],"--",label="cmd")
        axes[4,2].set_title("Forward velocity (m/s)")
        axes[4,2].legend()
        for ax in axes[4]:
            ax.set_xlabel("Time (s)")
        fig.suptitle(f"{labels[trial]} | trial {trial} | terminated={bool(len(ends))}")
        fig.tight_layout()
        fig.savefig(out/f"{trial:03d}-{labels[trial]}.png",dpi=120)
        plt.close(fig)
    (out/"coverage.json").write_text(json.dumps(dict(trials=len(labels), plotted=selected,
        ground_filtered=has_ground, joint_torque_available="leg_torque_nm" in data,
        source=a.trace.name),indent=2))
