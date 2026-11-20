"""Deterministic DR sampler - faithful port of dr-handoff/sampler.py.

Kept byte-compatible in behaviour with the reference implementation: the random
stream is seeded with the SAME string "seed:stage:env_id:episode:split", so a
given (stage, env, episode) draws exactly what the reference sampler draws. That
is what makes the GPU-side sampling verifiable against sampler.py.

Reference: AS2-W_CPRO/部署测试/04_调试记录/logs/20261119_himloco_handoff_feedback_audit/dr-handoff/sampler.py
"""
import json
import random
from pathlib import Path


class Sampler:
    def __init__(self, directory, seed=42):
        directory = Path(directory)
        self.config = json.loads((directory / "stages.json").read_text())
        self.sessions = json.loads((directory / "fixtures.json").read_text())["sessions"]
        self.seed = seed

    def stage(self, name):
        return next(s for s in self.config["stages"] if s["name"] == name)

    def stage_names(self):
        return [s["name"] for s in self.config["stages"]]

    def sample(self, stage, env_id, episode, split="train"):
        cfg = self.stage(stage)
        rng = random.Random(f"{self.seed}:{stage}:{env_id}:{episode}:{split}")
        sessions = [s for s in self.sessions if s["split"] == split]
        fixture = None
        if rng.random() < cfg["field_probability"]:
            session = rng.choice(sessions)
            fixture = {"session": session["name"], **rng.choice(session["frames"])}
        return {
            "fixture": fixture,
            "base_mass_delta_kg": rng.uniform(*cfg["base_mass_delta_kg"]),
            "com_delta_x_m": rng.uniform(*cfg["com_delta_x_m"]),
            "friction_scale": rng.uniform(*cfg["friction_scale"]),
            "delay_steps": rng.randint(*cfg["delay_steps"]),
        }
