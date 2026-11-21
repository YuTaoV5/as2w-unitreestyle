"""Deterministic episode sampling. Outputs absolute deltas from the v2 asset."""
import json
import random
from pathlib import Path

class Sampler:
    def __init__(self, directory=None, seed=42):
        directory = Path(directory or Path(__file__).parent)
        self.config = json.loads((directory / "stages.json").read_text())
        self.sessions = json.loads((directory / "fixtures.json").read_text())["sessions"]
        self.seed = seed

    def sample(self, stage, env_id, episode, split="train"):
        cfg = next(s for s in self.config["stages"] if s["name"] == stage)
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
