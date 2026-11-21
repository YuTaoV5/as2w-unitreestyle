"""Options that can be carried across reviewed PPO training segments."""
from math import isfinite

WEIGHT_LIMITS = {
    "swing_load_weight": 2.,
    "mirror_loss_weight": 1.,
    "action_overrun_weight": 2.,
    "lateral_progress_weight": 2.,
    "swing_placement_weight": 2.,
    "rolling_stability_weight": 2.,
}
OPTION_NAMES = (*WEIGHT_LIMITS, "tire_clearance", "leg_action_center_rad", "lateral_curriculum_updates")


def add_training_options(parser):
    for name in WEIGHT_LIMITS:
        parser.add_argument(flag(name), type=float, default=0.)
    parser.add_argument("--tire-clearance", action="store_true")
    parser.add_argument("--leg-action-center-rad", type=float, nargs=12)
    parser.add_argument("--lateral-curriculum-updates", type=int, default=0)


def flag(name):
    return "--" + name.replace("_", "-")


def validate_options(values, mode):
    if not isinstance(values, dict):
        raise ValueError("training_options must be a JSON object")
    unsupported = values.keys() - set(OPTION_NAMES)
    if unsupported:
        raise ValueError("Unsupported training options: " + ", ".join(sorted(unsupported)))
    for name, limit in WEIGHT_LIMITS.items():
        weight = values.get(name, 0.)
        if type(weight) not in (float, int) or not 0. <= weight <= limit:
            raise ValueError(f"{flag(name)} must be a number between 0 and {limit:g}")
    if type(values.get("tire_clearance", False)) is not bool:
        raise ValueError("tire_clearance must be true or false")
    center = values.get("leg_action_center_rad")
    if center is not None and (
        not isinstance(center, list) or len(center) != 12
        or not all(type(value) in (float, int) and isfinite(value) for value in center)
    ):
        raise ValueError("leg_action_center_rad must contain 12 finite coordinates or null")
    duration = values.get("lateral_curriculum_updates", 0)
    if type(duration) is not int or not 0 <= duration <= 2000:
        raise ValueError("lateral_curriculum_updates must be an integer between 0 and 2000")
    if mode != "unitree_style" and values.get("mirror_loss_weight", 0.):
        raise ValueError("Mirror loss is implemented for the Unitree-style PPO runner")


def options_from_namespace(args):
    return {name: getattr(args, name) for name in OPTION_NAMES}


def options_to_flags(values):
    flags = []
    for name in WEIGHT_LIMITS:
        if values.get(name, 0.):
            flags.extend((flag(name), str(values[name])))
    if values.get("tire_clearance", False):
        flags.append("--tire-clearance")
    center = values.get("leg_action_center_rad")
    if center is not None:
        flags.extend(("--leg-action-center-rad", *map(str, center)))
    if values.get("lateral_curriculum_updates", 0):
        flags.extend(("--lateral-curriculum-updates", str(values["lateral_curriculum_updates"])))
    return flags
