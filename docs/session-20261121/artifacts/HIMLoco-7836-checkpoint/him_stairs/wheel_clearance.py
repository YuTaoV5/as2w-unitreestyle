"""Wheel collision support height for the training reward."""
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import torch


def wheel_support_table(asset_path, wheel_names, samples=257):
    """Precompute the support height of each tire's radial profile."""
    import trimesh

    path = Path(asset_path)
    robot = ET.parse(path)
    tilt = np.linspace(-np.pi / 2., np.pi / 2., samples)
    tables = []
    for name in wheel_names:
        link = robot.find(f"link[@name='{name}']")
        collision = link.find("collision")
        origin = collision.find("origin")
        if origin is not None and any(
            float(value) != 0. for axis in ("xyz", "rpy") for value in origin.get(axis, "0 0 0").split()
        ):
            raise ValueError(f"{name}: expected the wheel collision mesh in the link frame")
        shape = collision.find("geometry/mesh")
        points = trimesh.load_mesh(path.parent / shape.attrib["filename"]).vertices
        points = points * np.fromstring(shape.get("scale", "1 1 1"), sep=" ")
        radial = np.hypot(points[:, 0], points[:, 2])
        table = (np.sin(tilt[:, None]) * points[None, :, 1] - np.cos(tilt[:, None]) * radial).min(axis=1)
        tables.append(table)
    return np.asarray(tables, dtype=np.float32)


def tire_bottom_z(origin_z, axle_z, support_table):
    """Interpolate tire support height relative to the wheel link origin."""
    angle = axle_z.to(support_table.dtype).clamp(-1., 1.).asin()
    index = (angle / math.pi + .5) * (support_table.shape[1] - 1)
    lo = index.floor().long().clamp(0, support_table.shape[1] - 1)
    hi = (lo + 1).clamp_max(support_table.shape[1] - 1)
    by_wheel = support_table[None, :, :].expand(origin_z.shape[0], -1, -1)
    z0 = by_wheel.gather(2, lo[..., None]).squeeze(2)
    z1 = by_wheel.gather(2, hi[..., None]).squeeze(2)
    return origin_z + torch.lerp(z0, z1, index - lo)
