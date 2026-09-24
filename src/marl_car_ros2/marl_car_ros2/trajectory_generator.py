from __future__ import annotations

import math
from typing import Callable, Dict, List, Mapping

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def curvature_function(trajectory_type: str, s: float, length: float, config: Mapping[str, float]) -> float:
    kind = trajectory_type.strip().lower()
    kappa_max = float(config.get("kappa_max", 0.0))
    if kind == "straight":
        return 0.0
    if kind == "constant_curvature":
        return kappa_max
    if kind == "s_curve":
        frequency = float(config.get("curvature_frequency", 1.0))
        return kappa_max * math.cos(2.0 * math.pi * frequency * s / max(length, 1e-9))
    if kind == "clothoid":
        rate = float(config.get("curvature_rate", 0.0))
        return min(kappa_max, max(0.0, rate * s))
    if kind == "sharp_corner":
        center = float(config.get("corner_center_s_m", length * 0.5))
        width = max(float(config.get("corner_length_m", 1.0)), 1e-6)
        local_s = (s - (center - width * 0.5)) / width
        if 0.0 <= local_s <= 1.0:
            return kappa_max * 0.5 * (1.0 - math.cos(2.0 * math.pi * local_s))
        return 0.0
    raise ValueError(f"Unsupported trajectory_type: {trajectory_type}")


def generate_trajectory(config: Mapping[str, object]) -> List[tuple[float, float, float]]:
    trajectory_type = str(config.get("type", "straight"))
    length = max(float(config.get("length_m", 8.0)), 0.0)
    ds = max(float(config.get("ds_m", 0.05)), 1e-4)
    numeric_config = {key: float(value) for key, value in config.items() if isinstance(value, (int, float))}
    sample_count = max(int(math.ceil(length / ds)), 1)
    points: List[tuple[float, float, float]] = [(0.0, 0.0, 0.0)]
    x = 0.0
    y = 0.0
    theta = 0.0
    for index in range(sample_count):
        s = min(index * ds, length)
        step = min(ds, length - s)
        kappa = curvature_function(trajectory_type, s, length, numeric_config)
        theta_next = theta + kappa * step
        x += step * math.cos(theta)
        y += step * math.sin(theta)
        theta = theta_next
        points.append((x, y, theta))
    return points


def path_from_trajectory(config: Mapping[str, object], *, stamp=None) -> Path:
    frame_id = str(config.get("frame_id", "map"))
    path = Path()
    path.header.frame_id = frame_id
    if stamp is not None:
        path.header.stamp = stamp
    for x, y, yaw in generate_trajectory(config):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = _quaternion_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        path.poses.append(pose)
    return path
