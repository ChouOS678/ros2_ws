from __future__ import annotations

import math
from typing import Iterable, List, Tuple

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


PATH_PRESETS = {
    "A": 45.0,
    "B": 90.0,
    "C": 135.0,
}


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _build_pose(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    qx, qy, qz, qw = _yaw_to_quaternion(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def build_corner_points(
    *,
    straight_length: float,
    turn_length: float,
    angle_deg: float,
) -> List[Tuple[float, float, float]]:
    angle_rad = math.radians(float(angle_deg))
    turn_dx = float(turn_length) * math.cos(angle_rad)
    turn_dy = float(turn_length) * math.sin(angle_rad)
    waypoints = [
        (0.0, 0.0, 0.0),
        (float(straight_length), 0.0, 0.0),
        (float(straight_length) + turn_dx, turn_dy, angle_rad),
    ]
    return waypoints


def build_path(
    *,
    frame_id: str,
    angle_deg: float,
    straight_length: float = 2.0,
    turn_length: float = 2.0,
    samples_per_segment: int = 20,
) -> Path:
    path = Path()
    path.header.frame_id = frame_id

    corners = build_corner_points(
        straight_length=straight_length,
        turn_length=turn_length,
        angle_deg=angle_deg,
    )

    def sample_segment(
        start: Tuple[float, float, float],
        end: Tuple[float, float, float],
    ) -> Iterable[PoseStamped]:
        sx, sy, syaw = start
        ex, ey, eyaw = end
        for idx in range(samples_per_segment):
            t = idx / float(max(samples_per_segment - 1, 1))
            x = sx + (ex - sx) * t
            y = sy + (ey - sy) * t
            yaw = syaw + (eyaw - syaw) * t
            yield _build_pose(frame_id, x, y, yaw)

    first = list(sample_segment(corners[0], corners[1]))
    second = list(sample_segment(corners[1], corners[2]))
    path.poses = first + second[1:]
    return path
