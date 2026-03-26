from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class ObservationSnapshot:
    wall_time: float
    pose_x: float
    pose_y: float
    yaw: float
    linear_speed: float
    angular_speed: float
    goal_dist: float
    goal_heading: float
    min_range: float
    scan_normalized: np.ndarray
    world_event: Dict[str, object]


class ObservationBuilder:
    """
    Runtime observation constructor decoupled from control/reward/termination logic.

    This helper attaches subscriptions to a host ROS2 node and provides normalized
    observations + rich snapshot metadata for decision layers.
    """

    def __init__(
        self,
        node: Node,
        *,
        goal_x: float,
        goal_y: float,
        scan_bins: int = 24,
        max_scan_range: float = 8.0,
    ) -> None:
        self.node = node
        self.goal_x = float(goal_x)
        self.goal_y = float(goal_y)
        self.scan_bins = int(scan_bins)
        self.max_scan_range = float(max_scan_range)

        self._lock = threading.Lock()
        self._latest_odom: Optional[Odometry] = None
        self._latest_scan: Optional[LaserScan] = None
        self._latest_world_event: Dict[str, object] = {}
        self._odom_wall_time = 0.0
        self._scan_wall_time = 0.0

        self.node.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.node.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.node.create_subscription(String, "/world_model/events", self._world_event_cb, 20)

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odom = msg
            self._odom_wall_time = time.time()

    def _scan_cb(self, msg: LaserScan) -> None:
        with self._lock:
            self._latest_scan = msg
            self._scan_wall_time = time.time()

    def _world_event_cb(self, msg: String) -> None:
        import json

        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                with self._lock:
                    self._latest_world_event = payload
        except json.JSONDecodeError:
            return

    def has_fresh_data(self, *, max_age_s: float = 1.5) -> bool:
        now = time.time()
        with self._lock:
            odom_age = now - self._odom_wall_time
            scan_age = now - self._scan_wall_time
            odom_ok = self._latest_odom is not None and odom_age <= max_age_s
            scan_ok = self._latest_scan is not None and scan_age <= max_age_s
        return bool(odom_ok and scan_ok)

    def get_raw_messages(self) -> Tuple[Optional[Odometry], Optional[LaserScan]]:
        with self._lock:
            return self._latest_odom, self._latest_scan

    def build_snapshot(self) -> Optional[ObservationSnapshot]:
        with self._lock:
            odom = self._latest_odom
            scan = self._latest_scan
            world_event = dict(self._latest_world_event)

        if odom is None or scan is None:
            return None

        px = float(odom.pose.pose.position.x)
        py = float(odom.pose.pose.position.y)
        q = odom.pose.pose.orientation
        yaw = yaw_from_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        vx = float(odom.twist.twist.linear.x)
        wz = float(odom.twist.twist.angular.z)

        goal_dx = self.goal_x - px
        goal_dy = self.goal_y - py
        goal_dist = math.hypot(goal_dx, goal_dy)
        goal_heading = math.atan2(goal_dy, goal_dx) - yaw
        goal_heading = math.atan2(math.sin(goal_heading), math.cos(goal_heading))

        scan_arr = np.asarray(scan.ranges, dtype=np.float32)
        scan_arr = np.nan_to_num(
            scan_arr,
            nan=self.max_scan_range,
            posinf=self.max_scan_range,
            neginf=0.0,
        )
        scan_arr = np.clip(scan_arr, 0.0, self.max_scan_range)
        scan_norm = self.pool_1d(scan_arr, self.scan_bins) / max(self.max_scan_range, 1e-6)
        min_range = float(np.min(scan_arr)) if scan_arr.size > 0 else self.max_scan_range

        return ObservationSnapshot(
            wall_time=time.time(),
            pose_x=px,
            pose_y=py,
            yaw=yaw,
            linear_speed=vx,
            angular_speed=wz,
            goal_dist=goal_dist,
            goal_heading=goal_heading,
            min_range=min_range,
            scan_normalized=scan_norm.astype(np.float32),
            world_event=world_event,
        )

    def build_vector(self, *, max_speed: float, max_omega: float, goal_norm_dist: float = 20.0) -> np.ndarray:
        snapshot = self.build_snapshot()
        if snapshot is None:
            return np.zeros(6 + self.scan_bins, dtype=np.float32)

        ego = np.array(
            [
                float(snapshot.linear_speed / max(max_speed, 1e-6)),
                float(snapshot.angular_speed / max(max_omega, 1e-6)),
                float(math.sin(snapshot.yaw)),
                float(math.cos(snapshot.yaw)),
                float(min(snapshot.goal_dist / max(goal_norm_dist, 1e-6), 1.0)),
                float(snapshot.goal_heading / math.pi),
            ],
            dtype=np.float32,
        )
        return np.concatenate([ego, snapshot.scan_normalized], axis=0).astype(np.float32)

    @staticmethod
    def pool_1d(arr: np.ndarray, bins: int) -> np.ndarray:
        if bins <= 0:
            return np.zeros(0, dtype=np.float32)
        if arr.size == 0:
            return np.zeros(bins, dtype=np.float32)
        chunks = np.array_split(arr, bins)
        return np.array([float(np.mean(c)) if c.size > 0 else 0.0 for c in chunks], dtype=np.float32)
