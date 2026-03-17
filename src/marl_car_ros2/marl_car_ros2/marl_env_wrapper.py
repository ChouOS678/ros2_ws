from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class JointAction:
    planner_target_speed: float
    chassis_target_omega: float


class Ros2MarlEnvWrapper(Node):
    def __init__(self) -> None:
        super().__init__("marl_env_wrapper")
        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("scan_downsample_bins", 24)
        self.declare_parameter("max_scan_range", 8.0)
        self.declare_parameter("collision_range", 0.22)
        self.declare_parameter("max_speed", 1.8)
        self.declare_parameter("max_omega", 1.6)
        self.declare_parameter("max_accel", 1.2)
        self.declare_parameter("step_hz", 10.0)
        self.declare_parameter("stuck_speed_eps", 0.03)
        self.declare_parameter("stuck_steps_limit", 30)

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.scan_bins = int(self.get_parameter("scan_downsample_bins").value)
        self.max_scan_range = float(self.get_parameter("max_scan_range").value)
        self.collision_range = float(self.get_parameter("collision_range").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_omega = float(self.get_parameter("max_omega").value)
        self.max_accel = float(self.get_parameter("max_accel").value)
        self.stuck_speed_eps = float(self.get_parameter("stuck_speed_eps").value)
        self.stuck_steps_limit = int(self.get_parameter("stuck_steps_limit").value)

        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(Bool, "/llm_override/enabled", self._override_flag_cb, 10)
        self.create_subscription(Twist, "/llm_override/cmd_vel", self._override_cmd_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.reward_pub = self.create_publisher(Float32MultiArray, "/marl/reward", 10)
        self.obs_pub = self.create_publisher(Float32MultiArray, "/marl/obs_debug", 10)

        self._lock = threading.Lock()
        self.latest_odom: Optional[Odometry] = None
        self.latest_scan: Optional[LaserScan] = None
        self.override_enabled = False
        self.override_cmd = Twist()
        self.prev_goal_dist: Optional[float] = None
        self.prev_speed_cmd = 0.0
        self.prev_exec_speed = 0.0
        self.stuck_counter = 0
        self.last_step_time = time.time()

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self.latest_odom = msg

    def _scan_cb(self, msg: LaserScan) -> None:
        with self._lock:
            self.latest_scan = msg

    def _override_flag_cb(self, msg: Bool) -> None:
        self.override_enabled = bool(msg.data)

    def _override_cmd_cb(self, msg: Twist) -> None:
        self.override_cmd = msg

    def reset(self) -> np.ndarray:
        self.prev_goal_dist = None
        self.prev_speed_cmd = 0.0
        self.prev_exec_speed = 0.0
        self.stuck_counter = 0
        self.last_step_time = time.time()
        return self._build_observation()

    def step(self, joint_action: JointAction) -> Tuple[np.ndarray, Dict[str, float], bool, Dict]:
        obs = self._build_observation()
        now = time.time()
        dt = max(1e-3, now - self.last_step_time)
        self.last_step_time = now

        cmd = self.override_cmd if self.override_enabled else self._fuse_joint_action_to_cmd(joint_action, dt)
        self.cmd_pub.publish(cmd)
        rewards, done, info = self._compute_rewards_and_done(dt)

        reward_msg = Float32MultiArray()
        reward_msg.data = [rewards["planner"], rewards["chassis"]]
        self.reward_pub.publish(reward_msg)

        obs_msg = Float32MultiArray()
        obs_msg.data = obs.tolist()
        self.obs_pub.publish(obs_msg)
        return obs, rewards, done, info

    def _build_observation(self) -> np.ndarray:
        with self._lock:
            odom = self.latest_odom
            scan = self.latest_scan
        if odom is None or scan is None:
            return np.zeros(6 + self.scan_bins, dtype=np.float32)

        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        q = odom.pose.pose.orientation
        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        vx = odom.twist.twist.linear.x
        wz = odom.twist.twist.angular.z

        goal_dx = self.goal_x - px
        goal_dy = self.goal_y - py
        goal_dist = math.hypot(goal_dx, goal_dy)
        goal_heading = math.atan2(goal_dy, goal_dx) - yaw
        goal_heading = math.atan2(math.sin(goal_heading), math.cos(goal_heading))

        scan_arr = np.asarray(scan.ranges, dtype=np.float32)
        scan_arr = np.nan_to_num(scan_arr, nan=self.max_scan_range, posinf=self.max_scan_range, neginf=0.0)
        scan_arr = np.clip(scan_arr, 0.0, self.max_scan_range)
        pooled = self._pool_1d(scan_arr, self.scan_bins) / max(self.max_scan_range, 1e-6)

        ego = np.array(
            [
                float(vx / max(self.max_speed, 1e-6)),
                float(wz / max(self.max_omega, 1e-6)),
                float(math.sin(yaw)),
                float(math.cos(yaw)),
                float(min(goal_dist / 20.0, 1.0)),
                float(goal_heading / math.pi),
            ],
            dtype=np.float32,
        )
        return np.concatenate([ego, pooled], axis=0).astype(np.float32)

    @staticmethod
    def _pool_1d(arr: np.ndarray, bins: int) -> np.ndarray:
        if arr.size == 0:
            return np.zeros(bins, dtype=np.float32)
        chunks = np.array_split(arr, bins)
        return np.array([float(np.mean(c)) if c.size > 0 else 0.0 for c in chunks], dtype=np.float32)

    def _fuse_joint_action_to_cmd(self, action: JointAction, dt: float) -> Twist:
        with self._lock:
            scan = self.latest_scan
        cmd = Twist()

        target_v = float(np.clip(action.planner_target_speed, 0.0, self.max_speed))
        target_w = float(np.clip(action.chassis_target_omega, -self.max_omega, self.max_omega))

        min_range = self.max_scan_range
        if scan is not None and len(scan.ranges) > 0:
            ranges = np.asarray(scan.ranges, dtype=np.float32)
            ranges = np.nan_to_num(ranges, nan=self.max_scan_range, posinf=self.max_scan_range, neginf=0.0)
            ranges = np.clip(ranges, 0.0, self.max_scan_range)
            min_range = float(np.min(ranges))

        if min_range < 0.6:
            target_v *= 0.4
        if min_range < 0.35:
            target_v = min(target_v, 0.15)

        dv_max = self.max_accel * dt
        safe_v = float(np.clip(target_v, self.prev_speed_cmd - dv_max, self.prev_speed_cmd + dv_max))
        self.prev_speed_cmd = safe_v
        cmd.linear.x = safe_v
        cmd.angular.z = target_w
        return cmd

    def _compute_rewards_and_done(self, dt: float) -> Tuple[Dict[str, float], bool, Dict]:
        with self._lock:
            odom = self.latest_odom
            scan = self.latest_scan

        planner_reward = -0.1
        chassis_reward = -0.1
        done = False
        info: Dict[str, float] = {}
        if odom is None or scan is None:
            return {"planner": planner_reward, "chassis": chassis_reward}, done, info

        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        exec_v = float(odom.twist.twist.linear.x)
        goal_dist = math.hypot(self.goal_x - px, self.goal_y - py)

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=self.max_scan_range, posinf=self.max_scan_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_scan_range)
        min_range = float(np.min(ranges)) if ranges.size > 0 else self.max_scan_range

        progress = 0.0
        if self.prev_goal_dist is not None:
            progress = self.prev_goal_dist - goal_dist
        self.prev_goal_dist = goal_dist
        planner_reward = (-1.0 * dt) + (4.0 * progress)

        accel = (exec_v - self.prev_exec_speed) / max(dt, 1e-3)
        self.prev_exec_speed = exec_v
        smooth_penalty = -0.3 * abs(accel)
        clearance_bonus = 0.2 * min(min_range / max(self.max_scan_range, 1e-6), 1.0)
        chassis_reward = smooth_penalty + clearance_bonus

        if min_range <= self.collision_range:
            planner_reward -= 50.0
            chassis_reward -= 100.0
            done = True
            info["termination"] = "collision"
        if goal_dist < 0.25:
            planner_reward += 100.0
            chassis_reward += 20.0
            done = True
            info["termination"] = "goal_reached"

        if abs(exec_v) < self.stuck_speed_eps and goal_dist > 0.5:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
        if self.stuck_counter >= self.stuck_steps_limit:
            info["stuck"] = 1.0
            planner_reward -= 1.0
            chassis_reward -= 2.0

        info["goal_dist"] = goal_dist
        info["min_range"] = min_range
        return {"planner": planner_reward, "chassis": chassis_reward}, done, info

