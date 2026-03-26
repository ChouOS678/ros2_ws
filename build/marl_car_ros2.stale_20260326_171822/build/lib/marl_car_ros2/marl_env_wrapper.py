from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from .observation_builder import ObservationBuilder
from .reward_evaluator import RewardConfig, RewardEvaluator
from .termination_checker import TerminationChecker, TerminationConfig


@dataclass
class JointAction:
    planner_target_speed: float
    chassis_target_omega: float


class Ros2MarlEnvWrapper(Node):
    """
    Legacy compatibility node for MARL training/evaluation.

    Runtime control path for thesis architecture should use:
      task_agent -> supervisor_node -> /cmd_vel

    This wrapper remains for training utilities and old scripts.
    """

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

        self.obs_builder = ObservationBuilder(
            self,
            goal_x=self.goal_x,
            goal_y=self.goal_y,
            scan_bins=self.scan_bins,
            max_scan_range=self.max_scan_range,
        )

        self.create_subscription(Bool, "/llm_override/enabled", self._override_flag_cb, 10)
        self.create_subscription(Twist, "/llm_override/cmd_vel", self._override_cmd_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.reward_pub = self.create_publisher(Float32MultiArray, "/marl/reward", 10)
        self.obs_pub = self.create_publisher(Float32MultiArray, "/marl/obs_debug", 10)

        self.override_enabled = False
        self.override_cmd = Twist()
        self.prev_speed_cmd = 0.0
        self.last_step_time = time.time()

        self.reward_evaluator = RewardEvaluator(
            RewardConfig(
                max_scan_range=self.max_scan_range,
            )
        )
        self.termination_checker = TerminationChecker(
            TerminationConfig(
                collision_range=self.collision_range,
                stuck_speed_eps=self.stuck_speed_eps,
                stuck_steps_limit=self.stuck_steps_limit,
            )
        )

    def _override_flag_cb(self, msg: Bool) -> None:
        self.override_enabled = bool(msg.data)

    def _override_cmd_cb(self, msg: Twist) -> None:
        self.override_cmd = msg

    def reset(self) -> np.ndarray:
        self.prev_speed_cmd = 0.0
        self.last_step_time = time.time()
        self.reward_evaluator.reset()
        self.termination_checker.reset()
        return self.obs_builder.build_vector(max_speed=self.max_speed, max_omega=self.max_omega)

    def step(self, joint_action: JointAction) -> Tuple[np.ndarray, Dict[str, float], bool, Dict]:
        obs = self.obs_builder.build_vector(max_speed=self.max_speed, max_omega=self.max_omega)
        snapshot = self.obs_builder.build_snapshot()

        now = time.time()
        dt = max(1e-3, now - self.last_step_time)
        self.last_step_time = now

        cmd = self.override_cmd if self.override_enabled else self._fuse_joint_action_to_cmd(joint_action, dt)
        self.cmd_pub.publish(cmd)

        if snapshot is None:
            rewards = {"planner": -0.1, "chassis": -0.1}
            done = False
            info: Dict[str, float] = {}
        else:
            rewards = self.reward_evaluator.evaluate(snapshot=snapshot, dt=dt)
            done, info = self.termination_checker.evaluate(snapshot=snapshot)

            if str(info.get("termination", "")) == "collision":
                rewards["planner"] -= 50.0
                rewards["chassis"] -= 100.0
            elif str(info.get("termination", "")) == "goal_reached":
                rewards["planner"] += 100.0
                rewards["chassis"] += 20.0

            if float(info.get("stuck", 0.0)) > 0.0:
                rewards["planner"] -= 1.0
                rewards["chassis"] -= 2.0

        reward_msg = Float32MultiArray()
        reward_msg.data = [float(rewards["planner"]), float(rewards["chassis"])]
        self.reward_pub.publish(reward_msg)

        obs_msg = Float32MultiArray()
        obs_msg.data = obs.tolist()
        self.obs_pub.publish(obs_msg)

        return obs, rewards, done, info

    def _fuse_joint_action_to_cmd(self, action: JointAction, dt: float) -> Twist:
        _, scan = self.obs_builder.get_raw_messages()
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
