from __future__ import annotations

import random

import rclpy

from .marl_env_wrapper import JointAction, Ros2MarlEnvWrapper


class DummyJointPolicy:
    def act(self) -> JointAction:
        return JointAction(
            planner_target_speed=random.uniform(0.0, 1.2),
            chassis_target_omega=random.uniform(-0.8, 0.8),
        )


def main() -> None:
    rclpy.init()
    node = Ros2MarlEnvWrapper()
    policy = DummyJointPolicy()
    node.reset()
    step_hz = float(node.get_parameter("step_hz").value)
    timer = node.create_timer(1.0 / step_hz, lambda: node.step(policy.act()))

    try:
        rclpy.spin(node)
    finally:
        timer.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

