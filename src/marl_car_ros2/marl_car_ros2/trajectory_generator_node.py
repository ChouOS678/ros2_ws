from __future__ import annotations

import os
from typing import Any, Dict

import rclpy
import yaml
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .trajectory_generator import path_from_trajectory


class TrajectoryGeneratorNode(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_generator_node")
        self.declare_parameter("scenario_name", "straight")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("output_topic", "/reference_path")
        self.declare_parameter("publish_once", True)
        self.declare_parameter("publish_rate_hz", 1.0)

        scenario_name = str(self.get_parameter("scenario_name").value)
        scenario_file = str(self.get_parameter("scenario_file").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publish_once = bool(self.get_parameter("publish_once").value)
        publish_rate = max(float(self.get_parameter("publish_rate_hz").value), 0.1)
        scenario = self._load_scenario(scenario_file, scenario_name)
        trajectory = scenario.get("trajectory", {})
        if not isinstance(trajectory, dict):
            raise ValueError(f"Scenario '{scenario_name}' has no valid trajectory configuration")

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.path_pub = self.create_publisher(Path, output_topic, qos)
        self.path = path_from_trajectory(trajectory, stamp=self.get_clock().now().to_msg())
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_path)
        self.get_logger().info(
            f"Generated deterministic '{trajectory.get('type', 'straight')}' path with "
            f"{len(self.path.poses)} poses on {output_topic}"
        )

    @staticmethod
    def _load_scenario(path: str, scenario_name: str) -> Dict[str, Any]:
        if not path or not os.path.exists(path):
            raise FileNotFoundError(f"Scenario file does not exist: {path}")
        with open(path, "r", encoding="utf-8") as scenario_file:
            data = yaml.safe_load(scenario_file) or {}
        scenarios = data.get("scenarios", {})
        scenario = scenarios.get(scenario_name)
        if not isinstance(scenario, dict):
            raise KeyError(f"Unknown trajectory scenario: {scenario_name}")
        return scenario

    def _publish_path(self) -> None:
        self.path.header.stamp = self.get_clock().now().to_msg()
        for pose in self.path.poses:
            pose.header.stamp = self.path.header.stamp
        self.path_pub.publish(self.path)
        if self.publish_once:
            self.timer.cancel()


def main() -> None:
    rclpy.init()
    node = TrajectoryGeneratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
