from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from .observation_builder import ObservationBuilder


class BaselineNavNode(Node):
    """
    Compatibility baseline navigator.

    This node is a deterministic local-navigation baseline used when full Nav2 stack
    is not launched. It can publish to /cmd_vel (baseline mode) or /cmd_vel_nav
    (agent+supervisor mode).
    """

    def __init__(self) -> None:
        super().__init__("baseline_nav_node")

        self.declare_parameter("step_hz", 10.0)
        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("scan_bins", 24)
        self.declare_parameter("max_scan_range", 8.0)
        self.declare_parameter("max_speed", 1.2)
        self.declare_parameter("max_omega", 1.4)
        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("hard_stop_range", 0.26)
        self.declare_parameter("cautious_range", 0.8)
        self.declare_parameter("output_topic", "/cmd_vel")

        step_hz = float(self.get_parameter("step_hz").value)
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        scan_bins = int(self.get_parameter("scan_bins").value)
        max_scan_range = float(self.get_parameter("max_scan_range").value)

        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_omega = float(self.get_parameter("max_omega").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.hard_stop_range = float(self.get_parameter("hard_stop_range").value)
        self.cautious_range = float(self.get_parameter("cautious_range").value)

        output_topic = str(self.get_parameter("output_topic").value)

        self.obs_builder = ObservationBuilder(
            self,
            goal_x=goal_x,
            goal_y=goal_y,
            scan_bins=scan_bins,
            max_scan_range=max_scan_range,
        )

        self.cmd_pub = self.create_publisher(Twist, output_topic, 20)
        self.status_pub = self.create_publisher(String, "/baseline/status", 20)
        self.start_wall = time.time()
        self.step_count = 0
        self.mission_status = "running"

        self.timer = self.create_timer(1.0 / max(step_hz, 1e-3), self._step)

    def _step(self) -> None:
        self.step_count += 1
        snapshot = self.obs_builder.build_snapshot()
        cmd = Twist()
        reason = "running"

        if snapshot is None:
            reason = "sensor_not_ready"
        elif snapshot.goal_dist <= self.goal_tolerance:
            self.mission_status = "success"
            reason = "goal_reached"
        elif snapshot.min_range <= self.hard_stop_range:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.6
            reason = "hard_stop_turn"
        else:
            heading = float(snapshot.goal_heading)
            turn = max(-self.max_omega, min(self.max_omega, 1.2 * heading))
            align = max(0.25, 1.0 - min(abs(heading) / math.pi, 1.0))
            speed_scale = 1.0
            if snapshot.min_range < self.cautious_range:
                speed_scale = 0.5
                reason = "cautious"
            cmd.linear.x = max(0.0, min(self.max_speed, self.max_speed * align * speed_scale))
            cmd.angular.z = turn

        self.cmd_pub.publish(cmd)
        self._publish_status(snapshot, cmd, reason)

    def _publish_status(self, snapshot, cmd: Twist, reason: str) -> None:
        payload = {
            "wall_time": time.time(),
            "uptime_s": time.time() - self.start_wall,
            "step_count": self.step_count,
            "mission_status": self.mission_status,
            "reason": reason,
            "goal_dist": float(snapshot.goal_dist) if snapshot is not None else -1.0,
            "min_range": float(snapshot.min_range) if snapshot is not None else -1.0,
            "cmd_linear_x": float(cmd.linear.x),
            "cmd_angular_z": float(cmd.angular.z),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = BaselineNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
