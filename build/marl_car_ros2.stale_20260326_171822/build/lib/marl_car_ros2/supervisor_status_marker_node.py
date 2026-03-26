from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class SupervisorStatusMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("supervisor_status_marker")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic_in", "/supervisor/status")
        self.declare_parameter("topic_out", "/visualization/supervisor_markers")

        self.frame_id = str(self.get_parameter("frame_id").value)
        topic_in = str(self.get_parameter("topic_in").value)
        topic_out = str(self.get_parameter("topic_out").value)

        self.latest = {
            "manager_mode": "UNKNOWN",
            "mode_reason": "",
            "mission_status": "unknown",
        }

        self.create_subscription(String, topic_in, self._cb, 20)
        self.pub = self.create_publisher(MarkerArray, topic_out, 10)
        self.timer = self.create_timer(0.2, self._publish)

    def _cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.latest = payload
        except json.JSONDecodeError:
            return

    def _publish(self) -> None:
        ma = MarkerArray()

        mode = str(self.latest.get("manager_mode", "UNKNOWN"))
        reason = str(self.latest.get("mode_reason", ""))
        mission = str(self.latest.get("mission_status", "unknown"))

        text = Marker()
        text.header.frame_id = self.frame_id
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "supervisor_status"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = -1.8
        text.pose.position.y = 1.4
        text.pose.position.z = 1.5
        text.scale.z = 0.22
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.text = f"Supervisor: {mode} | mission: {mission} | reason: {reason[:48]}"

        badge = Marker()
        badge.header = text.header
        badge.ns = "supervisor_status"
        badge.id = 2
        badge.type = Marker.SPHERE
        badge.action = Marker.ADD
        badge.pose.position.x = -2.15
        badge.pose.position.y = 1.4
        badge.pose.position.z = 1.35
        badge.scale.x = 0.18
        badge.scale.y = 0.18
        badge.scale.z = 0.18
        badge.color.a = 0.95
        if mode == "NOMINAL":
            badge.color.r, badge.color.g, badge.color.b = (0.1, 0.9, 0.1)
        elif mode == "CAUTIOUS":
            badge.color.r, badge.color.g, badge.color.b = (0.95, 0.75, 0.15)
        elif mode in ("BLOCKED", "RECOVERY_REQUESTED"):
            badge.color.r, badge.color.g, badge.color.b = (0.95, 0.2, 0.2)
        else:
            badge.color.r, badge.color.g, badge.color.b = (0.7, 0.7, 0.7)

        ma.markers = [text, badge]
        self.pub.publish(ma)


def main() -> None:
    rclpy.init()
    node = SupervisorStatusMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
