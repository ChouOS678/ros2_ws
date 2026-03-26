from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class RiskMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("risk_marker")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic_in", "/supervisor/status")
        self.declare_parameter("topic_out", "/visualization/risk_markers")

        self.frame_id = str(self.get_parameter("frame_id").value)
        topic_in = str(self.get_parameter("topic_in").value)
        topic_out = str(self.get_parameter("topic_out").value)

        self.latest = {
            "risk_level": "none",
            "min_range": -1.0,
            "override_reason": "",
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
        risk = str(self.latest.get("risk_level", "none")).lower()
        min_range = float(self.latest.get("min_range", -1.0))
        reason = str(self.latest.get("override_reason", ""))

        text = Marker()
        text.header.frame_id = self.frame_id
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "risk"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = -1.8
        text.pose.position.y = 1.05
        text.pose.position.z = 1.35
        text.scale.z = 0.20
        text.color.a = 1.0
        if risk in ("critical", "high"):
            text.color.r, text.color.g, text.color.b = (1.0, 0.25, 0.25)
        elif risk == "medium":
            text.color.r, text.color.g, text.color.b = (1.0, 0.8, 0.2)
        else:
            text.color.r, text.color.g, text.color.b = (0.75, 0.95, 0.75)
        text.text = f"Risk: {risk} | min_range: {min_range:.2f} | override: {reason[:40]}"

        ma.markers = [text]
        self.pub.publish(ma)


def main() -> None:
    rclpy.init()
    node = RiskMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
