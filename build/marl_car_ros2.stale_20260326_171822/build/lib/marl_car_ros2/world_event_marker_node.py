from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class WorldEventMarkerNode(Node):
    def __init__(self) -> None:
        super().__init__("world_event_marker")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic_in", "/world_model/events")
        self.declare_parameter("topic_out", "/visualization/world_event_markers")

        self.frame_id = str(self.get_parameter("frame_id").value)
        topic_in = str(self.get_parameter("topic_in").value)
        topic_out = str(self.get_parameter("topic_out").value)

        self.latest = {
            "type": "none",
            "status": "idle",
            "severity": 0.0,
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
        event_type = str(self.latest.get("type", self.latest.get("event_type", "none")))
        status = str(self.latest.get("status", self.latest.get("event_status", "idle")))
        severity = float(self.latest.get("severity", 0.0))

        text = Marker()
        text.header.frame_id = self.frame_id
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = "world_event"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = -1.8
        text.pose.position.y = 0.7
        text.pose.position.z = 1.2
        text.scale.z = 0.18
        text.color.a = 1.0
        text.color.r = 0.7
        text.color.g = 0.9
        text.color.b = 1.0
        text.text = f"WorldEvent: {event_type} | status: {status} | severity: {severity:.2f}"

        ma.markers = [text]
        self.pub.publish(ma)


def main() -> None:
    rclpy.init()
    node = WorldEventMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
