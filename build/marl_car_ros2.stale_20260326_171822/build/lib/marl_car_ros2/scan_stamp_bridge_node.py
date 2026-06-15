from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanStampBridgeNode(Node):
    """Normalize Gazebo scan timestamps onto the active ROS clock."""

    def __init__(self) -> None:
        super().__init__("scan_stamp_bridge")

        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("output_frame", "")
        self.declare_parameter("restamp_with_now", True)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame = str(self.get_parameter("output_frame").value).strip()
        self.restamp_with_now = bool(self.get_parameter("restamp_with_now").value)

        self._pub = self.create_publisher(LaserScan, output_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, input_topic, self._on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f"scan_stamp_bridge relaying {input_topic} -> {output_topic}, "
            f"restamp_with_now={self.restamp_with_now}"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        out = deepcopy(msg)
        if self.restamp_with_now:
            out.header.stamp = self.get_clock().now().to_msg()
        if self.output_frame:
            out.header.frame_id = self.output_frame
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanStampBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
