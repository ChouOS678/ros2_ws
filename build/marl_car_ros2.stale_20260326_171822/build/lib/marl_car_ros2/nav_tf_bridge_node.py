from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class NavTfBridgeNode(Node):
    """Bridge odometry topic to TF and publish static lidar transform for Nav2/SLAM."""

    def __init__(self) -> None:
        super().__init__("nav_tf_bridge")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("laser_frame", "lidar_link")
        self.declare_parameter("laser_offset_x", 0.12)
        self.declare_parameter("laser_offset_y", 0.0)
        self.declare_parameter("laser_offset_z", 0.18)
        self.declare_parameter("laser_roll", 0.0)
        self.declare_parameter("laser_pitch", 0.0)
        self.declare_parameter("laser_yaw", 0.0)
        self.declare_parameter("publish_laser_tf", True)
        self.declare_parameter("use_msg_frame_ids", False)
        self.declare_parameter("use_msg_stamp", False)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.laser_frame = str(self.get_parameter("laser_frame").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.publish_laser_tf = bool(self.get_parameter("publish_laser_tf").value)
        self.use_msg_frame_ids = bool(self.get_parameter("use_msg_frame_ids").value)
        self.use_msg_stamp = bool(self.get_parameter("use_msg_stamp").value)
        if self.publish_laser_tf:
            self._publish_static_laser_tf()

        odom_topic = str(self.get_parameter("odom_topic").value)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 30)
        self.get_logger().info(f"nav_tf_bridge listening on {odom_topic}")

    def _publish_static_laser_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.laser_frame

        t.transform.translation.x = float(self.get_parameter("laser_offset_x").value)
        t.transform.translation.y = float(self.get_parameter("laser_offset_y").value)
        t.transform.translation.z = float(self.get_parameter("laser_offset_z").value)

        roll = float(self.get_parameter("laser_roll").value)
        pitch = float(self.get_parameter("laser_pitch").value)
        yaw = float(self.get_parameter("laser_yaw").value)
        qx, qy, qz, qw = self._quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.static_tf_broadcaster.sendTransform(t)

    def _on_odom(self, msg: Odometry) -> None:
        t = TransformStamped()

        if not self.use_msg_stamp or (msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0):
            t.header.stamp = self.get_clock().now().to_msg()
        else:
            t.header.stamp = msg.header.stamp

        header_frame = msg.header.frame_id.strip()
        child_frame = msg.child_frame_id.strip()

        if self.use_msg_frame_ids:
            t.header.frame_id = header_frame if header_frame else self.odom_frame
            t.child_frame_id = child_frame if child_frame else self.base_frame
        else:
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)


def main() -> None:
    rclpy.init()
    node = NavTfBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
