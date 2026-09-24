from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class NavGoalSenderNode(Node):
    """Send a single NavigateToPose goal after Nav2 is ready."""

    def __init__(self) -> None:
        super().__init__("nav_goal_sender")

        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("goal_frame", "odom")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("controller_id", "PP")
        self.declare_parameter("controller_selector_topic", "controller_selector")
        self.declare_parameter("check_period_s", 0.5)
        self.declare_parameter("startup_delay_s", 2.0)
        self.declare_parameter("use_reference_path_goal", False)
        self.declare_parameter("reference_path_topic", "/reference_path")

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)
        self.goal_frame = str(self.get_parameter("goal_frame").value)
        self.controller_id = str(self.get_parameter("controller_id").value).strip()
        self.startup_delay_s = float(self.get_parameter("startup_delay_s").value)
        self.use_reference_path_goal = bool(self.get_parameter("use_reference_path_goal").value)

        goal_topic = str(self.get_parameter("goal_topic").value)
        controller_selector_topic = str(self.get_parameter("controller_selector_topic").value)
        period = max(float(self.get_parameter("check_period_s").value), 0.1)

        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        controller_selector_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.controller_selector_pub = self.create_publisher(
            String,
            controller_selector_topic,
            controller_selector_qos,
        )
        self.reference_path = None
        if self.use_reference_path_goal:
            self.reference_path_sub = self.create_subscription(
                Path,
                str(self.get_parameter("reference_path_topic").value),
                self._reference_path_callback,
                1,
            )
        action_type = FollowPath if self.use_reference_path_goal else NavigateToPose
        action_name = "follow_path" if self.use_reference_path_goal else "navigate_to_pose"
        self.action_client = ActionClient(self, action_type, action_name)
        self.goal_sent = False
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.timer = self.create_timer(period, self._tick)

    def _reference_path_callback(self, path: Path) -> None:
        if path.poses:
            self.reference_path = path

    def _tick(self) -> None:
        if self.goal_sent:
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        if now_s - self.start_time < self.startup_delay_s:
            return

        if self.use_reference_path_goal and self.reference_path is None:
            self.get_logger().info("Waiting for /reference_path...", throttle_duration_sec=2.0)
            return

        if not self.action_client.server_is_ready():
            self.get_logger().info("Waiting for navigate_to_pose action server...", throttle_duration_sec=2.0)
            if not self.action_client.wait_for_server(timeout_sec=0.1):
                return

        pose = PoseStamped()
        if self.use_reference_path_goal and self.reference_path is not None:
            pose = self.reference_path.poses[-1]
            self.goal_frame = self.reference_path.header.frame_id
        pose.header.frame_id = self.goal_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.goal_x
        pose.pose.position.y = self.goal_y
        qx, qy, qz, qw = quaternion_from_yaw(self.goal_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.goal_pub.publish(pose)

        if self.controller_id:
            controller_msg = String()
            controller_msg.data = self.controller_id
            self.controller_selector_pub.publish(controller_msg)

        if self.use_reference_path_goal and self.reference_path is not None:
            goal_msg = FollowPath.Goal()
            goal_msg.path = self.reference_path
            goal_msg.controller_id = self.controller_id
            goal_msg.goal_checker_id = ""
            goal_msg.progress_checker_id = ""
        else:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
        self.action_client.send_goal_async(goal_msg)
        self.goal_sent = True
        self.get_logger().info(
            f"Sent {'FollowPath' if self.use_reference_path_goal else 'NavigateToPose'} goal "
            f"frame={self.goal_frame} x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f} "
            f"controller={self.controller_id or 'default'}"
        )


def main() -> None:
    rclpy.init()
    node = NavGoalSenderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
