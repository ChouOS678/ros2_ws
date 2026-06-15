from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray


class BenchmarkVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("benchmark_visualizer")

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("reference_input_topic", "/benchmark/reference_path_cmd")
        self.declare_parameter("selection_topic", "/benchmark/selection")
        self.declare_parameter("clear_topic", "/benchmark/clear_trajectory")
        self.declare_parameter("reference_output_topic", "/benchmark/reference_path")
        self.declare_parameter("robot_path_topic", "/benchmark/robot_path")
        self.declare_parameter("markers_topic", "/benchmark/markers")

        self.frame_id = str(self.get_parameter("frame_id").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        reference_input_topic = str(self.get_parameter("reference_input_topic").value)
        selection_topic = str(self.get_parameter("selection_topic").value)
        clear_topic = str(self.get_parameter("clear_topic").value)
        reference_output_topic = str(self.get_parameter("reference_output_topic").value)
        robot_path_topic = str(self.get_parameter("robot_path_topic").value)
        markers_topic = str(self.get_parameter("markers_topic").value)

        self.reference_path = Path()
        self.reference_path.header.frame_id = self.frame_id
        self.robot_path = Path()
        self.robot_path.header.frame_id = self.frame_id
        self.selected_label = "Path A"
        self.selected_angle_deg = 45.0
        self._last_xy: tuple[float, float] | None = None

        self.reference_pub = self.create_publisher(Path, reference_output_topic, 10)
        self.robot_path_pub = self.create_publisher(Path, robot_path_topic, 10)
        self.markers_pub = self.create_publisher(MarkerArray, markers_topic, 10)

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 20)
        self.create_subscription(Path, reference_input_topic, self._reference_cb, 10)
        self.create_subscription(String, selection_topic, self._selection_cb, 10)
        self.create_subscription(Empty, clear_topic, self._clear_cb, 10)

        self.marker_timer = self.create_timer(0.5, self._publish_markers)

    def _selection_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.selected_label = str(payload.get("path_name", self.selected_label))
        self.selected_angle_deg = float(payload.get("angle_deg", self.selected_angle_deg))
        self._publish_markers()

    def _reference_cb(self, msg: Path) -> None:
        self.reference_path = msg
        if not self.reference_path.header.frame_id:
            self.reference_path.header.frame_id = self.frame_id
        self.reference_pub.publish(self.reference_path)
        self._publish_markers()

    def _clear_cb(self, _: Empty) -> None:
        self.robot_path = Path()
        self.robot_path.header.frame_id = self.frame_id
        self._last_xy = None
        self.robot_path_pub.publish(self.robot_path)

    def _odom_cb(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose

        xy = (float(pose.pose.position.x), float(pose.pose.position.y))
        if self._last_xy is not None:
            dx = xy[0] - self._last_xy[0]
            dy = xy[1] - self._last_xy[1]
            if dx * dx + dy * dy < 0.0004:
                return

        self._last_xy = xy
        self.robot_path.header.frame_id = self.frame_id
        self.robot_path.poses.append(pose)
        if len(self.robot_path.poses) > 3000:
            self.robot_path.poses = self.robot_path.poses[-3000:]
        self.robot_path_pub.publish(self.robot_path)

    def _publish_markers(self) -> None:
        markers = MarkerArray()

        if self.reference_path.poses:
            start = self.reference_path.poses[0].pose.position
            end = self.reference_path.poses[-1].pose.position

            start_marker = Marker()
            start_marker.header.frame_id = self.frame_id
            start_marker.ns = "benchmark_points"
            start_marker.id = 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose.position = start
            start_marker.pose.orientation.w = 1.0
            start_marker.scale.x = 0.18
            start_marker.scale.y = 0.18
            start_marker.scale.z = 0.18
            start_marker.color.a = 1.0
            start_marker.color.r = 0.2
            start_marker.color.g = 0.85
            start_marker.color.b = 0.25
            markers.markers.append(start_marker)

            end_marker = Marker()
            end_marker.header.frame_id = self.frame_id
            end_marker.ns = "benchmark_points"
            end_marker.id = 2
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            end_marker.pose.position = end
            end_marker.pose.orientation.w = 1.0
            end_marker.scale.x = 0.18
            end_marker.scale.y = 0.18
            end_marker.scale.z = 0.18
            end_marker.color.a = 1.0
            end_marker.color.r = 0.9
            end_marker.color.g = 0.25
            end_marker.color.b = 0.25
            markers.markers.append(end_marker)

            label_marker = Marker()
            label_marker.header.frame_id = self.frame_id
            label_marker.ns = "benchmark_labels"
            label_marker.id = 3
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.pose.position = end
            label_marker.pose.position.z += 0.4
            label_marker.pose.orientation.w = 1.0
            label_marker.scale.z = 0.22
            label_marker.color.a = 1.0
            label_marker.color.r = 1.0
            label_marker.color.g = 1.0
            label_marker.color.b = 1.0
            label_marker.text = f"{self.selected_label} ({self.selected_angle_deg:.0f} deg)"
            markers.markers.append(label_marker)

        self.markers_pub.publish(markers)


def main() -> None:
    rclpy.init()
    node = BenchmarkVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
