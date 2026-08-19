from __future__ import annotations

import json
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Empty, String

from .benchmark_paths import PATH_PRESETS, build_path


class BenchmarkGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("benchmark_gui")

        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("follow_path_action", "follow_path")
        self.declare_parameter("selection_topic", "/benchmark/selection")
        self.declare_parameter("reference_input_topic", "/benchmark/reference_path_cmd")
        self.declare_parameter("clear_topic", "/benchmark/clear_trajectory")
        self.declare_parameter("controller_ids", ["PP", "APP", "RPP", "DWPP"])
        self.declare_parameter("world_name", "minimal")
        self.declare_parameter("model_name", "simple_marl_car")
        self.declare_parameter("warp_service", "")
        self.declare_parameter("warp_z", 0.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.controller_ids = list(self.get_parameter("controller_ids").value)
        self.world_name = str(self.get_parameter("world_name").value)
        self.model_name = str(self.get_parameter("model_name").value)
        warp_service = str(self.get_parameter("warp_service").value).strip()
        if not warp_service:
            warp_service = f"/world/{self.world_name}/set_pose"
        self.warp_service = warp_service
        self.warp_z = float(self.get_parameter("warp_z").value)

        follow_path_action = str(self.get_parameter("follow_path_action").value)
        selection_topic = str(self.get_parameter("selection_topic").value)
        reference_input_topic = str(self.get_parameter("reference_input_topic").value)
        clear_topic = str(self.get_parameter("clear_topic").value)

        self.follow_path_client = ActionClient(self, FollowPath, follow_path_action)
        self.selection_pub = self.create_publisher(String, selection_topic, 10)
        self.reference_pub = self.create_publisher(Path, reference_input_topic, 10)
        self.clear_pub = self.create_publisher(Empty, clear_topic, 10)
        self.warp_client = self.create_client(SetEntityPose, self.warp_service)

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._status_callback = None

    def set_status_callback(self, cb) -> None:
        self._status_callback = cb

    def _set_status(self, text: str) -> None:
        self.get_logger().info(text)
        if self._status_callback is not None:
            self._status_callback(text)

    def publish_selection(self, *, path_name: str, angle_deg: float, controller_id: str) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "path_name": path_name,
                "angle_deg": float(angle_deg),
                "controller_id": controller_id,
            },
            ensure_ascii=True,
        )
        self.selection_pub.publish(msg)

    def clear_trajectory(self) -> None:
        self.clear_pub.publish(Empty())
        self._set_status("trajectory cleared")

    def warp_to_origin(self) -> None:
        if not self.warp_client.wait_for_service(timeout_sec=0.5):
            self._set_status(f"warp service unavailable: {self.warp_service}")
            return

        req = SetEntityPose.Request()
        req.entity = Entity()
        req.entity.name = self.model_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = 0.0
        req.pose.position.y = 0.0
        req.pose.position.z = self.warp_z
        req.pose.orientation.w = 1.0

        future = self.warp_client.call_async(req)

        def _done(fut):
            try:
                result = fut.result()
            except Exception as exc:  # pragma: no cover - runtime callback
                self._set_status(f"warp failed: {exc}")
                return
            if result is not None and result.success:
                self._set_status("robot warped to origin")
            else:
                self._set_status("warp request returned failure")

        future.add_done_callback(_done)

    def cancel_goal(self) -> None:
        with self._goal_lock:
            goal_handle = self._goal_handle
        if goal_handle is None:
            self._set_status("no active goal")
            return
        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _: self._set_status("cancel requested"))

    def send_path(self, *, path_name: str, angle_deg: float, controller_id: str) -> None:
        if not self.follow_path_client.wait_for_server(timeout_sec=1.0):
            self._set_status("follow_path action server unavailable")
            return

        path = build_path(frame_id=self.frame_id, angle_deg=angle_deg)
        self.reference_pub.publish(path)
        self.publish_selection(path_name=path_name, angle_deg=angle_deg, controller_id=controller_id)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = controller_id
        goal.goal_checker_id = ""
        goal.progress_checker_id = ""

        send_future = self.follow_path_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(
            lambda fut: self._goal_response_cb(fut, controller_id=controller_id, path_name=path_name, angle_deg=angle_deg)
        )
        self._set_status(f"sending {path_name} at {angle_deg:.0f} deg with {controller_id}")

    def _goal_response_cb(self, future, *, controller_id: str, path_name: str, angle_deg: float) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - runtime callback
            self._set_status(f"goal send failed: {exc}")
            return
        if not goal_handle.accepted:
            self._set_status("follow_path goal rejected")
            return
        with self._goal_lock:
            self._goal_handle = goal_handle
        self._set_status(f"active: {path_name} {angle_deg:.0f} deg on {controller_id}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self._set_status(
            f"feedback: dist={feedback.distance_to_goal:.2f} speed={feedback.speed:.2f}"
        )

    def _result_cb(self, future) -> None:
        with self._goal_lock:
            self._goal_handle = None
        try:
            wrapped = future.result()
        except Exception as exc:  # pragma: no cover - runtime callback
            self._set_status(f"goal result failed: {exc}")
            return
        result = wrapped.result
        if result.error_code == FollowPath.Result.NONE:
            self._set_status("follow_path succeeded")
        else:
            self._set_status(f"follow_path failed: {result.error_code} {result.error_msg}")


def run_gui() -> None:
    rclpy.init()
    node = BenchmarkGuiNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title("Benchmark Control")
    root.geometry("360x420")

    controller_var = tk.StringVar(value=node.controller_ids[0] if node.controller_ids else "PP")
    angle_var = tk.DoubleVar(value=PATH_PRESETS["B"])
    status_var = tk.StringVar(value="ready")

    def set_status(text: str) -> None:
        root.after(0, lambda: status_var.set(text))

    node.set_status_callback(set_status)

    frame = ttk.Frame(root, padding=12)
    frame.pack(fill=tk.BOTH, expand=True)

    ttk.Label(frame, text="Controller").pack(anchor=tk.W)
    controller_combo = ttk.Combobox(frame, textvariable=controller_var, values=node.controller_ids, state="readonly")
    controller_combo.pack(fill=tk.X, pady=(0, 12))

    ttk.Label(frame, text="Corner Angle (deg)").pack(anchor=tk.W)
    angle_scale = ttk.Scale(frame, from_=30.0, to=150.0, orient=tk.HORIZONTAL, variable=angle_var)
    angle_scale.pack(fill=tk.X)
    angle_value = ttk.Label(frame, text=f"{angle_var.get():.0f} deg")
    angle_value.pack(anchor=tk.E, pady=(2, 12))

    def _update_angle_label(*_args) -> None:
        angle_value.config(text=f"{angle_var.get():.0f} deg")

    angle_var.trace_add("write", _update_angle_label)

    preset_frame = ttk.LabelFrame(frame, text="Preset Paths", padding=8)
    preset_frame.pack(fill=tk.X, pady=(0, 12))

    def run_preset(name: str) -> None:
        angle = PATH_PRESETS[name]
        angle_var.set(angle)
        node.send_path(path_name=f"Path {name}", angle_deg=angle, controller_id=controller_var.get())

    ttk.Button(preset_frame, text="Path A (45 deg)", command=lambda: run_preset("A")).pack(fill=tk.X, pady=2)
    ttk.Button(preset_frame, text="Path B (90 deg)", command=lambda: run_preset("B")).pack(fill=tk.X, pady=2)
    ttk.Button(preset_frame, text="Path C (135 deg)", command=lambda: run_preset("C")).pack(fill=tk.X, pady=2)

    ttk.Button(
        frame,
        text="Run Current Angle",
        command=lambda: node.send_path(
            path_name="Custom Angle",
            angle_deg=angle_var.get(),
            controller_id=controller_var.get(),
        ),
    ).pack(fill=tk.X, pady=2)

    ttk.Button(frame, text="Warp Robot (0,0,0)", command=node.warp_to_origin).pack(fill=tk.X, pady=2)
    ttk.Button(frame, text="Clear Trajectory", command=node.clear_trajectory).pack(fill=tk.X, pady=2)
    ttk.Button(frame, text="Cancel / Stop", command=node.cancel_goal).pack(fill=tk.X, pady=2)

    ttk.Separator(frame).pack(fill=tk.X, pady=10)
    ttk.Label(frame, text="Status").pack(anchor=tk.W)
    ttk.Label(frame, textvariable=status_var, wraplength=320, justify=tk.LEFT).pack(fill=tk.X)

    def _shutdown() -> None:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _shutdown)
    root.mainloop()


def main() -> None:
    run_gui()


if __name__ == "__main__":
    main()
