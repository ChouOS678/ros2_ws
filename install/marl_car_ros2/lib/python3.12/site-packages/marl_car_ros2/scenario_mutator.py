from __future__ import annotations

import json
import random
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState, SetLinkProperties, SpawnEntity

    GAZEBO_AVAILABLE = True
except ImportError:
    GAZEBO_AVAILABLE = False


@dataclass
class EventSpec:
    event_id: str
    event_type: str
    severity: float
    started_at: float
    duration_s: float


class ScenarioMutator(Node):
    """
    Scenario/disturbance injector for reproducible experiments.

    Supported events:
    1) ghost_probe
    2) friction_drop
    """

    def __init__(self) -> None:
        super().__init__("scenario_mutator")

        self.declare_parameter("tick_hz", 4.0)
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("enable_ghost_event", True)
        self.declare_parameter("enable_friction_event", True)
        self.declare_parameter("ghost_event_prob", 0.05)
        self.declare_parameter("friction_event_prob", 0.03)
        self.declare_parameter("event_cooldown_s", 4.0)
        self.declare_parameter("ghost_speed", 1.0)
        self.declare_parameter("ground_link_name", "ground_plane::link")
        self.declare_parameter("friction_low", 0.12)
        self.declare_parameter("friction_restore", 1.0)
        self.declare_parameter("friction_duration_s", 6.0)

        self.tick_hz = float(self.get_parameter("tick_hz").value)
        random_seed = int(self.get_parameter("random_seed").value)
        self.enable_ghost_event = bool(self.get_parameter("enable_ghost_event").value)
        self.enable_friction_event = bool(self.get_parameter("enable_friction_event").value)
        self.ghost_prob = float(self.get_parameter("ghost_event_prob").value)
        self.friction_prob = float(self.get_parameter("friction_event_prob").value)
        self.cooldown_s = float(self.get_parameter("event_cooldown_s").value)
        self.ghost_speed = float(self.get_parameter("ghost_speed").value)
        self.ground_link_name = str(self.get_parameter("ground_link_name").value)
        self.friction_low = float(self.get_parameter("friction_low").value)
        self.friction_restore = float(self.get_parameter("friction_restore").value)
        self.friction_duration_s = float(self.get_parameter("friction_duration_s").value)

        self.rng = random.Random(random_seed if random_seed >= 0 else None)

        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.event_pub = self.create_publisher(String, "/world_model/events", 10)
        self.perturb_pub = self.create_publisher(Float32, "/world_model/perturbation_level", 10)

        self.latest_odom: Optional[Odometry] = None
        self.last_event_ts = 0.0
        self.active_event: Optional[EventSpec] = None
        self.ghost_spawned = False

        self.spawn_cli = None
        self.entity_state_cli = None
        self.link_prop_cli = None
        if GAZEBO_AVAILABLE:
            self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
            self.entity_state_cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")
            self.link_prop_cli = self.create_client(SetLinkProperties, "/gazebo/set_link_properties")

        self.timer = self.create_timer(1.0 / max(self.tick_hz, 1e-3), self._tick)
        self.get_logger().info(
            f"scenario mutator ready, gazebo_msgs available={GAZEBO_AVAILABLE}, seed={random_seed}"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def _tick(self) -> None:
        now = time.time()
        if self.active_event is not None and now >= self.active_event.started_at + self.active_event.duration_s:
            self._end_event(self.active_event)
            self.active_event = None

        if now - self.last_event_ts < self.cooldown_s:
            return
        if self.latest_odom is None:
            return

        p = self.rng.random()
        if self.enable_ghost_event and p < self.ghost_prob:
            self._trigger_ghost_probe(now)
            return

        if self.enable_friction_event and p < self.ghost_prob + self.friction_prob:
            self._trigger_friction_drop(now)

    def _trigger_ghost_probe(self, now_ts: float) -> None:
        odom = self.latest_odom
        if odom is None:
            return

        ex = float(odom.pose.pose.position.x)
        ey = float(odom.pose.pose.position.y)

        crossing_x = ex + self.rng.uniform(1.8, 3.2)
        crossing_y = ey + self.rng.choice([-1.0, 1.0]) * self.rng.uniform(0.6, 1.2)
        direction = -1.0 if crossing_y > ey else 1.0

        severity = self.rng.uniform(0.55, 0.95)
        self.active_event = EventSpec(
            event_id=f"ghost-{int(now_ts * 1000)}",
            event_type="ghost_probe",
            severity=severity,
            started_at=now_ts,
            duration_s=3.5,
        )
        self.last_event_ts = now_ts

        if GAZEBO_AVAILABLE:
            self._spawn_or_move_ghost(
                x=crossing_x,
                y=crossing_y,
                vx=0.0,
                vy=direction * self.ghost_speed * (0.6 + 0.8 * severity),
            )

        self._publish_event(
            {
                "event_id": self.active_event.event_id,
                "type": self.active_event.event_type,
                "severity": severity,
                "position_hint": {"x": crossing_x, "y": crossing_y},
                "duration_s": self.active_event.duration_s,
                "status": "start",
            }
        )

    def _trigger_friction_drop(self, now_ts: float) -> None:
        severity = self.rng.uniform(0.5, 1.0)
        mu = max(0.05, self.friction_low * (1.1 - 0.8 * severity))
        self.active_event = EventSpec(
            event_id=f"friction-{int(now_ts * 1000)}",
            event_type="friction_drop",
            severity=severity,
            started_at=now_ts,
            duration_s=self.friction_duration_s,
        )
        self.last_event_ts = now_ts

        if GAZEBO_AVAILABLE:
            self._set_friction(mu)

        self._publish_event(
            {
                "event_id": self.active_event.event_id,
                "type": self.active_event.event_type,
                "severity": severity,
                "target_mu": mu,
                "duration_s": self.active_event.duration_s,
                "status": "start",
            }
        )

    def _end_event(self, spec: EventSpec) -> None:
        if spec.event_type == "friction_drop" and GAZEBO_AVAILABLE:
            self._set_friction(self.friction_restore)
        self._publish_event(
            {
                "event_id": spec.event_id,
                "type": spec.event_type,
                "severity": spec.severity,
                "status": "end",
            }
        )

    def _spawn_or_move_ghost(self, x: float, y: float, vx: float, vy: float) -> None:
        if self.spawn_cli is None or self.entity_state_cli is None:
            return

        ghost_name = "ghost_probe_actor"
        if not self.ghost_spawned:
            req = SpawnEntity.Request()
            req.name = ghost_name
            req.reference_frame = "world"
            req.initial_pose = Pose()
            req.initial_pose.position.x = x
            req.initial_pose.position.y = y
            req.initial_pose.position.z = 0.2
            req.xml = self._ghost_sdf()
            if self.spawn_cli.wait_for_service(timeout_sec=0.4):
                self.spawn_cli.call_async(req)
                self.ghost_spawned = True

        state_req = SetEntityState.Request()
        state_req.state = EntityState()
        state_req.state.name = ghost_name
        state_req.state.pose.position.x = x
        state_req.state.pose.position.y = y
        state_req.state.pose.position.z = 0.2
        state_req.state.twist = Twist()
        state_req.state.twist.linear.x = vx
        state_req.state.twist.linear.y = vy
        if self.entity_state_cli.wait_for_service(timeout_sec=0.2):
            self.entity_state_cli.call_async(state_req)

    def _set_friction(self, mu: float) -> None:
        if self.link_prop_cli is None:
            return

        req = SetLinkProperties.Request()
        req.link_name = self.ground_link_name
        req.gravity_mode = True
        req.com = Pose().position
        req.mass = 1.0
        req.ixx = 1.0
        req.iyy = 1.0
        req.izz = 1.0
        req.ixy = 0.0
        req.ixz = 0.0
        req.iyz = 0.0
        req.mu1 = float(mu)
        req.mu2 = float(mu)
        if self.link_prop_cli.wait_for_service(timeout_sec=0.4):
            self.link_prop_cli.call_async(req)

    def _publish_event(self, payload: Dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.event_pub.publish(msg)

        level = Float32()
        level.data = float(payload.get("severity", 0.0))
        self.perturb_pub.publish(level)

    @staticmethod
    def _ghost_sdf() -> str:
        return """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='ghost_probe_actor'>
    <static>false</static>
    <link name='link'>
      <pose>0 0 0.2 0 0 0</pose>
      <inertial><mass>4.0</mass></inertial>
      <collision name='collision'>
        <geometry><box><size>0.35 0.35 0.4</size></box></geometry>
      </collision>
      <visual name='visual'>
        <geometry><box><size>0.35 0.35 0.4</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>"""


def main() -> None:
    rclpy.init()
    node = ScenarioMutator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
