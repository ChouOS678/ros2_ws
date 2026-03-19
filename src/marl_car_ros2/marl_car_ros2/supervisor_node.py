from __future__ import annotations

import json
import math
import time
from collections import deque
from typing import Deque, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .agent_trace import AgentEventPayload, AgentState, AgentStatusPayload, new_event_id, new_trace_id, timeline_fields
from .decision_protocol import AgentDecision, DecisionType
from .observation_builder import ObservationBuilder


class SupervisorNode(Node):
    """
    Safety arbiter and final motion authority.

    Input command source (Nav2/baseline) should publish to /cmd_vel_nav.
    This node validates decisions + safety constraints and publishes final /cmd_vel.
    """

    def __init__(self) -> None:
        super().__init__("supervisor_node")

        self.declare_parameter("step_hz", 10.0)
        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("scan_bins", 24)
        self.declare_parameter("max_scan_range", 8.0)
        self.declare_parameter("decision_timeout_s", 1.0)
        self.declare_parameter("hard_stop_range", 0.24)
        self.declare_parameter("cautious_speed_scale", 0.55)
        self.declare_parameter("no_progress_timeout_s", 6.0)
        self.declare_parameter("no_progress_dist_eps", 0.10)
        self.declare_parameter("goal_tolerance", 0.25)

        step_hz = float(self.get_parameter("step_hz").value)
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        scan_bins = int(self.get_parameter("scan_bins").value)
        max_scan_range = float(self.get_parameter("max_scan_range").value)

        self.decision_timeout_s = float(self.get_parameter("decision_timeout_s").value)
        self.hard_stop_range = float(self.get_parameter("hard_stop_range").value)
        self.cautious_speed_scale = float(self.get_parameter("cautious_speed_scale").value)
        self.no_progress_timeout_s = float(self.get_parameter("no_progress_timeout_s").value)
        self.no_progress_dist_eps = float(self.get_parameter("no_progress_dist_eps").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)

        self.obs_builder = ObservationBuilder(
            self,
            goal_x=goal_x,
            goal_y=goal_y,
            scan_bins=scan_bins,
            max_scan_range=max_scan_range,
        )

        self.create_subscription(String, "/agent/decision", self._decision_cb, 20)
        self.create_subscription(Twist, "/cmd_vel_nav", self._nav_cmd_cb, 20)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 20)
        self.supervisor_status_pub = self.create_publisher(String, "/supervisor/status", 20)
        self.replan_pub = self.create_publisher(String, "/supervisor/replan_request", 10)
        self.recovery_pub = self.create_publisher(String, "/supervisor/recovery_request", 10)
        self.agent_status_pub = self.create_publisher(String, "/agents/status", 20)
        self.agent_events_pub = self.create_publisher(String, "/agents/events", 40)
        self.query_srv = self.create_service(Trigger, "/supervisor/query_state", self._query_state_cb)

        self.trace_id = new_trace_id()
        self.mission_id = "mission-default"

        self.latest_decision: Optional[AgentDecision] = None
        self.latest_decision_wall = 0.0
        self.latest_nav_cmd = Twist()
        self.latest_nav_cmd_wall = 0.0

        self.override_count = 0
        self.replan_count = 0
        self.recovery_count = 0
        self.blocked_duration_s = 0.0
        self.blocked_since: Optional[float] = None
        self.mission_status = "running"

        self._last_override_reason = ""
        self._last_override_event_wall = 0.0
        self._last_replan_wall = 0.0
        self._last_recovery_wall = 0.0

        self.pose_history: Deque[Tuple[float, float, float]] = deque(maxlen=200)
        self.state = AgentState.IDLE.value
        self.step_count = 0

        self.timer = self.create_timer(1.0 / max(step_hz, 1e-3), self._step)

    def _decision_cb(self, msg: String) -> None:
        try:
            decision = AgentDecision.from_json(msg.data)
        except Exception:
            self.get_logger().warn("invalid decision payload; ignored")
            return
        self.latest_decision = decision
        self.latest_decision_wall = time.time()

    def _nav_cmd_cb(self, msg: Twist) -> None:
        self.latest_nav_cmd = msg
        self.latest_nav_cmd_wall = time.time()

    def _step(self) -> None:
        self.step_count += 1
        now = time.time()
        snapshot = self.obs_builder.build_snapshot()

        nav_cmd = self.latest_nav_cmd
        final_cmd = Twist()
        override_reason = ""
        decision_type = DecisionType.FALLBACK_TO_NAV2

        if snapshot is not None:
            self.pose_history.append((now, float(snapshot.pose_x), float(snapshot.pose_y)))

        decision = self._active_decision(now)
        if decision is not None:
            decision_type = decision.decision_type

        if snapshot is None:
            final_cmd = Twist()
            override_reason = "sensor_not_ready"
            self.state = AgentState.WAITING.value
        elif snapshot.min_range <= self.hard_stop_range:
            final_cmd = Twist()
            override_reason = f"hard_stop:{snapshot.min_range:.2f}"
            self.state = AgentState.BLOCKED.value
        else:
            final_cmd, override_reason = self._apply_decision(decision, nav_cmd)

        no_progress = self._detect_no_progress(now)
        if no_progress:
            final_cmd = Twist()
            if not override_reason:
                override_reason = "no_progress_timeout"
            self._emit_recovery(now, reason=override_reason)
            self.state = AgentState.BLOCKED.value

        if snapshot is not None and snapshot.goal_dist <= self.goal_tolerance:
            self.mission_status = "success"
        elif override_reason == "sensor_not_ready":
            self.mission_status = "degraded"

        self._update_blocked_duration(now, bool(override_reason))
        self.cmd_pub.publish(final_cmd)

        if override_reason:
            self.override_count += 1
            self._emit_override_event(now, decision_type=decision_type, reason=override_reason)

        self._publish_supervisor_status(now, decision_type, override_reason, snapshot)
        self._publish_agent_status(now, decision_type, override_reason, snapshot)

    def _active_decision(self, now: float) -> Optional[AgentDecision]:
        if self.latest_decision is None:
            return None
        if now - self.latest_decision_wall > self.decision_timeout_s:
            return None
        return self.latest_decision

    def _apply_decision(self, decision: Optional[AgentDecision], nav_cmd: Twist) -> Tuple[Twist, str]:
        if decision is None:
            self.state = AgentState.EXECUTING.value
            return nav_cmd, ""

        out = Twist()
        typ = decision.decision_type

        if typ in (DecisionType.NORMAL_NAVIGATION, DecisionType.FALLBACK_TO_NAV2):
            self.state = AgentState.EXECUTING.value
            return nav_cmd, ""

        if typ == DecisionType.CAUTIOUS_MODE:
            scale = float(decision.constraints.get("speed_scale", self.cautious_speed_scale))
            out.linear.x = float(nav_cmd.linear.x) * scale
            out.angular.z = float(nav_cmd.angular.z) * scale
            self.state = AgentState.EXECUTING.value
            return out, f"cautious_mode:{scale:.2f}"

        if typ == DecisionType.PAUSE_AND_WAIT:
            self.state = AgentState.WAITING.value
            return out, "pause_and_wait"

        if typ == DecisionType.TRIGGER_REPLAN:
            self.state = AgentState.PLANNING.value
            self._emit_replan(time.time(), reason=decision.reason)
            return out, "trigger_replan"

        if typ == DecisionType.RECOVERY_REQUEST:
            self.state = AgentState.BLOCKED.value
            self._emit_recovery(time.time(), reason=decision.reason)
            return out, "recovery_request"

        self.state = AgentState.EXECUTING.value
        return nav_cmd, ""

    def _detect_no_progress(self, now: float) -> bool:
        if len(self.pose_history) < 2:
            return False

        latest_t, latest_x, latest_y = self.pose_history[-1]
        oldest_t, oldest_x, oldest_y = self.pose_history[0]
        while len(self.pose_history) >= 2 and latest_t - oldest_t > self.no_progress_timeout_s:
            self.pose_history.popleft()
            oldest_t, oldest_x, oldest_y = self.pose_history[0]

        duration = latest_t - oldest_t
        if duration < self.no_progress_timeout_s:
            return False

        displacement = math.hypot(latest_x - oldest_x, latest_y - oldest_y)
        cmd_mag = abs(float(self.latest_nav_cmd.linear.x)) + abs(float(self.latest_nav_cmd.angular.z))
        return bool(cmd_mag > 0.1 and displacement < self.no_progress_dist_eps)

    def _emit_override_event(self, now: float, *, decision_type: str, reason: str) -> None:
        if reason == self._last_override_reason and now - self._last_override_event_wall < 0.8:
            return
        self._last_override_reason = reason
        self._last_override_event_wall = now
        self._publish_event(
            event_type="supervisor_override",
            result=decision_type,
            details={"reason": reason, "override_count": self.override_count},
        )

    def _emit_replan(self, now: float, *, reason: str) -> None:
        if now - self._last_replan_wall < 1.0:
            return
        self._last_replan_wall = now
        self.replan_count += 1
        msg = String()
        msg.data = json.dumps({"reason": reason, "wall_time": now}, ensure_ascii=True)
        self.replan_pub.publish(msg)
        self._publish_event(
            event_type="replan_requested",
            result="requested",
            details={"reason": reason, "replan_count": self.replan_count},
        )

    def _emit_recovery(self, now: float, *, reason: str) -> None:
        if now - self._last_recovery_wall < 1.0:
            return
        self._last_recovery_wall = now
        self.recovery_count += 1
        msg = String()
        msg.data = json.dumps({"reason": reason, "wall_time": now}, ensure_ascii=True)
        self.recovery_pub.publish(msg)
        self._publish_event(
            event_type="recovery_requested",
            result="requested",
            details={"reason": reason, "recovery_count": self.recovery_count},
        )

    def _update_blocked_duration(self, now: float, is_blocked: bool) -> None:
        if is_blocked and self.blocked_since is None:
            self.blocked_since = now
        if not is_blocked and self.blocked_since is not None:
            self.blocked_duration_s += now - self.blocked_since
            self.blocked_since = None

    def _publish_supervisor_status(self, now: float, decision_type: str, override_reason: str, snapshot) -> None:
        payload = {
            "wall_time": now,
            "trace_id": self.trace_id,
            "mission_id": self.mission_id,
            "state": self.state,
            "decision_type": decision_type,
            "override_reason": override_reason,
            "override_count": self.override_count,
            "replan_count": self.replan_count,
            "recovery_count": self.recovery_count,
            "blocked_duration_s": self.blocked_duration_s,
            "mission_status": self.mission_status,
            "goal_dist": float(snapshot.goal_dist) if snapshot is not None else -1.0,
            "min_range": float(snapshot.min_range) if snapshot is not None else -1.0,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.supervisor_status_pub.publish(msg)

    def _publish_agent_status(self, now: float, decision_type: str, override_reason: str, snapshot) -> None:
        t = timeline_fields(self)
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        world_summary = {"goal_dist": -1.0, "min_range": -1.0, "stuck": 0.0}
        if snapshot is not None:
            pose = {"x": float(snapshot.pose_x), "y": float(snapshot.pose_y), "yaw": float(snapshot.yaw)}
            world_summary = {
                "goal_dist": float(snapshot.goal_dist),
                "min_range": float(snapshot.min_range),
                "stuck": 1.0 if self.state == AgentState.BLOCKED.value else 0.0,
            }

        payload = AgentStatusPayload(
            agent_id="supervisor",
            role="safety_arbiter",
            state=self.state,
            current_goal="enforce_safe_execution",
            current_subtask=decision_type,
            progress=float(min(self.step_count / 1200.0, 1.0)),
            health=1.0 if self.state != AgentState.BLOCKED.value else 0.7,
            last_heartbeat_ts=t["wall_time"],
            task_id=self.mission_id,
            parent_task_id="",
            owner_agent="supervisor",
            dependencies=["task_agent_decision", "nav_cmd"],
            queue_backlog=0,
            blocked_reason=override_reason if self.state == AgentState.BLOCKED.value else "",
            trace_id=self.trace_id,
            correlation_id=f"{self.mission_id}-step-{self.step_count}",
            sim_time=t["sim_time"],
            wall_time=t["wall_time"],
            robot_pose=pose,
            world_state_summary=world_summary,
        )
        msg = String()
        msg.data = payload.to_json()
        self.agent_status_pub.publish(msg)

    def _publish_event(self, *, event_type: str, result: str, details: Dict[str, object]) -> None:
        t = timeline_fields(self)
        payload = AgentEventPayload(
            event_id=new_event_id(),
            event_type=event_type,
            sender="supervisor",
            receiver="monitor",
            message_type="control",
            phase="feedback",
            latency_ms=0.0,
            timeout_ms=250.0,
            retry_count=0,
            task_id=self.mission_id,
            trace_id=self.trace_id,
            correlation_id=f"{self.mission_id}-step-{self.step_count}",
            result=result,
            failure_reason="",
            details=details,
            sim_time=t["sim_time"],
            wall_time=t["wall_time"],
        )
        msg = String()
        msg.data = payload.to_json()
        self.agent_events_pub.publish(msg)

    def _query_state_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        response.success = True
        response.message = json.dumps(
            {
                "state": self.state,
                "mission_id": self.mission_id,
                "trace_id": self.trace_id,
                "override_count": self.override_count,
                "replan_count": self.replan_count,
                "recovery_count": self.recovery_count,
                "blocked_duration_s": self.blocked_duration_s,
                "mission_status": self.mission_status,
                "last_decision": self.latest_decision.__dict__ if self.latest_decision else {},
                "query_wall_time": time.time(),
            },
            ensure_ascii=True,
        )
        return response


def main() -> None:
    rclpy.init()
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
