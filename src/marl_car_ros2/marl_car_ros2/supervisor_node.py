from __future__ import annotations

import json
import math
import time
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .agent_trace import AgentEventPayload, AgentState, AgentStatusPayload, new_event_id, new_trace_id, timeline_fields
from .decision_protocol import AgentDecision, DecisionType, ManagerMode, mode_from_decision_type
from .observation_builder import ObservationBuilder
from .shared_types import BlockReason, RecoveryReason, block_reason_from_text, recovery_reason_from_text


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
        self.declare_parameter("mode_cautious_range", 0.85)
        self.declare_parameter("mode_medium_range", 1.20)
        self.declare_parameter("risk_cautious_severity", 0.55)
        self.declare_parameter("risk_blocked_severity", 0.85)
        self.declare_parameter("trend_window_s", 2.5)
        self.declare_parameter("trend_goal_progress_min", 0.05)
        self.declare_parameter("trend_heading_progress_min", 0.08)
        self.declare_parameter("oscillation_window_s", 2.5)
        self.declare_parameter("oscillation_min_sign_flips", 4)
        self.declare_parameter("oscillation_max_disp", 0.12)
        self.declare_parameter("oscillation_max_lin_speed", 0.08)
        self.declare_parameter("oscillation_min_ang_speed", 0.35)
        self.declare_parameter("blocked_to_recovery_s", 2.5)
        self.declare_parameter("replan_fail_limit", 2)
        self.declare_parameter("replan_grace_s", 2.0)

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
        self.mode_cautious_range = float(self.get_parameter("mode_cautious_range").value)
        self.mode_medium_range = float(self.get_parameter("mode_medium_range").value)
        self.risk_cautious_severity = float(self.get_parameter("risk_cautious_severity").value)
        self.risk_blocked_severity = float(self.get_parameter("risk_blocked_severity").value)
        self.trend_window_s = float(self.get_parameter("trend_window_s").value)
        self.trend_goal_progress_min = float(self.get_parameter("trend_goal_progress_min").value)
        self.trend_heading_progress_min = float(self.get_parameter("trend_heading_progress_min").value)
        self.oscillation_window_s = float(self.get_parameter("oscillation_window_s").value)
        self.oscillation_min_sign_flips = int(self.get_parameter("oscillation_min_sign_flips").value)
        self.oscillation_max_disp = float(self.get_parameter("oscillation_max_disp").value)
        self.oscillation_max_lin_speed = float(self.get_parameter("oscillation_max_lin_speed").value)
        self.oscillation_min_ang_speed = float(self.get_parameter("oscillation_min_ang_speed").value)
        self.blocked_to_recovery_s = float(self.get_parameter("blocked_to_recovery_s").value)
        self.replan_fail_limit = int(self.get_parameter("replan_fail_limit").value)
        self.replan_grace_s = float(self.get_parameter("replan_grace_s").value)

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
        self.metric_history: Deque[Dict[str, float]] = deque(maxlen=300)
        self.state = AgentState.IDLE.value
        self.mode_state = ManagerMode.BLOCKED
        self.mode_enter_wall = time.time()
        self.last_mode_reason = "boot"
        self.replan_pending_since: Optional[float] = None
        self.replan_failure_streak = 0
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
        advisory_mode = ManagerMode.NOMINAL

        if snapshot is not None:
            self.pose_history.append((now, float(snapshot.pose_x), float(snapshot.pose_y)))

        decision = self._active_decision(now)
        if decision is not None:
            decision_type = decision.decision_type
            advisory_mode = decision.manager_mode or mode_from_decision_type(decision_type)
        else:
            advisory_mode = mode_from_decision_type(decision_type)

        diagnostics = self._compute_mode_diagnostics(now, snapshot)
        manager_mode, mode_reason = self._transition_mode(now, snapshot, advisory_mode, diagnostics)
        self.mode_state = manager_mode
        self.last_mode_reason = mode_reason

        final_cmd, override_reason = self._apply_mode(
            now=now,
            mode=manager_mode,
            reason=mode_reason,
            decision=decision,
            nav_cmd=nav_cmd,
            snapshot=snapshot,
        )

        if snapshot is not None and snapshot.goal_dist <= self.goal_tolerance:
            self.mission_status = "success"
        elif override_reason == "sensor_not_ready":
            self.mission_status = "degraded"

        self._update_blocked_duration(now, bool(override_reason))
        self.cmd_pub.publish(final_cmd)

        if override_reason:
            self.override_count += 1
            self._emit_override_event(now, decision_type=decision_type, reason=override_reason)

        self._publish_supervisor_status(now, decision_type, manager_mode, override_reason, snapshot)
        self._publish_agent_status(now, decision_type, manager_mode, override_reason, snapshot)

    def _active_decision(self, now: float) -> Optional[AgentDecision]:
        if self.latest_decision is None:
            return None
        if now - self.latest_decision_wall > self.decision_timeout_s:
            return None
        return self.latest_decision

    def _compute_mode_diagnostics(self, now: float, snapshot) -> Dict[str, object]:
        if snapshot is not None:
            self.metric_history.append(
                {
                    "t": now,
                    "x": float(snapshot.pose_x),
                    "y": float(snapshot.pose_y),
                    "goal_dist": float(snapshot.goal_dist),
                    "heading_abs": abs(float(snapshot.goal_heading)),
                    "lin": float(snapshot.linear_speed),
                    "ang": float(snapshot.angular_speed),
                    "min_range": float(snapshot.min_range),
                    "severity": float(snapshot.world_event.get("severity", 0.0)) if snapshot.world_event else 0.0,
                }
            )

        no_progress = self._detect_no_progress(now)
        risk_level = self._risk_level(snapshot)
        oscillating = self._detect_oscillation(now)

        trend_fail = False
        recent: List[Dict[str, float]] = [p for p in self.metric_history if now - p["t"] <= self.trend_window_s]
        if len(recent) >= 2:
            oldest = recent[0]
            latest = recent[-1]
            goal_progress = oldest["goal_dist"] - latest["goal_dist"]
            heading_progress = oldest["heading_abs"] - latest["heading_abs"]
            trend_fail = bool(
                goal_progress < self.trend_goal_progress_min and
                heading_progress < self.trend_heading_progress_min
            )

        self._update_replan_failure(now, no_progress)
        return {
            "no_progress": no_progress,
            "trend_fail": trend_fail,
            "oscillating": oscillating,
            "risk_level": risk_level,
            "replan_failure_streak": self.replan_failure_streak,
        }

    def _transition_mode(
        self,
        now: float,
        snapshot,
        advisory_mode: str,
        diagnostics: Dict[str, object],
    ) -> Tuple[str, str]:
        if snapshot is None:
            if self.mode_state != ManagerMode.BLOCKED:
                self.mode_enter_wall = now
            return ManagerMode.BLOCKED, "sensor_not_ready"

        risk_level = int(diagnostics["risk_level"])
        no_progress = bool(diagnostics["no_progress"])
        trend_fail = bool(diagnostics["trend_fail"])
        oscillating = bool(diagnostics["oscillating"])
        replan_fail = int(diagnostics["replan_failure_streak"])

        if risk_level >= 3:
            if self.mode_state != ManagerMode.BLOCKED:
                self.mode_enter_wall = now
            return ManagerMode.BLOCKED, "critical_risk"

        if self.mode_state == ManagerMode.RECOVERY_REQUESTED:
            if no_progress or risk_level >= 2:
                return ManagerMode.RECOVERY_REQUESTED, "recovery_in_progress"
            self.mode_enter_wall = now
            return ManagerMode.CAUTIOUS, "recovery_completed"

        if self.mode_state == ManagerMode.BLOCKED:
            blocked_dur = now - self.mode_enter_wall
            if blocked_dur >= self.blocked_to_recovery_s:
                self.mode_enter_wall = now
                return ManagerMode.RECOVERY_REQUESTED, "blocked_timeout"
            return ManagerMode.BLOCKED, "blocked_waiting"

        if no_progress and (trend_fail or oscillating):
            self.mode_enter_wall = now
            return ManagerMode.BLOCKED, "no_progress_with_bad_trend"

        if replan_fail >= self.replan_fail_limit:
            self.mode_enter_wall = now
            return ManagerMode.BLOCKED, "replan_fail_streak"

        if advisory_mode in (ManagerMode.BLOCKED, ManagerMode.RECOVERY_REQUESTED):
            self.mode_enter_wall = now
            return advisory_mode, "advisory_blocked"

        if risk_level >= 2 or advisory_mode == ManagerMode.CAUTIOUS or trend_fail:
            if self.mode_state != ManagerMode.CAUTIOUS:
                self.mode_enter_wall = now
            return ManagerMode.CAUTIOUS, "elevated_risk_or_trend"

        if self.mode_state != ManagerMode.NOMINAL:
            self.mode_enter_wall = now
        return ManagerMode.NOMINAL, "nominal"

    def _apply_mode(
        self,
        *,
        now: float,
        mode: str,
        reason: str,
        decision: Optional[AgentDecision],
        nav_cmd: Twist,
        snapshot,
    ) -> Tuple[Twist, str]:
        if snapshot is None:
            self.state = AgentState.WAITING.value
            return Twist(), "sensor_not_ready"

        if mode == ManagerMode.NOMINAL:
            self.state = AgentState.EXECUTING.value
            return nav_cmd, ""

        if mode == ManagerMode.CAUTIOUS:
            self.state = AgentState.EXECUTING.value
            out = Twist()
            scale = self.cautious_speed_scale
            if decision is not None and decision.decision_type == DecisionType.CAUTIOUS_MODE:
                scale = float(decision.constraints.get("speed_scale", scale))
            out.linear.x = float(nav_cmd.linear.x) * scale
            out.angular.z = float(nav_cmd.angular.z) * scale
            return out, f"cautious:{reason}"

        if mode == ManagerMode.BLOCKED:
            self.state = AgentState.BLOCKED.value
            self._emit_replan(now, reason=reason)
            return Twist(), f"blocked:{reason}"

        self.state = AgentState.BLOCKED.value
        self._emit_recovery(now, reason=reason)
        return Twist(), f"recovery:{reason}"

    def _risk_level(self, snapshot) -> int:
        if snapshot is None:
            return 3
        sev = float(snapshot.world_event.get("severity", 0.0)) if snapshot.world_event else 0.0
        min_range = float(snapshot.min_range)
        if min_range <= self.hard_stop_range or sev >= self.risk_blocked_severity:
            return 3
        if min_range <= self.mode_cautious_range or sev >= self.risk_cautious_severity:
            return 2
        if min_range <= self.mode_medium_range or sev >= 0.35:
            return 1
        return 0

    def _update_replan_failure(self, now: float, no_progress: bool) -> None:
        if self.replan_count <= 0:
            return
        if self.replan_pending_since is None:
            self.replan_pending_since = now
            return
        if no_progress and now - self.replan_pending_since >= self.replan_grace_s:
            self.replan_failure_streak = min(self.replan_failure_streak + 1, 1000)
            self.replan_pending_since = now
        if not no_progress:
            self.replan_failure_streak = 0
            self.replan_pending_since = None

    def _detect_oscillation(self, now: float) -> bool:
        recent: List[Dict[str, float]] = [p for p in self.metric_history if now - p["t"] <= self.oscillation_window_s]
        if len(recent) < 4:
            return False

        signs: List[int] = []
        for p in recent:
            if abs(p["lin"]) <= self.oscillation_max_lin_speed and abs(p["ang"]) >= self.oscillation_min_ang_speed:
                signs.append(1 if p["ang"] > 0.0 else -1)
        if len(signs) < 4:
            return False

        flips = 0
        for i in range(1, len(signs)):
            if signs[i] != signs[i - 1]:
                flips += 1

        oldest = recent[0]
        latest = recent[-1]
        displacement = math.hypot(latest["x"] - oldest["x"], latest["y"] - oldest["y"])
        return bool(flips >= self.oscillation_min_sign_flips and displacement <= self.oscillation_max_disp)

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
        self.replan_pending_since = now
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
        self.replan_pending_since = None
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

    def _publish_supervisor_status(
        self,
        now: float,
        decision_type: str,
        manager_mode: str,
        override_reason: str,
        snapshot,
    ) -> None:
        block_reason_code = block_reason_from_text(override_reason) if override_reason else BlockReason.NONE.value
        recovery_reason_code = (
            recovery_reason_from_text(override_reason)
            if manager_mode == ManagerMode.RECOVERY_REQUESTED or "recovery" in override_reason
            else RecoveryReason.NONE.value
        )
        payload = {
            "wall_time": now,
            "sim_time": float(self.get_clock().now().nanoseconds / 1e9),
            "trace_id": self.trace_id,
            "mission_id": self.mission_id,
            "state": self.state,
            "decision_type": decision_type,
            "manager_mode": manager_mode,
            "mode_reason": self.last_mode_reason,
            "override_reason": override_reason,
            "block_reason_code": block_reason_code,
            "recovery_reason_code": recovery_reason_code,
            "override_count": self.override_count,
            "replan_count": self.replan_count,
            "recovery_count": self.recovery_count,
            "replan_failure_streak": self.replan_failure_streak,
            "blocked_duration_s": self.blocked_duration_s,
            "mission_status": self.mission_status,
            "goal_dist": float(snapshot.goal_dist) if snapshot is not None else -1.0,
            "min_range": float(snapshot.min_range) if snapshot is not None else -1.0,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True)
        self.supervisor_status_pub.publish(msg)

    def _publish_agent_status(
        self,
        now: float,
        decision_type: str,
        manager_mode: str,
        override_reason: str,
        snapshot,
    ) -> None:
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
        blocked = self.state == AgentState.BLOCKED.value
        block_reason_code = block_reason_from_text(override_reason) if blocked else BlockReason.NONE.value
        payload = AgentStatusPayload(
            agent_id="supervisor",
            role="safety_arbiter",
            state=self.state,
            current_goal="enforce_safe_execution",
            current_subtask=manager_mode,
            progress=float(min(self.step_count / 1200.0, 1.0)),
            health=1.0 if self.state != AgentState.BLOCKED.value else 0.7,
            last_heartbeat_ts=t["wall_time"],
            task_id=self.mission_id,
            parent_task_id="",
            owner_agent="supervisor",
            dependencies=["task_agent_decision", "nav_cmd"],
            queue_backlog=0,
            blocked_reason=override_reason if self.state == AgentState.BLOCKED.value else "",
            block_reason_code=block_reason_code,
            block_reason_detail=override_reason if blocked else "",
            trace_id=self.trace_id,
            correlation_id=f"{self.mission_id}-step-{self.step_count}",
            sim_time=t["sim_time"],
            wall_time=t["wall_time"],
            robot_pose=pose,
            world_state_summary=world_summary,
            robot_mode=manager_mode,
            risk_level=(
                "critical" if self._risk_level(snapshot) >= 3 else
                "high" if self._risk_level(snapshot) == 2 else
                "medium" if self._risk_level(snapshot) == 1 else
                "none"
            ),
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
                "mode_state": self.mode_state,
                "mode_reason": self.last_mode_reason,
                "mission_id": self.mission_id,
                "trace_id": self.trace_id,
                "override_count": self.override_count,
                "replan_count": self.replan_count,
                "recovery_count": self.recovery_count,
                "replan_failure_streak": self.replan_failure_streak,
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
