from __future__ import annotations

import json
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .agent_trace import AgentEventPayload, AgentState, AgentStatusPayload, new_event_id, new_trace_id, timeline_fields
from .decision_protocol import AgentDecision, DecisionType, ManagerMode
from .observation_builder import ObservationBuilder
from .shared_types import BlockReason, block_reason_from_text


class TaskAgentNode(Node):
    """
    Mode/decision manager node.

    This node monitors runtime status and publishes high-level mode decisions.
    It does not publish low-level velocity control.
    """

    def __init__(self) -> None:
        super().__init__("task_agent")

        self.declare_parameter("step_hz", 5.0)
        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("scan_bins", 24)
        self.declare_parameter("max_scan_range", 8.0)
        self.declare_parameter("hard_stop_range", 0.28)
        self.declare_parameter("cautious_range", 0.75)
        self.declare_parameter("stuck_speed_eps", 0.03)
        self.declare_parameter("stuck_goal_dist_min", 0.5)
        self.declare_parameter("stuck_steps_limit", 30)
        self.declare_parameter("congestion_range", 1.0)
        self.declare_parameter("congestion_steps_limit", 18)
        self.declare_parameter("world_event_cautious_severity", 0.55)
        self.declare_parameter("goal_tolerance", 0.25)

        step_hz = float(self.get_parameter("step_hz").value)
        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        scan_bins = int(self.get_parameter("scan_bins").value)
        max_scan_range = float(self.get_parameter("max_scan_range").value)

        self.hard_stop_range = float(self.get_parameter("hard_stop_range").value)
        self.cautious_range = float(self.get_parameter("cautious_range").value)
        self.stuck_speed_eps = float(self.get_parameter("stuck_speed_eps").value)
        self.stuck_goal_dist_min = float(self.get_parameter("stuck_goal_dist_min").value)
        self.stuck_steps_limit = int(self.get_parameter("stuck_steps_limit").value)
        self.congestion_range = float(self.get_parameter("congestion_range").value)
        self.congestion_steps_limit = int(self.get_parameter("congestion_steps_limit").value)
        self.world_event_cautious_severity = float(self.get_parameter("world_event_cautious_severity").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)

        self.obs_builder = ObservationBuilder(
            self,
            goal_x=goal_x,
            goal_y=goal_y,
            scan_bins=scan_bins,
            max_scan_range=max_scan_range,
        )

        self.decision_pub = self.create_publisher(String, "/agent/decision", 20)
        self.agent_status_pub = self.create_publisher(String, "/agents/status", 20)
        self.agent_events_pub = self.create_publisher(String, "/agents/events", 40)
        self.query_srv = self.create_service(Trigger, "/task_agent/query_state", self._query_state_cb)

        self.trace_id = new_trace_id()
        self.mission_id = "mission-default"
        self.step_count = 0
        self.last_decision = AgentDecision(decision_type=DecisionType.PAUSE_AND_WAIT, reason="boot")
        self.last_decision_wall = 0.0
        self.manager_mode = ManagerMode.BLOCKED
        self.agent_state = AgentState.IDLE.value
        self.stuck_counter = 0
        self.congestion_counter = 0
        self.mission_status = "running"

        self.timer = self.create_timer(1.0 / max(step_hz, 1e-3), self._step)

    def _step(self) -> None:
        started = time.time()
        self.step_count += 1
        snapshot = self.obs_builder.build_snapshot()

        decision = self._decide(snapshot)
        decision.decision_latency_ms = max((time.time() - started) * 1000.0, 0.0)
        decision.trace_id = self.trace_id
        decision.mission_id = self.mission_id
        self.manager_mode = decision.manager_mode
        self.last_decision = decision
        self.last_decision_wall = time.time()

        msg = String()
        msg.data = decision.to_json()
        self.decision_pub.publish(msg)

        self._publish_status(snapshot)
        self._publish_event(
            event_type="decision_emitted",
            result=decision.decision_type,
            details={
                "manager_mode": decision.manager_mode,
                "reason": decision.reason,
                "decision_latency_ms": decision.decision_latency_ms,
                "confidence": decision.confidence,
            },
        )

    def _decide(self, snapshot) -> AgentDecision:
        if snapshot is None:
            decision = AgentDecision(
                decision_type=DecisionType.PAUSE_AND_WAIT,
                reason="sensor_not_ready",
                confidence=0.2,
                manager_mode=ManagerMode.BLOCKED,
            )
            self._set_agent_state(decision)
            return decision

        if abs(snapshot.linear_speed) < self.stuck_speed_eps and snapshot.goal_dist > self.stuck_goal_dist_min:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        if (
            abs(snapshot.linear_speed) < self.stuck_speed_eps
            and snapshot.min_range <= self.congestion_range
            and snapshot.goal_dist > self.stuck_goal_dist_min
        ):
            self.congestion_counter += 1
        else:
            self.congestion_counter = 0

        if snapshot.goal_dist <= self.goal_tolerance and self.mission_status != "success":
            self.mission_status = "success"
            self._publish_event(
                event_type="mission_completed",
                result="goal_reached",
                details={"goal_dist": float(snapshot.goal_dist)},
            )

        if snapshot.min_range <= self.hard_stop_range:
            decision = AgentDecision(
                decision_type=DecisionType.RECOVERY_REQUEST,
                reason=f"obstacle_too_close:{snapshot.min_range:.2f}",
                confidence=0.95,
                manager_mode=ManagerMode.RECOVERY_REQUESTED,
                recovery_reason_code="obstacle_too_close",
            )
            self._set_agent_state(decision)
            return decision

        if self.stuck_counter >= self.stuck_steps_limit * 2:
            decision = AgentDecision(
                decision_type=DecisionType.RECOVERY_REQUEST,
                reason=f"persistent_stuck:{self.stuck_counter}",
                confidence=0.9,
                manager_mode=ManagerMode.RECOVERY_REQUESTED,
                recovery_reason_code="persistent_stuck",
            )
            self._set_agent_state(decision)
            return decision

        if self.stuck_counter >= self.stuck_steps_limit:
            decision = AgentDecision(
                decision_type=DecisionType.TRIGGER_REPLAN,
                reason=f"no_progress:{self.stuck_counter}",
                confidence=0.85,
                manager_mode=ManagerMode.BLOCKED,
                recovery_reason_code="no_progress_timeout",
            )
            self._set_agent_state(decision)
            return decision

        sev = float(snapshot.world_event.get("severity", 0.0)) if snapshot.world_event else 0.0
        status = str(snapshot.world_event.get("status", "")) if snapshot.world_event else ""
        event_type = str(snapshot.world_event.get("event_type", snapshot.world_event.get("type", ""))) if snapshot.world_event else ""

        if self.congestion_counter >= self.congestion_steps_limit:
            decision = AgentDecision(
                decision_type=DecisionType.CAUTIOUS_MODE,
                reason=f"congestion:{self.congestion_counter}",
                confidence=0.78,
                manager_mode=ManagerMode.CAUTIOUS,
            )
            self._set_agent_state(decision)
            return decision

        if sev >= self.world_event_cautious_severity and status in ("start", "active"):
            decision = AgentDecision(
                decision_type=DecisionType.CAUTIOUS_MODE,
                reason=f"world_event:{event_type or 'unknown'}:{sev:.2f}",
                confidence=0.8,
                manager_mode=ManagerMode.CAUTIOUS,
            )
            self._set_agent_state(decision)
            return decision

        if snapshot.min_range <= self.cautious_range:
            decision = AgentDecision(
                decision_type=DecisionType.CAUTIOUS_MODE,
                reason=f"near_obstacle:{snapshot.min_range:.2f}",
                confidence=0.75,
                manager_mode=ManagerMode.CAUTIOUS,
            )
            self._set_agent_state(decision)
            return decision

        decision = AgentDecision(
            decision_type=DecisionType.NORMAL_NAVIGATION,
            reason="nominal",
            confidence=0.85,
            manager_mode=ManagerMode.NOMINAL,
        )
        self._set_agent_state(decision)
        return decision

    def _set_agent_state(self, decision: AgentDecision) -> None:
        if decision.manager_mode == ManagerMode.RECOVERY_REQUESTED:
            self.agent_state = AgentState.BLOCKED.value
            return
        if decision.manager_mode == ManagerMode.BLOCKED:
            if decision.decision_type == DecisionType.TRIGGER_REPLAN:
                self.agent_state = AgentState.PLANNING.value
            else:
                self.agent_state = AgentState.WAITING.value
            return
        self.agent_state = AgentState.EXECUTING.value

    def _publish_status(self, snapshot) -> None:
        t = timeline_fields(self)
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        world_summary: Dict[str, float] = {"goal_dist": -1.0, "min_range": -1.0, "stuck": 0.0}
        if snapshot is not None:
            pose = {"x": float(snapshot.pose_x), "y": float(snapshot.pose_y), "yaw": float(snapshot.yaw)}
            world_summary = {
                "goal_dist": float(snapshot.goal_dist),
                "min_range": float(snapshot.min_range),
                "stuck": 1.0 if self.stuck_counter >= self.stuck_steps_limit else 0.0,
            }

        payload = AgentStatusPayload(
            agent_id="task_agent",
            role="mode_manager",
            state=self.agent_state,
            current_goal="reach_goal",
            current_subtask=self.last_decision.manager_mode,
            progress=float(min(self.step_count / 1200.0, 1.0)),
            health=1.0 if self.agent_state != AgentState.BLOCKED.value else 0.6,
            last_heartbeat_ts=t["wall_time"],
            task_id=self.mission_id,
            parent_task_id="",
            owner_agent="task_agent",
            dependencies=["odom_ready", "scan_ready"],
            queue_backlog=0,
            blocked_reason=self.last_decision.reason if self.agent_state == AgentState.BLOCKED.value else "",
            trace_id=self.trace_id,
            correlation_id=f"{self.mission_id}-step-{self.step_count}",
            sim_time=t["sim_time"],
            wall_time=t["wall_time"],
            robot_pose=pose,
            world_state_summary=world_summary,
            robot_mode=self.last_decision.manager_mode,
            block_reason_code=(
                block_reason_from_text(self.last_decision.reason)
                if self.agent_state == AgentState.BLOCKED.value
                else BlockReason.NONE.value
            ),
            block_reason_detail=self.last_decision.reason if self.agent_state == AgentState.BLOCKED.value else "",
        )
        msg = String()
        msg.data = payload.to_json()
        self.agent_status_pub.publish(msg)

    def _publish_event(self, *, event_type: str, result: str, details: Dict[str, object]) -> None:
        t = timeline_fields(self)
        payload = AgentEventPayload(
            event_id=new_event_id(),
            event_type=event_type,
            sender="task_agent",
            receiver="supervisor",
            message_type="decision",
            phase="result",
            latency_ms=float(self.last_decision.decision_latency_ms),
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
                "agent_state": self.agent_state,
                "manager_mode": self.last_decision.manager_mode,
                "mission_id": self.mission_id,
                "trace_id": self.trace_id,
                "step_count": self.step_count,
                "stuck_counter": self.stuck_counter,
                "congestion_counter": self.congestion_counter,
                "last_decision": self.last_decision.__dict__,
                "mission_status": self.mission_status,
                "query_wall_time": time.time(),
            },
            ensure_ascii=True,
        )
        return response


def main() -> None:
    rclpy.init()
    node = TaskAgentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
