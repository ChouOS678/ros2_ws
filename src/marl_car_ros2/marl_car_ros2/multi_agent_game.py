from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Dict

import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, Float32MultiArray, String

from .agent_trace import AgentEventPayload, AgentState, AgentStatusPayload, new_event_id, new_trace_id, timeline_fields
from .marl_env_wrapper import JointAction, Ros2MarlEnvWrapper


@dataclass
class PlannerActionSpace:
    min_speed: float
    max_speed: float


@dataclass
class ChassisActionSpace:
    min_omega: float
    max_omega: float


class AggressivePlannerAgent:
    """
    Observation input:
      [vx, wz, sin(yaw), cos(yaw), goal_dist_norm, goal_heading_norm, scan_bins...]
    Action output:
      planner_target_speed in [0, max_speed]
    """

    def __init__(self, max_speed: float) -> None:
        self.action_space = PlannerActionSpace(0.0, max_speed)

    def act(self, obs: np.ndarray) -> float:
        goal_dist_norm = float(np.clip(obs[4], 0.0, 1.0))
        goal_heading_norm = float(obs[5])
        heading_cost = min(abs(goal_heading_norm), 1.0)
        min_scan = float(np.min(obs[6:])) if obs.shape[0] > 6 else 1.0
        risk_scale = 0.25 + 0.75 * max(min_scan, 0.0)

        target = self.action_space.max_speed * (0.35 + 0.75 * goal_dist_norm) * (1.0 - 0.35 * heading_cost) * risk_scale
        return float(np.clip(target, self.action_space.min_speed, self.action_space.max_speed))


class ConservativeChassisAgent:
    """
    Observation input:
      same as planner.
    Action output:
      chassis_target_omega in [-max_omega, max_omega]
    """

    def __init__(self, max_omega: float) -> None:
        self.action_space = ChassisActionSpace(-max_omega, max_omega)

    def act(self, obs: np.ndarray) -> float:
        goal_heading = float(np.clip(obs[5], -1.0, 1.0)) * math.pi
        left = float(np.mean(obs[6:14])) if obs.shape[0] >= 14 else 1.0
        right = float(np.mean(obs[-8:])) if obs.shape[0] >= 14 else 1.0
        avoid_bias = (right - left) * 1.2
        omega = 0.9 * goal_heading + avoid_bias
        return float(np.clip(omega, self.action_space.min_omega, self.action_space.max_omega))


class MultiAgentGameNode(Node):
    """
    ROS2 Node-as-Agent orchestration layer.
    - Coordinates planner/chassis actions
    - Applies optional LLM reward deltas
    - Publishes structured episode events
    """

    def __init__(self) -> None:
        super().__init__("multi_agent_game")
        self.env = Ros2MarlEnvWrapper()

        self.declare_parameter("step_hz", 10.0)
        step_hz = float(self.get_parameter("step_hz").value)

        self.planner = AggressivePlannerAgent(max_speed=self.env.max_speed)
        self.chassis = ConservativeChassisAgent(max_omega=self.env.max_omega)
        self.current_obs = self.env.reset()
        self.episode_step = 0
        self.episode_id = 0
        self.trace_id = new_trace_id()
        self.last_state_enter_wall = time.time()
        self.planner_state = AgentState.IDLE.value
        self.chassis_state = AgentState.IDLE.value

        self.llm_reward_delta = np.zeros(2, dtype=np.float32)
        self.create_subscription(Float32MultiArray, "/llm_reasoner/reward_delta", self._llm_reward_cb, 10)

        self.joint_action_pub = self.create_publisher(Float32MultiArray, "/marl/joint_action", 10)
        self.reward_breakdown_pub = self.create_publisher(String, "/marl/reward_breakdown", 10)
        self.planner_action_pub = self.create_publisher(Float32, "/marl/planner_action", 10)
        self.chassis_action_pub = self.create_publisher(Float32, "/marl/chassis_action", 10)
        self.agent_status_pub = self.create_publisher(String, "/agents/status", 20)
        self.agent_events_pub = self.create_publisher(String, "/agents/events", 40)
        self.query_srv = self.create_service(Trigger, "/agents/query_state", self._query_state_cb)

        self.timer = self.create_timer(1.0 / step_hz, self._loop_step)

    def _llm_reward_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 2:
            self.llm_reward_delta[0] = float(msg.data[0])
            self.llm_reward_delta[1] = float(msg.data[1])

    def _loop_step(self) -> None:
        self._transition_states(AgentState.PLANNING.value, AgentState.WAITING.value)
        planner_v = self.planner.act(self.current_obs)
        chassis_w = self.chassis.act(self.current_obs)
        self._transition_states(AgentState.EXECUTING.value, AgentState.EXECUTING.value)
        planner_msg = Float32()
        planner_msg.data = planner_v
        self.planner_action_pub.publish(planner_msg)
        chassis_msg = Float32()
        chassis_msg.data = chassis_w
        self.chassis_action_pub.publish(chassis_msg)

        joint_msg = Float32MultiArray()
        joint_msg.data = [planner_v, chassis_w]
        self.joint_action_pub.publish(joint_msg)

        obs, rewards, done, info = self.env.step(
            JointAction(planner_target_speed=planner_v, chassis_target_omega=chassis_w)
        )
        self.current_obs = obs
        self.episode_step += 1

        intrinsic = np.array([rewards["planner"], rewards["chassis"]], dtype=np.float32)
        adjusted = intrinsic + self.llm_reward_delta
        done_state = AgentState.DONE.value if done else AgentState.EXECUTING.value
        blocked_state = AgentState.BLOCKED.value if float(info.get("stuck", 0.0)) > 0.0 else done_state
        self._transition_states(done_state, blocked_state)
        self._publish_statuses(info)

        payload: Dict[str, object] = {
            "episode_id": self.episode_id,
            "step": self.episode_step,
            "planner_reward_intrinsic": float(intrinsic[0]),
            "chassis_reward_intrinsic": float(intrinsic[1]),
            "planner_reward_adjusted": float(adjusted[0]),
            "chassis_reward_adjusted": float(adjusted[1]),
            "llm_reward_delta": [float(self.llm_reward_delta[0]), float(self.llm_reward_delta[1])],
            "goal_dist": float(info.get("goal_dist", -1.0)),
            "min_range": float(info.get("min_range", -1.0)),
            "stuck": float(info.get("stuck", 0.0)),
            "episode_done": bool(done),
            "termination": str(info.get("termination", "")),
        }
        event = String()
        event.data = json.dumps(payload, ensure_ascii=True)
        self.reward_breakdown_pub.publish(event)
        self._publish_comm_event(
            event_type="task_feedback",
            sender="env",
            receiver="planner",
            result="ok" if not done else str(info.get("termination", "done")),
            details={
                "planner_reward": float(adjusted[0]),
                "chassis_reward": float(adjusted[1]),
                "stuck": float(info.get("stuck", 0.0)),
            },
        )

        if done:
            self._publish_comm_event(
                event_type="task_finished" if str(info.get("termination", "")) == "goal_reached" else "task_failed",
                sender="env",
                receiver="planner",
                result=str(info.get("termination", "done")),
                details={"episode_id": self.episode_id, "step": self.episode_step},
            )
            self.episode_id += 1
            self.episode_step = 0
            self.trace_id = new_trace_id()
            self._transition_states(AgentState.IDLE.value, AgentState.IDLE.value)
            self.current_obs = self.env.reset()

    def _transition_states(self, planner_state: str, chassis_state: str) -> None:
        now_wall = time.time()
        if planner_state != self.planner_state:
            self._publish_comm_event(
                event_type="state_transition",
                sender="planner",
                receiver="monitor",
                result=f"{self.planner_state}->{planner_state}",
                details={"agent_id": "planner", "from": self.planner_state, "to": planner_state},
            )
            self.last_state_enter_wall = now_wall
        if chassis_state != self.chassis_state:
            self._publish_comm_event(
                event_type="state_transition",
                sender="chassis",
                receiver="monitor",
                result=f"{self.chassis_state}->{chassis_state}",
                details={"agent_id": "chassis", "from": self.chassis_state, "to": chassis_state},
            )
            self.last_state_enter_wall = now_wall
        self.planner_state = planner_state
        self.chassis_state = chassis_state

    def _publish_statuses(self, info: Dict[str, float]) -> None:
        t = timeline_fields(self.env)
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        if self.env.latest_odom is not None:
            pose["x"] = float(self.env.latest_odom.pose.pose.position.x)
            pose["y"] = float(self.env.latest_odom.pose.pose.position.y)
        world_summary = {
            "goal_dist": float(info.get("goal_dist", -1.0)),
            "min_range": float(info.get("min_range", -1.0)),
            "stuck": float(info.get("stuck", 0.0)),
        }
        task_id = f"ep-{self.episode_id}"
        common = {
            "current_goal": "reach_goal",
            "current_subtask": "drive_step",
            "progress": float(min(self.episode_step / 1000.0, 1.0)),
            "health": 1.0 if float(info.get("stuck", 0.0)) <= 0.0 else 0.6,
            "last_heartbeat_ts": t["wall_time"],
            "task_id": task_id,
            "parent_task_id": "",
            "owner_agent": "planner",
            "dependencies": ["scan_ready", "odom_ready"],
            "queue_backlog": 0,
            "blocked_reason": "stuck" if float(info.get("stuck", 0.0)) > 0.0 else "",
            "trace_id": self.trace_id,
            "correlation_id": f"{task_id}-step-{self.episode_step}",
            "sim_time": t["sim_time"],
            "wall_time": t["wall_time"],
            "robot_pose": pose,
            "world_state_summary": world_summary,
        }
        planner = AgentStatusPayload(
            agent_id="planner",
            role="aggressive_planner",
            state=self.planner_state,
            **common,
        )
        chassis = AgentStatusPayload(
            agent_id="chassis",
            role="conservative_chassis",
            state=self.chassis_state,
            **common,
        )
        planner.owner_agent = "planner"
        chassis.owner_agent = "chassis"
        for payload in (planner, chassis):
            msg = String()
            msg.data = payload.to_json()
            self.agent_status_pub.publish(msg)

    def _publish_comm_event(
        self,
        event_type: str,
        sender: str,
        receiver: str,
        result: str,
        details: Dict[str, object],
        *,
        message_type: str = "control",
        phase: str = "feedback",
        latency_ms: float = 0.0,
        timeout_ms: float = 200.0,
        retry_count: int = 0,
        failure_reason: str = "",
    ) -> None:
        t = timeline_fields(self.env)
        payload = AgentEventPayload(
            event_id=new_event_id(),
            event_type=event_type,
            sender=sender,
            receiver=receiver,
            message_type=message_type,
            phase=phase,
            latency_ms=latency_ms,
            timeout_ms=timeout_ms,
            retry_count=retry_count,
            task_id=f"ep-{self.episode_id}",
            trace_id=self.trace_id,
            correlation_id=f"ep-{self.episode_id}-step-{self.episode_step}",
            result=result,
            failure_reason=failure_reason,
            details=details,
            sim_time=t["sim_time"],
            wall_time=t["wall_time"],
        )
        msg = String()
        msg.data = payload.to_json()
        self.agent_events_pub.publish(msg)

    def _query_state_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        snapshot = {
            "episode_id": self.episode_id,
            "episode_step": self.episode_step,
            "planner_state": self.planner_state,
            "chassis_state": self.chassis_state,
            "trace_id": self.trace_id,
            "llm_reward_delta": [float(self.llm_reward_delta[0]), float(self.llm_reward_delta[1])],
            "query_wall_time": time.time(),
        }
        response.success = True
        response.message = json.dumps(snapshot, ensure_ascii=True)
        return response

    def destroy_node(self) -> bool:
        if hasattr(self, "timer") and self.timer is not None:
            self.timer.cancel()
        if self.env is not None:
            self.env.destroy_node()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MultiAgentGameNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
