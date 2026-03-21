from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional

import rclpy

from .shared_types import SCHEMA_VERSION_V1


class AgentState(str, Enum):
    IDLE = "idle"
    PLANNING = "planning"
    WAITING = "waiting"
    EXECUTING = "executing"
    BLOCKED = "blocked"
    FAILED = "failed"
    DONE = "done"


def new_trace_id() -> str:
    return f"trace-{uuid.uuid4().hex[:16]}"


def new_event_id() -> str:
    return f"evt-{uuid.uuid4().hex[:12]}"


def timeline_fields(node: rclpy.node.Node) -> Dict[str, float]:
    now = node.get_clock().now().nanoseconds / 1e9
    return {
        "sim_time": float(now),
        "wall_time": float(time.time()),
    }


@dataclass
class AgentStatusPayload:
    agent_id: str
    role: str
    state: str
    current_goal: str
    current_subtask: str
    progress: float
    health: float
    last_heartbeat_ts: float
    task_id: str
    parent_task_id: str
    owner_agent: str
    dependencies: List[str]
    queue_backlog: int
    blocked_reason: str
    trace_id: str
    correlation_id: str
    sim_time: float
    wall_time: float
    robot_pose: Dict[str, float]
    world_state_summary: Dict[str, float]
    schema_version: str = SCHEMA_VERSION_V1
    robot_mode: str = ""
    risk_level: str = ""
    block_reason_code: str = ""
    block_reason_detail: str = ""

    def to_json(self) -> str:
        return json.dumps(self.__dict__, ensure_ascii=True)


@dataclass
class AgentEventPayload:
    event_id: str
    event_type: str
    sender: str
    receiver: str
    message_type: str
    phase: str
    latency_ms: float
    timeout_ms: float
    retry_count: int
    task_id: str
    trace_id: str
    correlation_id: str
    result: str
    failure_reason: str
    details: Dict[str, object]
    sim_time: float
    wall_time: float
    schema_version: str = SCHEMA_VERSION_V1
    supervisor_decision: str = ""
    risk_level: str = ""
    block_reason_code: str = ""
    recovery_reason_code: str = ""

    def to_json(self) -> str:
        return json.dumps(self.__dict__, ensure_ascii=True)
