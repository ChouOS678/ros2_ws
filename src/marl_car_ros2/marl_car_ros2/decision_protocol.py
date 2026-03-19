from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass, field
from typing import Dict


class DecisionType:
    NORMAL_NAVIGATION = "normal_navigation"
    TRIGGER_REPLAN = "trigger_replan"
    CAUTIOUS_MODE = "cautious_mode"
    PAUSE_AND_WAIT = "pause_and_wait"
    RECOVERY_REQUEST = "recovery_request"
    FALLBACK_TO_NAV2 = "fallback_to_nav2"


VALID_DECISIONS = {
    DecisionType.NORMAL_NAVIGATION,
    DecisionType.TRIGGER_REPLAN,
    DecisionType.CAUTIOUS_MODE,
    DecisionType.PAUSE_AND_WAIT,
    DecisionType.RECOVERY_REQUEST,
    DecisionType.FALLBACK_TO_NAV2,
}


def new_decision_id() -> str:
    return f"dec-{uuid.uuid4().hex[:12]}"


@dataclass
class AgentDecision:
    decision_type: str
    reason: str
    confidence: float = 0.8
    constraints: Dict[str, float] = field(default_factory=dict)
    mission_id: str = "mission-default"
    trace_id: str = ""
    decision_id: str = field(default_factory=new_decision_id)
    decision_wall_time: float = field(default_factory=time.time)
    decision_latency_ms: float = 0.0

    def to_json(self) -> str:
        return json.dumps(self.__dict__, ensure_ascii=True)

    @staticmethod
    def from_json(raw: str) -> "AgentDecision":
        payload = json.loads(raw)
        if not isinstance(payload, dict):
            raise ValueError("decision payload must be dict")
        typ = str(payload.get("decision_type", ""))
        if typ not in VALID_DECISIONS:
            raise ValueError(f"invalid decision type: {typ}")
        return AgentDecision(
            decision_type=typ,
            reason=str(payload.get("reason", "")),
            confidence=float(payload.get("confidence", 0.0)),
            constraints=dict(payload.get("constraints", {})) if isinstance(payload.get("constraints", {}), dict) else {},
            mission_id=str(payload.get("mission_id", "mission-default")),
            trace_id=str(payload.get("trace_id", "")),
            decision_id=str(payload.get("decision_id", new_decision_id())),
            decision_wall_time=float(payload.get("decision_wall_time", time.time())),
            decision_latency_ms=float(payload.get("decision_latency_ms", 0.0)),
        )
