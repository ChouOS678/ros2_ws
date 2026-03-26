from __future__ import annotations

from enum import Enum
from typing import Dict


class RobotMode(str, Enum):
    IDLE = "idle"
    PLANNING = "planning"
    WAITING = "waiting"
    EXECUTING = "executing"
    BLOCKED = "blocked"
    FAILED = "failed"
    DONE = "done"


class RiskLevel(str, Enum):
    NONE = "none"
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class BlockReason(str, Enum):
    NONE = "none"
    SENSOR_NOT_READY = "sensor_not_ready"
    HARD_STOP = "hard_stop"
    NO_PROGRESS = "no_progress"
    EXTERNAL_PAUSE = "external_pause"
    UNKNOWN = "unknown"


class RecoveryReason(str, Enum):
    NONE = "none"
    OBSTACLE_TOO_CLOSE = "obstacle_too_close"
    PERSISTENT_STUCK = "persistent_stuck"
    NO_PROGRESS_TIMEOUT = "no_progress_timeout"
    SUPERVISOR_OVERRIDE = "supervisor_override"
    UNKNOWN = "unknown"


class SupervisorDecision(str, Enum):
    PASS_THROUGH = "pass_through"
    CAUTIOUS_MODE = "cautious_mode"
    PAUSE_AND_WAIT = "pause_and_wait"
    TRIGGER_REPLAN = "trigger_replan"
    REQUEST_RECOVERY = "request_recovery"
    HARD_STOP = "hard_stop"


class ExperimentEventType(str, Enum):
    GHOST_PROBE = "ghost_probe"
    FRICTION_DROP = "friction_drop"
    OCCLUSION = "occlusion"
    COLLISION_RISK = "collision_risk"
    CUSTOM = "custom"


class ExperimentEventStatus(str, Enum):
    START = "start"
    ACTIVE = "active"
    END = "end"


SCHEMA_VERSION_V1 = "contract.v1"


def risk_level_from_severity(severity: float) -> str:
    sev = float(max(0.0, min(1.0, severity)))
    if sev >= 0.85:
        return RiskLevel.CRITICAL.value
    if sev >= 0.65:
        return RiskLevel.HIGH.value
    if sev >= 0.40:
        return RiskLevel.MEDIUM.value
    if sev > 0.0:
        return RiskLevel.LOW.value
    return RiskLevel.NONE.value


def block_reason_from_text(text: str) -> str:
    raw = str(text or "").lower()
    if not raw:
        return BlockReason.NONE.value
    if raw.startswith("blocked:"):
        raw = raw.split(":", 1)[1]
    if raw.startswith("sensor_not_ready"):
        return BlockReason.SENSOR_NOT_READY.value
    if raw.startswith("hard_stop"):
        return BlockReason.HARD_STOP.value
    if raw.startswith("no_progress") or "no_progress" in raw:
        return BlockReason.NO_PROGRESS.value
    if raw.startswith("blocked_timeout"):
        return BlockReason.NO_PROGRESS.value
    if raw.startswith("pause_and_wait"):
        return BlockReason.EXTERNAL_PAUSE.value
    return BlockReason.UNKNOWN.value


def recovery_reason_from_text(text: str) -> str:
    raw = str(text or "").lower()
    if not raw:
        return RecoveryReason.NONE.value
    if raw.startswith("recovery:"):
        raw = raw.split(":", 1)[1]
    if raw.startswith("obstacle_too_close"):
        return RecoveryReason.OBSTACLE_TOO_CLOSE.value
    if raw.startswith("persistent_stuck"):
        return RecoveryReason.PERSISTENT_STUCK.value
    if raw.startswith("no_progress_timeout") or "no_progress" in raw:
        return RecoveryReason.NO_PROGRESS_TIMEOUT.value
    if (
        raw.startswith("recovery_request")
        or raw.startswith("hard_stop")
        or raw.startswith("blocked_timeout")
        or raw.startswith("recovery_in_progress")
    ):
        return RecoveryReason.SUPERVISOR_OVERRIDE.value
    return RecoveryReason.UNKNOWN.value


def supervisor_decision_from_legacy(decision_type: str, override_reason: str = "") -> str:
    if str(override_reason).startswith("hard_stop"):
        return SupervisorDecision.HARD_STOP.value
    d = str(decision_type or "")
    if d == "cautious_mode":
        return SupervisorDecision.CAUTIOUS_MODE.value
    if d == "pause_and_wait":
        return SupervisorDecision.PAUSE_AND_WAIT.value
    if d == "trigger_replan":
        return SupervisorDecision.TRIGGER_REPLAN.value
    if d == "recovery_request":
        return SupervisorDecision.REQUEST_RECOVERY.value
    return SupervisorDecision.PASS_THROUGH.value


def normalize_experiment_event(payload: Dict[str, object]) -> Dict[str, object]:
    if not isinstance(payload, dict):
        return {}
    out = dict(payload)
    event_type = str(out.get("event_type", out.get("type", "")) or "")
    event_status = str(out.get("event_status", out.get("status", "")) or "")
    severity = float(out.get("severity", 0.0) or 0.0)
    out.setdefault("schema_version", SCHEMA_VERSION_V1)
    out["event_type"] = event_type
    out["event_status"] = event_status
    out.setdefault("type", event_type)
    out.setdefault("status", event_status)
    out.setdefault("risk_level", risk_level_from_severity(severity))
    out.setdefault("source", "scenario_mutator")
    out.setdefault("details", {})
    return out
