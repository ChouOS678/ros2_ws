from __future__ import annotations

import json
import sqlite3
import statistics
import math
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Dict, List, Tuple, Any

try:
    import yaml  # type: ignore
except ImportError:  # pragma: no cover - fallback path for lightweight runtime envs
    yaml = None


DEFAULT_BENCHMARK_META: Dict[str, Dict[str, object]] = {
    "narrow_corridor": {
        "suite": "narrow_corridor",
        "result_group": "primary",
        "objective": "tracking_safety",
        "centerline": {"axis": "y", "value": 0.0},
        "scrape_min_clearance_m": 0.08,
    },
    "sharp_turns": {
        "suite": "sharp_turns",
        "result_group": "primary",
        "objective": "high_curvature_tracking",
        "reference_path": [
            {"x": 0.0, "y": 0.0},
            {"x": 2.8, "y": 0.0},
            {"x": 2.8, "y": 2.2},
            {"x": 0.8, "y": 2.2},
            {"x": 0.8, "y": 0.2},
            {"x": 3.8, "y": 0.2},
        ],
        "turn_windows": [
            {"x_min": 2.1, "x_max": 3.5, "y_min": -0.2, "y_max": 1.8, "name": "l_turn"},
            {"x_min": 0.0, "x_max": 1.7, "y_min": 0.3, "y_max": 2.5, "name": "hairpin_return"},
        ],
        "cmd_constraints": {"max_linear_x": 0.8, "max_angular_z": 1.2},
    },
    "dynamic_crossing": {
        "suite": "dynamic_obstacle",
        "result_group": "extension",
        "objective": "dynamic_disturbance_robustness",
        "stop_speed_threshold_mps": 0.05,
        "event_type": "dynamic_obstacle_triggered",
    },
}


@dataclass
class MetricsReport:
    scenario_name: str
    suite: str
    result_group: str
    objective: str
    mission_completion_status: str
    mission_duration_s: float
    primary_metrics: Dict[str, Any]
    supporting_metrics: Dict[str, Any]
    timeline_metrics: Dict[str, Any]


def _mean(values: List[float]) -> float:
    return float(sum(values) / len(values)) if values else 0.0


def _bool_any(values: List[bool]) -> bool:
    return any(bool(v) for v in values)


def _distance_point_to_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    denom = abx * abx + aby * aby
    if denom <= 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / denom))
    qx = ax + t * abx
    qy = ay + t * aby
    return math.hypot(px - qx, py - qy)


def _distance_to_polyline(px: float, py: float, pts: List[Tuple[float, float]]) -> float:
    if len(pts) < 2:
        return 0.0
    return min(
        _distance_point_to_segment(px, py, pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
        for i in range(len(pts) - 1)
    )


def _inside_window(x: float, y: float, w: Dict[str, object]) -> bool:
    return (
        float(w.get("x_min", 0.0)) <= x <= float(w.get("x_max", 0.0))
        and float(w.get("y_min", 0.0)) <= y <= float(w.get("y_max", 0.0))
    )


def _load_scenario_meta(db_path: str, scenario_name: str) -> Dict[str, object]:
    del db_path
    bench: Dict[str, object] = dict(DEFAULT_BENCHMARK_META.get(scenario_name, {}))
    if yaml is None:
        return bench
    cfg_path = Path(__file__).resolve().parents[1] / "config" / "baseline_world_scenarios.yaml"
    if not cfg_path.exists():
        return bench
    with cfg_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    scenarios = data.get("scenarios", {})
    if not isinstance(scenarios, dict):
        return bench
    scenario = scenarios.get(scenario_name, {})
    if not isinstance(scenario, dict):
        return bench
    file_bench = scenario.get("benchmark", {})
    if isinstance(file_bench, dict):
        bench.update(file_bench)
    return bench


def _read_latest_summary(db: sqlite3.Connection) -> Dict[str, object]:
    cur = db.cursor()
    row = cur.execute("SELECT payload_json FROM arch_metrics ORDER BY wall_time DESC LIMIT 1").fetchone()
    if not row:
        return {}
    try:
        payload = json.loads(row[0])
        return payload if isinstance(payload, dict) else {}
    except json.JSONDecodeError:
        return {}


def _read_telemetry(db: sqlite3.Connection, scenario_name: str) -> List[Dict[str, object]]:
    cur = db.cursor()
    try:
        rows = cur.execute(
            """
            SELECT payload_json FROM benchmark_telemetry
            WHERE scenario_name = ?
            ORDER BY wall_time ASC
            """,
            (scenario_name,),
        ).fetchall()
    except sqlite3.OperationalError:
        return []
    out: List[Dict[str, object]] = []
    for (raw,) in rows:
        try:
            payload = json.loads(raw)
            if isinstance(payload, dict):
                out.append(payload)
        except json.JSONDecodeError:
            continue
    return out


def _read_timeline_counts(timeline_jsonl: str) -> Dict[str, int]:
    p = Path(timeline_jsonl)
    out = {
        "mode_switch_count": 0,
        "no_progress_count": 0,
        "recovery_start_count": 0,
        "replan_count_timeline": 0,
    }
    if not p.exists():
        return out
    with p.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                evt = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(evt, dict):
                continue
            et = str(evt.get("event_type", ""))
            ph = str(evt.get("event_phase", ""))
            if et == "mode_switch":
                out["mode_switch_count"] += 1
            elif et == "no_progress" and ph == "start":
                out["no_progress_count"] += 1
            elif et == "recovery" and ph == "start":
                out["recovery_start_count"] += 1
            elif et == "replan" and ph == "start":
                out["replan_count_timeline"] += 1
    return out


def _mission_duration(telemetry: List[Dict[str, object]]) -> float:
    if len(telemetry) < 2:
        return 0.0
    return max(0.0, float(telemetry[-1]["wall_time"]) - float(telemetry[0]["wall_time"]))


def _compute_narrow_corridor_metrics(telemetry: List[Dict[str, object]], bench: Dict[str, object]) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    centerline = bench.get("centerline", {}) if isinstance(bench.get("centerline", {}), dict) else {}
    center_value = float(centerline.get("value", 0.0))
    scrape_threshold = float(bench.get("scrape_min_clearance_m", 0.08))
    min_ranges = [float(t.get("min_range", -1.0)) for t in telemetry if float(t.get("min_range", -1.0)) >= 0.0]
    lateral_offsets = [abs(float(t.get("y", 0.0)) - center_value) for t in telemetry]
    linear_speeds = [abs(float(t.get("linear_speed", 0.0))) for t in telemetry]
    primary = {
        "min_obstacle_clearance_m": min(min_ranges) if min_ranges else 0.0,
        "mean_lateral_offset_m": _mean(lateral_offsets),
        "scrape_or_contact_risk": bool(min(min_ranges) <= scrape_threshold) if min_ranges else False,
        "average_speed_mps": _mean(linear_speeds),
    }
    supporting = {
        "max_lateral_offset_m": max(lateral_offsets) if lateral_offsets else 0.0,
        "speed_p95_mps": float(statistics.quantiles(linear_speeds, n=20)[-1]) if len(linear_speeds) >= 20 else (max(linear_speeds) if linear_speeds else 0.0),
    }
    return primary, supporting


def _compute_sharp_turn_metrics(telemetry: List[Dict[str, object]], bench: Dict[str, object]) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    ref_pts_cfg = bench.get("reference_path", [])
    ref_pts = [(float(p["x"]), float(p["y"])) for p in ref_pts_cfg if isinstance(p, dict) and "x" in p and "y" in p]
    tracking_errors = [_distance_to_polyline(float(t.get("x", 0.0)), float(t.get("y", 0.0)), ref_pts) for t in telemetry]
    windows = [w for w in bench.get("turn_windows", []) if isinstance(w, dict)]
    turn_samples = [t for t in telemetry if any(_inside_window(float(t.get("x", 0.0)), float(t.get("y", 0.0)), w) for w in windows)]
    cmd_limits = bench.get("cmd_constraints", {}) if isinstance(bench.get("cmd_constraints", {}), dict) else {}
    max_lin = float(cmd_limits.get("max_linear_x", 0.8))
    max_ang = float(cmd_limits.get("max_angular_z", 1.2))
    violations = []
    for t in telemetry:
        lin = abs(float(t.get("cmd_linear_x", 0.0)))
        ang = abs(float(t.get("cmd_angular_z", 0.0)))
        violations.append(lin > max_lin + 1e-6 or ang > max_ang + 1e-6)
    turn_time = 0.0
    if len(turn_samples) >= 2:
        turn_time = max(0.0, float(turn_samples[-1]["wall_time"]) - float(turn_samples[0]["wall_time"]))
    primary = {
        "average_tracking_error_m": _mean(tracking_errors),
        "max_overshoot_m": max(tracking_errors) if tracking_errors else 0.0,
        "turn_completion_time_s": turn_time,
        "command_constraint_compliance_rate": 1.0 - (_mean([1.0 if v else 0.0 for v in violations]) if violations else 0.0),
    }
    supporting = {
        "max_cmd_linear_x_mps": max(abs(float(t.get("cmd_linear_x", 0.0))) for t in telemetry) if telemetry else 0.0,
        "max_cmd_angular_z_radps": max(abs(float(t.get("cmd_angular_z", 0.0))) for t in telemetry) if telemetry else 0.0,
        "turn_sample_count": len(turn_samples),
    }
    return primary, supporting


def _compute_dynamic_obstacle_metrics(
    telemetry: List[Dict[str, object]],
    bench: Dict[str, object],
    summary: Dict[str, object],
    timeline_counts: Dict[str, int],
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    stop_speed_threshold = float(bench.get("stop_speed_threshold_mps", 0.05))
    event_type = str(bench.get("event_type", "dynamic_obstacle_triggered"))
    active = [
        t for t in telemetry
        if bool(t.get("dynamic_event_active", False)) or str(t.get("event_type", "")) == event_type
    ]
    min_ranges = [float(t.get("min_range", -1.0)) for t in active if float(t.get("min_range", -1.0)) >= 0.0]
    stop_flags = [abs(float(t.get("linear_speed", 0.0))) <= stop_speed_threshold for t in active]
    arch = summary.get("arch_metrics", {}) if isinstance(summary.get("arch_metrics", {}), dict) else {}
    primary = {
        "stopped_for_dynamic_obstacle": _bool_any(stop_flags),
        "replanning_count": int(arch.get("replan_count", timeline_counts.get("replan_count_timeline", 0))),
        "passage_time_s": _mission_duration(telemetry),
        "min_safety_gap_m": min(min_ranges) if min_ranges else 0.0,
    }
    supporting = {
        "dynamic_window_sample_count": len(active),
        "recovery_trigger_count": int(arch.get("recovery_trigger_count", 0)),
    }
    return primary, supporting


def compute_metrics(db_path: str, timeline_jsonl: str = "") -> MetricsReport:
    db = sqlite3.connect(db_path)
    summary = _read_latest_summary(db)
    experiment_tags = summary.get("experiment_tags", {}) if isinstance(summary.get("experiment_tags", {}), dict) else {}
    scenario_name = str(experiment_tags.get("scenario_name", "unspecified"))
    telemetry = _read_telemetry(db, scenario_name)
    bench = _load_scenario_meta(db_path, scenario_name)
    timeline_counts = _read_timeline_counts(timeline_jsonl) if timeline_jsonl else {
        "mode_switch_count": 0,
        "no_progress_count": 0,
        "recovery_start_count": 0,
        "replan_count_timeline": 0,
    }

    suite = str(bench.get("suite", scenario_name))
    result_group = str(bench.get("result_group", "primary"))
    objective = str(bench.get("objective", "benchmark_validation"))
    mission_status = str(
        (summary.get("arch_metrics", {}) if isinstance(summary.get("arch_metrics", {}), dict) else {}).get(
            "mission_completion_status", "unknown"
        )
    )

    if suite == "narrow_corridor":
        primary, supporting = _compute_narrow_corridor_metrics(telemetry, bench)
    elif suite == "sharp_turns":
        primary, supporting = _compute_sharp_turn_metrics(telemetry, bench)
    elif suite == "dynamic_obstacle":
        primary, supporting = _compute_dynamic_obstacle_metrics(telemetry, bench, summary, timeline_counts)
    else:
        primary = {}
        supporting = {}

    db.close()
    return MetricsReport(
        scenario_name=scenario_name,
        suite=suite,
        result_group=result_group,
        objective=objective,
        mission_completion_status=mission_status,
        mission_duration_s=_mission_duration(telemetry),
        primary_metrics=primary,
        supporting_metrics=supporting,
        timeline_metrics=timeline_counts,
    )


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--db", required=True, help="Path to monitor SQLite db")
    parser.add_argument(
        "--timeline-events",
        default="",
        help="Optional path to monitor_timeline_event.jsonl for replay metrics",
    )
    args = parser.parse_args()

    report = compute_metrics(args.db, timeline_jsonl=args.timeline_events)
    print(json.dumps(asdict(report), ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
