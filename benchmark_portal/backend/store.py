from __future__ import annotations

import json
import math
import sqlite3
from pathlib import Path
from typing import Any


def connect(database: Path) -> sqlite3.Connection:
    database.parent.mkdir(parents=True, exist_ok=True)
    db = sqlite3.connect(database)
    db.row_factory = sqlite3.Row
    db.execute("PRAGMA foreign_keys = ON")
    db.executescript(
        """
        CREATE TABLE IF NOT EXISTS runs (
            run_id TEXT PRIMARY KEY,
            scenario TEXT NOT NULL,
            controller TEXT NOT NULL,
            status TEXT NOT NULL,
            stop_reason TEXT NOT NULL DEFAULT '',
            mission_duration_s REAL,
            primary_metrics_json TEXT NOT NULL,
            supporting_metrics_json TEXT NOT NULL,
            timeline_metrics_json TEXT NOT NULL,
            imported_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
            source_name TEXT NOT NULL DEFAULT ''
        );
        CREATE TABLE IF NOT EXISTS telemetry_samples (
            run_id TEXT NOT NULL REFERENCES runs(run_id) ON DELETE CASCADE,
            sample_index INTEGER NOT NULL,
            wall_time REAL,
            sim_time REAL,
            x REAL,
            y REAL,
            yaw REAL,
            linear_speed REAL,
            angular_speed REAL,
            min_range REAL,
            cmd_linear_x REAL,
            cmd_angular_z REAL,
            PRIMARY KEY (run_id, sample_index)
        );
        CREATE INDEX IF NOT EXISTS telemetry_run_time ON telemetry_samples(run_id, sim_time);
        """
    )
    db.commit()
    return db


def _finite_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _args(run: dict[str, Any]) -> list[str]:
    command = run.get("launch_command", [])
    return [str(value) for value in command] if isinstance(command, list) else []


def _argument_value(args: list[str], name: str) -> str:
    prefix = f"{name}:="
    for value in args:
        if value.startswith(prefix):
            return value[len(prefix):]
    return ""


def normalize_report(payload: dict[str, Any], source_name: str = "") -> dict[str, Any]:
    report = payload.get("report", payload)
    if not isinstance(report, dict):
        raise ValueError("report must be an object")
    args = _args(payload)
    run_id = str(payload.get("run_id", "")).strip()
    if not run_id:
        raise ValueError("report is missing run_id")
    scenario = str(report.get("scenario_name") or "")
    if scenario.lower() in {"", "unspecified", "unknown", "none"}:
        scenario = _argument_value(args, "scenario_name") or "unknown"
    controller = str(_argument_value(args, "planner_profile") or "unknown")
    if controller == "unknown":
        parts = run_id.split("_")
        controller = next((item.lower() for item in ("pp", "app", "rpp", "dwpp") if item in parts), "unknown")
    if controller.lower() in {"", "unspecified", "none"}:
        controller = "unknown"
    primary = report.get("primary_metrics", {})
    supporting = report.get("supporting_metrics", {})
    timeline = report.get("timeline_metrics", {})
    if not all(isinstance(item, dict) for item in (primary, supporting, timeline)):
        raise ValueError("metric groups must be objects")
    mission = str(report.get("mission_completion_status", "unknown"))
    stop_reason = str(payload.get("stop_reason", ""))
    status = "success" if mission.lower() in {"success", "completed", "goal_reached"} or stop_reason.startswith("terminal_status:success") else "failed" if mission.lower() == "failed" or "timeout" in stop_reason.lower() or "failed" in stop_reason.lower() else "unknown"
    return {
        "run_id": run_id,
        "scenario": scenario,
        "controller": controller.lower(),
        "status": status,
        "stop_reason": stop_reason,
        "mission_duration_s": _finite_or_none(report.get("mission_duration_s")),
        "primary_metrics": primary,
        "supporting_metrics": supporting,
        "timeline_metrics": timeline,
        "source_name": source_name,
    }


def persist_report(db: sqlite3.Connection, payload: dict[str, Any], source_name: str = "") -> dict[str, Any]:
    run = normalize_report(payload, source_name)
    db.execute(
        """INSERT INTO runs(run_id, scenario, controller, status, stop_reason, mission_duration_s,
           primary_metrics_json, supporting_metrics_json, timeline_metrics_json, source_name)
           VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
           ON CONFLICT(run_id) DO UPDATE SET scenario=excluded.scenario, controller=excluded.controller,
           status=excluded.status, stop_reason=excluded.stop_reason, mission_duration_s=excluded.mission_duration_s,
           primary_metrics_json=excluded.primary_metrics_json, supporting_metrics_json=excluded.supporting_metrics_json,
           timeline_metrics_json=excluded.timeline_metrics_json, source_name=excluded.source_name""",
        (
            run["run_id"], run["scenario"], run["controller"], run["status"], run["stop_reason"],
            run["mission_duration_s"], json.dumps(run["primary_metrics"]),
            json.dumps(run["supporting_metrics"]), json.dumps(run["timeline_metrics"]), run["source_name"],
        ),
    )
    db.commit()
    return run


def import_telemetry(db: sqlite3.Connection, source: Path, run_ids: set[str] | None = None) -> int:
    source_db = sqlite3.connect(f"file:{source.resolve()}?mode=ro", uri=True)
    source_db.row_factory = sqlite3.Row
    try:
        columns = {row[1] for row in source_db.execute("PRAGMA table_info(benchmark_telemetry)")}
        if "payload_json" not in columns:
            raise ValueError("SQLite source has no benchmark_telemetry.payload_json table")
        payloads = source_db.execute("SELECT payload_json FROM benchmark_telemetry ORDER BY wall_time").fetchall()
        arch_status = {}
        try:
            for row in source_db.execute("SELECT payload_json FROM arch_metrics ORDER BY wall_time"):
                try:
                    summary = json.loads(row["payload_json"])
                    tags = summary.get("experiment_tags", {})
                    arch = summary.get("arch_metrics", {})
                    if isinstance(tags, dict) and isinstance(arch, dict) and tags.get("run_id"):
                        arch_status[str(tags["run_id"])] = str(arch.get("mission_completion_status", "unknown"))
                except (json.JSONDecodeError, TypeError):
                    continue
        except sqlite3.OperationalError:
            pass
    except sqlite3.OperationalError as error:
        raise ValueError("SQLite source has no benchmark_telemetry table") from error
    finally:
        source_db.close()

    inserted = 0
    grouped: dict[str, list[dict[str, Any]]] = {}
    for row in payloads:
        try:
            sample = json.loads(row["payload_json"])
        except (json.JSONDecodeError, TypeError):
            continue
        tags = sample.get("experiment_tags", {}) if isinstance(sample, dict) else {}
        run_id = str(tags.get("run_id", "")) if isinstance(tags, dict) else ""
        if not run_id:
            continue
        grouped.setdefault(run_id, []).append(sample)

    for run_id, samples in grouped.items():
        exists = db.execute("SELECT 1 FROM runs WHERE run_id=?", (run_id,)).fetchone()
        if not exists:
            tags = samples[0].get("experiment_tags", {})
            if not isinstance(tags, dict):
                tags = {}
            controller = str(tags.get("planner_profile", "unknown")).lower()
            if controller in {"", "unspecified", "none"}:
                controller = "unknown"
            mission = arch_status.get(run_id, "unknown").lower()
            status = "success" if mission in {"success", "completed", "goal_reached"} else "failed" if mission == "failed" else "unknown"
            times = [_finite_or_none(sample.get("sim_time")) for sample in samples]
            times = [value for value in times if value is not None]
            duration = max(times) - min(times) if len(times) > 1 else None
            db.execute(
                "INSERT INTO runs(run_id, scenario, controller, status, mission_duration_s, primary_metrics_json, supporting_metrics_json, timeline_metrics_json, source_name) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (run_id, str(tags.get("scenario_name", "unknown")), controller, status, duration, "{}", "{}", "{}", "telemetry database"),
            )
        elif run_ids is not None and run_id not in run_ids:
            continue
        db.execute("DELETE FROM telemetry_samples WHERE run_id=?", (run_id,))
        normalized = []
        for sample in samples:
            normalized.append((
                run_id, len(normalized), _finite_or_none(sample.get("wall_time")),
                _finite_or_none(sample.get("sim_time")), _finite_or_none(sample.get("x")),
                _finite_or_none(sample.get("y")), _finite_or_none(sample.get("yaw")),
                _finite_or_none(sample.get("linear_speed")), _finite_or_none(sample.get("angular_speed")),
                _finite_or_none(sample.get("min_range")), _finite_or_none(sample.get("cmd_linear_x")),
                _finite_or_none(sample.get("cmd_angular_z")),
            ))
        db.executemany("INSERT INTO telemetry_samples VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)", normalized)
        inserted += len(normalized)
    db.commit()
    return inserted
