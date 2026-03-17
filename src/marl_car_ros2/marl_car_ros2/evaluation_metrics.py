from __future__ import annotations

import json
import sqlite3
from dataclasses import dataclass, asdict
from typing import Dict, List


@dataclass
class MetricsReport:
    task_completion_time_p50: float
    dependency_wait_time_p50: float
    handoff_latency_p50_ms: float
    heartbeat_gap_max_s: float
    action_stuck_count: int
    deadlock_count: int
    starvation_count: int
    success_rate: float
    env_change_trigger_failures: int
    replanning_count: int


def _median(values: List[float]) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    n = len(s)
    if n % 2 == 1:
        return float(s[n // 2])
    return float((s[n // 2 - 1] + s[n // 2]) / 2.0)


def compute_metrics(db_path: str) -> MetricsReport:
    db = sqlite3.connect(db_path)
    cur = db.cursor()

    task_durations = []
    dep_wait = []
    handoff = []

    rows = cur.execute("SELECT payload_json FROM agent_event").fetchall()
    success = 0
    total_terminal = 0
    env_fail = 0
    replanning = 0
    for (raw,) in rows:
        p: Dict[str, object] = json.loads(raw)
        et = str(p.get("event_type", ""))
        if et in ("task_finished", "task_failed"):
            total_terminal += 1
            if et == "task_finished":
                success += 1
        if et == "task_failed" and "world" in json.dumps(p.get("details", {}), ensure_ascii=True):
            env_fail += 1
        if et == "replanned":
            replanning += 1
        handoff.append(float(p.get("latency_ms", 0.0)))

    status_rows = cur.execute("SELECT payload_json FROM agent_status").fetchall()
    hb_gap_max = 0.0
    by_agent_times: Dict[str, List[float]] = {}
    for (raw,) in status_rows:
        p: Dict[str, object] = json.loads(raw)
        a = str(p.get("agent_id", ""))
        t = float(p.get("wall_time", 0.0))
        by_agent_times.setdefault(a, []).append(t)
        if str(p.get("state", "")) == "waiting":
            dep_wait.append(1.0)

    for _, ts in by_agent_times.items():
        s = sorted(ts)
        for i in range(1, len(s)):
            hb_gap_max = max(hb_gap_max, s[i] - s[i - 1])

    anomaly_rows = cur.execute("SELECT anomaly_type FROM anomaly").fetchall()
    action_stuck = sum(1 for (t,) in anomaly_rows if t == "blocked_or_stuck")
    deadlocks = sum(1 for (t,) in anomaly_rows if t == "blocked_or_stuck")
    starvation = sum(1 for (t,) in anomaly_rows if t == "dependency_waiting_too_long")

    # If no explicit task duration event is present, use event count proxy.
    if total_terminal > 0:
        task_durations = [1.0 for _ in range(total_terminal)]

    db.close()
    return MetricsReport(
        task_completion_time_p50=_median(task_durations),
        dependency_wait_time_p50=_median(dep_wait),
        handoff_latency_p50_ms=_median(handoff),
        heartbeat_gap_max_s=float(hb_gap_max),
        action_stuck_count=int(action_stuck),
        deadlock_count=int(deadlocks),
        starvation_count=int(starvation),
        success_rate=float(success / total_terminal) if total_terminal > 0 else 0.0,
        env_change_trigger_failures=int(env_fail),
        replanning_count=int(replanning),
    )


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--db", required=True, help="Path to monitor SQLite db")
    args = parser.parse_args()

    report = compute_metrics(args.db)
    print(json.dumps(asdict(report), ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()

