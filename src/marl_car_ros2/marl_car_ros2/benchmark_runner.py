from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, Optional, Sequence

from .evaluation_metrics import compute_metrics


TERMINAL_MISSION_STATES = {"success", "failed", "degraded"}


def _timestamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S", time.localtime())


def _read_last_summary(summary_path: Path) -> Dict[str, object]:
    if not summary_path.exists():
        return {}
    try:
        with summary_path.open("r", encoding="utf-8") as f:
            lines = [ln.strip() for ln in f if ln.strip()]
    except OSError:
        return {}
    if not lines:
        return {}
    try:
        payload = json.loads(lines[-1])
        return payload if isinstance(payload, dict) else {}
    except json.JSONDecodeError:
        return {}


def _terminate_proc(proc: subprocess.Popen[str], grace_s: float = 8.0) -> None:
    descendants = _descendant_pids(proc.pid)
    for pid in reversed(descendants):
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass
    deadline = time.time() + grace_s
    while time.time() < deadline:
        if proc.poll() is not None and not any(_pid_is_running(pid) for pid in descendants):
            return
        time.sleep(0.2)
    for pid in reversed(descendants):
        if not _pid_is_running(pid):
            continue
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except ProcessLookupError:
        pass


def _descendant_pids(root_pid: int) -> list[int]:
    children: dict[int, list[int]] = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text(encoding="utf-8")
            fields = stat[stat.rfind(")") + 2 :].split()
            parent_pid = int(fields[1])
        except (OSError, IndexError, ValueError):
            continue
        children.setdefault(parent_pid, []).append(int(entry.name))

    descendants: list[int] = []
    pending = list(children.get(root_pid, []))
    while pending:
        pid = pending.pop()
        descendants.append(pid)
        pending.extend(children.get(pid, []))
    return descendants


def _pid_is_running(pid: int) -> bool:
    try:
        stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except OSError:
        return False
    return stat[stat.rfind(")") + 2 :].split()[0] != "Z"


def _build_launch_cmd(args: argparse.Namespace, run_id: str) -> Sequence[str]:
    cmd = [
        "ros2",
        "launch",
        "marl_car_ros2",
        "evaluation.launch.py",
        f"scenario_name:={args.scenario_name}",
        f"agent_mode:={str(args.agent_mode).lower()}",
        f"planner_profile:={args.planner_profile}",
        f"run_id:={run_id}",
        f"start_gazebo:={str(args.start_gazebo).lower()}",
        f"start_monitor:={str(args.start_monitor).lower()}",
        f"start_mutator:={str(args.start_mutator).lower()}",
        f"start_bridge:={str(args.start_bridge).lower()}",
        f"start_rviz:={str(args.start_rviz).lower()}",
        f"start_visualizer:={str(args.start_visualizer).lower()}",
    ]
    if args.params_file:
        cmd.append(f"params_file:={args.params_file}")
    return cmd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one benchmark scenario and emit benchmark metrics JSON.")
    parser.add_argument(
        "--scenario-name",
        default="narrow_corridor",
        choices=("straight", "constant_curvature", "s_curve", "clothoid", "sharp_corner", "custom"),
        help="Benchmark scenario to run.",
    )
    parser.add_argument("--planner-profile", default="unspecified", help="planner_profile passed to evaluation launch.")
    parser.add_argument("--params-file", default="", help="Optional explicit Nav2 params file override.")
    parser.add_argument("--agent-mode", action="store_true", default=True, help="Run in agent mode.")
    parser.add_argument("--no-agent-mode", dest="agent_mode", action="store_false", help="Run baseline mode.")
    parser.add_argument("--start-gazebo", action="store_true", default=True)
    parser.add_argument("--no-start-gazebo", dest="start_gazebo", action="store_false")
    parser.add_argument("--start-monitor", action="store_true", default=True)
    parser.add_argument("--no-start-monitor", dest="start_monitor", action="store_false")
    parser.add_argument("--start-mutator", action="store_true", default=True)
    parser.add_argument("--no-start-mutator", dest="start_mutator", action="store_false")
    parser.add_argument("--start-bridge", action="store_true", default=True)
    parser.add_argument("--no-start-bridge", dest="start_bridge", action="store_false")
    parser.add_argument("--start-rviz", action="store_true", help="Start RViz with the benchmark run.")
    parser.add_argument("--start-visualizer", action="store_true", help="Publish the odometry trace for RViz.")
    parser.add_argument("--log-dir", default="/tmp/marl_logs", help="Directory used by monitor_logger.")
    parser.add_argument("--report-path", default="", help="Optional explicit output report path.")
    parser.add_argument("--launch-log-path", default="", help="Optional explicit launch stdout/stderr log path.")
    parser.add_argument("--run-timeout-s", type=float, default=90.0, help="Hard timeout for a single run.")
    parser.add_argument(
        "--terminal-grace-s",
        type=float,
        default=None,
        help="Minimum runtime before terminal mission states are accepted; defaults to 40s for trajectory cases and 8s otherwise.",
    )
    parser.add_argument(
        "--settle-after-terminal-s",
        type=float,
        default=2.0,
        help="Extra wait after observing terminal mission state before stopping launch.",
    )
    parser.add_argument(
        "--keep-log-dir",
        action="store_true",
        help="Keep existing log_dir contents instead of clearing them before the run.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.terminal_grace_s is None:
        args.terminal_grace_s = 40.0 if args.scenario_name in {
            "straight",
            "constant_curvature",
            "s_curve",
            "clothoid",
            "sharp_corner",
        } else 8.0
    run_id = f"{args.scenario_name}_{args.planner_profile}_{_timestamp()}"
    log_dir = Path(args.log_dir)
    if log_dir.exists() and not args.keep_log_dir:
        shutil.rmtree(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    summary_path = log_dir / "monitor_summary.jsonl"
    db_path = log_dir / "timeline.db"
    timeline_path = log_dir / "monitor_timeline_event.jsonl"
    report_path = Path(args.report_path) if args.report_path else (log_dir / f"benchmark_report_{run_id}.json")
    launch_log_path = Path(args.launch_log_path) if args.launch_log_path else (log_dir / f"launch_{run_id}.log")

    launch_cmd = _build_launch_cmd(args, run_id)
    with launch_log_path.open("w", encoding="utf-8") as log_f:
        proc = subprocess.Popen(
            launch_cmd,
            stdout=log_f,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )

        start_wall = time.time()
        final_summary: Dict[str, object] = {}
        stop_reason = "timeout"
        try:
            while True:
                if proc.poll() is not None:
                    stop_reason = f"launch_exit_rc={proc.returncode}"
                    break

                elapsed = time.time() - start_wall
                if elapsed >= args.run_timeout_s:
                    stop_reason = "run_timeout"
                    break

                final_summary = _read_last_summary(summary_path)
                arch = final_summary.get("arch_metrics", {}) if isinstance(final_summary.get("arch_metrics", {}), dict) else {}
                mission_status = str(arch.get("mission_completion_status", "unknown"))
                if mission_status in TERMINAL_MISSION_STATES and elapsed >= args.terminal_grace_s:
                    time.sleep(max(args.settle_after_terminal_s, 0.0))
                    stop_reason = f"terminal_status:{mission_status}"
                    break

                time.sleep(1.0)
        finally:
            _terminate_proc(proc)
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                pass

    if not db_path.exists():
        raise RuntimeError(f"Benchmark run ended without metrics DB: {db_path} ({stop_reason})")

    report = compute_metrics(str(db_path), timeline_jsonl=str(timeline_path))
    payload = {
        "run_id": run_id,
        "stop_reason": stop_reason,
        "launch_command": list(launch_cmd),
        "db_path": str(db_path),
        "timeline_events_path": str(timeline_path),
        "summary_path": str(summary_path),
        "launch_log_path": str(launch_log_path),
        "report": asdict(report),
    }
    with report_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=True, indent=2)
        f.write("\n")
    print(json.dumps(payload, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"benchmark_runner failed: {exc}", file=sys.stderr)
        raise
