#!/usr/bin/env python3
"""Batch comparison script: PP vs APP across 5 baseline scenarios, 5 repetitions each.

Total: 2 controllers × 5 scenarios × 5 reps = 50 runs.

Usage:
    python3 -m marl_car_ros2.batch_compare [--dry-run]

Output directory: /tmp/marl_batch_compare/<timestamp>/
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from .process_cleanup import kill_all_processes

# ── Configuration ──────────────────────────────────────────────────────────

CONTROLLERS = ["PP", "APP"]
SCENARIOS = [
    "high_curvature_open",
    "moderate_curvature_narrow",
    "narrow_corridor",
    "narrow_gentle_turn",
    "sharp_turns",
]
REPETITIONS = 5

RUN_TIMEOUT_S = 120  # per-run timeout
INTER_RUN_COOLDOWN_S = 3  # cooldown between runs


def _timestamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S", time.localtime())


def _kill_all() -> None:
    """Clean up any lingering ROS 2 / Nav2 / Gazebo processes.

    Delegates to process_cleanup which provides:
    - recursive child-process tree killing
    - Nav2 lifecycle / component node coverage
    - D-state detection
    - post-kill verification + retry
    """
    kill_all_processes(verbose=True)


def _run_one(
    scenario: str,
    controller: str,
    rep: int,
    batch_dir: Path,
    dry_run: bool = False,
) -> Optional[Path]:
    """Run a single benchmark and return the report JSON path, or None on failure."""
    run_id = f"{scenario}_{controller}_rep{rep}"
    report_path = batch_dir / f"report_{run_id}.json"

    cmd = [
        sys.executable, "-m", "marl_car_ros2.benchmark_runner",
        "--scenario-name", scenario,
        "--controller-profile", controller,
        "--planner-profile", "unspecified",
        "--run-timeout-s", str(RUN_TIMEOUT_S),
        "--log-dir", str(batch_dir / "logs"),
        "--report-path", str(report_path),
        "--launch-log-path", str(batch_dir / "launch_logs" / f"launch_{run_id}.log"),
    ]

    if dry_run:
        print(f"  [DRY-RUN] {' '.join(cmd)}")
        return None

    # Ensure log directories exist
    (batch_dir / "launch_logs").mkdir(parents=True, exist_ok=True)
    stdout_path = batch_dir / "launch_logs" / f"stdout_{run_id}.log"
    stderr_path = batch_dir / "launch_logs" / f"stderr_{run_id}.log"

    print(f"  [{controller}] {scenario} rep {rep} ... ", end="", flush=True)
    try:
        with stdout_path.open("w", encoding="utf-8") as stdout_f, \
             stderr_path.open("w", encoding="utf-8") as stderr_f:
            result = subprocess.run(
                cmd,
                stdout=stdout_f,
                stderr=stderr_f,
                timeout=RUN_TIMEOUT_S + 30,  # extra grace
            )
        if result.returncode != 0:
            print(f"FAIL (rc={result.returncode})")
            # Show last few stderr lines
            try:
                with stderr_path.open("r", encoding="utf-8") as f:
                    lines = f.readlines()
                for line in lines[-3:]:
                    print(f"    stderr: {line.rstrip()}")
            except Exception:
                pass
            return None
        print("OK")
        return report_path if report_path.exists() else None
    except subprocess.TimeoutExpired:
        print("TIMEOUT")
        _kill_all()
        return None
    except Exception as exc:
        print(f"ERROR: {exc}")
        _kill_all()
        return None
    finally:
        time.sleep(INTER_RUN_COOLDOWN_S)


def _parse_report(report_path: Path) -> Dict[str, object]:
    """Extract key metrics from a benchmark report JSON."""
    try:
        with report_path.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return {}
    if not isinstance(data, dict):
        return {}

    report = data.get("report", {})
    if not isinstance(report, dict):
        report = {}

    arch = report.get("arch_metrics", {})
    if not isinstance(arch, dict):
        arch = {}

    tracking = report.get("tracking_metrics", {})
    if not isinstance(tracking, dict):
        tracking = {}

    return {
        "run_id": data.get("run_id", ""),
        "stop_reason": data.get("stop_reason", ""),
        "total_wall_s": data.get("total_wall_s", 0),
        "overhead_wall_s": data.get("overhead_wall_s", 0),
        "mission_duration_s": report.get("mission_duration_s", 0),
        "mission_status": arch.get("mission_completion_status", "unknown"),
        "path_length_m": arch.get("path_length_m", 0),
        "avg_speed_mps": arch.get("avg_speed_mps", 0),
        "max_cross_track_error_m": tracking.get("max_cross_track_error_m", 0),
        "mean_cross_track_error_m": tracking.get("mean_cross_track_error_m", 0),
    }


def _generate_summary(batch_dir: Path, results: List[dict]) -> None:
    """Write summary CSV and JSON."""
    # CSV
    csv_path = batch_dir / "summary.csv"
    if results:
        fieldnames = list(results[0].keys())
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)
        print(f"\nSummary CSV: {csv_path}")

    # Grouped summary JSON
    grouped: Dict[str, Dict[str, List[dict]]] = {}
    for r in results:
        key = f"{r['scenario']}_{r['controller']}"
        grouped.setdefault(key, []).append(r)

    summary_json = {}
    for key, runs in grouped.items():
        vals = [r for r in runs if r.get("mission_status") == "success"]
        if not vals:
            summary_json[key] = {"count": len(runs), "success_count": 0, "note": "no successful runs"}
            continue
        summary_json[key] = {
            "count": len(runs),
            "success_count": len(vals),
            "avg_total_wall_s": round(sum(v["total_wall_s"] for v in vals) / len(vals), 2),
            "avg_mission_duration_s": round(sum(v["mission_duration_s"] for v in vals) / len(vals), 2),
            "avg_speed_mps": round(sum(v["avg_speed_mps"] for v in vals) / len(vals), 3),
            "avg_max_cross_track_error_m": round(
                sum(v["max_cross_track_error_m"] for v in vals) / len(vals), 4
            ),
            "avg_mean_cross_track_error_m": round(
                sum(v["mean_cross_track_error_m"] for v in vals) / len(vals), 4
            ),
        }

    summary_path = batch_dir / "summary.json"
    with summary_path.open("w", encoding="utf-8") as f:
        json.dump(summary_json, f, indent=2, ensure_ascii=False)
    print(f"Summary JSON: {summary_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Batch PP vs APP comparison across 5 scenarios.")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without executing.")
    parser.add_argument("--output-dir", default="", help="Override output directory.")
    args = parser.parse_args()

    total = len(CONTROLLERS) * len(SCENARIOS) * REPETITIONS
    ts = _timestamp()
    batch_dir = Path(args.output_dir) if args.output_dir else Path(f"/tmp/marl_batch_compare/{ts}")
    batch_dir.mkdir(parents=True, exist_ok=True)

    print(f"=== Batch Comparison: PP vs APP ===")
    print(f"Controllers:  {CONTROLLERS}")
    print(f"Scenarios:    {SCENARIOS}")
    print(f"Repetitions:  {REPETITIONS}")
    print(f"Total runs:   {total}")
    print(f"Output dir:   {batch_dir}")
    print(f"Mode:         {'DRY-RUN' if args.dry_run else 'LIVE'}")
    print()

    results: List[dict] = []
    run_num = 0
    start_wall = time.time()

    for scenario in SCENARIOS:
        for controller in CONTROLLERS:
            for rep in range(1, REPETITIONS + 1):
                run_num += 1
                print(f"[{run_num}/{total}]", end=" ")
                report_path = _run_one(scenario, controller, rep, batch_dir, dry_run=args.dry_run)

                if report_path and not args.dry_run:
                    metrics = _parse_report(report_path)
                    metrics["scenario"] = scenario
                    metrics["controller"] = controller
                    metrics["rep"] = rep
                    results.append(metrics)

    elapsed = time.time() - start_wall
    print(f"\n=== Done in {elapsed:.0f}s ===")
    print(f"Successful runs: {len(results)}/{total}")

    if results and not args.dry_run:
        _generate_summary(batch_dir, results)

    # Print quick summary table
    if results:
        print("\n=== Quick Summary ===")
        print(f"{'Scenario':<30} {'Ctrl':<6} {'#OK':<5} {'AvgDur(s)':<10} {'AvgSpd':<8} {'MaxXTE(m)':<10}")
        print("-" * 75)
        grouped: Dict[str, Dict[str, List[dict]]] = {}
        for r in results:
            gk = r["scenario"]
            ck = r["controller"]
            grouped.setdefault(gk, {}).setdefault(ck, []).append(r)
        for scenario in SCENARIOS:
            for controller in CONTROLLERS:
                runs = grouped.get(scenario, {}).get(controller, [])
                ok_runs = [r for r in runs if r.get("mission_status") == "success"]
                if ok_runs:
                    avg_dur = sum(r["mission_duration_s"] for r in ok_runs) / len(ok_runs)
                    avg_spd = sum(r["avg_speed_mps"] for r in ok_runs) / len(ok_runs)
                    avg_xte = sum(r["max_cross_track_error_m"] for r in ok_runs) / len(ok_runs)
                else:
                    avg_dur = avg_spd = avg_xte = 0
                print(
                    f"{scenario:<30} {controller:<6} {len(ok_runs):<5} "
                    f"{avg_dur:<10.2f} {avg_spd:<8.3f} {avg_xte:<10.4f}"
                )


if __name__ == "__main__":
    main()
