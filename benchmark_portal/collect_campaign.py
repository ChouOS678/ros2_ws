from __future__ import annotations

import json
import shutil
import sqlite3
import subprocess
import sys
import time
from pathlib import Path

from .backend.store import connect, import_telemetry, persist_report


ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = Path(__file__).resolve().parent / "data"
MONITOR_DIR = Path("/tmp/marl_logs")
SCENARIOS = ("straight", "constant_curvature", "s_curve", "clothoid", "sharp_corner")
CONTROLLERS = ("pp", "app", "rpp", "dwpp")


def main() -> int:
    campaign_id = time.strftime("campaign_%Y%m%d_%H%M%S")
    campaign_dir = DATA_DIR / "campaigns" / campaign_id
    campaign_dir.mkdir(parents=True, exist_ok=False)
    if MONITOR_DIR.is_dir():
        shutil.copytree(MONITOR_DIR, campaign_dir / "pre_campaign_marl_logs")
    portal_db = connect(DATA_DIR / "benchmark.sqlite")
    outcomes = []

    for scenario in SCENARIOS:
        for controller in CONTROLLERS:
            case_dir = campaign_dir / f"{scenario}_{controller}"
            case_dir.mkdir()
            archive_dir = case_dir / "monitor"
            report_path = case_dir / "report.json"
            command = [
                sys.executable, "-m", "marl_car_ros2.benchmark_runner",
                "--scenario-name", scenario,
                "--planner-profile", controller,
                "--run-timeout-s", "90",
                "--log-dir", str(MONITOR_DIR),
                "--report-path", str(report_path),
                "--launch-log-path", str(case_dir / "launch.log"),
            ]
            print(f"START {len(outcomes) + 1:02d}/20 {scenario} + {controller}", flush=True)
            with (case_dir / "runner.log").open("w", encoding="utf-8") as output:
                process = subprocess.run(command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT, check=False)

            result = {"scenario": scenario, "controller": controller, "return_code": process.returncode, "run_id": "", "status": "runner_failed", "telemetry_samples": 0}
            if MONITOR_DIR.is_dir():
                shutil.copytree(MONITOR_DIR, archive_dir)
            if report_path.is_file():
                try:
                    with report_path.open("r", encoding="utf-8") as report_file:
                        payload = json.load(report_file)
                    payload["db_path"] = str(archive_dir / "timeline.db")
                    payload["summary_path"] = str(archive_dir / "monitor_summary.jsonl")
                    payload["timeline_events_path"] = str(archive_dir / "monitor_timeline_event.jsonl")
                    report_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                    run = persist_report(portal_db, payload, str(report_path.relative_to(ROOT)))
                    telemetry_db = Path(str(payload.get("db_path", "")))
                    if telemetry_db.is_file():
                        result["telemetry_samples"] = import_telemetry(portal_db, telemetry_db, {run["run_id"]})
                    result.update({"run_id": run["run_id"], "status": run["status"], "stop_reason": run["stop_reason"]})
                except (OSError, ValueError, sqlite3.Error) as error:
                    result["error"] = str(error)
            outcomes.append(result)
            summary_path = campaign_dir / "campaign_summary.json"
            summary_path.write_text(json.dumps({"campaign_id": campaign_id, "completed_runs": len(outcomes), "planned_runs": 20, "results": outcomes}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(f"DONE  {len(outcomes):02d}/20 {scenario} + {controller}: {result['status']}, telemetry={result['telemetry_samples']}", flush=True)

    portal_db.close()
    successful = sum(result["status"] == "success" for result in outcomes)
    print(f"CAMPAIGN {campaign_id}: {successful}/20 successful runs; reports and telemetry stored in {DATA_DIR}", flush=True)
    return 0 if len(outcomes) == 20 else 1


if __name__ == "__main__":
    raise SystemExit(main())
