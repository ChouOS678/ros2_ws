import json
import sqlite3
import tempfile
import unittest
from pathlib import Path

from backend.analytics import overview
from backend.store import connect, import_telemetry, persist_report


def report(run_id, profile, error=0.1, status="success"):
    return {
        "run_id": run_id,
        "stop_reason": f"terminal_status:{status}",
        "launch_command": ["ros2", "launch", "evaluation.launch.py", "scenario_name:=s_curve", f"planner_profile:={profile}"],
        "report": {
            "scenario_name": "s_curve",
            "mission_completion_status": status,
            "mission_duration_s": 12.0,
            "primary_metrics": {"mean_tracking_error_m": error, "average_speed_mps": 0.4},
            "supporting_metrics": {"telemetry_sample_count": 3},
            "timeline_metrics": {},
        },
    }


class PortalTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.db = connect(self.root / "portal.sqlite")

    def tearDown(self):
        self.db.close()
        self.temp.cleanup()

    def test_report_import_is_idempotent_and_extracts_controller(self):
        first = persist_report(self.db, report("s_curve_pp_001", "pp"), "report.json")
        persist_report(self.db, report("s_curve_pp_001", "pp", error=0.2), "report.json")
        self.assertEqual(first["scenario"], "s_curve")
        self.assertEqual(first["controller"], "pp")
        self.assertEqual(self.db.execute("SELECT count(*) FROM runs").fetchone()[0], 1)
        saved = self.db.execute("SELECT primary_metrics_json FROM runs").fetchone()[0]
        self.assertEqual(json.loads(saved)["mean_tracking_error_m"], 0.2)

    def test_placeholder_scenario_falls_back_to_launch_argument(self):
        payload = report("s_curve_app_001", "app")
        payload["report"]["scenario_name"] = "unspecified"
        saved = persist_report(self.db, payload, "report.json")
        self.assertEqual(saved["scenario"], "s_curve")

    def test_telemetry_is_linked_by_run_id_and_metrics_are_aggregated(self):
        for run_id, profile, error in [("s_curve_pp_001", "pp", 0.10), ("s_curve_pp_002", "pp", 0.20), ("s_curve_pp_003", "pp", 0.30), ("s_curve_app_001", "app", 0.05)]:
            persist_report(self.db, report(run_id, profile, error), "report.json")

        source = self.root / "timeline.db"
        telemetry = sqlite3.connect(source)
        telemetry.execute("CREATE TABLE benchmark_telemetry(wall_time REAL, payload_json TEXT)")
        for index, run_id in enumerate(("s_curve_pp_001", "s_curve_app_001", "unimported_run")):
            payload = {"wall_time": index + 1, "sim_time": index, "x": index * 0.1, "y": 0.0,
                       "yaw": 0.0, "linear_speed": 0.5, "angular_speed": 0.1, "min_range": 0.4,
                       "experiment_tags": {"run_id": run_id}}
            telemetry.execute("INSERT INTO benchmark_telemetry VALUES (?, ?)", (index + 1, json.dumps(payload)))
        telemetry.commit()
        telemetry.close()

        count = import_telemetry(self.db, source, {"s_curve_pp_001", "s_curve_app_001"})
        self.assertEqual(count, 3)
        result = overview(self.db)
        case = result["cases"][0]
        pp = next(item for item in case["algorithms"] if item["controller"] == "pp")
        app = next(item for item in case["algorithms"] if item["controller"] == "app")
        self.assertAlmostEqual(pp["averages"]["mean_tracking_error_m"], 0.2)
        self.assertTrue(pp["repeat_ready"])
        self.assertAlmostEqual(app["averages"]["angular_speed_rms_radps"], 0.1)
        self.assertEqual(result["summary"]["telemetry_run_count"], 3)
        self.assertEqual(self.db.execute("SELECT count(*) FROM runs").fetchone()[0], 5)

    def test_bad_report_and_bad_telemetry_schema_are_rejected(self):
        with self.assertRaises(ValueError):
            persist_report(self.db, {"report": {}}, "bad.json")
        bad_db = self.root / "empty.db"
        sqlite3.connect(bad_db).close()
        with self.assertRaises(ValueError):
            import_telemetry(self.db, bad_db)

    def test_latest_campaign_is_separated_from_historical_runs(self):
        persist_report(self.db, report("straight_pp_old", "pp"), "acceptance_pp.json")
        campaign_id = "campaign_20260924_114023"
        persist_report(self.db, report("straight_app_new", "app"), f"benchmark_portal/data/campaigns/{campaign_id}/straight_app/report.json")
        current = overview(self.db)
        self.assertEqual(current["active_campaign"], campaign_id)
        self.assertEqual(current["summary"]["run_count"], 1)
        history = overview(self.db, "historical")
        self.assertEqual(history["summary"]["run_count"], 1)
        combined = overview(self.db, "all")
        self.assertEqual(combined["summary"]["run_count"], 2)


if __name__ == "__main__":
    unittest.main()
