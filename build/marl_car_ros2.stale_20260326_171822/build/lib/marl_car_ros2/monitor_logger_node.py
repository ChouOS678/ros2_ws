from __future__ import annotations

import json
import math
import os
import sqlite3
import time
import uuid
from typing import Dict, List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .shared_types import SCHEMA_VERSION_V1, RiskLevel, normalize_experiment_event, risk_level_from_severity


class MonitorLoggerNode(Node):
    """
    Cross-agent timeline aggregator for online monitoring and offline replay.

    In addition to legacy status/event tracking, this node now records
    architecture-level thesis metrics from task_agent + supervisor.
    """

    def __init__(self) -> None:
        super().__init__("monitor_logger")
        self.declare_parameter("heartbeat_timeout_s", 2.0)
        self.declare_parameter("waiting_timeout_s", 8.0)
        self.declare_parameter("stuck_timeout_s", 8.0)
        self.declare_parameter("log_dir", "/tmp/marl_logs")
        self.declare_parameter("experiment_scenario_name", os.getenv("MARL_EXPERIMENT_SCENARIO", "unspecified"))
        self.declare_parameter("experiment_world_file", os.getenv("MARL_EXPERIMENT_WORLD_FILE", ""))
        self.declare_parameter("experiment_spawn_x", float(os.getenv("MARL_EXPERIMENT_SPAWN_X", "0.0")))
        self.declare_parameter("experiment_spawn_y", float(os.getenv("MARL_EXPERIMENT_SPAWN_Y", "0.0")))
        self.declare_parameter("experiment_spawn_z", float(os.getenv("MARL_EXPERIMENT_SPAWN_Z", "0.0")))
        self.declare_parameter("experiment_spawn_yaw", float(os.getenv("MARL_EXPERIMENT_SPAWN_YAW", "0.0")))
        self.declare_parameter("experiment_goal_x", float(os.getenv("MARL_EXPERIMENT_GOAL_X", "0.0")))
        self.declare_parameter("experiment_goal_y", float(os.getenv("MARL_EXPERIMENT_GOAL_Y", "0.0")))
        self.declare_parameter("experiment_agent_mode", os.getenv("MARL_EXPERIMENT_AGENT_MODE", "unknown"))
        self.declare_parameter("experiment_planner_profile", os.getenv("MARL_EXPERIMENT_PLANNER_PROFILE", "unspecified"))
        self.declare_parameter("experiment_run_id", os.getenv("MARL_EXPERIMENT_RUN_ID", ""))

        self.heartbeat_timeout_s = float(self.get_parameter("heartbeat_timeout_s").value)
        self.waiting_timeout_s = float(self.get_parameter("waiting_timeout_s").value)
        self.stuck_timeout_s = float(self.get_parameter("stuck_timeout_s").value)
        self.log_dir = str(self.get_parameter("log_dir").value)
        self.experiment_tags = {
            "scenario_name": str(self.get_parameter("experiment_scenario_name").value),
            "world_file": str(self.get_parameter("experiment_world_file").value),
            "spawn": {
                "x": float(self.get_parameter("experiment_spawn_x").value),
                "y": float(self.get_parameter("experiment_spawn_y").value),
                "z": float(self.get_parameter("experiment_spawn_z").value),
                "yaw": float(self.get_parameter("experiment_spawn_yaw").value),
            },
            "goal": {
                "x": float(self.get_parameter("experiment_goal_x").value),
                "y": float(self.get_parameter("experiment_goal_y").value),
            },
            "agent_mode": str(self.get_parameter("experiment_agent_mode").value),
            "planner_profile": str(self.get_parameter("experiment_planner_profile").value),
            "run_id": str(self.get_parameter("experiment_run_id").value),
        }
        os.makedirs(self.log_dir, exist_ok=True)

        self.status_file = open(os.path.join(self.log_dir, "agent_status.jsonl"), "a", encoding="utf-8")
        self.event_file = open(os.path.join(self.log_dir, "agent_event.jsonl"), "a", encoding="utf-8")
        self.summary_file = open(os.path.join(self.log_dir, "monitor_summary.jsonl"), "a", encoding="utf-8")
        self.timeline_file = open(
            os.path.join(self.log_dir, "monitor_timeline_event.jsonl"), "a", encoding="utf-8"
        )
        self.benchmark_file = open(
            os.path.join(self.log_dir, "benchmark_telemetry.jsonl"), "a", encoding="utf-8"
        )
        self.db_path = os.path.join(self.log_dir, "timeline.db")
        self.db = sqlite3.connect(self.db_path, timeout=10.0, check_same_thread=False)
        self.db.execute("PRAGMA journal_mode=WAL;")
        self.db.execute("PRAGMA synchronous=NORMAL;")
        self.db.execute("PRAGMA busy_timeout=10000;")
        self._init_db()

        self.status_cache: Dict[str, Dict[str, object]] = {}
        self.last_event_by_agent: Dict[str, float] = {}
        self.latest_world_event: Dict[str, object] = {}
        self.latest_odom_xy = (0.0, 0.0)
        self.latest_pose_yaw = 0.0
        self.latest_speed = {"linear_x": 0.0, "angular_z": 0.0}
        self.latest_cmd_vel = {"linear_x": 0.0, "angular_z": 0.0}
        self.latest_cmd_vel_nav = {"linear_x": 0.0, "angular_z": 0.0}
        self.latest_scan_min_range = -1.0
        self.latest_sim_time = 0.0
        self.dynamic_event_active = False
        self.dynamic_event_started_wall: Optional[float] = None
        self.deadlock_risk = 0
        self.starvation_risk = 0

        self.decision_latency_samples: List[float] = []
        self.supervisor_override_count = 0
        self.replan_count = 0
        self.recovery_trigger_count = 0
        self.blocked_stuck_duration_s = 0.0
        self.mission_completion_status = "unknown"
        self.last_supervisor_status: Dict[str, object] = {}
        self.last_manager_mode = ""
        self.last_mode_reason = ""
        self.last_risk_level = RiskLevel.NONE.value
        self.last_no_progress = False
        self.recovery_active = False
        self.recovery_started_wall: Optional[float] = None
        self.last_recovery_count_observed = 0
        self.last_replan_count_observed = 0
        self.topic_timing: Dict[str, Dict[str, float]] = {}
        self.last_block_reason_code = ""
        self.last_recovery_reason_code = ""

        self.create_subscription(String, "/agents/status", self._status_cb, 50)
        self.create_subscription(String, "/agents/events", self._event_cb, 100)
        self.create_subscription(String, "/world_model/events", self._world_cb, 20)
        self.create_subscription(String, "/marl/reward_breakdown", self._reward_cb, 20)
        self.create_subscription(String, "/agent/decision", self._decision_cb, 20)
        self.create_subscription(String, "/supervisor/status", self._supervisor_status_cb, 20)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 20)
        self.create_subscription(Twist, "/cmd_vel_nav", self._cmd_vel_nav_cb, 20)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)

        self.monitor_pub = self.create_publisher(String, "/monitor/summary", 20)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 20)
        self.query_srv = self.create_service(Trigger, "/monitor/query_state", self._query_state_cb)
        self.timer = self.create_timer(1.0, self._tick)

    def _init_db(self) -> None:
        cur = self.db.cursor()
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS agent_status(
                wall_time REAL,
                sim_time REAL,
                agent_id TEXT,
                state TEXT,
                task_id TEXT,
                trace_id TEXT,
                payload_json TEXT
            )
            """
        )
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS agent_event(
                wall_time REAL,
                sim_time REAL,
                event_id TEXT,
                event_type TEXT,
                sender TEXT,
                receiver TEXT,
                task_id TEXT,
                trace_id TEXT,
                payload_json TEXT
            )
            """
        )
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS anomaly(
                wall_time REAL,
                anomaly_type TEXT,
                agent_id TEXT,
                detail TEXT
            )
            """
        )
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS arch_metrics(
                wall_time REAL,
                decision_latency_ms REAL,
                supervisor_override_count INTEGER,
                replan_count INTEGER,
                recovery_trigger_count INTEGER,
                blocked_stuck_duration_s REAL,
                mission_completion_status TEXT,
                payload_json TEXT
            )
            """
        )
        cur.execute(
            """
            CREATE TABLE IF NOT EXISTS benchmark_telemetry(
                wall_time REAL,
                sim_time REAL,
                scenario_name TEXT,
                x REAL,
                y REAL,
                yaw REAL,
                linear_speed REAL,
                angular_speed REAL,
                min_range REAL,
                cmd_linear_x REAL,
                cmd_angular_z REAL,
                nav_cmd_linear_x REAL,
                nav_cmd_angular_z REAL,
                event_type TEXT,
                event_status TEXT,
                dynamic_event_active INTEGER,
                payload_json TEXT
            )
            """
        )
        self.db.commit()

    def _status_cb(self, msg: String) -> None:
        now = time.time()
        self._record_topic_timing("/agents/status", now, self._extract_float(msg.data, "sim_time"), self._extract_float(msg.data, "wall_time"))
        payload = self._parse_json(msg.data)
        if not payload:
            return
        agent_id = str(payload.get("agent_id", "unknown"))
        self.status_cache[agent_id] = payload
        self.status_file.write(json.dumps(payload, ensure_ascii=True) + "\n")
        self.status_file.flush()
        self._insert_status(payload)

    def _event_cb(self, msg: String) -> None:
        now = time.time()
        self._record_topic_timing("/agents/events", now, self._extract_float(msg.data, "sim_time"), self._extract_float(msg.data, "wall_time"))
        payload = self._parse_json(msg.data)
        if not payload:
            return
        payload = self._normalize_agent_event(payload)
        sender = str(payload.get("sender", "unknown"))
        self.last_event_by_agent[sender] = time.time()

        event_type = str(payload.get("event_type", ""))
        result = str(payload.get("result", ""))
        details = payload.get("details", {})
        if not isinstance(details, dict):
            details = {}
        if event_type == "mission_completed" or result == "goal_reached":
            self.mission_completion_status = "success"
        if event_type in ("task_failed", "mission_failed"):
            self.mission_completion_status = "failed"
        if event_type == "replan_requested":
            self.replan_count = max(self.replan_count, int(details.get("replan_count", 0)))
        if event_type == "recovery_requested":
            self.recovery_trigger_count = max(
                self.recovery_trigger_count,
                int(details.get("recovery_count", 0)),
            )
        if event_type == "supervisor_override":
            self.supervisor_override_count = max(
                self.supervisor_override_count,
                int(details.get("override_count", 0)),
            )

        self.event_file.write(json.dumps(payload, ensure_ascii=True) + "\n")
        self.event_file.flush()
        self._insert_event(payload)

    def _world_cb(self, msg: String) -> None:
        now = time.time()
        self._record_topic_timing("/world_model/events", now, self._extract_float(msg.data, "sim_time"), self._extract_float(msg.data, "wall_time"))
        payload = self._parse_json(msg.data)
        if payload:
            self.latest_world_event = normalize_experiment_event(payload)
            event_type = str(self.latest_world_event.get("event_type", ""))
            event_status = str(self.latest_world_event.get("event_status", self.latest_world_event.get("status", "")))
            if event_type == "dynamic_obstacle_triggered" and event_status in ("start", "active", ""):
                self.dynamic_event_active = True
                self.dynamic_event_started_wall = now
            elif event_type == "dynamic_obstacle_completed" and event_status in ("end", ""):
                self.dynamic_event_active = False

    def _reward_cb(self, msg: String) -> None:
        now = time.time()
        self._record_topic_timing("/marl/reward_breakdown", now, self._extract_float(msg.data, "sim_time"), self._extract_float(msg.data, "wall_time"))
        payload = self._parse_json(msg.data)
        if payload and float(payload.get("stuck", 0.0)) > 0.0:
            self.deadlock_risk += 1

    def _decision_cb(self, msg: String) -> None:
        now = time.time()
        self._record_topic_timing("/agent/decision", now, self._extract_float(msg.data, "sim_time"), self._extract_float(msg.data, "wall_time"))
        payload = self._parse_json(msg.data)
        if not payload:
            return
        lat = float(payload.get("decision_latency_ms", 0.0))
        if lat > 0.0:
            self.decision_latency_samples.append(lat)
            if len(self.decision_latency_samples) > 500:
                self.decision_latency_samples = self.decision_latency_samples[-500:]

    def _supervisor_status_cb(self, msg: String) -> None:
        now = time.time()
        self._record_topic_timing("/supervisor/status", now, self._extract_float(msg.data, "sim_time"), self._extract_float(msg.data, "wall_time"))
        payload = self._parse_json(msg.data)
        if not payload:
            return
        self.last_supervisor_status = payload
        self.supervisor_override_count = max(self.supervisor_override_count, int(payload.get("override_count", 0)))
        self.replan_count = max(self.replan_count, int(payload.get("replan_count", 0)))
        self.recovery_trigger_count = max(self.recovery_trigger_count, int(payload.get("recovery_count", 0)))
        self.blocked_stuck_duration_s = float(payload.get("blocked_duration_s", self.blocked_stuck_duration_s))
        mission_status = str(payload.get("mission_status", ""))
        if mission_status:
            self.mission_completion_status = mission_status
        self._derive_timeline_events_from_supervisor(payload, now)

    def _odom_cb(self, msg: Odometry) -> None:
        stamp_sim = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self._record_topic_timing("/odom", time.time(), stamp_sim, None)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
        cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
        self.latest_odom_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        self.latest_pose_yaw = float(math.atan2(siny_cosp, cosy_cosp))
        self.latest_speed = {
            "linear_x": float(msg.twist.twist.linear.x),
            "angular_z": float(msg.twist.twist.angular.z),
        }
        self.latest_sim_time = stamp_sim
        self._write_benchmark_sample()

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self._record_topic_timing("/cmd_vel", time.time(), None, None)
        self.latest_cmd_vel = {
            "linear_x": float(msg.linear.x),
            "angular_z": float(msg.angular.z),
        }

    def _cmd_vel_nav_cb(self, msg: Twist) -> None:
        self._record_topic_timing("/cmd_vel_nav", time.time(), None, None)
        self.latest_cmd_vel_nav = {
            "linear_x": float(msg.linear.x),
            "angular_z": float(msg.angular.z),
        }

    def _scan_cb(self, msg: LaserScan) -> None:
        self._record_topic_timing("/scan", time.time(), None, None)
        if not msg.ranges:
            self.latest_scan_min_range = -1.0
            return
        finite = [float(v) for v in msg.ranges if v == v and v != float("inf")]
        self.latest_scan_min_range = min(finite) if finite else -1.0

    def _tick(self) -> None:
        now = time.time()
        anomalies = []
        for agent_id, st in self.status_cache.items():
            hb = float(st.get("last_heartbeat_ts", 0.0))
            if now - hb > self.heartbeat_timeout_s:
                anomalies.append((agent_id, "heartbeat_lost", f"gap={now - hb:.2f}s"))

            state = str(st.get("state", ""))
            sim_time = float(st.get("sim_time", 0.0))
            blocked_reason = str(st.get("blocked_reason", ""))
            blocked_code = str(st.get("block_reason_code", ""))
            if state == "waiting":
                evt_ts = self.last_event_by_agent.get(agent_id, now)
                if now - evt_ts > self.waiting_timeout_s:
                    anomalies.append((agent_id, "dependency_waiting_too_long", f"waiting={now - evt_ts:.2f}s"))
            if state == "blocked" or blocked_reason:
                detail = blocked_code or blocked_reason or f"sim_time={sim_time:.2f}"
                anomalies.append((agent_id, "blocked_or_stuck", detail))

        self.starvation_risk = sum(1 for _, st in self.status_cache.items() if str(st.get("state", "")) == "waiting")
        for agent_id, typ, detail in anomalies:
            self._insert_anomaly(typ, agent_id, detail)

        decision_latency = self._avg(self.decision_latency_samples)

        summary = {
            "wall_time": now,
            "agent_count": len(self.status_cache),
            "experiment_tags": self.experiment_tags,
            "deadlock_risk_count": self.deadlock_risk,
            "starvation_risk_count": self.starvation_risk,
            "latest_world_event": self.latest_world_event,
            "robot_pose_xy": {"x": self.latest_odom_xy[0], "y": self.latest_odom_xy[1]},
            "anomalies": [{"agent_id": a, "type": t, "detail": d} for a, t, d in anomalies],
            "arch_metrics": {
                "agent_decision_latency_ms": decision_latency,
                "supervisor_override_count": self.supervisor_override_count,
                "replan_count": self.replan_count,
                "recovery_trigger_count": self.recovery_trigger_count,
                "blocked_stuck_duration_s": self.blocked_stuck_duration_s,
                "mission_completion_status": self.mission_completion_status,
            },
            "replay_signals": {
                "manager_mode": str(self.last_supervisor_status.get("manager_mode", "")),
                "mode_reason": str(self.last_supervisor_status.get("mode_reason", "")),
                "risk_level": self.last_risk_level,
                "no_progress_active": self.last_no_progress,
                "topic_timing": self._topic_alignment_snapshot(now),
            },
        }
        msg = String()
        msg.data = json.dumps(summary, ensure_ascii=True)
        self.monitor_pub.publish(msg)
        self.summary_file.write(msg.data + "\n")
        self.summary_file.flush()

        self._insert_arch_metrics(summary)
        self._publish_diagnostics(summary)

    def _publish_diagnostics(self, summary: Dict[str, object]) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "multi_agent_monitor"
        status.hardware_id = "sim"
        anomaly_cnt = len(summary.get("anomalies", []))
        status.level = DiagnosticStatus.WARN if anomaly_cnt > 0 else DiagnosticStatus.OK
        status.message = "anomaly_detected" if anomaly_cnt > 0 else "healthy"

        arch = summary.get("arch_metrics", {}) if isinstance(summary.get("arch_metrics", {}), dict) else {}
        status.values = [
            KeyValue(key="agent_count", value=str(summary.get("agent_count", 0))),
            KeyValue(key="deadlock_risk_count", value=str(summary.get("deadlock_risk_count", 0))),
            KeyValue(key="starvation_risk_count", value=str(summary.get("starvation_risk_count", 0))),
            KeyValue(key="decision_latency_ms", value=str(arch.get("agent_decision_latency_ms", 0.0))),
            KeyValue(key="supervisor_override_count", value=str(arch.get("supervisor_override_count", 0))),
            KeyValue(key="replan_count", value=str(arch.get("replan_count", 0))),
            KeyValue(key="recovery_trigger_count", value=str(arch.get("recovery_trigger_count", 0))),
            KeyValue(key="mission_completion_status", value=str(arch.get("mission_completion_status", "unknown"))),
        ]
        arr.status = [status]
        self.diag_pub.publish(arr)

    def _query_state_cb(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        response.success = True
        response.message = json.dumps(
            {
                "status_cache": self.status_cache,
                "latest_world_event": self.latest_world_event,
                "deadlock_risk_count": self.deadlock_risk,
                "starvation_risk_count": self.starvation_risk,
                "arch_metrics": {
                    "agent_decision_latency_ms": self._avg(self.decision_latency_samples),
                    "supervisor_override_count": self.supervisor_override_count,
                    "replan_count": self.replan_count,
                    "recovery_trigger_count": self.recovery_trigger_count,
                    "blocked_stuck_duration_s": self.blocked_stuck_duration_s,
                    "mission_completion_status": self.mission_completion_status,
                },
                "db_path": self.db_path,
                "experiment_tags": self.experiment_tags,
            },
            ensure_ascii=True,
        )
        return response

    def _insert_status(self, payload: Dict[str, object]) -> None:
        try:
            self.db.execute(
                "INSERT INTO agent_status VALUES (?, ?, ?, ?, ?, ?, ?)",
                (
                    float(payload.get("wall_time", time.time())),
                    float(payload.get("sim_time", 0.0)),
                    str(payload.get("agent_id", "")),
                    str(payload.get("state", "")),
                    str(payload.get("task_id", "")),
                    str(payload.get("trace_id", "")),
                    json.dumps(payload, ensure_ascii=True),
                ),
            )
            self.db.commit()
        except sqlite3.OperationalError as exc:
            if "locked" in str(exc).lower():
                self.get_logger().warn("timeline.db is locked; skipping one status write")
                return
            raise

    def _insert_event(self, payload: Dict[str, object]) -> None:
        try:
            self.db.execute(
                "INSERT INTO agent_event VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (
                    float(payload.get("wall_time", time.time())),
                    float(payload.get("sim_time", 0.0)),
                    str(payload.get("event_id", "")),
                    str(payload.get("event_type", "")),
                    str(payload.get("sender", "")),
                    str(payload.get("receiver", "")),
                    str(payload.get("task_id", "")),
                    str(payload.get("trace_id", "")),
                    json.dumps(payload, ensure_ascii=True),
                ),
            )
            self.db.commit()
        except sqlite3.OperationalError as exc:
            if "locked" in str(exc).lower():
                self.get_logger().warn("timeline.db is locked; skipping one event write")
                return
            raise

    def _insert_anomaly(self, anomaly_type: str, agent_id: str, detail: str) -> None:
        try:
            self.db.execute(
                "INSERT INTO anomaly VALUES (?, ?, ?, ?)",
                (time.time(), anomaly_type, agent_id, detail),
            )
            self.db.commit()
        except sqlite3.OperationalError as exc:
            if "locked" in str(exc).lower():
                self.get_logger().warn("timeline.db is locked; skipping one anomaly write")
                return
            raise

    def _insert_arch_metrics(self, summary: Dict[str, object]) -> None:
        try:
            arch = summary.get("arch_metrics", {}) if isinstance(summary.get("arch_metrics", {}), dict) else {}
            self.db.execute(
                "INSERT INTO arch_metrics VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                (
                    float(summary.get("wall_time", time.time())),
                    float(arch.get("agent_decision_latency_ms", 0.0)),
                    int(arch.get("supervisor_override_count", 0)),
                    int(arch.get("replan_count", 0)),
                    int(arch.get("recovery_trigger_count", 0)),
                    float(arch.get("blocked_stuck_duration_s", 0.0)),
                    str(arch.get("mission_completion_status", "unknown")),
                    json.dumps(summary, ensure_ascii=True),
                ),
            )
            self.db.commit()
        except sqlite3.OperationalError as exc:
            if "locked" in str(exc).lower():
                self.get_logger().warn("timeline.db is locked; skipping one arch metric write")
                return
            raise

    @staticmethod
    def _parse_json(raw: str) -> Dict[str, object]:
        try:
            out = json.loads(raw)
            if isinstance(out, dict):
                return out
            return {}
        except json.JSONDecodeError:
            return {}

    @staticmethod
    def _avg(values: List[float]) -> float:
        if not values:
            return 0.0
        return float(sum(values) / len(values))

    @staticmethod
    def _normalize_agent_event(payload: Dict[str, object]) -> Dict[str, object]:
        out = dict(payload)
        out.setdefault("schema_version", SCHEMA_VERSION_V1)
        et = str(out.get("event_type", "") or "")
        if not et:
            et = str(out.get("type", "") or "")
        if et == "replanned":
            et = "replan_requested"
        out["event_type"] = et
        return out

    @staticmethod
    def _extract_float(raw: str, key: str) -> Optional[float]:
        try:
            payload = json.loads(raw)
            if isinstance(payload, dict) and key in payload:
                return float(payload.get(key))
        except Exception:
            return None
        return None

    def _record_topic_timing(
        self,
        topic: str,
        recv_wall_time: float,
        source_sim_time: Optional[float],
        source_wall_time: Optional[float],
    ) -> None:
        row = {
            "recv_wall_time": float(recv_wall_time),
        }
        if source_sim_time is not None:
            row["source_sim_time"] = float(source_sim_time)
        if source_wall_time is not None:
            row["source_wall_time"] = float(source_wall_time)
        self.topic_timing[topic] = row

    def _topic_alignment_snapshot(self, now_wall: float) -> Dict[str, Dict[str, float]]:
        out: Dict[str, Dict[str, float]] = {}
        for topic, row in self.topic_timing.items():
            snap: Dict[str, float] = {
                "recv_wall_time": float(row.get("recv_wall_time", 0.0)),
                "age_recv_s": max(0.0, now_wall - float(row.get("recv_wall_time", now_wall))),
            }
            if "source_sim_time" in row:
                snap["source_sim_time"] = float(row["source_sim_time"])
            if "source_wall_time" in row:
                snap["source_wall_time"] = float(row["source_wall_time"])
                snap["transport_delay_s"] = max(0.0, float(row["recv_wall_time"]) - float(row["source_wall_time"]))
            out[topic] = snap
        return out

    def _derive_timeline_events_from_supervisor(self, payload: Dict[str, object], now_wall: float) -> None:
        mode = str(payload.get("manager_mode", "") or "")
        mode_reason = str(payload.get("mode_reason", "") or "")
        override_reason = str(payload.get("override_reason", "") or "")
        block_reason_code = str(payload.get("block_reason_code", "") or "")
        recovery_reason_code = str(payload.get("recovery_reason_code", "") or "")
        trace_id = str(payload.get("trace_id", ""))
        mission_id = str(payload.get("mission_id", ""))
        sim_time = float(payload.get("sim_time", now_wall))

        min_range = float(payload.get("min_range", -1.0))
        severity = float(self.latest_world_event.get("severity", 0.0)) if self.latest_world_event else 0.0
        risk = self._risk_from_signals(min_range, severity)

        common = {
            "schema_version": "monitor.timeline.v1",
            "trace_id": trace_id,
            "mission_id": mission_id,
            "sim_time": sim_time,
            "wall_time": now_wall,
            "experiment_tags": self.experiment_tags,
            "source_topic": "/supervisor/status",
            "topic_timing": self._topic_alignment_snapshot(now_wall),
            "manager_mode": mode,
            "mode_reason": mode_reason,
            "risk_level": risk,
            "min_range": min_range,
            "severity": severity,
            "override_reason": override_reason,
            "block_reason_code": block_reason_code,
            "recovery_reason_code": recovery_reason_code,
            "local_goal_bias": payload.get("local_goal_bias", None),
        }

        if mode and self.last_manager_mode and mode != self.last_manager_mode:
            self._write_timeline_event(
                event_type="mode_switch",
                phase="transition",
                details={
                    "from_mode": self.last_manager_mode,
                    "to_mode": mode,
                    "from_reason": self.last_mode_reason,
                    "to_reason": mode_reason,
                },
                common=common,
            )

        if risk != self.last_risk_level:
            self._write_timeline_event(
                event_type="risk_level_change",
                phase="transition",
                details={"from_risk": self.last_risk_level, "to_risk": risk},
                common=common,
            )

        no_progress = (
            block_reason_code == "no_progress"
            or "no_progress" in mode_reason
            or "no_progress" in override_reason
        )
        if no_progress and not self.last_no_progress:
            self._write_timeline_event(
                event_type="no_progress",
                phase="start",
                details={"reason": mode_reason or override_reason},
                common=common,
            )
        if not no_progress and self.last_no_progress:
            self._write_timeline_event(
                event_type="no_progress",
                phase="end",
                details={"reason": "cleared"},
                common=common,
            )

        recovery_count = int(payload.get("recovery_count", 0))
        if recovery_count > self.last_recovery_count_observed:
            self.recovery_active = True
            self.recovery_started_wall = now_wall
            self._write_timeline_event(
                event_type="recovery",
                phase="start",
                details={
                    "recovery_count": recovery_count,
                    "reason": mode_reason or override_reason,
                },
                common=common,
            )

        if self.recovery_active and self.last_manager_mode == "RECOVERY_REQUESTED" and mode in ("CAUTIOUS", "NOMINAL"):
            dur = 0.0
            if self.recovery_started_wall is not None:
                dur = max(0.0, now_wall - self.recovery_started_wall)
            self._write_timeline_event(
                event_type="recovery",
                phase="end",
                details={
                    "duration_s": dur,
                    "end_mode": mode,
                    "effective": mode in ("CAUTIOUS", "NOMINAL"),
                },
                common=common,
            )
            self.recovery_active = False
            self.recovery_started_wall = None

        replan_count = int(payload.get("replan_count", 0))
        if replan_count > self.last_replan_count_observed:
            self._write_timeline_event(
                event_type="replan",
                phase="start",
                details={"replan_count": replan_count, "reason": mode_reason or override_reason},
                common=common,
            )

        self.last_manager_mode = mode
        self.last_mode_reason = mode_reason
        self.last_risk_level = risk
        self.last_no_progress = no_progress
        self.last_recovery_count_observed = recovery_count
        self.last_replan_count_observed = replan_count
        self.last_block_reason_code = block_reason_code
        self.last_recovery_reason_code = recovery_reason_code

    @staticmethod
    def _risk_from_signals(min_range: float, severity: float) -> str:
        if min_range > 0.0:
            if min_range <= 0.24:
                return RiskLevel.CRITICAL.value
            if min_range <= 0.55:
                return RiskLevel.HIGH.value
            if min_range <= 0.9:
                return RiskLevel.MEDIUM.value
        sev_level = risk_level_from_severity(severity)
        order = {
            RiskLevel.NONE.value: 0,
            RiskLevel.LOW.value: 1,
            RiskLevel.MEDIUM.value: 2,
            RiskLevel.HIGH.value: 3,
            RiskLevel.CRITICAL.value: 4,
        }
        if order.get(sev_level, 0) > order.get(RiskLevel.NONE.value, 0):
            return sev_level
        return RiskLevel.NONE.value

    def _write_timeline_event(self, *, event_type: str, phase: str, details: Dict[str, object], common: Dict[str, object]) -> None:
        payload = dict(common)
        payload["event_id"] = f"mevt-{uuid.uuid4().hex[:12]}"
        payload["event_type"] = event_type
        payload["event_phase"] = phase
        payload["details"] = details
        self.timeline_file.write(json.dumps(payload, ensure_ascii=True) + "\n")
        self.timeline_file.flush()

    def _write_benchmark_sample(self) -> None:
        scenario_name = str(self.experiment_tags.get("scenario_name", "unspecified"))
        event_type = str(self.latest_world_event.get("event_type", "")) if self.latest_world_event else ""
        event_status = str(self.latest_world_event.get("event_status", self.latest_world_event.get("status", ""))) if self.latest_world_event else ""
        payload = {
            "wall_time": time.time(),
            "sim_time": self.latest_sim_time,
            "scenario_name": scenario_name,
            "x": self.latest_odom_xy[0],
            "y": self.latest_odom_xy[1],
            "yaw": self.latest_pose_yaw,
            "linear_speed": self.latest_speed["linear_x"],
            "angular_speed": self.latest_speed["angular_z"],
            "min_range": self.latest_scan_min_range,
            "cmd_linear_x": self.latest_cmd_vel["linear_x"],
            "cmd_angular_z": self.latest_cmd_vel["angular_z"],
            "nav_cmd_linear_x": self.latest_cmd_vel_nav["linear_x"],
            "nav_cmd_angular_z": self.latest_cmd_vel_nav["angular_z"],
            "event_type": event_type,
            "event_status": event_status,
            "dynamic_event_active": self.dynamic_event_active,
            "experiment_tags": self.experiment_tags,
        }
        self.benchmark_file.write(json.dumps(payload, ensure_ascii=True) + "\n")
        self.benchmark_file.flush()
        try:
            self.db.execute(
                "INSERT INTO benchmark_telemetry VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (
                    float(payload["wall_time"]),
                    float(payload["sim_time"]),
                    scenario_name,
                    float(payload["x"]),
                    float(payload["y"]),
                    float(payload["yaw"]),
                    float(payload["linear_speed"]),
                    float(payload["angular_speed"]),
                    float(payload["min_range"]),
                    float(payload["cmd_linear_x"]),
                    float(payload["cmd_angular_z"]),
                    float(payload["nav_cmd_linear_x"]),
                    float(payload["nav_cmd_angular_z"]),
                    event_type,
                    event_status,
                    1 if self.dynamic_event_active else 0,
                    json.dumps(payload, ensure_ascii=True),
                ),
            )
            self.db.commit()
        except sqlite3.OperationalError as exc:
            if "locked" in str(exc).lower():
                self.get_logger().warn("timeline.db is locked; skipping one benchmark telemetry write")
                return
            raise

    def destroy_node(self) -> bool:
        try:
            self.status_file.close()
            self.event_file.close()
            self.summary_file.close()
            self.timeline_file.close()
            self.benchmark_file.close()
        finally:
            self.db.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MonitorLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
