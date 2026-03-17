from __future__ import annotations

import json
import os
import sqlite3
import time
from typing import Dict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MonitorLoggerNode(Node):
    """
    Cross-agent timeline aggregator for online monitoring and offline replay.
    """

    def __init__(self) -> None:
        super().__init__("monitor_logger")
        self.declare_parameter("heartbeat_timeout_s", 2.0)
        self.declare_parameter("waiting_timeout_s", 8.0)
        self.declare_parameter("stuck_timeout_s", 8.0)
        self.declare_parameter("log_dir", "/tmp/marl_logs")

        self.heartbeat_timeout_s = float(self.get_parameter("heartbeat_timeout_s").value)
        self.waiting_timeout_s = float(self.get_parameter("waiting_timeout_s").value)
        self.stuck_timeout_s = float(self.get_parameter("stuck_timeout_s").value)
        self.log_dir = str(self.get_parameter("log_dir").value)
        os.makedirs(self.log_dir, exist_ok=True)

        self.status_file = open(os.path.join(self.log_dir, "agent_status.jsonl"), "a", encoding="utf-8")
        self.event_file = open(os.path.join(self.log_dir, "agent_event.jsonl"), "a", encoding="utf-8")
        self.summary_file = open(os.path.join(self.log_dir, "monitor_summary.jsonl"), "a", encoding="utf-8")
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
        self.deadlock_risk = 0
        self.starvation_risk = 0

        self.create_subscription(String, "/agents/status", self._status_cb, 50)
        self.create_subscription(String, "/agents/events", self._event_cb, 100)
        self.create_subscription(String, "/world_model/events", self._world_cb, 20)
        self.create_subscription(String, "/marl/reward_breakdown", self._reward_cb, 20)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)

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
        self.db.commit()

    def _status_cb(self, msg: String) -> None:
        payload = self._parse_json(msg.data)
        if not payload:
            return
        agent_id = str(payload.get("agent_id", "unknown"))
        self.status_cache[agent_id] = payload
        self.status_file.write(json.dumps(payload, ensure_ascii=True) + "\n")
        self.status_file.flush()
        self._insert_status(payload)

    def _event_cb(self, msg: String) -> None:
        payload = self._parse_json(msg.data)
        if not payload:
            return
        sender = str(payload.get("sender", "unknown"))
        self.last_event_by_agent[sender] = time.time()
        self.event_file.write(json.dumps(payload, ensure_ascii=True) + "\n")
        self.event_file.flush()
        self._insert_event(payload)

    def _world_cb(self, msg: String) -> None:
        payload = self._parse_json(msg.data)
        if payload:
            self.latest_world_event = payload

    def _reward_cb(self, msg: String) -> None:
        payload = self._parse_json(msg.data)
        if payload and float(payload.get("stuck", 0.0)) > 0.0:
            self.deadlock_risk += 1

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

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
            if state == "waiting":
                evt_ts = self.last_event_by_agent.get(agent_id, now)
                if now - evt_ts > self.waiting_timeout_s:
                    anomalies.append((agent_id, "dependency_waiting_too_long", f"waiting={now - evt_ts:.2f}s"))
            if state == "blocked" or blocked_reason:
                anomalies.append((agent_id, "blocked_or_stuck", blocked_reason or f"sim_time={sim_time:.2f}"))

        self.starvation_risk = sum(1 for _, st in self.status_cache.items() if str(st.get("state", "")) == "waiting")
        for agent_id, typ, detail in anomalies:
            self._insert_anomaly(typ, agent_id, detail)

        summary = {
            "wall_time": now,
            "agent_count": len(self.status_cache),
            "deadlock_risk_count": self.deadlock_risk,
            "starvation_risk_count": self.starvation_risk,
            "latest_world_event": self.latest_world_event,
            "robot_pose_xy": {"x": self.latest_odom_xy[0], "y": self.latest_odom_xy[1]},
            "anomalies": [{"agent_id": a, "type": t, "detail": d} for a, t, d in anomalies],
        }
        msg = String()
        msg.data = json.dumps(summary, ensure_ascii=True)
        self.monitor_pub.publish(msg)
        self.summary_file.write(msg.data + "\n")
        self.summary_file.flush()
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
        status.values = [
            KeyValue(key="agent_count", value=str(summary.get("agent_count", 0))),
            KeyValue(key="deadlock_risk_count", value=str(summary.get("deadlock_risk_count", 0))),
            KeyValue(key="starvation_risk_count", value=str(summary.get("starvation_risk_count", 0))),
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
                "db_path": self.db_path,
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
            # Avoid crashing the monitor when an old process still holds DB locks.
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

    @staticmethod
    def _parse_json(raw: str) -> Dict[str, object]:
        try:
            out = json.loads(raw)
            if isinstance(out, dict):
                return out
            return {}
        except json.JSONDecodeError:
            return {}

    def destroy_node(self) -> bool:
        try:
            self.status_file.close()
            self.event_file.close()
            self.summary_file.close()
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
