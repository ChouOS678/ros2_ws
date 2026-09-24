from __future__ import annotations

import json
import os
import sqlite3
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from .analytics import explain_case, overview
from .store import connect, import_telemetry, persist_report


APP_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = APP_DIR.parent
FRONTEND_DIR = APP_DIR / "frontend"
DATABASE = Path(os.environ.get("BENCHMARK_PORTAL_DB", APP_DIR / "data" / "benchmark.sqlite"))
TELEMETRY_DB = Path(os.environ.get("BENCHMARK_TELEMETRY_DB", "/tmp/marl_logs/timeline.db"))
REPORT_DIR = Path(os.environ.get("BENCHMARK_REPORT_DIR", REPO_DIR))
MAX_BODY = 25 * 1024 * 1024


def _seed() -> None:
    db = connect(DATABASE)
    patterns = ("acceptance_*.json", "last_run*report.json")
    for pattern in patterns:
        for path in REPORT_DIR.glob(pattern):
            try:
                with path.open("r", encoding="utf-8") as file:
                    persist_report(db, json.load(file), path.name)
            except (OSError, json.JSONDecodeError, ValueError) as error:
                print(f"Skipping report {path.name}: {error}")
    if TELEMETRY_DB.is_file():
        try:
            run_ids = {row[0] for row in db.execute("SELECT run_id FROM runs")}
            count = import_telemetry(db, TELEMETRY_DB, run_ids)
            print(f"Imported {count} telemetry samples from {TELEMETRY_DB}")
        except (OSError, ValueError, sqlite3.Error) as error:
            print(f"Telemetry import unavailable: {error}")
    print(f"Benchmark portal database: {DATABASE}")
    db.close()


class Handler(BaseHTTPRequestHandler):
    server_version = "BenchmarkPortal/1.0"

    def log_message(self, fmt: str, *args: object) -> None:
        print(f"{self.log_date_time_string()} {fmt % args}")

    def _json(self, payload: object, status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _body(self) -> bytes:
        size = int(self.headers.get("Content-Length", "0"))
        if size < 0 or size > MAX_BODY:
            raise ValueError("上传内容不能超过 25 MB")
        return self.rfile.read(size)

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/health":
            self._json({"ok": True, "database": str(DATABASE), "telemetry_source": str(TELEMETRY_DB)})
            return
        if path == "/api/overview":
            db = connect(DATABASE)
            query = parse_qs(urlparse(self.path).query)
            result = overview(db, query.get("campaign", [None])[0])
            labels = result["metric_labels"]
            for case in result["cases"]:
                case["explanations"] = explain_case(case, labels)
            db.close()
            self._json(result)
            return
        if path == "/api/runs":
            db = connect(DATABASE)
            self._json(overview(db)["runs"])
            db.close()
            return
        relative = "index.html" if path == "/" else path.lstrip("/")
        target = (FRONTEND_DIR / relative).resolve()
        if not target.is_relative_to(FRONTEND_DIR.resolve()) or not target.is_file():
            self._json({"error": "not found"}, 404)
            return
        content = target.read_bytes()
        content_type = "text/html; charset=utf-8" if target.suffix == ".html" else "text/css; charset=utf-8" if target.suffix == ".css" else "text/javascript; charset=utf-8" if target.suffix == ".js" else "application/octet-stream"
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            raw = self._body()
            if path == "/api/import":
                payload = json.loads(raw.decode("utf-8"))
                if not isinstance(payload, dict):
                    raise ValueError("报告必须是 JSON 对象")
                db = connect(DATABASE)
                run = persist_report(db, payload, self.headers.get("X-Filename", "upload.json"))
                sample_count = 0
                if TELEMETRY_DB.is_file():
                    sample_count = import_telemetry(db, TELEMETRY_DB, {run["run_id"]})
                db.close()
                self._json({"ok": True, "run_id": run["run_id"], "scenario": run["scenario"], "controller": run["controller"], "telemetry_samples": sample_count}, 201)
                return
            if path == "/api/sync-telemetry":
                db = connect(DATABASE)
                run_ids = {row[0] for row in db.execute("SELECT run_id FROM runs")}
                count = import_telemetry(db, TELEMETRY_DB, run_ids)
                db.close()
                self._json({"ok": True, "telemetry_samples": count})
                return
            self._json({"error": "not found"}, 404)
        except (UnicodeDecodeError, json.JSONDecodeError, ValueError, OSError, sqlite3.Error) as error:
            self._json({"error": str(error)}, 400)


def main() -> None:
    _seed()
    host = os.environ.get("BENCHMARK_PORTAL_HOST", "127.0.0.1")
    port = int(os.environ.get("BENCHMARK_PORTAL_PORT", "8765"))
    print(f"Benchmark portal ready at http://localhost:{port}")
    ThreadingHTTPServer((host, port), Handler).serve_forever()


if __name__ == "__main__":
    main()
