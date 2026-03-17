#!/usr/bin/env python3
"""
auto_eval_pipeline.py

Robust CI / evaluation loop for ROS 2 Jazzy + Gazebo Sim multi-agent stacks.

Lifecycle per epoch:
1) Teardown (aggressive cleanup)
2) Setup (launch world/base stack)
3) Execute (ensure agent logic and world mutator are running)
4) Evaluate (compute KPIs from monitor_logger SQLite)
5) Teardown (again, always)

Designed to be idempotent and resilient against hangs/crashes.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import signal
import sqlite3
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


# =========================
# Configuration Data Models
# =========================


@dataclasses.dataclass
class LaunchConfig:
    ros_setup: str = "/opt/ros/jazzy/setup.bash"
    ws_setup: str = "/home/grok/ros2_ws/install/setup.bash"

    # Setup stage (Gazebo + bridge + monitor + optional game, depending on launch args)
    setup_cmd: str = "ros2 launch marl_car_ros2 marl_stack_minimal.launch.py start_game:=false"

    # Execute stage: explicitly run/ensure these nodes for this epoch
    game_cmd: str = "ros2 run marl_car_ros2 multi_agent_game"
    mutator_cmd: str = "ros2 run marl_car_ros2 world_model_mutator"

    # Health / readiness gates
    ready_topics: Tuple[str, ...] = ("/clock", "/odom")

    # Paths
    monitor_db_path: str = "/tmp/marl_logs/timeline.db"
    output_dir: str = "/tmp/auto_eval_pipeline"


@dataclasses.dataclass
class EpochResult:
    epoch: int
    status: str  # success | timeout | crash | setup_failed
    reason: str
    start_wall: float
    end_wall: float
    duration_s: float
    kpi: Dict[str, float]


# ==================
# Shell / Proc Utils
# ==================


def _now() -> float:
    return time.time()


def _ts() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def log(msg: str) -> None:
    print(f"[{_ts()}] {msg}", flush=True)


def ros_bash_prefix(cfg: LaunchConfig) -> str:
    return f"source {cfg.ros_setup} && source {cfg.ws_setup}"


def run_shell(
    cmd: str,
    *,
    check: bool = False,
    timeout: Optional[float] = None,
    capture: bool = True,
) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", "-lc", cmd],
        check=check,
        timeout=timeout,
        text=True,
        capture_output=capture,
    )


def popen_shell(cmd: str, log_file: Path) -> subprocess.Popen:
    log_file.parent.mkdir(parents=True, exist_ok=True)
    fh = open(log_file, "a", encoding="utf-8")
    # Keep file handle attached to child stdio; parent process keeps object alive via proc._log_fh.
    proc = subprocess.Popen(
        ["bash", "-lc", cmd],
        stdout=fh,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )
    proc._log_fh = fh  # type: ignore[attr-defined]
    return proc


def terminate_proc(proc: subprocess.Popen, grace_s: float = 6.0) -> None:
    """Graceful terminate first, then SIGKILL process group as fallback."""
    if proc.poll() is not None:
        try:
            proc._log_fh.close()  # type: ignore[attr-defined]
        except Exception:
            pass
        return

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        pass

    deadline = _now() + grace_s
    while _now() < deadline:
        if proc.poll() is not None:
            break
        time.sleep(0.2)

    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass

    try:
        proc.wait(timeout=2)
    except Exception:
        pass

    try:
        proc._log_fh.close()  # type: ignore[attr-defined]
    except Exception:
        pass


# =========================
# Hard Teardown (Idempotent)
# =========================


def teardown(*, clear_all_shm: bool = False) -> None:
    """
    Aggressive cleanup protocol required for Gazebo Sim retention issues.

    Required actions:
      - pkill -9 -f gazebo
      - pkill -9 -f gz
      - pkill -9 -f ruby
      - ros2 daemon stop
      - clear shared memory
    """
    log("[teardown] starting aggressive cleanup")

    cmds = [
        "pkill -9 -f gazebo || true",
        "pkill -9 -f 'gz ' || true",
        "pkill -9 -f ruby || true",
        # Also clear the concrete stack processes to enforce idempotence.
        "pkill -9 -f 'ros2 launch marl_car_ros2 marl_stack_minimal.launch.py' || true",
        "pkill -9 -f '/marl_car_ros2/multi_agent_game' || true",
        "pkill -9 -f '/marl_car_ros2/world_model_mutator' || true",
        "pkill -9 -f '/marl_car_ros2/monitor_logger' || true",
        "pkill -9 -f 'ros_gz_bridge/parameter_bridge' || true",
        "ros2 daemon stop || true",
    ]

    for c in cmds:
        run_shell(c, check=False, capture=True)

    # Shared-memory cleanup:
    # - default: remove Gazebo/FastDDS artifacts only (safer)
    # - optional strict mode: rm -rf /dev/shm/*
    if clear_all_shm:
        run_shell("rm -rf /dev/shm/* || true", check=False, capture=True)
    else:
        run_shell("rm -rf /dev/shm/gz-* /dev/shm/ign-* /dev/shm/fastdds* /dev/shm/fastrtps* || true", check=False)

    # Restart daemon lazily for later CLI calls.
    run_shell("ros2 daemon start || true", check=False, capture=True)
    time.sleep(0.5)
    log("[teardown] done")


# ==========================
# ROS Readiness / Node Checks
# ==========================


def wait_for_topic_active(cfg: LaunchConfig, topic: str, timeout_s: float) -> bool:
    """
    Active-publishing check (not just topic existence).

    Strategy:
      - repeatedly call `ros2 topic echo --once <topic>`
      - success only when a message is actually received
    """
    deadline = _now() + timeout_s
    while _now() < deadline:
        cmd = (
            f"{ros_bash_prefix(cfg)} && "
            f"timeout 2s ros2 topic echo --once {topic} >/dev/null 2>&1"
        )
        cp = run_shell(cmd, check=False, capture=True)
        if cp.returncode == 0:
            return True
        time.sleep(0.4)
    return False


def wait_for_readiness(cfg: LaunchConfig, timeout_s: float) -> bool:
    per_topic = max(3.0, timeout_s / max(1, len(cfg.ready_topics)))
    for t in cfg.ready_topics:
        ok = wait_for_topic_active(cfg, t, per_topic)
        if not ok:
            log(f"[readiness] topic not active: {t}")
            return False
        log(f"[readiness] topic active: {t}")
    return True


def list_nodes(cfg: LaunchConfig) -> List[str]:
    cp = run_shell(f"{ros_bash_prefix(cfg)} && ros2 node list", check=False, capture=True)
    if cp.returncode != 0:
        return []
    return [ln.strip() for ln in cp.stdout.splitlines() if ln.strip()]


def node_exists(cfg: LaunchConfig, node_name_suffix: str) -> bool:
    nodes = list_nodes(cfg)
    return any(n.endswith(node_name_suffix) for n in nodes)


# ==============
# KPI Computation
# ==============


def _safe_json(raw: str) -> Dict:
    try:
        val = json.loads(raw)
        return val if isinstance(val, dict) else {}
    except Exception:
        return {}


def calculate_kpi(db_path: str, wall_start: float, wall_end: float, step_hz: float = 10.0) -> Dict[str, float]:
    """
    Compute KPI from monitor_logger SQLite timeline.

    KPI definitions:
      - success_rate: task_finished(goal_reached)/episodes_done
      - collision_rate: task_failed(collision)/episodes_done
      - avg_time_to_destination_s: mean(step/step_hz) on successful episodes
      - rtf: delta(sim_time)/delta(wall_time) from agent_status
    """
    if not os.path.exists(db_path):
        return {
            "success_rate": 0.0,
            "collision_rate": 0.0,
            "avg_time_to_destination_s": 0.0,
            "rtf": 0.0,
            "episodes_done": 0.0,
        }

    conn = sqlite3.connect(db_path)
    try:
        cur = conn.cursor()

        # Events in epoch window
        cur.execute(
            """
            SELECT wall_time, event_type, result, payload_json
            FROM agent_event
            WHERE wall_time >= ? AND wall_time <= ?
            ORDER BY wall_time ASC
            """,
            (wall_start, wall_end),
        )
        rows = cur.fetchall()

        finished = 0
        success = 0
        collision = 0
        ttd_list: List[float] = []

        for _, event_type, result, payload_json in rows:
            et = str(event_type or "")
            rs = str(result or "")
            payload = _safe_json(payload_json or "{}")
            details = payload.get("details", {}) if isinstance(payload.get("details", {}), dict) else {}

            if et in ("task_finished", "task_failed"):
                finished += 1

            if et == "task_finished" and rs == "goal_reached":
                success += 1
                step = details.get("step", None)
                if isinstance(step, (int, float)):
                    ttd_list.append(float(step) / max(step_hz, 1e-6))

            if et == "task_failed" and rs == "collision":
                collision += 1

        success_rate = (success / finished) if finished > 0 else 0.0
        collision_rate = (collision / finished) if finished > 0 else 0.0
        avg_ttd = (sum(ttd_list) / len(ttd_list)) if ttd_list else 0.0

        # RTF via agent_status progression
        cur.execute(
            """
            SELECT wall_time, sim_time
            FROM agent_status
            WHERE wall_time >= ? AND wall_time <= ?
            ORDER BY wall_time ASC
            """,
            (wall_start, wall_end),
        )
        st = cur.fetchall()
        if len(st) >= 2:
            wall_dt = float(st[-1][0]) - float(st[0][0])
            sim_dt = float(st[-1][1]) - float(st[0][1])
            rtf = (sim_dt / wall_dt) if wall_dt > 1e-6 else 0.0
        else:
            rtf = 0.0

        return {
            "success_rate": float(success_rate),
            "collision_rate": float(collision_rate),
            "avg_time_to_destination_s": float(avg_ttd),
            "rtf": float(rtf),
            "episodes_done": float(finished),
        }
    finally:
        conn.close()


# ==================
# Epoch Runner Logic
# ==================


def run_epoch(
    epoch_idx: int,
    cfg: LaunchConfig,
    *,
    epoch_timeout_s: float,
    readiness_timeout_s: float,
    clear_all_shm: bool,
) -> EpochResult:
    epoch_start = _now()
    log(f"[epoch {epoch_idx}] start")

    setup_proc: Optional[subprocess.Popen] = None
    game_proc: Optional[subprocess.Popen] = None
    mutator_proc: Optional[subprocess.Popen] = None

    status = "success"
    reason = "ok"

    out_dir = Path(cfg.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        # Always start from hard-clean state
        teardown(clear_all_shm=clear_all_shm)

        # Setup stage
        setup_cmd = f"{ros_bash_prefix(cfg)} && {cfg.setup_cmd}"
        setup_log = out_dir / f"epoch_{epoch_idx:03d}_setup.log"
        setup_proc = popen_shell(setup_cmd, setup_log)
        log(f"[epoch {epoch_idx}] setup launched pid={setup_proc.pid}")

        if not wait_for_readiness(cfg, readiness_timeout_s):
            status = "setup_failed"
            reason = "readiness_timeout"
            raise RuntimeError("setup readiness timeout")

        # Execute stage
        # 1) Ensure mutator active (start if not already present)
        if not node_exists(cfg, "/world_model_mutator"):
            mutator_cmd = f"{ros_bash_prefix(cfg)} && {cfg.mutator_cmd}"
            mutator_log = out_dir / f"epoch_{epoch_idx:03d}_mutator.log"
            mutator_proc = popen_shell(mutator_cmd, mutator_log)
            log(f"[epoch {epoch_idx}] mutator launched pid={mutator_proc.pid}")
        else:
            log(f"[epoch {epoch_idx}] mutator already running")

        # 2) Ensure game active (start if not already present)
        if not node_exists(cfg, "/multi_agent_game"):
            game_cmd = f"{ros_bash_prefix(cfg)} && {cfg.game_cmd}"
            game_log = out_dir / f"epoch_{epoch_idx:03d}_game.log"
            game_proc = popen_shell(game_cmd, game_log)
            log(f"[epoch {epoch_idx}] game launched pid={game_proc.pid}")
        else:
            log(f"[epoch {epoch_idx}] game already running")

        # Epoch execution window
        loop_deadline = _now() + epoch_timeout_s
        while _now() < loop_deadline:
            # Crash detection: if setup proc exits unexpectedly, epoch fails.
            if setup_proc.poll() is not None:
                status = "crash"
                reason = f"setup_proc_exited_rc={setup_proc.returncode}"
                raise RuntimeError(reason)

            # Keep checking active clock to detect dead simulation.
            if not wait_for_topic_active(cfg, "/clock", timeout_s=2.0):
                status = "crash"
                reason = "clock_stalled"
                raise RuntimeError(reason)

            time.sleep(1.0)

        # Timeout is treated as controlled end-of-evaluation window.
        status = "success"
        reason = "epoch_window_complete"

    except Exception as exc:
        if status == "success":
            # Unexpected exception without pre-tagged status
            status = "crash"
            reason = str(exc)
        log(f"[epoch {epoch_idx}] exception: {exc}")

    finally:
        # Attempt graceful stop for tracked processes before hard teardown.
        for p in (game_proc, mutator_proc, setup_proc):
            if p is not None:
                terminate_proc(p)

        # Always hard-clean between epochs.
        teardown(clear_all_shm=clear_all_shm)

    epoch_end = _now()

    kpi = calculate_kpi(
        db_path=cfg.monitor_db_path,
        wall_start=epoch_start,
        wall_end=epoch_end,
    )

    return EpochResult(
        epoch=epoch_idx,
        status=status,
        reason=reason,
        start_wall=epoch_start,
        end_wall=epoch_end,
        duration_s=epoch_end - epoch_start,
        kpi=kpi,
    )


# ============
# Main Entrypoint
# ============


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Automated CI/Eval loop for ROS2 multi-agent stack")
    p.add_argument("--epochs", type=int, default=10, help="Number of epochs")
    p.add_argument("--epoch-timeout", type=float, default=60.0, help="Execution window per epoch (s)")
    p.add_argument("--readiness-timeout", type=float, default=45.0, help="Setup readiness timeout (s)")
    p.add_argument("--ros-setup", type=str, default="/opt/ros/jazzy/setup.bash")
    p.add_argument("--ws-setup", type=str, default="/home/grok/ros2_ws/install/setup.bash")
    p.add_argument(
        "--setup-cmd",
        type=str,
        default="ros2 launch marl_car_ros2 marl_stack_minimal.launch.py start_game:=false",
        help="Setup stage command",
    )
    p.add_argument("--game-cmd", type=str, default="ros2 run marl_car_ros2 multi_agent_game")
    p.add_argument("--mutator-cmd", type=str, default="ros2 run marl_car_ros2 world_model_mutator")
    p.add_argument("--db-path", type=str, default="/tmp/marl_logs/timeline.db")
    p.add_argument("--output-dir", type=str, default="/tmp/auto_eval_pipeline")
    p.add_argument(
        "--clear-all-shm",
        action="store_true",
        help="Use strict shm wipe: rm -rf /dev/shm/* (dangerous, but requested for hard reset)",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()

    cfg = LaunchConfig(
        ros_setup=args.ros_setup,
        ws_setup=args.ws_setup,
        setup_cmd=args.setup_cmd,
        game_cmd=args.game_cmd,
        mutator_cmd=args.mutator_cmd,
        monitor_db_path=args.db_path,
        output_dir=args.output_dir,
    )

    out_dir = Path(cfg.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    results_jsonl = out_dir / "epoch_results.jsonl"

    log("pipeline start")
    log(f"output_dir={cfg.output_dir}")
    log(f"db_path={cfg.monitor_db_path}")

    # Pre-run global teardown for idempotence.
    teardown(clear_all_shm=args.clear_all_shm)

    all_results: List[EpochResult] = []

    for i in range(1, args.epochs + 1):
        res = run_epoch(
            i,
            cfg,
            epoch_timeout_s=args.epoch_timeout,
            readiness_timeout_s=args.readiness_timeout,
            clear_all_shm=args.clear_all_shm,
        )
        all_results.append(res)

        with open(results_jsonl, "a", encoding="utf-8") as f:
            f.write(
                json.dumps(
                    {
                        "epoch": res.epoch,
                        "status": res.status,
                        "reason": res.reason,
                        "start_wall": res.start_wall,
                        "end_wall": res.end_wall,
                        "duration_s": res.duration_s,
                        "kpi": res.kpi,
                    },
                    ensure_ascii=True,
                )
                + "\n"
            )

        log(
            f"[epoch {res.epoch}] status={res.status} reason={res.reason} "
            f"kpi={json.dumps(res.kpi, ensure_ascii=True)}"
        )

    # Final summary
    succ = sum(1 for r in all_results if r.status == "success")
    crash = sum(1 for r in all_results if r.status in ("crash", "timeout", "setup_failed"))
    log(f"pipeline done: success_epochs={succ}, failed_epochs={crash}, total={len(all_results)}")

    # Post-run global teardown (strict lifecycle guarantee).
    teardown(clear_all_shm=args.clear_all_shm)

    return 0


if __name__ == "__main__":
    sys.exit(main())
