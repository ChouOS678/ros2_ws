"""Shared process cleanup utilities for benchmark infrastructure.

Provides robust kill-all with:
- Expanded process target list (including Nav2 lifecycle / component nodes)
- Recursive child process tree killing (kill children before parent)
- D-state (uninterruptible sleep) detection and warning
- Post-kill verification
- Retry mechanism (up to 3 attempts)

Used by: benchmark_runner.py, batch_compare.py
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from typing import List, Set, Tuple

# ── Process target patterns ──────────────────────────────────────────────

# Patterns for pgrep/pkill -f (full command-line regex match)
_PATTERNS_FULL = [
    "ros2",
    "gz sim",
    "gz server",
    "gz sim server",
    "gzclient",
    "gzserver",
    "ruby",
    # ── Nav2 lifecycle / component nodes ──
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
    "lifecycle_manager",
    "component_container",
    "component_container_isolated",
    "smoother_server",
    "collision_monitor",
    # ── ROS 2 infrastructure ──
    "robot_state_publisher",
    "rviz2",
    "ros_gz_bridge",
    "parameter_bridge",
    # ── marl_car_ros2 internal nodes ──
    "monitor_logger",
    "supervisor_node",
    "benchmark_gui",
    "benchmark_visualizer",
    "scenario_mutator",
    "baseline_nav_node",
    "task_agent",
    "nav_goal_sender",
    "nav_tf_bridge",
    "scan_stamp_bridge",
    "risk_marker",
    "world_event_marker",
    "supervisor_status_marker",
    "fault_tolerant_lifecycle_manager",
    "dummy_joint_runner",
]

# Exact process names for pgrep/pkill (no -f, matches /proc/[pid]/comm)
_PATTERNS_EXACT = [
    "gzserver",
    "gzclient",
    "ruby",
]


# ── Internal helpers ─────────────────────────────────────────────────────


def _get_pids_matching(pattern: str, *, full_cmdline: bool = True) -> Set[int]:
    """Return PIDs whose command line (or comm) matches *pattern*."""
    cmd = ["pgrep"]
    if full_cmdline:
        cmd.append("-f")
    cmd.append(pattern)
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
        if result.returncode != 0:
            return set()
        return {int(pid) for pid in result.stdout.strip().split("\n") if pid.strip()}
    except Exception:
        return set()


def _get_children_recursive(pid: int) -> Set[int]:
    """Recursively collect all descendant PIDs of *pid* (children, grandchildren, …)."""
    all_children: Set[int] = set()
    try:
        result = subprocess.run(
            ["pgrep", "-P", str(pid)],
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        if result.returncode != 0:
            return all_children
        direct = {int(p) for p in result.stdout.strip().split("\n") if p.strip()}
        for child_pid in direct:
            all_children.add(child_pid)
            all_children.update(_get_children_recursive(child_pid))
    except Exception:
        pass
    return all_children


def _kill_pid_tree(pid: int, sig: int = 9) -> None:
    """Kill *pid* and all its descendant processes (bottom-up: children first).

    Uses os.kill so that permissions / already-exited processes are handled
    gracefully.
    """
    children = _get_children_recursive(pid)
    # Children first, in reverse order (leaves → root)
    for child_pid in sorted(children, reverse=True):
        try:
            os.kill(child_pid, sig)
        except (ProcessLookupError, PermissionError):
            pass
    # Then the parent
    try:
        os.kill(pid, sig)
    except (ProcessLookupError, PermissionError):
        pass


def _check_d_state(pids: Set[int]) -> List[Tuple[int, str]]:
    """Return (pid, comm) for every PID currently in D state (uninterruptible sleep).

    D-state processes are immune to SIGKILL — the kernel will not deliver the
    signal until the uninterruptible I/O operation completes.
    """
    d_state: List[Tuple[int, str]] = []
    for pid in pids:
        try:
            with open(f"/proc/{pid}/status", "r") as f:
                for line in f:
                    if line.startswith("State:"):
                        raw_state = line.split(":", 1)[1].strip()
                        if raw_state.startswith("D"):
                            # Read comm for a human-readable label
                            d_state.append((pid, f"pid={pid} state={raw_state}"))
                        break
                    if line.startswith("Name:"):
                        continue  # we read State: first; comm is secondary
        except (OSError, FileNotFoundError):
            pass
    return d_state


def _collect_all_target_pids() -> Set[int]:
    """Return the union of all PIDs matched by any target pattern."""
    pids: Set[int] = set()
    for pattern in _PATTERNS_FULL:
        pids.update(_get_pids_matching(pattern, full_cmdline=True))
    for pattern in _PATTERNS_EXACT:
        pids.update(_get_pids_matching(pattern, full_cmdline=False))
    return pids


# ── Public API ───────────────────────────────────────────────────────────


def kill_all_processes(verbose: bool = True) -> None:
    """Force-kill every benchmark-related process with verification and retry.

    Strategy (per attempt):
    1. Collect every PID matching the known target patterns.
    2. For each PID, recursively kill its entire process tree (children first).
    3. Wait 1 s for processes to actually die.
    4. Re-scan — if any survivors remain and retries are left, repeat.
    5. Detect D-state processes and emit explicit warnings (SIGKILL cannot
       kill them).

    Retries: up to 3 attempts.
    """
    max_retries = 3

    if verbose:
        print("[cleanup] Starting aggressive process cleanup…", file=sys.stderr)

    for attempt in range(1, max_retries + 1):
        # ── Phase 1: collect ──
        all_pids = _collect_all_target_pids()
        if not all_pids:
            if verbose:
                print(f"[cleanup] No target processes found (attempt {attempt}).", file=sys.stderr)
            break

        if verbose:
            print(f"[cleanup] Found {len(all_pids)} target PID(s): {sorted(all_pids)}", file=sys.stderr)

        # ── Phase 2: kill tree ──
        for pid in sorted(all_pids):
            _kill_pid_tree(pid, sig=9)

        # Allow processes to actually terminate
        time.sleep(1.0)

        # ── Phase 3: verify ──
        remaining = _collect_all_target_pids()
        if not remaining:
            if verbose:
                print(f"[cleanup] All processes killed (attempt {attempt}).", file=sys.stderr)
            return

        # ── Phase 4: D-state check ──
        d_state = _check_d_state(remaining)
        if d_state and verbose:
            for pid, label in d_state:
                print(
                    f"[cleanup] ⚠  PID {label} — D-state (uninterruptible sleep). "
                    f"SIGKILL cannot kill this process. It will survive until the "
                    f"underlying I/O completes, or until the WSL2 VM is restarted.",
                    file=sys.stderr,
                )

        if attempt < max_retries:
            if verbose:
                non_d = len(remaining) - len(d_state)
                print(
                    f"[cleanup] {len(remaining)} process(es) still alive "
                    f"(D-state={len(d_state)}, other={non_d}). "
                    f"Retrying ({attempt + 1}/{max_retries})…",
                    file=sys.stderr,
                )
        else:
            if verbose:
                print(
                    f"[cleanup] FINAL: {len(remaining)} process(es) could not be "
                    f"killed after {max_retries} attempts: {sorted(remaining)}",
                    file=sys.stderr,
                )
                if d_state:
                    print(
                        f"[cleanup] D-state processes above will survive SIGKILL. "
                        f"Restart WSL2 (`wsl --shutdown`) if they persist.",
                        file=sys.stderr,
                    )

    if verbose:
        print("[cleanup] Cleanup finished.", file=sys.stderr)
