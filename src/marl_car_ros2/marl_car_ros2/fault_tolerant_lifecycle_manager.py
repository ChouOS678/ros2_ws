#!/usr/bin/env python3
"""
Fault-tolerant Nav2 lifecycle manager.

This replaces the default nav2_lifecycle_manager with a parallel,
fault-tolerant approach:

- Nodes are split into CRITICAL (must succeed) and OPTIONAL (best-effort).
- Transitions happen in parallel with per-node timeouts.
- Failed optional nodes are skipped; failed critical nodes retry then error.
- This avoids the all-or-nothing failure mode of the default manager.

Usage in launch file (replaces nav2_lifecycle_manager):
    Node(
        package="marl_car_ros2",
        executable="fault_tolerant_lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[{
            "critical_nodes": ["controller_server", "planner_server",
                               "behavior_server", "bt_navigator"],
            "optional_nodes": ["smoother_server", "velocity_smoother",
                               "collision_monitor", "waypoint_follower",
                               "docking_server"],
            "autostart": True,
            "service_timeout_s": 8.0,
            "retry_count": 2,
        }],
    )
"""

from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# Lifecycle state constants
STATE_UNCONFIGURED = 1
STATE_INACTIVE = 2
STATE_ACTIVE = 3

# Transition constants
TRANSITION_CONFIGURE = 1  # unconfigured → inactive
TRANSITION_ACTIVATE = 3   # inactive → active


class FaultTolerantLifecycleManager(Node):
    """Parallel, fault-tolerant lifecycle manager for Nav2 nodes."""

    def __init__(self) -> None:
        super().__init__("fault_tolerant_lifecycle_manager")

        self.declare_parameter("critical_nodes", [""])
        self.declare_parameter("optional_nodes", [""])
        self.declare_parameter("autostart", True)
        self.declare_parameter("service_timeout_s", 8.0)
        self.declare_parameter("retry_count", 2)
        self.declare_parameter("startup_delay_s", 2.0)
        self.declare_parameter("node_ready_timeout_s", 30.0)

        self._critical_nodes: List[str] = [
            n for n in self.get_parameter("critical_nodes").value if n
        ]
        self._optional_nodes: List[str] = [
            n for n in self.get_parameter("optional_nodes").value if n
        ]
        self._autostart = bool(self.get_parameter("autostart").value)
        self._service_timeout = float(self.get_parameter("service_timeout_s").value)
        self._retry_count = int(self.get_parameter("retry_count").value)
        self._startup_delay = float(self.get_parameter("startup_delay_s").value)
        self._node_ready_timeout = float(self.get_parameter("node_ready_timeout_s").value)

        self._node_status: Dict[str, str] = {}
        self._lock = threading.Lock()
        self._done = threading.Event()
        self._executor: MultiThreadedExecutor | None = None

        if self._autostart:
            self._timer = self.create_timer(0.5, self._try_start)
        else:
            self.get_logger().info("autostart=False — waiting for manual activation")

    def set_executor(self, executor: MultiThreadedExecutor) -> None:
        """Store the executor so we can spin from within callbacks."""
        self._executor = executor

    def _try_start(self) -> None:
        """Single-shot delayed start to let nodes spawn."""
        self.destroy_timer(self._timer)
        # Run activation in a *separate* Python thread so the
        # MultiThreadedExecutor's 4 threads remain free to dispatch
        # service responses while we wait on futures.
        threading.Thread(target=self._delayed_activate, daemon=True).start()

    def _delayed_activate(self) -> None:
        time.sleep(self._startup_delay)
        self.get_logger().info(
            f"Starting lifecycle activation: "
            f"{len(self._critical_nodes)} critical + "
            f"{len(self._optional_nodes)} optional nodes"
        )
        self._activate_all_nodes()

    def _activate_all_nodes(self) -> None:
        """Activate all nodes in parallel, critical first, then optional."""
        all_nodes = self._critical_nodes + self._optional_nodes
        total = len(all_nodes)
        if total == 0:
            self.get_logger().warn("No nodes configured for lifecycle management")
            return

        # Phase 1: wait for all critical node services to appear
        self.get_logger().info("Waiting for critical node lifecycle services...")
        deadline = time.time() + self._node_ready_timeout
        ready_nodes: List[str] = []
        missing: List[str] = []

        for name in self._critical_nodes:
            if self._wait_for_service(name, deadline):
                ready_nodes.append(name)
            else:
                missing.append(name)
                with self._lock:
                    self._node_status[name] = "service_unavailable"

        if missing:
            self.get_logger().error(
                f"CRITICAL nodes with unavailable lifecycle services: {missing}. "
                f"These nodes may not exist or may have crashed."
            )
        else:
            self.get_logger().info(
                f"All {len(ready_nodes)} critical node services are available"
            )

        # Phase 2: wait for optional node services (best-effort)
        opt_ready: List[str] = []
        for name in self._optional_nodes:
            if self._wait_for_service(name, deadline):
                opt_ready.append(name)
            else:
                with self._lock:
                    self._node_status[name] = "skipped_unavailable"
                self.get_logger().warn(
                    f"Optional node '{name}' lifecycle service unavailable — skipping"
                )

        all_ready = ready_nodes + opt_ready
        if not all_ready:
            self.get_logger().error("No nodes ready for activation — aborting")
            self._print_summary()
            return

        # Phase 3: pre-create ALL service clients in the main thread.
        #   create_client MUST be called from a thread known to the
        #   executor so that response subscriptions are dispatched.
        self.get_logger().info(
            f"Pre-creating lifecycle clients for {len(all_ready)} nodes..."
        )
        get_state_clients: Dict[str, Client] = {}
        change_state_clients: Dict[str, Client] = {}
        for name in all_ready:
            get_state_clients[name] = self.create_client(
                GetState, f"/{name}/get_state"
            )
            change_state_clients[name] = self.create_client(
                ChangeState, f"/{name}/change_state"
            )
        self.get_logger().info("All lifecycle clients created — starting parallel activation")

        # Phase 4: parallel CONFIGURE → ACTIVATE for all ready nodes.
        #   All async calls are issued from this timer-callback thread
        #   (which is an executor thread).  The other executor threads
        #   dispatch responses while we round-robin-wait on futures.
        self.get_logger().info(
            f"Starting parallel activation of {len(all_ready)} nodes..."
        )

        # Step 4a — query current state for every node in parallel
        state_futures: Dict[str, object] = {}
        for name in all_ready:
            req = GetState.Request()
            state_futures[name] = get_state_clients[name].call_async(req)

        # Round-robin wait for all state queries
        state_deadline = time.time() + min(self._service_timeout, 10.0)
        pending = set(state_futures)
        ex = self._executor
        while pending and time.time() < state_deadline and ex is not None:
            for name in list(pending):
                done = state_futures[name].done()
                if not done:
                    try:
                        ex.spin_until_future_complete(
                            state_futures[name], timeout_sec=0.1
                        )
                    except Exception:
                        pass
                if state_futures[name].done():
                    pending.discard(name)

        # Step 4b — issue configure / activate as needed
        change_futures: Dict[str, Tuple[str, object]] = {}
        for name in all_ready:
            current = STATE_UNCONFIGURED  # fallback
            if state_futures[name].done():
                try:
                    result = state_futures[name].result()
                    if result is not None:
                        current = int(result.current_state.id)
                except Exception:
                    pass

            if current == STATE_UNCONFIGURED:
                req = ChangeState.Request()
                req.transition.id = TRANSITION_CONFIGURE
                req.transition.label = "configure"
                change_futures[name] = ("configure", change_state_clients[name].call_async(req))
            elif current == STATE_INACTIVE:
                req = ChangeState.Request()
                req.transition.id = TRANSITION_ACTIVATE
                req.transition.label = "activate"
                change_futures[name] = ("activate", change_state_clients[name].call_async(req))
            elif current == STATE_ACTIVE:
                self.get_logger().info(f"  ✓ '{name}' already ACTIVE")
                with self._lock:
                    self._node_status[name] = "active"
            else:
                self.get_logger().warn(f"  '{name}' in unexpected state {current}")
                with self._lock:
                    self._node_status[name] = f"unexpected_state:{current}"

        # Step 4c — round-robin wait for all change_state calls.
        #   If a "configure" succeeds, immediately fire "activate".
        change_deadline = time.time() + self._service_timeout + 5.0
        change_pending = set(change_futures)
        while change_pending and time.time() < change_deadline and ex is not None:
            for name in list(change_pending):
                kind, future = change_futures[name]
                if not future.done():
                    try:
                        ex.spin_until_future_complete(future, timeout_sec=0.1)
                    except Exception:
                        pass
                if not future.done():
                    continue  # still not ready

                # Future resolved — process result
                try:
                    result2 = future.result()
                except Exception:
                    result2 = None

                if result2 is not None and result2.success:
                    if kind == "configure":
                        # Fire activate immediately
                        req2 = ChangeState.Request()
                        req2.transition.id = TRANSITION_ACTIVATE
                        req2.transition.label = "activate"
                        change_futures[name] = (
                            "activate",
                            change_state_clients[name].call_async(req2),
                        )
                        continue  # stay in pending
                    else:
                        self.get_logger().info(f"  ✓ '{name}' → ACTIVE")
                        with self._lock:
                            self._node_status[name] = "active"
                else:
                    self.get_logger().error(
                        f"  ✗ '{name}' {kind} rejected"
                    )
                    with self._lock:
                        self._node_status[name] = f"{kind}_rejected"
                change_pending.discard(name)

        # Handle any nodes that timed out
        for name in change_pending:
            kind, _ = change_futures[name]
            self.get_logger().error(f"  ✗ '{name}' {kind} timed out")
            with self._lock:
                self._node_status[name] = f"{kind}_timeout"

        # Phase 5: report summary
        self._print_summary()

    def _activate_single_node(
        self,
        node_name: str,
        get_state_client: Client,
        change_state_client: Client,
    ) -> Tuple[bool, str]:
        """Transition a node to ACTIVE, respecting its current state.

        Nav2 nodes with autostart=True may already be INACTIVE by the
        time we reach them, so we query the current state first instead
        of blindly calling CONFIGURE.

        Clients are pre-created in the main thread and passed in to
        avoid cross-thread rclpy issues.
        """
        is_critical = node_name in self._critical_nodes
        max_attempts = self._retry_count + 1 if is_critical else 1

        for attempt in range(max_attempts):
            if attempt > 0:
                self.get_logger().info(
                    f"Retrying '{node_name}' (attempt {attempt + 1}/{max_attempts})..."
                )
                time.sleep(1.0)

            # Step 0: query current state
            current = self._get_current_state(node_name, get_state_client)
            if current is None:
                if attempt < max_attempts - 1:
                    continue
                return False, "state_query_failed"

            # Step 1: UNCONFIGURED → INACTIVE (only if needed)
            if current == STATE_UNCONFIGURED:
                ok, msg = self._call_change_state(
                    node_name, TRANSITION_CONFIGURE, change_state_client
                )
                if not ok:
                    if attempt < max_attempts - 1:
                        continue
                    return False, f"configure_failed:{msg}"
                # Re-check state after configure
                current = self._get_current_state(node_name, get_state_client) or current

            # Step 2: INACTIVE → ACTIVE
            if current == STATE_INACTIVE:
                ok, msg = self._call_change_state(
                    node_name, TRANSITION_ACTIVATE, change_state_client
                )
                if not ok:
                    if attempt < max_attempts - 1:
                        continue
                    return False, f"activate_failed:{msg}"

            elif current == STATE_ACTIVE:
                self.get_logger().info(f"  ✓ '{node_name}' already ACTIVE")
                return True, "already_active"

            self.get_logger().info(f"  ✓ '{node_name}' → ACTIVE")
            return True, "active"

        return False, "max_retries_exceeded"

    def _get_current_state(
        self, node_name: str, client: Client | None = None
    ) -> Optional[int]:
        """Query /<node_name>/get_state and return the state id, or None.

        If *client* is provided (pre-created in main thread), use it.
        Otherwise create one inline (for use from the main thread only).
        """
        if client is None:
            client = self.create_client(GetState, f"/{node_name}/get_state")
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        req = GetState.Request()
        future = client.call_async(req)
        try:
            result = future.result(timeout=2.0)
            if result is not None:
                return int(result.current_state.id)
        except Exception:
            pass
        return None

    def _call_change_state(
        self,
        node_name: str,
        transition_id: int,
        client: Client | None = None,
    ) -> Tuple[bool, str]:
        """Call /<node_name>/change_state service.

        Uses call_async + future.result().  The MultiThreadedExecutor spins
        in background threads, so service responses are processed while the
        worker thread simply waits on the future.

        If *client* is provided (pre-created in main thread), use it.
        Otherwise create one inline (for use from the main thread only).
        """
        if client is None:
            client = self.create_client(ChangeState, f"/{node_name}/change_state")

        if not client.wait_for_service(timeout_sec=self._service_timeout):
            return False, "service_timeout"

        req = ChangeState.Request()
        req.transition.id = transition_id
        labels = {1: "configure", 3: "activate"}
        req.transition.label = labels.get(transition_id, "unknown")

        future = client.call_async(req)
        try:
            result = future.result(timeout=self._service_timeout)
        except Exception:
            return False, "call_timeout"

        if result is None:
            return False, "null_result"

        if result.success:
            return True, ""
        else:
            return False, "transition_rejected"

    def _wait_for_service(self, node_name: str, deadline: float) -> bool:
        """Wait for a node's change_state service to become available."""
        srv_name = f"/{node_name}/change_state"
        client = self.create_client(ChangeState, srv_name)
        while time.time() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"  Service ready: {srv_name}")
                return True
            self.get_logger().debug(f"  Waiting for {srv_name}...")
        return False

    def _print_summary(self) -> None:
        """Log a summary of all node states."""
        active = [n for n, s in self._node_status.items() if s == "active"]
        failed = [n for n, s in self._node_status.items() if s != "active"]

        self.get_logger().info("=" * 60)
        self.get_logger().info("LIFECYCLE ACTIVATION SUMMARY")
        self.get_logger().info(f"  Active  ({len(active)}): {active}")
        if failed:
            self.get_logger().warn(f"  Failed  ({len(failed)}):")
            for n in failed:
                self.get_logger().warn(f"    {n}: {self._node_status.get(n, 'unknown')}")

        critical_failed = [n for n in self._critical_nodes if self._node_status.get(n) != "active"]
        if critical_failed:
            self.get_logger().error(
                f"CRITICAL nodes failed to activate: {critical_failed}. "
                f"Navigation may not function correctly."
            )
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = FaultTolerantLifecycleManager()
    executor = MultiThreadedExecutor(num_threads=4)
    node.set_executor(executor)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
