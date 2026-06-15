#!/usr/bin/env bash
set -euo pipefail

# WSL2 local one-click demo controller for ROS2 Jazzy + Gazebo Sim
# Usage:
#   ./wsl2_demo_ctl.sh up
#   ./wsl2_demo_ctl.sh down
#   ./wsl2_demo_ctl.sh status
#   ./wsl2_demo_ctl.sh test [SECONDS]
#
# Notes:
# - "up" does: hard cleanup -> build -> launch benchmark demo entry -> readiness checks
# - "down" does: stop tracked launch + aggressive process cleanup + shm cleanup

WS_ROOT="/home/grok/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$WS_ROOT/install/setup.bash"
PKG="marl_car_ros2"
LAUNCH_FILE="benchmark_demo.launch.py"
LAUNCH_ARGS=""

RUN_DIR="/tmp/wsl2_demo"
PID_FILE="$RUN_DIR/launch.pid"
LOG_FILE="$RUN_DIR/launch.log"

mkdir -p "$RUN_DIR"

ts() { date +"%Y-%m-%d %H:%M:%S"; }
log() { echo "[$(ts)] $*"; }

run_ros() {
  local cmd="$1"
  bash -lc "source '$ROS_SETUP' && source '$WS_SETUP' && $cmd"
}

hard_cleanup() {
  log "hard cleanup start"

  # Stop tracked launch first (if present)
  if [[ -f "$PID_FILE" ]]; then
    local pid
    pid="$(cat "$PID_FILE" || true)"
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      log "stopping tracked launch pid=$pid"
      kill "$pid" 2>/dev/null || true
      sleep 1
      kill -9 "$pid" 2>/dev/null || true
    fi
    rm -f "$PID_FILE"
  fi

  # Aggressive cleanup for Gazebo/ROS residue in WSL2
  pkill -9 -f "ros2 launch $PKG $LAUNCH_FILE" 2>/dev/null || true
  pkill -9 -f "/$PKG/benchmark_gui" 2>/dev/null || true
  pkill -9 -f "/$PKG/benchmark_visualizer" 2>/dev/null || true
  pkill -9 -f "/$PKG/monitor_logger" 2>/dev/null || true
  pkill -9 -f "/$PKG/supervisor_node" 2>/dev/null || true
  pkill -9 -f "ros_gz_bridge/parameter_bridge" 2>/dev/null || true

  pkill -9 -f gazebo 2>/dev/null || true
  pkill -9 -f "gz " 2>/dev/null || true
  pkill -9 -f ruby 2>/dev/null || true

  bash -lc "source '$ROS_SETUP' && ros2 daemon stop" >/dev/null 2>&1 || true
  rm -rf /dev/shm/gz-* /dev/shm/ign-* /dev/shm/fastdds* /dev/shm/fastrtps* 2>/dev/null || true
  bash -lc "source '$ROS_SETUP' && ros2 daemon start" >/dev/null 2>&1 || true

  log "hard cleanup done"
}

wait_topic_once() {
  local topic="$1"
  local timeout_s="$2"
  local started
  started="$(date +%s)"

  while true; do
    if run_ros "timeout 2s ros2 topic echo --once $topic >/dev/null 2>&1"; then
      log "topic active: $topic"
      return 0
    fi

    local now
    now="$(date +%s)"
    if (( now - started >= timeout_s )); then
      log "topic timeout: $topic"
      return 1
    fi
    sleep 1
  done
}

build_ws() {
  log "building package: $PKG"
  bash -lc "source '$ROS_SETUP' && cd '$WS_ROOT' && colcon build --packages-select '$PKG'"
}

up() {
  hard_cleanup
  build_ws

  log "launching demo: $PKG $LAUNCH_FILE $LAUNCH_ARGS"
  run_ros "cd '$WS_ROOT' && nohup ros2 launch '$PKG' '$LAUNCH_FILE' $LAUNCH_ARGS > '$LOG_FILE' 2>&1 & echo \$! > '$PID_FILE'"

  local pid
  pid="$(cat "$PID_FILE")"
  log "launch pid=$pid"

  wait_topic_once /clock 40
  wait_topic_once /odom 40

  log "demo is UP"
  log "log file: $LOG_FILE"
}

down() {
  hard_cleanup
}

status() {
  local alive="no"
  local pid=""
  if [[ -f "$PID_FILE" ]]; then
    pid="$(cat "$PID_FILE" || true)"
    if [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      alive="yes"
    fi
  fi

  log "tracked launch alive: $alive${pid:+ (pid=$pid)}"

  log "process snapshot:"
  ps -eo pid,ppid,stat,cmd \
    | rg -n "benchmark_demo.launch.py|benchmark_gui|benchmark_visualizer|monitor_logger|supervisor_node|parameter_bridge|gz sim server|gazebo" -S \
    || true

  log "ROS nodes snapshot:"
  run_ros "ros2 node list" || true
}

test_loop() {
  local duration_s="${1:-30}"
  up

  log "smoke test window: ${duration_s}s"
  local end_at=$(( $(date +%s) + duration_s ))

  while (( $(date +%s) < end_at )); do
    if ! run_ros "timeout 2s ros2 topic echo --once /clock >/dev/null 2>&1"; then
      log "[FAIL] /clock stalled"
      down
      return 1
    fi
    if ! run_ros "timeout 2s ros2 topic echo --once /odom >/dev/null 2>&1"; then
      log "[FAIL] /odom stalled"
      down
      return 1
    fi
    sleep 2
  done

  log "[PASS] smoke test passed"
  down
}

cmd="${1:-}"
case "$cmd" in
  up)
    up
    ;;
  down)
    down
    ;;
  status)
    status
    ;;
  test)
    test_loop "${2:-30}"
    ;;
  *)
    cat <<EOF
Usage:
  $0 up
  $0 down
  $0 status
  $0 test [SECONDS]
EOF
    exit 1
    ;;
esac
