# Multi-Agent Robot Architecture (ROS 2 Jazzy + Gazebo Sim)

## 1. Project Summary
This workspace hosts a **Multi-Agent Robot Architecture** prototype built on ROS 2 (Jazzy) and Gazebo Sim.

The goal is not only to drive a demo car, but to validate an AI-native robotics architecture where:
- ROS 2 nodes act as independent agents,
- agents coordinate under uncertainty,
- the system remains robust under perturbations (for example, sudden obstacle / "鬼探头" style events),
- the architecture can be continuously evaluated with reproducible local CI loops.

Current package: `marl_car_ros2`

---

## 2. What Was Done Today

### 2.1 Gazebo Backend Migration and Launch Stabilization
- Migrated launch flow away from Gazebo Classic assumptions (`gazebo`, `gazebo_ros`) to **Gazebo Sim + ros_gz** in Jazzy.
- Updated launch to use:
  - `ros_gz_sim` for simulation startup and spawning
  - `ros_gz_bridge` for ROS <-> Gazebo topic bridging
- Fixed startup failure where model spawn parameters used wrong numeric type (`x/y` int vs required double).

### 2.2 World and Model Compatibility Fixes
- Reworked world file to avoid dependency on missing `model://ground_plane` / `model://sun` in local environment.
- Added required Gazebo Sim world systems (`Physics`, `Sensors`, `UserCommands`, `SceneBroadcaster`) for stable runtime and sensor pipeline.
- Migrated robot model plugins from Gazebo Classic plugins to Gazebo Sim compatible setup.

### 2.3 Runtime Control/Loop Fixes
- Identified and mitigated control instability from duplicate / residual processes and bridge loop risks.
- Added launch-time control behavior switch:
  - `start_game:=false` for passive mode
  - `start_game:=true` for active autonomous game loop

### 2.4 Monitor Logger Robustness
- Hardened `monitor_logger_node.py` SQLite behavior:
  - enabled WAL and busy timeout,
  - added lock-tolerant write paths to reduce crash probability under process overlap.

### 2.5 Automated CI / Evaluation Pipeline Entrypoint
Added:
- `auto_eval_pipeline.py`

Capabilities:
- strict epoch lifecycle: teardown -> setup -> execute -> evaluate -> teardown,
- aggressive cleanup protocol (`pkill -9`, daemon reset, shared-memory cleanup),
- readiness gating based on active topic publication (`/clock`, `/odom`),
- timeout/crash handling with automatic recovery to next epoch,
- KPI extraction from monitor SQLite:
  - success rate,
  - collision rate,
  - average time to destination,
  - real-time factor (RTF).

### 2.6 One-Click WSL2 Local Automation Script
Added:
- `wsl2_demo_ctl.sh`

Commands:
- `up`: hard cleanup + build + launch demo + readiness checks
- `down`: full cleanup
- `status`: process/node snapshot
- `test [SECONDS]`: smoke run with auto cleanup

This script targets the practical WSL2 pain point of stale Gazebo/ROS processes.

---

## 3. High-Level Architecture (Current)

### Core Nodes
- `multi_agent_game`
  - orchestrates planner/chassis agent outputs
  - drives joint action loop
- `marl_env_wrapper`
  - converts ROS topics (`/odom`, `/scan`) into observations
  - computes rewards and termination
  - publishes `/cmd_vel`
- `world_model_mutator`
  - injects non-stationary world perturbation events
- `monitor_logger`
  - aggregates multi-agent status/events
  - writes JSONL + SQLite timeline DB

### Simulation/Bridge Layer
- Gazebo Sim (`ros_gz_sim`)
- Topic bridge (`ros_gz_bridge`)
  - `/clock`
  - command/odometry channels
  - laser scan channel

### Data/Artifacts
- Runtime logs: `/tmp/marl_logs`
- Timeline DB: `/tmp/marl_logs/timeline.db`
- Evaluation outputs: `/tmp/auto_eval_pipeline`

---

## 4. Quick Start

### 4.1 One-click local demo (recommended)
```bash
cd /home/grok/ros2_ws
./wsl2_demo_ctl.sh up
```

Check status:
```bash
./wsl2_demo_ctl.sh status
```

Stop and hard-clean:
```bash
./wsl2_demo_ctl.sh down
```

Smoke test:
```bash
./wsl2_demo_ctl.sh test 60
```

### 4.2 Automated evaluation loop
```bash
cd /home/grok/ros2_ws
python3 auto_eval_pipeline.py --epochs 10 --epoch-timeout 90 --readiness-timeout 60
```

---

## 5. Project Structure (Key Files)
- `src/marl_car_ros2/launch/marl_stack_minimal.launch.py`
- `src/marl_car_ros2/models/simple_marl_car/model.sdf`
- `src/marl_car_ros2/worlds/minimal.world`
- `src/marl_car_ros2/marl_car_ros2/multi_agent_game.py`
- `src/marl_car_ros2/marl_car_ros2/marl_env_wrapper.py`
- `src/marl_car_ros2/marl_car_ros2/world_model_mutator.py`
- `src/marl_car_ros2/marl_car_ros2/monitor_logger_node.py`
- `auto_eval_pipeline.py`
- `wsl2_demo_ctl.sh`

---

## 6. Current Limitations / Next Milestones
- Multi-agent logic is still centralized relative to a full distributed agent society.
- More explicit inter-agent protocol (task negotiation, arbitration, retry semantics) is needed.
- Safety supervisor node and stronger fault-containment boundaries should be added.
- World mutator should be expanded for richer corner cases and repeatable scenario seeds.
- KPI suite should include broader mission-level metrics and regression dashboards.

---

## 7. Development Notes
- Use `Ctrl+C` to stop launches; avoid `Ctrl+Z` to prevent suspended process residue.
- If anything behaves strangely, run `./wsl2_demo_ctl.sh down` before relaunch.

