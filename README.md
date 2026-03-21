# ROS 2 + Gazebo + RViz + Nav2 Research Platform

This repository is a navigation research platform (not a finished algorithm product).
It already has a clear runtime architecture, baseline scenarios, and evaluation/logging skeleton, with extension points reserved for controller/costmap/behavior research.

## Current Status

### Completed
- Main control chain is established in agent mode:
  - `Nav2 -> /cmd_vel_nav -> supervisor -> /cmd_vel`
  - References:
    - `src/marl_car_ros2/launch/agent_nav2.launch.py`
    - `src/marl_car_ros2/marl_car_ros2/supervisor_node.py`
- Task/supervisor decision semantics are structured and traceable:
  - reason code normalization (`block_reason_code`, `recovery_reason_code`) is implemented and consumed downstream
  - References:
    - `src/marl_car_ros2/marl_car_ros2/shared_types.py`
    - `src/marl_car_ros2/marl_car_ros2/task_agent.py`
    - `src/marl_car_ros2/marl_car_ros2/supervisor_node.py`
    - `src/marl_car_ros2/marl_car_ros2/monitor_logger_node.py`
- Launch entrypoints are in place for simulation, baseline, agent, evaluation, and visualization:
  - `sim.launch.py`, `baseline_nav2.launch.py`, `agent_nav2.launch.py`, `evaluation.launch.py`, `nav2_visualization.launch.py`
- Baseline worlds exist and are mapped:
  - narrow corridor, sharp turns, dynamic crossing
  - References:
    - `src/marl_car_ros2/worlds/baseline_narrow_corridor.world`
    - `src/marl_car_ros2/worlds/baseline_sharp_turns.world`
    - `src/marl_car_ros2/worlds/baseline_dynamic_crossing.world`
    - `src/marl_car_ros2/config/baseline_world_scenarios.yaml`
- Monitor/evaluation logging foundation exists:
  - JSONL + SQLite timeline logging
  - experiment tags injected from evaluation launch
  - References:
    - `src/marl_car_ros2/launch/evaluation.launch.py`
    - `src/marl_car_ros2/marl_car_ros2/monitor_logger_node.py`
- Frontend assets exist:
  - robot model (`SDF` + `URDF`), RViz config, visualization marker nodes
  - References:
    - `src/marl_car_ros2/models/simple_marl_car/model.sdf`
    - `src/marl_car_ros2/urdf/simple_marl_car.urdf`
    - `src/marl_car_ros2/rviz/nav2_default.rviz`

### Partially Completed / In Progress
- Nav2 stack integration is wired in launch and params, but runtime stability still needs fixes for robust experiments.
- RViz display configuration is complete, but real-time data quality/consistency (especially full sensor-driven loop stability) still needs hardening.
- Baseline comparison scaffolding exists, but controller-profile switching workflow (DWB/RPP/improved) is not yet a complete one-command benchmark pipeline.

## Recent Changes (Audited)

### 1) Control Flow
- Kept supervisor as final velocity authority.
- Kept baseline node as compatibility/fallback path, not primary architecture.
- Explicitly remapped Nav2 `/cmd_vel` output to `/cmd_vel_nav` in agent mode.

### 2) State Semantics
- Unified blocked/no-progress/recovery reasoning fields across task agent and supervisor outputs.
- Logger derives timeline events (mode switch, no-progress, recovery, replan) from supervisor/status stream.

### 3) Launch and Scenario Layer
- Added/maintained scenario-aware evaluation entrypoint:
  - `scenario_name` selection (`narrow_corridor`, `sharp_turns`, `dynamic_crossing`, `custom`)
  - start/goal/world mapping from `baseline_world_scenarios.yaml`
  - dynamic obstacle deterministic defaults for `dynamic_crossing`
- Added baseline world convenience launches:
  - `baseline_world_narrow.launch.py`
  - `baseline_world_turns.launch.py`
  - `baseline_world_dynamic.launch.py`

### 4) Evaluation Infrastructure
- Added experiment tags propagation (`scenario/world/spawn/goal/agent_mode/planner_profile/run_id`) into monitor logger summaries/timeline payloads.

### 5) Visualization Layer
- Added RViz displays for:
  - RobotModel, TF, LaserScan
  - Global/Local path
  - Global/Local costmaps
  - Footprint and goal pose
  - Supervisor/risk/world-event markers

## Control Flow

### Agent mode (research path)
1. Nav2 publishes candidate command to `/cmd_vel_nav`
2. `task_agent` publishes high-level decision to `/agent/decision`
3. `supervisor_node` applies safety/mode arbitration
4. `supervisor_node` publishes final command to `/cmd_vel`
5. Gazebo bridge applies `/cmd_vel` to robot model

### Baseline mode (fallback path)
- `baseline_nav_node` can publish directly to `/cmd_vel` (or `/cmd_vel_nav` in compatibility setups)

## Architecture Overview

- `task_agent.py`
  - high-level mode/decision manager only
  - no direct low-level motor control
- `supervisor_node.py`
  - final `/cmd_vel` authority
  - blocked/cautious/recovery arbitration and safety checks
- `baseline_nav_node.py`
  - deterministic fallback local navigator for non-Nav2 or compatibility use
- `scenario_mutator.py`
  - repeatable dynamic disturbance control (including deterministic obstacle mode)
- `monitor_logger_node.py`
  - timeline/event/summary recorder for online monitoring and offline analysis
- `observation_builder.py`
  - shared snapshot interface for agent/supervisor/baseline logic

## Launch Entrypoints

- `src/marl_car_ros2/launch/sim.launch.py`
  - Gazebo + bridge + optional mutator + monitor
- `src/marl_car_ros2/launch/baseline_nav2.launch.py`
  - baseline navigation chain
- `src/marl_car_ros2/launch/agent_nav2.launch.py`
  - Nav2 + task_agent + supervisor chain
- `src/marl_car_ros2/launch/evaluation.launch.py`
  - scenario-based evaluation entrypoint
- `src/marl_car_ros2/launch/nav2_visualization.launch.py`
  - simulation + Nav2 + RViz visualization stack

## Baseline Worlds

- `narrow_corridor`
  - objective: near-obstacle conservative behavior + path centering
- `sharp_turns`
  - objective: pre-turn deceleration + mode switching through tight turns
- `dynamic_crossing`
  - objective: predictable dynamic obstacle interaction (slowdown/stop/replan behavior)

Mapping source:
- `src/marl_car_ros2/config/baseline_world_scenarios.yaml`

## Controller Reality Check

Current Nav2 local controller in repository is **DWB**, not RPP.

Reference:
- `src/marl_car_ros2/config/nav2_params.yaml`
  - `controller_server -> FollowPath -> plugin: dwb_core::DWBLocalPlanner`

## Known Gaps / Limitations

- Platform is not yet "algorithm complete"; it is a research-ready skeleton with pending runtime stabilization work.
- Nav2 runtime robustness still needs hardening for repeatable experiments across all baselines.
- Sensor/bridge consistency and full visualization data reliability still require validation hardening before large-scale controller comparisons.
- `planner_profile` is currently recorded as experiment metadata, not yet a full automatic controller-switch execution framework by itself.

## Planned Extension Points

The following are planned research extensions (not marked as completed):

1. Predicted risk layer / local costmap risk projection
2. APF or virtual-carrot guided RPP controller plugin
3. BT or supervisor mode-switching and recovery strategy refinement
4. Gazebo + RViz frontend experiment presentation hardening
5. Baseline world protocol refinement for:
   - narrow corridor: conservative + centered behavior
   - sharp turns: pre-turn decel + turning mode transition
   - dynamic obstacle: predict then slow/stop/replan

## Next Milestones

1. Stabilize runtime loop for repeatable baseline runs
2. Finalize DWB baseline benchmark protocol and metrics export
3. Add RPP integration path and controlled DWB-vs-RPP comparison
4. Add improved controller/plugin prototype and ablation pipeline
5. Promote visualization/evaluation scripts to one-command experiment workflow

## Build

```bash
cd /home/grok/ros2_ws
colcon build --packages-select marl_car_ros2
source /opt/ros/jazzy/setup.bash
source /home/grok/ros2_ws/install/setup.bash
```

## Quick Start

```bash
# baseline fallback chain
ros2 launch marl_car_ros2 baseline_nav2.launch.py

# agent research chain
ros2 launch marl_car_ros2 agent_nav2.launch.py

# scenario-based evaluation
ros2 launch marl_car_ros2 evaluation.launch.py scenario_name:=narrow_corridor agent_mode:=true

# visualization stack
ros2 launch marl_car_ros2 nav2_visualization.launch.py
```
