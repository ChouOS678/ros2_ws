# Mobile Robot Thesis Architecture (ROS 2 Jazzy + Gazebo Sim)

This workspace now supports a research-oriented architecture for thesis experiments:

- Baseline path: navigation without the Agent task-control layer
- Proposed path: Nav-executor + `task_agent` + `supervisor`
- Disturbance/scenario injection with reproducible seeds
- Unified timeline logging for online/offline experiment analysis

The design intentionally avoids overengineering:
- ROS 2 nodes are implementation/deployment units
- Agent is a higher-level decision layer (goal/state/decision)
- Focus is task-level control over complex navigation situations, not distributed MAS theory

---

## 1) Old vs New Architecture

### Old (prototype, centralized)
- `multi_agent_game` contained planner/chassis policies and drove main control loop
- `marl_env_wrapper` mixed runtime control with reward and termination logic
- System was useful for early validation but too coupled for thesis experiments

### New (thesis-oriented, layered)
- Lower execution layer remains ROS2/Gazebo command chain
- Added upper task-control layer:
  - `task_agent`: emits high-level decisions only
  - `supervisor_node`: validates decisions, enforces safety, arbitrates final motion
- Refactored eval logic out of runtime path:
  - `observation_builder.py`
  - `reward_evaluator.py` (eval/training utility)
  - `termination_checker.py` (eval/training utility)
- `scenario_mutator.py` replaces mutator role for repeatable disturbance experiments

---

## 2) Module Responsibilities

### Runtime control modules
- `marl_car_ros2/task_agent.py`
  - mission/task-level decision making
  - outputs one of:
    - `normal_navigation`
    - `trigger_replan`
    - `cautious_mode`
    - `pause_and_wait`
    - `recovery_request`
    - `fallback_to_nav2`
  - does not publish low-level velocity

- `marl_car_ros2/supervisor_node.py`
  - consumes `/agent/decision`
  - consumes lower-layer velocity command source (`/cmd_vel_nav`)
  - applies safety constraints / timeout / no-progress checks
  - owns final command authority and publishes `/cmd_vel`

- `marl_car_ros2/observation_builder.py`
  - builds runtime observation snapshots from `/odom`, `/scan`, `/world_model/events`
  - shared by task agent / supervisor / compatibility nodes

- `marl_car_ros2/scenario_mutator.py`
  - injects disturbance events (ghost probe / friction drop)
  - supports `random_seed` and configurable event probabilities

- `marl_car_ros2/monitor_logger_node.py`
  - keeps JSONL + SQLite timeline logging
  - logs architecture-level metrics:
    - decision latency
    - supervisor override count
    - replan count
    - recovery trigger count
    - blocked/stuck duration
    - mission completion status

### Eval/training utility modules
- `marl_car_ros2/reward_evaluator.py`
- `marl_car_ros2/termination_checker.py`

### Compatibility modules
- `marl_car_ros2/marl_env_wrapper.py`
  - kept for legacy MARL/eval workflows
  - no longer the recommended runtime control path

- `marl_car_ros2/world_model_mutator.py`
  - compatibility wrapper to `scenario_mutator`

- `marl_car_ros2/multi_agent_game.py`
  - legacy centralized loop retained for backward compatibility

---

## 3) Topic / Service Flow (New Agent Mode)

### Core control flow
1. Lower nav executor publishes command candidate to `/cmd_vel_nav`
2. `task_agent` publishes high-level decision to `/agent/decision`
3. `supervisor_node` arbitrates and publishes final `/cmd_vel`
4. Gazebo bridge executes `/cmd_vel` and provides `/odom` + `/scan`

### Monitoring and experiment flow
- `task_agent` / `supervisor_node` publish structured status/events:
  - `/agents/status`
  - `/agents/events`
- `scenario_mutator` publishes disturbance events:
  - `/world_model/events`
- `monitor_logger` publishes summary:
  - `/monitor/summary`
- Query services:
  - `/task_agent/query_state`
  - `/supervisor/query_state`
  - `/monitor/query_state`

---

## 4) Launch Layout

New launch files:
- `launch/sim.launch.py`
  - Gazebo Sim + ros_gz bridge + scenario mutator + monitor
- `launch/baseline_nav2.launch.py`
  - baseline mode without task-agent layer
- `launch/agent_nav2.launch.py`
  - agent-enhanced mode (`task_agent + supervisor`)
- `launch/evaluation.launch.py`
  - switch between baseline/agent mode for experiments
- `launch/marl_stack_minimal.launch.py`
  - compatibility entrypoint (legacy workflows still supported)

---

## 5) How to Run

### A. Baseline mode (no task-agent layer)
```bash
source /opt/ros/jazzy/setup.bash
source /home/grok/ros2_ws/install/setup.bash
ros2 launch marl_car_ros2 baseline_nav2.launch.py
```

### B. Agent-enhanced mode (proposed method)
```bash
source /opt/ros/jazzy/setup.bash
source /home/grok/ros2_ws/install/setup.bash
ros2 launch marl_car_ros2 agent_nav2.launch.py
```

### C. Evaluation launch switch
```bash
# agent mode
ros2 launch marl_car_ros2 evaluation.launch.py agent_mode:=true

# baseline mode
ros2 launch marl_car_ros2 evaluation.launch.py agent_mode:=false
```

### D. Legacy compatibility path
```bash
# legacy centralized game loop
ros2 launch marl_car_ros2 marl_stack_minimal.launch.py start_game:=true
```

---

## 6) Build

```bash
cd /home/grok/ros2_ws
colcon build --packages-select marl_car_ros2
source install/setup.bash
```

---

## 7) Experiment Support Mapping

This architecture is ready for staged thesis experiments:

1. End-to-end architecture run
- use `agent_nav2.launch.py`

2. Traditional baseline
- use `baseline_nav2.launch.py`

3. Fixed complex scenarios (2~3)
- configure `scenario_mutator` params and seed

4. Anomaly recovery group
- evaluate `recovery_request` / supervisor intervention behavior

5. Ablations (1~2)
- disable task agent (`start_task_agent:=false`) or adjust supervisor checks
- compare metrics from monitor SQLite/JSONL

---

## 8) Notes on Nav2 Integration

The new architecture keeps a clear command interface (`/cmd_vel_nav` -> supervisor -> `/cmd_vel`) so a full Nav2 stack can be plugged in directly as the lower executor.

For local reproducibility in this repository, a compatibility baseline nav executor node is included (`baseline_nav_node`) to keep simulation runnable even when a full Nav2 configuration is not yet wired.

---

## 9) What Was Done Today (2026-03-19)

### Refactor delivered
- Implemented thesis-oriented layered architecture:
  - added `task_agent.py` (task-level decisions only)
  - added `supervisor_node.py` (final motion arbitration + safety authority)
- Decoupled runtime observation from evaluation logic:
  - added `observation_builder.py`
  - added `reward_evaluator.py`
  - added `termination_checker.py`
- Repositioned disturbance module:
  - added `scenario_mutator.py` (seeded/configurable injector)
  - kept `world_model_mutator.py` as compatibility wrapper
- Added compatibility lower executor:
  - added `baseline_nav_node.py` (publishes to `/cmd_vel` or `/cmd_vel_nav`)
- Upgraded monitor/logger:
  - `monitor_logger_node.py` now logs architecture-level metrics
  - SQLite now includes `arch_metrics` table
- Added/updated launch architecture:
  - `sim.launch.py`
  - `baseline_nav2.launch.py`
  - `agent_nav2.launch.py`
  - `evaluation.launch.py`
  - `marl_stack_minimal.launch.py` kept as compatibility entrypoint
- Updated package entrypoints in `setup.py`.

### Build and launch verification
- Build passed:
  - `colcon build --packages-select marl_car_ros2`
- Launch argument parsing passed:
  - baseline / agent / evaluation / compatibility launch files.
- Smoke startup passed for baseline and agent mode (processes start successfully).

### Gazebo real run result (today)
- Executed real Gazebo run with `agent_nav2.launch.py`.
- System startup succeeded:
  - Gazebo, bridge, scenario mutator, monitor, task agent, supervisor all running
  - robot spawn successful (`Entity creation successful`)
- Runtime observation:
  - `/odom` available
  - `/agent/decision` reported `pause_and_wait` with reason `sensor_not_ready`
  - `/supervisor/status` stayed in `waiting`
  - `/cmd_vel` remained zero due to supervisor safety hold
- Current blocking point:
  - `/scan` topic exists but no message observed during test window
  - therefore agent layer correctly refuses to enter active navigation.

### Immediate next action
- Fix `/scan` data path (sensor publish/bridge chain) to unlock active movement in agent mode.
