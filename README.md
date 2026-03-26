# ROS 2 + Gazebo + RViz + Nav2 Benchmark Platform

This repository is organized around one formal benchmark backbone and one separate manual demo/debug chain.

## Refactor Summary

This convergence pass made the following structural changes:

- formal benchmark execution was converged to `evaluation.launch.py`
- manual demo/debug execution was converged to `benchmark_demo.launch.py`
- duplicate historical launch entrypoints were removed
- benchmark default parameters were centralized in `src/marl_car_ros2/config/benchmark_defaults.yaml`
- shared config loading was centralized in `src/marl_car_ros2/marl_car_ros2/benchmark_config.py`
- scenario registration was kept in `src/marl_car_ros2/config/baseline_world_scenarios.yaml`
- scenario categories were normalized around:
  - high curvature / sharp turns
  - narrow passage
  - dynamic obstacle extension
- historical coupled helper script `auto_eval_pipeline.py` was removed
- `wsl2_demo_ctl.sh` was migrated to the demo/debug entrypoint

## Official Benchmark Backbone

Formal benchmark runs use only:

- `src/marl_car_ros2/launch/evaluation.launch.py`
- `src/marl_car_ros2/marl_car_ros2/monitor_logger_node.py`
- `src/marl_car_ros2/marl_car_ros2/evaluation_metrics.py`

`evaluation.launch.py` is the only formal scenario-based benchmark entrypoint. It resolves scenario/world/spawn/goal settings from `src/marl_car_ros2/config/baseline_world_scenarios.yaml`, resolves benchmark defaults from `src/marl_car_ros2/config/benchmark_defaults.yaml`, and feeds the runtime stack that produces benchmark logs and metrics.

`monitor_logger` is the runtime recording source for timeline and summary data. `evaluation_metrics` is the formal offline metrics computation path for benchmark outputs.

Formal benchmark flow:

1. `evaluation.launch.py` resolves defaults from `benchmark_defaults.yaml`
2. `evaluation.launch.py` resolves scenario settings from `baseline_world_scenarios.yaml`
3. runtime data is recorded by `monitor_logger`
4. offline results are computed by `evaluation_metrics` or `benchmark_runner`

## Manual Demo / Debug Chain

Manual validation and interactive visualization use:

- `src/marl_car_ros2/launch/benchmark_demo.launch.py`
- `src/marl_car_ros2/marl_car_ros2/benchmark_gui.py`
- `src/marl_car_ros2/marl_car_ros2/benchmark_visualizer.py`

This chain is intended for human-triggered integration/debug sessions, RViz/Gazebo visibility checks, and interactive verification. It is not the formal benchmark result source.

## Runtime Responsibilities

- `supervisor_node.py`
  final `/cmd_vel` authority and safety gate
- `task_agent.py`
  high-level mode decision only
- `baseline_nav_node.py`
  baseline/fallback navigation path only
- `scenario_mutator.py`
  repeatable scenario disturbance control
- `monitor_logger_node.py`
  benchmark timeline and summary recording

## Scenario Registry

Scenario registration is centralized in `src/marl_car_ros2/config/baseline_world_scenarios.yaml`.

Current benchmark categories:

- `sharp_turns`
  high-curvature / sharp-turn benchmark
- `narrow_corridor`
  narrow-passage benchmark
- `dynamic_crossing`
  dynamic-obstacle extension benchmark

Scenario metadata and registration stay in one place:

- `src/marl_car_ros2/config/baseline_world_scenarios.yaml`

Formal launch examples:

```bash
ros2 launch marl_car_ros2 evaluation.launch.py scenario_name:=narrow_corridor

ros2 launch marl_car_ros2 evaluation.launch.py scenario_name:=sharp_turns planner_profile:=rpp

ros2 run marl_car_ros2 benchmark_runner --scenario-name dynamic_crossing --planner-profile rpp
```

Manual demo example:

```bash
ros2 launch marl_car_ros2 benchmark_demo.launch.py
```

## Single Parameter Source

Benchmark defaults are centralized in `src/marl_car_ros2/config/benchmark_defaults.yaml`.

That file now provides the shared default source for:

- formal benchmark default scenario
- default controller/planner profile mapping inputs
- default world/spawn/goal values
- default dynamic obstacle parameters
- supervisor `benchmark_mode` default semantics
- demo-chain default visualization toggles

Formal scenario metadata remains centralized in `src/marl_car_ros2/config/baseline_world_scenarios.yaml`.

Internal launch files such as `agent_nav2.launch.py`, `baseline_nav2.launch.py`, and `sim.launch.py` are still used as internal building blocks, but they are no longer documented as public benchmark entrypoints.

## Build

```bash
cd /home/grok/ros2_ws
colcon build --packages-select marl_car_ros2
source /opt/ros/jazzy/setup.bash
source /home/grok/ros2_ws/install/setup.bash
```

## Migration Notes

- `marl_stack.launch.py` and `marl_stack_minimal.launch.py`
  use `benchmark_demo.launch.py` for manual sessions, or `evaluation.launch.py` for formal runs
- `baseline_world_narrow.launch.py`, `baseline_world_turns.launch.py`, `baseline_world_dynamic.launch.py`
  use `evaluation.launch.py scenario_name:=...`
- `nav2_visualization.launch.py`
  use `benchmark_demo.launch.py`
- old ad-hoc evaluation script paths
  use `ros2 run marl_car_ros2 benchmark_runner`

## Current Validation Notes

- The formal benchmark backbone is available for scenario-based evaluation and result generation.
- The demo/debug chain is available for RViz/Gazebo validation, operator-triggered checks, and integration debugging.
- Stability and mission quality should still be verified before making strong comparative claims across controller profiles.
