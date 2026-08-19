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

## Sensor Bridge Contract

Gazebo Harmonic laser data is bridged directly by `ros_gz_bridge`:

```text
Gazebo LaserScan -> /scan (sensor_msgs/msg/LaserScan) -> Nav2 costmaps
```

The bridge configuration is in `src/marl_car_ros2/launch/sim.launch.py`. The ROS topic is unified as `/scan`; no `/scan_raw` or self-looping `/scan` stamp bridge is used. The LaserScan message carries `frame_id: lidar_link`; TF is used by Nav2 to relate that sensor frame to `base_link`, while the scan data itself is not transformed into a different topic.

The bridge also provides:

- Gazebo clock to `/clock`
- Gazebo odometry to `/odom`
- ROS velocity commands to the simulated vehicle

## Controller Implementations

The controller comparison stack contains four actual controller implementations:

| Controller ID | Implementation | Role |
|---|---|---|
| `PP` | `nav2_pure_pursuit_controller::PurePursuitController` | Fixed-lookahead pure pursuit |
| `APP` | `nav2_pure_pursuit_controller::AdaptivePurePursuitController` | Velocity-scaled adaptive lookahead |
| `RPP` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | Nav2 regulated pure pursuit |
| `DWPP` | `dwb_core::DWBLocalPlanner` | Dynamic-window controller through Nav2 DWB |

The custom PP and APP plugins are implemented in:

- `src/marl_nav2_plugins/include/marl_nav2_plugins/pure_pursuit_controller.hpp`
- `src/marl_nav2_plugins/src/pure_pursuit_controller.cpp`
- `src/marl_nav2_plugins/controller_plugins.xml`

The controller instances are registered in `src/marl_car_ros2/config/nav2_params.yaml`:

```yaml
controller_plugins: [PP, APP, RPP, DWPP, FollowPath]
```

`FollowPath` is a compatibility controller ID that points to the PP plugin. The project default controller is `PP`, including the default profile resolver and goal sender. This preserves compatibility with Nav2 behavior trees that request `FollowPath`, while making PP the explicit default instance.

The effective controller mapping is:

```text
PP          -> custom fixed-lookahead pure pursuit
APP         -> custom velocity-scaled pure pursuit
RPP         -> official Nav2 regulated pure pursuit
DWPP        -> official Nav2 DWB dynamic-window controller
FollowPath  -> PP
```

## Controller Parameter Rules

Each controller parameter block contains only parameters supported by its implementation.

PP uses:

```yaml
PP:
  plugin: nav2_pure_pursuit_controller::PurePursuitController
  desired_linear_vel: 0.65
  lookahead_dist: 0.8
  min_lookahead_dist: 0.35
  max_lookahead_dist: 1.2
```

APP uses the same parameters plus velocity-scaled lookahead:

```yaml
APP:
  plugin: nav2_pure_pursuit_controller::AdaptivePurePursuitController
  desired_linear_vel: 0.65
  lookahead_dist: 0.8
  min_lookahead_dist: 0.35
  max_lookahead_dist: 1.2
  lookahead_time: 1.5
```

RPP retains the official Nav2 RPP parameters, including regulation, collision checking, cost-based speed scaling, and lookahead controls. DWPP retains the DWB velocity sampling and critic parameters.

`FollowPath` has the same plugin and parameter values as `PP`. It is an alias, not a separate algorithm.

The removed `nav2_params_rpp.yaml` file was a duplicate parameter preset. All profiles now use `src/marl_car_ros2/config/nav2_params.yaml`; selecting `planner_profile:=rpp` changes the controller ID to `RPP` without selecting a duplicate file.

## Controller Validation

The controller plugin package and the ROS 2 application package are built together:

```bash
cd /home/grok/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select marl_nav2_plugins marl_car_ros2 --symlink-install
source /home/grok/ros2_ws/install/setup.bash
```

A direct `controller_server` lifecycle configuration verified successful loading of:

- `PP`
- `APP`
- `RPP`
- `DWPP`
- `FollowPath`

The complete navigation launch may still depend on the separately provided `fault_tolerant_lifecycle_manager` executable.

## Build

```bash
cd /home/grok/ros2_ws
colcon build --packages-select marl_nav2_plugins marl_car_ros2 --symlink-install
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
