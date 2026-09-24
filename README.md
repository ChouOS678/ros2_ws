# ROS 2 + Gazebo + RViz + Nav2 Benchmark Platform

This repository is organized around one formal benchmark backbone and one separate manual demo/debug chain.

## Project Phase and Progress (2026-09-24)

The project has entered **Phase 2: initial comparative data collection**. The parameterized trajectory scenarios and the benchmark execution path are substantially in place; the current focus is collecting enough comparable runs to evaluate controller performance with confidence.

The first five-scenario comparison campaign ran `PP`, `APP`, `RPP`, and `DWPP` once each per scenario: 20 runs total, 18 successful and 2 unsuccessful. All 20 reports and their corresponding telemetry were imported into the benchmark portal database, totaling 34,454 telemetry samples for this campaign. The portal keeps this campaign separate from historical runs so the comparison is not mixed with older results.

Initial observations from this single-run-per-combination campaign:

- `straight`: all four controllers had effectively zero tracking error; this case does not discriminate controller quality well.
- `constant_curvature`: `PP` had the lowest mean tracking error at about 0.091 m, closely followed by `RPP` at about 0.092 m.
- `s_curve`: `PP` had the lowest mean tracking error at about 0.0058 m, followed by `RPP` at about 0.0077 m.
- `sharp_corner`: `APP` had the lowest mean tracking error at about 0.261 m; `DWPP` had a slightly lower maximum deviation than the other controllers.
- `clothoid`: `PP` and `APP` completed. `RPP` and `DWPP` did not complete: the RPP log reports an empty plan (`Resulting plan has 0 poses in it`), while DWPP reports `Failed to make progress`.

These are preliminary observations, not stable rankings: each scenario/controller combination has only one run. In particular, DWPP's low recorded tracking error on the failed clothoid run must not be interpreted as a successful result. No single controller was best across all scenarios.

The independent analysis web app is in `benchmark_portal/`. It stores run reports and run-ID-matched telemetry in `benchmark_portal/data/benchmark.sqlite`, shows historical and campaign data separately, and reports tracking error, speed, completion rate, and available telemetry-derived metrics. Start it from the workspace root with:

```bash
python3 -m benchmark_portal.backend
```

Then open `http://localhost:8765`. The campaign reports, logs, and telemetry archives are stored beneath `benchmark_portal/data/campaigns/campaign_20260924_114023/`.

### Next Phase

- Run repeated trials for each controller/scenario combination (at least three, preferably five or more) with the same initial state and configuration.
- Investigate the two clothoid failures and verify that completion, tracking error, speed, and smoothness are measured consistently.
- Use repeated-run distributions and completion rates to explain scenario-specific strengths and trade-offs; avoid causal or universal claims from one run.
- Grow the dataset through `python3 -m benchmark_portal.collect_campaign`; the benchmark code and scenario definitions remain the source of truth.

## Refactor Summary

This convergence pass made the following structural changes:

- formal benchmark execution was converged to `evaluation.launch.py`
- manual demo/debug execution was converged to `benchmark_demo.launch.py`
- duplicate historical launch entrypoints were removed
- benchmark default parameters were centralized in `src/marl_car_ros2/config/benchmark_defaults.yaml`
- shared config loading was centralized in `src/marl_car_ros2/marl_car_ros2/benchmark_config.py`
- scenario registration was moved to `src/marl_car_ros2/config/trajectory_scenarios.yaml`
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

`evaluation.launch.py` is the only formal trajectory benchmark entrypoint. It resolves trajectory/spawn/goal settings from `src/marl_car_ros2/config/trajectory_scenarios.yaml`, resolves runtime defaults from `src/marl_car_ros2/config/benchmark_defaults.yaml`, and feeds the runtime stack that produces benchmark logs and metrics.

`monitor_logger` is the runtime recording source for timeline and summary data. `evaluation_metrics` is the formal offline metrics computation path for benchmark outputs.

Formal benchmark flow:

1. `evaluation.launch.py` resolves defaults from `benchmark_defaults.yaml`
2. `trajectory_generator_node` deterministically generates `/reference_path` from `trajectory_scenarios.yaml`
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

## Unified Robot Description

The robot is described by one source file:

- `src/marl_car_ros2/urdf/simple_marl_car.urdf`

The URDF is used directly by `robot_state_publisher` for RViz and is also passed to `ros_gz_sim` when Gazebo spawns the robot. Gazebo-specific lidar, RGB-D camera, four-wheel skid-steer friction, and differential-drive configuration is embedded in the URDF using `<gazebo>` tags, so the robot geometry, joints, sensors, and drive plugins stay synchronized between Gazebo and RViz.

The frame hierarchy is `base_footprint -> base_link`, with four continuous wheel joints, `laser_link`, and `camera_link`. The runtime topics are `/scan`, `/odom`, `/camera/image_raw`, `/camera/points`, and `/camera/camera_info`.

## Scenario Registry

Trajectory benchmark registration is centralized in `src/marl_car_ros2/config/trajectory_scenarios.yaml`.

Current parameterized trajectory scenarios:

- `straight`
- `constant_curvature`
- `s_curve`
- `clothoid`
- `sharp_corner`

Each scenario defines trajectory geometry, sampling interval, curvature parameters, and goal pose in one place:

- `src/marl_car_ros2/config/trajectory_scenarios.yaml`

Formal launch examples:

```bash
ros2 launch marl_car_ros2 evaluation.launch.py scenario_name:=straight planner_profile:=pp
ros2 launch marl_car_ros2 evaluation.launch.py scenario_name:=s_curve planner_profile:=app
ros2 run marl_car_ros2 benchmark_runner --scenario-name clothoid --planner-profile rpp
```

To run the full 5-scenario × 4-controller comparison and import each result into the portal database:

```bash
python3 -m benchmark_portal.collect_campaign
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

Formal trajectory metadata remains centralized in `src/marl_car_ros2/config/trajectory_scenarios.yaml`.

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

## Trajectory Benchmark Usage

The formal benchmark no longer selects scenario-specific Gazebo worlds. `minimal.world` is only the empty simulation carrier for the robot; benchmark geometry is generated by `trajectory_generator_node`.

```bash
ros2 launch marl_car_ros2 evaluation.launch.py scenario_name:=straight
ros2 launch marl_car_ros2 evaluation.launch.py scenario_name:=constant_curvature planner_profile:=app
ros2 launch marl_car_ros2 evaluation.launch.py scenario_name:=s_curve planner_profile:=rpp
ros2 launch marl_car_ros2 evaluation.launch.py scenario_name:=clothoid planner_profile:=dwpp
ros2 launch marl_car_ros2 evaluation.launch.py scenario_name:=sharp_corner planner_profile:=pp
```

## Current Validation Notes

- The formal benchmark backbone is available for scenario-based evaluation and result generation.
- The demo/debug chain is available for RViz/Gazebo validation, operator-triggered checks, and integration debugging.
- One initial run exists for each of the 20 scenario/controller combinations; repeated runs are still required before making strong comparative claims.
- The latest campaign database and per-run telemetry are available in `benchmark_portal/data/benchmark.sqlite`; current campaign completion is 18/20.
- Clothoid completion remains a known evaluation issue for RPP and DWPP and should be investigated in the next data-collection phase.
