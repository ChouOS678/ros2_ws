from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _load_baseline_scenarios(pkg_share: str) -> dict:
    cfg_path = os.path.join(pkg_share, "config", "baseline_world_scenarios.yaml")
    with open(cfg_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    scenarios = data.get("scenarios", {})
    if not isinstance(scenarios, dict):
        return {}
    return scenarios


def _build_eval_stack(context, pkg_share: str, launch_dir: str, scenarios: dict):
    scenario_name = LaunchConfiguration("scenario_name").perform(context).strip()
    agent_mode = LaunchConfiguration("agent_mode").perform(context).strip().lower() == "true"

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    start_gazebo = LaunchConfiguration("start_gazebo").perform(context)
    start_mutator = LaunchConfiguration("start_mutator").perform(context)
    start_bridge = LaunchConfiguration("start_bridge").perform(context)
    start_monitor = LaunchConfiguration("start_monitor").perform(context)
    planner_profile = LaunchConfiguration("planner_profile").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context).strip()
    run_id = LaunchConfiguration("run_id").perform(context)

    world_file = LaunchConfiguration("world_file").perform(context)
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
    goal_x = LaunchConfiguration("goal_x").perform(context)
    goal_y = LaunchConfiguration("goal_y").perform(context)

    enable_deterministic_obstacle = LaunchConfiguration("enable_deterministic_obstacle").perform(context)
    dynamic_obstacle_mode = LaunchConfiguration("dynamic_obstacle_mode").perform(context)
    dynamic_obstacle_cmd_topic = LaunchConfiguration("dynamic_obstacle_cmd_topic").perform(context)
    dynamic_obstacle_speed_mps = LaunchConfiguration("dynamic_obstacle_speed_mps").perform(context)
    dynamic_obstacle_trigger_mode = LaunchConfiguration("dynamic_obstacle_trigger_mode").perform(context)
    dynamic_obstacle_trigger_time_s = LaunchConfiguration("dynamic_obstacle_trigger_time_s").perform(context)
    dynamic_obstacle_trigger_robot_x = LaunchConfiguration("dynamic_obstacle_trigger_robot_x").perform(context)
    dynamic_obstacle_crossing_span_m = LaunchConfiguration("dynamic_obstacle_crossing_span_m").perform(context)
    dynamic_obstacle_initial_direction = LaunchConfiguration("dynamic_obstacle_initial_direction").perform(context)
    dynamic_obstacle_repeat = LaunchConfiguration("dynamic_obstacle_repeat").perform(context)

    if scenario_name and scenario_name != "custom":
        if scenario_name not in scenarios:
            names = ", ".join(sorted(scenarios.keys()))
            raise RuntimeError(f"Unknown scenario_name='{scenario_name}'. Available: {names}, custom")
        s = scenarios[scenario_name]
        if not isinstance(s, dict):
            raise RuntimeError(f"Scenario '{scenario_name}' config is invalid")
        world_rel = str(s.get("world_file", ""))
        world_file = world_rel if os.path.isabs(world_rel) else os.path.join(pkg_share, world_rel)

        spawn_cfg = s.get("spawn", {}) if isinstance(s.get("spawn", {}), dict) else {}
        goal_cfg = s.get("goal", {}) if isinstance(s.get("goal", {}), dict) else {}
        spawn_x = str(spawn_cfg.get("x", spawn_x))
        spawn_y = str(spawn_cfg.get("y", spawn_y))
        spawn_z = str(spawn_cfg.get("z", spawn_z))
        spawn_yaw = str(spawn_cfg.get("yaw", spawn_yaw))
        goal_x = str(goal_cfg.get("x", goal_x))
        goal_y = str(goal_cfg.get("y", goal_y))

        dyn_cfg = s.get("dynamic_obstacle", {}) if isinstance(s.get("dynamic_obstacle", {}), dict) else {}
        if dyn_cfg:
            enable_deterministic_obstacle = str(dyn_cfg.get("enable", enable_deterministic_obstacle)).lower()
            dynamic_obstacle_mode = str(dyn_cfg.get("mode", dynamic_obstacle_mode))
            dynamic_obstacle_cmd_topic = str(dyn_cfg.get("cmd_topic", dynamic_obstacle_cmd_topic))
            dynamic_obstacle_speed_mps = str(dyn_cfg.get("speed_mps", dynamic_obstacle_speed_mps))
            dynamic_obstacle_trigger_mode = str(dyn_cfg.get("trigger_mode", dynamic_obstacle_trigger_mode))
            dynamic_obstacle_trigger_time_s = str(dyn_cfg.get("trigger_time_s", dynamic_obstacle_trigger_time_s))
            dynamic_obstacle_trigger_robot_x = str(dyn_cfg.get("trigger_robot_x", dynamic_obstacle_trigger_robot_x))
            dynamic_obstacle_crossing_span_m = str(dyn_cfg.get("crossing_span_m", dynamic_obstacle_crossing_span_m))
            dynamic_obstacle_initial_direction = str(
                dyn_cfg.get("initial_direction", dynamic_obstacle_initial_direction)
            )
            dynamic_obstacle_repeat = str(dyn_cfg.get("repeat", dynamic_obstacle_repeat)).lower()

    if not params_file:
        profile = planner_profile.strip().lower()
        if profile in ("rpp", "regulated_pure_pursuit", "regulated-pure-pursuit"):
            params_file = os.path.join(pkg_share, "config", "nav2_params_rpp.yaml")
        else:
            params_file = os.path.join(pkg_share, "config", "nav2_params.yaml")

    env_actions = [
        SetEnvironmentVariable("MARL_EXPERIMENT_SCENARIO", scenario_name or "custom"),
        SetEnvironmentVariable("MARL_EXPERIMENT_WORLD_FILE", world_file),
        SetEnvironmentVariable("MARL_EXPERIMENT_SPAWN_X", spawn_x),
        SetEnvironmentVariable("MARL_EXPERIMENT_SPAWN_Y", spawn_y),
        SetEnvironmentVariable("MARL_EXPERIMENT_SPAWN_Z", spawn_z),
        SetEnvironmentVariable("MARL_EXPERIMENT_SPAWN_YAW", spawn_yaw),
        SetEnvironmentVariable("MARL_EXPERIMENT_GOAL_X", goal_x),
        SetEnvironmentVariable("MARL_EXPERIMENT_GOAL_Y", goal_y),
        SetEnvironmentVariable("MARL_EXPERIMENT_AGENT_MODE", "true" if agent_mode else "false"),
        SetEnvironmentVariable("MARL_EXPERIMENT_PLANNER_PROFILE", planner_profile),
        SetEnvironmentVariable("MARL_EXPERIMENT_RUN_ID", run_id),
    ]

    common_args = {
        "use_sim_time": use_sim_time,
        "start_gazebo": start_gazebo,
        "start_mutator": start_mutator,
        "start_bridge": start_bridge,
        "start_monitor": start_monitor,
        "world_file": world_file,
        "spawn_x": spawn_x,
        "spawn_y": spawn_y,
        "spawn_z": spawn_z,
        "spawn_yaw": spawn_yaw,
        "goal_x": goal_x,
        "goal_y": goal_y,
        "enable_deterministic_obstacle": enable_deterministic_obstacle,
        "dynamic_obstacle_mode": dynamic_obstacle_mode,
        "dynamic_obstacle_cmd_topic": dynamic_obstacle_cmd_topic,
        "dynamic_obstacle_speed_mps": dynamic_obstacle_speed_mps,
        "dynamic_obstacle_trigger_mode": dynamic_obstacle_trigger_mode,
        "dynamic_obstacle_trigger_time_s": dynamic_obstacle_trigger_time_s,
        "dynamic_obstacle_trigger_robot_x": dynamic_obstacle_trigger_robot_x,
        "dynamic_obstacle_crossing_span_m": dynamic_obstacle_crossing_span_m,
        "dynamic_obstacle_initial_direction": dynamic_obstacle_initial_direction,
        "dynamic_obstacle_repeat": dynamic_obstacle_repeat,
    }

    if agent_mode:
        nav_stack = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "agent_nav2.launch.py")),
            launch_arguments={
                **common_args,
                "params_file": params_file,
                "start_nav2": "true",
                "start_nav_executor": "false",
                "start_task_agent": "true",
                "start_supervisor": "true",
            }.items(),
        )
    else:
        nav_stack = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "baseline_nav2.launch.py")),
            launch_arguments={
                **common_args,
                "start_baseline_controller": "true",
            }.items(),
        )

    return [*env_actions, nav_stack]


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("marl_car_ros2")
    launch_dir = os.path.join(pkg_share, "launch")
    scenarios = _load_baseline_scenarios(pkg_share)

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_mutator", default_value="true"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_monitor", default_value="true"),
            DeclareLaunchArgument("agent_mode", default_value="true"),
            DeclareLaunchArgument("scenario_name", default_value="custom"),
            DeclareLaunchArgument("planner_profile", default_value="unspecified"),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("run_id", default_value=""),
            DeclareLaunchArgument(
                "world_file",
                default_value=os.path.join(pkg_share, "worlds", "minimal.world"),
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.1"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("goal_x", default_value="8.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("enable_deterministic_obstacle", default_value="false"),
            DeclareLaunchArgument("dynamic_obstacle_mode", default_value="crossing_deterministic"),
            DeclareLaunchArgument("dynamic_obstacle_cmd_topic", default_value="/model/dynamic_crossing_box/cmd_vel"),
            DeclareLaunchArgument("dynamic_obstacle_speed_mps", default_value="0.45"),
            DeclareLaunchArgument("dynamic_obstacle_trigger_mode", default_value="time_after_start"),
            DeclareLaunchArgument("dynamic_obstacle_trigger_time_s", default_value="4.0"),
            DeclareLaunchArgument("dynamic_obstacle_trigger_robot_x", default_value="2.0"),
            DeclareLaunchArgument("dynamic_obstacle_crossing_span_m", default_value="1.6"),
            DeclareLaunchArgument("dynamic_obstacle_initial_direction", default_value="1.0"),
            DeclareLaunchArgument("dynamic_obstacle_repeat", default_value="true"),
            OpaqueFunction(function=lambda context: _build_eval_stack(context, pkg_share, launch_dir, scenarios)),
        ]
    )
