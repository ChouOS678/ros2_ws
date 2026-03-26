from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from marl_car_ros2.benchmark_config import load_benchmark_defaults, resolve_pkg_path
import os


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_mutator = LaunchConfiguration("start_mutator")
    start_bridge = LaunchConfiguration("start_bridge")
    start_monitor = LaunchConfiguration("start_monitor")
    start_baseline_controller = LaunchConfiguration("start_baseline_controller")
    world_file = LaunchConfiguration("world_file")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    enable_deterministic_obstacle = LaunchConfiguration("enable_deterministic_obstacle")
    dynamic_obstacle_mode = LaunchConfiguration("dynamic_obstacle_mode")
    dynamic_obstacle_cmd_topic = LaunchConfiguration("dynamic_obstacle_cmd_topic")
    dynamic_obstacle_speed_mps = LaunchConfiguration("dynamic_obstacle_speed_mps")
    dynamic_obstacle_trigger_mode = LaunchConfiguration("dynamic_obstacle_trigger_mode")
    dynamic_obstacle_trigger_time_s = LaunchConfiguration("dynamic_obstacle_trigger_time_s")
    dynamic_obstacle_trigger_robot_x = LaunchConfiguration("dynamic_obstacle_trigger_robot_x")
    dynamic_obstacle_crossing_span_m = LaunchConfiguration("dynamic_obstacle_crossing_span_m")
    dynamic_obstacle_initial_direction = LaunchConfiguration("dynamic_obstacle_initial_direction")
    dynamic_obstacle_repeat = LaunchConfiguration("dynamic_obstacle_repeat")
    pkg_share = get_package_share_directory("marl_car_ros2")
    defaults = load_benchmark_defaults(pkg_share)
    spawn_defaults = defaults.get("spawn", {}) if isinstance(defaults.get("spawn", {}), dict) else {}
    goal_defaults = defaults.get("goal", {}) if isinstance(defaults.get("goal", {}), dict) else {}
    dynamic_defaults = (
        defaults.get("dynamic_obstacle", {}) if isinstance(defaults.get("dynamic_obstacle", {}), dict) else {}
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("marl_car_ros2"), "launch", "sim.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_gazebo": start_gazebo,
            "start_bridge": start_bridge,
            "start_monitor": start_monitor,
            "start_mutator": start_mutator,
            "world_file": world_file,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
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
        }.items(),
    )

    baseline_controller = Node(
        package="marl_car_ros2",
        executable="baseline_nav_node",
        name="baseline_nav_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "output_topic": "/cmd_vel",
                "goal_x": goal_x,
                "goal_y": goal_y,
            }
        ],
        condition=IfCondition(start_baseline_controller),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_mutator", default_value="true"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_monitor", default_value="true"),
            DeclareLaunchArgument("start_baseline_controller", default_value="true"),
            DeclareLaunchArgument(
                "world_file",
                default_value=resolve_pkg_path(pkg_share, str(defaults.get("world_file", "")), fallback="worlds/minimal.world"),
            ),
            DeclareLaunchArgument("spawn_x", default_value=str(spawn_defaults.get("x", 0.0))),
            DeclareLaunchArgument("spawn_y", default_value=str(spawn_defaults.get("y", 0.0))),
            DeclareLaunchArgument("spawn_z", default_value=str(spawn_defaults.get("z", 0.0))),
            DeclareLaunchArgument("spawn_yaw", default_value=str(spawn_defaults.get("yaw", 0.0))),
            DeclareLaunchArgument("goal_x", default_value=str(goal_defaults.get("x", 8.0))),
            DeclareLaunchArgument("goal_y", default_value=str(goal_defaults.get("y", 0.0))),
            DeclareLaunchArgument("enable_deterministic_obstacle", default_value=str(bool(dynamic_defaults.get("enable", False))).lower()),
            DeclareLaunchArgument("dynamic_obstacle_mode", default_value=str(dynamic_defaults.get("mode", "crossing_deterministic"))),
            DeclareLaunchArgument("dynamic_obstacle_cmd_topic", default_value=str(dynamic_defaults.get("cmd_topic", "/model/dynamic_crossing_box/cmd_vel"))),
            DeclareLaunchArgument("dynamic_obstacle_speed_mps", default_value=str(dynamic_defaults.get("speed_mps", 0.45))),
            DeclareLaunchArgument("dynamic_obstacle_trigger_mode", default_value=str(dynamic_defaults.get("trigger_mode", "time_after_start"))),
            DeclareLaunchArgument("dynamic_obstacle_trigger_time_s", default_value=str(dynamic_defaults.get("trigger_time_s", 4.0))),
            DeclareLaunchArgument("dynamic_obstacle_trigger_robot_x", default_value=str(dynamic_defaults.get("trigger_robot_x", 2.0))),
            DeclareLaunchArgument("dynamic_obstacle_crossing_span_m", default_value=str(dynamic_defaults.get("crossing_span_m", 1.6))),
            DeclareLaunchArgument("dynamic_obstacle_initial_direction", default_value=str(dynamic_defaults.get("initial_direction", 1.0))),
            DeclareLaunchArgument("dynamic_obstacle_repeat", default_value=str(bool(dynamic_defaults.get("repeat", True))).lower()),
            sim,
            baseline_controller,
        ]
    )
