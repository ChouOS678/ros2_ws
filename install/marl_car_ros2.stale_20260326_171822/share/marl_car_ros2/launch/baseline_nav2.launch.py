from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
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
                default_value=os.path.join(get_package_share_directory("marl_car_ros2"), "worlds", "minimal.world"),
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
            sim,
            baseline_controller,
        ]
    )
