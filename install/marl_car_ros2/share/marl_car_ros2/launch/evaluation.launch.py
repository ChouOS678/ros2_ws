from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_mutator = LaunchConfiguration("start_mutator")
    start_bridge = LaunchConfiguration("start_bridge")
    start_monitor = LaunchConfiguration("start_monitor")
    agent_mode = LaunchConfiguration("agent_mode")

    launch_dir = os.path.join(get_package_share_directory("marl_car_ros2"), "launch")

    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "baseline_nav2.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_gazebo": start_gazebo,
            "start_mutator": start_mutator,
            "start_bridge": start_bridge,
            "start_monitor": start_monitor,
            "start_baseline_controller": "true",
        }.items(),
        condition=UnlessCondition(agent_mode),
    )

    agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "agent_nav2.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_gazebo": start_gazebo,
            "start_mutator": start_mutator,
            "start_bridge": start_bridge,
            "start_monitor": start_monitor,
            "start_nav_executor": "true",
            "start_task_agent": "true",
            "start_supervisor": "true",
        }.items(),
        condition=IfCondition(agent_mode),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_mutator", default_value="true"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_monitor", default_value="true"),
            DeclareLaunchArgument("agent_mode", default_value="true"),
            baseline,
            agent,
        ]
    )
