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
    start_nav_executor = LaunchConfiguration("start_nav_executor")
    start_task_agent = LaunchConfiguration("start_task_agent")
    start_supervisor = LaunchConfiguration("start_supervisor")

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
        }.items(),
    )

    nav_executor = Node(
        package="marl_car_ros2",
        executable="baseline_nav_node",
        name="nav_executor_compat",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "output_topic": "/cmd_vel_nav",
            }
        ],
        condition=IfCondition(start_nav_executor),
    )

    task_agent = Node(
        package="marl_car_ros2",
        executable="task_agent",
        name="task_agent",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_task_agent),
    )

    supervisor = Node(
        package="marl_car_ros2",
        executable="supervisor_node",
        name="supervisor_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_supervisor),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_mutator", default_value="true"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_monitor", default_value="true"),
            DeclareLaunchArgument("start_nav_executor", default_value="true"),
            DeclareLaunchArgument("start_task_agent", default_value="true"),
            DeclareLaunchArgument("start_supervisor", default_value="true"),
            sim,
            nav_executor,
            task_agent,
            supervisor,
        ]
    )
