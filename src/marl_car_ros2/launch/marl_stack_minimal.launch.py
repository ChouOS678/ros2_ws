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
    start_game = LaunchConfiguration("start_game")
    start_agent_layer = LaunchConfiguration("start_agent_layer")
    start_nav_executor = LaunchConfiguration("start_nav_executor")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("marl_car_ros2"), "launch", "sim.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_gazebo": start_gazebo,
            "start_bridge": "true",
            "start_monitor": "true",
            "start_mutator": "false",
        }.items(),
    )

    # Legacy centralized game loop (kept for compatibility / old scripts).
    multi_agent_game = Node(
        package="marl_car_ros2",
        executable="multi_agent_game",
        name="multi_agent_game",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_game),
    )

    # Keep legacy mutator naming for compatibility with existing scripts/tools.
    world_model_mutator = Node(
        package="marl_car_ros2",
        executable="world_model_mutator",
        name="world_model_mutator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # New architecture path (task agent + supervisor).
    nav_executor = Node(
        package="marl_car_ros2",
        executable="baseline_nav_node",
        name="nav_executor_compat",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "output_topic": "/cmd_vel_nav"}],
        condition=IfCondition(start_nav_executor),
    )

    task_agent = Node(
        package="marl_car_ros2",
        executable="task_agent",
        name="task_agent",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_agent_layer),
    )

    supervisor = Node(
        package="marl_car_ros2",
        executable="supervisor_node",
        name="supervisor_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_agent_layer),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_game", default_value="false"),
            DeclareLaunchArgument("start_agent_layer", default_value="false"),
            DeclareLaunchArgument("start_nav_executor", default_value="false"),
            sim,
            world_model_mutator,
            multi_agent_game,
            nav_executor,
            task_agent,
            supervisor,
        ]
    )
