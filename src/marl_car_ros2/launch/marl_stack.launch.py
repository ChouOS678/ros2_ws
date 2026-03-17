from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_gazebo = LaunchConfiguration("start_gazebo")

    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose"],
        output="screen",
        condition=IfCondition(start_gazebo),
    )

    world_model_mutator = Node(
        package="marl_car_ros2",
        executable="world_model_mutator",
        name="world_model_mutator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    multi_agent_game = Node(
        package="marl_car_ros2",
        executable="multi_agent_game",
        name="multi_agent_game",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    monitor_logger = Node(
        package="marl_car_ros2",
        executable="monitor_logger",
        name="monitor_logger",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_gazebo", default_value="false"),
            gazebo,
            world_model_mutator,
            multi_agent_game,
            monitor_logger,
        ]
    )
