from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("marl_car_ros2")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world_file")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    start_rviz = LaunchConfiguration("start_rviz")
    start_gui = LaunchConfiguration("start_gui")
    start_visualizer = LaunchConfiguration("start_visualizer")
    benchmark_mode = LaunchConfiguration("benchmark_mode")
    world_name = LaunchConfiguration("world_name")

    agent_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "agent_nav2.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world_file": world_file,
            "params_file": params_file,
            "benchmark_mode": benchmark_mode,
            "start_goal_sender": "false",
            "start_task_agent": "false",
            "start_supervisor": "true",
            "start_nav2": "true",
            "start_nav_executor": "false",
            "start_monitor": "true",
            "spawn_x": "0.0",
            "spawn_y": "0.0",
            "spawn_z": "0.0",
            "spawn_yaw": "0.0",
        }.items(),
    )

    robot_description_path = os.path.join(pkg_share, "urdf", "simple_marl_car.urdf")
    with open(robot_description_path, "r", encoding="utf-8") as f:
        robot_description_xml = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_xml,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_rviz),
    )

    visualizer = Node(
        package="marl_car_ros2",
        executable="benchmark_visualizer",
        name="benchmark_visualizer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "frame_id": "odom"}],
        condition=IfCondition(start_visualizer),
    )

    gui = Node(
        package="marl_car_ros2",
        executable="benchmark_gui",
        name="benchmark_gui",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "frame_id": "odom",
                "world_name": world_name,
                "warp_z": 0.0,
            }
        ],
        condition=IfCondition(start_gui),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "world_file",
                default_value=os.path.join(pkg_share, "worlds", "minimal.world"),
            ),
            DeclareLaunchArgument("world_name", default_value="minimal"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(pkg_share, "rviz", "benchmark.rviz"),
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_gui", default_value="true"),
            DeclareLaunchArgument("start_visualizer", default_value="true"),
            DeclareLaunchArgument("benchmark_mode", default_value="true"),
            agent_stack,
            robot_state_publisher,
            rviz,
            visualizer,
            gui,
        ]
    )
