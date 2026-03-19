from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_bridge = LaunchConfiguration("start_bridge")
    start_monitor = LaunchConfiguration("start_monitor")
    start_mutator = LaunchConfiguration("start_mutator")

    pkg_share = get_package_share_directory("marl_car_ros2")
    world_path = os.path.join(pkg_share, "worlds", "minimal.world")
    model_path = os.path.join(pkg_share, "models", "simple_marl_car", "model.sdf")

    gazebo_backend_info = LogInfo(msg="Using Gazebo Sim backend via ros_gz_sim (Jazzy default).")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
        condition=IfCondition(start_gazebo),
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_spawn_model.launch.py")
        ),
        launch_arguments={
            "file": model_path,
            "entity_name": "simple_marl_car",
            "x": "0.0",
            "y": "0.0",
            "z": "0.1",
        }.items(),
        condition=IfCondition(start_gazebo),
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/simple_marl_car/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/simple_marl_car/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            ("/model/simple_marl_car/cmd_vel", "/cmd_vel"),
            ("/model/simple_marl_car/odometry", "/odom"),
        ],
        condition=IfCondition(start_bridge),
    )

    scenario_mutator = Node(
        package="marl_car_ros2",
        executable="scenario_mutator",
        name="scenario_mutator",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_mutator),
    )

    monitor_logger = Node(
        package="marl_car_ros2",
        executable="monitor_logger",
        name="monitor_logger",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(start_monitor),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_monitor", default_value="true"),
            DeclareLaunchArgument("start_mutator", default_value="true"),
            gazebo_backend_info,
            gazebo,
            TimerAction(period=2.0, actions=[spawn_entity]),
            ros_gz_bridge,
            scenario_mutator,
            monitor_logger,
        ]
    )
