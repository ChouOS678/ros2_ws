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
    start_scan_stamp_bridge = LaunchConfiguration("start_scan_stamp_bridge")
    start_monitor = LaunchConfiguration("start_monitor")
    start_mutator = LaunchConfiguration("start_mutator")
    world_file = LaunchConfiguration("world_file")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
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
    default_world_path = os.path.join(pkg_share, "worlds", "minimal.world")
    model_path = os.path.join(pkg_share, "models", "simple_marl_car", "model.sdf")

    gazebo_backend_info = LogInfo(msg="Using Gazebo Sim backend via ros_gz_sim (Jazzy default).")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r ", world_file]}.items(),
        condition=IfCondition(start_gazebo),
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_spawn_model.launch.py")
        ),
        launch_arguments={
            "file": model_path,
            "entity_name": "simple_marl_car",
            "x": spawn_x,
            "y": spawn_y,
            "z": spawn_z,
            "yaw": spawn_yaw,
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
            [dynamic_obstacle_cmd_topic, "@geometry_msgs/msg/Twist]gz.msgs.Twist"],
            "/model/simple_marl_car/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            ("/model/simple_marl_car/cmd_vel", "/cmd_vel"),
            ("/model/simple_marl_car/odometry", "/odom"),
        ],
        condition=IfCondition(start_bridge),
    )

    scan_stamp_bridge = Node(
        package="marl_car_ros2",
        executable="scan_stamp_bridge",
        name="scan_stamp_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/scan_raw",
                "output_topic": "/scan",
                "output_frame": "lidar_link",
                "restamp_with_now": True,
            }
        ],
        condition=IfCondition(start_scan_stamp_bridge),
    )

    scenario_mutator = Node(
        package="marl_car_ros2",
        executable="scenario_mutator",
        name="scenario_mutator",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
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
        ],
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
            DeclareLaunchArgument("start_scan_stamp_bridge", default_value="true"),
            DeclareLaunchArgument("start_monitor", default_value="true"),
            DeclareLaunchArgument("start_mutator", default_value="true"),
            DeclareLaunchArgument("world_file", default_value=default_world_path),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
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
            gazebo_backend_info,
            gazebo,
            TimerAction(period=2.0, actions=[spawn_entity]),
            ros_gz_bridge,
            scan_stamp_bridge,
            scenario_mutator,
            monitor_logger,
        ]
    )
