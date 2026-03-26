from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import os


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_mutator = LaunchConfiguration("start_mutator")
    start_bridge = LaunchConfiguration("start_bridge")
    start_monitor = LaunchConfiguration("start_monitor")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_nav_tf_bridge = LaunchConfiguration("start_nav_tf_bridge")
    start_nav_executor = LaunchConfiguration("start_nav_executor")
    start_task_agent = LaunchConfiguration("start_task_agent")
    start_supervisor = LaunchConfiguration("start_supervisor")
    start_goal_sender = LaunchConfiguration("start_goal_sender")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    controller_profile = LaunchConfiguration("controller_profile")
    benchmark_mode = LaunchConfiguration("benchmark_mode")
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
    start_nav2_default = "true"
    nav2_launch_path = None
    missing_pkgs_info = []
    try:
        nav2_launch_path = os.path.join(
            get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py"
        )
    except PackageNotFoundError:
        start_nav2_default = "false"
        missing_pkgs_info.append(
            LogInfo(msg="Package 'nav2_bringup' not found: start_nav2 forced to false.")
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

    nav2_group = None
    if nav2_launch_path is not None:
        nav2_group = TimerAction(
            period=2.5,
            actions=[
                GroupAction(
                    actions=[
                        SetRemap(src="/cmd_vel", dst="/cmd_vel_nav"),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(nav2_launch_path),
                            launch_arguments={
                                "use_sim_time": use_sim_time,
                                "autostart": autostart,
                                "params_file": params_file,
                            }.items(),
                        ),
                    ],
                    condition=IfCondition(start_nav2),
                )
            ],
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
                "goal_x": goal_x,
                "goal_y": goal_y,
            }
        ],
        condition=IfCondition(start_nav_executor),
    )

    nav_tf_bridge = Node(
        package="marl_car_ros2",
        executable="nav_tf_bridge",
        name="nav_tf_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "publish_laser_tf": True,
                "use_msg_frame_ids": False,
                "use_msg_stamp": False,
            }
        ],
        condition=IfCondition(start_nav_tf_bridge),
    )

    task_agent = Node(
        package="marl_car_ros2",
        executable="task_agent",
        name="task_agent",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "goal_x": goal_x, "goal_y": goal_y}],
        condition=IfCondition(start_task_agent),
    )

    supervisor = Node(
        package="marl_car_ros2",
        executable="supervisor_node",
        name="supervisor_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "goal_x": goal_x,
                "goal_y": goal_y,
                "benchmark_mode": benchmark_mode,
            }
        ],
        condition=IfCondition(start_supervisor),
    )

    goal_sender = Node(
        package="marl_car_ros2",
        executable="nav_goal_sender",
        name="nav_goal_sender",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "goal_x": goal_x,
                "goal_y": goal_y,
                "goal_frame": "odom",
                "startup_delay_s": 4.0,
                "controller_id": controller_profile,
            }
        ],
        condition=IfCondition(start_goal_sender),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_mutator", default_value="true"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_monitor", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value=start_nav2_default),
            DeclareLaunchArgument("start_nav_tf_bridge", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "world_file",
                default_value=os.path.join(pkg_share, "worlds", "minimal.world"),
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.0"),
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
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument("controller_profile", default_value="FollowPath"),
            DeclareLaunchArgument("benchmark_mode", default_value="false"),
            DeclareLaunchArgument("start_nav_executor", default_value="false"),
            DeclareLaunchArgument("start_task_agent", default_value="true"),
            DeclareLaunchArgument("start_supervisor", default_value="true"),
            DeclareLaunchArgument("start_goal_sender", default_value="true"),
            sim,
            *missing_pkgs_info,
            *( [nav2_group] if nav2_group is not None else [] ),
            nav_tf_bridge,
            nav_executor,
            task_agent,
            supervisor,
            goal_sender,
        ]
    )
