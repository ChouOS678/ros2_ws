from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
import os


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_bridge = LaunchConfiguration("start_bridge")
    start_monitor = LaunchConfiguration("start_monitor")
    start_mutator = LaunchConfiguration("start_mutator")
    start_slam = LaunchConfiguration("start_slam")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_rviz = LaunchConfiguration("start_rviz")
    start_robot_state_publisher = LaunchConfiguration("start_robot_state_publisher")
    start_supervisor_markers = LaunchConfiguration("start_supervisor_markers")
    start_risk_markers = LaunchConfiguration("start_risk_markers")
    start_world_event_markers = LaunchConfiguration("start_world_event_markers")
    use_agent_layer = LaunchConfiguration("use_agent_layer")
    start_task_agent = LaunchConfiguration("start_task_agent")
    start_supervisor = LaunchConfiguration("start_supervisor")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    world_file = LaunchConfiguration("world_file")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    goal_x = LaunchConfiguration("goal_x")
    goal_y = LaunchConfiguration("goal_y")
    start_nav2_and_not_agent = PythonExpression(
        ["'", start_nav2, "' == 'true' and '", use_agent_layer, "' != 'true'"]
    )
    start_nav2_and_agent = PythonExpression(
        ["'", start_nav2, "' == 'true' and '", use_agent_layer, "' == 'true'"]
    )
    agent_layer_and_task = PythonExpression(
        ["'", use_agent_layer, "' == 'true' and '", start_task_agent, "' == 'true'"]
    )
    agent_layer_and_supervisor = PythonExpression(
        ["'", use_agent_layer, "' == 'true' and '", start_supervisor, "' == 'true'"]
    )

    pkg_share = get_package_share_directory("marl_car_ros2")
    launch_dir = os.path.join(pkg_share, "launch")
    start_slam_default = "true"
    start_nav2_default = "true"
    slam_launch_path = None
    nav2_launch_path = None
    missing_pkgs_info = []

    try:
        slam_launch_path = os.path.join(
            get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py"
        )
    except PackageNotFoundError:
        start_slam_default = "false"
        missing_pkgs_info.append(
            LogInfo(msg="Package 'slam_toolbox' not found: start_slam forced to false.")
        )

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
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "sim.launch.py")),
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
        }.items(),
    )

    nav_tf_bridge = Node(
        package="marl_car_ros2",
        executable="nav_tf_bridge",
        name="nav_tf_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "publish_laser_tf": False}],
    )

    robot_description_xml = ""
    robot_description_path = os.path.join(pkg_share, "urdf", "simple_marl_car.urdf")
    if os.path.exists(robot_description_path):
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
        condition=IfCondition(start_robot_state_publisher),
    )

    slam = None
    if slam_launch_path is not None:
        slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                "use_sim_time": use_sim_time,
            }.items(),
            condition=IfCondition(start_slam),
        )

    nav2_direct = None
    nav2_agent = None
    if nav2_launch_path is not None:
        nav2_direct = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
            }.items(),
            condition=IfCondition(start_nav2_and_not_agent),
        )

        nav2_agent = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "params_file": params_file,
            }.items(),
            condition=IfCondition(start_nav2_and_agent),
        )

    remap_cmd_vel_for_agent = SetRemap(
        src="/cmd_vel",
        dst="/cmd_vel_nav",
        condition=IfCondition(start_nav2_and_agent),
    )

    task_agent = Node(
        package="marl_car_ros2",
        executable="task_agent",
        name="task_agent",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "goal_x": goal_x, "goal_y": goal_y}],
        condition=IfCondition(agent_layer_and_task),
    )

    supervisor = Node(
        package="marl_car_ros2",
        executable="supervisor_node",
        name="supervisor_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "goal_x": goal_x, "goal_y": goal_y}],
        condition=IfCondition(agent_layer_and_supervisor),
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

    supervisor_markers = Node(
        package="marl_car_ros2",
        executable="supervisor_status_marker",
        name="supervisor_status_marker",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "frame_id": "map"}],
        condition=IfCondition(start_supervisor_markers),
    )

    risk_markers = Node(
        package="marl_car_ros2",
        executable="risk_marker",
        name="risk_marker",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "frame_id": "map"}],
        condition=IfCondition(start_risk_markers),
    )

    world_event_markers = Node(
        package="marl_car_ros2",
        executable="world_event_marker",
        name="world_event_marker",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "frame_id": "map"}],
        condition=IfCondition(start_world_event_markers),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_monitor", default_value="true"),
            DeclareLaunchArgument("start_mutator", default_value="false"),
            DeclareLaunchArgument("start_slam", default_value=start_slam_default),
            DeclareLaunchArgument("start_nav2", default_value=start_nav2_default),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_robot_state_publisher", default_value="true"),
            DeclareLaunchArgument("start_supervisor_markers", default_value="true"),
            DeclareLaunchArgument("start_risk_markers", default_value="true"),
            DeclareLaunchArgument("start_world_event_markers", default_value="true"),
            DeclareLaunchArgument("use_agent_layer", default_value="false"),
            DeclareLaunchArgument("start_task_agent", default_value="true"),
            DeclareLaunchArgument("start_supervisor", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "world_file",
                default_value=os.path.join(pkg_share, "worlds", "minimal.world"),
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.1"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("goal_x", default_value="8.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(pkg_share, "rviz", "nav2_default.rviz"),
            ),
            sim,
            nav_tf_bridge,
            robot_state_publisher,
            *missing_pkgs_info,
            *( [slam] if slam is not None else [] ),
            remap_cmd_vel_for_agent,
            *( [nav2_direct] if nav2_direct is not None else [] ),
            *( [nav2_agent] if nav2_agent is not None else [] ),
            task_agent,
            supervisor,
            supervisor_markers,
            risk_markers,
            world_event_markers,
            rviz,
        ]
    )
