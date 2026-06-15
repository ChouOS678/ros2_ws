import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from ament_index_python.packages import get_package_share_directory
from marl_car_ros2.cpu_affinity import CpuAffinityPrefix
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("marl_car_ros2")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    container_name = LaunchConfiguration("container_name")
    container_name_full = (namespace, "/", container_name)
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    use_cpu_affinity = LaunchConfiguration("use_cpu_affinity")

    # Route server is intentionally excluded from the formal benchmark bringup.
    # Split into critical (must succeed) and optional (best-effort) for
    # the fault-tolerant lifecycle manager.
    _all_lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
        "waypoint_follower",
        "docking_server",
    ]
    critical_lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]
    optional_lifecycle_nodes = [
        "smoother_server",
        "velocity_smoother",
        "collision_monitor",
        "waypoint_follower",
        "docking_server",
    ]

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={"autostart": autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation clock if true"
    )
    declare_params_file_cmd = DeclareLaunchArgument("params_file", default_value="", description="Nav2 params file")
    declare_autostart_cmd = DeclareLaunchArgument("autostart", default_value="true")
    declare_use_composition_cmd = DeclareLaunchArgument("use_composition", default_value="False")
    declare_container_name_cmd = DeclareLaunchArgument("container_name", default_value="nav2_container")
    declare_use_respawn_cmd = DeclareLaunchArgument("use_respawn", default_value="False")
    declare_log_level_cmd = DeclareLaunchArgument("log_level", default_value="info")
    declare_use_cpu_affinity_cmd = DeclareLaunchArgument("use_cpu_affinity", default_value="false")

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["not ", use_composition])),
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                prefix=CpuAffinityPrefix("controller_server", TextSubstitution(text=pkg_share), use_cpu_affinity),
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                prefix=CpuAffinityPrefix("smoother_server", TextSubstitution(text=pkg_share), use_cpu_affinity),
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                prefix=CpuAffinityPrefix("planner_server", TextSubstitution(text=pkg_share), use_cpu_affinity),
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                prefix=CpuAffinityPrefix("bt_navigator", TextSubstitution(text=pkg_share), use_cpu_affinity),
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                prefix=CpuAffinityPrefix("velocity_smoother", TextSubstitution(text=pkg_share), use_cpu_affinity),
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                prefix=CpuAffinityPrefix("collision_monitor", TextSubstitution(text=pkg_share), use_cpu_affinity),
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="opennav_docking",
                executable="opennav_docking",
                name="docking_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="marl_car_ros2",
                executable="fault_tolerant_lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[{
                    "autostart": autostart,
                    "critical_nodes": critical_lifecycle_nodes,
                    "optional_nodes": optional_lifecycle_nodes,
                    "service_timeout_s": 8.0,
                    "retry_count": 2,
                    "startup_delay_s": 3.0,
                    "node_ready_timeout_s": 30.0,
                }],
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package="nav2_controller",
                        plugin="nav2_controller::ControllerServer",
                        name="controller_server",
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ComposableNode(
                        package="nav2_smoother",
                        plugin="nav2_smoother::SmootherServer",
                        name="smoother_server",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_planner",
                        plugin="nav2_planner::PlannerServer",
                        name="planner_server",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_behaviors",
                        plugin="behavior_server::BehaviorServer",
                        name="behavior_server",
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ComposableNode(
                        package="nav2_bt_navigator",
                        plugin="nav2_bt_navigator::BtNavigator",
                        name="bt_navigator",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_waypoint_follower",
                        plugin="nav2_waypoint_follower::WaypointFollower",
                        name="waypoint_follower",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_velocity_smoother",
                        plugin="nav2_velocity_smoother::VelocitySmoother",
                        name="velocity_smoother",
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                    ),
                    ComposableNode(
                        package="nav2_collision_monitor",
                        plugin="nav2_collision_monitor::CollisionMonitor",
                        name="collision_monitor",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="opennav_docking",
                        plugin="opennav_docking::DockingServer",
                        name="docking_server",
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package="nav2_lifecycle_manager",
                        plugin="nav2_lifecycle_manager::LifecycleManager",
                        name="lifecycle_manager_navigation",
                        parameters=[{"autostart": autostart, "node_names": _all_lifecycle_nodes}],
                    ),
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            stdout_linebuf_envvar,
            declare_namespace_cmd,
            declare_use_sim_time_cmd,
            declare_params_file_cmd,
            declare_autostart_cmd,
            declare_use_composition_cmd,
            declare_container_name_cmd,
            declare_use_respawn_cmd,
            declare_log_level_cmd,
            declare_use_cpu_affinity_cmd,
            load_nodes,
            load_composable_nodes,
        ]
    )
