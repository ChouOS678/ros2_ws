from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("marl_car_ros2")
    launch_dir = os.path.join(pkg_share, "launch")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "evaluation.launch.py")),
                launch_arguments={
                    "world_file": os.path.join(pkg_share, "worlds", "baseline_dynamic_crossing.world"),
                    "spawn_x": "0.0",
                    "spawn_y": "0.0",
                    "spawn_z": "0.1",
                    "spawn_yaw": "0.0",
                    "goal_x": "8.0",
                    "goal_y": "0.0",
                    "agent_mode": "true",
                    "start_mutator": "true",
                    "enable_deterministic_obstacle": "true",
                    "dynamic_obstacle_mode": "crossing_deterministic",
                    "dynamic_obstacle_speed_mps": "0.45",
                    "dynamic_obstacle_trigger_mode": "time_after_start",
                    "dynamic_obstacle_trigger_time_s": "4.0",
                    "dynamic_obstacle_trigger_robot_x": "2.0",
                    "dynamic_obstacle_crossing_span_m": "1.6",
                    "dynamic_obstacle_initial_direction": "1.0",
                    "dynamic_obstacle_repeat": "true",
                }.items(),
            )
        ]
    )
