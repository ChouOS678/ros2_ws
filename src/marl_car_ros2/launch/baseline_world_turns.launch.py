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
                    "world_file": os.path.join(pkg_share, "worlds", "baseline_sharp_turns.world"),
                    "spawn_x": "0.0",
                    "spawn_y": "0.0",
                    "spawn_z": "0.1",
                    "spawn_yaw": "0.0",
                    "goal_x": "3.8",
                    "goal_y": "0.2",
                    "agent_mode": "true",
                    "start_mutator": "false",
                }.items(),
            )
        ]
    )
