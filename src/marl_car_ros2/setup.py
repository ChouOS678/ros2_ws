from glob import glob

from setuptools import setup

package_name = "marl_car_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        (f"share/{package_name}/urdf", glob("urdf/*.urdf")),
        (f"share/{package_name}/worlds", glob("worlds/*.world")),
        (f"share/{package_name}/models/simple_marl_car", glob("models/simple_marl_car/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="AI-native node-as-agent MARL bridge for ROS2 car simulation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dummy_joint_runner = marl_car_ros2.dummy_joint_runner:main",
            "train_rllib = marl_car_ros2.train_rllib:main",
            "baseline_nav_node = marl_car_ros2.baseline_nav_node:main",
            "task_agent = marl_car_ros2.task_agent:main",
            "supervisor_node = marl_car_ros2.supervisor_node:main",
            "scenario_mutator = marl_car_ros2.scenario_mutator:main",
            "world_model_mutator = marl_car_ros2.world_model_mutator:main",
            "multi_agent_game = marl_car_ros2.multi_agent_game:main",
            "monitor_logger = marl_car_ros2.monitor_logger_node:main",
            "nav_tf_bridge = marl_car_ros2.nav_tf_bridge_node:main",
            "scan_stamp_bridge = marl_car_ros2.scan_stamp_bridge_node:main",
            "nav_goal_sender = marl_car_ros2.nav_goal_sender:main",
            "benchmark_gui = marl_car_ros2.benchmark_gui:main",
            "benchmark_visualizer = marl_car_ros2.benchmark_visualizer:main",
            "supervisor_status_marker = marl_car_ros2.supervisor_status_marker_node:main",
            "risk_marker = marl_car_ros2.risk_marker_node:main",
            "world_event_marker = marl_car_ros2.world_event_marker_node:main",
            "evaluation_metrics = marl_car_ros2.evaluation_metrics:main",
            "benchmark_runner = marl_car_ros2.benchmark_runner:main",
        ],
    },
)
