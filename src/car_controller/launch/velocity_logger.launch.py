import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("car_controller"),
        "config",
        "velocity_logger.yaml",
    )

    return LaunchDescription([
        Node(
            package="car_controller",
            executable="velocity_logger",
            name="velocity_logger_node",
            output="screen",
            parameters=[config_file],
        )
    ])
