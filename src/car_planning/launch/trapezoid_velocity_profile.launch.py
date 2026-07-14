import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = os.path.join(
        get_package_share_directory("car_planning"),
        "config",
        "trapezoid_velocity_profile.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the ROS /clock topic instead of the system clock",
        ),
        Node(
            package="car_planning",
            executable="trapezoid_velocity_profile",
            name="trapezoid_velocity_profile_node",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        )
    ])
