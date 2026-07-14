from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
from pathlib import Path
import yaml


def get_workspace_root(package_share_dir):
    package_share_path = Path(package_share_dir).resolve()
    for parent in package_share_path.parents:
        if parent.name == "install":
            return parent.parent
    return package_share_path


def load_node_parameters(config_file, node_name):
    with open(config_file, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)
    parameters = dict(config["velocity_controller_common"]["ros__parameters"])
    parameters.update(config.get(node_name, {}).get("ros__parameters", {}))
    return parameters


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    car_controller_share = get_package_share_directory("car_controller")
    workspace_root = get_workspace_root(car_controller_share)
    fuzzy_pid_config_file = os.path.join(
        car_controller_share,
        "config",
        "fuzzy_pid_controller.yaml",
    )
    velocity_logger_config_file = os.path.join(
        car_controller_share,
        "config",
        "velocity_logger.yaml",
    )
    fuzzy_pid_parameters = load_node_parameters(
        fuzzy_pid_config_file,
        "fuzzy_pid_controller_node",
    )

    fuzzy_pid_controller_node = Node(
        package="car_controller",
        executable="fuzzy_pid_controller",
        name="fuzzy_pid_controller_node",
        output="screen",
        parameters=[fuzzy_pid_parameters, {"use_sim_time": use_sim_time}],
    )

    velocity_logger_node = Node(
        package="car_controller",
        executable="velocity_logger",
        name="velocity_logger_node",
        output="screen",
        parameters=[
            velocity_logger_config_file,
            {
                "log_file_path": str(workspace_root / "logs" / "fuzzy_pid_velocity_log.csv"),
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the ROS /clock topic instead of the system clock",
        ),
        fuzzy_pid_controller_node,
        velocity_logger_node
    ])
