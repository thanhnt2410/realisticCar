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
    vehicle_backend = LaunchConfiguration("vehicle_backend")
    scenario = LaunchConfiguration("scenario")
    random_seed = LaunchConfiguration("random_seed")
    car_controller_share = get_package_share_directory("car_controller")
    workspace_root = get_workspace_root(car_controller_share)
    controller_config_file = os.path.join(
        car_controller_share,
        "config",
        "pid_controller.yaml",
    )
    velocity_logger_config_file = os.path.join(
        car_controller_share,
        "config",
        "velocity_logger_pid.yaml",
    )
    pid_velocity_parameters = load_node_parameters(
        controller_config_file,
        "pid_velocity_controller_node",
    )

    pid_velocity_controller_node = Node(
        package="car_controller",
        executable="pid_velocity_controller",
        name="pid_velocity_controller_node",
        output="screen",
        parameters=[pid_velocity_parameters, {"use_sim_time": use_sim_time}],
    )

    velocity_logger_node = Node(
        package="car_controller",
        executable="velocity_logger",
        name="velocity_logger_node",
        output="screen",
        parameters=[
            velocity_logger_config_file,
            {
                "log_file_path": str(workspace_root / "logs" / "pid_velocity_log.csv"),
                "use_sim_time": use_sim_time,
                "vehicle_backend": vehicle_backend,
                "scenario": scenario,
                "seed": random_seed,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "vehicle_backend",
            default_value="longitudinal_sim",
            description="Vehicle backend recorded in the CSV log",
        ),
        DeclareLaunchArgument(
            "scenario",
            default_value="baseline",
            description="Scenario label recorded in the CSV log",
        ),
        DeclareLaunchArgument(
            "random_seed",
            default_value="1001",
            description="Random seed recorded in the CSV log",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the ROS /clock topic instead of the system clock",
        ),
        pid_velocity_controller_node,
        velocity_logger_node
    ])
