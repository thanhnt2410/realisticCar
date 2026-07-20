"""Launch the Gazebo Sim 8 rear-wheel-effort vehicle backend."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory("car_simulation")
    controller_share = get_package_share_directory("car_controller")
    description_share = get_package_share_directory("car_description")
    prefix = get_package_prefix("car_controller")
    controller = LaunchConfiguration("controller")
    scenario = LaunchConfiguration("scenario")
    scenario_file = LaunchConfiguration("scenario_file")
    random_seed = LaunchConfiguration("random_seed")
    planning_start_delay = LaunchConfiguration("planning_start_delay")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    world = os.path.join(simulation_share, "worlds", "prius_effort_spike.sdf")
    vehicle_urdf = os.path.join(description_share, "urdf", "prius_gz.urdf")
    gazebo_env = {
        "GZ_SIM_RESOURCE_PATH": os.path.dirname(description_share),
        "GZ_SIM_SYSTEM_PLUGIN_PATH": os.path.join(prefix, "lib"),
    }
    gazebo_with_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen",
        condition=IfCondition(gazebo_gui),
        additional_env=gazebo_env,
    )
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world],
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", gazebo_gui, "' != 'true'"])
        ),
        additional_env=gazebo_env,
    )
    entity_request = (
        f'sdf_filename: "{vehicle_urdf}", name: "car", '
        "pose: {position: {z: 0.03}, orientation: "
        "{z: 0.7071067811865475, w: 0.7071067811865476}}"
    )
    spawn_vehicle = ExecuteProcess(
        cmd=[
            "gz", "service",
            "-s", "/world/prius_effort_spike/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "30000",
            "--req", entity_request,
        ],
        additional_env=gazebo_env,
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gazebo_effort_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/car/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/rear_wheel_effort/left@std_msgs/msg/Float64]gz.msgs.Double",
            "/rear_wheel_effort/right@std_msgs/msg/Float64]gz.msgs.Double",
        ],
        remappings=[
            ("/model/car/odometry", "/simulation/gazebo_odometry"),
        ],
        output="screen",
    )
    adapter = Node(
        package="car_controller",
        executable="odometry_adapter",
        parameters=[
            os.path.join(controller_share, "config", "odometry_adapter.yaml"),
            scenario_file,
            {"random_seed": random_seed},
        ],
        output="screen",
    )
    interface = Node(
        package="car_controller",
        executable="gazebo_vehicle_interface",
        parameters=[
            os.path.join(controller_share, "config", "gazebo_vehicle_interface.yaml")
        ],
        output="screen",
    )
    fuzzy_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                controller_share, "launch", "fuzzy_pid_controller.launch.py"
            )
        ),
        condition=IfCondition(PythonExpression(["'", controller, "' == 'fuzzy_pid'"])),
        launch_arguments={
            "use_sim_time": "true",
            "vehicle_backend": "gazebo_effort",
            "scenario": scenario,
            "random_seed": random_seed,
        }.items(),
    )
    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                controller_share, "launch", "pid_velocity_controller.launch.py"
            )
        ),
        condition=IfCondition(PythonExpression(["'", controller, "' == 'pid'"])),
        launch_arguments={
            "use_sim_time": "true",
            "vehicle_backend": "gazebo_effort",
            "scenario": scenario,
            "random_seed": random_seed,
        }.items(),
    )
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("car_planning"),
                "launch",
                "trapezoid_velocity_profile.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    wait_for_odometry = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "until gz topic -l 2>/dev/null | grep -Fxq "
            "'/model/car/odometry'; do sleep 0.2; done",
        ]
    )
    start_pipeline = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_odometry,
            on_exit=[
                adapter,
                interface,
                fuzzy_controller,
                pid_controller,
                TimerAction(period=planning_start_delay, actions=[planner]),
            ],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="fuzzy_pid"),
        DeclareLaunchArgument("scenario", default_value="baseline"),
        DeclareLaunchArgument(
            "scenario_file",
            default_value=os.path.join(
                controller_share,
                "config",
                "simulation_scenarios",
                "baseline.yaml",
            ),
        ),
        DeclareLaunchArgument("random_seed", default_value="1001"),
        DeclareLaunchArgument("planning_start_delay", default_value="3.0"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        gazebo_with_gui,
        gazebo_headless,
        spawn_vehicle,
        bridge,
        wait_for_odometry,
        start_pipeline,
    ])
