from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

import os


def generate_launch_description():
    # --- Launch arguments ---
    start_gazebo = LaunchConfiguration("start_gazebo")
    gazebo_world = LaunchConfiguration("gazebo_world")
    controller = LaunchConfiguration("controller")
    vehicle_model = LaunchConfiguration("vehicle_model")
    scenario = LaunchConfiguration("scenario")
    scenario_file = LaunchConfiguration("scenario_file")
    random_seed = LaunchConfiguration("random_seed")
    planning_start_delay = LaunchConfiguration("planning_start_delay")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    gz_odom_topic = LaunchConfiguration("gz_odom_topic")
    ros_odom_topic = LaunchConfiguration("ros_odom_topic")

    car_controller_share = get_package_share_directory("car_controller")
    car_simulation_share = get_package_share_directory("car_simulation")

    gazebo_effort_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_simulation_share, "launch", "gazebo_effort.launch.py")
        ),
        condition=IfCondition(
            PythonExpression(["'", vehicle_model, "' == 'gazebo_effort'"])
        ),
        launch_arguments={
            "controller": controller,
            "scenario": scenario,
            "scenario_file": scenario_file,
            "random_seed": random_seed,
            "planning_start_delay": planning_start_delay,
            "gazebo_gui": gazebo_gui,
        }.items(),
    )

    # ----------------------------------------------------------------
    # Planning node
    # ----------------------------------------------------------------
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("car_planning"),
                "launch",
                "trapezoid_velocity_profile.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "scenario": scenario,
            "random_seed": random_seed,
        }.items(),
    )

    # ----------------------------------------------------------------
    # Longitudinal vehicle simulator (new plant, default mode)
    # ----------------------------------------------------------------
    vehicle_simulator_node = Node(
        package="car_controller",
        executable="longitudinal_vehicle_simulator",
        name="longitudinal_vehicle_simulator_node",
        output="screen",
        parameters=[
            scenario_file,
            {"use_sim_time": True, "random_seed": random_seed},
        ],
        condition=IfCondition(
            PythonExpression(["'", vehicle_model, "' == 'longitudinal_sim'"])
        ),
    )

    # ----------------------------------------------------------------
    # Controllers
    # ----------------------------------------------------------------
    fuzzy_pid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_controller_share, "launch", "fuzzy_pid_controller.launch.py")
        ),
        condition=IfCondition(PythonExpression(["'", controller, "' == 'fuzzy_pid'"])),
        launch_arguments={"use_sim_time": "true", "scenario": scenario}.items(),
    )

    pid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(car_controller_share, "launch", "pid_velocity_controller.launch.py")
        ),
        condition=IfCondition(PythonExpression(["'", controller, "' == 'pid'"])),
        launch_arguments={
            "use_sim_time": "true",
            "scenario": scenario,
            "random_seed": random_seed,
        }.items(),
    )

    # (Velocity logger is launched by the individual controller launch files)
    # ----------------------------------------------------------------
    # Gazebo bridge
    # ----------------------------------------------------------------
    # In longitudinal_sim mode: bridge /simulation/gazebo_cmd_vel -> /cmd_vel in Gazebo.
    # In ideal_diff_drive mode: controllers publish /cmd_vel directly (legacy).
    gazebo_bridge_longitudinal_sim = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="prius_sonoma_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Gazebo odometry bridged for diagnostics only.
            [gz_odom_topic, "@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
            # Plant velocity -> Gazebo visualization.
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        remappings=[
            # Gazebo odometry -> diagnostic topic (NOT used as controller feedback).
            (gz_odom_topic, "/simulation/gazebo_odometry"),
            # Plant cmd_vel -> Gazebo /cmd_vel (visualization only).
            ("/cmd_vel", "/simulation/gazebo_cmd_vel"),
        ],
        condition=IfCondition(
            PythonExpression(["'", vehicle_model, "' == 'longitudinal_sim'"])
        ),
    )

    # Legacy bridge for ideal_diff_drive mode (controllers publish /cmd_vel directly).
    gazebo_bridge_ideal = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="prius_sonoma_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            [gz_odom_topic, "@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        remappings=[
            (gz_odom_topic, ros_odom_topic),
        ],
        condition=IfCondition(
            PythonExpression(["'", vehicle_model, "' == 'ideal_diff_drive'"])
        ),
    )

    # ----------------------------------------------------------------
    # Gazebo process
    # ----------------------------------------------------------------
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", gazebo_world],
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'", start_gazebo, "' == 'true' and '", vehicle_model,
                "' != 'gazebo_effort'",
            ])
        ),
    )

    stop_stack_when_gazebo_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo,
            on_exit=[
                LogInfo(msg="Gazebo exited; stopping all nodes"),
                EmitEvent(event=Shutdown(reason="Gazebo process exited")),
            ],
        )
    )

    wait_for_gazebo = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            (
                'until gz topic -l 2>/dev/null | grep -Fxq -- "$1"; '
                "do sleep 0.5; done"
            ),
            "wait_for_prius_odometry",
            gz_odom_topic,
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", vehicle_model, "' != 'gazebo_effort'"])
        ),
    )

    start_control_when_gazebo_is_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_gazebo,
            on_exit=[
                LogInfo(msg=["Gazebo Prius ready; starting plant, controllers and planning"]),
                # Plant must start before controllers so /localization/kinematic_state exists.
                vehicle_simulator_node,
                fuzzy_pid_launch,
                pid_launch,
                LogInfo(
                    msg=["Delaying planning for ", planning_start_delay, " seconds"]
                ),
                TimerAction(
                    period=planning_start_delay,
                    actions=[planning_launch],
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            # --- Arguments ---
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="true",
                description="Start Gazebo Sim with the configured Prius world",
            ),
            DeclareLaunchArgument(
                "controller",
                default_value="fuzzy_pid",
                choices=["fuzzy_pid", "pid"],
                description="Velocity controller to run after Gazebo is ready",
            ),
            DeclareLaunchArgument(
                "vehicle_model",
                default_value="gazebo_effort",
                choices=["longitudinal_sim", "gazebo_effort"],
                description=(
                    "gazebo_effort: Gazebo Sim 8 rear-wheel torque backend (default); "
                    "longitudinal_sim: deterministic 1D baseline"
                ),
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value="baseline",
                description="Scenario label recorded in controller CSV logs",
            ),
            DeclareLaunchArgument(
                "scenario_file",
                default_value=os.path.join(
                    car_controller_share,
                    "config",
                    "simulation_scenarios",
                    "baseline.yaml",
                ),
                description="Path to scenario YAML passed to the vehicle simulator",
            ),
            DeclareLaunchArgument(
                "random_seed",
                default_value="1001",
                description="RNG seed for vehicle simulator noise (overrides scenario_file value)",
            ),
            DeclareLaunchArgument(
                "planning_start_delay",
                default_value="3.0",
                description="Seconds to wait after Gazebo is ready before starting planning",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                choices=["true", "false"],
                description="Open Gazebo GUI; set false for batch/headless runs",
            ),
            DeclareLaunchArgument(
                "gazebo_world",
                default_value=os.path.join(
                    car_simulation_share, "worlds", "prius_effort_spike.sdf"
                ),
                description="Path to the Gazebo world containing the Prius",
            ),
            DeclareLaunchArgument(
                "gz_odom_topic",
                default_value="/model/prius_hybrid/odometry",
                description="Gazebo Transport odometry topic published by the Prius model",
            ),
            DeclareLaunchArgument(
                "ros_odom_topic",
                default_value="/odom",
                description="ROS 2 odometry topic (used in ideal_diff_drive mode only)",
            ),

            # --- Nodes ---
            stop_stack_when_gazebo_exits,
            gazebo,
            gazebo_bridge_longitudinal_sim,
            gazebo_bridge_ideal,
            start_control_when_gazebo_is_ready,
            wait_for_gazebo,
            gazebo_effort_launch,
        ]
    )
