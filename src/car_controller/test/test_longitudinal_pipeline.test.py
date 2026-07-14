import os
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory


@pytest.mark.launch_test
def generate_test_description():
    car_controller_share = get_package_share_directory("car_controller")

    scenario_file = os.path.join(
        car_controller_share, "config", "simulation_scenarios", "baseline.yaml"
    )

    # 1. Planning node
    planning_node = Node(
        package="car_planning",
        executable="trapezoid_velocity_profile",
        name="trapezoid_velocity_profile_node",
        parameters=[
            {"use_sim_time": False},
            {"publish_rate": 50.0},
            {"velocity_points": [0.0, 5.0, 5.0]},
            {"ramp_duration": 2.0},
            {"hold_duration": 2.0},
        ],
    )

    # 2. Controller node
    fuzzy_pid_config = os.path.join(
        car_controller_share, "config", "fuzzy_pid_controller.yaml"
    )
    fuzzy_pid_node = Node(
        package="car_controller",
        executable="fuzzy_pid_controller",
        name="fuzzy_pid_controller_node",
        parameters=[
            fuzzy_pid_config,
            {"use_sim_time": False},
        ],
    )

    # 3. Plant node
    vehicle_simulator_node = Node(
        package="car_controller",
        executable="longitudinal_vehicle_simulator",
        name="longitudinal_vehicle_simulator_node",
        parameters=[
            scenario_file,
            {"use_sim_time": False, "random_seed": 42},
        ],
    )

    return LaunchDescription(
        [
            planning_node,
            fuzzy_pid_node,
            vehicle_simulator_node,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestLongitudinalPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_longitudinal_pipeline_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_pipeline_accelerates(self):
        """Test that the pipeline successfully commands and accelerates the plant."""
        msgs = []

        def odom_callback(msg: Odometry):
            msgs.append(msg)

        self.node.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            odom_callback,
            10,
        )

        # Wait for the velocity to exceed 2.0 m/s (target is 5.0 m/s after 2s)
        timeout = 5.0
        start_time = time.time()
        success = False

        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if len(msgs) > 0:
                current_vel = msgs[-1].twist.twist.linear.x
                if current_vel > 2.0:
                    success = True
                    break

        self.assertTrue(success, "Vehicle did not accelerate past 2.0 m/s within timeout.")
