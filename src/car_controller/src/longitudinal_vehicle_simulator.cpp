// Copyright 2026 realisticCar project
//
// Longitudinal Vehicle Simulator node.
//
// Subscribes:
//   /control/trajectory_follower/longitudinal_cmd  (car_msgs/Longitudinal)
//
// Publishes:
//   /simulation/ground_truth/odometry   (nav_msgs/Odometry)  – true state, NO noise
//   /localization/kinematic_state       (nav_msgs/Odometry)  – measured state WITH noise
//   /localization/acceleration          (std_msgs/Float64)   – applied acceleration
//   /simulation/gazebo_cmd_vel          (geometry_msgs/Twist)– bridged to Gazebo /cmd_vel
//   /simulation/vehicle_dynamics/debug  (std_msgs/Float64MultiArray)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <string>

#include "car_msgs/msg/longitudinal.hpp"
#include "car_controller/longitudinal_vehicle_model.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class LongitudinalVehicleSimulator : public rclcpp::Node
{
public:
  LongitudinalVehicleSimulator()
  : Node("longitudinal_vehicle_simulator_node"),
    latest_cmd_accel_(0.0),
    has_cmd_(false)
  {
    // ---- Params ----
    car_controller::LongitudinalVehicleModel::Params model_params;
    model_params.actuator_delay_s = declare_parameter<double>("actuator_delay_s", 0.1);
    model_params.actuator_lag_tau_s = declare_parameter<double>("actuator_lag_tau_s", 0.15);
    model_params.max_accel = declare_parameter<double>("max_accel", 3.0);
    model_params.min_accel = declare_parameter<double>("min_accel", -3.0);
    model_params.max_jerk = declare_parameter<double>("max_jerk", 5.0);
    model_params.min_jerk = declare_parameter<double>("min_jerk", -5.0);
    model_params.grade_percent = declare_parameter<double>("grade_percent", 0.0);
    model_params.rolling_resistance_coeff =
      declare_parameter<double>("rolling_resistance_coeff", 0.01);
    model_params.drag_coeff = declare_parameter<double>("drag_coeff", 0.35575);
    model_params.mass_kg = declare_parameter<double>("mass_kg", 1423.0);
    model_params.process_noise_sigma = declare_parameter<double>("process_noise_sigma", 0.0);
    model_params.random_seed = static_cast<uint64_t>(
      declare_parameter<int>("random_seed", 42));
    model_params.gravity = declare_parameter<double>("gravity", 9.81);

    velocity_noise_sigma_ = declare_parameter<double>("velocity_noise_sigma", 0.0);
    step_rate_hz_ = declare_parameter<double>("step_rate_hz", 50.0);
    step_rate_hz_ = std::max(1.0, step_rate_hz_);
    dt_ = 1.0 / step_rate_hz_;

    // Sensor noise RNG (separate seed so it doesn't alias process noise).
    sensor_rng_.seed(model_params.random_seed + 999999ULL);

    model_ = std::make_unique<car_controller::LongitudinalVehicleModel>(model_params);

    // ---- Topic names ----
    const std::string cmd_topic = declare_parameter<std::string>(
      "longitudinal_cmd_topic", "/control/trajectory_follower/longitudinal_cmd");
    const std::string gt_odom_topic = declare_parameter<std::string>(
      "ground_truth_odom_topic", "/simulation/ground_truth/odometry");
    const std::string kin_state_topic = declare_parameter<std::string>(
      "kinematic_state_topic", "/localization/kinematic_state");
    const std::string accel_topic = declare_parameter<std::string>(
      "acceleration_topic", "/localization/acceleration");
    const std::string gazebo_vel_topic = declare_parameter<std::string>(
      "gazebo_cmd_vel_topic", "/simulation/gazebo_cmd_vel");
    const std::string debug_topic = declare_parameter<std::string>(
      "debug_topic", "/simulation/vehicle_dynamics/debug");

    cmd_sub_ = create_subscription<car_msgs::msg::Longitudinal>(
      cmd_topic, 10,
      [this](const car_msgs::msg::Longitudinal::SharedPtr msg) {
        latest_cmd_accel_ = static_cast<double>(msg->acceleration);
        has_cmd_ = true;
      });

    gt_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(gt_odom_topic, 10);
    kin_state_pub_ = create_publisher<nav_msgs::msg::Odometry>(kin_state_topic, 10);
    accel_pub_ = create_publisher<std_msgs::msg::Float64>(accel_topic, 10);
    gazebo_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(gazebo_vel_topic, 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(debug_topic, 10);

    step_timer_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(dt_))),
      std::bind(&LongitudinalVehicleSimulator::stepLoop, this));

    RCLCPP_INFO(
      get_logger(),
      "Longitudinal vehicle simulator started (dt=%.3f s, grade=%.1f%%, "
      "delay=%.2f s, lag=%.2f s)",
      dt_,
      model_->params().grade_percent,
      model_->params().actuator_delay_s,
      model_->params().actuator_lag_tau_s);
  }

private:
  void stepLoop()
  {
    const auto now = get_clock()->now();
    if (now.nanoseconds() == 0) {return;}

    // Use zero command until first message arrives.
    const double cmd = has_cmd_ ? latest_cmd_accel_ : 0.0;

    model_->step(cmd, dt_, model_->params().grade_percent);

    const double v_true = model_->true_velocity();
    const double a_applied = model_->applied_acceleration();

    // ---- Ground truth odometry (NO noise) ----
    nav_msgs::msg::Odometry gt_msg;
    gt_msg.header.stamp = now;
    gt_msg.header.frame_id = "odom";
    gt_msg.child_frame_id = "base_link";
    gt_msg.twist.twist.linear.x = v_true;
    gt_odom_pub_->publish(gt_msg);

    // ---- Measured / kinematic state (add sensor noise) ----
    double v_measured = v_true;
    if (velocity_noise_sigma_ > 0.0) {
      std::normal_distribution<double> dist(0.0, velocity_noise_sigma_);
      v_measured += dist(sensor_rng_);
    }
    nav_msgs::msg::Odometry kin_msg = gt_msg;
    kin_msg.twist.twist.linear.x = v_measured;
    kin_state_pub_->publish(kin_msg);

    // ---- Applied acceleration ----
    std_msgs::msg::Float64 accel_msg;
    accel_msg.data = a_applied;
    accel_pub_->publish(accel_msg);

    // ---- Gazebo visualization cmd_vel (use true velocity) ----
    geometry_msgs::msg::Twist gz_vel_msg;
    gz_vel_msg.linear.x = v_true;
    gz_vel_msg.angular.z = 0.0;
    gazebo_vel_pub_->publish(gz_vel_msg);

    // ---- Debug ----
    // [0] v_true, [1] v_measured, [2] a_applied, [3] a_grade,
    // [4] a_rolling, [5] a_drag, [6] a_noise, [7] a_total,
    // [8] delayed_cmd, [9] grade_percent
    std_msgs::msg::Float64MultiArray dbg_msg;
    dbg_msg.data = {
      v_true,
      v_measured,
      a_applied,
      model_->last_a_grade(),
      model_->last_a_rolling(),
      model_->last_a_drag(),
      model_->last_a_noise(),
      model_->last_a_total(),
      model_->last_delayed_cmd(),
      model_->params().grade_percent,
    };
    debug_pub_->publish(dbg_msg);
  }

  rclcpp::Subscription<car_msgs::msg::Longitudinal>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kin_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr accel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr gazebo_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr step_timer_;

  std::unique_ptr<car_controller::LongitudinalVehicleModel> model_;
  std::mt19937_64 sensor_rng_;

  double latest_cmd_accel_;
  double velocity_noise_sigma_{0.0};
  double step_rate_hz_{50.0};
  double dt_{0.02};
  bool has_cmd_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LongitudinalVehicleSimulator>());
  rclcpp::shutdown();
  return 0;
}
