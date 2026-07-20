// Copyright 2026 realisticCar project
//
// PID velocity controller node – Autoware-style longitudinal control.
//
// Subscribes:
//   /planning/longitudinal_reference  (car_msgs/LongitudinalReference)
//   /localization/kinematic_state     (nav_msgs/Odometry – noisy/measured)
//
// Publishes:
//   /control/trajectory_follower/longitudinal_cmd         (car_msgs/Longitudinal)
//   /control/trajectory_follower/acceleration_correction  (std_msgs/Float64, debug)
//   /control/pid/debug                                    (std_msgs/Float64MultiArray, debug)
//
// Does NOT publish /cmd_vel directly.
// The longitudinal vehicle simulator is the sole publisher to Gazebo.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "car_msgs/msg/longitudinal.hpp"
#include "car_msgs/msg/longitudinal_reference.hpp"
#include "car_controller/longitudinal_limits.hpp"
#include "car_controller/pid_core.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class PidVelocityController : public rclcpp::Node
{
public:
  PidVelocityController()
  : Node("pid_velocity_controller_node")
  {
    // ---- PID params ----
    car_controller::PidCore::Params pid_params;
    pid_params.kp = declare_parameter<double>("kp", 0.8);
    pid_params.ki = declare_parameter<double>("ki", 0.05);
    pid_params.kd = declare_parameter<double>("kd", 0.02);
    pid_params.max_integral_error = declare_parameter<double>("max_integral_error", 5.0);
    pid_params.max_output = declare_parameter<double>("max_acceleration_correction", 2.0);
    pid_params.derivative_filter_alpha =
      declare_parameter<double>("derivative_filter_alpha", 0.2);

    // ---- Limiter params ----
    car_controller::LongitudinalLimits::Params lim_params;
    lim_params.min_target_acceleration = declare_parameter<double>("min_target_acceleration", -3.0);
    lim_params.max_target_acceleration = declare_parameter<double>("max_target_acceleration", 2.0);
    lim_params.min_jerk = declare_parameter<double>("min_jerk", -5.0);
    lim_params.max_jerk = declare_parameter<double>("max_jerk", 5.0);
    lim_params.max_acceleration_correction = pid_params.max_output;

    control_period_ms_ = declare_parameter<double>("control_period_ms", 20.0);
    control_period_ms_ = std::max(1.0, control_period_ms_);

    // ---- Topic names ----
    const std::string ref_topic = declare_parameter<std::string>(
      "longitudinal_reference_topic", "/planning/longitudinal_reference");
    const std::string odom_topic = declare_parameter<std::string>(
      "odom_topic", "/localization/kinematic_state");
    const std::string cmd_topic = declare_parameter<std::string>(
      "longitudinal_cmd_topic",
      "/control/trajectory_follower/longitudinal_cmd");
    const std::string correction_topic = declare_parameter<std::string>(
      "acceleration_correction_topic",
      "/control/trajectory_follower/acceleration_correction");
    const std::string debug_topic = declare_parameter<std::string>(
      "debug_topic", "/control/pid/debug");

    pid_ = std::make_unique<car_controller::PidCore>(pid_params);
    limiter_ = std::make_unique<car_controller::LongitudinalLimits>(lim_params);

    reference_sub_ =
      create_subscription<car_msgs::msg::LongitudinalReference>(
      ref_topic, 10,
      [this](const car_msgs::msg::LongitudinalReference::SharedPtr msg) {
        if (!std::isfinite(msg->velocity) || !std::isfinite(msg->acceleration) ||
          !std::isfinite(msg->jerk) || std::abs(msg->velocity) > 100.0F)
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Rejected invalid longitudinal reference");
          return;
        }
        reference_ = *msg;
        has_reference_ = true;
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        measured_velocity_ = msg->twist.twist.linear.x;
        has_velocity_ = true;
      });

    longitudinal_cmd_pub_ =
      create_publisher<car_msgs::msg::Longitudinal>(cmd_topic, 10);
    correction_pub_ = create_publisher<std_msgs::msg::Float64>(correction_topic, 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(debug_topic, 10);

    control_loop_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double, std::milli>(control_period_ms_))),
      std::bind(&PidVelocityController::controlLoop, this));

    last_cycle_time_ = get_clock()->now();
    RCLCPP_INFO(get_logger(), "PID longitudinal controller started (acceleration output).");
  }

private:
  void controlLoop()
  {
    if (!has_reference_ || !has_velocity_) {
      return;
    }

    const auto now = get_clock()->now();
    if (now.nanoseconds() == 0) {return;}

    const double nominal_dt = control_period_ms_ / 1000.0;
    double dt = nominal_dt;
    if (last_cycle_time_.nanoseconds() > 0) {
      const double measured_dt = (now - last_cycle_time_).seconds();
      if (measured_dt > 0.0 && measured_dt <= 10.0 * nominal_dt) {
        dt = measured_dt;
      }
    }

    const float v_ref = reference_.velocity;
    const double a_ref = reference_.is_defined_acceleration ?
      static_cast<double>(reference_.acceleration) : 0.0;

    // Controller error on measured velocity.
    const double error = static_cast<double>(v_ref) - measured_velocity_;

    // Reset if no target (near-zero velocity and small error).
    if (std::abs(static_cast<double>(v_ref)) < 1e-3 && std::abs(error) < 0.05) {
      pid_->reset();
      limiter_->reset();
    }

    const double raw_correction = pid_->update(error, dt);

    double a_correction_out, a_unlimited, a_target, jerk_out;
    limiter_->apply(a_ref, raw_correction, dt, a_correction_out, a_unlimited, a_target, jerk_out);

    // --- Publish longitudinal command ---
    car_msgs::msg::Longitudinal cmd_msg;
    cmd_msg.stamp = now;
    cmd_msg.control_time = now;
    cmd_msg.velocity = v_ref;
    cmd_msg.acceleration = static_cast<float>(a_target);
    cmd_msg.jerk = static_cast<float>(jerk_out);
    cmd_msg.is_defined_acceleration = true;
    cmd_msg.is_defined_jerk = true;
    longitudinal_cmd_pub_->publish(cmd_msg);

    // --- Publish correction (debug) ---
    std_msgs::msg::Float64 corr_msg;
    corr_msg.data = a_correction_out;
    correction_pub_->publish(corr_msg);

    // --- Publish debug array ---
    // [0] error, [1] p_term, [2] i_term, [3] d_term,
    // [4] a_correction, [5] a_unlimited, [6] a_target, [7] jerk
    std_msgs::msg::Float64MultiArray dbg_msg;
    dbg_msg.data = {
      error,
      pid_->p_term(),
      pid_->i_term(),
      pid_->d_term(),
      a_correction_out,
      a_unlimited,
      a_target,
      jerk_out,
    };
    debug_pub_->publish(dbg_msg);

    last_cycle_time_ = now;
  }

  // Subscriptions
  rclcpp::Subscription<car_msgs::msg::LongitudinalReference>::SharedPtr reference_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publishers
  rclcpp::Publisher<car_msgs::msg::Longitudinal>::SharedPtr longitudinal_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr correction_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;

  rclcpp::TimerBase::SharedPtr control_loop_;
  rclcpp::Time last_cycle_time_;

  std::unique_ptr<car_controller::PidCore> pid_;
  std::unique_ptr<car_controller::LongitudinalLimits> limiter_;

  car_msgs::msg::LongitudinalReference reference_{};
  double measured_velocity_{0.0};
  double control_period_ms_{20.0};
  bool has_reference_{false};
  bool has_velocity_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidVelocityController>());
  rclcpp::shutdown();
  return 0;
}
