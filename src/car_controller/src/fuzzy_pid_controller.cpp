// Copyright 2026 realisticCar project
//
// Fuzzy PID velocity controller node – Autoware-style longitudinal control.
//
// Subscribes:
//   /planning/longitudinal_reference  (car_msgs/LongitudinalReference)
//   /localization/kinematic_state     (nav_msgs/Odometry – noisy/measured)
//
// Publishes:
//   /control/trajectory_follower/longitudinal_cmd         (car_msgs/Longitudinal)
//   /control/trajectory_follower/acceleration_correction  (std_msgs/Float64, debug)
//   /control/fuzzy_pid/adaptive_gains                     (std_msgs/Float64MultiArray, debug)
//
// Does NOT publish /cmd_vel directly.
// The longitudinal vehicle simulator is the sole publisher to Gazebo.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "car_msgs/msg/longitudinal.hpp"
#include "car_msgs/msg/longitudinal_reference.hpp"
#include "car_controller/fuzzy_pid_core.hpp"
#include "car_controller/longitudinal_limits.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace controller
{

class FuzzyPidController : public rclcpp::Node
{
public:
  FuzzyPidController()
  : Node("fuzzy_pid_controller_node")
  {
    // ---- Fuzzy PID params ----
    car_controller::FuzzyPidCore::Params fp_params;
    fp_params.kp = declare_parameter<double>("kp", 0.8);
    fp_params.ki = declare_parameter<double>("ki", 0.05);
    fp_params.kd = declare_parameter<double>("kd", 0.02);
    fp_params.fuzzy_error_gain = declare_parameter<double>("fuzzy_error_gain", 0.8);
    fp_params.fuzzy_error_derivative_gain =
      declare_parameter<double>("fuzzy_error_derivative_gain", 0.5);
    fp_params.fuzzy_kd_min_ratio =
      std::clamp(declare_parameter<double>("fuzzy_kd_min_ratio", 0.5), 0.0, 1.0);
    fp_params.max_integral_error = declare_parameter<double>("max_integral_error", 5.0);
    fp_params.max_output = declare_parameter<double>("max_acceleration_correction", 2.0);
    fp_params.derivative_filter_alpha =
      declare_parameter<double>("derivative_filter_alpha", 0.2);

    // ---- Limiter params ----
    car_controller::LongitudinalLimits::Params lim_params;
    lim_params.min_target_acceleration = declare_parameter<double>("min_target_acceleration", -3.0);
    lim_params.max_target_acceleration = declare_parameter<double>("max_target_acceleration", 2.0);
    lim_params.min_jerk = declare_parameter<double>("min_jerk", -5.0);
    lim_params.max_jerk = declare_parameter<double>("max_jerk", 5.0);
    lim_params.max_acceleration_correction = fp_params.max_output;

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
    const std::string gains_topic = declare_parameter<std::string>(
      "adaptive_gains_topic", "/control/fuzzy_pid/adaptive_gains");

    fuzzy_pid_ = std::make_unique<car_controller::FuzzyPidCore>(fp_params);
    limiter_ = std::make_unique<car_controller::LongitudinalLimits>(lim_params);

    reference_sub_ =
      create_subscription<car_msgs::msg::LongitudinalReference>(
      ref_topic, 10,
      [this](const car_msgs::msg::LongitudinalReference::SharedPtr msg) {
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
    adaptive_gains_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(gains_topic, 10);

    control_loop_ = rclcpp::create_timer(
      this, get_clock(),
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double, std::milli>(control_period_ms_))),
      std::bind(&FuzzyPidController::controlLoop, this));

    last_cycle_time_ = get_clock()->now();
    RCLCPP_INFO(get_logger(), "Fuzzy PID longitudinal controller started (acceleration output).");
  }

private:
  void controlLoop()
  {
    if (!has_reference_ || !has_velocity_) {
      return;
    }
    const auto processing_start = std::chrono::steady_clock::now();

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

    // Reset if no target.
    if (std::abs(static_cast<double>(v_ref)) < 1e-3 &&
      std::abs(static_cast<double>(v_ref) - measured_velocity_) < 0.05)
    {
      fuzzy_pid_->reset();
      limiter_->reset();
    }

    const double error = static_cast<double>(v_ref) - measured_velocity_;
    const double raw_correction = fuzzy_pid_->update(error, dt);

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

    const auto processing_end = std::chrono::steady_clock::now();
    const double controller_processing_duration_us =
      std::chrono::duration<double, std::micro>(processing_end - processing_start).count();

    // --- Publish adaptive gains + PID terms ---
    // [0] adaptive_kp, [1] adaptive_ki, [2] adaptive_kd,
    // [3] error, [4] filtered_derivative,
    // [5] p_term, [6] i_term, [7] d_term,
    // [8] a_correction, [9] a_unlimited, [10] a_target, [11] jerk,
    // [12] fuzzy_e, [13] fuzzy_ec,
    // [14] fuzzy_inference_duration_us, [15] controller_processing_duration_us
    std_msgs::msg::Float64MultiArray gains_msg;
    gains_msg.data = {
      fuzzy_pid_->adaptive_kp(),
      fuzzy_pid_->adaptive_ki(),
      fuzzy_pid_->adaptive_kd(),
      error,
      fuzzy_pid_->filtered_derivative(),
      fuzzy_pid_->p_term(),
      fuzzy_pid_->i_term(),
      fuzzy_pid_->d_term(),
      a_correction_out,
      a_unlimited,
      a_target,
      jerk_out,
      fuzzy_pid_->fuzzy_error_input(),
      fuzzy_pid_->fuzzy_error_change_input(),
      fuzzy_pid_->fuzzy_inference_duration_us(),
      controller_processing_duration_us,
    };
    adaptive_gains_pub_->publish(gains_msg);

    last_cycle_time_ = now;
  }

  rclcpp::Subscription<car_msgs::msg::LongitudinalReference>::SharedPtr reference_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<car_msgs::msg::Longitudinal>::SharedPtr longitudinal_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr correction_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr adaptive_gains_pub_;
  rclcpp::TimerBase::SharedPtr control_loop_;
  rclcpp::Time last_cycle_time_;

  std::unique_ptr<car_controller::FuzzyPidCore> fuzzy_pid_;
  std::unique_ptr<car_controller::LongitudinalLimits> limiter_;

  car_msgs::msg::LongitudinalReference reference_{};
  double measured_velocity_{0.0};
  double control_period_ms_{20.0};
  bool has_reference_{false};
  bool has_velocity_{false};
};

}  // namespace controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<controller::FuzzyPidController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
