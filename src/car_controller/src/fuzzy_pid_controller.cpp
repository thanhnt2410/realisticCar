// Copyright 2026 realisticCar project
//
// Fuzzy PID velocity controller node – Autoware-style longitudinal control.
//
// Subscribes:
//   /planning/longitudinal_reference  (car_msgs/LongitudinalReference)
//   /localization/kinematic_state     (nav_msgs/Odometry – noisy/measured)
//
// Publishes:
//   /control/trajectory_follower/longitudinal_cmd (car_msgs/Longitudinal)
//   /control/trajectory_follower/acceleration_correction  (std_msgs/Float64,
//   debug) /control/fuzzy_pid/adaptive_gains (std_msgs/Float64MultiArray,
//   debug)
//
// Does NOT publish /cmd_vel directly.
// The longitudinal vehicle simulator is the sole publisher to Gazebo.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <system_error>
#include <vector>

#include "car_controller/fuzzy_pid_core.hpp"
#include "car_controller/longitudinal_limits.hpp"
#include "car_msgs/msg/longitudinal.hpp"
#include "car_msgs/msg/longitudinal_reference.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace controller {

class FuzzyPidController : public rclcpp::Node {
public:
  FuzzyPidController() : Node("fuzzy_pid_controller_node") {
    // ---- Fuzzy PID params ----
    car_controller::FuzzyPidCore::Params fp_params;
    kp_ = declare_parameter<double>("kp", 0.8);
    ki_ = declare_parameter<double>("ki", 0.05);
    kd_ = declare_parameter<double>("kd", 0.02);
    fp_params.kp = kp_;
    fp_params.ki = ki_;
    fp_params.kd = kd_;
    fp_params.fuzzy_error_gain =
        declare_parameter<double>("fuzzy_error_gain", 0.8);
    fp_params.fuzzy_error_derivative_gain =
        declare_parameter<double>("fuzzy_error_derivative_gain", 0.5);
    fp_params.fuzzy_kp_min_ratio = std::clamp(
        declare_parameter<double>("fuzzy_kp_min_ratio", 0.7), 0.1, 1.0);
    fp_params.fuzzy_kd_min_ratio = std::clamp(
        declare_parameter<double>("fuzzy_kd_min_ratio", 0.8), 0.1, 1.0);
    fp_params.gain_rate_limit =
        std::max(0.0, declare_parameter<double>("gain_rate_limit", 0.3));
    fp_params.max_integral_error =
        declare_parameter<double>("max_integral_error", 5.0);
    fp_params.max_output =
        declare_parameter<double>("max_acceleration_correction", 2.0);
    fp_params.derivative_filter_alpha =
        declare_parameter<double>("derivative_filter_alpha", 0.2);

    // ---- Limiter params ----
    car_controller::LongitudinalLimits::Params lim_params;
    lim_params.min_target_acceleration =
        declare_parameter<double>("min_target_acceleration", -3.0);
    lim_params.max_target_acceleration =
        declare_parameter<double>("max_target_acceleration", 2.0);
    lim_params.min_jerk = declare_parameter<double>("min_jerk", -5.0);
    lim_params.max_jerk = declare_parameter<double>("max_jerk", 5.0);
    lim_params.max_acceleration_correction = fp_params.max_output;

    control_period_ms_ = declare_parameter<double>("control_period_ms", 20.0);
    control_period_ms_ = std::max(1.0, control_period_ms_);
    publish_debug_ = declare_parameter<bool>("publish_debug", false);
    measure_timing_ = declare_parameter<bool>("measure_timing", false);
    timing_log_file_ = declare_parameter<std::string>(
        "timing_log_file", "logs/fuzzy_pid_internal_timing.csv");
    fp_params.measure_timing = measure_timing_;
    if (measure_timing_) {
      timing_samples_.reserve(10000);
    }

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

    reference_sub_ = create_subscription<car_msgs::msg::LongitudinalReference>(
        ref_topic, 10,
        [this](const car_msgs::msg::LongitudinalReference::SharedPtr msg) {
          if (!std::isfinite(msg->velocity) || !std::isfinite(msg->acceleration) ||
              !std::isfinite(msg->jerk) || std::abs(msg->velocity) > 100.0F) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Rejected invalid longitudinal reference");
            return;
          }
          reference_ = *msg;
          has_reference_ = true;
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          measured_velocity_ = msg->twist.twist.linear.x;
          has_velocity_ = true;
        });

    longitudinal_cmd_pub_ =
        create_publisher<car_msgs::msg::Longitudinal>(cmd_topic, 10);
    if (publish_debug_) {
      correction_pub_ =
          create_publisher<std_msgs::msg::Float64>(correction_topic, 10);
      adaptive_gains_pub_ =
          create_publisher<std_msgs::msg::Float64MultiArray>(gains_topic, 10);
    }

    control_loop_ = rclcpp::create_timer(
        this, get_clock(),
        rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double, std::milli>(control_period_ms_))),
        std::bind(&FuzzyPidController::controlLoop, this));

    last_cycle_time_ = get_clock()->now();
    RCLCPP_INFO(
        get_logger(),
        "Fuzzy PID longitudinal controller started (acceleration output). "
        "Base gains: Kp=%.3f  Ki=%.3f  Kd=%.3f",
        kp_, ki_, kd_);
  }

  ~FuzzyPidController() override {
    writeTimingLog();
  }

private:
  struct TimingSample {
    uint64_t cycle;
    double fuzzy_inference_us;
    double state_and_dt_us;
    double core_update_us;
    double limiter_us;
    double command_message_us;
    double command_publish_us;
    double correction_message_us;
    double correction_publish_us;
    double gains_message_us;
    double gains_publish_us;
    double controller_processing_us;
    double full_callback_us;
  };

  void writeTimingLog() noexcept {
    if (!measure_timing_) {
      return;
    }

    try {
      const std::filesystem::path log_path(timing_log_file_);
      if (log_path.has_parent_path()) {
        std::error_code error;
        std::filesystem::create_directories(log_path.parent_path(), error);
        if (error) {
          RCLCPP_ERROR(
              get_logger(), "Failed to create timing log directory: %s",
              error.message().c_str());
          return;
        }
      }

      std::ofstream log_file(timing_log_file_, std::ios::out | std::ios::trunc);
      if (!log_file.is_open()) {
        RCLCPP_ERROR(
            get_logger(), "Failed to open timing log: %s", timing_log_file_.c_str());
        return;
      }

      log_file
          << "cycle,publish_debug,fuzzy_inference_duration_us,"
          << "state_and_dt_duration_us,core_update_duration_us,limiter_duration_us,"
          << "command_message_duration_us,command_publish_duration_us,"
          << "correction_message_duration_us,correction_publish_duration_us,"
          << "gains_message_duration_us,gains_publish_duration_us,"
          << "controller_processing_duration_us,full_callback_duration_us\n";
      log_file << std::fixed << std::setprecision(6);
      for (const auto & sample : timing_samples_) {
        log_file
            << sample.cycle << ','
            << (publish_debug_ ? 1 : 0) << ','
            << sample.fuzzy_inference_us << ','
            << sample.state_and_dt_us << ','
            << sample.core_update_us << ','
            << sample.limiter_us << ','
            << sample.command_message_us << ','
            << sample.command_publish_us << ','
            << sample.correction_message_us << ','
            << sample.correction_publish_us << ','
            << sample.gains_message_us << ','
            << sample.gains_publish_us << ','
            << sample.controller_processing_us << ','
            << sample.full_callback_us << '\n';
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Failed to write timing log: %s", error.what());
    }
  }

  void controlLoop() {
    if (!has_reference_ || !has_velocity_) {
      return;
    }
    const auto processing_start = measure_timing_ ?
      std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    const auto duration_us = [](const auto & end, const auto & start) {
        return std::chrono::duration<double, std::micro>(end - start).count();
      };

    const auto now = get_clock()->now();
    if (now.nanoseconds() == 0) {
      return;
    }

    const double nominal_dt = control_period_ms_ / 1000.0;
    double dt = nominal_dt;
    if (last_cycle_time_.nanoseconds() > 0) {
      const double measured_dt = (now - last_cycle_time_).seconds();
      if (measured_dt > 0.0 && measured_dt <= 10.0 * nominal_dt) {
        dt = measured_dt;
      }
    }

    const float v_ref = reference_.velocity;
    const double a_ref = reference_.is_defined_acceleration
                             ? static_cast<double>(reference_.acceleration)
                             : 0.0;

    const bool at_standstill = std::abs(static_cast<double>(v_ref)) < 1e-3 &&
                               std::abs(measured_velocity_) < 0.05;
    if (at_standstill && has_moved_) {
      fuzzy_pid_->reset();
      limiter_->reset();
    }
    // Track whether the vehicle has ever left standstill.
    if (!has_moved_ && std::abs(static_cast<double>(v_ref)) > 0.05) {
      has_moved_ = true;
    }

    const double error = static_cast<double>(v_ref) - measured_velocity_;
    const auto state_end = measure_timing_ ?
      std::chrono::steady_clock::now() : processing_start;

    const auto core_start = state_end;
    const double raw_correction = fuzzy_pid_->update(error, dt);
    const auto core_end = measure_timing_ ?
      std::chrono::steady_clock::now() : core_start;

    double a_correction_out, a_unlimited, a_target, jerk_out;
    limiter_->apply(a_ref, raw_correction, dt, a_correction_out, a_unlimited,
                    a_target, jerk_out);
    const auto limiter_end = measure_timing_ ?
      std::chrono::steady_clock::now() : core_end;

    // --- Publish longitudinal command ---
    car_msgs::msg::Longitudinal cmd_msg;
    cmd_msg.stamp = now;
    cmd_msg.control_time = now;
    cmd_msg.velocity = v_ref;
    cmd_msg.acceleration = static_cast<float>(a_target);
    cmd_msg.jerk = static_cast<float>(jerk_out);
    cmd_msg.is_defined_acceleration = true;
    cmd_msg.is_defined_jerk = true;
    const auto command_message_end = measure_timing_ ?
      std::chrono::steady_clock::now() : limiter_end;
    longitudinal_cmd_pub_->publish(cmd_msg);
    const auto command_publish_end = measure_timing_ ?
      std::chrono::steady_clock::now() : command_message_end;

    double correction_message_duration_us = 0.0;
    double correction_publish_duration_us = 0.0;
    auto processing_end = command_publish_end;
    if (publish_debug_) {
      // --- Publish correction (debug) ---
      std_msgs::msg::Float64 corr_msg;
      corr_msg.data = a_correction_out;
      const auto correction_message_end = measure_timing_ ?
        std::chrono::steady_clock::now() : command_publish_end;
      correction_pub_->publish(corr_msg);
      processing_end = measure_timing_ ?
        std::chrono::steady_clock::now() : correction_message_end;
      correction_message_duration_us =
          duration_us(correction_message_end, command_publish_end);
      correction_publish_duration_us =
          duration_us(processing_end, correction_message_end);
    }

    const double controller_processing_duration_us =
        std::chrono::duration<double, std::micro>(processing_end -
                                                  processing_start)
            .count();

    const double state_and_dt_duration_us = duration_us(state_end, processing_start);
    const double core_update_duration_us = duration_us(core_end, core_start);
    const double limiter_duration_us = duration_us(limiter_end, core_end);
    const double command_message_duration_us =
        duration_us(command_message_end, limiter_end);
    const double command_publish_duration_us =
        duration_us(command_publish_end, command_message_end);
    double gains_message_duration_us = 0.0;
    double gains_publish_duration_us = 0.0;
    if (publish_debug_) {
      const auto gains_message_start = measure_timing_ ?
        std::chrono::steady_clock::now() : processing_end;
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
      };
      const auto gains_message_end = measure_timing_ ?
        std::chrono::steady_clock::now() : gains_message_start;
      gains_message_duration_us = duration_us(gains_message_end, gains_message_start);

      const auto gains_publish_start = gains_message_end;
      adaptive_gains_pub_->publish(gains_msg);
      const auto gains_publish_end = measure_timing_ ?
        std::chrono::steady_clock::now() : gains_publish_start;
      gains_publish_duration_us = duration_us(gains_publish_end, gains_publish_start);
    }

    last_cycle_time_ = now;
    const auto full_callback_end = measure_timing_ ?
      std::chrono::steady_clock::now() : processing_start;
    if (measure_timing_) {
      timing_samples_.push_back({
        cycle_index_++,
        fuzzy_pid_->fuzzy_inference_duration_us(),
        state_and_dt_duration_us,
        core_update_duration_us,
        limiter_duration_us,
        command_message_duration_us,
        command_publish_duration_us,
        correction_message_duration_us,
        correction_publish_duration_us,
        gains_message_duration_us,
        gains_publish_duration_us,
        controller_processing_duration_us,
        duration_us(full_callback_end, processing_start),
      });
    }
  }

  rclcpp::Subscription<car_msgs::msg::LongitudinalReference>::SharedPtr
      reference_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<car_msgs::msg::Longitudinal>::SharedPtr
      longitudinal_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr correction_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      adaptive_gains_pub_;
  rclcpp::TimerBase::SharedPtr control_loop_;
  rclcpp::Time last_cycle_time_;

  std::unique_ptr<car_controller::FuzzyPidCore> fuzzy_pid_;
  std::unique_ptr<car_controller::LongitudinalLimits> limiter_;

  car_msgs::msg::LongitudinalReference reference_{};
  double measured_velocity_{0.0};
  double control_period_ms_{20.0};
  bool has_reference_{false};
  bool has_velocity_{false};
  bool has_moved_{false};
  bool publish_debug_{false};
  bool measure_timing_{false};
  std::string timing_log_file_;
  std::vector<TimingSample> timing_samples_;
  uint64_t cycle_index_{0};

  /// Base PID gains declared from ROS parameters and stored as member variables
  /// so they are accessible from any method in the class (e.g., logging,
  /// reset).
  double kp_{0.0};
  double ki_{0.0};
  double kd_{0.0};
};

} // namespace controller

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<controller::FuzzyPidController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
