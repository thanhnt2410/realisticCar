// Copyright 2026 realisticCar project
//
// Velocity logger – new CSV schema aligned with Autoware-style architecture.
//
// Subscribes (triggered by plant debug topic):
//   /simulation/vehicle_dynamics/debug   (std_msgs/Float64MultiArray) – plant step
//   /simulation/ground_truth/odometry    (nav_msgs/Odometry)          – true velocity
//   /localization/kinematic_state        (nav_msgs/Odometry)          – measured velocity
//   /planning/longitudinal_reference     (car_msgs/LongitudinalReference)
//   /control/trajectory_follower/longitudinal_cmd (car_msgs/Longitudinal)
//   /control/trajectory_follower/acceleration_correction (std_msgs/Float64)
//   /control/fuzzy_pid/adaptive_gains    (std_msgs/Float64MultiArray) – optional
//
// CSV columns (new schema v2):
//   time_sec, sim_step, scenario, seed, controller,
//   target_velocity_mps, reference_acceleration_mps2,
//   true_velocity_mps, measured_velocity_mps,
//   true_velocity_error_mps, controller_velocity_error_mps,
//   acceleration_correction_mps2,
//   unlimited_target_acceleration_mps2, target_acceleration_mps2,
//   applied_acceleration_mps2, jerk_mps3, left_wheel_torque_nm, right_wheel_torque_nm,
//   grade_percent, grade_acceleration_mps2,
//   drag_acceleration_mps2, rolling_acceleration_mps2,
//   process_disturbance_acceleration_mps2,
//   adaptive_kp, adaptive_ki, adaptive_kd,
//   fuzzy_error_e_mps, fuzzy_error_change_ec_mps2,
//   fuzzy_normalized_error_e, fuzzy_normalized_error_change_ec,
//   p_term_mps2, i_term_mps2, d_term_mps2,
//   fuzzy_inference_duration_us, controller_processing_duration_us,
//   correction_saturated

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include "car_msgs/msg/longitudinal.hpp"
#include "car_msgs/msg/longitudinal_reference.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class VelocityLogger : public rclcpp::Node
{
public:
  VelocityLogger()
  : Node("velocity_logger_node")
  {
    // ---- Params ----
    log_file_path_ = declare_parameter<std::string>("log_file_path", "logs/car_velocity_log.csv");
    log_rate_ = declare_parameter<double>("log_rate", 50.0);
    scenario_ = declare_parameter<std::string>("scenario", "baseline");
    seed_ = declare_parameter<int>("seed", 1001);
    controller_name_ = declare_parameter<std::string>("controller", "fuzzy_pid");
    vehicle_backend_ = declare_parameter<std::string>("vehicle_backend", "longitudinal_sim");
    correction_saturation_threshold_ = declare_parameter<double>(
      "correction_saturation_threshold", 2.0);
    log_adaptive_gains_ = declare_parameter<bool>("log_adaptive_gains", false);

    log_rate_ = std::max(log_rate_, 1.0);

    // ---- Topics ----
    const std::string gt_odom_topic = declare_parameter<std::string>(
      "ground_truth_odom_topic", "/simulation/ground_truth/odometry");
    const std::string kin_state_topic = declare_parameter<std::string>(
      "kinematic_state_topic", "/localization/kinematic_state");
    const std::string ref_topic = declare_parameter<std::string>(
      "longitudinal_reference_topic", "/planning/longitudinal_reference");
    const std::string cmd_topic = declare_parameter<std::string>(
      "longitudinal_cmd_topic", "/control/trajectory_follower/longitudinal_cmd");
    const std::string correction_topic = declare_parameter<std::string>(
      "acceleration_correction_topic", "/control/trajectory_follower/acceleration_correction");
    const std::string gains_topic = declare_parameter<std::string>(
      "adaptive_gains_topic", "/control/fuzzy_pid/adaptive_gains");
    const std::string plant_debug_topic = declare_parameter<std::string>(
      "plant_debug_topic", "/simulation/vehicle_dynamics/debug");
    const std::string left_effort_topic = declare_parameter<std::string>(
      "left_effort_topic", "/rear_wheel_effort/left");
    const std::string right_effort_topic = declare_parameter<std::string>(
      "right_effort_topic", "/rear_wheel_effort/right");

    // ---- Open log file ----
    const std::filesystem::path log_path(log_file_path_);
    if (log_path.has_parent_path()) {
      std::filesystem::create_directories(log_path.parent_path());
    }
    log_file_.open(log_file_path_, std::ios::out | std::ios::trunc);
    if (!log_file_.is_open()) {
      throw std::runtime_error("Failed to open log file: " + log_file_path_);
    }

    writeHeader();
    log_file_ << std::fixed << std::setprecision(6);

    // ---- Subscriptions ----
    gt_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      gt_odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto stamp = rclcpp::Time(msg->header.stamp);
        if (has_true_velocity_ && stamp > last_ground_truth_stamp_) {
          applied_acceleration_from_odom_ =
          (msg->twist.twist.linear.x - true_velocity_) /
          (stamp - last_ground_truth_stamp_).seconds();
        }
        true_velocity_ = msg->twist.twist.linear.x;
        last_ground_truth_stamp_ = stamp;
        has_true_velocity_ = true;
      });

    kin_state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      kin_state_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        measured_velocity_ = msg->twist.twist.linear.x;
        has_measured_velocity_ = true;
      });

    ref_sub_ = create_subscription<car_msgs::msg::LongitudinalReference>(
      ref_topic, 10,
      [this](const car_msgs::msg::LongitudinalReference::SharedPtr msg) {
        v_ref_ = static_cast<double>(msg->velocity);
        a_ref_ = static_cast<double>(msg->acceleration);
        has_reference_ = true;
      });

    cmd_sub_ = create_subscription<car_msgs::msg::Longitudinal>(
      cmd_topic, 10,
      [this](const car_msgs::msg::Longitudinal::SharedPtr msg) {
        target_acceleration_ = static_cast<double>(msg->acceleration);
        jerk_ = static_cast<double>(msg->jerk);
        has_cmd_ = true;
      });

    correction_sub_ = create_subscription<std_msgs::msg::Float64>(
      correction_topic, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        acceleration_correction_ = msg->data;
        has_correction_ = true;
      });

    // Plant debug: [0]=v_true [1]=v_measured [2]=a_applied [3]=a_grade
    // [4]=a_rolling [5]=a_drag [6]=a_noise [7]=a_total [8]=delayed_cmd [9]=grade_percent
    plant_debug_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      plant_debug_topic, 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 10) {
          applied_acceleration_ = msg->data[2];
          grade_acceleration_ = msg->data[3];
          rolling_acceleration_ = msg->data[4];
          drag_acceleration_ = msg->data[5];
          process_disturbance_ = msg->data[6];
          grade_percent_ = msg->data[9];
          has_plant_debug_ = true;
        }
      });

    left_effort_sub_ = create_subscription<std_msgs::msg::Float64>(
      left_effort_topic, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {left_wheel_torque_ = msg->data;});
    right_effort_sub_ = create_subscription<std_msgs::msg::Float64>(
      right_effort_topic, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {right_wheel_torque_ = msg->data;});

    if (log_adaptive_gains_) {
      adaptive_gains_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        gains_topic, 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          // [0]=kp [1]=ki [2]=kd [3]=error [4]=deriv [5]=p_term [6]=i_term [7]=d_term
          if (msg->data.size() >= 8) {
            adaptive_kp_ = msg->data[0];
            adaptive_ki_ = msg->data[1];
            adaptive_kd_ = msg->data[2];
            fuzzy_error_e_ = msg->data[3];
            fuzzy_error_change_ec_ = msg->data[4];
            p_term_ = msg->data[5];
            i_term_ = msg->data[6];
            d_term_ = msg->data[7];
          }
          if (msg->data.size() >= 11) {
            unlimited_target_acceleration_ = msg->data[9];
          }
          if (msg->data.size() >= 14) {
            fuzzy_normalized_error_e_ = msg->data[12];
            fuzzy_normalized_error_change_ec_ = msg->data[13];
          }
          if (msg->data.size() >= 16) {
            fuzzy_inference_duration_us_ = msg->data[14];
            controller_processing_duration_us_ = msg->data[15];
          }
          has_gains_ = true;
        });
    }

    const auto period = std::chrono::duration<double>(1.0 / log_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VelocityLogger::writeSample, this));

    RCLCPP_INFO(get_logger(), "Logging to %s (schema v2)", log_file_path_.c_str());
  }

  ~VelocityLogger() override
  {
    if (log_file_.is_open()) {
      log_file_.flush();
      log_file_.close();
    }
  }

private:
  void writeHeader()
  {
    log_file_ <<
      "time_sec,sim_step,scenario,seed,controller,vehicle_backend,"
      "target_velocity_mps,reference_acceleration_mps2,"
      "true_velocity_mps,measured_velocity_mps,"
      "true_velocity_error_mps,controller_velocity_error_mps,"
      "acceleration_correction_mps2,"
      "unlimited_target_acceleration_mps2,target_acceleration_mps2,"
      "applied_acceleration_mps2,jerk_mps3,"
      "left_wheel_torque_nm,right_wheel_torque_nm,"
      "grade_percent,grade_acceleration_mps2,"
      "drag_acceleration_mps2,rolling_acceleration_mps2,"
      "process_disturbance_acceleration_mps2,"
      "adaptive_kp,adaptive_ki,adaptive_kd,"
      "fuzzy_error_e_mps,fuzzy_error_change_ec_mps2,"
      "fuzzy_normalized_error_e,fuzzy_normalized_error_change_ec,"
      "p_term_mps2,i_term_mps2,d_term_mps2,"
      "fuzzy_inference_duration_us,controller_processing_duration_us,"
      "correction_saturated\n";
  }

  void writeSample()
  {
    // Need at least true velocity and reference.
    if (!has_true_velocity_ || !has_reference_) {
      return;
    }

    const auto now = get_clock()->now();
    if (now.nanoseconds() == 0) {return;}
    if (!logging_started_) {
      start_time_ = now;
      logging_started_ = true;
    }

    const double time_sec = (now - start_time_).seconds();
    const double true_error = v_ref_ - true_velocity_;
    const double ctrl_error = v_ref_ - measured_velocity_;
    const bool saturated = has_correction_ &&
      std::abs(acceleration_correction_) >= correction_saturation_threshold_;

    log_file_
      << time_sec << ','
      << sim_step_++ << ','
      << scenario_ << ','
      << seed_ << ','
      << controller_name_ << ','
      << vehicle_backend_ << ','
      << v_ref_ << ','
      << a_ref_ << ','
      << true_velocity_ << ','
      << (has_measured_velocity_ ? measured_velocity_ : true_velocity_) << ','
      << true_error << ','
      << ctrl_error << ','
      << (has_correction_ ? acceleration_correction_ : 0.0) << ','
      << unlimited_target_acceleration_ << ','
      << (has_cmd_ ? target_acceleration_ : 0.0) << ','
      << (has_plant_debug_ ? applied_acceleration_ : applied_acceleration_from_odom_) << ','
      << (has_cmd_ ? jerk_ : 0.0) << ','
      << left_wheel_torque_ << ','
      << right_wheel_torque_ << ','
      << grade_percent_ << ','
      << (has_plant_debug_ ? grade_acceleration_ : 0.0) << ','
      << (has_plant_debug_ ? drag_acceleration_ : 0.0) << ','
      << (has_plant_debug_ ? rolling_acceleration_ : 0.0) << ','
      << (has_plant_debug_ ? process_disturbance_ : 0.0) << ','
      << adaptive_kp_ << ','
      << adaptive_ki_ << ','
      << adaptive_kd_ << ','
      << fuzzy_error_e_ << ','
      << fuzzy_error_change_ec_ << ','
      << fuzzy_normalized_error_e_ << ','
      << fuzzy_normalized_error_change_ec_ << ','
      << p_term_ << ','
      << i_term_ << ','
      << d_term_ << ','
      << fuzzy_inference_duration_us_ << ','
      << controller_processing_duration_us_ << ','
      << (saturated ? 1 : 0)
      << '\n';
  }

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kin_state_sub_;
  rclcpp::Subscription<car_msgs::msg::LongitudinalReference>::SharedPtr ref_sub_;
  rclcpp::Subscription<car_msgs::msg::Longitudinal>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr correction_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr plant_debug_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_effort_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_effort_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr adaptive_gains_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  rclcpp::Time last_ground_truth_stamp_;

  std::ofstream log_file_;
  std::string log_file_path_;
  std::string scenario_;
  std::string controller_name_;
  std::string vehicle_backend_;
  int seed_{1001};
  double log_rate_{50.0};
  double correction_saturation_threshold_{2.0};
  bool log_adaptive_gains_{false};
  bool logging_started_{false};
  uint64_t sim_step_{0};

  // State cache
  double v_ref_{0.0}, a_ref_{0.0};
  double true_velocity_{0.0}, measured_velocity_{0.0};
  double acceleration_correction_{0.0};
  double unlimited_target_acceleration_{0.0};
  double target_acceleration_{0.0};
  double jerk_{0.0};
  double applied_acceleration_{0.0};
  double applied_acceleration_from_odom_{0.0};
  double left_wheel_torque_{0.0}, right_wheel_torque_{0.0};
  double grade_percent_{0.0};
  double grade_acceleration_{0.0};
  double drag_acceleration_{0.0};
  double rolling_acceleration_{0.0};
  double process_disturbance_{0.0};
  double adaptive_kp_{0.0}, adaptive_ki_{0.0}, adaptive_kd_{0.0};
  double fuzzy_error_e_{0.0}, fuzzy_error_change_ec_{0.0};
  double fuzzy_normalized_error_e_{0.0}, fuzzy_normalized_error_change_ec_{0.0};
  double p_term_{0.0}, i_term_{0.0}, d_term_{0.0};
  double fuzzy_inference_duration_us_{0.0}, controller_processing_duration_us_{0.0};

  bool has_true_velocity_{false};
  bool has_measured_velocity_{false};
  bool has_reference_{false};
  bool has_cmd_{false};
  bool has_correction_{false};
  bool has_plant_debug_{false};
  bool has_gains_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityLogger>());
  rclcpp::shutdown();
  return 0;
}
