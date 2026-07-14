#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "car_control_msgs/msg/longitudinal_reference.hpp"
#include "car_planning/longitudinal_profile_core.hpp"
#include "rclcpp/rclcpp.hpp"

/// Trapezoid velocity profile publisher.
///
/// Publishes LongitudinalReference on /planning/longitudinal_reference.
/// Each segment is a ramp (linear velocity transition) followed by a hold.
///
///   v_ref  = v0 + alpha * (v1 - v0)   during ramp  (alpha = t_seg / ramp_duration)
///   a_ref  = (v1 - v0) / ramp_duration              during ramp  (constant, analytic)
///   a_ref  = 0                                       during hold
///   jerk   = 0 always (piecewise-linear profile; let the controller jerk-limiter handle edges)
///
/// NOTE: do NOT compute a_ref as finite-difference of consecutive v_ref samples.
/// Timer jitter would inject unwanted noise into the feed-forward path.

class TrapezoidVelocityProfile : public rclcpp::Node
{
public:
  TrapezoidVelocityProfile()
  : Node("trapezoid_velocity_profile_node")
  {
    longitudinal_reference_topic_ = declare_parameter<std::string>(
      "longitudinal_reference_topic", "/planning/longitudinal_reference");

    car_planning::LongitudinalProfileCoreParams core_params;
    core_params.velocity_points = declare_parameter<std::vector<double>>(
      "velocity_points", core_params.velocity_points);
    core_params.ramp_duration =
      declare_parameter<double>("ramp_duration", core_params.ramp_duration);
    core_params.hold_duration =
      declare_parameter<double>("hold_duration", core_params.hold_duration);
    core_params.loop = declare_parameter<bool>("loop", core_params.loop);

    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);
    publish_rate_ = std::max(publish_rate_, 1.0);

    core_ = std::make_unique<car_planning::LongitudinalProfileCore>(core_params);

    longitudinal_reference_pub_ =
      create_publisher<car_control_msgs::msg::LongitudinalReference>(
      longitudinal_reference_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TrapezoidVelocityProfile::publishProfile, this));
  }

private:
  void publishProfile()
  {
    const auto current_time = now();
    // With use_sim_time enabled, now() is zero until the first /clock message.
    // Starting the profile at zero would make it skip ahead by however long
    // Gazebo had already been running when this node received its first clock.
    if (current_time.nanoseconds() == 0) {
      return;
    }
    if (!profile_started_) {
      start_time_ = current_time;
      profile_started_ = true;
    }

    const double elapsed_time = (current_time - start_time_).seconds();

    double velocity = 0.0;
    double acceleration = 0.0;
    bool is_defined_acceleration = true;

    core_->calculateReference(elapsed_time, velocity, acceleration, is_defined_acceleration);

    car_control_msgs::msg::LongitudinalReference msg;
    msg.stamp = current_time;
    msg.velocity = static_cast<float>(velocity);
    msg.acceleration = static_cast<float>(acceleration);
    msg.jerk = 0.0f;
    msg.is_defined_acceleration = is_defined_acceleration;
    msg.is_defined_jerk = false;

    longitudinal_reference_pub_->publish(msg);
  }

  std::unique_ptr<car_planning::LongitudinalProfileCore> core_;
  rclcpp::Publisher<car_control_msgs::msg::LongitudinalReference>::SharedPtr
    longitudinal_reference_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;

  std::string longitudinal_reference_topic_;
  double publish_rate_;
  bool profile_started_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrapezoidVelocityProfile>());
  rclcpp::shutdown();
  return 0;
}
