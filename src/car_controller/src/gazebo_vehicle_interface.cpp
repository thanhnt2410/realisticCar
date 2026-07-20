#include <chrono>
#include <memory>
#include <string>

#include "car_msgs/msg/longitudinal.hpp"
#include "car_controller/gazebo_effort_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class GazeboVehicleInterface : public rclcpp::Node
{
public:
  GazeboVehicleInterface()
  : Node("gazebo_vehicle_interface"),
    core_(
      declare_parameter<double>("effective_mass_kg", 1423.0),
      declare_parameter<double>("wheel_radius_m", 0.31265),
      declare_parameter<double>("max_acceleration_mps2", 3.0),
      declare_parameter<double>("max_wheel_torque_nm", 800.0))
  {
    timeout_ = rclcpp::Duration::from_seconds(
      declare_parameter<double>("command_timeout_sec", 0.25));
    const auto input = declare_parameter<std::string>(
      "input_topic", "/control/trajectory_follower/longitudinal_cmd");
    const auto left_output = declare_parameter<std::string>(
      "left_effort_topic", "/rear_wheel_effort/left");
    const auto right_output = declare_parameter<std::string>(
      "right_effort_topic", "/rear_wheel_effort/right");
    left_publisher_ = create_publisher<std_msgs::msg::Float64>(left_output, 10);
    right_publisher_ = create_publisher<std_msgs::msg::Float64>(right_output, 10);
    subscription_ = create_subscription<car_msgs::msg::Longitudinal>(
      input, 10, std::bind(&GazeboVehicleInterface::onCommand, this, std::placeholders::_1));
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration(20ms),
      std::bind(&GazeboVehicleInterface::watchdog, this));
    publishEffort(0.0);
  }

private:
  void onCommand(const car_msgs::msg::Longitudinal::SharedPtr msg)
  {
    if (!msg->is_defined_acceleration ||
      !core_.validCommand(msg->velocity, msg->acceleration, msg->jerk))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected invalid command");
      return;
    }
    const auto converted = core_.convert(msg->acceleration);
    publishEffort(converted.torque_each);
    last_valid_command_ = now();
    have_valid_command_ = true;
    zero_sent_ = false;
  }

  void watchdog()
  {
    if ((!have_valid_command_ || now() - last_valid_command_ > timeout_) && !zero_sent_) {
      publishEffort(0.0);
      zero_sent_ = true;
    }
  }

  void publishEffort(double torque)
  {
    std_msgs::msg::Float64 output;
    output.data = torque;
    left_publisher_->publish(output);
    right_publisher_->publish(output);
  }

  car_controller::GazeboEffortCore core_;
  rclcpp::Duration timeout_{0, 0};
  rclcpp::Time last_valid_command_{0, 0, RCL_ROS_TIME};
  bool have_valid_command_{false};
  bool zero_sent_{false};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_publisher_;
  rclcpp::Subscription<car_msgs::msg::Longitudinal>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboVehicleInterface>());
  rclcpp::shutdown();
  return 0;
}
