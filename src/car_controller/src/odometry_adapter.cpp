#include <memory>
#include <string>

#include "car_controller/odometry_adapter_core.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class OdometryAdapter : public rclcpp::Node
{
public:
  OdometryAdapter()
  : Node("odometry_adapter"),
    noise_(declare_parameter<double>("velocity_noise_sigma", 0.0),
      static_cast<unsigned int>(declare_parameter<int>("random_seed", 1001)))
  {
    const auto frame = declare_parameter<std::string>("twist_frame", "body");
    frame_ = frame == "world" ? car_controller::TwistFrame::WORLD :
      car_controller::TwistFrame::BODY;
    use_y_axis_ = declare_parameter<std::string>("body_longitudinal_axis", "x") == "y";
    longitudinal_sign_ = declare_parameter<double>("body_longitudinal_sign", 1.0);
    const auto input = declare_parameter<std::string>("input_topic", "/simulation/gazebo_odometry");
    ground_truth_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/simulation/ground_truth/odometry", 10);
    measured_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", 10);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input, 10, std::bind(&OdometryAdapter::onOdometry, this, std::placeholders::_1));
  }

private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & q = msg->pose.pose.orientation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double velocity = frame_ == car_controller::TwistFrame::BODY ?
      car_controller::bodyLongitudinalVelocity(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y,
      use_y_axis_, longitudinal_sign_) :
      car_controller::longitudinalVelocity(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y, yaw, frame_);
    auto truth = *msg;
    truth.twist.twist.linear.x = velocity;
    ground_truth_pub_->publish(truth);
    auto measured = truth;
    measured.twist.twist.linear.x = noise_.add(velocity);
    measured_pub_->publish(measured);
  }

  car_controller::TwistFrame frame_;
  bool use_y_axis_{false};
  double longitudinal_sign_{1.0};
  car_controller::SeededGaussianNoise noise_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr measured_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryAdapter>());
  rclcpp::shutdown();
  return 0;
}
