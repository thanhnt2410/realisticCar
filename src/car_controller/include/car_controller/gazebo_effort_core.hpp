#pragma once

#include <algorithm>
#include <cmath>

namespace car_controller
{

struct EffortConversion
{
  double acceleration{0.0};
  double torque_each{0.0};
};

class GazeboEffortCore
{
public:
  GazeboEffortCore(
    double effective_mass, double wheel_radius,
    double max_acceleration, double max_torque)
  : effective_mass_(effective_mass), wheel_radius_(wheel_radius),
    max_acceleration_(std::abs(max_acceleration)), max_torque_(std::abs(max_torque)) {}

  bool validCommand(double velocity, double acceleration, double jerk) const
  {
    return std::isfinite(velocity) && std::isfinite(acceleration) && std::isfinite(jerk);
  }

  EffortConversion convert(double acceleration) const
  {
    const double safe_acceleration = std::clamp(
      acceleration, -max_acceleration_, max_acceleration_);
    const double raw_torque = effective_mass_ * safe_acceleration * wheel_radius_ / 2.0;
    return {safe_acceleration, std::clamp(raw_torque, -max_torque_, max_torque_)};
  }

private:
  double effective_mass_;
  double wheel_radius_;
  double max_acceleration_;
  double max_torque_;
};

}  // namespace car_controller
