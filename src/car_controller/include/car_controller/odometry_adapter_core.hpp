#pragma once

#include <cmath>
#include <random>

namespace car_controller
{

enum class TwistFrame { BODY, WORLD };

inline double longitudinalVelocity(double vx, double vy, double yaw, TwistFrame frame)
{
  return frame == TwistFrame::BODY ? vx : vx * std::cos(yaw) + vy * std::sin(yaw);
}

inline double bodyLongitudinalVelocity(double vx, double vy, bool use_y_axis, double sign)
{
  return sign * (use_y_axis ? vy : vx);
}

class SeededGaussianNoise
{
public:
  SeededGaussianNoise(double sigma, unsigned int seed)
  : sigma_(sigma), generator_(seed), distribution_(0.0, sigma) {}

  double add(double value)
  {
    return sigma_ == 0.0 ? value : value + distribution_(generator_);
  }

private:
  double sigma_;
  std::mt19937 generator_;
  std::normal_distribution<double> distribution_;
};

}  // namespace car_controller
