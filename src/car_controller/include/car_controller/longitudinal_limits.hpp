// Copyright 2026 realisticCar project
#ifndef CAR_CONTROLLER__LONGITUDINAL_LIMITS_HPP_
#define CAR_CONTROLLER__LONGITUDINAL_LIMITS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace car_controller
{

/// Parameters for LongitudinalLimits.
/// Defined outside class to avoid GCC C++17 default-argument nested struct bug.
struct LongitudinalLimitsParams
{
  double min_target_acceleration = -3.0;     ///< Lower bound on output acceleration [m/s^2]
  double max_target_acceleration = 2.0;      ///< Upper bound on output acceleration [m/s^2]
  double min_jerk = -5.0;                    ///< Minimum jerk [m/s^3]
  double max_jerk = 5.0;                     ///< Maximum jerk [m/s^3]
  double max_acceleration_correction = 2.0;  ///< Clamp on feedback correction magnitude [m/s^2]
};

/// Acceleration + jerk limiter shared by PID and Fuzzy PID controllers.
class LongitudinalLimits
{
public:
  using Params = LongitudinalLimitsParams;

  explicit LongitudinalLimits(const Params & params = Params())
  : params_(params), prev_acceleration_(std::numeric_limits<double>::quiet_NaN())
  {}

  /// Apply correction clamp, acceleration clamp, and jerk limit.
  void apply(
    double a_ref,
    double a_correction,
    double dt,
    double & a_correction_out,
    double & a_unlimited,
    double & a_target,
    double & jerk_out)
  {
    a_correction_out = std::clamp(
      a_correction,
      -params_.max_acceleration_correction,
      params_.max_acceleration_correction);

    a_unlimited = a_ref + a_correction_out;

    double a_clamped = std::clamp(
      a_unlimited,
      params_.min_target_acceleration,
      params_.max_target_acceleration);

    if (std::isnan(prev_acceleration_) || dt <= 0.0) {
      a_target = a_clamped;
      jerk_out = 0.0;
    } else {
      const double max_delta = params_.max_jerk * dt;
      const double min_delta = params_.min_jerk * dt;
      const double delta = std::clamp(a_clamped - prev_acceleration_, min_delta, max_delta);
      a_target = prev_acceleration_ + delta;
      jerk_out = delta / dt;
    }

    prev_acceleration_ = a_target;
  }

  void reset()
  {
    prev_acceleration_ = std::numeric_limits<double>::quiet_NaN();
  }

  const Params & params() const {return params_;}

private:
  Params params_;
  double prev_acceleration_;
};

}  // namespace car_controller

#endif  // CAR_CONTROLLER__LONGITUDINAL_LIMITS_HPP_
