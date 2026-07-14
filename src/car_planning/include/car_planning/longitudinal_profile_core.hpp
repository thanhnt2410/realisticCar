// Copyright 2026 realisticCar project
#ifndef CAR_PLANNING__LONGITUDINAL_PROFILE_CORE_HPP_
#define CAR_PLANNING__LONGITUDINAL_PROFILE_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace car_planning
{

struct LongitudinalProfileCoreParams
{
  std::vector<double> velocity_points{0.0, 0.3, 0.1, 0.2, 0.0};
  double ramp_duration = 3.0;
  double hold_duration = 5.0;
  bool loop = false;
};

class LongitudinalProfileCore
{
public:
  using Params = LongitudinalProfileCoreParams;

  explicit LongitudinalProfileCore(const Params & params = Params())
  : params_(params)
  {
    if (params_.velocity_points.size() < 2) {
      params_.velocity_points = {0.0, 0.3, 0.1, 0.2, 0.0};
    }
    params_.ramp_duration = std::max(params_.ramp_duration, 0.001);
    params_.hold_duration = std::max(params_.hold_duration, 0.0);
  }

  /// Compute reference velocity and acceleration analytically.
  void calculateReference(
    double elapsed_time,
    double & velocity,
    double & acceleration,
    bool & is_defined_acceleration) const
  {
    const double segment_duration = params_.ramp_duration + params_.hold_duration;
    const std::size_t segment_count = params_.velocity_points.size() - 1;
    const double profile_duration = segment_duration * static_cast<double>(segment_count);

    if (profile_duration <= 0.0) {
      velocity = params_.velocity_points.back();
      acceleration = 0.0;
      is_defined_acceleration = true;
      return;
    }

    double profile_time = elapsed_time;
    if (params_.loop) {
      profile_time = std::fmod(elapsed_time, profile_duration);
    } else if (elapsed_time >= profile_duration) {
      velocity = params_.velocity_points.back();
      acceleration = 0.0;
      is_defined_acceleration = true;
      return;
    }

    const auto segment_index = static_cast<std::size_t>(
      std::clamp(
        std::floor(profile_time / segment_duration),
        0.0,
        static_cast<double>(segment_count - 1)));
    const double segment_time =
      profile_time - static_cast<double>(segment_index) * segment_duration;

    const double start_velocity = params_.velocity_points[segment_index];
    const double end_velocity = params_.velocity_points[segment_index + 1];
    const double delta_velocity = end_velocity - start_velocity;

    if (segment_time < params_.ramp_duration) {
      // Ramp phase: constant acceleration.
      const double alpha = segment_time / params_.ramp_duration;
      velocity = start_velocity + alpha * delta_velocity;
      acceleration = delta_velocity / params_.ramp_duration;
      is_defined_acceleration = true;
    } else {
      // Hold phase: zero acceleration.
      velocity = end_velocity;
      acceleration = 0.0;
      is_defined_acceleration = true;
    }
  }

  const Params & params() const {return params_;}

private:
  Params params_;
};

}  // namespace car_planning

#endif  // CAR_PLANNING__LONGITUDINAL_PROFILE_CORE_HPP_
