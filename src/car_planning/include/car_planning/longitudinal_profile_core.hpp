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
  double acceleration_ramp_duration = 0.5;
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
    params_.acceleration_ramp_duration = std::clamp(
      params_.acceleration_ramp_duration, 1e-6, 0.5 * params_.ramp_duration);
    params_.hold_duration = std::max(params_.hold_duration, 0.0);
  }

  /// Compute a jerk-limited S-curve reference analytically.
  void calculateReference(
    double elapsed_time,
    double & velocity,
    double & acceleration,
    double & jerk,
    bool & is_defined_acceleration) const
  {
    const double segment_duration = params_.ramp_duration + params_.hold_duration;
    const std::size_t segment_count = params_.velocity_points.size() - 1;
    const double profile_duration = segment_duration * static_cast<double>(segment_count);

    if (profile_duration <= 0.0) {
      velocity = params_.velocity_points.back();
      acceleration = 0.0;
      jerk = 0.0;
      is_defined_acceleration = true;
      return;
    }

    double profile_time = elapsed_time;
    if (params_.loop) {
      profile_time = std::fmod(elapsed_time, profile_duration);
    } else if (elapsed_time >= profile_duration) {
      velocity = params_.velocity_points.back();
      acceleration = 0.0;
      jerk = 0.0;
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
      // Symmetric S-curve: ramp acceleration up, hold it, then ramp it down.
      // The area under this trapezoidal acceleration is exactly delta_velocity.
      const double ramp_time = params_.acceleration_ramp_duration;
      const double peak_acceleration =
        delta_velocity / (params_.ramp_duration - ramp_time);
      const double jerk_magnitude = peak_acceleration / ramp_time;
      const double deceleration_start = params_.ramp_duration - ramp_time;

      if (segment_time < ramp_time) {
        jerk = jerk_magnitude;
        acceleration = jerk * segment_time;
        velocity = start_velocity + 0.5 * jerk * segment_time * segment_time;
      } else if (segment_time < deceleration_start) {
        const double constant_acceleration_time = segment_time - ramp_time;
        jerk = 0.0;
        acceleration = peak_acceleration;
        velocity = start_velocity + 0.5 * peak_acceleration * ramp_time +
          peak_acceleration * constant_acceleration_time;
      } else {
        const double ramp_down_time = segment_time - deceleration_start;
        jerk = -jerk_magnitude;
        acceleration = peak_acceleration + jerk * ramp_down_time;
        const double ramp_down_start_velocity = start_velocity +
          peak_acceleration * (params_.ramp_duration - 1.5 * ramp_time);
        velocity = ramp_down_start_velocity +
          peak_acceleration * ramp_down_time +
          0.5 * jerk * ramp_down_time * ramp_down_time;
      }
      is_defined_acceleration = true;
    } else {
      // Hold phase: zero acceleration.
      velocity = end_velocity;
      acceleration = 0.0;
      jerk = 0.0;
      is_defined_acceleration = true;
    }
  }

  const Params & params() const {return params_;}

private:
  Params params_;
};

}  // namespace car_planning

#endif  // CAR_PLANNING__LONGITUDINAL_PROFILE_CORE_HPP_
