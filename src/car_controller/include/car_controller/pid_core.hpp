// Copyright 2026 realisticCar project
#ifndef CAR_CONTROLLER__PID_CORE_HPP_
#define CAR_CONTROLLER__PID_CORE_HPP_

#include <algorithm>
#include <cmath>

namespace car_controller
{

/// Parameters for PidCore.
/// Defined outside class to avoid GCC C++17 default-argument nested struct bug.
struct PidCoreParams
{
  double kp = 1.0;
  double ki = 0.0;
  double kd = 0.05;
  double max_integral_error = 5.0;       ///< Clamp on integral accumulator [m*s]
  double max_output = 2.0;               ///< Output clamp on correction [m/s^2]
  double derivative_filter_alpha = 0.2; ///< EMA filter coefficient (0=off, 1=raw)
};

/// ROS-free PID core that outputs acceleration correction [m/s^2].
///
/// Gain units (NOT legacy velocity-correction gains):
///   Kp  [1/s]           – velocity error [m/s] -> acceleration [m/s^2]
///   Ki  [1/s^2]         – integral of error [m] -> acceleration [m/s^2]
///   Kd  [dimensionless] – error rate [m/s^2] -> acceleration [m/s^2]
///
/// Anti-windup: conditional-integration scheme.
class PidCore
{
public:
  using Params = PidCoreParams;

  explicit PidCore(const Params & params = Params())
  : params_(params), integral_(0.0), prev_error_(0.0),
    filtered_derivative_(0.0), has_prev_error_(false)
  {}

  /// Compute acceleration correction for one control cycle.
  /// @param error  Velocity error e_v = v_ref - v_measured [m/s]
  /// @param dt     Time step [s]; must be > 0
  /// @return Acceleration correction [m/s^2], clamped to ±max_output
  double update(double error, double dt)
  {
    if (dt <= 0.0 || std::isnan(dt)) {
      return 0.0;
    }

    const double raw_derivative = has_prev_error_ ? (error - prev_error_) / dt : 0.0;
    filtered_derivative_ =
      params_.derivative_filter_alpha * raw_derivative +
      (1.0 - params_.derivative_filter_alpha) * filtered_derivative_;

    const double prev_integral = integral_;
    integral_ = std::clamp(
      integral_ + error * dt,
      -params_.max_integral_error,
      params_.max_integral_error);

    double output = computeOutput(error);

    // Conditional-integration anti-windup.
    const bool saturated =
      params_.max_output > 0.0 &&
      std::abs(output) >= params_.max_output - 1e-9;
    if (saturated && error * output > 0.0) {
      integral_ = prev_integral;
      output = computeOutput(error);
    }

    prev_error_ = error;
    has_prev_error_ = true;

    return std::clamp(output, -params_.max_output, params_.max_output);
  }

  void reset()
  {
    integral_ = 0.0;
    prev_error_ = 0.0;
    filtered_derivative_ = 0.0;
    has_prev_error_ = false;
  }

  double p_term() const {return params_.kp * last_error_;}
  double i_term() const {return params_.ki * integral_;}
  double d_term() const {return params_.kd * filtered_derivative_;}
  double integral() const {return integral_;}
  double filtered_derivative() const {return filtered_derivative_;}
  const Params & params() const {return params_;}

private:
  double computeOutput(double error)
  {
    last_error_ = error;
    return params_.kp * error +
           params_.ki * integral_ +
           params_.kd * filtered_derivative_;
  }

  Params params_;
  double integral_;
  double prev_error_;
  double filtered_derivative_;
  double last_error_{0.0};
  bool has_prev_error_;
};

}  // namespace car_controller

#endif  // CAR_CONTROLLER__PID_CORE_HPP_
