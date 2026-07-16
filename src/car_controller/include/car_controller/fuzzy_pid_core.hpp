// Copyright 2026 realisticCar project
#ifndef CAR_CONTROLLER__FUZZY_PID_CORE_HPP_
#define CAR_CONTROLLER__FUZZY_PID_CORE_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

namespace car_controller
{

struct FuzzyPidCoreParams
{
  // Initial gains (used before the first inference and after reset).
  double kp = 1.0;
  double ki = 0.0;
  double kd = 0.05;

  // Input scaling. Scaled e and de are clipped to [-1, 1].
  double fuzzy_error_gain = 0.8;
  double fuzzy_error_derivative_gain = 0.5;

  // Experimental gain ranges in equations (4)-(6) of JMST.7.
  double kp_min = 0.0;
  double kp_max = 2.0;
  double ki_min = 0.0;
  double ki_max = 0.0;
  double kd_min = 0.0;
  double kd_max = 0.1;

  double max_integral_error = 5.0;
  double max_output = 2.0;
  double derivative_filter_alpha = 0.2;
};

/// ROS-free self-tuning fuzzy PID core.
///
/// This implements Table 1 of JMST.7: two five-term inputs (NB, NS, Z, PS,
/// PB), three seven-level zero-order Sugeno outputs (VS ... VB), followed by
/// the direct range transformations in equations (4)-(6). The gains are
/// recomputed from the current e/de every cycle; fuzzy outputs are not deltas.
class FuzzyPidCore
{
public:
  using Params = FuzzyPidCoreParams;

  explicit FuzzyPidCore(const Params & params = Params())
  : params_(params), adaptive_kp_(params.kp), adaptive_ki_(params.ki),
    adaptive_kd_(params.kd)
  {
    sanitizeRanges();
  }

  double update(double error, double dt)
  {
    if (dt <= 0.0 || std::isnan(dt)) {return 0.0;}

    const double raw_derivative = has_prev_error_ ? (error - prev_error_) / dt : 0.0;
    filtered_derivative_ =
      params_.derivative_filter_alpha * raw_derivative +
      (1.0 - params_.derivative_filter_alpha) * filtered_derivative_;
    adaptGains(error, filtered_derivative_);

    const double prev_integral = integral_;
    integral_ = std::clamp(
      integral_ + error * dt, -params_.max_integral_error, params_.max_integral_error);
    double output = computeOutput(error);
    const bool saturated = params_.max_output > 0.0 &&
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
    adaptive_kp_ = params_.kp;
    adaptive_ki_ = params_.ki;
    adaptive_kd_ = params_.kd;
    has_prev_error_ = false;
  }

  double adaptive_kp() const {return adaptive_kp_;}
  double adaptive_ki() const {return adaptive_ki_;}
  double adaptive_kd() const {return adaptive_kd_;}
  double p_term() const {return adaptive_kp_ * last_error_;}
  double i_term() const {return adaptive_ki_ * integral_;}
  double d_term() const {return adaptive_kd_ * filtered_derivative_;}
  double integral() const {return integral_;}
  double filtered_derivative() const {return filtered_derivative_;}
  double fuzzy_error_input() const {return fuzzy_error_input_;}
  double fuzzy_error_change_input() const {return fuzzy_error_change_input_;}
  double fuzzy_inference_duration_us() const {return fuzzy_inference_duration_us_;}
  const Params & params() const {return params_;}

private:
  // VS, S, MS, M, MB, B, VB map uniformly to K' in [0, 1].
  enum OutputLevel : int {VS = 0, S = 1, MS = 2, M = 3, MB = 4, B = 5, VB = 6};
  using RuleTable = std::array<std::array<OutputLevel, 5>, 5>;

  void sanitizeRanges()
  {
    if (params_.kp_min > params_.kp_max) {std::swap(params_.kp_min, params_.kp_max);}
    if (params_.ki_min > params_.ki_max) {std::swap(params_.ki_min, params_.ki_max);}
    if (params_.kd_min > params_.kd_max) {std::swap(params_.kd_min, params_.kd_max);}
  }

  static double trimf(double v, double l, double c, double r)
  {
    if (v <= l || v >= r) {return 0.0;}
    if (v == c) {return 1.0;}
    return v < c ? (v - l) / (c - l) : (r - v) / (r - c);
  }

  static std::array<double, 5> fuzzify(double value)
  {
    const double v = std::clamp(value, -1.0, 1.0);
    // Shoulder functions ensure full membership at the clipped endpoints.
    const double nb = v <= -1.0 ? 1.0 : (v < -0.5 ? (-0.5 - v) / 0.5 : 0.0);
    const double pb = v >= 1.0 ? 1.0 : (v > 0.5 ? (v - 0.5) / 0.5 : 0.0);
    return {nb, trimf(v, -1.0, -0.5, 0.0), trimf(v, -0.5, 0.0, 0.5),
      trimf(v, 0.0, 0.5, 1.0), pb};
  }

  static double mapGain(double normalized, double minimum, double maximum)
  {
    return (maximum - minimum) * normalized + minimum;
  }

  void adaptGains(double error, double error_derivative)
  {
    const auto start = std::chrono::steady_clock::now();
    fuzzy_error_input_ = std::clamp(params_.fuzzy_error_gain * error, -1.0, 1.0);
    fuzzy_error_change_input_ = std::clamp(
      params_.fuzzy_error_derivative_gain * error_derivative, -1.0, 1.0);
    const auto me = fuzzify(fuzzy_error_input_);
    const auto mde = fuzzify(fuzzy_error_change_input_);

    // Rows: e = NB, NS, Z, PS, PB. Columns: de = NB, NS, Z, PS, PB.
    constexpr RuleTable kp_rules {{
      {{VB, MB, MB, MB, MB}}, {{MB, MB, MB, B, MB}}, {{VS, VS, S, MS, MS}},
      {{MB, MB, MB, B, MB}}, {{MB, MB, MB, MB, MB}}
    }};
    constexpr RuleTable ki_rules {{
      {{M, M, M, M, M}}, {{MS, MS, MS, MS, MS}}, {{S, S, VS, S, S}},
      {{MS, MS, MS, MS, MS}}, {{M, M, M, M, M}}
    }};
    constexpr RuleTable kd_rules {{
      {{VS, MB, M, B, MB}}, {{MS, MB, B, MB, MB}}, {{M, B, B, MB, MB}},
      {{MB, MB, MB, MB, MB}}, {{MB, MB, MB, MB, MB}}
    }};

    double kp_sum = 0.0, ki_sum = 0.0, kd_sum = 0.0, weight_sum = 0.0;
    for (std::size_t i = 0; i < 5; ++i) {
      for (std::size_t j = 0; j < 5; ++j) {
        const double w = me[i] * mde[j];
        kp_sum += w * static_cast<double>(kp_rules[i][j]) / 6.0;
        ki_sum += w * static_cast<double>(ki_rules[i][j]) / 6.0;
        kd_sum += w * static_cast<double>(kd_rules[i][j]) / 6.0;
        weight_sum += w;
      }
    }
    if (weight_sum > 1e-9) {
      adaptive_kp_ = mapGain(kp_sum / weight_sum, params_.kp_min, params_.kp_max);
      adaptive_ki_ = mapGain(ki_sum / weight_sum, params_.ki_min, params_.ki_max);
      adaptive_kd_ = mapGain(kd_sum / weight_sum, params_.kd_min, params_.kd_max);
    }
    fuzzy_inference_duration_us_ = std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - start).count();
  }

  double computeOutput(double error)
  {
    last_error_ = error;
    return adaptive_kp_ * error + adaptive_ki_ * integral_ +
           adaptive_kd_ * filtered_derivative_;
  }

  Params params_;
  double integral_{0.0}, prev_error_{0.0}, filtered_derivative_{0.0};
  double fuzzy_error_input_{0.0}, fuzzy_error_change_input_{0.0};
  double fuzzy_inference_duration_us_{0.0};
  double adaptive_kp_, adaptive_ki_, adaptive_kd_;
  double last_error_{0.0};
  bool has_prev_error_{false};
};

}  // namespace car_controller

#endif  // CAR_CONTROLLER__FUZZY_PID_CORE_HPP_
