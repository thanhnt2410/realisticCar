// Copyright 2026 realisticCar project
#ifndef CAR_CONTROLLER__FUZZY_PID_CORE_HPP_
#define CAR_CONTROLLER__FUZZY_PID_CORE_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#define ADAPTIVE 0
#define TUNING 1

namespace car_controller
{

/// Parameters for FuzzyPidCore.
/// Defined outside class to avoid GCC C++17 default-argument nested struct bug.
struct FuzzyPidCoreParams
{
  double kp = 1.0;
  double ki = 0.0;
  double kd = 0.05;
  double fuzzy_error_gain = 0.8;
  double fuzzy_error_derivative_gain = 0.5;
  double fuzzy_kd_min_ratio = 0.5;
  double max_integral_error = 5.0;
  double max_output = 2.0;
  double derivative_filter_alpha = 0.2;
};

/// ROS-free Fuzzy PID core that outputs acceleration correction [m/s^2].
///
/// Gains are adapted each cycle via a Sugeno-type fuzzy inference system.
///
/// Adaptive gain bounds:
///   adaptive_kp ∈ [0.7*kp, 2.0*kp]
///   adaptive_ki ∈ [0.0,    2.0*ki]
///   adaptive_kd ∈ [kd_min_ratio*kd, 2.0*kd]
class FuzzyPidCore
{
public:
  using Params = FuzzyPidCoreParams;

  explicit FuzzyPidCore(const Params & params = Params())
  : params_(params),
    integral_(0.0), prev_error_(0.0),
    filtered_derivative_(0.0),
    adaptive_kp_(params.kp),
    adaptive_ki_(params.ki),
    adaptive_kd_(params.kd),
    has_prev_error_(false)
  {
#if ADAPTIVE
    fuzzy_kp_gain_ = params_.kp * 0.1 / 3.0;
    fuzzy_ki_gain_ = (params_.ki > 0.0) ? params_.ki * 0.02 / 3.0 : 0.0;
    fuzzy_kd_gain_ = params_.kd * 0.07 / 3.0;
#elif TUNING
    fuzzy_kp_gain_ = params_.kp * 0.8 / 3.0;
    fuzzy_ki_gain_ = (params_.ki > 0.0) ? params_.ki * 0.1 / 3.0 : 0.0;
    fuzzy_kd_gain_ = params_.kd * 0.35 / 3.0;
#endif
  }

  double update(double error, double dt)
  {
    if (dt <= 0.0 || std::isnan(dt)) {
      return 0.0;
    }

    const double raw_derivative = has_prev_error_ ? (error - prev_error_) / dt : 0.0;
    filtered_derivative_ =
      params_.derivative_filter_alpha * raw_derivative +
      (1.0 - params_.derivative_filter_alpha) * filtered_derivative_;

    adaptGains(error, filtered_derivative_);

    const double prev_integral = integral_;
    integral_ = std::clamp(
      integral_ + error * dt,
      -params_.max_integral_error,
      params_.max_integral_error);

    double output = computeOutput(error);

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
  enum FuzzyLabel : int
  {
    NB = -3,
    NM = -2,
    NS = -1,
    ZO = 0,
    PS = 1,
    PM = 2,
    PB = 3,
  };

  using RuleTable = std::array<std::array<FuzzyLabel, 7>, 7>;

  static double trimf(double v, double l, double c, double r)
  {
    if (v <= l || v >= r) {return 0.0;}
    if (v == c) {return 1.0;}
    return v < c ? (v - l) / (c - l) : (r - v) / (r - c);
  }

  static double gaussmf(double v, double sigma, double center)
  {
    const double z = (v - center) / sigma;
    return std::exp(-0.5 * z * z);
  }

  static std::array<double, 7> fuzzify(double v)
  {
    const double u = std::clamp(v, -3.0, 3.0);
    return {
      gaussmf(u, 0.6, -3.0),
      trimf(u, -3.0, -2.0, -1.0),
      trimf(u, -2.0, -1.0, 0.0),
      trimf(u, -1.0, 0.0, 1.0),
      trimf(u, 0.0, 1.0, 2.0),
      trimf(u, 1.0, 2.0, 3.0),
      gaussmf(u, 0.6, 3.0),
    };
  }

  void adaptGains(double error, double error_derivative)
  {
    const auto inference_start = std::chrono::steady_clock::now();

    const double ne = std::clamp(params_.fuzzy_error_gain * error, -3.0, 3.0);
    const double nde =
      std::clamp(params_.fuzzy_error_derivative_gain * error_derivative, -3.0, 3.0);
    fuzzy_error_input_ = ne;
    fuzzy_error_change_input_ = nde;
    const auto me = fuzzify(ne);
    const auto md = fuzzify(nde);

    constexpr RuleTable kp_rules {{
      {{PB, PB, PM, PM, PS, ZO, ZO}},
      {{PB, PB, PM, PS, PS, ZO, NS}},
      {{PM, PM, PM, PS, ZO, NS, NS}},
      {{PM, PM, PS, ZO, NS, NM, NM}},
      {{PS, PS, ZO, NS, NS, NM, NM}},
      {{PS, ZO, NS, NM, NM, NM, NB}},
      {{ZO, ZO, NM, NM, NM, NB, NB}},
    }};
    constexpr RuleTable ki_rules {{
      {{NS, NS, NS, NS, ZO, ZO, ZO}},
      {{NS, NS, NS, ZO, ZO, PS, PS}},
      {{NS, NS, ZO, ZO, PS, PS, PS}},
      {{NM, NM, NS, ZO, PS, PM, PM}},
      {{ZO, ZO, PS, PS, PS, PM, PM}},
      {{ZO, PS, PS, PM, PM, PM, PB}},
      {{ZO, PS, PM, PM, PB, PB, PB}},
    }};
    constexpr RuleTable kd_rules {{
      {{PS, NS, NB, NB, NB, NM, PS}},
      {{PS, NS, NB, NM, NM, NS, ZO}},
      {{ZO, NS, NM, NM, NS, NS, ZO}},
      {{ZO, NS, NS, NS, NS, NS, ZO}},
      {{ZO, ZO, ZO, ZO, ZO, ZO, ZO}},
      {{PB, NS, PS, PS, PS, PS, PB}},
      {{PB, PM, PM, PM, PS, PS, PM}},
    }};

    double sum_kp = 0.0, sum_ki = 0.0, sum_kd = 0.0, w_sum = 0.0;
    for (std::size_t i = 0; i < 7; ++i) {
      for (std::size_t j = 0; j < 7; ++j) {
        const double w = me[i] * md[j];
        if (w <= 0.0) {continue;}
        sum_kp += w * static_cast<double>(kp_rules[i][j]);
        sum_ki += w * static_cast<double>(ki_rules[i][j]);
        sum_kd += w * static_cast<double>(kd_rules[i][j]);
        w_sum += w;
      }
    }

    if (w_sum <= 1e-6) {
      const auto inference_end = std::chrono::steady_clock::now();
      fuzzy_inference_duration_us_ =
        std::chrono::duration<double, std::micro>(inference_end - inference_start).count();
      return;
    }
#if ADAPTIVE
    adaptive_kp_ = std::clamp(
      adaptive_kp_ + fuzzy_kp_gain_ * sum_kp / w_sum,
      0.7 * params_.kp, 2.0 * params_.kp);
    adaptive_ki_ = std::clamp(
      adaptive_ki_ + fuzzy_ki_gain_ * sum_ki / w_sum,
      0.0, 2.0 * params_.ki);
    adaptive_kd_ = std::clamp(
      adaptive_kd_ + fuzzy_kd_gain_ * sum_kd / w_sum,
      params_.fuzzy_kd_min_ratio * params_.kd, 2.0 * params_.kd);
#elif TUNING
    adaptive_kp_ = std::clamp(
      params_.kp + fuzzy_kp_gain_ * sum_kp / w_sum, 0.7 * params_.kp, 2.0 * params_.kp);
    adaptive_ki_ = std::clamp(
      params_.ki + fuzzy_ki_gain_ * sum_ki / w_sum, 0.0, 2.0 * params_.ki);
    adaptive_kd_ = std::clamp(
      params_.kd + fuzzy_kd_gain_ * sum_kd / w_sum, params_.fuzzy_kd_min_ratio * params_.kd, 2.0 * params_.kd);
#endif
    const auto inference_end = std::chrono::steady_clock::now();
    fuzzy_inference_duration_us_ =
      std::chrono::duration<double, std::micro>(inference_end - inference_start).count();
  }

  double computeOutput(double error)
  {
    last_error_ = error;
    return adaptive_kp_ * error +
           adaptive_ki_ * integral_ +
           adaptive_kd_ * filtered_derivative_;
  }

  Params params_;
  double integral_;
  double prev_error_;
  double filtered_derivative_;
  double fuzzy_error_input_{0.0};
  double fuzzy_error_change_input_{0.0};
  double fuzzy_inference_duration_us_{0.0};
  double adaptive_kp_, adaptive_ki_, adaptive_kd_;
  double fuzzy_kp_gain_, fuzzy_ki_gain_, fuzzy_kd_gain_;
  double last_error_{0.0};
  bool has_prev_error_;
};

}  // namespace car_controller

#endif  // CAR_CONTROLLER__FUZZY_PID_CORE_HPP_
