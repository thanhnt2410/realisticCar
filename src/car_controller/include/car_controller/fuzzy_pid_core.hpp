// Copyright 2026 realisticCar project
#ifndef CAR_CONTROLLER__FUZZY_PID_CORE_HPP_
#define CAR_CONTROLLER__FUZZY_PID_CORE_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>

#define ADAPTIVE 0
#define TUNING 0
#define MIX 1

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
  double fuzzy_kp_min_ratio = 0.7;   ///< Lower bound of adaptive Kp as a fraction of base kp (> 0)
  double fuzzy_kd_min_ratio = 0.8;   ///< Lower bound of adaptive Kd as a fraction of base kd
  double gain_rate_limit = 0.3;      ///< Max fractional change of adaptive_kp per control step [fraction of kp]
  double max_integral_error = 5.0;
  double max_output = 2.0;
  double derivative_filter_alpha = 0.2;
};

/// ROS-free Fuzzy PID core that outputs acceleration correction [m/s^2].
///
/// Gains are adapted each cycle via a Sugeno-type fuzzy inference system.
///
/// Adaptive gain bounds:
///   adaptive_kp ∈ [kp_min_ratio*kp, 2.0*kp]  (kp_min_ratio ∈ (0,1], default 0.7)
///   adaptive_ki ∈ [ki*0.5,           2.0*ki]  (never zero during operation)
///   adaptive_kd ∈ [kd_min_ratio*kd,  2.0*kd]  (kd_min_ratio default 0.8)
///
/// Gain rate limiting:
///   |Δadaptive_kp| per step ≤ gain_rate_limit * kp
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
    fuzzy_kp_gain_(0.0), fuzzy_ki_gain_(0.0), fuzzy_kd_gain_(0.0),
    has_prev_error_(false)
  {
    // Clamp min ratios to safe values to guarantee gains > 0.
    params_.fuzzy_kp_min_ratio = std::max(0.1, std::min(1.0, params_.fuzzy_kp_min_ratio));
    params_.fuzzy_kd_min_ratio = std::max(0.1, std::min(1.0, params_.fuzzy_kd_min_ratio));
    params_.gain_rate_limit    = std::max(0.0, params_.gain_rate_limit);

    // These bounds only depend on immutable controller parameters. Compute
    // them once instead of repeating the same work in every fuzzy inference.
    kp_lo_ = params_.fuzzy_kp_min_ratio * params_.kp;
    kp_hi_ = 2.0 * params_.kp;
    ki_lo_ = (params_.ki > 0.0) ? 0.5 * params_.ki : 0.0;
    ki_hi_ = 2.0 * params_.ki;
    kd_lo_ = params_.fuzzy_kd_min_ratio * params_.kd;
    kd_hi_ = 2.0 * params_.kd;
    kp_max_delta_ = params_.gain_rate_limit * params_.kp;
#if ADAPTIVE
    fuzzy_kp_gain_ = params_.kp * 0.1 / 3.0;
    fuzzy_ki_gain_ = (params_.ki > 0.0) ? params_.ki * 0.02 / 3.0 : 0.0;
    fuzzy_kd_gain_ = params_.kd * 0.07 / 3.0;
#elif TUNING
    fuzzy_kp_gain_ = params_.kp * 1.0 / 3.0;
    fuzzy_ki_gain_ = (params_.ki > 0.0) ? params_.ki * 0.1 / 3.0 : 0.0;
    fuzzy_kd_gain_ = params_.kd * 0.30 / 3.0;
#elif MIX
    fuzzy_kp_gain_ = params_.kp * 0.1 / 3.0;
    // Ki gain increased ×5 (0.1 → 0.5): expands Ki adaptation range from ±2% → ±10%
    // of base ki, enabling better disturbance rejection (grade, drag, wind).
    fuzzy_ki_gain_ = (params_.ki > 0.0) ? params_.ki * 0.5 / 3.0 : 0.0;
    fuzzy_kd_gain_ = params_.kd * 0.30 / 3.0;
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
    // Reset gains to base values — always guaranteed > 0 because kp_min_ratio >= 0.1.
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

  // Rule bases are immutable. Keep one class-level copy instead of declaring
  // three automatic tables inside adaptGains() on every control cycle.
  inline static constexpr RuleTable KP_RULES {{
    {{PB, PB, PM, PM, PS, ZO, ZO}},
    {{PB, PB, PM, PS, PS, ZO, NS}},
    {{PM, PM, PM, PS, ZO, NS, NS}},
    {{PM, PM, PS, ZO, NS, NM, NM}},
    {{PS, PS, ZO, NS, NS, NM, NM}},
    {{PS, ZO, NS, NM, NM, NM, NB}},
    {{ZO, ZO, NM, NM, NM, NB, NB}},
  }};
  inline static constexpr RuleTable KI_RULES {{
    {{NS, NS, NS, NS, ZO, ZO, ZO}},
    {{NS, NS, NS, ZO, ZO, PS, PS}},
    {{NS, NS, ZO, ZO, PS, PS, PS}},
    {{NM, NM, NS, ZO, PS, PM, PM}},
    {{ZO, ZO, PS, PS, PS, PM, PM}},
    {{ZO, PS, PS, PM, PM, PM, PB}},
    {{ZO, PS, PM, PM, PB, PB, PB}},
  }};
  inline static constexpr RuleTable KD_RULES {{
    {{PS, NS, NB, NB, NB, NM, PS}},
    {{PS, NS, NB, NM, NM, NS, ZO}},
    {{ZO, NS, NM, NM, NS, NS, ZO}},
    {{ZO, NS, NS, NS, NS, NS, ZO}},
    {{ZO, ZO, ZO, ZO, ZO, ZO, ZO}},
    {{PB, NS, PS, PS, PS, PS, PB}},
    {{PB, PM, PM, PM, PS, PS, PM}},
  }};

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

  static std::array<double, 7> fuzzify(double u)
  {
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

    double sum_kp = 0.0, sum_ki = 0.0, sum_kd = 0.0, w_sum = 0.0;
    for (std::size_t i = 0; i < 7; ++i) {
      if (me[i] <= 0.0) {continue;}
      for (std::size_t j = 0; j < 7; ++j) {
        if (md[j] <= 0.0) {continue;}
        const double w = me[i] * md[j];
        sum_kp += w * static_cast<double>(KP_RULES[i][j]);
        sum_ki += w * static_cast<double>(KI_RULES[i][j]);
        sum_kd += w * static_cast<double>(KD_RULES[i][j]);
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
    {
      const double kp_raw = adaptive_kp_ + fuzzy_kp_gain_ * sum_kp / w_sum;
      const double kp_delta =
        std::clamp(kp_raw - adaptive_kp_, -kp_max_delta_, kp_max_delta_);
      adaptive_kp_ = std::clamp(adaptive_kp_ + kp_delta, kp_lo_, kp_hi_);
    }
    adaptive_ki_ = std::clamp(
      adaptive_ki_ + fuzzy_ki_gain_ * sum_ki / w_sum, ki_lo_, ki_hi_);
    adaptive_kd_ = std::clamp(
      adaptive_kd_ + fuzzy_kd_gain_ * sum_kd / w_sum, kd_lo_, kd_hi_);
#elif TUNING
    {
      const double kp_raw = params_.kp + fuzzy_kp_gain_ * sum_kp / w_sum;
      const double kp_delta =
        std::clamp(kp_raw - adaptive_kp_, -kp_max_delta_, kp_max_delta_);
      adaptive_kp_ = std::clamp(adaptive_kp_ + kp_delta, kp_lo_, kp_hi_);
    }
    adaptive_ki_ = std::clamp(
      params_.ki + fuzzy_ki_gain_ * sum_ki / w_sum, ki_lo_, ki_hi_);
    adaptive_kd_ = std::clamp(
      params_.kd + fuzzy_kd_gain_ * sum_kd / w_sum, kd_lo_, kd_hi_);
#elif MIX
    {
      const double kp_raw = adaptive_kp_ + fuzzy_kp_gain_ * sum_kp / w_sum;
      const double kp_delta =
        std::clamp(kp_raw - adaptive_kp_, -kp_max_delta_, kp_max_delta_);
      adaptive_kp_ = std::clamp(adaptive_kp_ + kp_delta, kp_lo_, kp_hi_);
    }
    adaptive_ki_ = std::clamp(
      params_.ki + fuzzy_ki_gain_ * sum_ki / w_sum, ki_lo_, ki_hi_);
    adaptive_kd_ = std::clamp(
      params_.kd + fuzzy_kd_gain_ * sum_kd / w_sum, kd_lo_, kd_hi_);
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
  double kp_lo_{0.0}, kp_hi_{0.0};
  double ki_lo_{0.0}, ki_hi_{0.0};
  double kd_lo_{0.0}, kd_hi_{0.0};
  double kp_max_delta_{0.0};
  double last_error_{0.0};
  bool has_prev_error_;
};

}  // namespace car_controller

#endif  // CAR_CONTROLLER__FUZZY_PID_CORE_HPP_
