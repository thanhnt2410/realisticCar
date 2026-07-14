// Copyright 2026 realisticCar project
#ifndef CAR_CONTROLLER__LONGITUDINAL_VEHICLE_MODEL_HPP_
#define CAR_CONTROLLER__LONGITUDINAL_VEHICLE_MODEL_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <random>

namespace car_controller
{

/// Parameters for LongitudinalVehicleModel.
/// Defined outside the class to avoid GCC C++17 default-argument bug
/// (default member initializer required before end of enclosing class).
struct LongitudinalVehicleModelParams
{
  // Actuator.
  double actuator_delay_s = 0.1;           ///< Command delay [s]
  double actuator_lag_tau_s = 0.15;        ///< First-order lag time constant [s]
  double max_accel = 2.0;                  ///< Actuator accel clamp (positive) [m/s^2]
  double min_accel = -3.0;                 ///< Actuator accel clamp (negative) [m/s^2]
  double max_jerk = 5.0;                   ///< Jerk limit (positive) [m/s^3]
  double min_jerk = -5.0;                  ///< Jerk limit (negative) [m/s^3]
  // Road.
  double grade_percent = 0.0;              ///< Road grade [%] (+uphill, -downhill)
  double rolling_resistance_coeff = 0.01;
  double drag_coeff = 0.00025;             ///< F_drag/m ≈ drag_coeff * v^2 [1/m]
  double mass_kg = 1800.0;                 ///< Vehicle mass [kg]
  // Noise.
  double process_noise_sigma = 0.0;        ///< Process noise std-dev [m/s^2]
  uint64_t random_seed = 42;
  // Gravity.
  double gravity = 9.81;
};

/// 1-D longitudinal vehicle model – the authoritative plant.
///
/// Signal chain per step():
///   1. Command delay (circular buffer)
///   2. Acceleration clamp
///   3. First-order actuator lag  (discrete Euler)
///   4. Jerk / rate limit
///   5. External forces: grade + rolling resistance + aerodynamic drag
///   6. Process disturbance (seeded Gaussian)
///   7. Euler integration: v[k+1] = max(0, v[k] + a_total * dt)
///
/// Sensor noise is added externally by the ROS node.
class LongitudinalVehicleModel
{
public:
  using Params = LongitudinalVehicleModelParams;

  explicit LongitudinalVehicleModel(const Params & p = Params())
  : params_(p),
    true_velocity_(0.0),
    applied_acceleration_(0.0),
    last_applied_(0.0),
    sim_step_(0)
  {
    const std::size_t delay_steps = computeDelaySteps(p.actuator_delay_s, 0.02);
    delay_buffer_.assign(delay_steps + 1, 0.0);
    rng_.seed(static_cast<uint64_t>(p.random_seed));
  }

  /// Advance the model by one fixed step of duration dt [s].
  void step(double commanded_accel, double dt, double grade_percent_override)
  {
    if (dt <= 0.0) {return;}

    // 1. Delay.
    const std::size_t delay_steps = computeDelaySteps(params_.actuator_delay_s, dt);
    double delayed_cmd = commanded_accel;
    if (delay_steps > 0) {
      while (delay_buffer_.size() < delay_steps + 1) {
        delay_buffer_.push_front(0.0);
      }
      delay_buffer_.push_front(commanded_accel);
      delayed_cmd = delay_buffer_.back();
      while (delay_buffer_.size() > delay_steps + 1) {
        delay_buffer_.pop_back();
      }
    }

    // 2. Clamp.
    const double clamped_cmd = std::clamp(delayed_cmd, params_.min_accel, params_.max_accel);

    // 3. First-order lag.
    if (params_.actuator_lag_tau_s > 0.0) {
      const double alpha = dt / (params_.actuator_lag_tau_s + dt);
      applied_acceleration_ += alpha * (clamped_cmd - applied_acceleration_);
    } else {
      applied_acceleration_ = clamped_cmd;
    }

    // 4. Jerk limit.
    const double max_delta = params_.max_jerk * dt;
    const double min_delta = params_.min_jerk * dt;
    const double delta = std::clamp(applied_acceleration_ - last_applied_, min_delta, max_delta);
    applied_acceleration_ = last_applied_ + delta;
    last_applied_ = applied_acceleration_;

    // 5. External forces.
    const double grade_angle = std::atan(grade_percent_override / 100.0);
    const double a_grade = -params_.gravity * std::sin(grade_angle);
    const double a_rolling = (true_velocity_ > 0.05) ?
      -params_.rolling_resistance_coeff * params_.gravity : 0.0;
    const double a_drag = -params_.drag_coeff * true_velocity_ * std::abs(true_velocity_);

    // 6. Process noise.
    double a_noise = 0.0;
    if (params_.process_noise_sigma > 0.0) {
      std::normal_distribution<double> dist(0.0, params_.process_noise_sigma);
      a_noise = dist(rng_);
    }

    // 7. Integrate.
    const double a_total = applied_acceleration_ + a_grade + a_rolling + a_drag + a_noise;
    true_velocity_ = std::max(0.0, true_velocity_ + a_total * dt);

    last_a_grade_ = a_grade;
    last_a_rolling_ = a_rolling;
    last_a_drag_ = a_drag;
    last_a_noise_ = a_noise;
    last_a_total_ = a_total;
    last_delayed_cmd_ = delayed_cmd;
    ++sim_step_;
  }

  // --- Accessors ---
  double   true_velocity()        const {return true_velocity_;}
  double   applied_acceleration() const {return applied_acceleration_;}
  double   last_a_grade()         const {return last_a_grade_;}
  double   last_a_rolling()       const {return last_a_rolling_;}
  double   last_a_drag()          const {return last_a_drag_;}
  double   last_a_noise()         const {return last_a_noise_;}
  double   last_a_total()         const {return last_a_total_;}
  double   last_delayed_cmd()     const {return last_delayed_cmd_;}
  uint64_t sim_step()             const {return sim_step_;}
  const Params & params()         const {return params_;}

  void set_grade_percent(double g) {params_.grade_percent = g;}

  void reset()
  {
    true_velocity_ = 0.0;
    applied_acceleration_ = 0.0;
    last_applied_ = 0.0;
    sim_step_ = 0;
    std::fill(delay_buffer_.begin(), delay_buffer_.end(), 0.0);
    last_a_grade_ = last_a_rolling_ = last_a_drag_ = last_a_noise_ = last_a_total_ = 0.0;
    last_delayed_cmd_ = 0.0;
    rng_.seed(params_.random_seed);
  }

private:
  static std::size_t computeDelaySteps(double delay_s, double dt)
  {
    if (delay_s <= 0.0 || dt <= 0.0) {return 0;}
    return static_cast<std::size_t>(std::round(delay_s / dt));
  }

  Params params_;
  double true_velocity_;
  double applied_acceleration_;
  double last_applied_;
  std::deque<double> delay_buffer_;
  std::mt19937_64 rng_;
  uint64_t sim_step_;

  double last_a_grade_{0.0};
  double last_a_rolling_{0.0};
  double last_a_drag_{0.0};
  double last_a_noise_{0.0};
  double last_a_total_{0.0};
  double last_delayed_cmd_{0.0};
};

}  // namespace car_controller

#endif  // CAR_CONTROLLER__LONGITUDINAL_VEHICLE_MODEL_HPP_
