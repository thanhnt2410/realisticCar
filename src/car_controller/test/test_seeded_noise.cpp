// Copyright 2026 realisticCar project
// Unit tests for seeded noise behavior in LongitudinalVehicleModel
#include <gtest/gtest.h>
#include <cmath>
#include "car_controller/longitudinal_vehicle_model.hpp"

using car_controller::LongitudinalVehicleModel;

namespace
{
LongitudinalVehicleModel::Params noisy_params(uint64_t seed, double sigma)
{
  LongitudinalVehicleModel::Params p;
  p.actuator_delay_s = 0.0;
  p.actuator_lag_tau_s = 0.0;
  p.max_accel = 5.0;
  p.min_accel = -5.0;
  p.max_jerk = 100.0;
  p.min_jerk = -100.0;
  p.grade_percent = 0.0;
  p.rolling_resistance_coeff = 0.0;
  p.drag_coeff = 0.0;
  p.mass_kg = 1800.0;
  p.process_noise_sigma = sigma;
  p.random_seed = seed;
  return p;
}
}  // namespace

TEST(SeededNoise, ZeroSigmaAlwaysZeroNoise)
{
  LongitudinalVehicleModel model(noisy_params(42, 0.0));
  for (int i = 0; i < 50; ++i) {
    model.step(0.0, 0.02, 0.0);
    EXPECT_DOUBLE_EQ(model.last_a_noise(), 0.0);
  }
}

TEST(SeededNoise, SameSeedSameTrace)
{
  LongitudinalVehicleModel m1(noisy_params(1001, 0.1));
  LongitudinalVehicleModel m2(noisy_params(1001, 0.1));
  for (int i = 0; i < 100; ++i) {
    m1.step(1.0, 0.02, 0.0);
    m2.step(1.0, 0.02, 0.0);
    EXPECT_DOUBLE_EQ(m1.last_a_noise(), m2.last_a_noise());
    EXPECT_DOUBLE_EQ(m1.true_velocity(), m2.true_velocity());
  }
}

TEST(SeededNoise, DifferentSeedDifferentTrace)
{
  LongitudinalVehicleModel m1(noisy_params(1001, 0.1));
  LongitudinalVehicleModel m2(noisy_params(1002, 0.1));
  bool any_diff = false;
  for (int i = 0; i < 100; ++i) {
    m1.step(1.0, 0.02, 0.0);
    m2.step(1.0, 0.02, 0.0);
    if (m1.last_a_noise() != m2.last_a_noise()) {
      any_diff = true;
    }
  }
  EXPECT_TRUE(any_diff) << "Different seeds should produce different noise traces";
}

TEST(SeededNoise, SensorNoiseDoesNotChangeTrueState)
{
  // True state is determined by process dynamics + process noise only.
  // Sensor noise (in the ROS node) must not affect model's true_velocity().
  // Here we verify the model itself: run with zero process noise gives deterministic velocity.
  LongitudinalVehicleModel m_det(noisy_params(42, 0.0));
  LongitudinalVehicleModel m_det2(noisy_params(99, 0.0));  // different seed but sigma=0
  for (int i = 0; i < 50; ++i) {
    m_det.step(1.0, 0.02, 0.0);
    m_det2.step(1.0, 0.02, 0.0);
  }
  EXPECT_DOUBLE_EQ(m_det.true_velocity(), m_det2.true_velocity());
}
