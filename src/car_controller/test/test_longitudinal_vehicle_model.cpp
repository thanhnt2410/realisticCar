// Copyright 2026 realisticCar project
// Unit tests for LongitudinalVehicleModel
#include <gtest/gtest.h>
#include <cmath>
#include "car_controller/longitudinal_vehicle_model.hpp"

using car_controller::LongitudinalVehicleModel;

namespace
{
LongitudinalVehicleModel::Params flat_params()
{
  LongitudinalVehicleModel::Params p;
  p.actuator_delay_s = 0.0;
  p.actuator_lag_tau_s = 0.0;  // ideal: applied immediately
  p.max_accel = 5.0;
  p.min_accel = -5.0;
  p.max_jerk = 100.0;
  p.min_jerk = -100.0;
  p.grade_percent = 0.0;
  p.rolling_resistance_coeff = 0.0;
  p.drag_coeff = 0.0;
  p.mass_kg = 1800.0;
  p.process_noise_sigma = 0.0;
  p.random_seed = 42;
  return p;
}
}  // namespace

TEST(LongitudinalVehicleModel, ZeroCommand)
{
  LongitudinalVehicleModel model(flat_params());
  model.step(0.0, 0.02, 0.0);
  EXPECT_DOUBLE_EQ(model.true_velocity(), 0.0);
}

TEST(LongitudinalVehicleModel, PositiveAccelIncreases)
{
  LongitudinalVehicleModel model(flat_params());
  model.step(2.0, 0.1, 0.0);
  EXPECT_GT(model.true_velocity(), 0.0);
}

TEST(LongitudinalVehicleModel, NoNegativeVelocity)
{
  LongitudinalVehicleModel model(flat_params());
  // Large deceleration from rest – velocity should not go below 0.
  model.step(-5.0, 1.0, 0.0);
  EXPECT_GE(model.true_velocity(), 0.0);
}

TEST(LongitudinalVehicleModel, UphillReducesAcceleration)
{
  auto p = flat_params();
  p.rolling_resistance_coeff = 0.0;
  p.drag_coeff = 0.0;
  LongitudinalVehicleModel model_flat(p);
  LongitudinalVehicleModel model_uphill(p);
  model_flat.step(2.0, 0.1, 0.0);
  model_uphill.step(2.0, 0.1, 5.0);  // 5% grade uphill
  EXPECT_LT(model_uphill.true_velocity(), model_flat.true_velocity());
}

TEST(LongitudinalVehicleModel, DownhillIncreasesVelocity)
{
  auto p = flat_params();
  LongitudinalVehicleModel model_flat(p);
  LongitudinalVehicleModel model_downhill(p);
  model_flat.step(0.0, 1.0, 0.0);
  model_downhill.step(0.0, 1.0, -5.0);  // -5% grade downhill
  EXPECT_GT(model_downhill.true_velocity(), model_flat.true_velocity());
}

TEST(LongitudinalVehicleModel, AccelClamp)
{
  auto p = flat_params();
  p.max_accel = 1.0;
  p.min_accel = -1.0;
  p.actuator_lag_tau_s = 0.0;
  LongitudinalVehicleModel model(p);
  model.step(10.0, 1.0, 0.0);  // commanded 10, clamped to 1
  EXPECT_NEAR(model.true_velocity(), 1.0, 0.01);
}

TEST(LongitudinalVehicleModel, NoNanWithZeroDt)
{
  LongitudinalVehicleModel model(flat_params());
  model.step(2.0, 0.0, 0.0);  // should be a no-op
  EXPECT_FALSE(std::isnan(model.true_velocity()));
  EXPECT_DOUBLE_EQ(model.true_velocity(), 0.0);
}

TEST(LongitudinalVehicleModel, NoNoiseWhenSigmaZero)
{
  auto p = flat_params();
  p.process_noise_sigma = 0.0;
  LongitudinalVehicleModel model(p);
  model.step(1.0, 0.02, 0.0);
  EXPECT_DOUBLE_EQ(model.last_a_noise(), 0.0);
}

TEST(LongitudinalVehicleModel, SameSeedSameNoise)
{
  auto p = flat_params();
  p.process_noise_sigma = 0.1;
  p.random_seed = 42;
  LongitudinalVehicleModel m1(p), m2(p);
  for (int i = 0; i < 10; ++i) {
    m1.step(0.0, 0.02, 0.0);
    m2.step(0.0, 0.02, 0.0);
  }
  EXPECT_DOUBLE_EQ(m1.true_velocity(), m2.true_velocity());
}
