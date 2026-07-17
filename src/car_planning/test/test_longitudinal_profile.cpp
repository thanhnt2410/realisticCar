// Copyright 2026 realisticCar project
#include "gtest/gtest.h"
#include "car_planning/longitudinal_profile_core.hpp"

using car_planning::LongitudinalProfileCore;
using car_planning::LongitudinalProfileCoreParams;

TEST(LongitudinalProfileCore, ScurveRampAndHold)
{
  LongitudinalProfileCoreParams params;
  params.velocity_points = {0.0, 10.0, 5.0};
  params.ramp_duration = 5.0;
  params.acceleration_ramp_duration = 1.0;
  params.hold_duration = 2.0;
  params.loop = false;

  LongitudinalProfileCore core(params);

  double velocity = 0.0;
  double acceleration = 0.0;
  double jerk = 0.0;
  bool defined_accel = false;

  // First segment: peak acceleration=10/(5-1)=2.5, |jerk|=2.5.
  core.calculateReference(0.0, velocity, acceleration, jerk, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 0.0);
  EXPECT_DOUBLE_EQ(acceleration, 0.0);
  EXPECT_DOUBLE_EQ(jerk, 2.5);
  EXPECT_TRUE(defined_accel);

  // Middle of acceleration ramp-up.
  core.calculateReference(0.5, velocity, acceleration, jerk, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 0.3125);
  EXPECT_DOUBLE_EQ(acceleration, 1.25);
  EXPECT_DOUBLE_EQ(jerk, 2.5);

  // Middle of the constant-acceleration phase.
  core.calculateReference(2.5, velocity, acceleration, jerk, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 5.0);
  EXPECT_DOUBLE_EQ(acceleration, 2.5);
  EXPECT_DOUBLE_EQ(jerk, 0.0);

  // Middle of acceleration ramp-down.
  core.calculateReference(4.5, velocity, acceleration, jerk, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 9.6875);
  EXPECT_DOUBLE_EQ(acceleration, 1.25);
  EXPECT_DOUBLE_EQ(jerk, -2.5);

  // t = 6.0 (middle of first hold)
  core.calculateReference(6.0, velocity, acceleration, jerk, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 10.0);
  EXPECT_DOUBLE_EQ(acceleration, 0.0);
  EXPECT_DOUBLE_EQ(jerk, 0.0);

  // Start of the second (decelerating) S-curve.
  core.calculateReference(7.0, velocity, acceleration, jerk, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 10.0);
  EXPECT_DOUBLE_EQ(acceleration, 0.0);
  EXPECT_DOUBLE_EQ(jerk, -1.25);

  // t = 9.5 (middle of second ramp)
  core.calculateReference(9.5, velocity, acceleration, jerk, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 7.5);
  EXPECT_DOUBLE_EQ(acceleration, -1.25);
  EXPECT_DOUBLE_EQ(jerk, 0.0);

  // t = 13.0 (middle of second hold)
  core.calculateReference(13.0, velocity, acceleration, jerk, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 5.0);
  EXPECT_DOUBLE_EQ(acceleration, 0.0);
  EXPECT_DOUBLE_EQ(jerk, 0.0);

  // t = 15.0 (after profile ends, should hold last velocity)
  core.calculateReference(15.0, velocity, acceleration, jerk, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 5.0);
  EXPECT_DOUBLE_EQ(acceleration, 0.0);
  EXPECT_DOUBLE_EQ(jerk, 0.0);
}

TEST(LongitudinalProfileCore, Loop)
{
  LongitudinalProfileCoreParams params;
  params.velocity_points = {0.0, 10.0};
  params.ramp_duration = 5.0;
  params.acceleration_ramp_duration = 1.0;
  params.hold_duration = 2.0;
  params.loop = true;

  LongitudinalProfileCore core(params);

  double velocity = 0.0;
  double acceleration = 0.0;
  double jerk = 0.0;
  bool defined_accel = false;

  // Profile duration = 7.0s. At t=8.0 it should wrap to t=1.0.
  core.calculateReference(8.0, velocity, acceleration, jerk, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 1.25);
  EXPECT_DOUBLE_EQ(acceleration, 2.5);
  EXPECT_DOUBLE_EQ(jerk, 0.0);
}

TEST(LongitudinalProfileCore, InvalidPoints)
{
  LongitudinalProfileCoreParams params;
  params.velocity_points = {5.0}; // Invalid: needs at least 2 points

  LongitudinalProfileCore core(params); // Should fallback to default {0.0, 0.3, 0.1, 0.2, 0.0}

  double velocity = 0.0;
  double acceleration = 0.0;
  double jerk = 0.0;
  bool defined_accel = false;

  // The S-curve always starts with zero acceleration.
  core.calculateReference(0.0, velocity, acceleration, jerk, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 0.0);
  EXPECT_DOUBLE_EQ(acceleration, 0.0);
  EXPECT_NEAR(jerk, 0.24, 1e-12);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
