// Copyright 2026 realisticCar project
#include "gtest/gtest.h"
#include "car_planning/longitudinal_profile_core.hpp"

using car_planning::LongitudinalProfileCore;
using car_planning::LongitudinalProfileCoreParams;

TEST(LongitudinalProfileCore, RampAndHold)
{
  LongitudinalProfileCoreParams params;
  params.velocity_points = {0.0, 10.0, 5.0};
  params.ramp_duration = 5.0;
  params.hold_duration = 2.0;
  params.loop = false;

  LongitudinalProfileCore core(params);

  double velocity = 0.0;
  double acceleration = 0.0;
  bool defined_accel = false;

  // t = 0.0 (start of first ramp)
  core.calculateReference(0.0, velocity, acceleration, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 0.0);
  EXPECT_DOUBLE_EQ(acceleration, 2.0); // (10 - 0) / 5.0
  EXPECT_TRUE(defined_accel);

  // t = 2.5 (middle of first ramp)
  core.calculateReference(2.5, velocity, acceleration, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 5.0);
  EXPECT_DOUBLE_EQ(acceleration, 2.0);

  // t = 6.0 (middle of first hold)
  core.calculateReference(6.0, velocity, acceleration, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 10.0);
  EXPECT_DOUBLE_EQ(acceleration, 0.0);

  // t = 7.0 (start of second ramp)
  core.calculateReference(7.0, velocity, acceleration, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 10.0);
  EXPECT_DOUBLE_EQ(acceleration, -1.0); // (5 - 10) / 5.0

  // t = 9.5 (middle of second ramp)
  core.calculateReference(9.5, velocity, acceleration, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 7.5);
  EXPECT_DOUBLE_EQ(acceleration, -1.0);

  // t = 13.0 (middle of second hold)
  core.calculateReference(13.0, velocity, acceleration, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 5.0);
  EXPECT_DOUBLE_EQ(acceleration, 0.0);

  // t = 15.0 (after profile ends, should hold last velocity)
  core.calculateReference(15.0, velocity, acceleration, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 5.0);
  EXPECT_DOUBLE_EQ(acceleration, 0.0);
}

TEST(LongitudinalProfileCore, Loop)
{
  LongitudinalProfileCoreParams params;
  params.velocity_points = {0.0, 10.0};
  params.ramp_duration = 5.0;
  params.hold_duration = 2.0;
  params.loop = true;

  LongitudinalProfileCore core(params);

  double velocity = 0.0;
  double acceleration = 0.0;
  bool defined_accel = false;

  // Profile duration = 7.0s. At t=8.0 it should wrap to t=1.0.
  core.calculateReference(8.0, velocity, acceleration, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 2.0); // start=0, a=2, t=1 -> v=2
  EXPECT_DOUBLE_EQ(acceleration, 2.0);
}

TEST(LongitudinalProfileCore, InvalidPoints)
{
  LongitudinalProfileCoreParams params;
  params.velocity_points = {5.0}; // Invalid: needs at least 2 points

  LongitudinalProfileCore core(params); // Should fallback to default {0.0, 0.3, 0.1, 0.2, 0.0}

  double velocity = 0.0;
  double acceleration = 0.0;
  bool defined_accel = false;

  // With default points and default ramp (3.0s), first ramp is 0.0 to 0.3
  core.calculateReference(0.0, velocity, acceleration, defined_accel);
  EXPECT_DOUBLE_EQ(velocity, 0.0);
  EXPECT_DOUBLE_EQ(acceleration, 0.1); // (0.3 - 0) / 3.0
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
