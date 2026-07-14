// Copyright 2026 realisticCar project
// Unit tests for PidCore
#include <gtest/gtest.h>
#include <cmath>
#include "car_controller/pid_core.hpp"

using car_controller::PidCore;

namespace
{
PidCore::Params make_params(double kp = 1.0, double ki = 0.0, double kd = 0.0)
{
  PidCore::Params p;
  p.kp = kp;
  p.ki = ki;
  p.kd = kd;
  p.max_integral_error = 10.0;
  p.max_output = 5.0;
  p.derivative_filter_alpha = 1.0;  // raw derivative for test simplicity
  return p;
}
}  // namespace

TEST(PidCore, ZeroError)
{
  PidCore pid(make_params(2.0, 0.5, 0.1));
  const double out = pid.update(0.0, 0.02);
  EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(PidCore, CorrectSignPositiveError)
{
  PidCore pid(make_params(1.0));
  const double out = pid.update(1.0, 0.02);
  EXPECT_GT(out, 0.0) << "Positive error should produce positive acceleration correction";
}

TEST(PidCore, CorrectSignNegativeError)
{
  PidCore pid(make_params(1.0));
  const double out = pid.update(-1.0, 0.02);
  EXPECT_LT(out, 0.0);
}

TEST(PidCore, CorrectionBound)
{
  PidCore::Params p = make_params(100.0);  // very large Kp
  p.max_output = 3.0;
  PidCore pid(p);
  const double out = pid.update(10.0, 0.02);
  EXPECT_LE(out, 3.0);
  EXPECT_GE(out, -3.0);
}

TEST(PidCore, AntiWindup)
{
  PidCore::Params p = make_params(0.0, 1.0);
  p.max_output = 1.0;
  p.max_integral_error = 100.0;
  PidCore pid(p);
  // Drive integrator into saturation with large constant error.
  double out_prev = 0.0;
  for (int i = 0; i < 500; ++i) {
    out_prev = pid.update(5.0, 0.02);
  }
  // Output should stay at max_output or just below due to anti-windup.
  // The conditional-integration scheme prevents windup past saturation.
  EXPECT_LE(out_prev, 1.0 + 1e-6);
  EXPECT_GT(out_prev, 0.0);
}

TEST(PidCore, Reset)
{
  PidCore pid(make_params(1.0, 1.0, 1.0));
  pid.update(2.0, 0.02);
  pid.update(2.0, 0.02);
  pid.reset();
  const double out = pid.update(0.0, 0.02);
  EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(PidCore, InvalidDtNaN)
{
  PidCore pid(make_params(1.0));
  const double out = pid.update(1.0, 0.0);  // dt=0 → skip
  EXPECT_FALSE(std::isnan(out));
  EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(PidCore, OutputUnitAcceleration)
{
  // With Kp [1/s], error [m/s] → output [m/s^2]
  PidCore::Params p = make_params(2.0);
  p.max_output = 100.0;
  PidCore pid(p);
  const double error_mps = 3.0;
  const double out = pid.update(error_mps, 0.02);
  // P-only: out = Kp [1/s] * error [m/s] = 2 * 3 = 6 m/s^2
  EXPECT_NEAR(out, 6.0, 1e-9);
}

TEST(PidCore, IntegralAccumulation)
{
  PidCore::Params p = make_params(0.0, 1.0);  // Ki=1 only
  p.max_output = 100.0;
  PidCore pid(p);
  // After 1 step of dt=1s with error=1: integral=1, output=1
  const double out = pid.update(1.0, 1.0);
  EXPECT_NEAR(out, 1.0, 1e-9);
}
