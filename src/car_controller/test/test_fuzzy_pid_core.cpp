// Copyright 2026 realisticCar project
// Unit tests for FuzzyPidCore
#include <gtest/gtest.h>
#include <cmath>
#include "car_controller/fuzzy_pid_core.hpp"

using car_controller::FuzzyPidCore;

namespace
{
FuzzyPidCore::Params make_params()
{
  FuzzyPidCore::Params p;
  p.kp = 1.0;
  p.ki = 0.0;
  p.kd = 0.0;
  p.fuzzy_error_gain = 0.8;
  p.fuzzy_error_derivative_gain = 0.5;
  p.kp_min = 0.7;
  p.kp_max = 2.0;
  p.ki_min = 0.0;
  p.ki_max = 0.2;
  p.kd_min = 0.0;
  p.kd_max = 0.1;
  p.max_integral_error = 10.0;
  p.max_output = 5.0;
  p.derivative_filter_alpha = 1.0;
  return p;
}
}  // namespace

TEST(FuzzyPidCore, ZeroError)
{
  FuzzyPidCore fpid(make_params());
  const double out = fpid.update(0.0, 0.02);
  EXPECT_NEAR(out, 0.0, 1e-6);
}

TEST(FuzzyPidCore, CorrectSignPositiveError)
{
  FuzzyPidCore fpid(make_params());
  const double out = fpid.update(1.0, 0.02);
  EXPECT_GT(out, 0.0);
}

TEST(FuzzyPidCore, CorrectSignNegativeError)
{
  FuzzyPidCore fpid(make_params());
  const double out = fpid.update(-1.0, 0.02);
  EXPECT_LT(out, 0.0);
}

TEST(FuzzyPidCore, CorrectionBound)
{
  FuzzyPidCore::Params p = make_params();
  p.kp = 100.0;
  p.max_output = 3.0;
  FuzzyPidCore fpid(p);
  const double out = fpid.update(10.0, 0.02);
  EXPECT_LE(out, 3.0);
  EXPECT_GE(out, -3.0);
}

TEST(FuzzyPidCore, AntiWindup)
{
  FuzzyPidCore::Params p = make_params();
  p.kp = 0.0;
  p.ki = 1.0;
  p.max_output = 1.0;
  p.max_integral_error = 100.0;
  FuzzyPidCore fpid(p);
  double out_prev = 0.0;
  for (int i = 0; i < 500; ++i) {
    out_prev = fpid.update(5.0, 0.02);
  }
  EXPECT_LE(out_prev, 1.0 + 1e-6);
  EXPECT_GT(out_prev, 0.0);
}

TEST(FuzzyPidCore, Reset)
{
  FuzzyPidCore fpid(make_params());
  fpid.update(2.0, 0.02);
  fpid.update(2.0, 0.02);
  fpid.reset();
  const double out = fpid.update(0.0, 0.02);
  EXPECT_NEAR(out, 0.0, 1e-6);
}

TEST(FuzzyPidCore, InvalidDtNaN)
{
  FuzzyPidCore fpid(make_params());
  const double out = fpid.update(1.0, 0.0);
  EXPECT_FALSE(std::isnan(out));
  EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(FuzzyPidCore, AdaptiveGainBounds)
{
  FuzzyPidCore::Params p = make_params();
  p.kp = 1.0;
  p.ki = 0.1;
  p.kd = 0.05;
  FuzzyPidCore fpid(p);
  for (int i = 0; i < 50; ++i) {
    fpid.update(3.0, 0.02);
  }
  EXPECT_GE(fpid.adaptive_kp(), p.kp_min);
  EXPECT_LE(fpid.adaptive_kp(), p.kp_max);
  EXPECT_GE(fpid.adaptive_ki(), p.ki_min);
  EXPECT_LE(fpid.adaptive_ki(), p.ki_max);
  EXPECT_GE(fpid.adaptive_kd(), p.kd_min);
  EXPECT_LE(fpid.adaptive_kd(), p.kd_max);
}

TEST(FuzzyPidCore, OutputUnitAcceleration)
{
  // Verify dimensionality: Kp [1/s] * error [m/s] => [m/s^2]
  FuzzyPidCore::Params p = make_params();
  p.kp = 2.0;
  p.max_output = 100.0;
  // Fix all fuzzy gain ranges so the selected gains are deterministic.
  p.kp_min = p.kp_max = 2.0;
  p.ki_min = p.ki_max = 0.0;
  p.kd_min = p.kd_max = 0.0;
  FuzzyPidCore fpid(p);
  const double out = fpid.update(3.0, 0.02);
  // P-only with Kp=2, error=3 → 6 m/s^2
  EXPECT_NEAR(out, 6.0, 1e-9);
}

TEST(FuzzyPidCore, PaperTableCenterRuleSetsDirectGains)
{
  FuzzyPidCore::Params p = make_params();
  p.kp_min = 1.0; p.kp_max = 7.0;
  p.ki_min = 2.0; p.ki_max = 8.0;
  p.kd_min = 3.0; p.kd_max = 9.0;
  FuzzyPidCore fpid(p);
  fpid.update(0.0, 0.02);  // e=Z, de=Z => S / VS / B
  EXPECT_NEAR(fpid.adaptive_kp(), 2.0, 1e-9);
  EXPECT_NEAR(fpid.adaptive_ki(), 2.0, 1e-9);
  EXPECT_NEAR(fpid.adaptive_kd(), 8.0, 1e-9);
}

TEST(FuzzyPidCore, GainsDoNotAccumulateBetweenCycles)
{
  FuzzyPidCore::Params p = make_params();
  FuzzyPidCore fpid(p);
  fpid.update(0.0, 0.02);
  const double kp = fpid.adaptive_kp();
  const double ki = fpid.adaptive_ki();
  const double kd = fpid.adaptive_kd();
  for (int i = 0; i < 20; ++i) {
    fpid.update(0.0, 0.02);
  }
  EXPECT_DOUBLE_EQ(fpid.adaptive_kp(), kp);
  EXPECT_DOUBLE_EQ(fpid.adaptive_ki(), ki);
  EXPECT_DOUBLE_EQ(fpid.adaptive_kd(), kd);
}
