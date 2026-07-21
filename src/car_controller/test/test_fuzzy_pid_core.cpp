// Copyright 2026 realisticCar project
// Unit tests for FuzzyPidCore
#include <array>
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
  p.fuzzy_kd_min_ratio = 0.5;
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
  EXPECT_GE(fpid.adaptive_kp(), 0.7 * p.kp);
  EXPECT_LE(fpid.adaptive_kp(), 2.0 * p.kp);
  EXPECT_GE(fpid.adaptive_ki(), 0.0);
  EXPECT_LE(fpid.adaptive_ki(), 2.0 * p.ki);
  EXPECT_GE(fpid.adaptive_kd(), p.fuzzy_kd_min_ratio * p.kd);
  EXPECT_LE(fpid.adaptive_kd(), 2.0 * p.kd);
}

TEST(FuzzyPidCore, OutputUnitAcceleration)
{
  // Verify dimensionality: Kp [1/s] * error [m/s] => [m/s^2]
  FuzzyPidCore::Params p = make_params();
  p.kp = 2.0;
  p.max_output = 100.0;
  // Force zero fuzzy adjustment: zero gains so adaptive == base.
  p.fuzzy_error_gain = 0.0;
  FuzzyPidCore fpid(p);
  const double out = fpid.update(3.0, 0.02);
  // P-only with Kp=2, error=3 → 6 m/s^2
  EXPECT_NEAR(out, 6.0, 0.5);  // fuzzy may shift slightly
}

TEST(FuzzyPidCore, GoldenProductionBehavior)
{
  // These snapshots lock the pre-optimization behavior used by the current
  // production YAML. Deliberately exclude measured execution time because it
  // is nondeterministic.
  FuzzyPidCore::Params params;
  params.kp = 3.689581301;
  params.ki = 0.124693862;
  params.kd = 0.267581789;
  params.fuzzy_error_gain = 30.0;
  params.fuzzy_error_derivative_gain = 30.0;
  params.fuzzy_kp_min_ratio = 0.7;
  params.fuzzy_kd_min_ratio = 0.8;
  params.gain_rate_limit = 0.3;
  params.max_integral_error = 5.0;
  params.max_output = 4.0;
  params.derivative_filter_alpha = 0.15;

  struct Input
  {
    double error;
    double dt;
  };
  struct Snapshot
  {
    double output;
    double kp;
    double ki;
    double kd;
    double integral;
    double derivative;
    double fuzzy_error;
    double fuzzy_error_change;
    double p_term;
    double i_term;
    double d_term;
  };

  constexpr std::array<Input, 16> inputs {{
    {0.0, 0.02}, {0.003, 0.02}, {0.02, 0.02}, {0.08, 0.02},
    {0.15, 0.01}, {0.35, 0.04}, {-0.12, 0.02}, {-0.40, 0.02},
    {-0.08, 0.02}, {0.0, 0.02}, {0.70, 0.02}, {2.0, 0.02},
    {-2.0, 0.02}, {0.05, 0.04}, {-0.01, 0.01}, {0.0, 0.02},
  }};
  constexpr std::array<Snapshot, 16> expected {{
    {0.0, 3.689581301, 0.12469393944788548, 0.2408239092549852,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.01629002654546375, 3.6028801684617173, 0.1393448556641195,
      0.24324557105505556, 0.00006, 0.0225, 0.09, 0.675,
      0.010808640505385152, 0.00000836069133984717, 0.00547302534873875},
    {0.10645063025868332, 3.3568668420882171, 0.16626545138422621,
      0.26759973612468696, 0.00046, 0.146625, 0.6, 3.0,
      0.06713733684176435, 0.00007648210763674406, 0.03923681130928223},
    {0.43157745983198803, 2.9879087119882168, 0.187040793,
      0.334404818462885, 0.00206, 0.57463125, 2.4, 3.0,
      0.23903269695905735, 0.00038530403358, 0.19215945883935065},
    {0.8874975816944248, 2.6189505818882166, 0.187040793,
      0.3210981468, 0.00356, 1.5384365625, 3.0, 3.0,
      0.39284258728323246, 0.00066586522308, 0.4939891291881124},
    {1.5679462249799754, 2.5827069107, 0.187040793,
      0.3210981468, 0.01756, 2.057671078125, 3.0, 3.0,
      0.903947418745, 0.00328443632508, 0.6607143699098955},
    {-0.8753662793987731, 2.9516650408, 0.10391155166666667,
      0.2943399679, 0.01516, -1.7759795835937497, -3.0, -3.0,
      -0.354199804896, 0.0015752991232666667, -0.5227417736260397},
    {-2.389949701822201, 3.3206231709000003, 0.10391155166666667,
      0.2943399679, 0.00716, -3.6095826460546876, -3.0, -3.0,
      -1.3282492683600002, 0.0007440067099333333, -1.0624444401721338},
    {-0.4912506070390471, 3.6895813010000005, 0.10391155166666667,
      0.2943399679, 0.00556, -0.6681452491464843, -2.4, -3.0,
      -0.29516650408000006, 0.0005777482272666666, -0.1966618511863137},
    {0.008532330594985812, 3.5708346412928966, 0.14475994834694123,
      0.24090708379610962, 0.00556, 0.03207653822548828, 0.0,
      0.9622961467646485, 0.0, 0.0008048653128089932, 0.007727465282176819},
    {3.9394921058790766, 3.2018765111928964, 0.187040793,
      0.3210981468, 0.01956, 5.277265057491665, 3.0, 3.0,
      2.2413135578350274, 0.00365851791108, 1.6945200301329693},
    {4.0, 2.8329183810928962, 0.187040793, 0.3210981468,
      0.01956, 14.235675298867914, 3.0, 3.0,
      5.665836762185792, 0.00365851791108, 4.571048956913024},
    {-4.0, 3.2018765111928964, 0.10391155166666667, 0.2943399679,
      0.01956, -17.899675995962273, -3.0, -3.0,
      -6.403753022385793, 0.0020325099506, -5.268590058071936},
    {-2.1603133784448447, 3.319686351429075, 0.12469386199998786,
      0.3094083437795822, 0.02156, -7.527224596567933, 1.5, -3.0,
      0.16598431757145377, 0.002688399664719738, -2.3289860956810182},
    {-1.9865964525206552, 3.565663299036651, 0.08936452680164211,
      0.267582882701022, 0.02146, -7.298140907082743, -0.3, -3.0,
      -0.03565663299036651, 0.0019177627451632394, -1.952857582275452},
    {-1.638072012015401, 3.8116349274470718, 0.08312947367698978,
      0.2675821878708362, 0.02146, -6.128419771020331, 0.0, -3.0,
      0.0, 0.0017839585051082003, -1.6398559705205094},
  }};

  FuzzyPidCore controller(params);
  constexpr double tolerance = 1e-12;
  for (std::size_t index = 0; index < inputs.size(); ++index) {
    SCOPED_TRACE(index);
    const double output = controller.update(inputs[index].error, inputs[index].dt);
    EXPECT_NEAR(output, expected[index].output, tolerance);
    EXPECT_NEAR(controller.adaptive_kp(), expected[index].kp, tolerance);
    EXPECT_NEAR(controller.adaptive_ki(), expected[index].ki, tolerance);
    EXPECT_NEAR(controller.adaptive_kd(), expected[index].kd, tolerance);
    EXPECT_NEAR(controller.integral(), expected[index].integral, tolerance);
    EXPECT_NEAR(controller.filtered_derivative(), expected[index].derivative, tolerance);
    EXPECT_NEAR(controller.fuzzy_error_input(), expected[index].fuzzy_error, tolerance);
    EXPECT_NEAR(
      controller.fuzzy_error_change_input(), expected[index].fuzzy_error_change, tolerance);
    EXPECT_NEAR(controller.p_term(), expected[index].p_term, tolerance);
    EXPECT_NEAR(controller.i_term(), expected[index].i_term, tolerance);
    EXPECT_NEAR(controller.d_term(), expected[index].d_term, tolerance);
  }
}
