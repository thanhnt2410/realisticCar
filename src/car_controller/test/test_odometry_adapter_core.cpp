#include "car_controller/odometry_adapter_core.hpp"
#include "gtest/gtest.h"

TEST(OdometryAdapterCore, ProjectsFrames)
{
  EXPECT_DOUBLE_EQ(
    car_controller::longitudinalVelocity(
      2.0, 4.0, 1.0,
      car_controller::TwistFrame::BODY), 2.0);
  EXPECT_NEAR(
    car_controller::longitudinalVelocity(
      0.0, 2.0, M_PI_2,
      car_controller::TwistFrame::WORLD), 2.0, 1e-12);
}

TEST(OdometryAdapterCore, NoiseIsSeeded)
{
  car_controller::SeededGaussianNoise zero(0.0, 1);
  EXPECT_DOUBLE_EQ(zero.add(3.0), 3.0);
  car_controller::SeededGaussianNoise first(0.1, 42), second(0.1, 42), other(0.1, 43);
  EXPECT_DOUBLE_EQ(first.add(1.0), second.add(1.0));
  EXPECT_NE(first.add(1.0), other.add(1.0));
}

TEST(OdometryAdapterCore, SupportsNonstandardBodyAxis)
{
  EXPECT_DOUBLE_EQ(
    car_controller::bodyLongitudinalVelocity(0.0, -3.0, true, -1.0), 3.0);
}
