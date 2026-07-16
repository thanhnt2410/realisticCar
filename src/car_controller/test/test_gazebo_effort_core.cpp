#include <limits>
#include "car_controller/gazebo_effort_core.hpp"
#include "gtest/gtest.h"

TEST(GazeboEffortCore, ValidatesAndConverts)
{
  car_controller::GazeboEffortCore core(1000.0, 0.3, 2.0, 200.0);
  EXPECT_FALSE(core.validCommand(0.0, std::numeric_limits<double>::quiet_NaN(), 0.0));
  const auto nominal = core.convert(1.0);
  EXPECT_DOUBLE_EQ(nominal.torque_each, 150.0);
  const auto clamped = core.convert(10.0);
  EXPECT_DOUBLE_EQ(clamped.acceleration, 2.0);
  EXPECT_DOUBLE_EQ(clamped.torque_each, 200.0);
  EXPECT_DOUBLE_EQ(core.convert(-1.0).torque_each, -150.0);
}
