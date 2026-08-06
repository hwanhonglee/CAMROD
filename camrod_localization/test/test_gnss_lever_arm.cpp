#include <cmath>

#include <gtest/gtest.h>

#include "camrod_localization/gnss_lever_arm.hpp"

namespace
{

constexpr double kAntennaOffsetLeftM = 0.45;

// HH_260806 - Lock the measured left-antenna correction in robot and map axes.
TEST(GnssLeverArm, SubtractsLeftOffsetWhenRobotFacesMapPositiveX)
{
  const auto center = camrod_localization::antennaPositionToRobotCenter(
    10.0, 20.0, 0.0, 0.0, kAntennaOffsetLeftM);

  EXPECT_DOUBLE_EQ(center.x, 10.0);
  EXPECT_DOUBLE_EQ(center.y, 19.55);
}

TEST(GnssLeverArm, RotatesLeftOffsetWithRobotYaw)
{
  const auto center = camrod_localization::antennaPositionToRobotCenter(
    10.0, 20.0, M_PI_2, 0.0, kAntennaOffsetLeftM);

  EXPECT_NEAR(center.x, 10.45, 1.0e-12);
  EXPECT_NEAR(center.y, 20.0, 1.0e-12);
}

TEST(GnssLeverArm, SupportsCombinedForwardAndLeftOffsets)
{
  const auto center = camrod_localization::antennaPositionToRobotCenter(
    10.0, 20.0, M_PI, 0.20, kAntennaOffsetLeftM);

  EXPECT_NEAR(center.x, 10.20, 1.0e-12);
  EXPECT_NEAR(center.y, 20.45, 1.0e-12);
}

}  // namespace
