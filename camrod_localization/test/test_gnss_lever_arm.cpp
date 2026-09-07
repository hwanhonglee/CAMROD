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

TEST(GnssLeverArm, LeftAntennaOrbitKeepsCenterFixedThroughZeroTurn)
{
  // The antenna moves during a zero turn; the corrected center must not.
  // In particular, moving the TF is not a substitute for this subtraction.
  for (const double yaw : {0.0, M_PI_2, M_PI, -M_PI_2}) {
    const double antenna_x = 10.0 - std::sin(yaw) * kAntennaOffsetLeftM;
    const double antenna_y = 20.0 + std::cos(yaw) * kAntennaOffsetLeftM;
    const auto center = camrod_localization::antennaPositionToRobotCenter(
      antenna_x, antenna_y, yaw, 0.0, kAntennaOffsetLeftM);
    EXPECT_NEAR(center.x, 10.0, 1.0e-12);
    EXPECT_NEAR(center.y, 20.0, 1.0e-12);
  }
}

TEST(GnssLeverArm, CorrectMountOffsetDoesNotCancelMismatchedHeadingTime)
{
  // Synthetic 10-degree heading lag: a correct 45 cm mount offset can still
  // leave more than 5 cm of center error. This is not a measured field lag.
  const double yaw = M_PI_2;
  const double lag = 10.0 * M_PI / 180.0;
  const auto center = camrod_localization::antennaPositionToRobotCenter(
    10.0 - std::sin(yaw) * kAntennaOffsetLeftM,
    20.0 + std::cos(yaw) * kAntennaOffsetLeftM,
    yaw - lag, 0.0, kAntennaOffsetLeftM);
  const double error = std::hypot(center.x - 10.0, center.y - 20.0);
  EXPECT_NEAR(error, 2.0 * kAntennaOffsetLeftM * std::sin(lag / 2.0), 1.0e-12);
  EXPECT_GT(error, 0.05);
}

}  // namespace
