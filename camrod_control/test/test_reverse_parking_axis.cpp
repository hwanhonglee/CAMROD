// HH_260721 - Verify reverse parking axis conventions in native C++.

#include <cmath>

#include "camrod_control/reverse_parking_axis.hpp"
#include "camrod_control/parking_speed_profile.hpp"
#include "gtest/gtest.h"

namespace camrod_control
{

TEST(ParkingSpeedProfile, SlowsInsideConfiguredRemainingDistance)
{
  // HH_260818 - The generic profile is shared by reverse and tag approaches.
  EXPECT_DOUBLE_EQ(parkingApproachSpeed(0.61, 0.60, 0.50, 0.10), 0.50);
  EXPECT_NEAR(parkingApproachSpeed(0.30, 0.60, 0.50, 0.10), 0.30, 1.0e-9);
  EXPECT_DOUBLE_EQ(parkingApproachSpeed(0.0, 0.60, 0.50, 0.10), 0.10);
  EXPECT_DOUBLE_EQ(parkingApproachSpeed(0.31, 0.30, 0.40, 0.08), 0.40);
  EXPECT_NEAR(parkingApproachSpeed(0.15, 0.30, 0.40, 0.08), 0.24, 1.0e-9);
}

TEST(ParkingSpeedProfile, AprilTagCameraRangeRampsFromPointEightToPointFour)
{
  // HH_260824 - The controller uses the same camera-frame 3D norm as the UI.
  const double stop_m = 0.40;
  const double window_m = tagApproachSlowdownWindow(0.80, stop_m);
  EXPECT_NEAR(window_m, 0.40, 1.0e-12);
  EXPECT_NEAR(tagApproachRemainingDistance(0.80, stop_m), 0.40, 1.0e-12);
  EXPECT_NEAR(tagApproachRemainingDistance(0.60, stop_m), 0.20, 1.0e-12);
  EXPECT_DOUBLE_EQ(tagApproachRemainingDistance(0.40, stop_m), 0.0);
  EXPECT_DOUBLE_EQ(tagApproachRemainingDistance(0.35, stop_m), 0.0);

  EXPECT_DOUBLE_EQ(parkingApproachSpeed(0.40, window_m, 0.50, 0.10), 0.50);
  EXPECT_NEAR(parkingApproachSpeed(0.20, window_m, 0.50, 0.10), 0.30, 1.0e-12);
  EXPECT_DOUBLE_EQ(parkingApproachSpeed(0.0, window_m, 0.50, 0.10), 0.10);
}

TEST(ParkingSpeedProfile, AprilTagApproachKeepsMinimumAckermannTurnRadius)
{
  // HH_260819 - 0.85 m stays above Ranger's 0.81033 m spinning threshold.
  EXPECT_NEAR(
    limitApproachAngularSpeedForTurnRadius(1.0, 0.555556, 0.85),
    0.555556 / 0.85, 1.0e-12);
  EXPECT_NEAR(
    limitApproachAngularSpeedForTurnRadius(-1.0, 0.138889, 0.85),
    -0.138889 / 0.85, 1.0e-12);
  EXPECT_DOUBLE_EQ(
    limitApproachAngularSpeedForTurnRadius(0.10, 0.555556, 0.85), 0.10);
  EXPECT_DOUBLE_EQ(
    limitApproachAngularSpeedForTurnRadius(0.25, 0.10, 0.0), 0.25);
}

TEST(ReverseParkingAxis, DropZoneYawProducesReverseMotionTowardStation)
{
  const double station_yaw = 0.0;
  const double body_yaw = bodyYawForReverseAxis(station_yaw);
  EXPECT_NEAR(body_yaw, 3.14159265358979323846, 1.0e-9);
  EXPECT_NEAR(reverseAxisYawForBody(body_yaw), 0.0, 1.0e-9);
  EXPECT_GT(signedDistanceAlongAxis(0.0, 0.0, 2.0, 0.0, station_yaw), 0.0);
}

TEST(ReverseParkingAxis, StationYawUsedAsBodyFrontPointsReverseAway)
{
  const double station_yaw = 0.0;
  const double incorrect_reverse_axis = reverseAxisYawForBody(station_yaw);
  EXPECT_LT(signedDistanceAlongAxis(0.0, 0.0, 2.0, 0.0, incorrect_reverse_axis), 0.0);
}

}  // namespace camrod_control
