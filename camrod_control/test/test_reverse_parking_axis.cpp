// HH_260721 - Verify reverse parking axis conventions in native C++.

#include <cmath>

#include "camrod_control/reverse_parking_axis.hpp"
#include "camrod_control/parking_speed_profile.hpp"
#include "gtest/gtest.h"

namespace camrod_control
{

TEST(ParkingSpeedProfile, SlowsInsideConfiguredRemainingDistance)
{
  // HH_260818 - Docking starts slowing at 0.60 m and reverse parking at 0.30 m.
  EXPECT_DOUBLE_EQ(parkingApproachSpeed(0.61, 0.60, 0.50, 0.10), 0.50);
  EXPECT_NEAR(parkingApproachSpeed(0.30, 0.60, 0.50, 0.10), 0.30, 1.0e-9);
  EXPECT_DOUBLE_EQ(parkingApproachSpeed(0.0, 0.60, 0.50, 0.10), 0.10);
  EXPECT_DOUBLE_EQ(parkingApproachSpeed(0.31, 0.30, 0.40, 0.08), 0.40);
  EXPECT_NEAR(parkingApproachSpeed(0.15, 0.30, 0.40, 0.08), 0.24, 1.0e-9);
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
