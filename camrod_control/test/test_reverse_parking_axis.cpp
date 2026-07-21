// HH_260721 - Verify reverse parking axis conventions in native C++.

#include <cmath>

#include "camrod_control/reverse_parking_axis.hpp"
#include "gtest/gtest.h"

namespace camrod_control
{

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
