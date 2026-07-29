#include <limits>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "camrod_localization/navsat_fix_validation.hpp"

namespace
{

sensor_msgs::msg::NavSatFix validFix()
{
  sensor_msgs::msg::NavSatFix message;
  message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  message.latitude = 37.123;
  message.longitude = 127.456;
  message.altitude = 52.0;
  return message;
}

TEST(NavSatFixValidation, AcceptsFiniteFixInsideGeodeticBounds)
{
  EXPECT_TRUE(camrod_localization::navSatFixIsUsable(validFix()));
}

TEST(NavSatFixValidation, RejectsDisabledNoFixHeartbeat)
{
  auto message = validFix();
  message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  message.latitude = std::numeric_limits<double>::quiet_NaN();
  message.longitude = std::numeric_limits<double>::quiet_NaN();
  message.altitude = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(camrod_localization::navSatFixIsUsable(message));
}

TEST(NavSatFixValidation, RejectsNonFiniteOrOutOfBoundsCoordinates)
{
  auto message = validFix();
  message.altitude = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(camrod_localization::navSatFixIsUsable(message));

  message = validFix();
  message.latitude = 90.001;
  EXPECT_FALSE(camrod_localization::navSatFixIsUsable(message));

  message = validFix();
  message.longitude = -180.001;
  EXPECT_FALSE(camrod_localization::navSatFixIsUsable(message));
}

}  // namespace
