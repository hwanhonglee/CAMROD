// HH_261102 - Exercise real float32 BMS boundaries instead of only double literals.
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "camrod_control/charging_mission_override.hpp"

using camrod_control::ChargingMissionOverride;
using camrod_control::ChargingMissionOverrideConfig;

TEST(ChargingMissionWireSoc, AcceptsThirtyFivePercentButNotTheAdjacentLowerWireValue)
{
  ChargingMissionOverride policy;
  policy.setBatteryPercentage(static_cast<double>(0.35F));
  EXPECT_TRUE(policy.batteryReadyForDeparture());
  policy.setBatteryPercentage(static_cast<double>(std::nextafter(0.35F, 0.0F)));
  EXPECT_FALSE(policy.batteryReadyForDeparture());
  policy.setBatteryPercentage(0.34);
  EXPECT_FALSE(policy.batteryReadyForDeparture());
  policy.setBatteryPercentage(std::nullopt);
  EXPECT_FALSE(policy.batteryReadyForDeparture());
  policy.setBatteryPercentage(std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(policy.batteryReadyForDeparture());
}

TEST(ChargingMissionWireSoc, PreservesConfiguredMarginAtTheSameWirePrecision)
{
  ChargingMissionOverrideConfig config;
  config.minimum_departure_battery_percentage = 0.373;
  ChargingMissionOverride policy;
  policy.setConfig(config);
  policy.setBatteryPercentage(static_cast<double>(0.373F));
  EXPECT_TRUE(policy.batteryReadyForDeparture());
  policy.setBatteryPercentage(static_cast<double>(std::nextafter(0.373F, 0.0F)));
  EXPECT_FALSE(policy.batteryReadyForDeparture());
}

TEST(ChargingMissionWireSoc, CorrectSocDoesNotCreateAChargingOrMissionOverride)
{
  ChargingMissionOverride policy;
  policy.setBatteryPercentage(static_cast<double>(0.35F));
  EXPECT_FALSE(policy.isActive(100.0));
  EXPECT_FALSE(policy.activateForMission({"camping_site_1", "ui", 1, 0}, 100.0));
  policy.setCharging(true);
  EXPECT_FALSE(policy.activateForMission({"drop_zone", "ui", 2, 0}, 100.0));
  EXPECT_TRUE(policy.activateForMission({"camping_site_1", "ui", 3, 0}, 100.0));
  EXPECT_TRUE(policy.isActive(100.0));
}
