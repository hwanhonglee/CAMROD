#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <string>

#include "camrod_sensing/radar_dummy_contract.hpp"

TEST(RadarDummyContract, DerivesAbsoluteAndRelativePerChannelMarkers)
{
  EXPECT_EQ(
    camrod_sensing::radar_dummy_active_topic(
      "/sensing/radar/front1/range"),
    "/sensing/radar/front1/dummy_active");
  EXPECT_EQ(
    camrod_sensing::radar_dummy_active_topic("left2/range"),
    "left2/dummy_active");
}

TEST(RadarDummyContract, RejectsTopicsOutsideTheCanonicalRangeBoundary)
{
  EXPECT_THROW(
    camrod_sensing::radar_dummy_active_topic(
      "/sensing/radar/front1/range_ros"),
    std::invalid_argument);
  EXPECT_THROW(
    camrod_sensing::radar_dummy_active_topic("range"),
    std::invalid_argument);
  EXPECT_THROW(
    camrod_sensing::radar_dummy_active_topic(""),
    std::invalid_argument);
}

TEST(RadarDummyContract, NoTargetValueIsStrictlyAboveConfiguredMaximum)
{
  const double max_range_m = 0.80;
  const double no_target_m =
    camrod_sensing::radar_no_target_range_m(max_range_m);

  EXPECT_GT(no_target_m, max_range_m);
  EXPECT_NEAR(no_target_m - max_range_m, 0.001, 1e-12);
}

TEST(RadarDummyContract, RejectsInvalidMaximumRanges)
{
  EXPECT_THROW(
    camrod_sensing::radar_no_target_range_m(0.0),
    std::invalid_argument);
  EXPECT_THROW(
    camrod_sensing::radar_no_target_range_m(
      std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
}

TEST(RadarDummyContract, SuppressesCostOnlyForFreshPositiveDummyState)
{
  EXPECT_TRUE(
    camrod_sensing::radar_dummy_state_suppresses_cost(
      true, true, 0.10, 1.0));
  EXPECT_FALSE(
    camrod_sensing::radar_dummy_state_suppresses_cost(
      false, true, 0.10, 1.0));
  EXPECT_FALSE(
    camrod_sensing::radar_dummy_state_suppresses_cost(
      true, false, 0.10, 1.0));
  EXPECT_FALSE(
    camrod_sensing::radar_dummy_state_suppresses_cost(
      true, true, 1.01, 1.0));
  EXPECT_FALSE(
    camrod_sensing::radar_dummy_state_suppresses_cost(
      true, true, -0.01, 1.0));
  EXPECT_FALSE(
    camrod_sensing::radar_dummy_state_suppresses_cost(
      true, true, 0.10, 0.0));
}
