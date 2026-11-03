#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <string>

#include "camrod_sensing/radar_dummy_contract.hpp"

// HH_260729 - Dummy radar markers remain per-channel and suppress only fresh fake ranges.
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

// HH_261102 - No-return replies prove a live sensor, whereas missing or replayed data never prove clearance.
TEST(RadarDummyContract, DistinguishesFreshNoTargetFromDropoutAndMalformedRanges)
{
  EXPECT_TRUE(camrod_sensing::radar_range_heartbeat_valid(0.300f, 0.020f, 0.500f));
  EXPECT_TRUE(camrod_sensing::radar_range_heartbeat_valid(0.501f, 0.020f, 0.500f));
  for (const double bad_range : {0.019, 0.502, -1.0,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    EXPECT_FALSE(camrod_sensing::radar_range_heartbeat_valid(bad_range, 0.020, 0.500));
  }
  EXPECT_FALSE(camrod_sensing::radar_range_heartbeat_valid(0.501, 0.500, 0.020));
  EXPECT_TRUE(camrod_sensing::radar_observation_is_fresh(true, 0.10, 0.20, 0.35));
  EXPECT_TRUE(camrod_sensing::radar_observation_is_fresh(true, 0.35, 0.35, 0.35));
  EXPECT_FALSE(camrod_sensing::radar_observation_is_fresh(false, 0.0, 0.0, 0.35));
  EXPECT_FALSE(camrod_sensing::radar_observation_is_fresh(true, 0.36, 0.10, 0.35));
  EXPECT_FALSE(camrod_sensing::radar_observation_is_fresh(true, 0.10, 0.36, 0.35));
  EXPECT_FALSE(camrod_sensing::radar_observation_is_fresh(true, -0.01, 0.10, 0.35));
  EXPECT_FALSE(camrod_sensing::radar_observation_is_fresh(true, 0.10, -0.01, 0.35));
  EXPECT_FALSE(camrod_sensing::radar_observation_is_fresh(true, 0.10, 0.10, 0.0));
}
