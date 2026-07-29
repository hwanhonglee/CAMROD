#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "camrod_sensing/radar_self_echo_filter.hpp"

namespace
{

using camrod::sensing::radar_self_echo_filter::Band;
using camrod::sensing::radar_self_echo_filter::StartupCalibrationAction;

TEST(RadarSelfEchoFilter, BuildsMultipleNotchesForOneSensor)
{
  std::vector<Band> bands;
  std::string error;
  ASSERT_TRUE(camrod::sensing::radar_self_echo_filter::buildBands(
    std::vector<std::int64_t>{0, 0},
    std::vector<double>{0.111, 0.162},
    std::vector<double>{0.012, 0.010},
    7U, bands, error));
  EXPECT_TRUE(error.empty());
  ASSERT_EQ(bands.size(), 2U);

  EXPECT_TRUE(camrod::sensing::radar_self_echo_filter::matches(0U, 0.110, bands));
  EXPECT_TRUE(camrod::sensing::radar_self_echo_filter::matches(0U, 0.165, bands));
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::matches(1U, 0.165, bands));
}

// HH_260729 - Named min/max specs expose the physical sensor and ignored
// interval directly while preserving multiple narrow bands for one radar.
TEST(RadarSelfEchoFilter, BuildsReadableNamedMinMaxSpecs)
{
  const std::vector<std::string> input_topics{
    "/sensing/radar/front1/range",
    "/sensing/radar/front2/range",
    "/sensing/radar/left1/range"};
  std::vector<Band> bands;
  std::string error;
  ASSERT_TRUE(camrod::sensing::radar_self_echo_filter::buildBandsFromSpecs(
    std::vector<std::string>{
      "FRONT1:0.152:0.202",
      "front1:0.231:0.261",
      "LEFT1:0.095:0.119"},
    input_topics, bands, error));
  EXPECT_TRUE(error.empty());
  ASSERT_EQ(bands.size(), 3U);

  EXPECT_EQ(bands[0].sensor_index, 0U);
  EXPECT_NEAR(bands[0].center_m, 0.177, 1e-9);
  EXPECT_NEAR(bands[0].half_width_m, 0.025, 1e-9);
  EXPECT_EQ(bands[1].sensor_index, 0U);
  EXPECT_EQ(bands[2].sensor_index, 2U);
  EXPECT_TRUE(camrod::sensing::radar_self_echo_filter::matches(0U, 0.152, bands));
  EXPECT_TRUE(camrod::sensing::radar_self_echo_filter::matches(2U, 0.119, bands));
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::matches(1U, 0.177, bands));
}

// HH_260729 - A typo must leave zero notches rather than accepting the entries
// before it and silently hiding real obstacles on only part of the robot.
TEST(RadarSelfEchoFilter, MalformedOrUnknownNamedSpecFailsSafe)
{
  const std::vector<std::string> input_topics{
    "/sensing/radar/front1/range",
    "/sensing/radar/rear/range"};
  std::vector<Band> bands{{0U, 0.1, 0.01}};
  std::string error;

  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::buildBandsFromSpecs(
    std::vector<std::string>{"FRONT1:0.10", "REAR:0.12:0.14"},
    input_topics, bands, error));
  EXPECT_TRUE(bands.empty());
  EXPECT_FALSE(error.empty());

  bands.push_back(Band{0U, 0.1, 0.01});
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::buildBandsFromSpecs(
    std::vector<std::string>{"FRONT3:0.10:0.12"},
    input_topics, bands, error));
  EXPECT_TRUE(bands.empty());
  EXPECT_NE(error.find("unknown sensor FRONT3"), std::string::npos);

  bands.push_back(Band{0U, 0.1, 0.01});
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::buildBandsFromSpecs(
    std::vector<std::string>{"FRONT1:0.20:0.10"},
    input_topics, bands, error));
  EXPECT_TRUE(bands.empty());
  EXPECT_FALSE(error.empty());

  bands.push_back(Band{0U, 0.1, 0.01});
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::buildBandsFromSpecs(
    std::vector<std::string>{"FRONT1:0.10:0.25"},
    input_topics, bands, error));
  EXPECT_TRUE(bands.empty());
  EXPECT_NE(error.find("wider than"), std::string::npos);
}

// HH_260728 - A notch rejects only the measured body return. In particular,
// LEFT2 must regain every other obstacle distance hidden by the old one-sided
// 0.75 m threshold. The observed 0.68-0.80 m external returns remain costs.
TEST(RadarSelfEchoFilter, KeepsCloserAndFartherObstacles)
{
  const std::vector<Band> bands{{3U, 0.247, 0.020}};
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::matches(3U, 0.500, bands));
  EXPECT_TRUE(camrod::sensing::radar_self_echo_filter::matches(3U, 0.247, bands));
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::matches(3U, 0.695, bands));
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::matches(3U, 0.740, bands));
}

TEST(RadarSelfEchoFilter, IncludesBandEdgesOnly)
{
  const std::vector<Band> bands{{0U, 0.110, 0.010}};
  EXPECT_TRUE(camrod::sensing::radar_self_echo_filter::matches(0U, 0.100, bands));
  EXPECT_TRUE(camrod::sensing::radar_self_echo_filter::matches(0U, 0.120, bands));
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::matches(0U, 0.099, bands));
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::matches(0U, 0.121, bands));
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::matches(
    0U, std::numeric_limits<double>::quiet_NaN(), bands));
}

TEST(RadarSelfEchoFilter, InvalidConfigurationLeavesFilterEmpty)
{
  std::vector<Band> bands{{0U, 0.1, 0.01}};
  std::string error;
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::buildBands(
    std::vector<std::int64_t>{0, 3},
    std::vector<double>{0.110},
    std::vector<double>{0.010, 0.020},
    7U, bands, error));
  EXPECT_TRUE(bands.empty());
  EXPECT_FALSE(error.empty());

  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::buildBands(
    std::vector<std::int64_t>{7},
    std::vector<double>{0.110},
    std::vector<double>{0.010},
    7U, bands, error));
  EXPECT_TRUE(bands.empty());
}

TEST(RadarSelfEchoFilter, LearnsDominantTightStartupCluster)
{
  const std::vector<double> samples{
    0.203, 0.206, 0.207, 0.207, 0.208, 0.209, 0.210, 0.207,
    0.250, 0.410};
  Band learned;
  std::string error;
  ASSERT_TRUE(camrod::sensing::radar_self_echo_filter::learnDominantBand(
    5U, samples, 6U, 0.020, 0.60, 0.012, 0.030, 0.005, learned, error));
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(learned.sensor_index, 5U);
  EXPECT_NEAR(learned.center_m, 0.207, 1e-6);
  EXPECT_NEAR(learned.half_width_m, 0.012, 1e-6);
  EXPECT_TRUE(camrod::sensing::radar_self_echo_filter::matches(5U, 0.207, {learned}));
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::matches(5U, 0.500, {learned}));
}

TEST(RadarSelfEchoFilter, RejectsWeakOrBroadStartupClusters)
{
  Band learned;
  std::string error;
  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::learnDominantBand(
    3U, std::vector<double>{0.20, 0.20, 0.70, 0.70},
    2U, 0.020, 0.75, 0.012, 0.030, 0.005, learned, error));
  EXPECT_FALSE(error.empty());

  EXPECT_FALSE(camrod::sensing::radar_self_echo_filter::learnDominantBand(
    3U, std::vector<double>{0.60, 0.62, 0.64, 0.66, 0.68, 0.70},
    4U, 0.025, 0.60, 0.012, 0.030, 0.005, learned, error));
  EXPECT_EQ(error, "dominant startup cluster is too broad");
}

TEST(RadarSelfEchoFilter, StartsCollectionWindowPerSensor)
{
  using camrod::sensing::radar_self_echo_filter::startupCalibrationAction;
  const double no_sample = std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(
    startupCalibrationAction(false, no_sample, 7.9, 8.0, 15.0),
    StartupCalibrationAction::kWait);
  EXPECT_EQ(
    startupCalibrationAction(false, 7.0, 14.9, 8.0, 15.0),
    StartupCalibrationAction::kWait);
  EXPECT_EQ(
    startupCalibrationAction(false, 7.0, 15.0, 8.0, 15.0),
    StartupCalibrationAction::kFinalizeSamples);
  EXPECT_EQ(
    startupCalibrationAction(false, no_sample, 15.0, 8.0, 15.0),
    StartupCalibrationAction::kRejectNoTimelySample);
  // HH_260728 - A late first packet does not get a fresh 8 s window; learning
  // remains startup-only even under delayed USB enumeration or CPU overload.
  EXPECT_EQ(
    startupCalibrationAction(false, 15.0, 15.0, 8.0, 15.0),
    StartupCalibrationAction::kRejectNoTimelySample);
  EXPECT_EQ(
    startupCalibrationAction(false, 16.2, 16.3, 8.0, 15.0),
    StartupCalibrationAction::kRejectNoTimelySample);
  EXPECT_EQ(
    startupCalibrationAction(true, 1.0, 20.0, 8.0, 15.0),
    StartupCalibrationAction::kDone);
}

}  // namespace
