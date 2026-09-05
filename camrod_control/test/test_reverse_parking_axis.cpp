// HH_260721 - Verify reverse parking axis conventions in native C++.

#include <cmath>
#include <limits>

#include "camrod_control/apriltag_parking_retry.hpp"
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

TEST(AprilTagParkingRetry, RawParametersAcceptExactHardBoundsBeforeStorage)
{
  AprilTagParkingRetryRawParameters parameters;
  parameters.forward_distance_m = 0.10;
  parameters.forward_speed_mps = 0.02;
  parameters.forward_timeout_s = 1.0;
  parameters.yaw_alignment_timeout_s = 1.0;
  parameters.maximum_retries = 1;
  EXPECT_TRUE(aprilTagParkingRetryRawParametersValid(
    parameters, 0.03, 0.10, 0.40));

  parameters.forward_distance_m = 1.00;
  parameters.forward_speed_mps = 0.20;
  parameters.forward_timeout_s = 30.0;
  parameters.yaw_alignment_timeout_s = 15.0;
  parameters.maximum_retries = std::numeric_limits<int>::max();
  EXPECT_TRUE(aprilTagParkingRetryRawParametersValid(
    parameters, 0.03, 0.10, 0.40));
}

TEST(AprilTagParkingRetry, RawParametersRejectMalformedValuesWithoutNormalization)
{
  const AprilTagParkingRetryRawParameters valid;
  auto invalid = valid;

  invalid.forward_distance_m = -0.8;
  EXPECT_FALSE(aprilTagParkingRetryRawParametersValid(
    invalid, 0.03, 0.10, 0.40));
  invalid = valid;
  invalid.forward_speed_mps = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(aprilTagParkingRetryRawParametersValid(
    invalid, 0.03, 0.10, 0.40));
  invalid = valid;
  invalid.forward_timeout_s = 30.01;
  EXPECT_FALSE(aprilTagParkingRetryRawParametersValid(
    invalid, 0.03, 0.10, 0.40));
  invalid = valid;
  invalid.yaw_alignment_timeout_s = 0.0;
  EXPECT_FALSE(aprilTagParkingRetryRawParametersValid(
    invalid, 0.03, 0.10, 0.40));
  invalid = valid;
  invalid.maximum_lateral_error_m = 0.03;
  EXPECT_FALSE(aprilTagParkingRetryRawParametersValid(
    invalid, 0.03, 0.10, 0.40));
  invalid = valid;
  invalid.maximum_heading_error_rad = 0.10;
  EXPECT_FALSE(aprilTagParkingRetryRawParametersValid(
    invalid, 0.03, 0.10, 0.40));
  invalid = valid;
  invalid.maximum_forward_exit_lateral_drift_m = -0.15;
  EXPECT_FALSE(aprilTagParkingRetryRawParametersValid(
    invalid, 0.03, 0.10, 0.40));
  invalid = valid;
  invalid.maximum_odometry_step_m = 0.0;
  EXPECT_FALSE(aprilTagParkingRetryRawParametersValid(
    invalid, 0.03, 0.10, 0.40));
  invalid = valid;
  invalid.minimum_tag_distance_m = 0.46;
  EXPECT_FALSE(aprilTagParkingRetryRawParametersValid(
    invalid, 0.03, 0.10, 0.40));
  invalid = valid;
  invalid.maximum_retries = 0;
  EXPECT_FALSE(aprilTagParkingRetryRawParametersValid(
    invalid, 0.03, 0.10, 0.40));
  invalid.maximum_retries =
    static_cast<int64_t>(std::numeric_limits<int>::max()) + 1;
  EXPECT_FALSE(aprilTagParkingRetryRawParametersValid(
    invalid, 0.03, 0.10, 0.40));
  EXPECT_FALSE(aprilTagParkingRetryRawParametersValid(
    valid, std::numeric_limits<double>::quiet_NaN(), 0.10, 0.40));
}

TEST(AprilTagParkingRetry, DisabledFeatureAcceptsInvalidLegacyRetryValues)
{
  AprilTagParkingRetryRawParameters legacy_parameters;
  legacy_parameters.forward_distance_m = -1.0;
  legacy_parameters.maximum_retries = 0;
  const bool raw_parameters_valid = aprilTagParkingRetryRawParametersValid(
    legacy_parameters, 0.03, 0.10, 0.40);

  ASSERT_FALSE(raw_parameters_valid);
  EXPECT_TRUE(
    aprilTagParkingRetryStartupPermitted(
      false, raw_parameters_valid));
}

TEST(AprilTagParkingRetry, EnabledFeatureRejectsInvalidRetryValuesFailClosed)
{
  AprilTagParkingRetryRawParameters parameters;
  parameters.forward_speed_mps =
    std::numeric_limits<double>::quiet_NaN();
  const bool raw_parameters_valid = aprilTagParkingRetryRawParametersValid(
    parameters, 0.03, 0.10, 0.40);

  ASSERT_FALSE(raw_parameters_valid);
  EXPECT_FALSE(
    aprilTagParkingRetryStartupPermitted(
      true, raw_parameters_valid));
}

TEST(AprilTagParkingRetry, RequiresEveryFreshBoundedOptInCondition)
{
  AprilTagParkingRetryConfig config;
  config.enabled = true;
  config.maximum_retries = 2;

  EXPECT_TRUE(aprilTagParkingRetryEligible(
    config, 0, true, true, false, 0.399, -0.088, 0.249));
  EXPECT_FALSE(aprilTagParkingRetryEligible(
    config, 2, true, true, false, 0.399, -0.088, 0.249));
  EXPECT_FALSE(aprilTagParkingRetryEligible(
    config, 0, false, true, false, 0.399, -0.088, 0.249));
  EXPECT_FALSE(aprilTagParkingRetryEligible(
    config, 0, true, false, false, 0.399, -0.088, 0.249));
  EXPECT_FALSE(aprilTagParkingRetryEligible(
    config, 0, true, true, true, 0.399, -0.088, 0.249));

  config.enabled = false;
  EXPECT_FALSE(aprilTagParkingRetryEligible(
    config, 0, true, true, false, 0.399, -0.088, 0.249));
}

TEST(AprilTagParkingRetry, RejectsAcceptedPoseBoundaryViolations)
{
  AprilTagParkingRetryConfig config;
  config.enabled = true;
  config.maximum_retries = 1;

  EXPECT_FALSE(aprilTagParkingRetryEligible(
    config, 0, true, true, false, 0.399, 0.030, 0.10));
  EXPECT_FALSE(aprilTagParkingRetryEligible(
    config, 0, true, true, false, 0.399, 0.151, 0.10));
  EXPECT_FALSE(aprilTagParkingRetryEligible(
    config, 0, true, true, false, 0.399, 0.08, 0.351));
  EXPECT_FALSE(aprilTagParkingRetryEligible(
    config, 0, true, true, false, 0.349, 0.08, 0.10));
  EXPECT_FALSE(aprilTagParkingRetryEligible(
    config, 0, true, true, false, 0.451, 0.08, 0.10));
  EXPECT_FALSE(aprilTagParkingRetryEligible(
    config, 0, true, true, false,
    std::numeric_limits<double>::quiet_NaN(), 0.08, 0.10));
}

TEST(AprilTagParkingRetry, ProgressIsSignedAndPathLengthBounded)
{
  AprilTagParkingRetryProgress progress;
  EXPECT_FALSE(progress.begin(
    0.0, 0.0, std::numeric_limits<double>::quiet_NaN()));
  ASSERT_TRUE(progress.begin(0.0, 0.0, 0.0));
  ASSERT_TRUE(progress.observe(0.40, 0.0));
  EXPECT_NEAR(progress.forwardProgress(), 0.40, 1.0e-12);
  EXPECT_NEAR(progress.lateralDrift(), 0.0, 1.0e-12);
  EXPECT_FALSE(progress.reached(0.80));

  ASSERT_TRUE(progress.observe(0.40, 0.30));
  EXPECT_NEAR(progress.distance(), 0.70, 1.0e-12);
  EXPECT_NEAR(progress.forwardProgress(), 0.40, 1.0e-12);
  EXPECT_NEAR(progress.lateralDrift(), 0.30, 1.0e-12);
  EXPECT_TRUE(progress.exceededPathLimit(0.50, 0.10));

  ASSERT_TRUE(progress.begin(1.0, 2.0, M_PI_2));
  ASSERT_TRUE(progress.observe(1.0, 2.81));
  EXPECT_TRUE(progress.reached(0.80));
  EXPECT_NEAR(progress.forwardProgress(), 0.81, 1.0e-12);
  EXPECT_NEAR(progress.lateralDrift(), 0.0, 1.0e-12);
  ASSERT_TRUE(progress.begin(0.0, 0.0, 0.0));
  ASSERT_TRUE(progress.observe(-0.03, 0.0));
  EXPECT_LT(progress.forwardProgress(), -0.02);

  ASSERT_TRUE(progress.begin(0.0, 0.0, 0.0));
  ASSERT_TRUE(progress.observe(0.80, 0.0));
  EXPECT_TRUE(progress.reached(0.80));
  EXPECT_DOUBLE_EQ(progress.lastStep(), 0.80);
  // Runtime rejects this apparent completion against its audited 0.10 m
  // maximum odometry step, so a localization jump cannot complete a retry.
  EXPECT_GT(progress.lastStep(), 0.10);
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
