// Copyright 2026 hwanhonglee
//
// HH_260819 - Deterministic coverage for charger-current confirmation.  These
// tests keep the global regenerative-current guard independent from the narrow
// AprilTag docking fast path and require multiple physical BMS observations.
#include <gtest/gtest.h>

#include "camrod_platform/charging_detection_policy.hpp"

namespace camrod_platform
{
namespace
{

ChargingDetectionPolicy defaultPolicy()
{
  ChargingDetectionConfig config;
  config.minimum_samples = 2;
  config.global_confirm_s = 10.0;
  config.docking_fast_confirm_s = 1.5;
  config.release_s = 3.0;
  config.maximum_sample_gap_s = 1.0;
  config.assertion_false_grace_s = 0.75;
  return ChargingDetectionPolicy(config);
}

TEST(AprilTagChargingFastArm, RequiresExactHealthyModuleAndTerminalPhase)
{
  AprilTagChargingFastArm arm(1.5);
  EXPECT_FALSE(arm.observeStatus(
    "parking", "WAITING_FOR_CHARGING", true, 100.0, 100.0));
  EXPECT_FALSE(arm.observeStatus(
    "apriltag_parking_controller", "TAG_GUIDED_REVERSE", true, 100.0, 100.0));
  EXPECT_FALSE(arm.observeStatus(
    "apriltag_parking_controller", "FINAL_YAW_ALIGNMENT", false, 100.0, 100.0));

  EXPECT_TRUE(arm.observeStatus(
    "apriltag_parking_controller", "FINAL_YAW_ALIGNMENT", true, 100.0, 100.0));
  EXPECT_TRUE(arm.eligible(101.5));
  EXPECT_EQ(arm.phase(), "FINAL_YAW_ALIGNMENT");

  EXPECT_TRUE(arm.observeStatus(
    "apriltag_parking_controller", "WAITING_FOR_CHARGING", true, 101.5, 101.5));
  EXPECT_TRUE(arm.eligible(103.0));
  EXPECT_EQ(arm.phase(), "WAITING_FOR_CHARGING");
}

TEST(AprilTagChargingFastArm, RejectsStaleMissingAndFutureStatusStamps)
{
  AprilTagChargingFastArm arm(1.5);
  EXPECT_FALSE(arm.observeStatus(
    "apriltag_parking_controller", "WAITING_FOR_CHARGING", true, 0.0, 100.0));
  EXPECT_FALSE(arm.observeStatus(
    "apriltag_parking_controller", "WAITING_FOR_CHARGING", true, 98.4, 100.0));
  EXPECT_FALSE(arm.observeStatus(
    "apriltag_parking_controller", "WAITING_FOR_CHARGING", true, 100.2, 100.0));

  ASSERT_TRUE(arm.observeStatus(
    "apriltag_parking_controller", "WAITING_FOR_CHARGING", true, 100.0, 100.0));
  EXPECT_FALSE(arm.eligible(101.5001));
}

TEST(ChargingDetectionPolicy, GlobalPathStillRejectsSevenSecondRegenBurst)
{
  auto policy = defaultPolicy();
  EXPECT_FALSE(policy.observe(true, false, 0.0).charging);
  EXPECT_FALSE(policy.observe(true, false, 1.0).charging);
  EXPECT_FALSE(policy.observe(true, false, 7.0).charging);
  EXPECT_FALSE(policy.observe(false, false, 7.1).charging);

  ChargingDetectionResult confirmed;
  for (int i = 0; i <= 20; ++i) {
    confirmed = policy.observe(true, false, 20.0 + 0.5 * i);
  }
  EXPECT_TRUE(confirmed.changed);
  EXPECT_TRUE(confirmed.charging);
  EXPECT_EQ(confirmed.reason, ChargingTransitionReason::kGlobalConfirm);
  EXPECT_DOUBLE_EQ(confirmed.held_s, 10.0);
}

TEST(ChargingDetectionPolicy, FastPathNeedsContinuousFramesForOnePointFiveSeconds)
{
  auto policy = defaultPolicy();
  const auto first = policy.observe(true, true, 10.0);
  EXPECT_FALSE(first.charging);
  EXPECT_EQ(first.sample_count, 1);

  EXPECT_FALSE(policy.observe(true, true, 10.5).charging);
  EXPECT_FALSE(policy.observe(true, true, 11.0).charging);
  const auto confirmed = policy.observe(true, true, 11.5);
  EXPECT_TRUE(confirmed.changed);
  EXPECT_TRUE(confirmed.charging);
  EXPECT_EQ(confirmed.sample_count, 4);
  EXPECT_EQ(confirmed.reason, ChargingTransitionReason::kDockingFastConfirm);
}

TEST(ChargingDetectionPolicy, FastPathToleratesOneBriefFalseFrame)
{
  auto policy = defaultPolicy();
  EXPECT_FALSE(policy.observe(true, true, 10.0).charging);
  EXPECT_FALSE(policy.observe(true, true, 10.5).charging);

  const auto isolated_false = policy.observe(false, true, 10.75);
  EXPECT_FALSE(isolated_false.charging);
  EXPECT_EQ(isolated_false.sample_count, 2);

  const auto resumed = policy.observe(true, true, 11.0);
  EXPECT_FALSE(resumed.charging);
  EXPECT_EQ(resumed.sample_count, 3);
  EXPECT_DOUBLE_EQ(resumed.held_s, 0.75);
  EXPECT_FALSE(policy.observe(true, true, 11.5).charging);
  const auto confirmed = policy.observe(true, true, 11.75);
  EXPECT_TRUE(confirmed.changed);
  EXPECT_TRUE(confirmed.charging);
  EXPECT_EQ(confirmed.reason, ChargingTransitionReason::kDockingFastConfirm);
  EXPECT_DOUBLE_EQ(confirmed.held_s, 1.5);
}

TEST(ChargingDetectionPolicy, GlobalPathToleratesOneBriefFalseFrame)
{
  auto policy = defaultPolicy();
  for (int i = 0; i <= 8; ++i) {
    EXPECT_FALSE(policy.observe(true, false, 0.5 * i).charging);
  }
  EXPECT_FALSE(policy.observe(false, false, 4.25).charging);
  const auto resumed = policy.observe(true, false, 4.5);
  EXPECT_FALSE(resumed.charging);
  EXPECT_EQ(resumed.sample_count, 10);
  EXPECT_DOUBLE_EQ(resumed.held_s, 4.25);
  for (int i = 10; i < 20; ++i) {
    EXPECT_FALSE(policy.observe(true, false, 0.5 * i).charging);
  }
  EXPECT_FALSE(policy.observe(true, false, 10.0).charging);

  const auto confirmed = policy.observe(true, false, 10.25);
  EXPECT_TRUE(confirmed.changed);
  EXPECT_TRUE(confirmed.charging);
  EXPECT_EQ(confirmed.reason, ChargingTransitionReason::kGlobalConfirm);
  EXPECT_DOUBLE_EQ(confirmed.held_s, 10.0);
}

TEST(ChargingDetectionPolicy, TwoConsecutiveFalseFramesResetRisingDwell)
{
  auto policy = defaultPolicy();
  EXPECT_FALSE(policy.observe(true, true, 0.0).charging);
  EXPECT_FALSE(policy.observe(true, true, 0.5).charging);
  EXPECT_FALSE(policy.observe(false, true, 0.75).charging);
  EXPECT_FALSE(policy.observe(false, true, 1.0).charging);

  const auto restarted = policy.observe(true, true, 1.25);
  EXPECT_FALSE(restarted.charging);
  EXPECT_EQ(restarted.sample_count, 1);
  EXPECT_DOUBLE_EQ(restarted.held_s, 0.0);
  EXPECT_FALSE(policy.observe(true, true, 1.75).charging);
  EXPECT_FALSE(policy.observe(true, true, 2.25).charging);
  const auto confirmed = policy.observe(true, true, 2.75);
  EXPECT_TRUE(confirmed.changed);
  EXPECT_TRUE(confirmed.charging);
  EXPECT_EQ(confirmed.reason, ChargingTransitionReason::kDockingFastConfirm);
  EXPECT_DOUBLE_EQ(confirmed.held_s, 1.5);
}

TEST(ChargingDetectionPolicy, ExpiredFalseGraceResetsRisingDwell)
{
  ChargingDetectionConfig config;
  config.minimum_samples = 2;
  config.global_confirm_s = 10.0;
  config.docking_fast_confirm_s = 1.5;
  config.release_s = 3.0;
  config.maximum_sample_gap_s = 2.0;
  config.assertion_false_grace_s = 0.25;
  ChargingDetectionPolicy policy(config);

  EXPECT_FALSE(policy.observe(true, true, 0.0).charging);
  EXPECT_FALSE(policy.observe(true, true, 0.5).charging);
  EXPECT_FALSE(policy.observe(false, true, 0.6).charging);
  const auto restarted = policy.observe(true, true, 0.9);
  EXPECT_FALSE(restarted.charging);
  EXPECT_EQ(restarted.sample_count, 1);
  EXPECT_DOUBLE_EQ(restarted.held_s, 0.0);
  EXPECT_FALSE(policy.observe(true, true, 1.4).charging);
  EXPECT_FALSE(policy.observe(true, true, 1.9).charging);
  const auto confirmed = policy.observe(true, true, 2.4);
  EXPECT_TRUE(confirmed.charging);
  EXPECT_DOUBLE_EQ(confirmed.held_s, 1.5);
}

TEST(ChargingDetectionPolicy, IsolatedFalseGraceDoesNotAcceptSevenSecondRegen)
{
  auto policy = defaultPolicy();
  for (int i = 0; i <= 6; ++i) {
    EXPECT_FALSE(policy.observe(true, false, 0.5 * i).charging);
  }
  EXPECT_FALSE(policy.observe(false, false, 3.25).charging);
  for (int i = 7; i <= 14; ++i) {
    EXPECT_FALSE(policy.observe(true, false, 0.5 * i).charging);
  }
  EXPECT_FALSE(policy.observe(false, false, 7.25).charging);
}

TEST(ChargingDetectionPolicy, RepeatedIsolatedFalseIntervalsDoNotCountTowardGlobalHold)
{
  auto policy = defaultPolicy();
  EXPECT_FALSE(policy.observe(true, false, 0.0).charging);
  EXPECT_FALSE(policy.observe(true, false, 0.5).charging);
  EXPECT_FALSE(policy.observe(false, false, 0.75).charging);
  EXPECT_FALSE(policy.observe(true, false, 1.0).charging);
  EXPECT_FALSE(policy.observe(false, false, 1.25).charging);
  EXPECT_FALSE(policy.observe(true, false, 1.5).charging);

  for (int i = 4; i <= 20; ++i) {
    EXPECT_FALSE(policy.observe(true, false, 0.5 * i).charging);
  }
  const auto confirmed = policy.observe(true, false, 10.5);
  EXPECT_TRUE(confirmed.changed);
  EXPECT_TRUE(confirmed.charging);
  EXPECT_EQ(confirmed.reason, ChargingTransitionReason::kGlobalConfirm);
  EXPECT_DOUBLE_EQ(confirmed.held_s, 10.0);
}

TEST(ChargingDetectionPolicy, SparsePositiveFramesRestartFastDwell)
{
  auto policy = defaultPolicy();
  EXPECT_FALSE(policy.observe(true, true, 10.0).charging);

  // The second positive frame is distinct, but its 1.5 s gap exceeds the
  // measured-cadence allowance and therefore becomes a new first frame.
  const auto restarted = policy.observe(true, true, 11.5);
  EXPECT_FALSE(restarted.charging);
  EXPECT_EQ(restarted.sample_count, 1);
  EXPECT_DOUBLE_EQ(restarted.held_s, 0.0);

  EXPECT_FALSE(policy.observe(true, true, 12.0).charging);
  EXPECT_FALSE(policy.observe(true, true, 12.5).charging);
  const auto confirmed = policy.observe(true, true, 13.0);
  EXPECT_TRUE(confirmed.charging);
  EXPECT_EQ(confirmed.sample_count, 4);
  EXPECT_DOUBLE_EQ(confirmed.held_s, 1.5);
}

TEST(ChargingDetectionPolicy, FastPathNeverAcceptsOneFrameWhenGeneralMinimumIsOne)
{
  ChargingDetectionConfig config;
  config.minimum_samples = 1;
  config.global_confirm_s = 10.0;
  config.docking_fast_confirm_s = 0.0;
  config.release_s = 3.0;
  config.maximum_sample_gap_s = 1.0;
  config.assertion_false_grace_s = 0.75;
  ChargingDetectionPolicy policy(config);

  EXPECT_FALSE(policy.observe(true, true, 10.0).charging);
  const auto confirmed = policy.observe(true, true, 10.1);
  EXPECT_TRUE(confirmed.charging);
  EXPECT_EQ(confirmed.sample_count, 2);
}

TEST(ChargingDetectionPolicy, LosingFastArmResetsOnlyFastDwell)
{
  auto policy = defaultPolicy();
  EXPECT_FALSE(policy.observe(true, true, 0.0).charging);
  EXPECT_FALSE(policy.observe(true, true, 0.5).charging);
  EXPECT_FALSE(policy.observe(true, false, 1.0).charging);
  EXPECT_FALSE(policy.observe(true, true, 1.6).charging);
  EXPECT_FALSE(policy.observe(true, true, 2.1).charging);
  EXPECT_FALSE(policy.observe(true, true, 2.6).charging);
  const auto confirmed = policy.observe(true, true, 3.1);
  EXPECT_TRUE(confirmed.charging);
  EXPECT_EQ(confirmed.reason, ChargingTransitionReason::kDockingFastConfirm);
  EXPECT_NEAR(confirmed.held_s, 1.5, 1.0e-12);
}

TEST(ChargingDetectionPolicy, ReleaseStillRequiresThreeSeconds)
{
  auto policy = defaultPolicy();
  ASSERT_FALSE(policy.observe(true, true, 0.0).charging);
  ASSERT_FALSE(policy.observe(true, true, 0.5).charging);
  ASSERT_FALSE(policy.observe(true, true, 1.0).charging);
  ASSERT_TRUE(policy.observe(true, true, 1.5).charging);

  EXPECT_TRUE(policy.observe(false, false, 2.0).charging);
  EXPECT_TRUE(policy.observe(false, false, 2.5).charging);
  EXPECT_TRUE(policy.observe(false, false, 3.0).charging);
  EXPECT_TRUE(policy.observe(false, false, 3.5).charging);
  EXPECT_TRUE(policy.observe(false, false, 4.0).charging);
  EXPECT_TRUE(policy.observe(false, false, 4.5).charging);
  const auto released = policy.observe(false, false, 5.0);
  EXPECT_TRUE(released.changed);
  EXPECT_FALSE(released.charging);
  EXPECT_EQ(released.reason, ChargingTransitionReason::kRelease);
}

TEST(ChargingDetectionPolicy, AssertionFalseGraceDoesNotShortenReleaseDwell)
{
  auto policy = defaultPolicy();
  ASSERT_FALSE(policy.observe(true, true, 0.0).charging);
  ASSERT_FALSE(policy.observe(true, true, 0.5).charging);
  ASSERT_FALSE(policy.observe(true, true, 1.0).charging);
  ASSERT_TRUE(policy.observe(true, true, 1.5).charging);

  // The rising-edge one-frame grace is not consulted after confirmation. Even
  // consecutive false frames still require the configured 3 s release dwell.
  EXPECT_TRUE(policy.observe(false, false, 2.0).charging);
  EXPECT_TRUE(policy.observe(false, false, 2.25).charging);
  EXPECT_TRUE(policy.observe(false, false, 3.0).charging);
  EXPECT_TRUE(policy.observe(false, false, 4.0).charging);
  const auto released = policy.observe(false, false, 5.0);
  EXPECT_TRUE(released.changed);
  EXPECT_FALSE(released.charging);
  EXPECT_EQ(released.reason, ChargingTransitionReason::kRelease);
  EXPECT_DOUBLE_EQ(released.held_s, 3.0);
}

TEST(ChargingDetectionPolicy, SparseNegativeFramesRestartReleaseDwell)
{
  auto policy = defaultPolicy();
  ASSERT_FALSE(policy.observe(true, true, 0.0).charging);
  ASSERT_FALSE(policy.observe(true, true, 0.5).charging);
  ASSERT_FALSE(policy.observe(true, true, 1.0).charging);
  ASSERT_TRUE(policy.observe(true, true, 1.5).charging);

  EXPECT_TRUE(policy.observe(false, false, 2.0).charging);
  EXPECT_TRUE(policy.observe(false, false, 2.5).charging);
  const auto restarted = policy.observe(false, false, 4.0);
  EXPECT_TRUE(restarted.charging);
  EXPECT_EQ(restarted.sample_count, 1);
  EXPECT_DOUBLE_EQ(restarted.held_s, 0.0);

  ChargingDetectionResult released;
  for (int i = 1; i <= 6; ++i) {
    released = policy.observe(false, false, 4.0 + 0.5 * i);
  }
  EXPECT_TRUE(released.changed);
  EXPECT_FALSE(released.charging);
  EXPECT_EQ(released.reason, ChargingTransitionReason::kRelease);
  EXPECT_DOUBLE_EQ(released.held_s, 3.0);
}

TEST(ChargingDetectionPolicy, SparsePositiveFramesRestartGlobalDwell)
{
  auto policy = defaultPolicy();
  for (int i = 0; i <= 10; ++i) {
    EXPECT_FALSE(policy.observe(true, false, 0.5 * i).charging);
  }

  // A 1.5 s dropout at the middle of the global window discards the first 5 s.
  auto restarted = policy.observe(true, false, 6.5);
  EXPECT_FALSE(restarted.charging);
  EXPECT_EQ(restarted.sample_count, 1);
  EXPECT_DOUBLE_EQ(restarted.held_s, 0.0);

  ChargingDetectionResult confirmed;
  for (int i = 1; i <= 20; ++i) {
    confirmed = policy.observe(true, false, 6.5 + 0.5 * i);
  }
  EXPECT_TRUE(confirmed.changed);
  EXPECT_TRUE(confirmed.charging);
  EXPECT_EQ(confirmed.reason, ChargingTransitionReason::kGlobalConfirm);
  EXPECT_DOUBLE_EQ(confirmed.held_s, 10.0);
}

TEST(ChargingDetectionPolicy, RosTimeResetCannotReuseOldDwell)
{
  auto policy = defaultPolicy();
  EXPECT_FALSE(policy.observe(true, true, 100.0).charging);
  EXPECT_FALSE(policy.observe(true, true, 1.0).charging);
  EXPECT_FALSE(policy.observe(true, true, 1.5).charging);
  EXPECT_FALSE(policy.observe(true, true, 2.0).charging);
  EXPECT_TRUE(policy.observe(true, true, 2.5).charging);
}

}  // namespace
}  // namespace camrod_platform
