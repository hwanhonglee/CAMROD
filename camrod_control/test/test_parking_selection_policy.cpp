#include <limits>

#include "camrod_control/parking_selection_policy.hpp"
#include "gtest/gtest.h"

namespace camrod_control {

TEST(ParkingSelection, InitialReturnAlwaysReversesAndOnlyVerifiedExplicitDockReusesPark) {
  EXPECT_EQ(initialParkingMethod(false, false), ParkingMethod::kReverse);
  EXPECT_EQ(initialParkingMethod(false, true), ParkingMethod::kReverse);
  EXPECT_EQ(initialParkingMethod(true, false), ParkingMethod::kReverse);
  EXPECT_EQ(initialParkingMethod(true, true), ParkingMethod::kAprilTag);
}

TEST(ParkingSelection, BoundaryThirtyFiveUsesNonChargingReverse) {
  EXPECT_EQ(selectParkingMethod(35.0, 35.0, false), ParkingMethod::kReverse);
  EXPECT_EQ(selectParkingMethod(80.0, 35.0, false), ParkingMethod::kReverse);
  EXPECT_EQ(selectParkingMethod(34.999, 35.0, false), ParkingMethod::kAprilTag);
  EXPECT_EQ(selectParkingMethod(0.0, 35.0, false), ParkingMethod::kAprilTag);
}

TEST(ParkingSelection, MissingInvalidAndStaleBatteryRequireDocking) {
  for (const auto soc : {
      usableParkingBatteryPercent(false, 0.80F, 0.1, 2.0),
      usableParkingBatteryPercent(true, 0.80F, 2.01, 2.0),
      usableParkingBatteryPercent(true, 1.01F, 0.1, 2.0),
      usableParkingBatteryPercent(true, 80.0F, 0.1, 2.0),
      usableParkingBatteryPercent(true, -1.0F, 0.1, 2.0),
      usableParkingBatteryPercent(true, std::numeric_limits<float>::quiet_NaN(), 0.1, 2.0),
      usableParkingBatteryPercent(true, std::numeric_limits<float>::infinity(), 0.1, 2.0),
      usableParkingBatteryPercent(true, 0.80F, -0.1, 2.0)}) {
    EXPECT_FALSE(soc.has_value());
    EXPECT_EQ(selectParkingMethod(soc, 35.0, false), ParkingMethod::kAprilTag);
  }
  EXPECT_EQ(usableParkingBatteryPercent(true, 0.35F, 2.0, 2.0), 35.0);
}

// HH_260907 - Exercise the real platform wire scale, not synthetic percentages.
TEST(ParkingSelection, PlatformFractionAtObservedSocSelectsReverse) {
  for (const float fraction : {0.35F, 0.50F, 0.74F, 0.75F, 1.0F}) {
    const auto soc = usableParkingBatteryPercent(true, fraction, 0.1, 2.0);
    ASSERT_TRUE(soc.has_value());
    EXPECT_EQ(selectParkingMethod(soc, 35.0, false), ParkingMethod::kReverse);
    EXPECT_EQ(selectParkingMethod(soc, 35.0, true), ParkingMethod::kAprilTag);
  }
  EXPECT_EQ(usableParkingBatteryPercent(true, 0.75F, 0.1, 2.0), 75.0);
  EXPECT_EQ(usableParkingBatteryPercent(true, 0.74F, 0.1, 2.0), 74.0);
  EXPECT_EQ(usableParkingBatteryPercent(true, 1.0F, 0.1, 2.0), 100.0);
}

TEST(ParkingSelection, Float32ThirtyFiveBoundaryDoesNotForceCharging) {
  const float threshold = 0.35F;
  const auto percent = [](const float fraction) {
    return usableParkingBatteryPercent(true, fraction, 0.1, 2.0);
  };
  EXPECT_EQ(selectParkingMethod(percent(threshold), 35.0, false), ParkingMethod::kReverse);
  EXPECT_EQ(selectParkingMethod(percent(std::nextafter(threshold, 1.0F)), 35.0, false),
            ParkingMethod::kReverse);
  EXPECT_EQ(selectParkingMethod(percent(std::nextafter(threshold, 0.0F)), 35.0, false),
            ParkingMethod::kAprilTag);
  for (const float fraction : {0.0F, 0.25F, 0.349F}) {
    EXPECT_EQ(selectParkingMethod(percent(fraction), 35.0, false), ParkingMethod::kAprilTag);
  }
}

TEST(ParkingSelection, ExplicitDockingTokenOverridesEveryValidSoc) {
  for (const double soc : {0.0, 34.9, 35.0, 50.0, 100.0}) {
    EXPECT_EQ(selectParkingMethod(soc, 35.0, true), ParkingMethod::kAprilTag);
  }
  EXPECT_TRUE(hasForceDockingToken("robot_ui:force_docking"));
  EXPECT_TRUE(hasForceDockingToken("drop_zone:force_docking:attempt=2"));
  EXPECT_TRUE(hasForceDockingToken("force_docking"));
  EXPECT_FALSE(hasForceDockingToken("robot_ui:not_force_docking"));
  EXPECT_FALSE(hasForceDockingToken("force_docking_disabled"));
  EXPECT_FALSE(hasForceDockingToken("normal_return"));
}

TEST(ParkingOwnership, BothCancelsAndSelectedStartAreRequiredForOutput) {
  ParkingOwnershipPolicy policy;
  ASSERT_TRUE(policy.begin(ParkingMethod::kReverse));
  const auto attempt = policy.generation();
  EXPECT_FALSE(policy.owns(ParkingMethod::kReverse));
  EXPECT_FALSE(policy.acknowledgeStart(attempt));
  policy.acknowledgeCancel(ParkingMethod::kReverse, attempt);
  EXPECT_FALSE(policy.acknowledgeStart(attempt));
  policy.acknowledgeCancel(ParkingMethod::kAprilTag, attempt);
  EXPECT_TRUE(policy.acknowledgeStart(attempt));
  EXPECT_TRUE(policy.owns(ParkingMethod::kReverse));
  EXPECT_FALSE(policy.owns(ParkingMethod::kAprilTag));
}

TEST(ParkingOwnership, MovingAttemptDoesNotChangeAtSocBoundaryOrDuplicateStart) {
  ParkingOwnershipPolicy policy;
  ASSERT_TRUE(policy.begin(selectParkingMethod(35.0, 35.0, false)));
  const auto attempt = policy.generation();
  EXPECT_FALSE(policy.begin(selectParkingMethod(34.0, 35.0, false)));
  EXPECT_FALSE(policy.begin(selectParkingMethod(90.0, 35.0, true)));
  EXPECT_EQ(policy.selected(), ParkingMethod::kReverse);
  EXPECT_EQ(policy.generation(), attempt);
}

TEST(ParkingOwnership, CompletedParkCanTransferToDockingOnlyAfterFreshAcknowledgements) {
  ParkingOwnershipPolicy policy;
  ASSERT_TRUE(policy.begin(ParkingMethod::kReverse));
  const auto old_attempt = policy.generation();
  policy.acknowledgeCancel(ParkingMethod::kReverse, old_attempt);
  policy.acknowledgeCancel(ParkingMethod::kAprilTag, old_attempt);
  ASSERT_TRUE(policy.acknowledgeStart(old_attempt));
  policy.complete();
  EXPECT_TRUE(policy.owns(ParkingMethod::kReverse));
  ASSERT_TRUE(policy.begin(selectParkingMethod(34.0, 35.0, false)));
  EXPECT_FALSE(policy.owns(ParkingMethod::kReverse));
  EXPECT_FALSE(policy.owns(ParkingMethod::kAprilTag));
  policy.acknowledgeCancel(ParkingMethod::kReverse, old_attempt);
  policy.acknowledgeCancel(ParkingMethod::kAprilTag, old_attempt);
  EXPECT_FALSE(policy.acknowledgeStart(old_attempt));
  EXPECT_FALSE(policy.acknowledgeStart(policy.generation()));
}

TEST(ParkingOwnership, CancelInvalidatesLateAcksAndEveryOutputOwner) {
  ParkingOwnershipPolicy policy;
  ASSERT_TRUE(policy.begin(ParkingMethod::kAprilTag));
  const auto cancelled_attempt = policy.generation();
  policy.cancel();
  policy.cancel();
  policy.acknowledgeCancel(ParkingMethod::kReverse, cancelled_attempt);
  policy.acknowledgeCancel(ParkingMethod::kAprilTag, cancelled_attempt);
  EXPECT_FALSE(policy.acknowledgeStart(cancelled_attempt));
  EXPECT_FALSE(policy.busy());
  EXPECT_FALSE(policy.owns(ParkingMethod::kReverse));
  EXPECT_FALSE(policy.owns(ParkingMethod::kAprilTag));
  EXPECT_EQ(policy.selected(), ParkingMethod::kNone);
}

TEST(ParkingTelemetry, MovingCommandTimeoutRequiresExplicitFailure) {
  EXPECT_EQ(parkingTelemetryHealth("REVERSE_APPROACH", 0.51, 0.1, 0.5, 2.0),
            ParkingTelemetryHealth::kCommandTimeout);
  EXPECT_EQ(parkingTelemetryHealth("TAG_GUIDED_REVERSE", 0.5, 0.1, 0.5, 2.0),
            ParkingTelemetryHealth::kHealthy);
  EXPECT_EQ(parkingTelemetryHealth("FINAL_YAW_ALIGNMENT", 0.51, 0.1, 0.5, 2.0),
            ParkingTelemetryHealth::kCommandTimeout);
}

TEST(ParkingTelemetry, StationaryWaitsRequireStatusButNotVelocityHeartbeat) {
  for (const auto phase : {"PARKED", "WAITING_FOR_TAG", "WAITING_FOR_CHARGING", "WAIT_FOR_CHARGING", "ERROR"}) {
    EXPECT_EQ(parkingTelemetryHealth(phase, 90.0, 2.0, 0.5, 2.0),
              ParkingTelemetryHealth::kHealthy);
    EXPECT_EQ(parkingTelemetryHealth(phase, 0.0, 2.01, 0.5, 2.0),
              ParkingTelemetryHealth::kStatusTimeout);
  }
}

TEST(ParkingOwnership, TimeoutAbortAllowsRetryWithoutForwardingCancelledOwnerIdle) {
  ParkingOwnershipPolicy policy;
  ASSERT_TRUE(policy.begin(ParkingMethod::kAprilTag));
  const auto attempt = policy.generation();
  policy.acknowledgeCancel(ParkingMethod::kReverse, attempt);
  policy.acknowledgeCancel(ParkingMethod::kAprilTag, attempt);
  ASSERT_TRUE(policy.acknowledgeStart(attempt));
  policy.abort();
  EXPECT_FALSE(policy.busy());
  EXPECT_FALSE(policy.owns(ParkingMethod::kAprilTag));
  EXPECT_EQ(policy.selected(), ParkingMethod::kAprilTag);
  EXPECT_FALSE(policy.acknowledgeStart(attempt));
  EXPECT_TRUE(policy.begin(ParkingMethod::kAprilTag));
}

} // namespace camrod_control
