// Verify the campsite crab terminal-speed curve and its steady-clock timeout
// without starting ROS or depending on simulated /clock progress.

#include <chrono>
#include <cmath>
#include <limits>

#include "camrod_control/campsite_crab_approach.hpp"
#include "gtest/gtest.h"

namespace camrod_control {

TEST(CampsiteAdoptedWaitReturnYaw, SharedDefaultMatchesDevelopReconstruction) {
  const double current = 30.0 * M_PI / 180.0;
  EXPECT_NEAR(
      campsiteAdoptedWaitReturnStartYaw(current, false, false),
      -150.0 * M_PI / 180.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(
      campsiteAdoptedWaitReturnStartYaw(current, true, false), current);
}

TEST(CampsiteAdoptedWaitReturnYaw, ReverseFlagDoesNotChangeDevelopAdoptionYaw) {
  const double current = 30.0 * M_PI / 180.0;
  EXPECT_DOUBLE_EQ(
      campsiteAdoptedWaitReturnStartYaw(current, true, true), current);
  EXPECT_NEAR(
      campsiteAdoptedWaitReturnStartYaw(current, false, true),
      -150.0 * M_PI / 180.0, 1.0e-12);
  EXPECT_TRUE(std::isnan(campsiteAdoptedWaitReturnStartYaw(
      std::numeric_limits<double>::quiet_NaN(), true, true)));
}

TEST(CampsiteCrabEntryBodyYawTarget,
     DisabledCompensationIsExactProductionIdentity) {
  // Preserve the nominal value byte-for-byte when the opt-in is disabled;
  // even the crab direction is intentionally irrelevant in this mode.
  const double nominal = 4.25;
  EXPECT_DOUBLE_EQ(campsiteCrabEntryBodyYawTarget(nominal, 0.0, 0.0), nominal);
  EXPECT_DOUBLE_EQ(campsiteCrabEntryBodyYawTarget(
                       nominal, std::numeric_limits<double>::quiet_NaN(), 0.0),
                   nominal);
}

TEST(CampsiteCrabEntryBodyYawTarget,
     EligibilityExcludesRoadsideManualAndDisabledProfiles) {
  EXPECT_TRUE(campsiteCrabEntryBodyYawCompensationEligible(2.0, true, true));
  EXPECT_FALSE(campsiteCrabEntryBodyYawCompensationEligible(0.0, true, true));
  EXPECT_FALSE(campsiteCrabEntryBodyYawCompensationEligible(2.0, false, true));
  EXPECT_FALSE(campsiteCrabEntryBodyYawCompensationEligible(2.0, true, false));
  EXPECT_FALSE(campsiteCrabEntryBodyYawCompensationEligible(
      std::numeric_limits<double>::quiet_NaN(), true, true));
}

TEST(CampsiteCrabEntryBodyYawTarget,
     DirectionSignedTwoDegreesCancelsEightyEightDegreeWheelClip) {
  constexpr double degrees_to_radians = 0.017453292519943295769236907684886;
  const double nominal = 37.0 * degrees_to_radians;
  const double left_body = campsiteCrabEntryBodyYawTarget(nominal, 1.0, 2.0);
  const double right_body = campsiteCrabEntryBodyYawTarget(nominal, -1.0, 2.0);

  EXPECT_NEAR(left_body, nominal + 2.0 * degrees_to_radians, 1.0e-12);
  EXPECT_NEAR(right_body, nominal - 2.0 * degrees_to_radians, 1.0e-12);
  EXPECT_NEAR(std::atan2(std::sin(left_body + 88.0 * degrees_to_radians -
                                  (nominal + 90.0 * degrees_to_radians)),
                         std::cos(left_body + 88.0 * degrees_to_radians -
                                  (nominal + 90.0 * degrees_to_radians))),
              0.0, 1.0e-12);
  EXPECT_NEAR(std::atan2(std::sin(right_body - 88.0 * degrees_to_radians -
                                  (nominal - 90.0 * degrees_to_radians)),
                         std::cos(right_body - 88.0 * degrees_to_radians -
                                  (nominal - 90.0 * degrees_to_radians))),
              0.0, 1.0e-12);
}

TEST(CampsiteCrabEntryBodyYawTarget, WrapsAndRejectsInvalidActiveInputs) {
  constexpr double degrees_to_radians = 0.017453292519943295769236907684886;
  EXPECT_NEAR(
      campsiteCrabEntryBodyYawTarget(179.0 * degrees_to_radians, 1.0, 2.0),
      -179.0 * degrees_to_radians, 1.0e-12);
  EXPECT_TRUE(std::isnan(campsiteCrabEntryBodyYawTarget(0.0, 0.0, 2.0)));
  EXPECT_TRUE(std::isnan(campsiteCrabEntryBodyYawTarget(
      0.0, std::numeric_limits<double>::quiet_NaN(), 2.0)));
  EXPECT_TRUE(std::isnan(campsiteCrabEntryBodyYawTarget(0.0, 1.0, -2.0)));
  EXPECT_TRUE(std::isnan(campsiteCrabEntryBodyYawTarget(
      0.0, 1.0, std::numeric_limits<double>::infinity())));
}

TEST(CampsiteCrabBodyYawSequence,
     NominalCenteringPrecedesCompensationAndZeroSkipsExtraAlignments) {
  EXPECT_EQ(selectCampsiteCrabPostNominalAlignmentAction(true, false, true),
            CampsiteCrabPostNominalAlignmentAction::kBeginAnchorCentering);
  EXPECT_EQ(selectCampsiteCrabPostNominalAlignmentAction(true, true, true),
            CampsiteCrabPostNominalAlignmentAction::kBeginBodyYawAlignment);
  EXPECT_EQ(selectCampsiteCrabPostNominalAlignmentAction(false, false, true),
            CampsiteCrabPostNominalAlignmentAction::kBeginBodyYawAlignment);
  EXPECT_EQ(selectCampsiteCrabPostNominalAlignmentAction(true, true, false),
            CampsiteCrabPostNominalAlignmentAction::kBeginCrabEntry);
  EXPECT_EQ(selectCampsiteCrabPostEntryAction(true),
            CampsiteCrabPostEntryAction::kRestoreNominalYaw);
  EXPECT_EQ(selectCampsiteCrabPostEntryAction(false),
            CampsiteCrabPostEntryAction::kBeginRotation);
}

TEST(CampsiteCrabBodyYawAlignment,
     TightToleranceDoesNotAcceptTwoDegreeCompensationAtNominalYaw) {
  constexpr double degrees_to_radians = 0.017453292519943295769236907684886;
  EXPECT_FALSE(
      campsiteBodyYawWithinTolerance(0.0, 2.0 * degrees_to_radians, 0.5));
  EXPECT_TRUE(campsiteBodyYawWithinTolerance(1.6 * degrees_to_radians,
                                             2.0 * degrees_to_radians, 0.5));
  EXPECT_FALSE(campsiteBodyYawWithinTolerance(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.5));
}

TEST(CampsiteDirectedYawRotation,
     LeftAndRightRestoreResidualsKeepRequestedTurnDirection) {
  constexpr double degrees_to_radians = 0.017453292519943295769236907684886;
  constexpr double pi = 3.1415926535897932384626433832795;
  const double nominal = 37.0 * degrees_to_radians;

  // The 0.2-degree residuals deliberately lie on the wrong side of nominal.
  // The live-start latch must still choose clockwise for left entry and
  // counter-clockwise for right entry instead of unwrapping almost 360
  // degrees in the opposite direction.
  const double left_delta = campsiteDirectedYawTargetDelta(
      nominal + 0.2 * degrees_to_radians, nominal - pi, -1.0);
  const double right_delta = campsiteDirectedYawTargetDelta(
      nominal - 0.2 * degrees_to_radians, nominal + pi, 1.0);
  EXPECT_LT(left_delta, -pi);
  EXPECT_GT(right_delta, pi);
  EXPECT_NEAR(left_delta, -180.2 * degrees_to_radians, 1.0e-12);
  EXPECT_NEAR(right_delta, 180.2 * degrees_to_radians, 1.0e-12);
}

TEST(CampsiteDirectedYawRotation,
     IncrementalProgressTreatsInitialJitterAsNoiseAndUnwrapsPiBoundary) {
  constexpr double degrees_to_radians = 0.017453292519943295769236907684886;
  EXPECT_NEAR(campsiteIncrementalYawStep(10.0 * degrees_to_radians,
                                         10.01 * degrees_to_radians),
              0.01 * degrees_to_radians, 1.0e-12);
  EXPECT_NEAR(campsiteIncrementalYawStep(-179.9 * degrees_to_radians,
                                         179.9 * degrees_to_radians),
              -0.2 * degrees_to_radians, 1.0e-12);
}

TEST(CampsiteDirectedYawRotation, InvalidLatchInputsFailClosed) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(std::isnan(campsiteDirectedYawTargetDelta(nan, 0.0, 1.0)));
  EXPECT_TRUE(std::isnan(campsiteDirectedYawTargetDelta(0.0, 0.0, 0.0)));
  EXPECT_TRUE(std::isnan(campsiteIncrementalYawStep(0.0, nan)));
}

TEST(CampsiteCrabEntryAction,
     ActiveCenteringRetainsOwnershipAcrossEntryThresholdJitter) {
  EXPECT_EQ(selectCampsiteCrabEntryAction(false, false, true, true, false),
            CampsiteCrabEntryAction::kContinueCentering);
  EXPECT_EQ(selectCampsiteCrabEntryAction(true, false, true, true, false),
            CampsiteCrabEntryAction::kContinueCentering);
  EXPECT_EQ(selectCampsiteCrabEntryAction(false, false, true, true, true),
            CampsiteCrabEntryAction::kBeginRotation);
}

TEST(CampsiteCrabEntryAction, InactiveCenteringPreservesExistingEntryPolicy) {
  EXPECT_EQ(selectCampsiteCrabEntryAction(false, false, true, false, false),
            CampsiteCrabEntryAction::kApproach);
  EXPECT_EQ(selectCampsiteCrabEntryAction(true, true, true, false, false),
            CampsiteCrabEntryAction::kRoadsideComplete);
  EXPECT_EQ(selectCampsiteCrabEntryAction(true, false, true, false, false),
            CampsiteCrabEntryAction::kBeginCentering);
  EXPECT_EQ(selectCampsiteCrabEntryAction(true, false, false, false, true),
            CampsiteCrabEntryAction::kBeginRotation);
}

TEST(CampsiteEntryAnchorCenteringAction,
     ActiveLatchIgnoresPositionJitterUntilSequencedCompletion) {
  EXPECT_EQ(selectCampsiteEntryAnchorCenteringAction(true, false, false),
            CampsiteEntryAnchorCenteringAction::kContinue);
  EXPECT_EQ(selectCampsiteEntryAnchorCenteringAction(true, true, false),
            CampsiteEntryAnchorCenteringAction::kComplete);
  EXPECT_EQ(selectCampsiteEntryAnchorCenteringAction(false, true, false),
            CampsiteEntryAnchorCenteringAction::kInactive);
}

TEST(CampsiteEntryAnchorCenteringAction, TimeoutWinsAtExactSteadyBoundary) {
  using Clock = std::chrono::steady_clock;
  const Clock::time_point start{};
  const auto timeout = std::chrono::seconds(15);
  EXPECT_FALSE(campsiteCrabSteadyTimeoutExpired(
      start, start + timeout - std::chrono::nanoseconds(1), timeout));
  const bool expired =
      campsiteCrabSteadyTimeoutExpired(start, start + timeout, timeout);
  EXPECT_TRUE(expired);
  EXPECT_EQ(selectCampsiteEntryAnchorCenteringAction(true, true, expired),
            CampsiteEntryAnchorCenteringAction::kTimeout);
}

TEST(CampsiteCrabEntrySafety, FixedEntryLineCrossTrackUsesAuthoredYaw) {
  constexpr double half_pi = 1.57079632679489661923;
  EXPECT_DOUBLE_EQ(campsiteCrabEntryLineCrossTrack(0.10, 2.0, 0.0, 0.0, 0.0),
                   0.10);
  EXPECT_NEAR(campsiteCrabEntryLineCrossTrack(-2.0, 0.10, 0.0, 0.0, half_pi),
              0.10, 1.0e-12);
}

TEST(CampsiteCrabEntrySafety, DisabledLimitsAreExactIdentity) {
  const auto result = checkCampsiteCrabEntrySafety(
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  EXPECT_FALSE(result.invalid_input);
  EXPECT_FALSE(result.heading_drift_exceeded);
  EXPECT_FALSE(result.cross_track_exceeded);
}

TEST(CampsiteCrabEntrySafety, HeadingAndCrossTrackFailClosedAboveBounds) {
  constexpr double degrees_to_radians = 0.017453292519943295769236907684886;
  const auto boundary = checkCampsiteCrabEntrySafety(5.0 * degrees_to_radians,
                                                     0.0, -0.10, 5.0, 0.10);
  EXPECT_FALSE(boundary.invalid_input);
  EXPECT_FALSE(boundary.heading_drift_exceeded);
  EXPECT_FALSE(boundary.cross_track_exceeded);

  const auto exceeded = checkCampsiteCrabEntrySafety(5.01 * degrees_to_radians,
                                                     0.0, -0.101, 5.0, 0.10);
  EXPECT_TRUE(exceeded.heading_drift_exceeded);
  EXPECT_TRUE(exceeded.cross_track_exceeded);

  const auto wrapped = checkCampsiteCrabEntrySafety(
      -179.0 * degrees_to_radians, 179.0 * degrees_to_radians, 0.0, 5.0, 0.10);
  EXPECT_NEAR(wrapped.heading_drift_deg, 2.0, 1.0e-10);
  EXPECT_FALSE(wrapped.heading_drift_exceeded);
}

TEST(CampsiteCrabEntrySafety, EnabledLimitRejectsNonfiniteEvidence) {
  const auto result = checkCampsiteCrabEntrySafety(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 5.0, 0.10);
  EXPECT_TRUE(result.invalid_input);
}

TEST(CampsiteCrabApproachSpeed, DisabledWindowKeepsMaximumSpeed) {
  EXPECT_DOUBLE_EQ(campsiteCrabApproachSpeed(0.40, 0.10, 0.0, 2.0), 0.40);
  EXPECT_DOUBLE_EQ(campsiteCrabApproachSpeed(0.40, 0.10, 0.0, 0.0), 0.40);
}

TEST(CampsiteCrabApproachSpeed, HoldsMaximumBeforeLastMetre) {
  EXPECT_DOUBLE_EQ(campsiteCrabApproachSpeed(0.40, 0.10, 1.0, 1.01), 0.40);
  EXPECT_DOUBLE_EQ(campsiteCrabApproachSpeed(0.40, 0.10, 1.0, 1.00), 0.40);
}

TEST(CampsiteCrabApproachSpeed, DeceleratesLinearlyInsideLastMetre) {
  EXPECT_NEAR(campsiteCrabApproachSpeed(0.40, 0.10, 1.0, 0.75), 0.325, 1.0e-12);
  EXPECT_NEAR(campsiteCrabApproachSpeed(0.40, 0.10, 1.0, 0.50), 0.250, 1.0e-12);
  EXPECT_NEAR(campsiteCrabApproachSpeed(0.40, 0.10, 1.0, 0.25), 0.175, 1.0e-12);
}

TEST(CampsiteCrabApproachSpeed, ClampsToMinimumAtCompletionBoundary) {
  EXPECT_DOUBLE_EQ(campsiteCrabApproachSpeed(0.40, 0.10, 1.0, 0.0), 0.10);
  EXPECT_DOUBLE_EQ(campsiteCrabApproachSpeed(0.40, 0.10, 1.0, -0.01), 0.10);
  EXPECT_DOUBLE_EQ(campsiteCrabApproachSpeed(0.40, 0.80, 1.0, 0.0), 0.40);
}

TEST(CampsiteCrabApproachDuration, DisabledProfileIsDistanceOverMaximumSpeed) {
  EXPECT_DOUBLE_EQ(campsiteCrabApproachDuration(2.0, 0.40, 0.10, 0.0), 5.0);
}

TEST(CampsiteCrabApproachDuration, IntegratesActiveLinearTerminalWindow) {
  const double expected_full_window = 1.0 / 0.30 * std::log(4.0);
  EXPECT_NEAR(campsiteCrabApproachDuration(2.0, 0.40, 0.10, 1.0),
              1.0 / 0.40 + expected_full_window, 1.0e-12);

  const double expected_partial_window = 1.0 / 0.30 * std::log(2.5);
  EXPECT_NEAR(campsiteCrabApproachDuration(0.5, 0.40, 0.10, 1.0),
              expected_partial_window, 1.0e-12);
  EXPECT_GT(campsiteCrabApproachDuration(2.0, 0.40, 0.10, 1.0),
            campsiteCrabApproachDuration(2.0, 0.40, 0.10, 0.0));
}

TEST(CampsiteCrabApproachDuration, InvalidInputsFailClosedAsNonfinite) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(std::isinf(campsiteCrabApproachDuration(nan, 0.40, 0.10, 1.0)));
  EXPECT_TRUE(std::isinf(campsiteCrabApproachDuration(1.0, 0.0, 0.10, 1.0)));
  EXPECT_TRUE(std::isinf(campsiteCrabApproachDuration(1.0, 0.40, 0.0, 1.0)));
  EXPECT_TRUE(std::isinf(campsiteCrabApproachDuration(-1.0, 0.40, 0.10, 1.0)));
}

TEST(CampsiteCrabSteadyTimeout, ExpiresAtExactBoundary) {
  using Clock = std::chrono::steady_clock;
  const Clock::time_point start{};
  const auto timeout = std::chrono::seconds(5);

  EXPECT_FALSE(campsiteCrabSteadyTimeoutExpired(
      start, start + timeout - std::chrono::nanoseconds(1), timeout));
  EXPECT_TRUE(
      campsiteCrabSteadyTimeoutExpired(start, start + timeout, timeout));
  EXPECT_TRUE(campsiteCrabSteadyTimeoutExpired(
      start, start + timeout + std::chrono::nanoseconds(1), timeout));
}

TEST(CampsiteCrabSteadyTimeout, DoesNotExpireForRegressedSample) {
  using Clock = std::chrono::steady_clock;
  const Clock::time_point start{std::chrono::seconds(10)};
  EXPECT_FALSE(campsiteCrabSteadyTimeoutExpired(
      start, start - std::chrono::nanoseconds(1), std::chrono::seconds(0)));
  EXPECT_TRUE(
      campsiteCrabSteadyTimeoutExpired(start, start, std::chrono::seconds(0)));
}

} // namespace camrod_control
