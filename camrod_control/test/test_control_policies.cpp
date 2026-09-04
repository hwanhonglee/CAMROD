// HH_260721 - Verify command gating, charging mission override, and
// all-direction cost stopping.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "avg_msgs/msg/avg_occupancy_grid.hpp"
#include "avg_msgs/msg/avg_path.hpp"
#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "avg_msgs/msg/avg_twist.hpp"
#include "camrod_control/charging_mission_override.hpp"
#include "camrod_control/bounded_recovery_attempt_policy.hpp"
#include "camrod_control/cmd_vel_gate_policy.hpp"
#include "camrod_control/command_source_arbiter.hpp"
#include "camrod_control/drop_zone_charging_departure_authorization.hpp"
#include "camrod_control/motion_cost_stop.hpp"
#include "camrod_control/motion_geometry.hpp"
#include "camrod_control/route_recovery_candidate.hpp"
#include "camrod_control/route_safety_recovery.hpp"
#include "camrod_control/yaw_alignment_settling.hpp"
#include "camrod_sensor_kit/robot_boundary.hpp"
#include "gtest/gtest.h"

namespace camrod_control {

namespace {

TEST(BoundedRecoveryAttemptPolicy, RetriesAttemptWithoutRemovingTotalEnvelope) {
  const BoundedRecoveryAttemptLimits limits{10.0, 0.40, 50, 90.0, 1.50};
  EXPECT_EQ(
      EvaluateBoundedRecoveryAttempt(limits, 1, 10.0, 0.10, 10.0, 0.10),
      BoundedRecoveryAction::kRetry);
  EXPECT_EQ(
      EvaluateBoundedRecoveryAttempt(limits, 12, 1.0, 0.05, 89.0, 1.49),
      BoundedRecoveryAction::kContinue);
}

TEST(BoundedRecoveryAttemptPolicy, TotalTravelAndTimeRemainFinalStops) {
  const BoundedRecoveryAttemptLimits limits{10.0, 0.40, 50, 90.0, 1.50};
  EXPECT_EQ(
      EvaluateBoundedRecoveryAttempt(limits, 4, 1.0, 0.10, 90.0, 0.50),
      BoundedRecoveryAction::kFinalHold);
  EXPECT_EQ(
      EvaluateBoundedRecoveryAttempt(limits, 4, 1.0, 0.10, 30.0, 1.50),
      BoundedRecoveryAction::kFinalHold);
  EXPECT_EQ(
      EvaluateBoundedRecoveryAttempt(limits, 50, 10.0, 0.10, 80.0, 0.50),
      BoundedRecoveryAction::kFinalHold);
}

double degrees(const double value) { return value * M_PI / 180.0; }

avg_msgs::msg::AvgPoseStamped makeMotionPose(
    const double x, const double y, const double yaw_degrees) {
  avg_msgs::msg::AvgPoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  const double yaw = degrees(yaw_degrees);
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

TEST(YawAlignmentSettling, RequiresFreshStableSamplesBeforeTranslationHandoff) {
  YawAlignmentSettling settling({5.0, 1.0, 3.0});

  EXPECT_FALSE(settling.observe(degrees(10.0), degrees(0.0), 0.0));
  // Entering tolerance at 6 deg/s is still rotating, so the dwell does not
  // start.
  EXPECT_FALSE(settling.observe(degrees(4.0), degrees(6.0), 1.0));
  EXPECT_FALSE(settling.observe(degrees(3.5), degrees(6.5), 2.0));
  EXPECT_TRUE(settling.withinTolerance());
  EXPECT_NEAR(settling.yawRateDegps(), 0.5, 1.0e-9);

  // Re-evaluating one stale pose cannot satisfy the hold.
  EXPECT_FALSE(settling.observe(degrees(3.5), degrees(6.5), 2.0));
  EXPECT_FALSE(settling.observe(degrees(3.4), degrees(6.6), 2.5));
  EXPECT_TRUE(settling.observe(degrees(3.3), degrees(6.7), 3.0));
  EXPECT_NEAR(settling.stableDurationS(), 1.0, 1.0e-9);
}

TEST(YawAlignmentSettling, LeavingToleranceRestartsTheContinuousHold) {
  YawAlignmentSettling settling({4.0, 0.8, 3.0});

  EXPECT_FALSE(settling.observe(degrees(3.0), degrees(7.0), 0.0));
  EXPECT_FALSE(settling.observe(degrees(2.9), degrees(7.1), 0.1));
  EXPECT_FALSE(settling.observe(degrees(2.8), degrees(7.2), 0.6));
  EXPECT_FALSE(settling.observe(degrees(5.0), degrees(5.0), 0.7));
  EXPECT_FALSE(settling.withinTolerance());
  EXPECT_DOUBLE_EQ(settling.stableDurationS(), 0.0);

  EXPECT_FALSE(settling.observe(degrees(3.0), degrees(7.0), 1.0));
  EXPECT_FALSE(settling.observe(degrees(2.9), degrees(7.1), 1.4));
  EXPECT_TRUE(settling.observe(degrees(2.8), degrees(7.2), 2.2));
}

TEST(MotionGeometry,
     BodyTranslationTowardTargetCorrectsBothAxesAtBoundedSpeed) {
  const auto diagonal =
      bodyTranslationTowardTarget(0.0, 0.0, M_PI, 1.0, 1.0, 0.5);
  EXPECT_NEAR(diagonal.first, -0.5 / std::sqrt(2.0), 1.0e-9);
  EXPECT_NEAR(diagonal.second, -0.5 / std::sqrt(2.0), 1.0e-9);
  EXPECT_NEAR(std::hypot(diagonal.first, diagonal.second), 0.5, 1.0e-9);

  const auto lateral =
      bodyTranslationTowardTarget(0.0, 0.0, M_PI_2, 1.0, 0.0, 0.5);
  EXPECT_NEAR(lateral.first, 0.0, 1.0e-9);
  EXPECT_NEAR(lateral.second, -0.5, 1.0e-9);

  const auto stopped =
      bodyTranslationTowardTarget(1.0, 1.0, 0.0, 1.0, 1.0, 0.5);
  EXPECT_DOUBLE_EQ(stopped.first, 0.0);
  EXPECT_DOUBLE_EQ(stopped.second, 0.0);
}

TEST(MotionGeometry,
     CampsiteReturnUsesPureCrabBeforeStraightDriftCorrection) {
  const auto lateral = bodyAxisPrioritizedTranslationTowardTarget(
      0.0, 0.0, 0.0, 1.0, 1.0, 0.5, 4.0, 0.04);
  EXPECT_DOUBLE_EQ(lateral.first, 0.0);
  EXPECT_DOUBLE_EQ(lateral.second, 0.5);

  const auto longitudinal = bodyAxisPrioritizedTranslationTowardTarget(
      0.0, 0.98, 0.0, 1.0, 1.0, 0.5, 4.0, 0.04);
  EXPECT_DOUBLE_EQ(longitudinal.first, 0.5);
  EXPECT_DOUBLE_EQ(longitudinal.second, 0.0);

  // After the on-site 180 degree turn, body axes are reversed but each command
  // must still contain only one translational component.
  const auto reversed = bodyAxisPrioritizedTranslationTowardTarget(
      1.0, 1.0, M_PI, 0.0, 0.0, 0.5, 4.0, 0.04);
  EXPECT_DOUBLE_EQ(reversed.first, 0.0);
  EXPECT_DOUBLE_EQ(reversed.second, 0.5);
}

TEST(MotionGeometry, RestartedTurnaroundUsesCurrentHeadingAndSameMapAnchor) {
  avg_msgs::msg::AvgPoseStamped anchor;
  avg_msgs::msg::AvgPoseStamped site;
  anchor.pose.orientation.w = 1.0;
  site.pose.position.y = -2.0;

  const auto normal = relativeXyAtHeading(anchor, site, 0.0);
  const auto reversed = relativeXyAtHeading(anchor, site, M_PI);
  EXPECT_NEAR(normal.second, -2.0, 1.0e-9);
  EXPECT_NEAR(reversed.second, 2.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(turnaroundDirectionForCrab(normal.second), 1.0);
  EXPECT_DOUBLE_EQ(turnaroundDirectionForCrab(reversed.second), -1.0);
  EXPECT_DOUBLE_EQ(anchor.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(anchor.pose.position.y, 0.0);
}

TEST(MotionGeometry, AutomaticCampsiteEntryUsesLaneletYawButRestartUsesLiveYaw) {
  constexpr double live_yaw = -58.38 * M_PI / 180.0;
  constexpr double lanelet_yaw = -66.42 * M_PI / 180.0;

  const auto automatic =
      selectCampsiteEntryYaw(live_yaw, lanelet_yaw, true);
  EXPECT_EQ(automatic.source, CampsiteEntryYawSource::kLaneletSnap);
  EXPECT_NEAR(automatic.yaw_rad, lanelet_yaw, 1.0e-12);

  const auto restarted =
      selectCampsiteEntryYaw(live_yaw, lanelet_yaw, false);
  EXPECT_EQ(restarted.source, CampsiteEntryYawSource::kLivePoseFallback);
  EXPECT_NEAR(restarted.yaw_rad, live_yaw, 1.0e-12);

  const auto missing_snap =
      selectCampsiteEntryYaw(live_yaw, std::nullopt, true);
  EXPECT_EQ(missing_snap.source,
            CampsiteEntryYawSource::kLivePoseFallback);
  EXPECT_NEAR(missing_snap.yaw_rad, live_yaw, 1.0e-12);
}

TEST(MotionGeometry, AllActiveCampsitesUseAuthoredYawAndSignedCrabSide) {
  struct Fixture {
    double snap_x;
    double snap_y;
    double site_x;
    double site_y;
    double snap_yaw_deg;
    double signed_lateral_m;
    double operational_offset_m;
    bool roadside;
  };
  // HH_260824 - Locked against active lanelet2_maps.osm map-v22 and
  // camrod_planning/config/camping_sites.yaml. Signed values catch both a yaw
  // reversal and the reported B6 live-yaw axial skew.
  constexpr std::array<Fixture, 13> fixtures{{
      {22.357515191, -6.908337803, 25.8687, -5.13869, -63.251744,
       3.931930, 3.931930, false},
      {22.454856615, -7.101474235, 19.566, -8.55747, -63.251706,
       -3.235030, 3.235030, false},
      {19.772336735, -1.222936079, 23.3585, 0.427582, -65.285918,
       3.947756, 3.947756, false},
      {19.307410106, -0.152900333, 16.4276, -1.34999, -67.428154,
       -3.118706, 3.118706, false},
      {17.382903354, 4.336168303, 20.9591, 5.86183, -66.896141,
       3.888036, 3.888036, false},
      {16.786389776, 5.715787406, 13.9518, 4.47866, -66.421645,
       -3.092795, 3.092795, false},
      {15.085466253, 9.670478273, 18.6442, 11.1853, -66.942319,
       3.867722, 3.867722, false},
      {14.147071672, 11.827906846, 11.3705, 10.645, -66.924479,
       -3.018049, 3.018049, false},
      {12.679983302, 15.667433959, 16.3636, 17.1396, -68.215752,
       3.966901, 3.966901, false},
      {11.891069361, 17.727651240, 9.02731, 16.6389, -69.184081,
       -3.063739, 3.063739, false},
      {10.821530478, 20.516472190, 14.8445, 22.0782, -68.783648,
       4.315470, 0.300000, true},
      {10.103421888, 23.093735129, 6.85414, 22.4291, -78.439719,
       -3.316560, 0.300000, true},
      {9.626348588, 27.418616995, 0.610449, 27.604, -91.177942,
       -9.017805, 0.300000, true},
  }};

  for (std::size_t index = 0; index < fixtures.size(); ++index) {
    const auto & fixture = fixtures[index];
    SCOPED_TRACE("B" + std::to_string(index + 1U));
    avg_msgs::msg::AvgPoseStamped snap;
    snap.pose.position.x = fixture.snap_x;
    snap.pose.position.y = fixture.snap_y;
    const double snap_yaw = degrees(fixture.snap_yaw_deg);
    snap.pose.orientation.z = std::sin(snap_yaw * 0.5);
    snap.pose.orientation.w = std::cos(snap_yaw * 0.5);
    avg_msgs::msg::AvgPoseStamped site;
    site.pose.position.x = fixture.site_x;
    site.pose.position.y = fixture.site_y;
    site.pose.orientation.w = 1.0;

    const auto selected = selectCampsiteEntryYaw(
        normalizeAngle(snap_yaw + M_PI), yawFromPose(snap), true);
    ASSERT_EQ(selected.source, CampsiteEntryYawSource::kLaneletSnap);
    EXPECT_NEAR(normalizeAngle(selected.yaw_rad - snap_yaw), 0.0,
                degrees(0.2));
    const auto relative = relativeXyAtHeading(snap, site, selected.yaw_rad);
    EXPECT_NEAR(relative.first, 0.0, 0.01);
    EXPECT_NEAR(relative.second, fixture.signed_lateral_m, 0.02);
    const double direction = relative.second >= 0.0 ? 1.0 : -1.0;
    EXPECT_DOUBLE_EQ(direction,
                     fixture.signed_lateral_m >= 0.0 ? 1.0 : -1.0);
    const double clamped_offset = clamp(std::abs(relative.second), 0.2, 7.0);
    const double operational_offset =
        fixture.roadside ? std::min(clamped_offset, 0.30) : clamped_offset;
    EXPECT_NEAR(operational_offset, fixture.operational_offset_m, 0.02);

    const auto operational_target = lateralTargetFromAnchor(
        fixture.snap_x, fixture.snap_y, selected.yaw_rad, direction,
        operational_offset);
    avg_msgs::msg::AvgPoseStamped target;
    target.pose.position.x = operational_target.first;
    target.pose.position.y = operational_target.second;
    const auto target_relative =
        relativeXyAtHeading(snap, target, selected.yaw_rad);
    EXPECT_NEAR(target_relative.first, 0.0, 1.0e-9);
    EXPECT_NEAR(target_relative.second, direction * operational_offset,
                1.0e-9);
    if (fixture.roadside) {
      EXPECT_FALSE(roadsideOperationalArrivalMatches(
          0.0, 0.0, direction * operational_offset, 0.60, 0.15));
      EXPECT_TRUE(roadsideOperationalArrivalMatches(
          0.59, direction * (operational_offset - 0.15),
          direction * operational_offset, 0.60, 0.15));
      EXPECT_FALSE(roadsideOperationalArrivalMatches(
          0.61, direction * operational_offset,
          direction * operational_offset, 0.60, 0.15));
    }
  }
}

TEST(MotionGeometry, CampsiteMotionPoseRejectsInvalidPositionAndQuaternion) {
  avg_msgs::msg::AvgPoseStamped pose;
  pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(poseHasFiniteMotionGeometry(pose));

  pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(poseHasFiniteMotionGeometry(pose));
  pose.pose.position.x = 0.0;
  pose.pose.position.y = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(poseHasFiniteMotionGeometry(pose));

  pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(poseHasFiniteMotionGeometry(pose));
  EXPECT_TRUE(poseHasFinitePlanarPosition(pose));
  pose.pose.orientation.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(poseHasFiniteMotionGeometry(pose));

  pose.pose.position.x = std::numeric_limits<double>::infinity();
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  EXPECT_FALSE(poseHasFinitePlanarPosition(pose));
  EXPECT_FALSE(poseHasFiniteMotionGeometry(pose));
}

TEST(MotionGeometry, B6RestartRequiresRealFreshFiniteSnapAnchor) {
  avg_msgs::msg::AvgPoseStamped site;
  site.header.frame_id = "map";
  site.pose.position.x = 13.9518;
  site.pose.position.y = 4.47866;
  site.pose.orientation.w = 1.0;
  std::optional<avg_msgs::msg::AvgPoseStamped> missing;
  EXPECT_FALSE(campsiteAdoptAnchorIsFreshFinite(missing, site, 0.0, 1.0));

  avg_msgs::msg::AvgPoseStamped snap;
  snap.header.frame_id = "map";
  snap.pose.position.x = 16.786389776;
  snap.pose.position.y = 5.715787406;
  const double snap_yaw = degrees(-66.421645);
  snap.pose.orientation.z = std::sin(snap_yaw * 0.5);
  snap.pose.orientation.w = std::cos(snap_yaw * 0.5);
  EXPECT_TRUE(campsiteAdoptAnchorIsFreshFinite(snap, site, 0.1, 1.0));
  const auto relative = relativeXyAtHeading(snap, site, yawFromPose(snap));
  EXPECT_NEAR(relative.first, 0.0, 0.01);
  EXPECT_NEAR(relative.second, -3.092795, 0.02);

  avg_msgs::msg::AvgPoseStamped adjacent_b5_snap = snap;
  adjacent_b5_snap.pose.position.x = 17.382903354;
  adjacent_b5_snap.pose.position.y = 4.336168303;
  const double b5_yaw = degrees(-66.896141);
  adjacent_b5_snap.pose.orientation.z = std::sin(b5_yaw * 0.5);
  adjacent_b5_snap.pose.orientation.w = std::cos(b5_yaw * 0.5);
  EXPECT_FALSE(campsiteAdoptAnchorsCorrelated(
      snap, adjacent_b5_snap, 0.90));
  EXPECT_TRUE(campsiteAdoptAnchorsCorrelated(snap, snap, 0.90));

  EXPECT_FALSE(campsiteAdoptAnchorIsFreshFinite(snap, site, 1.1, 1.0));
  snap.pose.orientation.x = 0.0;
  snap.pose.orientation.y = 0.0;
  snap.pose.orientation.z = 0.0;
  snap.pose.orientation.w = 0.0;
  EXPECT_FALSE(campsiteAdoptAnchorIsFreshFinite(snap, site, 0.1, 1.0));
}

TEST(MotionGeometry, AutomaticCrabEntryAlignmentHasBoundedTimeoutOnly) {
  EXPECT_FALSE(automaticCrabEntryAlignmentTimedOut(true, 14.99, 15.0));
  EXPECT_TRUE(automaticCrabEntryAlignmentTimedOut(true, 15.01, 15.0));
  // The existing reverse-entry alignment policy is intentionally unchanged.
  EXPECT_FALSE(automaticCrabEntryAlignmentTimedOut(false, 100.0, 15.0));
  EXPECT_TRUE(automaticCrabEntryAlignmentTimedOut(
      true, std::numeric_limits<double>::quiet_NaN(), 15.0));
}

TEST(MotionGeometry, CampsiteLiveLaneletHandoffIgnoresLongitudinalEntryOffset) {
  auto current = makeMotionPose(12.0, 4.10, 180.0);
  auto live_lanelet = makeMotionPose(12.0, 4.00, 0.0);
  current.header.frame_id = "map";
  live_lanelet.header.frame_id = "map";

  // HH_260825 - The robot may leave a diagonal campsite exit metres away along
  // the lane. Only its 10 cm centerline distance decides route readiness.
  EXPECT_TRUE(campsiteLiveLaneletReturnHandoffEligible(
      current, live_lanelet, 0.1, 2.0, 0.15));
  current.pose.position.x = 25.0;
  live_lanelet.pose.position.x = 25.0;
  EXPECT_TRUE(campsiteLiveLaneletReturnHandoffEligible(
      current, live_lanelet, 0.1, 2.0, 0.15));

  current.pose.position.y = 4.16;
  EXPECT_FALSE(campsiteLiveLaneletReturnHandoffEligible(
      current, live_lanelet, 0.1, 2.0, 0.15));
}

TEST(MotionGeometry, CampsiteLiveLaneletHandoffFailsClosedOnStaleOrWrongFrame) {
  auto current = makeMotionPose(1.0, 2.0, 180.0);
  auto live_lanelet = makeMotionPose(1.0, 2.0, 0.0);
  current.header.frame_id = "map";
  live_lanelet.header.frame_id = "map";

  EXPECT_FALSE(campsiteLiveLaneletReturnHandoffEligible(
      current, live_lanelet, 2.01, 2.0, 0.15));
  live_lanelet.header.frame_id = "odom";
  EXPECT_FALSE(campsiteLiveLaneletReturnHandoffEligible(
      current, live_lanelet, 0.1, 2.0, 0.15));
  live_lanelet.header.frame_id = "map";
  live_lanelet.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(campsiteLiveLaneletReturnHandoffEligible(
      current, live_lanelet, 0.1, 2.0, 0.15));
}

TEST(MotionGeometry,
     CampsiteReturnLatchesLateralSettleLongitudinalWithoutModeChatter) {
  CampsiteCrabReturnSequencer sequencer(
      CampsiteCrabReturnConfig{0.02, 0.10, 1.20});
  sequencer.reset(10.0);

  const auto lateral = sequencer.update(0.40, 0.50, 10.0, 0.50, 4.0);
  EXPECT_EQ(lateral.stage, CampsiteCrabReturnStage::kLateral);
  EXPECT_DOUBLE_EQ(lateral.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(lateral.linear_y_mps, 0.50);

  const auto settle = sequencer.update(0.40, 0.019, 11.0, 0.50, 4.0);
  EXPECT_EQ(settle.stage, CampsiteCrabReturnStage::kSteeringSettle);
  EXPECT_TRUE(settle.stage_changed);
  EXPECT_DOUBLE_EQ(settle.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(settle.linear_y_mps, 0.0);

  // Jitter above the 2 cm transition threshold remains inside the 12 cm
  // latched envelope and cannot request parallel steering again.
  const auto jitter = sequencer.update(0.40, 0.08, 11.8, 0.50, 4.0);
  EXPECT_EQ(jitter.stage, CampsiteCrabReturnStage::kSteeringSettle);
  EXPECT_FALSE(jitter.stage_changed);
  EXPECT_DOUBLE_EQ(jitter.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(jitter.linear_y_mps, 0.0);

  const auto longitudinal =
      sequencer.update(0.40, 0.018, 12.2, 0.50, 4.0);
  EXPECT_EQ(longitudinal.stage, CampsiteCrabReturnStage::kLongitudinal);
  EXPECT_TRUE(longitudinal.stage_changed);
  EXPECT_DOUBLE_EQ(longitudinal.linear_x_mps, 0.50);
  EXPECT_DOUBLE_EQ(longitudinal.linear_y_mps, 0.0);

  const auto still_longitudinal =
      sequencer.update(-0.10, -0.08, 12.3, 0.50, 4.0);
  EXPECT_EQ(still_longitudinal.stage,
            CampsiteCrabReturnStage::kLongitudinal);
  EXPECT_DOUBLE_EQ(still_longitudinal.linear_x_mps, -0.40);
  EXPECT_DOUBLE_EQ(still_longitudinal.linear_y_mps, 0.0);

  // A real displacement beyond the hysteresis stops instead of initiating an
  // unbounded longitudinal/parallel steering oscillation.
  const auto latch_fault =
      sequencer.update(0.10, 0.121, 12.4, 0.50, 4.0);
  EXPECT_TRUE(latch_fault.lateral_latch_exceeded);
  EXPECT_EQ(latch_fault.stage, CampsiteCrabReturnStage::kLongitudinal);
  EXPECT_DOUBLE_EQ(latch_fault.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(latch_fault.linear_y_mps, 0.0);
}

TEST(MotionGeometry, CampsiteReturnCannotCompleteBeforeSteeringSettles) {
  constexpr double radial_error_m = 0.03;
  constexpr double tolerance_m = 0.04;
  EXPECT_FALSE(campsiteCrabReturnMayComplete(
      CampsiteCrabReturnStage::kLateral, radial_error_m, tolerance_m));
  EXPECT_FALSE(campsiteCrabReturnMayComplete(
      CampsiteCrabReturnStage::kSteeringSettle, radial_error_m, tolerance_m));
  EXPECT_TRUE(campsiteCrabReturnMayComplete(
      CampsiteCrabReturnStage::kLongitudinal, radial_error_m, tolerance_m));
  EXPECT_FALSE(campsiteCrabReturnMayComplete(
      CampsiteCrabReturnStage::kLongitudinal,
      std::numeric_limits<double>::quiet_NaN(), tolerance_m));
}

TEST(MotionGeometry, CampsiteReturnRejectsNonFiniteInputsWithoutAdvancingStage) {
  CampsiteCrabReturnSequencer sequencer(
      CampsiteCrabReturnConfig{0.02, 0.10, 1.20});
  sequencer.reset(0.0);

  const auto invalid = sequencer.update(
      std::numeric_limits<double>::quiet_NaN(), 0.30, 0.0, 0.50, 4.0);
  EXPECT_TRUE(invalid.invalid_input);
  EXPECT_EQ(invalid.stage, CampsiteCrabReturnStage::kLateral);
  EXPECT_EQ(sequencer.stage(), CampsiteCrabReturnStage::kLateral);
  EXPECT_DOUBLE_EQ(invalid.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(invalid.linear_y_mps, 0.0);

  const auto invalid_time = sequencer.update(
      0.10, 0.30, std::numeric_limits<double>::infinity(), 0.50, 4.0);
  EXPECT_TRUE(invalid_time.invalid_input);
  EXPECT_EQ(sequencer.stage(), CampsiteCrabReturnStage::kLateral);
}

TEST(MotionGeometry, CampsiteWheelSettleCoversDeployedNinetyDegreeTransition) {
  constexpr double deployed_transition_rate_radps = 1.5;
  constexpr double deployed_ready_error_rad = 0.05;
  constexpr double controller_settle_s = 1.20;
  const double worst_case_transition_s =
      (M_PI_2 - deployed_ready_error_rad) / deployed_transition_rate_radps;

  EXPECT_NEAR(worst_case_transition_s, 1.013864, 1.0e-6);
  EXPECT_GE(controller_settle_s, worst_case_transition_s);
}

TEST(MotionGeometry,
     RoadsideCrabExitCanReachFourCentimeterRadialToleranceAfterLatch) {
  CampsiteCrabReturnSequencer sequencer(
      CampsiteCrabReturnConfig{0.02, 0.10, 1.20});
  sequencer.reset(0.0);

  // B11-B13 cap lateral travel and raw speed at 0.30 m and 0.20 m/s.
  const auto lateral = sequencer.update(0.05, -0.30, 0.0, 0.20, 4.0);
  EXPECT_DOUBLE_EQ(lateral.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(lateral.linear_y_mps, -0.20);
  const auto settle = sequencer.update(0.05, -0.019, 2.0, 0.20, 4.0);
  EXPECT_EQ(settle.stage, CampsiteCrabReturnStage::kSteeringSettle);
  const auto longitudinal =
      sequencer.update(0.05, -0.019, 3.2, 0.20, 4.0);
  EXPECT_EQ(longitudinal.stage, CampsiteCrabReturnStage::kLongitudinal);
  EXPECT_DOUBLE_EQ(longitudinal.linear_x_mps, 0.20);
  EXPECT_DOUBLE_EQ(longitudinal.linear_y_mps, 0.0);

  // Once longitudinal error reaches 3 cm, the latched 1.9 cm lateral residual
  // is still inside the deployed 4 cm radial handoff tolerance.
  EXPECT_LT(std::hypot(0.030, 0.019), 0.04);
}

avg_msgs::msg::AvgOccupancyGrid makeGrid(
    const std::vector<std::tuple<double, double, int>> &occupied_cells = {},
    const double resolution = 0.1, const double origin_yaw = 0.0) {
  avg_msgs::msg::AvgOccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = resolution;
  // HH_260806 - Preserve the fixture's -4..+4 m world extent when a test uses
  // the runtime 0.05 m safety-grid resolution.
  grid.info.width = static_cast<std::uint32_t>(std::ceil(8.0 / resolution));
  grid.info.height = static_cast<std::uint32_t>(std::ceil(8.0 / resolution));
  grid.info.origin.position.x = -4.0;
  grid.info.origin.position.y = -4.0;
  grid.info.origin.orientation.z = std::sin(origin_yaw * 0.5);
  grid.info.origin.orientation.w = std::cos(origin_yaw * 0.5);
  grid.data.assign(grid.info.width * grid.info.height, 0);
  for (const auto &cell : occupied_cells) {
    const double world_x = std::get<0>(cell);
    const double world_y = std::get<1>(cell);
    // HH_260721 - Test fixtures use axis-aligned grids unless origin rotation
    // is tested explicitly.
    const int x = static_cast<int>(
        std::floor((world_x - grid.info.origin.position.x) / resolution));
    const int y = static_cast<int>(
        std::floor((world_y - grid.info.origin.position.y) / resolution));
    if (x >= 0 && y >= 0 && x < static_cast<int>(grid.info.width) &&
        y < static_cast<int>(grid.info.height)) {
      grid.data[y * static_cast<int>(grid.info.width) + x] = std::get<2>(cell);
    }
  }
  return grid;
}

avg_msgs::msg::AvgOccupancyGrid
makeRadarDiskCostGrid(const double center_x, const double center_y,
                      const double radius_m, const int cost = 90,
                      const double resolution = 0.1) {
  auto grid = makeGrid({}, resolution);
  const int center_grid_x = static_cast<int>(
      std::floor((center_x - grid.info.origin.position.x) / resolution));
  const int center_grid_y = static_cast<int>(
      std::floor((center_y - grid.info.origin.position.y) / resolution));
  const int radius_cells = static_cast<int>(std::ceil(radius_m / resolution));
  // HH_260728 - Match radar_cost_grid_node::markDisk exactly: the live node
  // rasterizes a cell-radius circle around the cell containing the raw return.
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      if (dx * dx + dy * dy > radius_cells * radius_cells) {
        continue;
      }
      const int x = center_grid_x + dx;
      const int y = center_grid_y + dy;
      if (x >= 0 && y >= 0 && x < static_cast<int>(grid.info.width) &&
          y < static_cast<int>(grid.info.height)) {
        grid.data[y * static_cast<int>(grid.info.width) + x] = cost;
      }
    }
  }
  return grid;
}

void setGridStamp(avg_msgs::msg::AvgOccupancyGrid &grid,
                  const double stamp_sec) {
  const double integral = std::floor(stamp_sec);
  grid.header.stamp.sec = static_cast<std::int32_t>(integral);
  grid.header.stamp.nanosec =
      static_cast<std::uint32_t>(std::llround((stamp_sec - integral) * 1.0e9));
}

avg_msgs::msg::AvgPath
makePath(const std::vector<std::pair<double, double>> &points) {
  avg_msgs::msg::AvgPath path;
  path.header.frame_id = "map";
  for (const auto &point : points) {
    avg_msgs::msg::AvgPoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    path.poses.push_back(pose);
  }
  return path;
}

MotionCostStopConfig baseCostConfig() {
  MotionCostStopConfig config;
  config.stale_stop_enabled = false;
  config.lanelet_enabled = false;
  config.lanelet_body_hard_stop_enabled = false;
  config.lanelet_footprint_enabled = false;
  config.require_dynamic_source = false;
  config.latch_enabled = false;
  config.use_speed_dependent_lookahead = false;
  config.fixed_front_lookahead_m = 2.0;
  config.front_width_m = 1.0;
  config.side_width_m = 0.6;
  config.rear_width_m = 0.9;
  config.side_lookahead_m = 1.2;
  config.rear_lookahead_m = 0.8;
  config.body_near_enabled = false;
  config.dynamic_front_use_local_path = false;
  // Most legacy fixtures isolate rectangular raster behavior. Dedicated tests
  // below enable and verify the production tapered/rounded geometry explicitly.
  config.tapered_rounded_boundary_enabled = false;
  return config;
}

MotionCostStop
makeMotionCostStop(MotionCostStopConfig config = baseCostConfig()) {
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test"});
  return cost_stop;
}

avg_msgs::msg::AvgTwist command(const double x, const double y = 0.0,
                                const double yaw = 0.0) {
  avg_msgs::msg::AvgTwist output;
  output.linear.x = x;
  output.linear.y = y;
  output.angular.z = yaw;
  return output;
}

} // namespace

TEST(RobotBoundaryGeometry,
     TaperedRoundedBodyAndOffsetPreserveMeasuredExtents) {
  const camrod::RobotBoundaryShape body{
      {0.70837, 0.68323, 0.53505, 0.53495}, 0.12, 0.12, 0.05, 4};
  const auto physical = camrod::makeRobotBoundary(body);
  const auto planning =
      camrod::makeExpandedRobotBoundary(body, {0.10, 0.10, 0.10, 0.10});

  ASSERT_EQ(physical.size(), 30U);
  ASSERT_EQ(planning.size(), 30U);
  const auto extrema = [](const auto &polygon) {
    double max_x = -std::numeric_limits<double>::infinity();
    double min_x = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    for (const auto &point : polygon) {
      max_x = std::max(max_x, point.x);
      min_x = std::min(min_x, point.x);
      max_y = std::max(max_y, point.y);
      min_y = std::min(min_y, point.y);
    }
    return std::array<double, 4>{max_x, min_x, max_y, min_y};
  };
  const auto physical_extrema = extrema(physical);
  const auto planning_extrema = extrema(planning);
  EXPECT_NEAR(physical_extrema[0], 0.70837, 1.0e-9);
  EXPECT_NEAR(physical_extrema[1], -0.68323, 1.0e-9);
  EXPECT_NEAR(physical_extrema[2], 0.53505, 1.0e-9);
  EXPECT_NEAR(physical_extrema[3], -0.53495, 1.0e-9);
  EXPECT_NEAR(planning_extrema[0], 0.80837, 1.0e-9);
  EXPECT_NEAR(planning_extrema[1], -0.78323, 1.0e-9);
  EXPECT_NEAR(planning_extrema[2], 0.63505, 1.0e-9);
  EXPECT_NEAR(planning_extrema[3], -0.63495, 1.0e-9);

  // HH_260809 - The old rectangular front-left corner is intentionally absent;
  // the short front face transitions through a rounded tapered shoulder.
  EXPECT_LT(physical.front().x, body.extents.front);
  EXPECT_LT(physical.front().y, body.extents.left);
}

TEST(MotionCostStop,
     TaperedFrontExcludesOldRectangleCornerButKeepsBodyHardStop) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_body_hard_stop_enabled = true;
  config.lanelet_body_hard_stop_threshold = 100;
  config.lanelet_footprint_enabled = false;
  config.tapered_rounded_boundary_enabled = true;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  cost_stop.setMergedGrid(makeGrid(), 1.0);

  // This cell was inside the old front-left rectangle but lies outside the
  // fabricated tapered shoulder, so it must no longer cause a body hard stop.
  cost_stop.setLaneletGrid(makeGrid({{0.675, 0.500, 100}}, 0.01), 1.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.0, 0.1), 1.0).blocked);

  cost_stop.setLaneletGrid(makeGrid({{0.600, 0.480, 100}}, 0.01), 1.1);
  const auto inside_body = cost_stop.evaluate(command(0.0, 0.0, 0.1), 1.1);
  EXPECT_TRUE(inside_body.blocked);
  EXPECT_EQ(inside_body.reason, "lanelet_physical_body_cost");
}

TEST(MotionCostStop, RoundedPlanningFallbackUsesTheSameTaperedOffset) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_body_hard_stop_enabled = false;
  config.lanelet_footprint_enabled = true;
  config.lanelet_footprint_threshold = 100;
  config.tapered_rounded_boundary_enabled = true;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  cost_stop.setMergedGrid(makeGrid(), 1.0);

  cost_stop.setLaneletGrid(makeGrid({{0.780, 0.600, 100}}, 0.01), 1.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.0, 0.1), 1.0).blocked);

  cost_stop.setLaneletGrid(makeGrid({{0.700, 0.550, 100}}, 0.01), 1.1);
  const auto inside_planning = cost_stop.evaluate(command(0.0, 0.0, 0.1), 1.1);
  EXPECT_TRUE(inside_planning.blocked);
  EXPECT_EQ(inside_planning.reason, "lanelet_footprint_cost");
}

TEST(CmdVelGatePolicy, RequiresEngageOperatorArmAndHealthyCan) {
  CmdVelGatePolicy policy;
  PlatformSafetyState platform;
  platform.received = true;
  platform.received_sec = 10.0;
  platform.control_mode = 1;
  policy.setPlatformState(platform);

  EXPECT_FALSE(policy.enabled(10.0, false, false));
  policy.setMissionEngage(true);
  EXPECT_FALSE(policy.enabled(10.0, false, false));
  policy.setPlatformDriveEnable(true);
  EXPECT_TRUE(policy.enabled(10.0, false, false));
  policy.setEstopSource("planning", true);
  EXPECT_FALSE(policy.enabled(10.0, false, false));
  policy.setEstopSource("planning", false);
  EXPECT_TRUE(policy.enabled(10.0, false, false));
}

TEST(CmdVelGatePolicy, BlocksCanFaultStaleStatusChargingAndCriticalSoc) {
  CmdVelGatePolicyConfig config;
  config.require_platform_drive_enable = false;
  CmdVelGatePolicy policy(config);
  policy.setManualEngage(true);
  PlatformSafetyState platform;
  platform.received = true;
  platform.received_sec = 20.0;
  platform.control_mode = 1;
  policy.setPlatformState(platform);
  EXPECT_TRUE(policy.enabled(20.0, false, false));
  EXPECT_FALSE(policy.enabled(20.6, false, false));

  platform.received_sec = 21.0;
  platform.error_code = 0x0100;
  policy.setPlatformState(platform);
  EXPECT_FALSE(policy.enabled(21.0, false, false));
  EXPECT_NE(
      CmdVelGatePolicy::formatPlatformErrorCode(0x0100).find("motor_driver"),
      std::string::npos);

  platform.error_code = 0;
  platform.battery_percentage = 0.19;
  policy.setPlatformState(platform);
  EXPECT_FALSE(policy.enabled(21.0, false, false));
  platform.battery_percentage = 0.20;
  policy.setPlatformState(platform);
  EXPECT_FALSE(policy.enabled(21.0, false, false));
  platform.battery_percentage = 0.21;
  policy.setPlatformState(platform);
  EXPECT_FALSE(policy.enabled(21.0, true, false));
  EXPECT_TRUE(policy.enabled(21.0, true, true));

  // A charging-departure authorization removes only the charging reason.
  // Every independent safety reason remains authoritative.
  policy.setEstopSource("test", true);
  EXPECT_FALSE(policy.enabled(21.0, true, true));
  policy.setEstopSource("test", false);
  platform.error_code = 0x0100;
  policy.setPlatformState(platform);
  EXPECT_FALSE(policy.enabled(21.0, true, true));
  platform.error_code = 0;
  platform.battery_percentage = 0.19;
  policy.setPlatformState(platform);
  EXPECT_FALSE(policy.enabled(21.0, true, true));
  platform.battery_percentage = 0.21;
  platform.received_sec = 21.0;
  policy.setPlatformState(platform);
  EXPECT_FALSE(policy.enabled(21.6, true, true));
  platform.received_sec = 22.0;
  policy.setPlatformState(platform);
  policy.setCostState(true, 23.0);
  EXPECT_FALSE(policy.enabled(22.0, true, true));
}

TEST(CmdVelGatePolicy, SeparatesNormalStandbyFromWarningAndErrorHolds) {
  // HH_260721 - Health severity must describe faults, not whether motion is
  // currently authorized.
  EXPECT_EQ(CmdVelGatePolicy::classifyHealth(
                {"engage=false(manual=false,mission=false)",
                 "platform_drive_enable=false"}),
            CmdVelGateHealth::kOk);
  EXPECT_EQ(CmdVelGatePolicy::classifyHealth({"charging"}),
            CmdVelGateHealth::kOk);
  EXPECT_EQ(CmdVelGatePolicy::classifyHealth({"cost_stop_latched"}),
            CmdVelGateHealth::kWarning);
  EXPECT_EQ(CmdVelGatePolicy::classifyHealth({"platform_status_stale=1.20s"}),
            CmdVelGateHealth::kError);
  EXPECT_EQ(
      CmdVelGatePolicy::classifyHealth({"platform_error=0x0100(motor_driver)"}),
      CmdVelGateHealth::kError);
}

TEST(ChargingMissionOverride, AcceptsOnlyFreshCampsiteRequestDuringCharging) {
  ChargingMissionOverride policy;
  policy.setCharging(true);
  MissionRequestIdentity drop_zone{"drop_zone", "ui", 1, 0};
  MissionRequestIdentity campsite{"camping_site_3", "ui", 2, 0};
  EXPECT_FALSE(policy.activateForMission(drop_zone, 100.0));
  EXPECT_FALSE(policy.activateForMission(campsite, 100.0));
  policy.setBatteryPercentage(0.34);
  EXPECT_FALSE(policy.activateForMission(campsite, 100.0));
  policy.setBatteryPercentage(0.35);
  EXPECT_TRUE(policy.activateForMission(campsite, 100.0));
  EXPECT_TRUE(policy.isActive(114.9));
  EXPECT_FALSE(policy.activateForMission(campsite, 100.5));
  EXPECT_FALSE(policy.isActive(115.1));
  policy.setCharging(false);
  EXPECT_FALSE(policy.isActive(101.0));
}

TEST(DropZoneChargingDepartureAuthorization, AcceptsOnlyExactHealthyExitPhases) {
  DropZoneChargingDepartureAuthorization authorization;

  EXPECT_FALSE(authorization.observe(
      "control", "exit_straight", true, 100.0,
      std::optional<double>{100.0}));
  EXPECT_FALSE(authorization.isActive(100.0));
  EXPECT_FALSE(authorization.observe(
      "control", "PARKED", true, 100.0,
      std::optional<double>{100.0}));
  EXPECT_FALSE(authorization.observe(
      "drop_zone_maneuver_controller", "EXIT_STRAIGHT", true, 100.0,
      std::optional<double>{100.0}));
  EXPECT_TRUE(authorization.observe(
      "control", "EXIT_STRAIGHT", true, 100.0,
      std::optional<double>{100.0}));
  EXPECT_TRUE(authorization.isActive(100.5));
  EXPECT_TRUE(authorization.observe(
      "control", "ALIGN_EXIT_YAW", true, 101.0,
      std::optional<double>{101.0}));
  EXPECT_TRUE(authorization.isActive(101.5));

  // Any unhealthy or unrelated heartbeat immediately revokes prior evidence.
  EXPECT_FALSE(authorization.observe(
      "control", "ALIGN_EXIT_YAW", false, 101.6,
      std::optional<double>{101.6}));
  EXPECT_FALSE(authorization.isActive(101.6));
  EXPECT_TRUE(authorization.phase().empty());
}

TEST(DropZoneChargingDepartureAuthorization, HeartbeatAndSourceStampExpire) {
  DropZoneChargingDepartureAuthorization authorization;
  ASSERT_TRUE(authorization.observe(
      "control", "EXIT_STRAIGHT", true, 10.0,
      std::optional<double>{9.9}));
  EXPECT_TRUE(authorization.isActive(11.89));
  EXPECT_FALSE(authorization.isActive(12.01));

  // A newly received replayed or excessively future-dated status is invalid.
  EXPECT_FALSE(authorization.observe(
      "control", "EXIT_STRAIGHT", true, 20.0,
      std::optional<double>{17.9}));
  EXPECT_FALSE(authorization.observe(
      "control", "EXIT_STRAIGHT", true, 20.0,
      std::optional<double>{20.26}));
  EXPECT_TRUE(authorization.phase().empty());

  // Production rejects unstamped evidence.  Simulation can opt into explicit
  // receipt-only freshness without weakening the deployed default.
  EXPECT_FALSE(authorization.observe(
      "control", "EXIT_STRAIGHT", true, 30.0));
  auto simulation_config = authorization.config();
  simulation_config.require_source_stamp = false;
  authorization.setConfig(simulation_config);
  EXPECT_TRUE(authorization.observe(
      "control", "EXIT_STRAIGHT", true, 30.0));
  EXPECT_TRUE(authorization.isActive(31.9));
  EXPECT_FALSE(authorization.isActive(32.1));
}

TEST(DropZoneChargingDepartureAuthorization, InvalidConfigurationFailsClosed) {
  DropZoneChargingDepartureAuthorizationConfig config;
  config.heartbeat_timeout_s = 0.0;
  EXPECT_FALSE(dropZoneChargingDepartureAuthorizationConfigIsValid(config));
  DropZoneChargingDepartureAuthorization authorization(config);
  EXPECT_FALSE(authorization.observe(
      "control", "EXIT_STRAIGHT", true, 1.0,
      std::optional<double>{1.0}));
  EXPECT_FALSE(authorization.isActive(1.0));
}

TEST(MotionCostStop, ForwardThresholdAndBelowThreshold) {
  auto cost_stop = makeMotionCostStop();
  cost_stop.setMergedGrid(makeGrid({{1.0, 0.0, 90}}), 0.0);
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 0.0).blocked);

  cost_stop = makeMotionCostStop();
  cost_stop.setMergedGrid(makeGrid({{1.0, 0.0, 60}}), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.0).blocked);
}

TEST(MotionCostStop, UnknownDynamicCellsNeverBecomeObstacleCost) {
  auto config = baseCostConfig();
  config.require_dynamic_source = true;
  auto cost_stop = makeMotionCostStop(config);
  const auto unknown_dynamic = makeGrid({{1.0, 0.0, -1}});
  cost_stop.setMergedGrid(unknown_dynamic, 0.0);
  cost_stop.setSourceGrid("radar", unknown_dynamic, 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.0).blocked);

  // HH_260904 - Unknown is fail-open only for obstacle sources. The separate
  // lanelet geometry contract intentionally remains fail-closed.
  config.lanelet_enabled = true;
  config.lanelet_body_hard_stop_enabled = true;
  config.lanelet_stop_on_unknown = true;
  config.lanelet_current_allow_route_reentry = false;
  cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  cost_stop.setLaneletGrid(makeGrid(), 0.0);
  cost_stop.setPose(PlanarPose{5.0, 0.0, 0.0, "map", "test"});
  const auto unknown_lanelet = cost_stop.evaluate(command(0.2), 0.0);
  EXPECT_TRUE(unknown_lanelet.lanelet_violation) << unknown_lanelet.reason;
}

TEST(MotionCostStop, CrabAndReverseUseTravelDirection) {
  auto cost_stop = makeMotionCostStop();
  cost_stop.setMergedGrid(makeGrid({{1.0, 0.0, 90}}), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.2), 0.0).blocked);
  const auto left_obstacle = makeGrid({{0.0, 0.5, 90}});
  cost_stop.setMergedGrid(left_obstacle, 0.1);
  // HH_260721 - Site-motion static bypass still requires live source cost to
  // stop.
  cost_stop.setSourceGrid("radar", left_obstacle, 0.1);
  EXPECT_TRUE(cost_stop.evaluate(command(0.0, 0.2), 0.1).blocked);

  cost_stop = makeMotionCostStop();
  cost_stop.setMergedGrid(makeGrid({{1.0, 0.0, 90}}), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(-0.2), 0.0).blocked);
  const auto rear_obstacle = makeGrid({{-0.35, 0.0, 90}});
  cost_stop.setMergedGrid(rear_obstacle, 0.1);
  cost_stop.setSourceGrid("radar", rear_obstacle, 0.1);
  EXPECT_TRUE(cost_stop.evaluate(command(-0.2), 0.1).blocked);
}

TEST(RouteSafetyRecovery, PreservesTriggerDirectionUntilContinuousClear) {
  RouteSafetyRecoveryConfig config;
  config.clear_required_s = 1.0;
  RouteSafetyRecovery recovery(config);
  const MotionCostStopDecision violation{true, false, true, false,
                                         "lanelet_footprint_cost"};

  EXPECT_TRUE(recovery.observeViolation(violation, command(0.3), 10.0));
  EXPECT_TRUE(recovery.active());
  EXPECT_DOUBLE_EQ(recovery.triggerCommand().linear.x, 0.3);

  // HH_260729 / TODOLIST 11-12 - Later zero or lateral commands cannot replace
  // the saved forward trigger that defines route-clear evidence.
  EXPECT_FALSE(recovery.observeViolation(violation, command(0.0, 0.2), 10.1));
  EXPECT_DOUBLE_EQ(recovery.triggerCommand().linear.x, 0.3);
  EXPECT_DOUBLE_EQ(recovery.triggerCommand().linear.y, 0.0);

  EXPECT_FALSE(recovery.updateProbe(MotionCostStopDecision{}, 11.0));
  EXPECT_TRUE(recovery.active());
  EXPECT_FALSE(recovery.recoveryMotionObserved());
  recovery.observeRecoveryMotion();
  EXPECT_TRUE(recovery.recoveryMotionObserved());
  EXPECT_FALSE(recovery.updateProbe(violation, 11.5));
  EXPECT_FALSE(recovery.updateProbe(MotionCostStopDecision{}, 12.0));
  EXPECT_TRUE(recovery.updateProbe(MotionCostStopDecision{}, 13.0));
  EXPECT_FALSE(recovery.active());
}

TEST(RouteSafetyRecovery, AdmitsOppositeAndOrthogonalProjectedCandidates) {
  RouteSafetyRecovery recovery;
  const MotionCostStopDecision violation{true, false, true, false,
                                         "lanelet_front_cost"};
  ASSERT_TRUE(recovery.observeViolation(violation, command(0.3), 1.0));

  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(-0.1)));
  EXPECT_FALSE(recovery.permitsProjectedRecoveryCandidate(command(0.1)));
  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(0.0, 0.1)));
  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(0.0, -0.1)));
  EXPECT_FALSE(
      recovery.permitsProjectedRecoveryCandidate(command(0.0, 0.0, 0.2)));
}

TEST(RouteSafetyRecovery,
     LatchesRapidRecontactAfterConfiguredAutomaticRelease) {
  RouteSafetyRecoveryConfig config;
  config.clear_required_s = 0.1;
  config.max_automatic_releases = 1;
  config.rapid_recontact_window_s = 5.0;
  RouteSafetyRecovery recovery(config);
  const MotionCostStopDecision violation{true, false, true, false,
                                         "lanelet_footprint_cost"};
  const MotionCostStopDecision clear{};

  ASSERT_TRUE(recovery.observeViolation(violation, command(0.3), 1.0));
  recovery.observeRecoveryMotion();
  EXPECT_FALSE(recovery.updateProbe(clear, 1.0));
  EXPECT_TRUE(recovery.updateProbe(clear, 1.2));
  EXPECT_FALSE(recovery.active());
  EXPECT_EQ(recovery.automaticReleasesInWindow(), 1);

  ASSERT_TRUE(recovery.observeViolation(violation, command(0.3), 2.0));
  EXPECT_TRUE(recovery.automaticReleaseBlocked());
  // HH_260807 - Retry containment blocks same-direction Nav2 release, not a
  // candidate that still has to pass the gate's projected inward-escape proof.
  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(0.0, 0.1)));
  EXPECT_FALSE(recovery.updateProbe(clear, 2.0));
  EXPECT_FALSE(recovery.updateProbe(clear, 3.0));
  EXPECT_FALSE(recovery.latestDecision().blocked);
  EXPECT_TRUE(recovery.active());
  // The clear robot stays stopped for replan instead of resuming the same Nav2
  // direction, while a later renewed contact could still request inward escape.
  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(0.0, -0.1)));

  recovery.reset();
  EXPECT_FALSE(recovery.automaticReleaseBlocked());
  EXPECT_EQ(recovery.automaticReleasesInWindow(), 0);
}

TEST(RouteSafetyRecovery, AllowsAutomaticRecoveryAgainAfterRecontactWindow) {
  RouteSafetyRecoveryConfig config;
  config.clear_required_s = 0.1;
  config.max_automatic_releases = 1;
  config.rapid_recontact_window_s = 5.0;
  RouteSafetyRecovery recovery(config);
  const MotionCostStopDecision violation{true, false, true, false,
                                         "lanelet_footprint_cost"};
  const MotionCostStopDecision clear{};

  ASSERT_TRUE(recovery.observeViolation(violation, command(0.3), 1.0));
  recovery.observeRecoveryMotion();
  EXPECT_FALSE(recovery.updateProbe(clear, 1.0));
  EXPECT_TRUE(recovery.updateProbe(clear, 1.2));

  ASSERT_TRUE(recovery.observeViolation(violation, command(0.3), 7.0));
  EXPECT_FALSE(recovery.automaticReleaseBlocked());
  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(0.0, 0.1)));
}

TEST(RouteSafetyRecovery, RotationViolationAdmitsTranslationForProjectedCheck) {
  RouteSafetyRecovery recovery;
  const MotionCostStopDecision violation{true, false, true, false,
                                         "lanelet_footprint_cost"};
  ASSERT_TRUE(
      recovery.observeViolation(violation, command(0.0, 0.0, 0.2), 1.0));

  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(0.0, 0.1)));
  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(-0.1)));
  EXPECT_FALSE(
      recovery.permitsProjectedRecoveryCandidate(command(0.0, 0.0, -0.2)));
}

TEST(RouteSafetyRecovery, ResetsRegionalBudgetOnlyAfterSignedForwardProgress) {
  RouteSafetyRecoveryConfig config;
  config.clear_required_s = 0.1;
  config.max_automatic_releases = 2;
  config.rapid_recontact_window_s = 0.5;
  config.progress_reset_distance_m = 0.75;
  RouteSafetyRecovery recovery(config);
  const MotionCostStopDecision clear{};
  const auto violation_at = [](const double x) {
    MotionCostStopDecision decision{true, false, true, false,
                                    "lanelet_footprint_cost"};
    decision.lanelet_contact_valid = true;
    decision.lanelet_pose_x = x;
    decision.lanelet_pose_y = 2.0;
    decision.lanelet_pose_yaw = 0.0;
    return decision;
  };

  ASSERT_TRUE(recovery.observeViolation(violation_at(0.0), command(0.3), 1.0));
  recovery.observeRecoveryMotion();
  EXPECT_FALSE(recovery.updateProbe(clear, 1.0));
  EXPECT_TRUE(recovery.updateProbe(clear, 1.2));
  EXPECT_EQ(recovery.automaticReleasesInWindow(), 1);

  // HH_260810 - Elapsed time alone must not erase a pose-aware contact budget.
  ASSERT_TRUE(recovery.observeViolation(violation_at(0.2), command(0.3), 10.0));
  EXPECT_EQ(recovery.automaticReleasesInWindow(), 1);
  recovery.observeRecoveryMotion();
  EXPECT_FALSE(recovery.updateProbe(clear, 10.0));
  EXPECT_TRUE(recovery.updateProbe(clear, 10.2));
  EXPECT_EQ(recovery.automaticReleasesInWindow(), 2);

  ASSERT_TRUE(recovery.observeViolation(violation_at(0.4), command(0.3), 20.0));
  EXPECT_TRUE(recovery.automaticReleaseBlocked());

  // Passing the original contact in the saved forward direction starts a new
  // regional episode even while this hold is still active.
  EXPECT_FALSE(
      recovery.observeViolation(violation_at(0.8), command(0.3), 20.1));
  EXPECT_TRUE(recovery.releaseBudgetResetByProgress());
  EXPECT_NEAR(recovery.lastProgressResetDistanceM(), 0.8, 1.0e-9);
  EXPECT_EQ(recovery.automaticReleasesInWindow(), 0);
  EXPECT_FALSE(recovery.automaticReleaseBlocked());
}

TEST(RouteRecoveryCandidate, SelectsUniqueClearCrabSide) {
  const MotionCostStopDecision clear{};
  const MotionCostStopDecision blocked{
      true, false, true, false,
      "route_recovery_predicted_lanelet_footprint_cost"};
  const auto selected = continueAdaptiveRouteRecoveryCandidate(
      command(0.3), RouteRecoveryCandidateKind::kNone, 0.10, 0.10,
      {clear, blocked, clear, blocked, blocked});

  ASSERT_TRUE(selected.available());
  EXPECT_EQ(selected.kind, RouteRecoveryCandidateKind::kCrabLeft);
  EXPECT_NEAR(selected.command.linear.x, 0.0, 1.0e-9);
  EXPECT_NEAR(selected.command.linear.y, 0.10, 1.0e-9);
  EXPECT_NEAR(selected.command.angular.z, 0.0, 1.0e-9);
}

TEST(RouteRecoveryCandidate, FallsBackToReverseWhenBothCrabSidesAreBlocked) {
  const MotionCostStopDecision clear{};
  const MotionCostStopDecision blocked{
      true, false, true, false,
      "route_recovery_predicted_lanelet_footprint_cost"};
  const auto selected = continueAdaptiveRouteRecoveryCandidate(
      command(0.3), RouteRecoveryCandidateKind::kNone, 0.10, 0.10,
      {blocked, blocked, clear, blocked, blocked});

  ASSERT_TRUE(selected.available());
  EXPECT_EQ(selected.kind, RouteRecoveryCandidateKind::kReverse);
  EXPECT_NEAR(selected.command.linear.x, -0.10, 1.0e-9);
  EXPECT_NEAR(selected.command.linear.y, 0.0, 1.0e-9);
}

TEST(RouteRecoveryCandidate, RotationOnlyTriggerStillProducesProjectedEscape) {
  const MotionCostStopDecision clear{};
  const MotionCostStopDecision blocked{
      true, false, true, false,
      "route_recovery_predicted_lanelet_footprint_cost"};
  const auto selected = continueAdaptiveRouteRecoveryCandidate(
      command(0.0, 0.0, 0.2), RouteRecoveryCandidateKind::kNone, 0.10, 0.10,
      {blocked, blocked, clear, blocked, blocked});

  ASSERT_TRUE(selected.available());
  EXPECT_EQ(selected.kind, RouteRecoveryCandidateKind::kReverse);
  EXPECT_NEAR(selected.command.linear.x, -0.10, 1.0e-9);
  EXPECT_NEAR(selected.command.linear.y, 0.0, 1.0e-9);
  EXPECT_NEAR(selected.command.angular.z, 0.0, 1.0e-9);
}

TEST(RouteRecoveryCandidate, StopsWhenNoProjectedStageIsClear) {
  const MotionCostStopDecision blocked{
      true, false, true, false,
      "route_recovery_predicted_lanelet_footprint_cost"};

  const auto none = continueAdaptiveRouteRecoveryCandidate(
      command(0.3), RouteRecoveryCandidateKind::kNone, 0.10, 0.10,
      {blocked, blocked, blocked, blocked, blocked});
  EXPECT_FALSE(none.available());
  EXPECT_EQ(none.reason, "no_projected_candidate_clear");
}

TEST(RouteRecoveryCandidate, SwitchesFromReverseToUniqueClearCrab) {
  const MotionCostStopDecision clear{};
  const MotionCostStopDecision blocked{
      true, false, true, false,
      "route_recovery_predicted_lanelet_footprint_cost"};
  const auto selected = continueAdaptiveRouteRecoveryCandidate(
      command(0.3), RouteRecoveryCandidateKind::kReverse, 0.10, 0.10,
      {blocked, clear, clear, blocked, blocked});

  ASSERT_TRUE(selected.available());
  EXPECT_EQ(selected.kind, RouteRecoveryCandidateKind::kCrabRight);
  EXPECT_NEAR(selected.command.linear.y, -0.10, 1.0e-9);
  EXPECT_EQ(selected.reason, "stage_switch_to_unique_lateral");
}

TEST(RouteRecoveryCandidate, SwitchesFromReverseToUniqueProjectedYaw) {
  const MotionCostStopDecision clear{};
  const MotionCostStopDecision blocked{
      true, false, true, false,
      "route_recovery_predicted_lanelet_footprint_cost"};
  const auto selected = continueAdaptiveRouteRecoveryCandidate(
      command(0.3), RouteRecoveryCandidateKind::kReverse, 0.10, 0.10,
      {blocked, blocked, clear, blocked, clear});

  ASSERT_TRUE(selected.available());
  EXPECT_EQ(selected.kind, RouteRecoveryCandidateKind::kReverseYawRight);
  EXPECT_NEAR(selected.command.linear.x, -0.10, 1.0e-9);
  EXPECT_NEAR(selected.command.angular.z, -0.10, 1.0e-9);
  EXPECT_EQ(selected.reason, "stage_switch_to_unique_reverse_yaw");
}

TEST(RouteRecoveryCandidate, UsesRppTurnSignWhenBothProjectedYawArcsAreClear) {
  const MotionCostStopDecision clear{};
  const MotionCostStopDecision blocked{
      true, false, true, false,
      "route_recovery_predicted_lanelet_footprint_cost"};
  const auto selected = continueAdaptiveRouteRecoveryCandidate(
      command(0.3, 0.0, -0.2), RouteRecoveryCandidateKind::kReverse, 0.10, 0.10,
      {blocked, blocked, clear, clear, clear});

  ASSERT_TRUE(selected.available());
  EXPECT_EQ(selected.kind, RouteRecoveryCandidateKind::kReverseYawRight);
  EXPECT_NEAR(selected.command.linear.x, -0.10, 1.0e-9);
  EXPECT_NEAR(selected.command.angular.z, -0.10, 1.0e-9);
}

TEST(RouteRecoveryCandidate, KeepsStraightReverseWithoutYawPreference) {
  const MotionCostStopDecision clear{};
  const MotionCostStopDecision blocked{
      true, false, true, false,
      "route_recovery_predicted_lanelet_footprint_cost"};
  const auto selected = continueAdaptiveRouteRecoveryCandidate(
      command(0.3), RouteRecoveryCandidateKind::kReverse, 0.10, 0.10,
      {blocked, blocked, clear, clear, clear});

  ASSERT_TRUE(selected.available());
  EXPECT_EQ(selected.kind, RouteRecoveryCandidateKind::kReverse);
  EXPECT_NEAR(selected.command.angular.z, 0.0, 1.0e-9);
}

TEST(RouteRecoveryCandidate, RepositionsWhenActiveCrabBecomesBlocked) {
  const MotionCostStopDecision clear{};
  const MotionCostStopDecision blocked{
      true, false, true, false,
      "route_recovery_predicted_lanelet_footprint_cost"};
  const auto selected = continueAdaptiveRouteRecoveryCandidate(
      command(0.3), RouteRecoveryCandidateKind::kCrabLeft, 0.10, 0.10,
      {blocked, blocked, clear, blocked, blocked});

  ASSERT_TRUE(selected.available());
  EXPECT_EQ(selected.kind, RouteRecoveryCandidateKind::kReverse);
  EXPECT_EQ(selected.reason, "active_stage_blocked_reverse_reposition");
}

TEST(MotionCostStop, RouteRecoveryProbeFailsClosedOnStaleLaneletEvidence) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.stale_timeout_s = 1.0;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 5.0});

  const auto missing =
      cost_stop.evaluateLaneletRecovery(command(0.2), 5.0, 0.5);
  EXPECT_TRUE(missing.blocked);
  EXPECT_TRUE(missing.lanelet_violation);
  EXPECT_TRUE(missing.stale_grid);

  cost_stop.setLaneletGrid(makeGrid(), 5.0);
  EXPECT_FALSE(
      cost_stop.evaluateLaneletRecovery(command(0.2), 5.5, 0.5).blocked);
  const auto stale = cost_stop.evaluateLaneletRecovery(command(0.2), 6.1, 2.0);
  EXPECT_TRUE(stale.blocked);
  EXPECT_EQ(stale.reason, "lanelet_recovery_grid_stale");
}

TEST(MotionCostStop, RouteRecoveryUsesIndependentLaneletGridAge) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.stale_timeout_s = 1.0;
  config.lanelet_recovery_stale_timeout_s = 12.0;
  MotionCostStop cost_stop(config);
  cost_stop.setLaneletGrid(makeGrid(), 1.0);
  cost_stop.setMergedGrid(makeGrid(), 11.0);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 11.0});

  EXPECT_FALSE(
      cost_stop.evaluateLaneletRecovery(command(0.2), 11.0, 0.5).blocked);
  EXPECT_FALSE(
      cost_stop.evaluateRouteRecoveryCommand(command(-0.1), 11.0, 0.25, 0.5)
          .blocked);
  EXPECT_EQ(cost_stop.evaluateLaneletRecovery(command(0.2), 13.1, 3.0).reason,
            "lanelet_recovery_grid_stale");
}

TEST(MotionCostStop, RouteRecoveryBoundsReverseYawRate) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  MotionCostStop cost_stop(config);
  cost_stop.setLaneletGrid(makeGrid(), 1.0);
  cost_stop.setMergedGrid(makeGrid(), 1.0);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});

  EXPECT_FALSE(cost_stop
                   .evaluateRouteRecoveryCommand(command(-0.1, 0.0, 0.10), 1.0,
                                                 0.25, 0.5)
                   .blocked);
  const auto excessive = cost_stop.evaluateRouteRecoveryCommand(
      command(-0.1, 0.0, 0.16), 1.0, 0.25, 0.5);
  EXPECT_TRUE(excessive.blocked);
  EXPECT_EQ(excessive.reason, "route_recovery_angular_rate_exceeded");
}

TEST(MotionCostStop, RouteRecoveryRejectsPhysicalContactAlongYawArc) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_body_hard_stop_enabled = true;
  config.lanelet_body_hard_stop_threshold = 100;
  config.body_front_m = 0.20;
  config.body_rear_m = 0.20;
  config.body_left_m = 0.10;
  config.body_right_m = 0.10;
  config.lanelet_footprint_enabled = false;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  cost_stop.setMergedGrid(makeGrid(), 1.0);
  // HH_260807 - This cell is clear of both endpoint rectangles, but the
  // reverse-left arc sweeps its rear corner through it near half distance.
  cost_stop.setLaneletGrid(makeGrid({{-0.399, 0.051, 100}}, 0.05), 1.0);

  const auto yaw_arc = cost_stop.evaluateRouteRecoveryCommand(
      command(-0.1, 0.0, 0.10), 1.0, 0.30, 0.5);
  EXPECT_TRUE(yaw_arc.blocked);
  EXPECT_EQ(yaw_arc.reason, "route_recovery_swept_physical_body_cost");
}

TEST(MotionCostStop, RouteRecoveryProbeFailsClosedOnStalePoseEvidence) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  MotionCostStop cost_stop(config);
  cost_stop.setLaneletGrid(makeGrid(), 5.0);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 4.0});

  const auto stale_pose =
      cost_stop.evaluateLaneletRecovery(command(0.2), 5.0, 0.5);
  EXPECT_TRUE(stale_pose.blocked);
  EXPECT_TRUE(stale_pose.lanelet_violation);
  EXPECT_TRUE(stale_pose.stale_grid);
  EXPECT_EQ(stale_pose.reason, "lanelet_recovery_pose_stale");
}

TEST(MotionCostStop, OppositeRecoveryRequiresProjectedFullFootprintClear) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_footprint_threshold = 100;
  config.footprint_front_m = 0.2;
  config.footprint_rear_m = 0.2;
  config.footprint_left_m = 0.2;
  config.footprint_right_m = 0.2;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  cost_stop.setMergedGrid(makeGrid(), 1.0);
  cost_stop.setLaneletGrid(makeGrid({{0.05, 0.0, 100}}), 1.0);

  ASSERT_TRUE(cost_stop.evaluate(command(-0.1), 1.0).blocked);
  EXPECT_FALSE(
      cost_stop.evaluateRouteRecoveryCommand(command(-0.1), 1.1, 0.3, 0.5)
          .blocked);
  const auto still_on_boundary =
      cost_stop.evaluateRouteRecoveryCommand(command(-0.1), 1.2, 0.05, 0.5);
  EXPECT_TRUE(still_on_boundary.blocked);
  EXPECT_NE(still_on_boundary.reason.find(
                "route_recovery_predicted_lanelet_footprint"),
            std::string::npos);
}

TEST(MotionCostStop, CrabRecoveryMovesAwayFromSideBoundaryOnly) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_footprint_threshold = 100;
  config.footprint_front_m = 0.2;
  config.footprint_rear_m = 0.2;
  config.footprint_left_m = 0.2;
  config.footprint_right_m = 0.2;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  cost_stop.setMergedGrid(makeGrid(), 1.0);
  cost_stop.setLaneletGrid(makeGrid({{0.0, 0.15, 100}}), 1.0);

  ASSERT_TRUE(cost_stop.evaluate(command(0.1), 1.0).blocked);
  EXPECT_FALSE(
      cost_stop.evaluateRouteRecoveryCommand(command(0.0, -0.1), 1.1, 0.3, 0.5)
          .blocked);
  const auto toward_boundary =
      cost_stop.evaluateRouteRecoveryCommand(command(0.0, 0.1), 1.2, 0.3, 0.5);
  EXPECT_TRUE(toward_boundary.blocked);
  EXPECT_NE(
      toward_boundary.reason.find("route_recovery_predicted_lanelet_footprint"),
      std::string::npos);
}

TEST(MotionCostStop,
     PhysicalBodyContactAllowsOnlyMonotonicProjectedClearEscape) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_body_hard_stop_enabled = true;
  config.lanelet_body_hard_stop_threshold = 100;
  config.body_front_m = 0.2;
  config.body_rear_m = 0.2;
  config.body_left_m = 0.2;
  config.body_right_m = 0.2;
  config.lanelet_footprint_enabled = true;
  config.footprint_front_m = 0.3;
  config.footprint_rear_m = 0.3;
  config.footprint_left_m = 0.3;
  config.footprint_right_m = 0.3;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  cost_stop.setMergedGrid(makeGrid(), 1.0);
  cost_stop.setLaneletGrid(makeGrid({{0.0, 0.15, 100}}), 1.0);

  const auto trigger = cost_stop.evaluate(command(0.1), 1.0);
  ASSERT_TRUE(trigger.blocked);
  EXPECT_EQ(trigger.reason, "lanelet_physical_body_cost");
  EXPECT_TRUE(trigger.lanelet_contact_valid);
  EXPECT_EQ(trigger.lanelet_hit_cost, 100);
  EXPECT_NEAR(trigger.lanelet_hit_body_x, 0.05, 1.0e-6);
  EXPECT_NEAR(trigger.lanelet_hit_body_y, 0.15, 1.0e-6);
  const auto inward =
      cost_stop.evaluateRouteRecoveryCommand(command(0.0, -0.1), 1.1, 0.3, 0.5);
  EXPECT_FALSE(inward.blocked) << inward.reason;

  // A reverse candidate is also valid here because the projection proves that
  // it moves this isolated side contact completely clear without overlap
  // growth.
  const auto reverse_clear =
      cost_stop.evaluateRouteRecoveryCommand(command(-0.1), 1.1, 0.3, 0.5);
  EXPECT_FALSE(reverse_clear.blocked) << reverse_clear.reason;

  const auto outward =
      cost_stop.evaluateRouteRecoveryCommand(command(0.0, 0.1), 1.1, 0.3, 0.5);
  EXPECT_TRUE(outward.blocked);
  EXPECT_NE(outward.reason.find("physical_body"), std::string::npos);
}

TEST(MotionCostStop,
     PhysicalContactEscapeStillRequiresPlanningFootprintEndpointClear) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_body_hard_stop_enabled = true;
  config.lanelet_body_hard_stop_threshold = 100;
  config.body_front_m = 0.2;
  config.body_rear_m = 0.2;
  config.body_left_m = 0.2;
  config.body_right_m = 0.2;
  config.lanelet_footprint_enabled = true;
  config.footprint_front_m = 0.4;
  config.footprint_rear_m = 0.4;
  config.footprint_left_m = 0.3;
  config.footprint_right_m = 0.3;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  cost_stop.setMergedGrid(makeGrid(), 1.0);
  // First cell is current body contact. The second is outside the swept body
  // but inside the endpoint's larger planning footprint after moving right.
  cost_stop.setLaneletGrid(makeGrid({{0.0, 0.15, 100}, {0.35, -0.25, 100}}),
                           1.0);

  const auto decision =
      cost_stop.evaluateRouteRecoveryCommand(command(0.0, -0.1), 1.1, 0.3, 0.5);
  EXPECT_TRUE(decision.blocked);
  EXPECT_NE(decision.reason.find("route_recovery_predicted_lanelet_footprint"),
            std::string::npos)
      << decision.reason;
}

TEST(MotionCostStop, PhysicalContactEscapeKeepsDynamicObstacleAuthoritative) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_body_hard_stop_enabled = true;
  config.lanelet_body_hard_stop_threshold = 100;
  config.body_front_m = 0.2;
  config.body_rear_m = 0.2;
  config.body_left_m = 0.2;
  config.body_right_m = 0.2;
  config.lanelet_footprint_enabled = true;
  config.footprint_front_m = 0.3;
  config.footprint_rear_m = 0.3;
  config.footprint_left_m = 0.3;
  config.footprint_right_m = 0.3;
  config.require_dynamic_source = true;
  config.dynamic_source_labels = {"radar"};
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  const auto radar = makeGrid({{0.0, -0.35, 90}});
  cost_stop.setMergedGrid(radar, 1.0);
  cost_stop.setSourceGrid("radar", radar, 1.0);
  cost_stop.setLaneletGrid(makeGrid({{0.0, 0.15, 100}}), 1.0);

  const auto decision =
      cost_stop.evaluateRouteRecoveryCommand(command(0.0, -0.1), 1.1, 0.3, 0.5);
  EXPECT_TRUE(decision.blocked);
  EXPECT_TRUE(decision.dynamic_obstacle) << decision.reason;
}

TEST(MotionCostStop, PlanningMarginContactStillAllowsProjectedCrabRecovery) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_body_hard_stop_enabled = true;
  config.body_front_m = 0.15;
  config.body_rear_m = 0.15;
  config.body_left_m = 0.15;
  config.body_right_m = 0.15;
  config.lanelet_footprint_enabled = true;
  config.lanelet_footprint_threshold = 100;
  config.footprint_front_m = 0.3;
  config.footprint_rear_m = 0.3;
  config.footprint_left_m = 0.3;
  config.footprint_right_m = 0.3;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  cost_stop.setMergedGrid(makeGrid(), 1.0);
  cost_stop.setLaneletGrid(makeGrid({{0.0, 0.25, 100}}), 1.0);

  const auto trigger = cost_stop.evaluate(command(0.1), 1.0);
  ASSERT_TRUE(trigger.blocked);
  EXPECT_EQ(trigger.reason, "lanelet_footprint_cost");
  EXPECT_TRUE(trigger.lanelet_contact_valid);
  EXPECT_EQ(trigger.lanelet_hit_cost, 100);
  EXPECT_NEAR(trigger.lanelet_hit_body_x, 0.05, 1.0e-6);
  EXPECT_NEAR(trigger.lanelet_hit_body_y, 0.25, 1.0e-6);
  EXPECT_FALSE(
      cost_stop.evaluateRouteRecoveryCommand(command(0.0, -0.1), 1.1, 0.3, 0.5)
          .blocked);
}

TEST(MotionCostStop, PlanningMarginIgnoresLethalCellWhoseCenterIsOutside) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_body_hard_stop_enabled = false;
  config.lanelet_footprint_enabled = true;
  config.lanelet_footprint_threshold = 100;
  config.footprint_front_m = 0.31;
  config.footprint_rear_m = 0.31;
  config.footprint_left_m = 0.31;
  config.footprint_right_m = 0.31;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  cost_stop.setMergedGrid(makeGrid(), 1.0);

  // HH_260806 - Reproduce the field case: the 0.05 m cell center is 1.5 cm
  // outside the planning polygon although the polygon edge maps to that cell.
  cost_stop.setLaneletGrid(makeGrid({{0.0, 0.325, 100}}, 0.05), 1.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.1), 1.0).blocked);

  cost_stop.setLaneletGrid(makeGrid({{0.0, 0.275, 100}}, 0.05), 1.1);
  const auto covered_center = cost_stop.evaluate(command(0.0, 0.1), 1.1);
  EXPECT_TRUE(covered_center.blocked);
  EXPECT_EQ(covered_center.reason, "lanelet_footprint_cost");
}

TEST(MotionCostStop, PhysicalBodyKeepsFailClosedEdgeCellContact) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_body_hard_stop_enabled = true;
  config.lanelet_body_hard_stop_threshold = 100;
  config.body_front_m = 0.31;
  config.body_rear_m = 0.31;
  config.body_left_m = 0.31;
  config.body_right_m = 0.31;
  config.lanelet_footprint_enabled = false;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  cost_stop.setMergedGrid(makeGrid(), 1.0);
  cost_stop.setLaneletGrid(makeGrid({{0.0, 0.325, 100}}, 0.05), 1.0);

  // HH_260806 - Unlike the recoverable margin, a lethal cell touched by the
  // measured body edge must remain a hard stop even when its center is outside.
  const auto decision = cost_stop.evaluate(command(0.0, 0.1), 1.0);
  EXPECT_TRUE(decision.blocked);
  EXPECT_EQ(decision.reason, "lanelet_physical_body_cost");
}

TEST(MotionCostStop, RouteRecoveryCommandFailsClosedWithoutFreshLaneletGrid) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.stale_timeout_s = 1.0;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  cost_stop.setMergedGrid(makeGrid(), 1.0);

  const auto missing =
      cost_stop.evaluateRouteRecoveryCommand(command(0.0, -0.1), 1.0, 0.3, 0.5);
  EXPECT_TRUE(missing.blocked);
  EXPECT_EQ(missing.reason, "route_recovery_lanelet_grid_missing");

  cost_stop.setLaneletGrid(makeGrid(), 1.0);
  const auto stale =
      cost_stop.evaluateRouteRecoveryCommand(command(0.0, -0.1), 2.1, 0.3, 2.0);
  EXPECT_TRUE(stale.blocked);
  EXPECT_EQ(stale.reason, "route_recovery_lanelet_grid_stale");
}

TEST(MotionCostStop, RouteRecoveryCommandFailsClosedWithoutMatchingFrames) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  MotionCostStop cost_stop(config);
  cost_stop.setMergedGrid(makeGrid(), 1.0);

  auto missing_frame_grid = makeGrid();
  missing_frame_grid.header.frame_id.clear();
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  cost_stop.setLaneletGrid(missing_frame_grid, 1.0);
  const auto missing =
      cost_stop.evaluateRouteRecoveryCommand(command(0.0, -0.1), 1.0, 0.3, 0.5);
  EXPECT_TRUE(missing.blocked);
  EXPECT_EQ(missing.reason, "route_recovery_frame_missing");

  cost_stop.setLaneletGrid(makeGrid(), 1.0);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "odom", "test", 1.0});
  const auto mismatch =
      cost_stop.evaluateRouteRecoveryCommand(command(0.0, -0.1), 1.0, 0.3, 0.5);
  EXPECT_TRUE(mismatch.blocked);
  EXPECT_EQ(mismatch.reason, "route_recovery_frame_mismatch");
}

TEST(MotionCostStop, OppositeRecoveryStillStopsForDynamicObstacle) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.footprint_front_m = 0.2;
  config.footprint_rear_m = 0.2;
  config.footprint_left_m = 0.2;
  config.footprint_right_m = 0.2;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 1.0});
  const auto rear_obstacle = makeGrid({{-0.3, 0.0, 90}});
  cost_stop.setMergedGrid(rear_obstacle, 1.0);
  cost_stop.setLaneletGrid(makeGrid({{0.15, 0.0, 100}}), 1.0);
  cost_stop.setSourceGrid("radar", rear_obstacle, 1.0);

  const auto decision =
      cost_stop.evaluateRouteRecoveryCommand(command(-0.1), 1.0, 0.3, 0.5);
  EXPECT_TRUE(decision.blocked);
  EXPECT_TRUE(decision.dynamic_obstacle);
  EXPECT_NE(decision.reason.find("dynamic_rear"), std::string::npos);
}

TEST(MotionCostStop, RotationStopsOnlyOnLiveDynamicSource) {
  auto config = baseCostConfig();
  config.require_dynamic_source = true;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid({{0.5, 0.0, 90}}), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.0, 0.3), 0.0).blocked);
  cost_stop.setSourceGrid("lidar", makeGrid({{0.5, 0.0, 90}}), 0.1);
  EXPECT_TRUE(cost_stop.evaluate(command(0.0, 0.0, 0.3), 0.1).blocked);
}

TEST(MotionCostStop, StationaryCommandDoesNotCreateRotationLatch) {
  auto config = baseCostConfig();
  config.require_dynamic_source = true;
  config.latch_enabled = true;
  auto cost_stop = makeMotionCostStop(config);
  const auto nearby_dynamic_obstacle = makeGrid({{0.05, 0.0, 95}});
  cost_stop.setMergedGrid(nearby_dynamic_obstacle, 0.0);
  cost_stop.setSourceGrid("radar", nearby_dynamic_obstacle, 0.0);

  // HH_260731 - A zero cmd_vel must not be interpreted as an in-place turn.
  // This used to create a persistent dynamic_rotation:radar latch while parked.
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.0, 0.0), 0.0).blocked);
  EXPECT_TRUE(cost_stop.evaluate(command(0.0, 0.0, 0.3), 0.1).blocked);
  // Once real motion triggered the latch, stopping remains fail-closed.
  EXPECT_TRUE(cost_stop.evaluate(command(0.0, 0.0, 0.0), 0.2).blocked);
}

TEST(MotionCostStop, LaneletRotationPolicyChecksCurrentCellWhenDisabled) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  config.lanelet_allow_rotation = true;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  // HH_260721 - Fill the pose neighborhood so grid-edge rounding cannot hide
  // current cost.
  const auto occupied_current_cell = makeGrid({
      {-0.1, -0.1, 100},
      {-0.1, 0.0, 100},
      {-0.1, 0.1, 100},
      {0.0, -0.1, 100},
      {0.0, 0.0, 100},
      {0.0, 0.1, 100},
      {0.1, -0.1, 100},
      {0.1, 0.0, 100},
      {0.1, 0.1, 100},
  });
  cost_stop.setLaneletGrid(occupied_current_cell, 0.0);

  // HH_260721 - Site zero-turn may bypass lanelet cost only when explicitly
  // configured.
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.0, 0.3), 0.0).blocked);
  config.lanelet_allow_rotation = false;
  cost_stop.setConfig(config);
  const auto blocked = cost_stop.evaluate(command(0.0, 0.0, 0.3), 0.1);
  EXPECT_TRUE(blocked.lanelet_violation) << blocked.reason;
}

TEST(MotionCostStop, LaneletFootprintBlocksWhenCenterLinkCellIsClear) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);

  // HH_260806 - robot_center_link at (0,0) is clear, but the configured
  // front-right planning boundary covers this raw lanelet cost cell.
  const auto boundary_cost = makeGrid({{0.65, -0.45, 100}});
  ASSERT_EQ(MotionCostStop::sampleGridCost(boundary_cost, 0.0, 0.0), 0);
  cost_stop.setLaneletGrid(boundary_cost, 0.0);
  const auto decision = cost_stop.evaluate(command(0.2), 0.0);
  EXPECT_TRUE(decision.lanelet_violation) << decision.reason;
  EXPECT_NE(decision.reason.find("lanelet_footprint"), std::string::npos);
}

TEST(MotionCostStop, LaneletFootprintAllowsSoftBoundaryButBlocksOffLane) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_footprint_threshold = 100;
  // HH_260806 - Isolate the complete-footprint 98/100 contract now that the
  // reduced footprint lies inside the legacy front-corridor width.
  config.lanelet_threshold = 100;
  config.lanelet_current_threshold = 100;
  config.lanelet_current_allow_route_reentry = false;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);

  // HH_260727 - The map uses 98 as a narrow-lane planning penalty and 100 as
  // truly off-lane. The full footprint may touch 98 but must stop on 100.
  cost_stop.setLaneletGrid(makeGrid({{0.65, -0.45, 98}}), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.0).blocked);

  cost_stop.setLaneletGrid(makeGrid({{0.65, -0.45, 100}}), 0.1);
  const auto blocked = cost_stop.evaluate(command(0.2), 0.1);
  EXPECT_TRUE(blocked.lanelet_violation) << blocked.reason;
  EXPECT_NE(blocked.reason.find("lanelet_footprint"), std::string::npos);
}

TEST(MotionCostStop,
     LocalPlanningBoundaryOverridesFallbackWithoutPoseTimingDependency) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  // HH_260727 - Deliberately tiny fallback; only the received planning boundary
  // can reach x=1.4.
  config.footprint_front_m = 0.2;
  config.footprint_rear_m = 0.2;
  config.footprint_left_m = 0.2;
  config.footprint_right_m = 0.2;
  auto cost_stop = makeMotionCostStop(config);
  // HH_260807 - The boundary is owned by robot_center_link. Its shape remains
  // exact even when localization has already advanced before the callback.
  cost_stop.setPose(PlanarPose{3.0, -2.0, 0.7, "map", "test", 0.0});
  cost_stop.setFootprintPolygonLocal(
      {{1.5, 0.6}, {1.5, -0.6}, {-0.4, -0.6}, {-0.4, 0.6}});
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 0.0});
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  cost_stop.setLaneletGrid(makeGrid({{1.4, 0.0, 100}}), 0.0);

  const auto decision = cost_stop.evaluate(command(0.2), 0.0);
  EXPECT_TRUE(decision.lanelet_violation) << decision.reason;
}

TEST(MotionCostStop, PlanningMarginStopsBeforeMeasuredBodyTouchesOffLane) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_footprint_threshold = 100;
  config.lanelet_current_allow_route_reentry = false;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);

  // HH_260806 - Exercise the measured body from robot_center_link.
  constexpr double body_front = 0.70837;
  constexpr double body_rear = 0.68323;
  constexpr double body_left = 0.53505;
  constexpr double body_right = 0.53495;
  constexpr double longitudinal_margin = 0.10;
  constexpr double lateral_margin = 0.10;
  // The cell center at y=0.60 m lies outside the 0.53505 m body and inside
  // the 0.63505 m planning boundary.
  auto margin_only_cost = makeGrid({{0.0, 0.60, 100}});
  cost_stop.setLaneletGrid(margin_only_cost, 0.0);

  // HH_260806 - The lethal cell is outside the measured body but inside the
  // published planning boundary. This proves the 0.10 m lateral margin stops
  // motion before the chassis itself reaches the off-lane cell.
  cost_stop.setFootprintPolygonLocal({{body_front, body_left},
                                      {body_front, -body_right},
                                      {-body_rear, -body_right},
                                      {-body_rear, body_left}});
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.2), 0.0).blocked);

  cost_stop.setFootprintPolygonLocal(
      {{body_front + longitudinal_margin, body_left + lateral_margin},
       {body_front + longitudinal_margin, -(body_right + lateral_margin)},
       {-(body_rear + longitudinal_margin), -(body_right + lateral_margin)},
       {-(body_rear + longitudinal_margin), body_left + lateral_margin}});
  const auto blocked = cost_stop.evaluate(command(0.0, 0.2), 0.1);
  EXPECT_TRUE(blocked.lanelet_violation) << blocked.reason;
  EXPECT_NE(blocked.reason.find("lanelet_footprint"), std::string::npos);
}

TEST(MotionCostStop, MeasuredBodyBoundaryStopsOnOffLaneCost) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_footprint_threshold = 100;
  config.lanelet_current_allow_route_reentry = false;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);

  // HH_260806 - Isolate the measured chassis polygon from its planning margin.
  // Use the deployed 0.25 m lanelet-grid resolution: cost 100 on the body edge
  // must stop translation even when crab static-cost bypass is enabled.
  cost_stop.setFootprintPolygonLocal({{0.70837, 0.53505},
                                      {0.70837, -0.53495},
                                      {-0.68323, -0.53495},
                                      {-0.68323, 0.53505}});
  cost_stop.setLaneletGrid(makeGrid({{0.0, 0.40, 100}}, 0.25), 0.0);
  const auto blocked = cost_stop.evaluate(command(0.0, 0.2), 0.0);
  EXPECT_TRUE(blocked.lanelet_violation) << blocked.reason;
  EXPECT_NE(blocked.reason.find("lanelet_footprint"), std::string::npos);
}

TEST(MotionCostStop,
     RotationChecksFullLaneletFootprintEvenWhenCenterRotationIsAllowed) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_allow_rotation = true;
  config.lanelet_current_allow_route_reentry = false;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  cost_stop.setLaneletGrid(makeGrid({{0.65, 0.45, 100}}), 0.0);

  const auto decision = cost_stop.evaluate(command(0.0, 0.0, 0.3), 0.0);
  EXPECT_TRUE(decision.lanelet_violation) << decision.reason;
}

TEST(CommandSourceArbiter, ManeuverOwnsCommandUntilStationaryReleaseCompletes) {
  CommandSourceArbiter arbiter;
  const auto started = arbiter.setManeuverPhases("", "CRAB_IN", "", 1.0);
  EXPECT_TRUE(started.campsite_started);
  EXPECT_EQ(arbiter.evaluate(true, 1.0), CommandSourceDecision::kIgnore);
  EXPECT_EQ(arbiter.evaluate(false, 1.0), CommandSourceDecision::kAllow);

  const auto rotating = arbiter.setManeuverPhases("", "ROTATE_180", "", 2.0);
  EXPECT_FALSE(rotating.maneuver_started);
  EXPECT_EQ(arbiter.evaluate(true, 2.0), CommandSourceDecision::kIgnore);

  const auto finished = arbiter.setManeuverPhases("", "WAIT_RETURN", "", 3.0);
  EXPECT_FALSE(finished.maneuver_finished);
  arbiter.setManeuverPhases("", "DONE", "", 4.0);
  EXPECT_EQ(arbiter.evaluate(true, 4.49), CommandSourceDecision::kHoldZero);
  EXPECT_EQ(arbiter.evaluate(true, 4.50), CommandSourceDecision::kAllow);
}

TEST(CommandSourceArbiter, NormalNav2CommandsNeverCreateAnArtificialHandoff) {
  CommandSourceArbiter arbiter;
  // HH_260806 - RotationShim may alternate pure rotation and translation on a
  // curved route. Both belong to Nav2 and must pass without a stop-go hold.
  EXPECT_EQ(arbiter.evaluate(true, 10.0), CommandSourceDecision::kAllow);
  EXPECT_EQ(arbiter.evaluate(true, 10.01), CommandSourceDecision::kAllow);
  EXPECT_EQ(arbiter.evaluate(true, 10.02), CommandSourceDecision::kAllow);
}

TEST(CommandSourceArbiter, ParkingOwnsRawCommandUntilControllerReturnsIdle) {
  CommandSourceArbiter arbiter;
  // HH_260807 - Parking is a third explicit owner. Nav2 must not interleave a
  // forward command while reverse or AprilTag docking controls the platform.
  const auto started =
      arbiter.setManeuverPhases("IDLE", "IDLE", "REVERSE_APPROACH", 1.0);
  EXPECT_TRUE(started.parking_started);
  EXPECT_EQ(arbiter.evaluate(true, 1.0), CommandSourceDecision::kIgnore);
  EXPECT_EQ(arbiter.evaluate(false, 1.0), CommandSourceDecision::kAllow);

  // HH_260819 - Final yaw and the explicit stopped charging wait retain
  // ownership, so Nav2 cannot interleave translation after the 0.60 m latch.
  const auto yaw =
      arbiter.setManeuverPhases("IDLE", "IDLE", "FINAL_YAW_ALIGNMENT", 1.5);
  EXPECT_FALSE(yaw.maneuver_finished);
  EXPECT_EQ(arbiter.evaluate(true, 1.5), CommandSourceDecision::kIgnore);
  const auto charging_wait =
      arbiter.setManeuverPhases("IDLE", "IDLE", "WAITING_FOR_CHARGING", 1.8);
  EXPECT_FALSE(charging_wait.maneuver_finished);
  EXPECT_EQ(arbiter.evaluate(true, 1.8), CommandSourceDecision::kIgnore);

  const auto parked = arbiter.setManeuverPhases("IDLE", "IDLE", "PARKED", 2.0);
  EXPECT_FALSE(parked.maneuver_finished);
  const auto finished = arbiter.setManeuverPhases("IDLE", "IDLE", "IDLE", 3.0);
  EXPECT_TRUE(finished.maneuver_finished);
  EXPECT_EQ(arbiter.evaluate(true, 3.49), CommandSourceDecision::kHoldZero);
  EXPECT_EQ(arbiter.evaluate(true, 3.50), CommandSourceDecision::kAllow);
}

TEST(CommandSourceArbiter, ExactDropZonePointOwnsCommandBeforeYawAlignment) {
  CommandSourceArbiter arbiter;
  const auto started = arbiter.setManeuverPhases(
      "POSITION_PARKING_POINT", "IDLE", "IDLE", 1.0);
  EXPECT_TRUE(started.drop_zone_started);
  EXPECT_EQ(arbiter.evaluate(false, 1.0), CommandSourceDecision::kAllow);
  EXPECT_EQ(arbiter.evaluate(true, 1.0), CommandSourceDecision::kIgnore);

  const auto align = arbiter.setManeuverPhases(
      "ALIGN_PARKING_YAW", "IDLE", "IDLE", 2.0);
  EXPECT_FALSE(align.maneuver_finished);
  EXPECT_EQ(arbiter.evaluate(true, 2.0), CommandSourceDecision::kIgnore);
}

TEST(MotionCostStop, ConfiguredCampsitePhasesBypassLaneletButKeepDynamicStop) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  const auto boundary_cost = makeGrid({{0.65, -0.45, 100}});

  // HH_260806 - A mapped campsite is outside the road lanelet, so only the
  // dedicated state machine may cross that static boundary.
  for (const auto &phase : config.campsite_lanelet_bypass_phases) {
    SCOPED_TRACE(phase);
    auto cost_stop = makeMotionCostStop(config);
    cost_stop.setManeuverPhases("", phase);
    cost_stop.setMergedGrid(boundary_cost, 0.0);
    cost_stop.setLaneletGrid(boundary_cost, 0.0);
    const auto decision = cost_stop.evaluate(command(0.0, 0.2), 0.0);
    EXPECT_FALSE(decision.blocked) << decision.reason;
  }

  // HH_260806 - The exception is map-only. Live LiDAR/radar evidence still
  // blocks the same crab motion inside an explicit campsite phase.
  auto dynamic_stop = makeMotionCostStop(config);
  dynamic_stop.setManeuverPhases("", "crab_in");
  dynamic_stop.setLaneletGrid(boundary_cost, 0.0);
  dynamic_stop.setMergedGrid(makeGrid({{0.0, 0.5, 90}}), 0.0);
  dynamic_stop.setSourceGrid("lidar", makeGrid({{0.0, 0.5, 90}}), 0.0);
  const auto dynamic_decision = dynamic_stop.evaluate(command(0.0, 0.2), 0.0);
  EXPECT_TRUE(dynamic_decision.blocked);
  EXPECT_TRUE(dynamic_decision.dynamic_obstacle) << dynamic_decision.reason;
}

TEST(MotionCostStop, DropZoneExitBypassesRoadLaneletButNeverLiveObstacle) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  const auto boundary_cost = makeGrid({{0.65, -0.45, 100}});

  config.require_dynamic_source = true;
  // HH_260807 - The charger is outside the road lanelet. Only explicit exit
  // phases may cross it, and the same ordinary command must remain fail-closed.
  auto ordinary_stop = makeMotionCostStop(config);
  ordinary_stop.setMergedGrid(boundary_cost, 0.0);
  ordinary_stop.setLaneletGrid(boundary_cost, 0.0);
  EXPECT_TRUE(ordinary_stop.evaluate(command(0.2), 0.0).lanelet_violation);

  for (const auto &phase : config.drop_zone_lanelet_bypass_phases) {
    SCOPED_TRACE(phase);
    auto cost_stop = makeMotionCostStop(config);
    cost_stop.setManeuverPhases(phase, "");
    cost_stop.setMergedGrid(boundary_cost, 0.0);
    cost_stop.setLaneletGrid(boundary_cost, 0.0);
    EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.0).blocked);

    const auto front_obstacle = makeGrid({{0.4, 0.0, 100}});
    cost_stop.setMergedGrid(front_obstacle, 0.1);
    cost_stop.setSourceGrid("radar", front_obstacle, 0.1);
    const auto obstacle_decision = cost_stop.evaluate(command(0.2), 0.1);
    EXPECT_TRUE(obstacle_decision.blocked);
    EXPECT_TRUE(obstacle_decision.dynamic_obstacle) << obstacle_decision.reason;
  }
}

TEST(MotionCostStop,
     PureLateralAndReverseStaticBypassStillCheckLaneletFootprint) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  config.static_lateral_bypass = true;
  config.static_reverse_bypass = true;
  const auto boundary_cost = makeGrid({{0.65, -0.45, 100}});

  // HH_260727 - The merged-grid static bypass remains useful for crab/reverse
  // maneuvers, but it must not suppress the independent raw-lanelet footprint.
  for (const auto &test_command : {command(0.0, 0.2), command(-0.2)}) {
    auto cost_stop = makeMotionCostStop(config);
    cost_stop.setMergedGrid(boundary_cost, 0.0);
    cost_stop.setLaneletGrid(boundary_cost, 0.0);
    const auto decision = cost_stop.evaluate(test_command, 0.0);
    EXPECT_TRUE(decision.lanelet_violation) << decision.reason;
    EXPECT_NE(decision.reason.find("lanelet_footprint"), std::string::npos);
  }
}

TEST(MotionCostStop, ManeuverBypassStillSkipsLegacyLaneletChecks) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = false;
  config.lanelet_current_allow_route_reentry = false;
  const auto current_cost = makeGrid({{0.0, 0.0, 100}});

  // HH_260727 - Moving footprint evaluation ahead of the bypass must not
  // remove the bounded exception for the legacy base-link/corridor policy.
  for (const auto &phases :
       {std::pair<std::string, std::string>{"", "crab_in"},
        std::pair<std::string, std::string>{"exit_straight", ""}}) {
    SCOPED_TRACE(phases.first + ":" + phases.second);
    auto cost_stop = makeMotionCostStop(config);
    cost_stop.setManeuverPhases(phases.first, phases.second);
    cost_stop.setMergedGrid(makeGrid(), 0.0);
    cost_stop.setLaneletGrid(current_cost, 0.0);
    EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.0).blocked);
  }
}

TEST(MotionCostStop, ClearAvoidancePathPassesAndBlockedPathStops) {
  auto config = baseCostConfig();
  config.require_dynamic_source = true;
  config.dynamic_front_use_local_path = true;
  auto cost_stop = makeMotionCostStop(config);
  const auto obstacle_grid = makeGrid({{1.0, 0.0, 90}});
  cost_stop.setMergedGrid(obstacle_grid, 0.0);
  cost_stop.setSourceGrid("lidar", obstacle_grid, 0.0);
  cost_stop.setLocalPath(makePath({{0.0, 0.0}, {0.5, 0.8}, {2.0, 0.8}}));
  EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.0).blocked);

  cost_stop.setLocalPath(makePath({{0.0, 0.0}, {2.0, 0.0}}));
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 0.1).blocked);
}

TEST(MotionCostStop, ClassifiedFusionStopsOnlyWithinTwoMeterRouteHorizon) {
  auto config = baseCostConfig();
  config.fixed_front_lookahead_m = 3.0;
  config.dynamic_front_use_local_path = true;
  config.dynamic_source_labels = {"fusion", "radar"};
  config.classified_dynamic_source_labels = {"fusion"};
  config.classified_front_lookahead_m = 2.0;
  const auto route = makePath({{0.0, 0.0}, {3.0, 0.0}});

  auto near_fusion = makeMotionCostStop(config);
  near_fusion.setLocalPath(route);
  near_fusion.setSourceGrid("fusion", makeGrid({{1.9, 0.0, 90}}), 0.0);
  const auto near_decision = near_fusion.evaluate(command(0.2), 0.0);
  EXPECT_TRUE(near_decision.blocked);
  EXPECT_EQ(near_decision.reason, "dynamic_front_path:fusion");

  auto far_fusion = makeMotionCostStop(config);
  far_fusion.setLocalPath(route);
  far_fusion.setSourceGrid("fusion", makeGrid({{2.2, 0.0, 90}}), 0.0);
  EXPECT_FALSE(far_fusion.evaluate(command(0.2), 0.0).blocked);

  // A classified object without a current route is not "on the route". Keep
  // it visible for perception diagnostics, but do not convert the fallback
  // rectangular corridor into an early semantic stop.
  auto no_route_fusion = makeMotionCostStop(config);
  no_route_fusion.setSourceGrid("fusion", makeGrid({{1.0, 0.0, 90}}), 0.0);
  EXPECT_FALSE(no_route_fusion.evaluate(command(0.2), 0.0).blocked);

  auto lateral_fusion = makeMotionCostStop(config);
  lateral_fusion.setSourceGrid("fusion", makeGrid({{0.0, 0.5, 90}}), 0.0);
  EXPECT_FALSE(lateral_fusion.evaluate(command(0.0, 0.2), 0.0).blocked);

  // The semantic cap is source-specific. Radar retains its existing independent
  // near-field/front policy instead of being weakened by the fusion horizon.
  auto radar = makeMotionCostStop(config);
  radar.setLocalPath(route);
  radar.setSourceGrid("radar", makeGrid({{2.2, 0.0, 90}}), 0.0);
  EXPECT_TRUE(radar.evaluate(command(0.2), 0.0).blocked);
}

// HH_260728 - Model the live radar cost disk, not a single occupied pixel. A
// return centered 1.0 m to the side must pass during forward travel after its
// 0.30 m inflation, while the same return remains blocking for a lateral move.
TEST(MotionCostStop,
     ForwardUsesNarrowBodySideCheckButManeuverKeepsWideProtection) {
  auto config = baseCostConfig();
  config.body_near_enabled = true;
  config.body_near_side_m = 0.60;
  config.maneuver_body_near_side_m = 1.20;
  config.side_width_m = 1.69160;

  const auto outside_forward_clearance = makeRadarDiskCostGrid(0.0, 1.0, 0.30);
  auto forward_stop = makeMotionCostStop(config);
  forward_stop.setMergedGrid(outside_forward_clearance, 0.0);
  forward_stop.setSourceGrid("radar", outside_forward_clearance, 0.0);
  EXPECT_FALSE(forward_stop.evaluate(command(0.2), 0.0).blocked);

  auto lateral_stop = makeMotionCostStop(config);
  lateral_stop.setMergedGrid(outside_forward_clearance, 0.0);
  lateral_stop.setSourceGrid("radar", outside_forward_clearance, 0.0);
  const auto lateral_decision = lateral_stop.evaluate(command(0.0, 0.2), 0.0);
  EXPECT_TRUE(lateral_decision.blocked);
  EXPECT_NE(lateral_decision.reason.find("left"), std::string::npos);

  const auto inside_forward_clearance = makeRadarDiskCostGrid(0.0, 0.8, 0.30);
  auto close_stop = makeMotionCostStop(config);
  close_stop.setMergedGrid(inside_forward_clearance, 0.0);
  close_stop.setSourceGrid("radar", inside_forward_clearance, 0.0);
  EXPECT_TRUE(close_stop.evaluate(command(0.2), 0.0).blocked);
}

TEST(MotionCostStop, DynamicLatchNeedsContinuousClearWindow) {
  auto config = baseCostConfig();
  config.latch_enabled = true;
  config.clear_required_s = 2.0;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid({{1.0, 0.0, 90}}), 0.0);
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 0.0).blocked);
  EXPECT_TRUE(cost_stop.latched());
  cost_stop.setMergedGrid(makeGrid(), 0.1);
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 0.1).blocked);
  cost_stop.setMergedGrid(makeGrid(), 1.0);
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 1.0).blocked);
  cost_stop.setMergedGrid(makeGrid(), 2.0);
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 2.0).blocked);
  cost_stop.setMergedGrid(makeGrid(), 2.2);
  EXPECT_FALSE(cost_stop.evaluate(command(0.2), 2.2).blocked);
  EXPECT_FALSE(cost_stop.latched());
}

TEST(MotionCostStop,
     LatchedFrontObstacleIgnoresZeroAndChangedDirectionCommands) {
  auto config = baseCostConfig();
  config.require_dynamic_source = true;
  config.latch_enabled = true;
  config.clear_required_s = 2.0;
  config.rotation_radius_m = 1.5;
  auto cost_stop = makeMotionCostStop(config);
  const auto front_obstacle = makeGrid({{1.8, 0.0, 90}});
  cost_stop.setMergedGrid(front_obstacle, 0.0);
  cost_stop.setSourceGrid("radar", front_obstacle, 0.0);

  ASSERT_TRUE(cost_stop.evaluate(command(0.2), 0.0).blocked);
  ASSERT_TRUE(cost_stop.latched());

  // HH_260728 - Reproduce the field stop/go cycle: upstream changes the command
  // after a stop, but the original FRONT corridor must remain the release
  // probe.
  const std::vector<avg_msgs::msg::AvgTwist> changed_commands{
      command(0.0), command(-0.2), command(0.0, 0.2), command(0.0, 0.0, 0.3)};
  const std::vector<double> times{0.5, 1.5, 2.5, 3.5};
  for (std::size_t index = 0; index < times.size(); ++index) {
    cost_stop.setMergedGrid(front_obstacle, times[index]);
    cost_stop.setSourceGrid("radar", front_obstacle, times[index]);
    EXPECT_TRUE(
        cost_stop.evaluate(changed_commands[index], times[index]).blocked);
    EXPECT_TRUE(cost_stop.latched());
  }
}

TEST(MotionCostStop, StaleTriggerSourceCannotProveLatchClear) {
  auto config = baseCostConfig();
  config.require_dynamic_source = true;
  config.latch_enabled = true;
  config.clear_required_s = 2.0;
  config.source_max_age_s = 1.0;
  auto cost_stop = makeMotionCostStop(config);
  const auto front_obstacle = makeGrid({{1.8, 0.0, 90}});
  cost_stop.setMergedGrid(front_obstacle, 0.0);
  cost_stop.setSourceGrid("radar", front_obstacle, 0.0);
  ASSERT_TRUE(cost_stop.evaluate(command(0.2), 0.0).blocked);

  // HH_260728 - A fresh merged clear grid is insufficient when the radar that
  // triggered the stop has stopped publishing; absence of evidence is
  // fail-closed.
  for (const double now_sec : {0.5, 1.5, 2.5, 3.5}) {
    cost_stop.setMergedGrid(makeGrid(), now_sec);
    EXPECT_TRUE(cost_stop.evaluate(command(0.0), now_sec).blocked);
    EXPECT_TRUE(cost_stop.latched());
  }
}

TEST(MotionCostStop, FreshTriggerSourceClearReleasesLatchAndStartsHold) {
  auto config = baseCostConfig();
  config.require_dynamic_source = true;
  config.latch_enabled = true;
  config.clear_required_s = 2.0;
  config.stop_hold_s = 1.0;
  auto cost_stop = makeMotionCostStop(config);
  const auto front_obstacle = makeGrid({{1.8, 0.0, 90}});
  cost_stop.setMergedGrid(front_obstacle, 0.0);
  cost_stop.setSourceGrid("radar", front_obstacle, 0.0);
  ASSERT_TRUE(cost_stop.evaluate(command(0.2), 0.0).blocked);

  // HH_260728 - Only continuously fresh clear frames from both the trigger
  // source and merged grid advance the release timer.
  for (const double now_sec : {0.1, 1.0, 2.0}) {
    const auto clear_grid = makeGrid();
    cost_stop.setMergedGrid(clear_grid, now_sec);
    cost_stop.setSourceGrid("radar", clear_grid, now_sec);
    EXPECT_TRUE(cost_stop.evaluate(command(0.0), now_sec).blocked);
    EXPECT_TRUE(cost_stop.latched());
  }
  const auto clear_grid = makeGrid();
  cost_stop.setMergedGrid(clear_grid, 2.2);
  cost_stop.setSourceGrid("radar", clear_grid, 2.2);
  EXPECT_FALSE(cost_stop.evaluate(command(0.0), 2.2).blocked);
  EXPECT_FALSE(cost_stop.latched());
  EXPECT_GE(cost_stop.holdUntilSec(), 3.2);
}

TEST(MotionCostStop, TriggerPathSnapshotSurvivesReplanWhileLatched) {
  auto config = baseCostConfig();
  config.require_dynamic_source = true;
  config.latch_enabled = true;
  config.dynamic_front_use_local_path = true;
  auto cost_stop = makeMotionCostStop(config);
  const auto front_obstacle = makeGrid({{1.8, 0.0, 90}});
  cost_stop.setLocalPath(makePath({{0.0, 0.0}, {2.0, 0.0}}));
  cost_stop.setMergedGrid(front_obstacle, 0.0);
  cost_stop.setSourceGrid("radar", front_obstacle, 0.0);
  ASSERT_TRUE(cost_stop.evaluate(command(0.2), 0.0).blocked);

  // HH_260728 - Replanning around the stopped robot cannot erase the physical
  // obstacle that caused the latch; its original path probe remains
  // authoritative.
  cost_stop.setLocalPath(makePath({{0.0, 0.0}, {0.5, 0.8}, {2.0, 0.8}}));
  cost_stop.setMergedGrid(front_obstacle, 2.5);
  cost_stop.setSourceGrid("radar", front_obstacle, 2.5);
  EXPECT_TRUE(cost_stop.evaluate(command(0.0), 2.5).blocked);
  EXPECT_TRUE(cost_stop.latched());
}

TEST(MotionCostStop, ZeroStampedOrReplayedClearGridCannotReleaseLatch) {
  auto config = baseCostConfig();
  config.require_dynamic_source = true;
  config.latch_enabled = true;
  config.clear_required_s = 0.5;
  config.stale_timeout_s = 1.0;
  config.source_max_age_s = 1.0;
  auto cost_stop = makeMotionCostStop(config);

  auto obstacle = makeGrid({{1.8, 0.0, 90}});
  setGridStamp(obstacle, 10.0);
  cost_stop.setMergedGrid(obstacle, 10.0);
  cost_stop.setSourceGrid("radar", obstacle, 10.0);
  ASSERT_TRUE(cost_stop.evaluate(command(0.2), 10.0).blocked);

  const auto zero_stamped_clear = makeGrid();
  cost_stop.setMergedGrid(zero_stamped_clear, 10.1);
  cost_stop.setSourceGrid("radar", zero_stamped_clear, 10.1);
  EXPECT_TRUE(cost_stop.evaluate(command(0.0), 10.1).blocked);
  EXPECT_TRUE(cost_stop.latched());

  auto replayed_clear = makeGrid();
  setGridStamp(replayed_clear, 10.1);
  // HH_260728 - Receiving the same old clear payload again may refresh the
  // callback time, but it cannot satisfy a continuous-new-evidence window.
  cost_stop.setMergedGrid(replayed_clear, 10.2);
  cost_stop.setSourceGrid("radar", replayed_clear, 10.2);
  EXPECT_TRUE(cost_stop.evaluate(command(0.0), 10.2).blocked);
  cost_stop.setMergedGrid(replayed_clear, 10.8);
  cost_stop.setSourceGrid("radar", replayed_clear, 10.8);
  EXPECT_TRUE(cost_stop.evaluate(command(0.0), 10.8).blocked);
  EXPECT_TRUE(cost_stop.latched());

  auto fresh_clear = makeGrid();
  setGridStamp(fresh_clear, 10.8);
  cost_stop.setMergedGrid(fresh_clear, 10.8);
  cost_stop.setSourceGrid("radar", fresh_clear, 10.8);
  EXPECT_FALSE(cost_stop.evaluate(command(0.0), 10.8).blocked);
  EXPECT_FALSE(cost_stop.latched());
}

TEST(MotionCostStop, RotationLatchKeepsTriggerGeometryAfterDynamicRetune) {
  auto config = baseCostConfig();
  config.require_dynamic_source = true;
  config.latch_enabled = true;
  config.rotation_radius_m = 1.5;
  config.rotation_threshold = 85;
  auto cost_stop = makeMotionCostStop(config);
  const auto obstacle = makeGrid({{1.0, 0.0, 90}});
  cost_stop.setMergedGrid(obstacle, 0.0);
  cost_stop.setSourceGrid("radar", obstacle, 0.0);
  ASSERT_TRUE(cost_stop.evaluate(command(0.0, 0.0, 0.2), 0.0).blocked);

  // HH_260728 - A dynamic UI retune applies to the next trigger; it cannot
  // shrink the saved release probe for an obstacle that is already latched.
  config.rotation_radius_m = 0.2;
  config.rotation_threshold = 100;
  cost_stop.setConfig(config);
  cost_stop.setMergedGrid(obstacle, 0.5);
  cost_stop.setSourceGrid("radar", obstacle, 0.5);
  EXPECT_TRUE(cost_stop.evaluate(command(0.0), 0.5).blocked);
  EXPECT_TRUE(cost_stop.latched());
}

TEST(MotionCostStop, MissingAndStaleMergedGridFailClosed) {
  auto config = baseCostConfig();
  config.stale_stop_enabled = true;
  config.stale_timeout_s = 1.0;
  auto cost_stop = makeMotionCostStop(config);
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 0.0).stale_grid);
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.5).blocked);
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 1.1).stale_grid);
}

TEST(MotionCostStop, LaneletPathAllowsNearbyBoundaryButStopsPathCost) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_front_use_local_path = true;
  config.lanelet_path_width_m = 0.2;
  config.lanelet_lookahead_m = 1.0;
  // HH_260721 - This case validates normal in-lane driving, not bounded route
  // re-entry.
  config.lanelet_current_allow_route_reentry = false;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  cost_stop.setLocalPath(makePath({{0.0, 0.0}, {2.0, 0.0}}));
  cost_stop.setLaneletGrid(makeGrid({{1.0, 0.6, 100}}), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.0).blocked);
  // HH_260721 - Use a finite blocked lanelet segment instead of a
  // quantization-sensitive pixel.
  const auto path_blocked_lanelet_grid = makeGrid({{0.9, -0.1, 100},
                                                   {0.9, 0.0, 100},
                                                   {0.9, 0.1, 100},
                                                   {1.0, -0.1, 100},
                                                   {1.0, 0.0, 100},
                                                   {1.0, 0.1, 100},
                                                   {1.1, -0.1, 100},
                                                   {1.1, 0.0, 100},
                                                   {1.1, 0.1, 100}});
  ASSERT_GE(MotionCostStop::sampleGridCost(path_blocked_lanelet_grid, 1.0, 0.0),
            85);
  cost_stop.setLaneletGrid(path_blocked_lanelet_grid, 0.1);
  const auto blocked_decision = cost_stop.evaluate(command(0.2), 0.1);
  EXPECT_TRUE(blocked_decision.lanelet_violation) << blocked_decision.reason;
}

TEST(MotionCostStop,
     ReverseParkingIgnoresDisabledLaneletDirectionButStopsDynamicObstacle) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  config.lanelet_check_reverse = false;
  config.require_dynamic_source = true;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  // HH_260721 - Fill the pose neighborhood so floating-point cell edges cannot
  // weaken this cost_stop.
  const auto current_lanelet_cost = makeGrid({
      {-0.1, -0.1, 100},
      {-0.1, 0.0, 100},
      {-0.1, 0.1, 100},
      {0.0, -0.1, 100},
      {0.0, 0.0, 100},
      {0.0, 0.1, 100},
      {0.1, -0.1, 100},
      {0.1, 0.0, 100},
      {0.1, 0.1, 100},
  });
  ASSERT_GE(MotionCostStop::sampleGridCost(current_lanelet_cost, 0.0, 0.0), 85);
  cost_stop.setLaneletGrid(current_lanelet_cost, 0.0);

  // HH_260721 - Final parking may leave the lanelet, while live rear obstacles
  // remain mandatory.
  EXPECT_FALSE(cost_stop.evaluate(command(-0.2), 0.0).blocked);
  const auto rear_obstacle = makeGrid({{-0.4, 0.0, 100}});
  cost_stop.setSourceGrid("lidar", rear_obstacle, 0.1);
  EXPECT_TRUE(cost_stop.evaluate(command(-0.2), 0.1).dynamic_obstacle);

  config.lanelet_check_reverse = true;
  cost_stop.setConfig(config);
  EXPECT_TRUE(cost_stop.evaluate(command(-0.2), 0.2).lanelet_violation);
}

TEST(MotionCostStop, ParkingPhaseBypassesRoadLaneletButNeverLiveObstacle) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_body_hard_stop_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_check_reverse = true;
  config.require_dynamic_source = true;
  auto cost_stop = makeMotionCostStop(config);
  const auto charger_outside_road = makeGrid({
      {-0.1, -0.1, 100},
      {-0.1, 0.0, 100},
      {-0.1, 0.1, 100},
      {0.0, -0.1, 100},
      {0.0, 0.0, 100},
      {0.0, 0.1, 100},
      {0.1, -0.1, 100},
      {0.1, 0.0, 100},
      {0.1, 0.1, 100},
  });
  cost_stop.setMergedGrid(charger_outside_road, 0.0);
  cost_stop.setLaneletGrid(charger_outside_road, 0.0);

  // HH_260807 - Ordinary reverse remains fail-closed at the same pose.
  EXPECT_TRUE(cost_stop.evaluate(command(-0.2), 0.0).lanelet_violation);

  cost_stop.setManeuverPhases("IDLE", "IDLE", "REVERSE_APPROACH");
  EXPECT_FALSE(cost_stop.evaluate(command(-0.2), 0.1).blocked);
  const auto rear_obstacle = makeGrid({{-0.4, 0.0, 100}});
  cost_stop.setSourceGrid("lidar", rear_obstacle, 0.2);
  EXPECT_TRUE(cost_stop.evaluate(command(-0.2), 0.2).dynamic_obstacle);

  auto retry_stop = makeMotionCostStop(config);
  retry_stop.setMergedGrid(charger_outside_road, 1.0);
  retry_stop.setLaneletGrid(charger_outside_road, 1.0);
  retry_stop.setManeuverPhases("IDLE", "IDLE", "RETRY_FORWARD_EXIT");
  EXPECT_FALSE(retry_stop.evaluate(command(0.2), 1.0).blocked);
  const auto front_obstacle = makeGrid({{0.4, 0.0, 100}});
  retry_stop.setSourceGrid("radar", front_obstacle, 1.1);
  EXPECT_TRUE(retry_stop.evaluate(command(0.2), 1.1).dynamic_obstacle);
}

TEST(MotionCostStop, ManeuverPhaseBypassesStaticButNotDynamicCost) {
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.require_dynamic_source = true;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setManeuverPhases("", "CRAB_IN");
  const auto blocked_grid = makeGrid({{0.0, 0.5, 100}});
  cost_stop.setMergedGrid(blocked_grid, 0.0);
  cost_stop.setLaneletGrid(blocked_grid, 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.2), 0.0).blocked);
  cost_stop.setSourceGrid("radar", blocked_grid, 0.1);
  EXPECT_TRUE(cost_stop.evaluate(command(0.0, 0.2), 0.1).dynamic_obstacle);
}

TEST(MotionCostStop, RotatedGridOriginSamplingIsCorrect) {
  auto grid = makeGrid({}, 0.1, 3.14159265358979323846 * 0.5);
  grid.info.origin.position.x = 4.0;
  grid.info.origin.position.y = -4.0;
  const int grid_x = 40;
  const int grid_y = 30;
  grid.data[grid_y * static_cast<int>(grid.info.width) + grid_x] = 92;
  const double local_x = (grid_x + 0.5) * grid.info.resolution;
  const double local_y = (grid_y + 0.5) * grid.info.resolution;
  const double world_x = grid.info.origin.position.x - local_y;
  const double world_y = grid.info.origin.position.y + local_x;
  EXPECT_EQ(MotionCostStop::sampleGridCost(grid, world_x, world_y), 92);
}

TEST(MotionCostStop, LargeSubthresholdClusterIsUnavoidable) {
  auto config = baseCostConfig();
  config.cost_stop_threshold = 95;
  config.unavoidable_threshold = 90;
  config.unavoidable_min_cells = 25;
  config.unavoidable_min_ratio = 0.20;
  auto cluster_grid = makeGrid();
  // HH_260721 - Fill cell centers directly to avoid fixture rounding gaps in
  // connectivity.
  for (int y = 0; y < static_cast<int>(cluster_grid.info.height); ++y) {
    for (int x = 0; x < static_cast<int>(cluster_grid.info.width); ++x) {
      const double world_x = cluster_grid.info.origin.position.x +
                             (x + 0.5) * cluster_grid.info.resolution;
      const double world_y = cluster_grid.info.origin.position.y +
                             (y + 0.5) * cluster_grid.info.resolution;
      if (world_x >= 0.4 && world_x <= 1.6 && world_y >= -0.6 &&
          world_y <= 0.6) {
        cluster_grid.data[y * static_cast<int>(cluster_grid.info.width) + x] =
            91;
      }
    }
  }
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(cluster_grid, 0.0);
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 0.0).blocked);
}

} // namespace camrod_control
