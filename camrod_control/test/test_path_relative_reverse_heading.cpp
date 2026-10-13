// HH_261006 - Explicit reverse episodes change only the chassis-heading target.

#include <cmath>
#include <limits>

#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "camrod_control/path_relative_route_recovery.hpp"
#include "camrod_control/route_safety_recovery.hpp"
#include "gtest/gtest.h"

namespace camrod_control {
namespace {

constexpr double kPi = 3.14159265358979323846;
double radians(const double degrees) { return degrees * kPi / 180.0; }

avg_msgs::msg::AvgTwist twist(const double x, const double y = 0.0,
                            const double yaw = 0.0) {
  avg_msgs::msg::AvgTwist result;
  result.linear.x = x;
  result.linear.y = y;
  result.angular.z = yaw;
  return result;
}

avg_msgs::msg::AvgPath straightPath() {
  avg_msgs::msg::AvgPath result;
  result.header.frame_id = "map";
  for (const double x : {-2.0, 0.0, 2.0}) {
    avg_msgs::msg::AvgPoseStamped point;
    point.header.frame_id = "map";
    point.pose.position.x = x;
    point.pose.orientation.w = 1.0;
    result.poses.push_back(point);
  }
  return result;
}

avg_msgs::msg::AvgOccupancyGrid clearGrid() {
  avg_msgs::msg::AvgOccupancyGrid result;
  result.header.frame_id = "map";
  result.info.resolution = 0.1;
  result.info.width = 100;
  result.info.height = 100;
  result.info.origin.position.x = -5.0;
  result.info.origin.position.y = -5.0;
  result.info.origin.orientation.w = 1.0;
  result.data.assign(10000, 0);
  return result;
}

TEST(PathRelativeReverseHeading, RequiresActiveFiniteDominantReverseTrigger) {
  EXPECT_TRUE(pathRelativeReverseHeadingFromTrigger(twist(-0.20), true));
  EXPECT_TRUE(pathRelativeReverseHeadingFromTrigger(twist(-0.10, 0.02), true));
  for (const auto &command : {twist(0.20), twist(0.0), twist(0.0, 0.10),
                             twist(0.0, 0.0, -0.10), twist(-0.01),
                             twist(-0.02), twist(-0.10, 0.10),
                             twist(-0.10, -0.20)}) {
    EXPECT_FALSE(pathRelativeReverseHeadingFromTrigger(command, true));
  }
  EXPECT_FALSE(pathRelativeReverseHeadingFromTrigger(twist(-0.20), false));
  EXPECT_FALSE(pathRelativeReverseHeadingFromTrigger(
      twist(-0.20), true, std::numeric_limits<double>::quiet_NaN()));
  // Invalid components never establish reverse authority, even off-plane ones.
  for (int index = 0; index < 6; ++index) {
    auto command = twist(-0.20);
    double *components[] = {&command.linear.x, &command.linear.y,
                            &command.linear.z, &command.angular.x,
                            &command.angular.y, &command.angular.z};
    *components[index] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(pathRelativeReverseHeadingFromTrigger(command, true));
  }
}

TEST(PathRelativeReverseHeading, ReverseChangesOnlyHeadingNotOrderedGeometry) {
  const PlanarPose pose{0.2, 0.047, radians(-154.08), "map", "test", 1.0};
  const auto forward = nearestPathRelativeRouteRelation(straightPath(), pose, 1.5);
  const auto reverse = nearestPathRelativeRouteRelation(straightPath(), pose, 1.5, true);
  ASSERT_TRUE(forward.valid);
  ASSERT_TRUE(reverse.valid);
  EXPECT_NEAR(forward.heading_error_rad, radians(-154.08), 1.0e-12);
  EXPECT_NEAR(reverse.heading_error_rad, radians(25.92), 1.0e-12);
  EXPECT_DOUBLE_EQ(reverse.signed_cte_m, forward.signed_cte_m);
  EXPECT_DOUBLE_EQ(reverse.distance_m, forward.distance_m);
  EXPECT_DOUBLE_EQ(reverse.tangent_x, forward.tangent_x);
  EXPECT_DOUBLE_EQ(reverse.tangent_y, forward.tangent_y);
  EXPECT_EQ(reverse.segment_index, forward.segment_index);
  EXPECT_EQ(reverse.reason, forward.reason);
  const auto forward_crab = pathRelativeInwardNormalCrabCandidate(pose, forward, 0.10);
  const auto reverse_crab = pathRelativeInwardNormalCrabCandidate(pose, reverse, 0.10);
  EXPECT_TRUE(pathRelativeCandidateCommandExactlyMatches(forward_crab, reverse_crab));
  EXPECT_DOUBLE_EQ(pathRelativeCteRate(forward_crab.command, pose, forward),
                   pathRelativeCteRate(reverse_crab.command, pose, reverse));
}

TEST(PathRelativeReverseHeading, DoesNotChooseWhicheverHeadingIsCloser) {
  const PlanarPose pose{0.0, 0.04, radians(12.2), "map", "test", 1.0};
  const auto forward = nearestPathRelativeRouteRelation(straightPath(), pose, 1.5);
  const auto reverse = nearestPathRelativeRouteRelation(straightPath(), pose, 1.5, true);
  EXPECT_NEAR(forward.heading_error_rad, radians(12.2), 1.0e-12);
  EXPECT_NEAR(reverse.heading_error_rad, radians(-167.8), 1.0e-12);
}

TEST(PathRelativeReverseHeading, CorrectsReverseResidualWithOppositeYawSign) {
  const PlanarPose pose{0.2, 0.047, radians(-154.08), "map", "test", 1.0};
  const auto forward = nearestPathRelativeRouteRelation(straightPath(), pose, 1.5);
  const auto reverse = nearestPathRelativeRouteRelation(straightPath(), pose, 1.5, true);
  const auto old_heading = pathRelativeHeadingCorrectionCandidate(pose, forward, 0.10, 0.10);
  const auto corrected = pathRelativeHeadingCorrectionCandidate(pose, reverse, 0.10, 0.10);
  ASSERT_TRUE(corrected.available());
  EXPECT_GT(old_heading.command.angular.z, 0.0);
  EXPECT_LT(corrected.command.angular.z, 0.0);
  EXPECT_TRUE(pathRelativeCommandStrictlyReducesAbsoluteCte(corrected.command, pose, reverse));
  const MotionCostStopDecision clear{};
  PathRelativeRecoveryState state;
  const auto selected = selectPathRelativeRouteRecoveryCandidate(
      twist(-0.20), pose, reverse, PathRelativeRecoveryConfig{}, 0.10, 0.10,
      {clear, clear, clear, clear, clear}, state);
  EXPECT_TRUE(pathRelativeCandidateCommandExactlyMatches(selected, corrected));
  EXPECT_TRUE(state.motion_authorized);
}

TEST(PathRelativeReverseHeading, ReverseAlignedChassisSettlesBeforeHandoff) {
  const PlanarPose pose{0.2, 0.04, kPi, "map", "test", 1.0};
  const auto relation = nearestPathRelativeRouteRelation(straightPath(), pose, 1.5, true);
  const MotionCostStopDecision clear{};
  const PathRelativeRecoveryConfig config;
  PathRelativeRecoveryState state;
  EXPECT_FALSE(pathRelativeRecoveryHandoffReady(state, relation, config));
  const auto selected = selectPathRelativeRouteRecoveryCandidate(
      twist(-0.20), pose, relation, config, 0.10, 0.10,
      {clear, clear, clear, clear, clear}, state);
  EXPECT_FALSE(selected.available());
  EXPECT_FALSE(state.motion_authorized);
  EXPECT_TRUE(state.settle_zero_commanded);
  EXPECT_TRUE(pathRelativeRecoveryHandoffReady(state, relation, config));
  const auto forward = nearestPathRelativeRouteRelation(straightPath(), pose, 1.5);
  EXPECT_FALSE(pathRelativeRecoveryHandoffReady(state, forward, config));
}

TEST(PathRelativeReverseHeading, StoredEpisodeTriggerCannotBeReplacedByRecovery) {
  RouteSafetyRecovery recovery;
  const MotionCostStopDecision boundary{true, false, true, false, "lanelet_cost"};
  ASSERT_TRUE(recovery.observeViolation(boundary, twist(-0.20), 1.0));
  EXPECT_TRUE(pathRelativeReverseHeadingFromTrigger(recovery.triggerCommand(), recovery.active()));
  EXPECT_FALSE(recovery.observeViolation(boundary, twist(0.10, 0.0, -0.10), 2.0));
  EXPECT_TRUE(pathRelativeReverseHeadingFromTrigger(recovery.triggerCommand(), recovery.active()));
  recovery.reset();
  EXPECT_FALSE(pathRelativeReverseHeadingFromTrigger(recovery.triggerCommand(), recovery.active()));
}

TEST(PathRelativeReverseHeading, CorrectedHeadingStillRejectsEveryBlockedProjection) {
  const PlanarPose pose{0.2, 0.047, radians(-154.08), "map", "test", 1.0};
  const auto relation = nearestPathRelativeRouteRelation(straightPath(), pose, 1.5, true);
  for (const auto &blocked : {
      MotionCostStopDecision{true, false, true, false, "route_recovery_physical_body_overlap_increased"},
      MotionCostStopDecision{true, true, false, false, "dynamic_rear:radar"},
      MotionCostStopDecision{true, false, true, true, "route_recovery_lanelet_grid_stale"}}) {
    PathRelativeRecoveryState state;
    const auto selected = selectPathRelativeRouteRecoveryCandidate(
        twist(-0.20), pose, relation, PathRelativeRecoveryConfig{}, 0.10, 0.10,
        {blocked, blocked, blocked, blocked, blocked}, state);
    EXPECT_FALSE(selected.available());
    EXPECT_FALSE(state.motion_authorized);
    EXPECT_FALSE(pathRelativeRecoveryHandoffReady(state, relation, PathRelativeRecoveryConfig{}));
  }
}

TEST(PathRelativeReverseHeading, StalePoseAndLaneletStillBlockActualProjection) {
  MotionCostStop cost_stop;
  const PlanarPose pose{0.2, 0.047, radians(-154.08), "map", "test", 1.0};
  const auto relation = nearestPathRelativeRouteRelation(straightPath(), pose, 1.5, true);
  const auto candidate = pathRelativeHeadingCorrectionCandidate(pose, relation, 0.10, 0.10);
  ASSERT_TRUE(candidate.available());
  cost_stop.setPose(pose);
  cost_stop.setLaneletGrid(clearGrid(), 3.0);
  cost_stop.setMergedGrid(clearGrid(), 3.0);
  cost_stop.setSourceGrid("radar", clearGrid(), 3.0);
  const auto stale_pose = cost_stop.probeRouteRecoveryCommand(candidate.command, 3.0, 0.25, 0.5);
  EXPECT_TRUE(stale_pose.blocked);
  EXPECT_TRUE(stale_pose.stale_grid);
  EXPECT_EQ(stale_pose.reason, "route_recovery_pose_stale");
  cost_stop.setPose(PlanarPose{pose.x, pose.y, pose.yaw, "map", "test", 5.0});
  const auto stale_grid = cost_stop.probeRouteRecoveryCommand(candidate.command, 5.0, 0.25, 0.5);
  EXPECT_TRUE(stale_grid.blocked);
  EXPECT_TRUE(stale_grid.stale_grid);
  EXPECT_EQ(stale_grid.reason, "route_recovery_lanelet_grid_stale");
}

} // namespace
} // namespace camrod_control
