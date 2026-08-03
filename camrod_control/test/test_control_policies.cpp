// HH_260721 - Verify command gating, charging mission override, and all-direction cost stopping.

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "avg_msgs/msg/avg_occupancy_grid.hpp"
#include "avg_msgs/msg/avg_path.hpp"
#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "avg_msgs/msg/avg_twist.hpp"
#include "camrod_control/charging_mission_override.hpp"
#include "camrod_control/cmd_vel_gate_policy.hpp"
#include "camrod_control/motion_cost_stop.hpp"
#include "camrod_control/route_recovery_candidate.hpp"
#include "camrod_control/route_safety_recovery.hpp"
#include "gtest/gtest.h"

namespace camrod_control
{

namespace
{

avg_msgs::msg::AvgOccupancyGrid makeGrid(
  const std::vector<std::tuple<double, double, int>> & occupied_cells = {},
  const double resolution = 0.1,
  const double origin_yaw = 0.0)
{
  avg_msgs::msg::AvgOccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = resolution;
  grid.info.width = 80;
  grid.info.height = 80;
  grid.info.origin.position.x = -4.0;
  grid.info.origin.position.y = -4.0;
  grid.info.origin.orientation.z = std::sin(origin_yaw * 0.5);
  grid.info.origin.orientation.w = std::cos(origin_yaw * 0.5);
  grid.data.assign(grid.info.width * grid.info.height, 0);
  for (const auto & cell : occupied_cells) {
    const double world_x = std::get<0>(cell);
    const double world_y = std::get<1>(cell);
    // HH_260721 - Test fixtures use axis-aligned grids unless origin rotation is tested explicitly.
    const int x =
      static_cast<int>(std::floor((world_x - grid.info.origin.position.x) / resolution));
    const int y =
      static_cast<int>(std::floor((world_y - grid.info.origin.position.y) / resolution));
    if (x >= 0 && y >= 0 && x < static_cast<int>(grid.info.width) &&
      y < static_cast<int>(grid.info.height))
    {
      grid.data[y * static_cast<int>(grid.info.width) + x] = std::get<2>(cell);
    }
  }
  return grid;
}

avg_msgs::msg::AvgOccupancyGrid makeRadarDiskCostGrid(
  const double center_x,
  const double center_y,
  const double radius_m,
  const int cost = 90,
  const double resolution = 0.1)
{
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
        y < static_cast<int>(grid.info.height))
      {
        grid.data[y * static_cast<int>(grid.info.width) + x] = cost;
      }
    }
  }
  return grid;
}

void setGridStamp(avg_msgs::msg::AvgOccupancyGrid & grid, const double stamp_sec)
{
  const double integral = std::floor(stamp_sec);
  grid.header.stamp.sec = static_cast<std::int32_t>(integral);
  grid.header.stamp.nanosec =
    static_cast<std::uint32_t>(std::llround((stamp_sec - integral) * 1.0e9));
}

avg_msgs::msg::AvgPath makePath(const std::vector<std::pair<double, double>> & points)
{
  avg_msgs::msg::AvgPath path;
  path.header.frame_id = "map";
  for (const auto & point : points) {
    avg_msgs::msg::AvgPoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    path.poses.push_back(pose);
  }
  return path;
}

MotionCostStopConfig baseCostConfig()
{
  MotionCostStopConfig config;
  config.stale_stop_enabled = false;
  config.lanelet_enabled = false;
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
  return config;
}

MotionCostStop makeMotionCostStop(MotionCostStopConfig config = baseCostConfig())
{
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test"});
  return cost_stop;
}

avg_msgs::msg::AvgTwist command(const double x, const double y = 0.0, const double yaw = 0.0)
{
  avg_msgs::msg::AvgTwist output;
  output.linear.x = x;
  output.linear.y = y;
  output.angular.z = yaw;
  return output;
}

}  // namespace

TEST(CmdVelGatePolicy, RequiresEngageOperatorArmAndHealthyCan)
{
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

TEST(CmdVelGatePolicy, BlocksCanFaultStaleStatusChargingAndCriticalSoc)
{
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
    CmdVelGatePolicy::formatPlatformErrorCode(0x0100).find(
      "motor_driver"), std::string::npos);

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
}

TEST(CmdVelGatePolicy, SeparatesNormalStandbyFromWarningAndErrorHolds)
{
  // HH_260721 - Health severity must describe faults, not whether motion is currently authorized.
  EXPECT_EQ(
    CmdVelGatePolicy::classifyHealth(
      {"engage=false(manual=false,mission=false)", "platform_drive_enable=false"}),
    CmdVelGateHealth::kOk);
  EXPECT_EQ(
    CmdVelGatePolicy::classifyHealth({"charging"}), CmdVelGateHealth::kOk);
  EXPECT_EQ(
    CmdVelGatePolicy::classifyHealth({"cost_stop_latched"}),
    CmdVelGateHealth::kWarning);
  EXPECT_EQ(
    CmdVelGatePolicy::classifyHealth({"platform_status_stale=1.20s"}),
    CmdVelGateHealth::kError);
  EXPECT_EQ(
    CmdVelGatePolicy::classifyHealth({"platform_error=0x0100(motor_driver)"}),
    CmdVelGateHealth::kError);
}

TEST(ChargingMissionOverride, AcceptsOnlyFreshCampsiteRequestDuringCharging)
{
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

TEST(MotionCostStop, ForwardThresholdAndBelowThreshold)
{
  auto cost_stop = makeMotionCostStop();
  cost_stop.setMergedGrid(makeGrid({{1.0, 0.0, 90}}), 0.0);
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 0.0).blocked);

  cost_stop = makeMotionCostStop();
  cost_stop.setMergedGrid(makeGrid({{1.0, 0.0, 60}}), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.0).blocked);
}

TEST(MotionCostStop, CrabAndReverseUseTravelDirection)
{
  auto cost_stop = makeMotionCostStop();
  cost_stop.setMergedGrid(makeGrid({{1.0, 0.0, 90}}), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.2), 0.0).blocked);
  const auto left_obstacle = makeGrid({{0.0, 0.5, 90}});
  cost_stop.setMergedGrid(left_obstacle, 0.1);
  // HH_260721 - Site-motion static bypass still requires live source cost to stop.
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

TEST(RouteSafetyRecovery, PreservesTriggerDirectionUntilContinuousClear)
{
  RouteSafetyRecoveryConfig config;
  config.clear_required_s = 1.0;
  RouteSafetyRecovery recovery(config);
  const MotionCostStopDecision violation{
    true, false, true, false, "lanelet_footprint_cost"};

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
  EXPECT_FALSE(recovery.updateProbe(violation, 11.5));
  EXPECT_FALSE(recovery.updateProbe(MotionCostStopDecision{}, 12.0));
  EXPECT_TRUE(recovery.updateProbe(MotionCostStopDecision{}, 13.0));
  EXPECT_FALSE(recovery.active());
}

TEST(RouteSafetyRecovery, AdmitsOppositeAndOrthogonalProjectedCandidates)
{
  RouteSafetyRecovery recovery;
  const MotionCostStopDecision violation{
    true, false, true, false, "lanelet_front_cost"};
  ASSERT_TRUE(recovery.observeViolation(violation, command(0.3), 1.0));

  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(-0.1)));
  EXPECT_FALSE(recovery.permitsProjectedRecoveryCandidate(command(0.1)));
  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(0.0, 0.1)));
  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(0.0, -0.1)));
  EXPECT_FALSE(
    recovery.permitsProjectedRecoveryCandidate(command(0.0, 0.0, 0.2)));
}

TEST(RouteSafetyRecovery, RotationViolationAdmitsTranslationForProjectedCheck)
{
  RouteSafetyRecovery recovery;
  const MotionCostStopDecision violation{
    true, false, true, false, "lanelet_footprint_cost"};
  ASSERT_TRUE(
    recovery.observeViolation(violation, command(0.0, 0.0, 0.2), 1.0));

  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(0.0, 0.1)));
  EXPECT_TRUE(recovery.permitsProjectedRecoveryCandidate(command(-0.1)));
  EXPECT_FALSE(
    recovery.permitsProjectedRecoveryCandidate(command(0.0, 0.0, -0.2)));
}

TEST(RouteRecoveryCandidate, SelectsOnlyUniqueClearCrabSide)
{
  const MotionCostStopDecision clear{};
  const MotionCostStopDecision blocked{
    true, false, true, false, "route_recovery_predicted_lanelet_footprint_cost"};
  const auto selected = selectRouteRecoveryCandidate(
    command(0.3), 0.10, clear, blocked, clear);

  ASSERT_TRUE(selected.available());
  EXPECT_EQ(selected.kind, RouteRecoveryCandidateKind::kCrabLeft);
  EXPECT_NEAR(selected.command.linear.x, 0.0, 1.0e-9);
  EXPECT_NEAR(selected.command.linear.y, 0.10, 1.0e-9);
  EXPECT_NEAR(selected.command.angular.z, 0.0, 1.0e-9);
}

TEST(RouteRecoveryCandidate, FallsBackToReverseWhenBothCrabSidesAreBlocked)
{
  const MotionCostStopDecision clear{};
  const MotionCostStopDecision blocked{
    true, false, true, false, "route_recovery_predicted_lanelet_footprint_cost"};
  const auto selected = selectRouteRecoveryCandidate(
    command(0.3), 0.10, blocked, blocked, clear);

  ASSERT_TRUE(selected.available());
  EXPECT_EQ(selected.kind, RouteRecoveryCandidateKind::kReverse);
  EXPECT_NEAR(selected.command.linear.x, -0.10, 1.0e-9);
  EXPECT_NEAR(selected.command.linear.y, 0.0, 1.0e-9);
}

TEST(RouteRecoveryCandidate, StopsWhenLateralChoiceIsAmbiguousOrNothingIsClear)
{
  const MotionCostStopDecision clear{};
  const MotionCostStopDecision blocked{
    true, false, true, false, "route_recovery_predicted_lanelet_footprint_cost"};

  const auto ambiguous = selectRouteRecoveryCandidate(
    command(0.3), 0.10, clear, clear, clear);
  EXPECT_FALSE(ambiguous.available());
  EXPECT_EQ(ambiguous.reason, "ambiguous_lateral_clear");

  const auto none = selectRouteRecoveryCandidate(
    command(0.3), 0.10, blocked, blocked, blocked);
  EXPECT_FALSE(none.available());
  EXPECT_EQ(none.reason, "no_projected_candidate_clear");
}

TEST(RouteRecoveryCandidate, KeepsInitialDirectionWhenBothSidesLaterBecomeClear)
{
  const MotionCostStopDecision clear{};
  const auto selected = continueRouteRecoveryCandidate(
    command(0.3), RouteRecoveryCandidateKind::kCrabLeft,
    0.10, clear, clear, clear);

  ASSERT_TRUE(selected.available());
  EXPECT_EQ(selected.kind, RouteRecoveryCandidateKind::kCrabLeft);
  EXPECT_NEAR(selected.command.linear.y, 0.10, 1.0e-9);
  EXPECT_EQ(selected.reason, "latched_candidate_still_clear");
}

TEST(RouteRecoveryCandidate, NeverSwitchesWhenLatchedDirectionBecomesBlocked)
{
  const MotionCostStopDecision clear{};
  const MotionCostStopDecision blocked{
    true, false, true, false, "route_recovery_predicted_lanelet_footprint_cost"};
  const auto selected = continueRouteRecoveryCandidate(
    command(0.3), RouteRecoveryCandidateKind::kCrabLeft,
    0.10, blocked, clear, clear);

  EXPECT_FALSE(selected.available());
  EXPECT_NE(selected.reason.find("latched_crab_left_blocked"), std::string::npos);
}

TEST(MotionCostStop, RouteRecoveryProbeFailsClosedOnStaleLaneletEvidence)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.stale_timeout_s = 1.0;
  MotionCostStop cost_stop(config);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 5.0});

  const auto missing = cost_stop.evaluateLaneletRecovery(command(0.2), 5.0, 0.5);
  EXPECT_TRUE(missing.blocked);
  EXPECT_TRUE(missing.lanelet_violation);
  EXPECT_TRUE(missing.stale_grid);

  cost_stop.setLaneletGrid(makeGrid(), 5.0);
  EXPECT_FALSE(cost_stop.evaluateLaneletRecovery(command(0.2), 5.5, 0.5).blocked);
  const auto stale = cost_stop.evaluateLaneletRecovery(command(0.2), 6.1, 2.0);
  EXPECT_TRUE(stale.blocked);
  EXPECT_EQ(stale.reason, "lanelet_recovery_grid_stale");
}

TEST(MotionCostStop, RouteRecoveryUsesIndependentLaneletGridAge)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.stale_timeout_s = 1.0;
  config.lanelet_recovery_stale_timeout_s = 12.0;
  MotionCostStop cost_stop(config);
  cost_stop.setLaneletGrid(makeGrid(), 1.0);
  cost_stop.setMergedGrid(makeGrid(), 11.0);
  cost_stop.setPose(PlanarPose{0.0, 0.0, 0.0, "map", "test", 11.0});

  EXPECT_FALSE(cost_stop.evaluateLaneletRecovery(command(0.2), 11.0, 0.5).blocked);
  EXPECT_FALSE(
    cost_stop.evaluateRouteRecoveryCommand(command(-0.1), 11.0, 0.25, 0.5).blocked);
  EXPECT_EQ(
    cost_stop.evaluateLaneletRecovery(command(0.2), 13.1, 3.0).reason,
    "lanelet_recovery_grid_stale");
}

TEST(MotionCostStop, RouteRecoveryProbeFailsClosedOnStalePoseEvidence)
{
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

TEST(MotionCostStop, OppositeRecoveryRequiresProjectedFullFootprintClear)
{
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
  cost_stop.setLaneletGrid(makeGrid({{0.15, 0.0, 100}}), 1.0);

  ASSERT_TRUE(cost_stop.evaluate(command(-0.1), 1.0).blocked);
  EXPECT_FALSE(
    cost_stop.evaluateRouteRecoveryCommand(command(-0.1), 1.1, 0.3, 0.5).blocked);
  const auto still_on_boundary =
    cost_stop.evaluateRouteRecoveryCommand(command(-0.1), 1.2, 0.05, 0.5);
  EXPECT_TRUE(still_on_boundary.blocked);
  EXPECT_NE(
    still_on_boundary.reason.find("route_recovery_predicted_lanelet_footprint"),
    std::string::npos);
}

TEST(MotionCostStop, CrabRecoveryMovesAwayFromSideBoundaryOnly)
{
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
    cost_stop.evaluateRouteRecoveryCommand(
      command(0.0, -0.1), 1.1, 0.3, 0.5).blocked);
  const auto toward_boundary =
    cost_stop.evaluateRouteRecoveryCommand(
    command(0.0, 0.1), 1.2, 0.3, 0.5);
  EXPECT_TRUE(toward_boundary.blocked);
  EXPECT_NE(
    toward_boundary.reason.find("route_recovery_predicted_lanelet_footprint"),
    std::string::npos);
}

TEST(MotionCostStop, RouteRecoveryCommandFailsClosedWithoutFreshLaneletGrid)
{
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

TEST(MotionCostStop, RouteRecoveryCommandFailsClosedWithoutMatchingFrames)
{
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

TEST(MotionCostStop, OppositeRecoveryStillStopsForDynamicObstacle)
{
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

TEST(MotionCostStop, RotationStopsOnlyOnLiveDynamicSource)
{
  auto config = baseCostConfig();
  config.require_dynamic_source = true;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid({{0.5, 0.0, 90}}), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.0, 0.3), 0.0).blocked);
  cost_stop.setSourceGrid("lidar", makeGrid({{0.5, 0.0, 90}}), 0.1);
  EXPECT_TRUE(cost_stop.evaluate(command(0.0, 0.0, 0.3), 0.1).blocked);
}

TEST(MotionCostStop, StationaryCommandDoesNotCreateRotationLatch)
{
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

TEST(MotionCostStop, LaneletRotationPolicyChecksCurrentCellWhenDisabled)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  config.lanelet_allow_rotation = true;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  // HH_260721 - Fill the pose neighborhood so grid-edge rounding cannot hide current cost.
  const auto occupied_current_cell = makeGrid(
    {
      {-0.1, -0.1, 100}, {-0.1, 0.0, 100}, {-0.1, 0.1, 100},
      {0.0, -0.1, 100}, {0.0, 0.0, 100}, {0.0, 0.1, 100},
      {0.1, -0.1, 100}, {0.1, 0.0, 100}, {0.1, 0.1, 100},
    });
  cost_stop.setLaneletGrid(occupied_current_cell, 0.0);

  // HH_260721 - Site zero-turn may bypass lanelet cost only when explicitly configured.
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.0, 0.3), 0.0).blocked);
  config.lanelet_allow_rotation = false;
  cost_stop.setConfig(config);
  const auto blocked = cost_stop.evaluate(command(0.0, 0.0, 0.3), 0.1);
  EXPECT_TRUE(blocked.lanelet_violation) << blocked.reason;
}

TEST(MotionCostStop, LaneletFootprintBlocksWhenCenterLinkCellIsClear)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);

  // HH_260803 - robot_center_link at (0,0) is clear, but the configured front-right body
  // boundary covers this raw lanelet cost cell.
  const auto boundary_cost = makeGrid({{0.75, -0.55, 100}});
  ASSERT_EQ(MotionCostStop::sampleGridCost(boundary_cost, 0.0, 0.0), 0);
  cost_stop.setLaneletGrid(boundary_cost, 0.0);
  const auto decision = cost_stop.evaluate(command(0.2), 0.0);
  EXPECT_TRUE(decision.lanelet_violation) << decision.reason;
  EXPECT_NE(decision.reason.find("lanelet_footprint"), std::string::npos);
}

TEST(MotionCostStop, LaneletFootprintAllowsSoftBoundaryButBlocksOffLane)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_footprint_threshold = 100;
  config.lanelet_current_allow_route_reentry = false;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);

  // HH_260727 - The map uses 98 as a narrow-lane planning penalty and 100 as
  // truly off-lane. The full footprint may touch 98 but must stop on 100.
  cost_stop.setLaneletGrid(makeGrid({{0.75, -0.55, 98}}), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.0).blocked);

  cost_stop.setLaneletGrid(makeGrid({{0.75, -0.55, 100}}), 0.1);
  const auto blocked = cost_stop.evaluate(command(0.2), 0.1);
  EXPECT_TRUE(blocked.lanelet_violation) << blocked.reason;
  EXPECT_NE(blocked.reason.find("lanelet_footprint"), std::string::npos);
}

TEST(MotionCostStop, PublishedPlanningBoundaryOverridesFallbackFootprint)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  // HH_260727 - Deliberately tiny fallback; only the received planning boundary can reach x=1.4.
  config.footprint_front_m = 0.2;
  config.footprint_rear_m = 0.2;
  config.footprint_left_m = 0.2;
  config.footprint_right_m = 0.2;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setFootprintPolygonWorld(
    {{1.5, 0.6}, {1.5, -0.6}, {-0.4, -0.6}, {-0.4, 0.6}});
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  cost_stop.setLaneletGrid(makeGrid({{1.4, 0.0, 100}}), 0.0);

  const auto decision = cost_stop.evaluate(command(0.2), 0.0);
  EXPECT_TRUE(decision.lanelet_violation) << decision.reason;
}

TEST(MotionCostStop, PlanningMarginStopsBeforeMeasuredBodyTouchesOffLane)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_footprint_threshold = 100;
  config.lanelet_current_allow_route_reentry = false;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);

  // HH_260803 - Same measured chassis as before, expressed from the axle midpoint.
  constexpr double body_front = 0.75837;
  constexpr double body_rear = 0.73323;
  constexpr double body_left = 0.53505;
  constexpr double body_right = 0.53495;
  constexpr double planning_margin = 0.10;
  auto margin_only_cost = makeGrid();
  // Offset the 0.10 m raster so the measured 0.53505 m body edge and the
  // 0.63505 m planning edge occupy adjacent cells instead of quantizing into
  // the same one.
  margin_only_cost.info.origin.position.y = -4.04;
  const int margin_grid_x = static_cast<int>(
    std::floor((0.0 - margin_only_cost.info.origin.position.x) /
    margin_only_cost.info.resolution));
  const int margin_grid_y = static_cast<int>(
    std::floor((0.60 - margin_only_cost.info.origin.position.y) /
    margin_only_cost.info.resolution));
  margin_only_cost.data[
    margin_grid_y * static_cast<int>(margin_only_cost.info.width) + margin_grid_x] = 100;
  cost_stop.setLaneletGrid(margin_only_cost, 0.0);

  // HH_260801 - The lethal cell is outside the measured body but inside the
  // published planning boundary. This proves the 0.10 m margin stops motion
  // before the chassis itself reaches the off-lane cell.
  cost_stop.setFootprintPolygonWorld(
    {{body_front, body_left}, {body_front, -body_right},
      {-body_rear, -body_right}, {-body_rear, body_left}});
  EXPECT_FALSE(cost_stop.evaluate(command(0.0, 0.2), 0.0).blocked);

  cost_stop.setFootprintPolygonWorld(
    {{body_front + planning_margin, body_left + planning_margin},
      {body_front + planning_margin, -(body_right + planning_margin)},
      {-(body_rear + planning_margin), -(body_right + planning_margin)},
      {-(body_rear + planning_margin), body_left + planning_margin}});
  const auto blocked = cost_stop.evaluate(command(0.0, 0.2), 0.1);
  EXPECT_TRUE(blocked.lanelet_violation) << blocked.reason;
  EXPECT_NE(blocked.reason.find("lanelet_footprint"), std::string::npos);
}

TEST(MotionCostStop, MeasuredBodyBoundaryStopsOnOffLaneCost)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_footprint_threshold = 100;
  config.lanelet_current_allow_route_reentry = false;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);

  // HH_260801 - Isolate the measured chassis polygon from its planning margin.
  // Use the deployed 0.25 m lanelet-grid resolution: cost 100 on the body edge
  // must stop translation even when crab static-cost bypass is enabled.
  cost_stop.setFootprintPolygonWorld(
    {{0.75837, 0.53505}, {0.75837, -0.53495},
      {-0.73323, -0.53495}, {-0.73323, 0.53505}});
  cost_stop.setLaneletGrid(makeGrid({{0.0, 0.50, 100}}, 0.25), 0.0);
  const auto blocked = cost_stop.evaluate(command(0.0, 0.2), 0.0);
  EXPECT_TRUE(blocked.lanelet_violation) << blocked.reason;
  EXPECT_NE(blocked.reason.find("lanelet_footprint"), std::string::npos);
}

TEST(MotionCostStop, RotationChecksFullLaneletFootprintEvenWhenCenterRotationIsAllowed)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_allow_rotation = true;
  config.lanelet_current_allow_route_reentry = false;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  cost_stop.setLaneletGrid(makeGrid({{0.75, 0.5, 100}}), 0.0);

  const auto decision = cost_stop.evaluate(command(0.0, 0.0, 0.3), 0.0);
  EXPECT_TRUE(decision.lanelet_violation) << decision.reason;
}

TEST(MotionCostStop, ConfiguredCampsiteBypassPhasesStillCheckLaneletFootprint)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  const auto boundary_cost = makeGrid({{0.75, -0.55, 100}});

  // HH_260727 - Every configured campsite exception skips only legacy static
  // checks; it can never permit the planning footprint to cross raw lanelet cost.
  for (const auto & phase : config.campsite_static_bypass_phases) {
    SCOPED_TRACE(phase);
    auto cost_stop = makeMotionCostStop(config);
    cost_stop.setManeuverPhases("", phase);
    cost_stop.setMergedGrid(boundary_cost, 0.0);
    cost_stop.setLaneletGrid(boundary_cost, 0.0);
    const auto decision = cost_stop.evaluate(command(0.0, 0.2), 0.0);
    EXPECT_TRUE(decision.lanelet_violation) << decision.reason;
    EXPECT_NE(decision.reason.find("lanelet_footprint"), std::string::npos);
  }
}

TEST(MotionCostStop, ConfiguredDropZoneBypassPhasesStillCheckLaneletFootprint)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  const auto boundary_cost = makeGrid({{0.75, -0.55, 100}});

  // HH_260727 - Forward drop-zone departure may bypass a legacy route corridor,
  // but the full occupied boundary remains mandatory.
  for (const auto & phase : config.drop_zone_static_bypass_phases) {
    SCOPED_TRACE(phase);
    auto cost_stop = makeMotionCostStop(config);
    cost_stop.setManeuverPhases(phase, "");
    cost_stop.setMergedGrid(boundary_cost, 0.0);
    cost_stop.setLaneletGrid(boundary_cost, 0.0);
    const auto decision = cost_stop.evaluate(command(0.2), 0.0);
    EXPECT_TRUE(decision.lanelet_violation) << decision.reason;
    EXPECT_NE(decision.reason.find("lanelet_footprint"), std::string::npos);
  }
}

TEST(MotionCostStop, PureLateralAndReverseStaticBypassStillCheckLaneletFootprint)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  config.static_lateral_bypass = true;
  config.static_reverse_bypass = true;
  const auto boundary_cost = makeGrid({{0.75, -0.55, 100}});

  // HH_260727 - The merged-grid static bypass remains useful for crab/reverse
  // maneuvers, but it must not suppress the independent raw-lanelet footprint.
  for (const auto & test_command : {command(0.0, 0.2), command(-0.2)}) {
    auto cost_stop = makeMotionCostStop(config);
    cost_stop.setMergedGrid(boundary_cost, 0.0);
    cost_stop.setLaneletGrid(boundary_cost, 0.0);
    const auto decision = cost_stop.evaluate(test_command, 0.0);
    EXPECT_TRUE(decision.lanelet_violation) << decision.reason;
    EXPECT_NE(decision.reason.find("lanelet_footprint"), std::string::npos);
  }
}

TEST(MotionCostStop, ManeuverBypassStillSkipsLegacyLaneletChecks)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_footprint_enabled = false;
  config.lanelet_current_allow_route_reentry = false;
  const auto current_cost = makeGrid({{0.0, 0.0, 100}});

  // HH_260727 - Moving footprint evaluation ahead of the bypass must not
  // remove the bounded exception for the legacy base-link/corridor policy.
  for (const auto & phases :
    {std::pair<std::string, std::string>{"", "crab_in"},
      std::pair<std::string, std::string>{"exit_straight", ""}})
  {
    SCOPED_TRACE(phases.first + ":" + phases.second);
    auto cost_stop = makeMotionCostStop(config);
    cost_stop.setManeuverPhases(phases.first, phases.second);
    cost_stop.setMergedGrid(makeGrid(), 0.0);
    cost_stop.setLaneletGrid(current_cost, 0.0);
    EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.0).blocked);
  }
}

TEST(MotionCostStop, ClearAvoidancePathPassesAndBlockedPathStops)
{
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

// HH_260728 - Model the live radar cost disk, not a single occupied pixel. A
// return centered 1.0 m to the side must pass during forward travel after its
// 0.30 m inflation, while the same return remains blocking for a lateral move.
TEST(MotionCostStop, ForwardUsesNarrowBodySideCheckButManeuverKeepsWideProtection)
{
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

TEST(MotionCostStop, DynamicLatchNeedsContinuousClearWindow)
{
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

TEST(MotionCostStop, LatchedFrontObstacleIgnoresZeroAndChangedDirectionCommands)
{
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
  // after a stop, but the original FRONT corridor must remain the release probe.
  const std::vector<avg_msgs::msg::AvgTwist> changed_commands{
    command(0.0), command(-0.2), command(0.0, 0.2), command(0.0, 0.0, 0.3)};
  const std::vector<double> times{0.5, 1.5, 2.5, 3.5};
  for (std::size_t index = 0; index < times.size(); ++index) {
    cost_stop.setMergedGrid(front_obstacle, times[index]);
    cost_stop.setSourceGrid("radar", front_obstacle, times[index]);
    EXPECT_TRUE(cost_stop.evaluate(changed_commands[index], times[index]).blocked);
    EXPECT_TRUE(cost_stop.latched());
  }
}

TEST(MotionCostStop, StaleTriggerSourceCannotProveLatchClear)
{
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
  // triggered the stop has stopped publishing; absence of evidence is fail-closed.
  for (const double now_sec : {0.5, 1.5, 2.5, 3.5}) {
    cost_stop.setMergedGrid(makeGrid(), now_sec);
    EXPECT_TRUE(cost_stop.evaluate(command(0.0), now_sec).blocked);
    EXPECT_TRUE(cost_stop.latched());
  }
}

TEST(MotionCostStop, FreshTriggerSourceClearReleasesLatchAndStartsHold)
{
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

TEST(MotionCostStop, TriggerPathSnapshotSurvivesReplanWhileLatched)
{
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
  // obstacle that caused the latch; its original path probe remains authoritative.
  cost_stop.setLocalPath(makePath({{0.0, 0.0}, {0.5, 0.8}, {2.0, 0.8}}));
  cost_stop.setMergedGrid(front_obstacle, 2.5);
  cost_stop.setSourceGrid("radar", front_obstacle, 2.5);
  EXPECT_TRUE(cost_stop.evaluate(command(0.0), 2.5).blocked);
  EXPECT_TRUE(cost_stop.latched());
}

TEST(MotionCostStop, ZeroStampedOrReplayedClearGridCannotReleaseLatch)
{
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

TEST(MotionCostStop, RotationLatchKeepsTriggerGeometryAfterDynamicRetune)
{
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

TEST(MotionCostStop, MissingAndStaleMergedGridFailClosed)
{
  auto config = baseCostConfig();
  config.stale_stop_enabled = true;
  config.stale_timeout_s = 1.0;
  auto cost_stop = makeMotionCostStop(config);
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 0.0).stale_grid);
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.5).blocked);
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 1.1).stale_grid);
}

TEST(MotionCostStop, LaneletPathAllowsNearbyBoundaryButStopsPathCost)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_front_use_local_path = true;
  config.lanelet_path_width_m = 0.2;
  config.lanelet_lookahead_m = 1.0;
  // HH_260721 - This case validates normal in-lane driving, not bounded route re-entry.
  config.lanelet_current_allow_route_reentry = false;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  cost_stop.setLocalPath(makePath({{0.0, 0.0}, {2.0, 0.0}}));
  cost_stop.setLaneletGrid(makeGrid({{1.0, 0.6, 100}}), 0.0);
  EXPECT_FALSE(cost_stop.evaluate(command(0.2), 0.0).blocked);
  // HH_260721 - Use a finite blocked lanelet segment instead of a quantization-sensitive pixel.
  const auto path_blocked_lanelet_grid = makeGrid(
    {
      {0.9, -0.1, 100}, {0.9, 0.0, 100}, {0.9, 0.1, 100},
      {1.0, -0.1, 100}, {1.0, 0.0, 100}, {1.0, 0.1, 100},
      {1.1, -0.1, 100}, {1.1, 0.0, 100}, {1.1, 0.1, 100}});
  ASSERT_GE(MotionCostStop::sampleGridCost(path_blocked_lanelet_grid, 1.0, 0.0), 85);
  cost_stop.setLaneletGrid(path_blocked_lanelet_grid, 0.1);
  const auto blocked_decision = cost_stop.evaluate(command(0.2), 0.1);
  EXPECT_TRUE(blocked_decision.lanelet_violation) << blocked_decision.reason;
}

TEST(MotionCostStop, ReverseParkingIgnoresDisabledLaneletDirectionButStopsDynamicObstacle)
{
  auto config = baseCostConfig();
  config.lanelet_enabled = true;
  config.lanelet_current_allow_route_reentry = false;
  config.lanelet_check_reverse = false;
  config.require_dynamic_source = true;
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(makeGrid(), 0.0);
  // HH_260721 - Fill the pose neighborhood so floating-point cell edges cannot weaken this cost_stop.
  const auto current_lanelet_cost = makeGrid(
    {
      {-0.1, -0.1, 100}, {-0.1, 0.0, 100}, {-0.1, 0.1, 100},
      {0.0, -0.1, 100}, {0.0, 0.0, 100}, {0.0, 0.1, 100},
      {0.1, -0.1, 100}, {0.1, 0.0, 100}, {0.1, 0.1, 100},
    });
  ASSERT_GE(MotionCostStop::sampleGridCost(current_lanelet_cost, 0.0, 0.0), 85);
  cost_stop.setLaneletGrid(current_lanelet_cost, 0.0);

  // HH_260721 - Final parking may leave the lanelet, while live rear obstacles remain mandatory.
  EXPECT_FALSE(cost_stop.evaluate(command(-0.2), 0.0).blocked);
  const auto rear_obstacle = makeGrid({{-0.4, 0.0, 100}});
  cost_stop.setSourceGrid("lidar", rear_obstacle, 0.1);
  EXPECT_TRUE(cost_stop.evaluate(command(-0.2), 0.1).dynamic_obstacle);

  config.lanelet_check_reverse = true;
  cost_stop.setConfig(config);
  EXPECT_TRUE(cost_stop.evaluate(command(-0.2), 0.2).lanelet_violation);
}

TEST(MotionCostStop, ManeuverPhaseBypassesStaticButNotDynamicCost)
{
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

TEST(MotionCostStop, RotatedGridOriginSamplingIsCorrect)
{
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

TEST(MotionCostStop, LargeSubthresholdClusterIsUnavoidable)
{
  auto config = baseCostConfig();
  config.cost_stop_threshold = 95;
  config.unavoidable_threshold = 90;
  config.unavoidable_min_cells = 25;
  config.unavoidable_min_ratio = 0.20;
  auto cluster_grid = makeGrid();
  // HH_260721 - Fill cell centers directly to avoid fixture rounding gaps in connectivity.
  for (int y = 0; y < static_cast<int>(cluster_grid.info.height); ++y) {
    for (int x = 0; x < static_cast<int>(cluster_grid.info.width); ++x) {
      const double world_x = cluster_grid.info.origin.position.x +
        (x + 0.5) * cluster_grid.info.resolution;
      const double world_y = cluster_grid.info.origin.position.y +
        (y + 0.5) * cluster_grid.info.resolution;
      if (world_x >= 0.4 && world_x <= 1.6 && world_y >= -0.6 && world_y <= 0.6) {
        cluster_grid.data[y * static_cast<int>(cluster_grid.info.width) + x] = 91;
      }
    }
  }
  auto cost_stop = makeMotionCostStop(config);
  cost_stop.setMergedGrid(cluster_grid, 0.0);
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 0.0).blocked);
}

}  // namespace camrod_control
