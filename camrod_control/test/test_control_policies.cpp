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
  platform.battery_percentage = 0.05;
  policy.setPlatformState(platform);
  EXPECT_FALSE(policy.enabled(21.0, false, false));
  platform.battery_percentage = 0.50;
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
  EXPECT_TRUE(cost_stop.evaluate(command(0.2), 2.0).blocked);
  EXPECT_FALSE(cost_stop.evaluate(command(0.2), 2.2).blocked);
  EXPECT_FALSE(cost_stop.latched());
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
