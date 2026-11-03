// HH_261102 - Verify the authored station and continuous body containment before native cargo movement.
#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "camrod_control/cargo_access_target.hpp"

namespace
{
using camrod_control::DropZoneExitTargetPolicyConfig;
using camrod_control::DropZoneExitTargetPolicyInput;
using camrod_control::DropZoneExitTargetSource;
using camrod_control::cargoAccessSweepInsideZone;
using camrod_control::makeCargoAccessTargetSelection;

const std::vector<std::pair<double, double>> kZone{{-3.0, -2.0}, {3.0, -2.0}, {3.0, 2.0}, {-3.0, 2.0}};

DropZoneExitTargetPolicyInput freshPose()
{
  DropZoneExitTargetPolicyInput input;
  input.vehicle_frame_id = "map";
  input.vehicle_header_stamp_available = true;
  input.vehicle_header_stamp_age_s = 0.01;
  input.vehicle_age_s = 0.01;
  return input;
}

TEST(CargoAccessTarget, UsesOnlyAuthoredStationAndInitialYawNotLiveRoadProjection)
{
  auto input = freshPose();
  input.lanelet_pose_available = true;
  input.lanelet_x_m = 6.0;
  input.lanelet_yaw_rad = 1.57;
  const auto target = makeCargoAccessTargetSelection({}, input, 1.0, 0.0, kZone, 0.9);
  ASSERT_TRUE(target.valid) << target.detail;
  EXPECT_EQ(target.source, DropZoneExitTargetSource::kStationPose);
  EXPECT_DOUBLE_EQ(target.target_x_m, 1.0);
  EXPECT_DOUBLE_EQ(target.target_y_m, 0.0);
  EXPECT_DOUBLE_EQ(target.target_yaw_rad, 0.0);
}

TEST(CargoAccessTarget, RejectsSameBehindFarOrLargeLateralAnchor)
{
  const auto input = freshPose();
  for (const auto &point : std::vector<std::pair<double, double>>{{0.0, 0.0}, {-1.0, 0.0}, {9.0, 0.0}, {1.0, 1.01}}) {
    EXPECT_FALSE(makeCargoAccessTargetSelection({}, input, point.first, point.second, kZone, 0.9).valid);
  }
}

TEST(CargoAccessTarget, RejectsUnknownFrameStaleAndUnstampedEvenWhenSimFallbackEnabled)
{
  DropZoneExitTargetPolicyConfig config;
  config.allow_unstamped_pose_fallback = true;
  auto input = freshPose();
  input.vehicle_frame_id = "odom";
  EXPECT_FALSE(makeCargoAccessTargetSelection(config, input, 1.0, 0.0, kZone, 0.9).valid);
  input.vehicle_frame_id = "map";
  input.vehicle_header_stamp_available = false;
  EXPECT_FALSE(makeCargoAccessTargetSelection(config, input, 1.0, 0.0, kZone, 0.9).valid);
  input = freshPose();
  input.vehicle_header_stamp_age_s = 3.0;
  EXPECT_FALSE(makeCargoAccessTargetSelection(config, input, 1.0, 0.0, kZone, 0.9).valid);
}

TEST(CargoAccessTarget, ConvexEndpointProofCoversAllIntermediateBodyDisks)
{
  EXPECT_TRUE(cargoAccessSweepInsideZone(kZone, -1.0, 0.0, 1.0, 0.0, 0.9));
  auto clockwise = kZone;
  std::reverse(clockwise.begin(), clockwise.end());
  EXPECT_TRUE(cargoAccessSweepInsideZone(clockwise, -1.0, 0.0, 1.0, 0.0, 0.9));
  for (int step = 0; step <= 100; ++step) {
    const double x = -1.0 + 2.0 * step / 100.0;
    EXPECT_TRUE(cargoAccessSweepInsideZone(kZone, x, 0.0, 1.0, 0.0, 0.9));
  }
}

TEST(CargoAccessTarget, CenterInsideIsNotEnoughWhenFullBodyTouchesOrLeavesZone)
{
  EXPECT_FALSE(cargoAccessSweepInsideZone(kZone, 0.0, 1.2, 1.0, 0.0, 0.9));
  EXPECT_FALSE(cargoAccessSweepInsideZone(kZone, 0.0, 0.0, 2.2, 0.0, 0.9));
  EXPECT_FALSE(cargoAccessSweepInsideZone(kZone, 0.0, 0.0, 1.0, 0.0, 0.0));
  EXPECT_FALSE(cargoAccessSweepInsideZone(kZone, 0.0, 0.0, 1.0, 0.0, NAN));
}

TEST(CargoAccessTarget, RejectsConcaveSelfCrossedDegenerateAndMissingZones)
{
  EXPECT_FALSE(cargoAccessSweepInsideZone({}, 0.0, 0.0, 1.0, 0.0, 0.1));
  EXPECT_FALSE(cargoAccessSweepInsideZone({{0, 0}, {1, 0}, {2, 0}}, 0.0, 0.0, 1.0, 0.0, 0.1));
  EXPECT_FALSE(cargoAccessSweepInsideZone({{-3, -2}, {3, -2}, {0, 0}, {3, 2}, {-3, 2}}, 0.0, 0.0, 1.0, 0.0, 0.1));
  EXPECT_FALSE(cargoAccessSweepInsideZone({{-3, -2}, {3, 2}, {3, -2}, {-3, 2}}, 0.0, 0.0, 1.0, 0.0, 0.1));
}

TEST(CargoAccessTarget, OnlyTheExactBackendSourceSelectsStationDeparture)
{
  EXPECT_TRUE(camrod_control::isCargoAccessExitSource("http:charging_access:leave:site_departure"));
  EXPECT_FALSE(camrod_control::isCargoAccessExitSource("http_ui_destination:site_departure"));
  EXPECT_FALSE(camrod_control::isCargoAccessExitSource("prefix:http:charging_access:leave:site_departure"));
  EXPECT_FALSE(camrod_control::isCargoAccessExitSource("http:charging_access:leave:site_departure:extra"));
}
}  // namespace
