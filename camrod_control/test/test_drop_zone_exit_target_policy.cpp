// HH_260819 - Keep drop-zone road-handoff selection independent of a live ROS graph.

#include <cmath>

#include "camrod_control/drop_zone_exit_target_policy.hpp"
#include "gtest/gtest.h"

namespace camrod_control
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

DropZoneExitTargetPolicyInput activeMapInput()
{
  // `centerline_snapper_node.cpp::findNearestCenterline` publishes the
  // directed centerline segment atan2(vy, vx), not the bearing from the robot
  // to the projection. In the active OSM this is lanelet 67, segment 4:
  // projection bearing=93.9753 deg, published road yaw=3.9753 deg.
  DropZoneExitTargetPolicyInput input;
  input.vehicle_x_m = -14.2347;
  input.vehicle_y_m = 39.7863;
  input.vehicle_yaw_rad = 97.7873 * kPi / 180.0;
  input.vehicle_frame_id = "map";
  input.vehicle_age_s = 0.05;
  input.vehicle_header_stamp_available = true;
  input.vehicle_header_stamp_age_s = 0.05;
  input.lanelet_pose_available = true;
  input.lanelet_x_m = -14.4918219;
  input.lanelet_y_m = 43.4862681;
  input.lanelet_yaw_rad = 3.9753 * kPi / 180.0;
  input.lanelet_frame_id = "map";
  input.lanelet_age_s = 0.04;
  input.lanelet_header_stamp_available = true;
  input.lanelet_header_stamp_age_s = 0.04;
  return input;
}

TEST(DropZoneExitTargetPolicy, CapturesActiveMapLaneletRoadHandoff)
{
  const auto input = activeMapInput();
  const auto selection = makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, input);

  ASSERT_TRUE(selection.valid) << selection.detail;
  EXPECT_EQ(selection.source, DropZoneExitTargetSource::kLaneletPose);
  EXPECT_NEAR(selection.initial_distance_m, 3.708891, 1.0e-5);
  EXPECT_NEAR(selection.initial_forward_m, 3.700686, 1.0e-5);
  EXPECT_NEAR(selection.initial_lateral_m, -0.24658, 1.0e-5);
  EXPECT_DOUBLE_EQ(selection.target_x_m, -14.4918219);
  EXPECT_DOUBLE_EQ(selection.target_y_m, 43.4862681);
  EXPECT_NEAR(
    dropZoneExitBearingError(
      selection, input.vehicle_x_m, input.vehicle_y_m,
      input.vehicle_yaw_rad) * 180.0 / kPi,
    -3.8120, 1.0e-3);
  EXPECT_NEAR(selection.target_yaw_rad * 180.0 / kPi, 3.9753, 1.0e-9);
}

TEST(DropZoneExitTargetPolicy, UsesTargetErrorInsteadOfFixedTravelDistance)
{
  const auto input = activeMapInput();
  const auto selection = makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, input);
  ASSERT_TRUE(selection.valid);

  const double one_point_two_x = input.vehicle_x_m +
    std::cos(input.vehicle_yaw_rad) * 1.2;
  const double one_point_two_y = input.vehicle_y_m +
    std::sin(input.vehicle_yaw_rad) * 1.2;
  EXPECT_FALSE(dropZoneExitTranslationComplete(
    selection, one_point_two_x, one_point_two_y, 0.20));
  EXPECT_GT(
    dropZoneExitTargetError(selection, one_point_two_x, one_point_two_y), 2.5);

  EXPECT_TRUE(dropZoneExitTranslationComplete(
    selection, selection.target_x_m + 0.10, selection.target_y_m, 0.20));
  EXPECT_FALSE(dropZoneExitTranslationComplete(
    selection, selection.target_x_m + 0.21, selection.target_y_m, 0.20));
}

TEST(DropZoneExitTargetPolicy, RejectsStaleOrMismatchedLaneletPoseByDefault)
{
  auto stale_vehicle = activeMapInput();
  stale_vehicle.vehicle_age_s = 2.01;
  const auto stale_vehicle_selection = makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, stale_vehicle);
  EXPECT_FALSE(stale_vehicle_selection.valid);
  EXPECT_EQ(stale_vehicle_selection.detail, "vehicle pose is stale");

  auto stale = activeMapInput();
  stale.lanelet_age_s = 2.01;
  const auto stale_selection = makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, stale);
  EXPECT_FALSE(stale_selection.valid);
  EXPECT_EQ(stale_selection.detail, "lanelet pose is stale");

  auto mismatched = activeMapInput();
  mismatched.lanelet_frame_id = "odom";
  const auto mismatched_selection = makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, mismatched);
  EXPECT_FALSE(mismatched_selection.valid);
  EXPECT_EQ(
    mismatched_selection.detail, "vehicle/lanelet pose frame mismatch");
}

TEST(DropZoneExitTargetPolicy, RejectsEmptyFramesAndStaleSourceStamps)
{
  auto empty_vehicle_frame = activeMapInput();
  empty_vehicle_frame.vehicle_frame_id.clear();
  auto selection = makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, empty_vehicle_frame);
  EXPECT_FALSE(selection.valid);
  EXPECT_EQ(selection.detail, "vehicle pose frame is empty");

  auto empty_lanelet_frame = activeMapInput();
  empty_lanelet_frame.lanelet_frame_id.clear();
  selection = makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, empty_lanelet_frame);
  EXPECT_FALSE(selection.valid);
  EXPECT_EQ(selection.detail, "lanelet pose frame is empty");

  auto stale_vehicle_header = activeMapInput();
  stale_vehicle_header.vehicle_header_stamp_available = true;
  stale_vehicle_header.vehicle_header_stamp_age_s = 2.01;
  selection = makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, stale_vehicle_header);
  EXPECT_FALSE(selection.valid);
  EXPECT_EQ(selection.detail, "vehicle pose is stale");

  auto stale_lanelet_header = activeMapInput();
  stale_lanelet_header.lanelet_header_stamp_available = true;
  stale_lanelet_header.lanelet_header_stamp_age_s = 2.01;
  selection = makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, stale_lanelet_header);
  EXPECT_FALSE(selection.valid);
  EXPECT_EQ(selection.detail, "lanelet pose is stale");

  // Existing zero-stamp simulation fixtures remain compatible through the
  // separately measured callback receipt age.
  auto zero_stamp = activeMapInput();
  zero_stamp.vehicle_header_stamp_available = false;
  zero_stamp.lanelet_header_stamp_available = false;
  EXPECT_FALSE(zero_stamp.vehicle_header_stamp_available);
  EXPECT_FALSE(zero_stamp.lanelet_header_stamp_available);
  EXPECT_FALSE(makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, zero_stamp).valid);

  auto simulation_fallback = DropZoneExitTargetPolicyConfig{};
  simulation_fallback.allow_unstamped_pose_fallback = true;
  EXPECT_TRUE(makeDropZoneExitTargetSelection(
    simulation_fallback, zero_stamp).valid);
}

TEST(DropZoneExitTargetPolicy, EnforcesDistanceForwardAndLateralSafetyBounds)
{
  auto input = activeMapInput();

  input.lanelet_x_m = input.vehicle_x_m + 0.10 *
    std::cos(input.vehicle_yaw_rad);
  input.lanelet_y_m = input.vehicle_y_m + 0.10 *
    std::sin(input.vehicle_yaw_rad);
  EXPECT_FALSE(makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, input).valid);

  input = activeMapInput();
  input.lanelet_x_m = input.vehicle_x_m - 1.0 *
    std::cos(input.vehicle_yaw_rad);
  input.lanelet_y_m = input.vehicle_y_m - 1.0 *
    std::sin(input.vehicle_yaw_rad);
  EXPECT_FALSE(makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, input).valid);

  input = activeMapInput();
  const double lateral_x = -std::sin(input.vehicle_yaw_rad) * 1.01;
  const double lateral_y = std::cos(input.vehicle_yaw_rad) * 1.01;
  input.lanelet_x_m = input.vehicle_x_m +
    std::cos(input.vehicle_yaw_rad) + lateral_x;
  input.lanelet_y_m = input.vehicle_y_m +
    std::sin(input.vehicle_yaw_rad) + lateral_y;
  EXPECT_FALSE(makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, input).valid);

  input = activeMapInput();
  input.lanelet_x_m = input.vehicle_x_m + 8.01 *
    std::cos(input.vehicle_yaw_rad);
  input.lanelet_y_m = input.vehicle_y_m + 8.01 *
    std::sin(input.vehicle_yaw_rad);
  EXPECT_FALSE(makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, input).valid);
}

TEST(DropZoneExitTargetPolicy, FixedDistanceFallbackRequiresExplicitOptIn)
{
  auto input = activeMapInput();
  input.lanelet_pose_available = false;

  DropZoneExitTargetPolicyConfig production;
  EXPECT_FALSE(makeDropZoneExitTargetSelection(production, input).valid);

  DropZoneExitTargetPolicyConfig maintenance;
  maintenance.require_lanelet_target = false;
  maintenance.allow_fixed_distance_fallback = true;
  const auto fallback = makeDropZoneExitTargetSelection(maintenance, input);
  ASSERT_TRUE(fallback.valid) << fallback.detail;
  EXPECT_EQ(
    fallback.source, DropZoneExitTargetSource::kFixedDistanceFallback);
  EXPECT_NEAR(fallback.initial_distance_m, 1.2, 1.0e-12);
  EXPECT_NEAR(fallback.initial_forward_m, 1.2, 1.0e-12);
  EXPECT_NEAR(fallback.initial_lateral_m, 0.0, 1.0e-12);
}

TEST(DropZoneExitTargetPolicy, CapturedTargetDoesNotChaseLaterLaneletSamples)
{
  const auto captured = makeDropZoneExitTargetSelection(
    DropZoneExitTargetPolicyConfig{}, activeMapInput());
  ASSERT_TRUE(captured.valid);

  auto later_sample = activeMapInput();
  later_sample.lanelet_x_m += 5.0;
  later_sample.lanelet_y_m -= 2.0;

  EXPECT_DOUBLE_EQ(captured.target_x_m, -14.4918219);
  EXPECT_DOUBLE_EQ(captured.target_y_m, 43.4862681);
  EXPECT_NE(captured.target_x_m, later_sample.lanelet_x_m);
  EXPECT_NE(captured.target_y_m, later_sample.lanelet_y_m);
}

}  // namespace
}  // namespace camrod_control
