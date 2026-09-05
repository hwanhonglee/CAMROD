#include <cmath>

#include <gtest/gtest.h>

#include "camrod_control/drop_zone_parking_approach.hpp"

namespace
{

using camrod_control::DropZoneParkingApproachConfig;
using camrod_control::makeDropZoneParkingApproachCommand;

TEST(DropZoneParkingApproach, ConvertsExactMapTargetIntoBodyVector)
{
  DropZoneParkingApproachConfig config;
  config.proportional_gain = 1.0;
  config.minimum_speed_mps = 0.02;
  config.maximum_speed_mps = 0.20;
  config.maximum_correction_m = 0.75;

  // At +90 degrees body yaw, map +Y is body-forward and map -X is body-right.
  const auto command = makeDropZoneParkingApproachCommand(
    config, 1.0, 2.0, M_PI_2, 0.9, 2.3);
  ASSERT_TRUE(command.valid);
  EXPECT_FALSE(command.reached);
  EXPECT_NEAR(command.distance_m, std::hypot(0.1, 0.3), 1.0e-9);
  EXPECT_NEAR(command.body_forward_error_m, 0.3, 1.0e-9);
  EXPECT_NEAR(command.body_lateral_error_m, 0.1, 1.0e-9);
  EXPECT_NEAR(std::hypot(command.linear_x_mps, command.linear_y_mps), 0.20,
              1.0e-9);
  EXPECT_GT(command.linear_x_mps, 0.0);
  EXPECT_GT(command.linear_y_mps, 0.0);
}

TEST(DropZoneParkingApproach, StopsInsideFiveCentimeterTolerance)
{
  DropZoneParkingApproachConfig config;
  config.position_tolerance_m = 0.05;
  const auto command = makeDropZoneParkingApproachCommand(
    config, 0.0, 0.0, 0.0, 0.03, 0.04);
  ASSERT_TRUE(command.valid);
  EXPECT_TRUE(command.reached);
  EXPECT_DOUBLE_EQ(command.linear_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(command.linear_y_mps, 0.0);
}

TEST(DropZoneParkingApproach, RejectsUnboundedOrNonFiniteCorrection)
{
  DropZoneParkingApproachConfig config;
  config.maximum_correction_m = 0.75;
  EXPECT_FALSE(makeDropZoneParkingApproachCommand(
    config, 0.0, 0.0, 0.0, 0.751, 0.0).valid);
  EXPECT_FALSE(makeDropZoneParkingApproachCommand(
    config, 0.0, 0.0, 0.0, NAN, 0.0).valid);
}

}  // namespace
