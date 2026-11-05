#include <gtest/gtest.h>
#include <limits>
#include "camrod_sensing/front_lidar_guard.hpp"

// HH_261102 - Compare exact body-clearance boundaries without confusing the forward-mounted LiDAR with the nose.
TEST(FrontLidarGuard, IncludesTenTwentyThirtyCentimetersButNotForty)
{
  const camrod::sensing::FrontLidarGuardConfig config;
  for (const double clearance : {0.10, 0.20, 0.30}) {
    EXPECT_TRUE(camrod::sensing::isFrontLidarGuardPoint(
      static_cast<float>(config.x_min_m + clearance), 0.0, 0.1, config));
  }
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(config.x_min_m + 0.40, 0.0, 0.1, config));
}

TEST(FrontLidarGuard, RejectsGroundBodySideHighAndInvalidPoints)
{
  const camrod::sensing::FrontLidarGuardConfig config;
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(config.x_min_m, 0.0, 0.1, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(0.6, 0.0, 0.1, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(0.9, 0.0, config.z_min_m - 0.05, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(0.9, 0.0, config.z_min_m, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(0.9, config.y_max_m + 0.01, 0.1, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(0.9, 0.0, config.z_max_m + 0.01, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.1, config));
  EXPECT_TRUE(camrod::sensing::isFrontLidarGuardPoint(0.9, config.y_min_m, 0.1, config));
}

// HJ_261105 - Every axis is bounded only by its own configured min/max.
TEST(FrontLidarGuard, CustomBoxBoundsEachAxis)
{
  camrod::sensing::FrontLidarGuardConfig config;
  config.x_min_m = 0.8;
  config.x_max_m = 1.3;
  config.y_min_m = -0.2;
  config.y_max_m = 0.4;
  config.z_min_m = 0.0;
  config.z_max_m = 0.5;
  EXPECT_TRUE(camrod::sensing::validFrontLidarGuardConfig(config));
  EXPECT_TRUE(camrod::sensing::isFrontLidarGuardPoint(1.25, 0.35, 0.45, config));
  EXPECT_TRUE(camrod::sensing::isFrontLidarGuardPoint(1.3, -0.2, 0.5, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(1.31, 0.0, 0.2, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(1.0, -0.21, 0.2, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(1.0, 0.41, 0.2, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(1.0, 0.0, 0.51, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(0.8, 0.0, 0.2, config));
}

TEST(FrontLidarGuard, RejectsInvertedOrNonFiniteBox)
{
  camrod::sensing::FrontLidarGuardConfig config;
  EXPECT_TRUE(camrod::sensing::validFrontLidarGuardConfig(config));
  config.x_max_m = config.x_min_m;
  EXPECT_FALSE(camrod::sensing::validFrontLidarGuardConfig(config));
  config = {};
  config.y_min_m = config.y_max_m + 0.1;
  EXPECT_FALSE(camrod::sensing::validFrontLidarGuardConfig(config));
  config = {};
  config.z_max_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(camrod::sensing::validFrontLidarGuardConfig(config));
  config = {};
  config.x_min_m = 0.0;
  EXPECT_FALSE(camrod::sensing::validFrontLidarGuardConfig(config));
}
