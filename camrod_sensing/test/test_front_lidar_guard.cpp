#include <gtest/gtest.h>
#include <limits>
#include "camrod_sensing/front_lidar_guard.hpp"

// HH_261102 - Compare exact body-clearance boundaries without confusing the forward-mounted LiDAR with the nose.
TEST(FrontLidarGuard, IncludesTenTwentyThirtyCentimetersButNotForty)
{
  const camrod::sensing::FrontLidarGuardConfig config;
  for (const double clearance : {0.10, 0.20, 0.30}) {
    EXPECT_TRUE(camrod::sensing::isFrontLidarGuardPoint(
      static_cast<float>(config.body_front_m + clearance), 0.0, 0.1, config));
  }
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(config.body_front_m + 0.40, 0.0, 0.1, config));
}

TEST(FrontLidarGuard, RejectsGroundBodySideHighAndInvalidPoints)
{
  const camrod::sensing::FrontLidarGuardConfig config;
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(config.body_front_m, 0.0, 0.1, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(0.6, 0.0, 0.1, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(0.9, 0.0, config.body_bottom_m, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(0.9, 0.0,
    config.body_bottom_m + config.ground_clearance_m, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(0.9, config.body_left_m + 0.01, 0.1, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(0.9, 0.0, config.body_top_m + 0.01, config));
  EXPECT_FALSE(camrod::sensing::isFrontLidarGuardPoint(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.1, config));
  EXPECT_TRUE(camrod::sensing::isFrontLidarGuardPoint(0.9, -config.body_right_m, 0.1, config));
}
