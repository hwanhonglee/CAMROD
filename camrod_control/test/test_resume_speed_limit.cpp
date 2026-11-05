#include <gtest/gtest.h>

#include <cmath>

#include "camrod_control/resume_speed_limit.hpp"

using camrod_control::PlanarVelocity;
using camrod_control::ResumeSpeedLimit;
using camrod_control::ResumeSpeedLimitConfig;

// HJ_261105 - Post-stop resume cap: window, curvature, and rotation behavior.
namespace {
ResumeSpeedLimit makeLimit() {
  ResumeSpeedLimitConfig config;
  config.enabled = true;
  config.duration_s = 4.0;
  config.max_linear_mps = 0.2;
  return ResumeSpeedLimit(config);
}
}  // namespace

TEST(ResumeSpeedLimit, NoStopMeansNoLimit) {
  auto limit = makeLimit();
  const auto out = limit.apply({0.55, 0.0, 0.1}, 10.0);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.55);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.1);
  EXPECT_FALSE(limit.active(10.0));
}

TEST(ResumeSpeedLimit, CapsForDurationFromFirstMovingCommand) {
  auto limit = makeLimit();
  limit.onStopHeld();
  // Zero commands (e.g. resume voice wait) do not start the window.
  limit.apply({0.0, 0.0, 0.0}, 10.0);
  EXPECT_FALSE(limit.active(12.0));
  auto out = limit.apply({0.55, 0.0, 0.0}, 13.0);
  EXPECT_NEAR(out.linear_x, 0.2, 1e-9);
  out = limit.apply({0.55, 0.0, 0.0}, 16.9);
  EXPECT_NEAR(out.linear_x, 0.2, 1e-9);
  out = limit.apply({0.55, 0.0, 0.0}, 17.0);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.55);
}

TEST(ResumeSpeedLimit, PreservesCurvatureAndDirection) {
  auto limit = makeLimit();
  limit.onStopHeld();
  const PlanarVelocity in{-0.3, 0.4, 0.5};
  const auto out = limit.apply(in, 1.0);
  EXPECT_NEAR(std::hypot(out.linear_x, out.linear_y), 0.2, 1e-9);
  EXPECT_NEAR(out.linear_x / out.linear_y, in.linear_x / in.linear_y, 1e-9);
  EXPECT_NEAR(out.angular_z / std::hypot(out.linear_x, out.linear_y),
              in.angular_z / 0.5, 1e-9);
}

TEST(ResumeSpeedLimit, SlowCommandsAndRotationPassThrough) {
  auto limit = makeLimit();
  limit.onStopHeld();
  auto out = limit.apply({0.0, 0.0, 0.8}, 1.0);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.8);
  EXPECT_FALSE(limit.active(1.0));
  out = limit.apply({0.15, 0.0, 0.3}, 1.1);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.15);
  EXPECT_DOUBLE_EQ(out.angular_z, 0.3);
  EXPECT_TRUE(limit.active(1.1));
}

TEST(ResumeSpeedLimit, NewStopRestartsWindow) {
  auto limit = makeLimit();
  limit.onStopHeld();
  limit.apply({0.55, 0.0, 0.0}, 1.0);
  limit.onStopHeld();
  EXPECT_FALSE(limit.active(2.0));
  auto out = limit.apply({0.55, 0.0, 0.0}, 3.0);
  EXPECT_NEAR(out.linear_x, 0.2, 1e-9);
  out = limit.apply({0.55, 0.0, 0.0}, 6.5);
  EXPECT_NEAR(out.linear_x, 0.2, 1e-9);
}

TEST(ResumeSpeedLimit, DisabledPassesThrough) {
  ResumeSpeedLimitConfig config;
  config.enabled = false;
  ResumeSpeedLimit limit(config);
  limit.onStopHeld();
  const auto out = limit.apply({0.55, 0.0, 0.0}, 1.0);
  EXPECT_DOUBLE_EQ(out.linear_x, 0.55);
}
