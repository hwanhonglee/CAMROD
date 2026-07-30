#pragma once

#include <cstdint>

namespace camrod_localization
{

// HH_260730 - The input adapter publishes odometry before the matching pose
// covariance.  Select by header stamp instead of callback order so the selector
// never republishes the previous 20 Hz pose while marking the new odometry
// stamp as consumed.
enum class PosePayload
{
  kNone,
  kPoseCovariance,
  kOdometry,
};

constexpr PosePayload selectFreshestPosePayload(
  const bool has_pose_covariance,
  const std::int64_t pose_covariance_stamp_ns,
  const bool has_odometry,
  const std::int64_t odometry_stamp_ns)
{
  if (has_pose_covariance &&
    (!has_odometry || pose_covariance_stamp_ns >= odometry_stamp_ns))
  {
    return PosePayload::kPoseCovariance;
  }
  if (has_odometry) {
    return PosePayload::kOdometry;
  }
  return PosePayload::kNone;
}

}  // namespace camrod_localization
