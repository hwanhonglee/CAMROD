#pragma once

// HH_260819 - Select one immutable, validated road handoff target before a
// drop-zone departure starts. The policy is ROS-independent so safety bounds
// remain directly regression-testable.

#include <cmath>
#include <string>

namespace camrod_control
{

enum class DropZoneExitTargetSource
{
  kNone,
  kLaneletPose,
  kFixedDistanceFallback,
};

inline const char * dropZoneExitTargetSourceName(
  const DropZoneExitTargetSource source)
{
  switch (source) {
    case DropZoneExitTargetSource::kLaneletPose:
      return "lanelet_pose";
    case DropZoneExitTargetSource::kFixedDistanceFallback:
      return "fixed_distance_fallback";
    case DropZoneExitTargetSource::kNone:
      break;
  }
  return "none";
}

struct DropZoneExitTargetPolicyConfig
{
  bool require_lanelet_target{true};
  bool allow_fixed_distance_fallback{false};
  double pose_timeout_s{2.0};
  double minimum_distance_m{0.3};
  double maximum_distance_m{8.0};
  double minimum_forward_m{0.1};
  double maximum_lateral_m{1.0};
  double fixed_distance_m{1.2};
  bool allow_unstamped_pose_fallback{false};
};

struct DropZoneExitTargetPolicyInput
{
  double vehicle_x_m{0.0};
  double vehicle_y_m{0.0};
  double vehicle_yaw_rad{0.0};
  std::string vehicle_frame_id;
  double vehicle_age_s{0.0};
  bool vehicle_header_stamp_available{false};
  double vehicle_header_stamp_age_s{0.0};

  bool lanelet_pose_available{false};
  double lanelet_x_m{0.0};
  double lanelet_y_m{0.0};
  double lanelet_yaw_rad{0.0};
  std::string lanelet_frame_id;
  double lanelet_age_s{0.0};
  bool lanelet_header_stamp_available{false};
  double lanelet_header_stamp_age_s{0.0};
};

struct DropZoneExitTargetSelection
{
  bool valid{false};
  DropZoneExitTargetSource source{DropZoneExitTargetSource::kNone};
  double target_x_m{0.0};
  double target_y_m{0.0};
  double target_yaw_rad{0.0};
  double initial_distance_m{0.0};
  double initial_forward_m{0.0};
  double initial_lateral_m{0.0};
  std::string detail;
};

inline bool dropZoneExitAgeIsFresh(const double age_s, const double timeout_s)
{
  return std::isfinite(age_s) && std::isfinite(timeout_s) &&
         age_s >= 0.0 && timeout_s > 0.0 && age_s <= timeout_s;
}

inline bool dropZoneExitPoseSampleIsFresh(
  const double receipt_age_s, const bool header_stamp_available,
  const double header_stamp_age_s, const bool allow_unstamped_fallback,
  const double timeout_s)
{
  if (!dropZoneExitAgeIsFresh(receipt_age_s, timeout_s)) {
    return false;
  }
  if (!header_stamp_available) {
    return allow_unstamped_fallback;
  }
  return dropZoneExitAgeIsFresh(header_stamp_age_s, timeout_s);
}

inline bool dropZoneExitTargetConfigIsValid(
  const DropZoneExitTargetPolicyConfig & config)
{
  return std::isfinite(config.pose_timeout_s) && config.pose_timeout_s > 0.0 &&
         std::isfinite(config.minimum_distance_m) &&
         config.minimum_distance_m >= 0.0 &&
         std::isfinite(config.maximum_distance_m) &&
         config.maximum_distance_m >= config.minimum_distance_m &&
         std::isfinite(config.minimum_forward_m) &&
         config.minimum_forward_m >= 0.0 &&
         std::isfinite(config.maximum_lateral_m) &&
         config.maximum_lateral_m >= 0.0 &&
         std::isfinite(config.fixed_distance_m) &&
         config.fixed_distance_m >= config.minimum_distance_m &&
         config.fixed_distance_m <= config.maximum_distance_m &&
         config.fixed_distance_m > config.minimum_forward_m;
}

inline DropZoneExitTargetSelection makeDropZoneExitTargetSelection(
  const DropZoneExitTargetPolicyConfig & config,
  const DropZoneExitTargetPolicyInput & input)
{
  DropZoneExitTargetSelection selection;
  if (!dropZoneExitTargetConfigIsValid(config)) {
    selection.detail = "invalid exit-target policy configuration";
    return selection;
  }
  if (!std::isfinite(input.vehicle_x_m) ||
      !std::isfinite(input.vehicle_y_m) ||
      !std::isfinite(input.vehicle_yaw_rad)) {
    selection.detail = "vehicle pose contains non-finite values";
    return selection;
  }
  if (!dropZoneExitPoseSampleIsFresh(
      input.vehicle_age_s, input.vehicle_header_stamp_available,
      input.vehicle_header_stamp_age_s,
      config.allow_unstamped_pose_fallback, config.pose_timeout_s)) {
    selection.detail = "vehicle pose is stale";
    return selection;
  }
  if (input.vehicle_frame_id.empty()) {
    selection.detail = "vehicle pose frame is empty";
    return selection;
  }
  std::string lanelet_rejection;
  if (!input.lanelet_pose_available) {
    lanelet_rejection = "lanelet pose unavailable";
  } else if (!dropZoneExitPoseSampleIsFresh(
      input.lanelet_age_s, input.lanelet_header_stamp_available,
      input.lanelet_header_stamp_age_s,
      config.allow_unstamped_pose_fallback, config.pose_timeout_s)) {
    lanelet_rejection = "lanelet pose is stale";
  } else if (input.lanelet_frame_id.empty()) {
    lanelet_rejection = "lanelet pose frame is empty";
  } else if (input.vehicle_frame_id != input.lanelet_frame_id)
  {
    lanelet_rejection = "vehicle/lanelet pose frame mismatch";
  } else if (
    !std::isfinite(input.lanelet_x_m) ||
    !std::isfinite(input.lanelet_y_m) ||
    !std::isfinite(input.lanelet_yaw_rad))
  {
    lanelet_rejection = "lanelet pose contains non-finite values";
  } else {
    const double dx = input.lanelet_x_m - input.vehicle_x_m;
    const double dy = input.lanelet_y_m - input.vehicle_y_m;
    const double cosine = std::cos(input.vehicle_yaw_rad);
    const double sine = std::sin(input.vehicle_yaw_rad);
    selection.initial_distance_m = std::hypot(dx, dy);
    selection.initial_forward_m = cosine * dx + sine * dy;
    selection.initial_lateral_m = -sine * dx + cosine * dy;
    if (selection.initial_distance_m < config.minimum_distance_m) {
      lanelet_rejection = "lanelet target is closer than minimum distance";
    } else if (selection.initial_distance_m > config.maximum_distance_m) {
      lanelet_rejection = "lanelet target exceeds maximum distance";
    } else if (selection.initial_forward_m <= config.minimum_forward_m) {
      lanelet_rejection = "lanelet target is not safely ahead";
    } else if (
      std::abs(selection.initial_lateral_m) > config.maximum_lateral_m)
    {
      lanelet_rejection = "lanelet target exceeds maximum lateral offset";
    } else {
      selection.valid = true;
      selection.source = DropZoneExitTargetSource::kLaneletPose;
      selection.target_x_m = input.lanelet_x_m;
      selection.target_y_m = input.lanelet_y_m;
      selection.target_yaw_rad = input.lanelet_yaw_rad;
      selection.detail = "captured fresh lanelet pose";
      return selection;
    }
  }

  if (config.require_lanelet_target ||
      !config.allow_fixed_distance_fallback)
  {
    selection.valid = false;
    selection.source = DropZoneExitTargetSource::kNone;
    selection.detail = lanelet_rejection;
    return selection;
  }

  selection.valid = true;
  selection.source = DropZoneExitTargetSource::kFixedDistanceFallback;
  selection.target_x_m = input.vehicle_x_m +
    std::cos(input.vehicle_yaw_rad) * config.fixed_distance_m;
  selection.target_y_m = input.vehicle_y_m +
    std::sin(input.vehicle_yaw_rad) * config.fixed_distance_m;
  selection.target_yaw_rad = input.vehicle_yaw_rad;
  selection.initial_distance_m = config.fixed_distance_m;
  selection.initial_forward_m = config.fixed_distance_m;
  selection.initial_lateral_m = 0.0;
  selection.detail = "fallback after: " + lanelet_rejection;
  return selection;
}

inline double dropZoneExitTargetError(
  const DropZoneExitTargetSelection & target,
  const double current_x_m,
  const double current_y_m)
{
  return std::hypot(
    target.target_x_m - current_x_m,
    target.target_y_m - current_y_m);
}

inline bool dropZoneExitTranslationComplete(
  const DropZoneExitTargetSelection & target,
  const double current_x_m,
  const double current_y_m,
  const double position_tolerance_m)
{
  return target.valid && std::isfinite(position_tolerance_m) &&
         position_tolerance_m >= 0.0 &&
         dropZoneExitTargetError(target, current_x_m, current_y_m) <=
         position_tolerance_m;
}

inline double dropZoneExitBearingError(
  const DropZoneExitTargetSelection & target,
  const double current_x_m,
  const double current_y_m,
  const double current_yaw_rad)
{
  const double bearing = std::atan2(
    target.target_y_m - current_y_m,
    target.target_x_m - current_x_m);
  const double raw_error = bearing - current_yaw_rad;
  return std::atan2(std::sin(raw_error), std::cos(raw_error));
}

}  // namespace camrod_control
