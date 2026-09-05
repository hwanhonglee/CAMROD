#pragma once

// Keep the optional AprilTag lateral-error retry policy deterministic and
// independently testable from ROS, camera, and vehicle hardware.  The runtime
// controller leaves this policy disabled unless a launch profile opts in.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace camrod_control
{

// Preserve the controller's existing hard motion limits while validating the
// unmodified ROS parameter values.  Invalid values must not become apparently
// safe through abs(), clamp(), or a narrowing integer conversion.
struct AprilTagParkingRetryRawParameters
{
  double forward_distance_m{0.8};
  double forward_speed_mps{0.10};
  double forward_timeout_s{25.0};
  double yaw_alignment_timeout_s{8.0};
  double maximum_lateral_error_m{0.15};
  double maximum_heading_error_rad{0.35};
  double maximum_forward_exit_lateral_drift_m{0.15};
  double maximum_odometry_step_m{0.10};
  double minimum_tag_distance_m{0.35};
  double maximum_tag_distance_m{0.45};
  int64_t maximum_retries{1};
};

inline bool aprilTagParkingRetryRawParametersValid(
  const AprilTagParkingRetryRawParameters & parameters,
  const double final_lateral_tolerance_m,
  const double final_heading_tolerance_rad,
  const double translation_stop_tag_distance_m)
{
  if (!std::isfinite(parameters.forward_distance_m) ||
    !std::isfinite(parameters.forward_speed_mps) ||
    !std::isfinite(parameters.forward_timeout_s) ||
    !std::isfinite(parameters.yaw_alignment_timeout_s) ||
    !std::isfinite(parameters.maximum_lateral_error_m) ||
    !std::isfinite(parameters.maximum_heading_error_rad) ||
    !std::isfinite(parameters.maximum_forward_exit_lateral_drift_m) ||
    !std::isfinite(parameters.maximum_odometry_step_m) ||
    !std::isfinite(parameters.minimum_tag_distance_m) ||
    !std::isfinite(parameters.maximum_tag_distance_m) ||
    !std::isfinite(final_lateral_tolerance_m) ||
    !std::isfinite(final_heading_tolerance_rad) ||
    !std::isfinite(translation_stop_tag_distance_m))
  {
    return false;
  }

  // These are the same hard ranges previously enforced with clamp().  Check
  // the raw values instead so zero, negative, infinity, and out-of-range input
  // stops startup rather than silently selecting a different motion command.
  if (parameters.forward_distance_m < 0.10 ||
    parameters.forward_distance_m > 1.00 ||
    parameters.forward_speed_mps < 0.02 ||
    parameters.forward_speed_mps > 0.20 ||
    parameters.forward_timeout_s < 1.0 ||
    parameters.forward_timeout_s > 30.0 ||
    parameters.yaw_alignment_timeout_s < 1.0 ||
    parameters.yaw_alignment_timeout_s > 15.0)
  {
    return false;
  }

  if (parameters.maximum_lateral_error_m <=
    std::abs(final_lateral_tolerance_m) ||
    parameters.maximum_heading_error_rad <=
    std::abs(final_heading_tolerance_rad) ||
    parameters.maximum_forward_exit_lateral_drift_m <= 0.0 ||
    parameters.maximum_odometry_step_m <= 0.0 ||
    parameters.minimum_tag_distance_m <= 0.0 ||
    parameters.maximum_tag_distance_m <= 0.0 ||
    parameters.minimum_tag_distance_m > parameters.maximum_tag_distance_m ||
    parameters.minimum_tag_distance_m > translation_stop_tag_distance_m ||
    parameters.maximum_tag_distance_m < translation_stop_tag_distance_m)
  {
    return false;
  }

  return parameters.maximum_retries > 0 &&
         parameters.maximum_retries <=
         static_cast<int64_t>(std::numeric_limits<int>::max());
}

// Keep the default-off controller compatible with develop and legacy robot
// parameter files.  Retry-only values cannot affect motion while the feature
// is disabled, so they must not prevent the controller from starting.  Once a
// profile opts in, the complete raw-value validation remains fail-closed.
inline bool aprilTagParkingRetryStartupPermitted(
  const bool enabled, const bool raw_parameters_valid)
{
  return !enabled || raw_parameters_valid;
}

struct AprilTagParkingRetryConfig
{
  bool enabled{false};
  int maximum_retries{0};
  double lateral_tolerance_m{0.03};
  double maximum_lateral_error_m{0.15};
  double maximum_heading_error_rad{0.35};
  double minimum_tag_distance_m{0.35};
  double maximum_tag_distance_m{0.45};
};

inline bool aprilTagParkingRetryEligible(
  const AprilTagParkingRetryConfig & config,
  const int completed_retries,
  const bool tag_fresh,
  const bool odometry_fresh,
  const bool charging_detected,
  const double tag_distance_m,
  const double lateral_error_m,
  const double heading_error_rad)
{
  if (!config.enabled || completed_retries < 0 ||
    completed_retries >= config.maximum_retries || !tag_fresh ||
    !odometry_fresh || charging_detected)
  {
    return false;
  }
  if (!std::isfinite(tag_distance_m) || !std::isfinite(lateral_error_m) ||
    !std::isfinite(heading_error_rad))
  {
    return false;
  }
  const double lateral = std::abs(lateral_error_m);
  return lateral > std::abs(config.lateral_tolerance_m) &&
         lateral <= std::abs(config.maximum_lateral_error_m) &&
         std::abs(heading_error_rad) <=
         std::abs(config.maximum_heading_error_rad) &&
         tag_distance_m >= std::min(
           config.minimum_tag_distance_m, config.maximum_tag_distance_m) &&
         tag_distance_m <= std::max(
           config.minimum_tag_distance_m, config.maximum_tag_distance_m);
}

// Accumulate actual odometry path length instead of start-to-end displacement,
// so a curved forward exit cannot travel farther than the configured bound
// while still appearing close to its start.
class AprilTagParkingRetryProgress
{
public:
  void reset()
  {
    active_ = false;
    start_x_m_ = 0.0;
    start_y_m_ = 0.0;
    start_yaw_rad_ = 0.0;
    last_x_m_ = 0.0;
    last_y_m_ = 0.0;
    distance_m_ = 0.0;
    last_step_m_ = 0.0;
  }

  bool begin(const double x_m, const double y_m, const double yaw_rad)
  {
    reset();
    if (!std::isfinite(x_m) || !std::isfinite(y_m) ||
      !std::isfinite(yaw_rad))
    {
      return false;
    }
    active_ = true;
    start_x_m_ = x_m;
    start_y_m_ = y_m;
    start_yaw_rad_ = yaw_rad;
    last_x_m_ = x_m;
    last_y_m_ = y_m;
    return true;
  }

  bool observe(const double x_m, const double y_m)
  {
    if (!active_ || !std::isfinite(x_m) || !std::isfinite(y_m)) {
      return false;
    }
    last_step_m_ = std::hypot(x_m - last_x_m_, y_m - last_y_m_);
    distance_m_ += last_step_m_;
    last_x_m_ = x_m;
    last_y_m_ = y_m;
    return std::isfinite(distance_m_);
  }

  bool active() const {return active_;}
  double distance() const {return distance_m_;}
  double lastStep() const {return last_step_m_;}
  double forwardProgress() const
  {
    const double dx = last_x_m_ - start_x_m_;
    const double dy = last_y_m_ - start_y_m_;
    return dx * std::cos(start_yaw_rad_) + dy * std::sin(start_yaw_rad_);
  }
  double lateralDrift() const
  {
    const double dx = last_x_m_ - start_x_m_;
    const double dy = last_y_m_ - start_y_m_;
    return -dx * std::sin(start_yaw_rad_) + dy * std::cos(start_yaw_rad_);
  }
  bool reached(const double target_distance_m) const
  {
    return active_ && std::isfinite(target_distance_m) &&
           forwardProgress() >= std::max(0.0, target_distance_m);
  }
  bool exceededPathLimit(
    const double target_distance_m, const double overshoot_margin_m) const
  {
    return active_ && std::isfinite(target_distance_m) &&
           std::isfinite(overshoot_margin_m) &&
           distance_m_ > std::max(0.0, target_distance_m) +
           std::max(0.0, overshoot_margin_m);
  }

private:
  bool active_{false};
  double start_x_m_{0.0};
  double start_y_m_{0.0};
  double start_yaw_rad_{0.0};
  double last_x_m_{0.0};
  double last_y_m_{0.0};
  double distance_m_{0.0};
  double last_step_m_{0.0};
};

}  // namespace camrod_control
