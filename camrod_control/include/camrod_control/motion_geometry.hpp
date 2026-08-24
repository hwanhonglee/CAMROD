#pragma once

// HH_260721 - Keep planar motion and heading geometry in one explicitly named header.

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace camrod_control
{

inline double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

inline double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

template<typename QuaternionT>
inline bool quaternionHasFiniteNonzeroNorm(const QuaternionT & orientation)
{
  const double norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w;
  return std::isfinite(norm_squared) && norm_squared > 1.0e-12;
}

inline bool poseHasFiniteMotionGeometry(
  const avg_msgs::msg::AvgPoseStamped & pose)
{
  return std::isfinite(pose.pose.position.x) &&
         std::isfinite(pose.pose.position.y) &&
         quaternionHasFiniteNonzeroNorm(pose.pose.orientation);
}

inline bool poseHasFinitePlanarPosition(
  const avg_msgs::msg::AvgPoseStamped & pose)
{
  return std::isfinite(pose.pose.position.x) &&
         std::isfinite(pose.pose.position.y);
}

inline bool campsiteAdoptAnchorIsFreshFinite(
  const std::optional<avg_msgs::msg::AvgPoseStamped> & anchor,
  const avg_msgs::msg::AvgPoseStamped & site_goal,
  const double anchor_age_s,
  const double freshness_timeout_s)
{
  if (!anchor.has_value() || !std::isfinite(anchor_age_s) ||
    !std::isfinite(freshness_timeout_s) || anchor_age_s < 0.0 ||
    freshness_timeout_s < 0.0 || anchor_age_s > freshness_timeout_s ||
    !poseHasFiniteMotionGeometry(*anchor) ||
    !poseHasFinitePlanarPosition(site_goal))
  {
    return false;
  }
  return anchor->header.frame_id.empty() || site_goal.header.frame_id.empty() ||
         anchor->header.frame_id == site_goal.header.frame_id;
}

inline bool campsiteAdoptAnchorsCorrelated(
  const avg_msgs::msg::AvgPoseStamped & lanelet_anchor,
  const avg_msgs::msg::AvgPoseStamped & route_anchor,
  const double maximum_separation_m)
{
  if (!poseHasFiniteMotionGeometry(lanelet_anchor) ||
    !poseHasFiniteMotionGeometry(route_anchor) ||
    !std::isfinite(maximum_separation_m) || maximum_separation_m < 0.0)
  {
    return false;
  }
  if (!lanelet_anchor.header.frame_id.empty() &&
    !route_anchor.header.frame_id.empty() &&
    lanelet_anchor.header.frame_id != route_anchor.header.frame_id)
  {
    return false;
  }
  return std::hypot(
    lanelet_anchor.pose.position.x - route_anchor.pose.position.x,
    lanelet_anchor.pose.position.y - route_anchor.pose.position.y) <=
         maximum_separation_m;
}

inline double yawFromPose(const avg_msgs::msg::AvgPoseStamped & pose)
{
  const auto & orientation = pose.pose.orientation;
  if (!quaternionHasFiniteNonzeroNorm(orientation)) {
    return 0.0;
  }
  const double inverse_norm = 1.0 / std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  const double x = orientation.x * inverse_norm;
  const double y = orientation.y * inverse_norm;
  const double z = orientation.z * inverse_norm;
  const double w = orientation.w * inverse_norm;
  const double sin_yaw = 2.0 *
    (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 *
    (y * y + z * z);
  return std::atan2(sin_yaw, cos_yaw);
}

inline std::pair<double, double> relativeXy(
  const avg_msgs::msg::AvgPoseStamped & reference,
  const avg_msgs::msg::AvgPoseStamped & target)
{
  const double yaw = yawFromPose(reference);
  const double dx = target.pose.position.x - reference.pose.position.x;
  const double dy = target.pose.position.y - reference.pose.position.y;
  return {
    std::cos(yaw) * dx + std::sin(yaw) * dy,
    -std::sin(yaw) * dx + std::cos(yaw) * dy};
}

// HH_260818 - Project a goal pair around its stable map anchor using the
// vehicle's current heading. This lets a restarted, 180-degree-reversed robot
// select the opposite crab side without changing the site's map coordinates.
inline std::pair<double, double> relativeXyAtHeading(
  const avg_msgs::msg::AvgPoseStamped & reference,
  const avg_msgs::msg::AvgPoseStamped & target,
  const double current_yaw)
{
  const double dx = target.pose.position.x - reference.pose.position.x;
  const double dy = target.pose.position.y - reference.pose.position.y;
  return {
    std::cos(current_yaw) * dx + std::sin(current_yaw) * dy,
    -std::sin(current_yaw) * dx + std::cos(current_yaw) * dy};
}

// HH_260824 - Materialize a capped crab request as the actual map target shown
// to the operator. This is especially important for B11-B13: their semantic
// site centers are several metres from the lane, but roadside motion is 0.30 m.
inline std::pair<double, double> lateralTargetFromAnchor(
  const double anchor_x,
  const double anchor_y,
  const double anchor_yaw,
  const double signed_direction,
  const double lateral_offset_m)
{
  return {
    anchor_x - std::sin(anchor_yaw) * signed_direction * lateral_offset_m,
    anchor_y + std::cos(anchor_yaw) * signed_direction * lateral_offset_m};
}

inline bool roadsideOperationalArrivalMatches(
  const double current_forward_m,
  const double current_lateral_m,
  const double operational_signed_lateral_m,
  const double forward_tolerance_m,
  const double lateral_tolerance_m)
{
  return std::isfinite(current_forward_m) &&
         std::isfinite(current_lateral_m) &&
         std::isfinite(operational_signed_lateral_m) &&
         std::isfinite(forward_tolerance_m) &&
         std::isfinite(lateral_tolerance_m) && forward_tolerance_m >= 0.0 &&
         lateral_tolerance_m >= 0.0 &&
         std::abs(current_forward_m) <= forward_tolerance_m + 1.0e-6 &&
         std::abs(current_lateral_m - operational_signed_lateral_m) <=
         lateral_tolerance_m + 1.0e-6;
}

// HH_260824 - A regulated campsite arrival owns a lanelet-authored heading in
// the snapped route goal. Use that stable heading for a normal automatic entry;
// only a manual/restarted adoption may infer its exit side from the live body.
enum class CampsiteEntryYawSource
{
  kLaneletSnap,
  kLivePoseFallback,
};

struct CampsiteEntryYawSelection
{
  double yaw_rad{0.0};
  CampsiteEntryYawSource source{CampsiteEntryYawSource::kLivePoseFallback};
};

inline CampsiteEntryYawSelection selectCampsiteEntryYaw(
  const double live_yaw_rad,
  const std::optional<double> lanelet_snap_yaw_rad,
  const bool automatic_arrival)
{
  if (automatic_arrival && lanelet_snap_yaw_rad.has_value() &&
    std::isfinite(*lanelet_snap_yaw_rad))
  {
    return {normalizeAngle(*lanelet_snap_yaw_rad),
      CampsiteEntryYawSource::kLaneletSnap};
  }
  return {normalizeAngle(live_yaw_rad),
    CampsiteEntryYawSource::kLivePoseFallback};
}

inline bool automaticCrabEntryAlignmentTimedOut(
  const bool automatic_crab_alignment,
  const double elapsed_s,
  const double timeout_s)
{
  if (!automatic_crab_alignment) {
    return false;
  }
  if (!std::isfinite(elapsed_s) || !std::isfinite(timeout_s)) {
    return true;
  }
  return elapsed_s > std::max(0.0, timeout_s);
}

// HH_260818 - A turnaround must rotate away from the lane side used for crab
// entry. Reversing the vehicle heading reverses both values together.
inline double turnaroundDirectionForCrab(const double crab_direction)
{
  if (std::abs(crab_direction) <= 1.0e-9) {
    return 0.0;
  }
  return crab_direction > 0.0 ? -1.0 : 1.0;
}

// HH_260807 - Convert a map-frame return-anchor vector into a constant-speed
// body-frame translation so crab mode can remove both lateral and axial drift.
inline std::pair<double, double> bodyTranslationTowardTarget(
  const double current_x,
  const double current_y,
  const double current_yaw,
  const double target_x,
  const double target_y,
  const double speed)
{
  const double dx = target_x - current_x;
  const double dy = target_y - current_y;
  const double distance = std::hypot(dx, dy);
  if (distance <= 1.0e-9 || speed <= 0.0) {
    return {0.0, 0.0};
  }
  const double body_x = std::cos(current_yaw) * dx + std::sin(current_yaw) * dy;
  const double body_y = -std::sin(current_yaw) * dx + std::cos(current_yaw) * dy;
  const double scale = speed / distance;
  return {body_x * scale, body_y * scale};
}

// HH_260818 - Campsite exit must not shorten its lateral clearance by taking a
// diagonal shortcut. Remove body-lateral error first with a pure +/-90 degree
// parallel command, then remove the remaining longitudinal drift with a pure
// straight command. The Ranger driver holds translation while the wheels move
// between those two geometries.
inline std::pair<double, double> bodyAxisPrioritizedTranslationTowardTarget(
  const double current_x,
  const double current_y,
  const double current_yaw,
  const double target_x,
  const double target_y,
  const double maximum_speed,
  const double proportional_gain,
  const double lateral_tolerance)
{
  if (maximum_speed <= 0.0 || proportional_gain <= 0.0) {
    return {0.0, 0.0};
  }
  const double dx = target_x - current_x;
  const double dy = target_y - current_y;
  const double body_x = std::cos(current_yaw) * dx + std::sin(current_yaw) * dy;
  const double body_y = -std::sin(current_yaw) * dx + std::cos(current_yaw) * dy;
  const auto axis_command = [maximum_speed, proportional_gain](const double error) {
      return clamp(
        proportional_gain * error, -maximum_speed, maximum_speed);
    };
  if (std::abs(body_y) > std::max(0.0, lateral_tolerance)) {
    return {0.0, axis_command(body_y)};
  }
  return {axis_command(body_x), 0.0};
}

// HH_260824 - Return from a campsite with one physical steering transition:
// pure lateral crab, a stationary wheel-settle dwell, then pure longitudinal
// correction. The longitudinal stage is latched and can never chatter back to
// parallel steering when localization noise crosses the lateral threshold.
enum class CampsiteCrabReturnStage
{
  kLateral,
  kSteeringSettle,
  kLongitudinal,
};

inline bool campsiteCrabReturnMayComplete(
  const CampsiteCrabReturnStage stage,
  const double radial_error_m,
  const double radial_tolerance_m)
{
  return stage == CampsiteCrabReturnStage::kLongitudinal &&
         std::isfinite(radial_error_m) &&
         std::isfinite(radial_tolerance_m) && radial_tolerance_m >= 0.0 &&
         radial_error_m <= radial_tolerance_m;
}

inline const char * campsiteCrabReturnStageName(
  const CampsiteCrabReturnStage stage)
{
  switch (stage) {
    case CampsiteCrabReturnStage::kLateral:
      return "lateral";
    case CampsiteCrabReturnStage::kSteeringSettle:
      return "steering_settle";
    case CampsiteCrabReturnStage::kLongitudinal:
      return "longitudinal";
  }
  return "lateral";
}

struct CampsiteCrabReturnConfig
{
  // Enter the settle stage more tightly than the final radial tolerance so a
  // pure longitudinal correction can still reach the exact route snap.
  double lateral_transition_tolerance_m{0.02};
  // Once lateral completion is latched, absorb ordinary 10 cm localization
  // noise without commanding another +/-90 <-> 0 degree wheel transition.
  double lateral_hysteresis_m{0.10};
  double steering_settle_s{1.20};
};

struct CampsiteCrabReturnCommand
{
  double linear_x_mps{0.0};
  double linear_y_mps{0.0};
  CampsiteCrabReturnStage stage{CampsiteCrabReturnStage::kLateral};
  bool stage_changed{false};
  // A displacement beyond the hysteresis is a localization/traction fault,
  // not permission to re-enter lateral steering and oscillate indefinitely.
  bool lateral_latch_exceeded{false};
  bool invalid_input{false};
};

class CampsiteCrabReturnSequencer
{
public:
  explicit CampsiteCrabReturnSequencer(
    CampsiteCrabReturnConfig config = CampsiteCrabReturnConfig())
  : config_(sanitized(config))
  {
  }

  void setConfig(const CampsiteCrabReturnConfig & config)
  {
    config_ = sanitized(config);
  }

  void reset(const double now_s = 0.0)
  {
    stage_ = CampsiteCrabReturnStage::kLateral;
    stage_start_s_ = std::isfinite(now_s) ? now_s : 0.0;
  }

  CampsiteCrabReturnStage stage() const
  {
    return stage_;
  }

  CampsiteCrabReturnCommand update(
    const double body_longitudinal_error_m,
    const double body_lateral_error_m,
    const double now_s,
    const double maximum_speed_mps,
    const double proportional_gain)
  {
    CampsiteCrabReturnCommand output;
    output.stage = stage_;
    if (!std::isfinite(body_longitudinal_error_m) ||
      !std::isfinite(body_lateral_error_m) || !std::isfinite(now_s) ||
      !std::isfinite(maximum_speed_mps) ||
      !std::isfinite(proportional_gain))
    {
      output.invalid_input = true;
      return output;
    }
    const double speed = std::max(0.0, maximum_speed_mps);
    const double gain = std::max(0.0, proportional_gain);
    const auto boundedCommand = [speed, gain](const double error) {
        return clamp(gain * error, -speed, speed);
      };

    if (stage_ == CampsiteCrabReturnStage::kLateral) {
      if (std::abs(body_lateral_error_m) >
        config_.lateral_transition_tolerance_m)
      {
        output.linear_y_mps = boundedCommand(body_lateral_error_m);
        return output;
      }
      stage_ = CampsiteCrabReturnStage::kSteeringSettle;
      stage_start_s_ = std::isfinite(now_s) ? now_s : stage_start_s_;
      output.stage = stage_;
      output.stage_changed = true;
      return output;
    }

    const double lateral_latch_limit =
      config_.lateral_transition_tolerance_m + config_.lateral_hysteresis_m;
    if (std::abs(body_lateral_error_m) > lateral_latch_limit) {
      output.lateral_latch_exceeded = true;
      return output;
    }

    if (stage_ == CampsiteCrabReturnStage::kSteeringSettle) {
      const double elapsed_s = std::isfinite(now_s) ? now_s - stage_start_s_ : 0.0;
      if (elapsed_s + 1.0e-9 < config_.steering_settle_s) {
        return output;
      }
      stage_ = CampsiteCrabReturnStage::kLongitudinal;
      output.stage = stage_;
      output.stage_changed = true;
    }

    output.linear_x_mps = boundedCommand(body_longitudinal_error_m);
    return output;
  }

private:
  static CampsiteCrabReturnConfig sanitized(CampsiteCrabReturnConfig config)
  {
    config.lateral_transition_tolerance_m =
      std::max(0.0, config.lateral_transition_tolerance_m);
    config.lateral_hysteresis_m = std::max(0.0, config.lateral_hysteresis_m);
    config.steering_settle_s = std::max(0.0, config.steering_settle_s);
    return config;
  }

  CampsiteCrabReturnConfig config_;
  CampsiteCrabReturnStage stage_{CampsiteCrabReturnStage::kLateral};
  double stage_start_s_{0.0};
};

inline geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion output;
  output.z = std::sin(yaw * 0.5);
  output.w = std::cos(yaw * 0.5);
  return output;
}

}  // namespace camrod_control
