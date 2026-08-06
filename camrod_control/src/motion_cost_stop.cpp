// HH_260721 - Implement forward, reverse, crab, and zero-turn cost-stop decisions and latching.

#include "camrod_control/motion_cost_stop.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <set>

namespace camrod_control
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

bool labelMatches(const std::string & value, const std::set<std::string> & accepted)
{
  for (const auto & label : accepted) {
    if (value == label || (!label.empty() && value.find(label) != std::string::npos)) {
      return true;
    }
  }
  return false;
}

bool pointInPolygon(
  const double x,
  const double y,
  const std::vector<std::pair<double, double>> & polygon)
{
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & current = polygon[i];
    const auto & previous = polygon[j];
    const bool crosses = ((current.second > y) != (previous.second > y)) &&
      (x < (previous.first - current.first) * (y - current.second) /
      (previous.second - current.second) + current.first);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

}  // namespace

MotionCostStop::MotionCostStop(MotionCostStopConfig config)
: config_(std::move(config))
{
}

void MotionCostStop::setConfig(const MotionCostStopConfig & config)
{
  config_ = config;
  if (!config_.latch_enabled) {
    clearLatch();
  }
}

const MotionCostStopConfig & MotionCostStop::config() const
{
  return config_;
}

void MotionCostStop::setMergedGrid(
  const avg_msgs::msg::AvgOccupancyGrid & grid, const double receive_sec)
{
  merged_grid_ = TimedGrid{grid, receive_sec, true};
}

void MotionCostStop::setLaneletGrid(
  const avg_msgs::msg::AvgOccupancyGrid & grid, const double receive_sec)
{
  lanelet_grid_ = TimedGrid{grid, receive_sec, true};
}

void MotionCostStop::setSourceGrid(
  const std::string & label,
  const avg_msgs::msg::AvgOccupancyGrid & grid,
  const double receive_sec)
{
  source_grids_[normalizeLabel(label)] = TimedGrid{grid, receive_sec, true};
}

void MotionCostStop::setPose(const PlanarPose & pose)
{
  pose_ = pose;
}

void MotionCostStop::setFootprintPolygonWorld(
  const std::vector<std::pair<double, double>> & polygon_world)
{
  if (!pose_.has_value() || polygon_world.size() < 3) {
    return;
  }
  const double cosine = std::cos(pose_->yaw);
  const double sine = std::sin(pose_->yaw);
  std::vector<std::pair<double, double>> local;
  local.reserve(polygon_world.size());
  for (const auto & point : polygon_world) {
    const double dx = point.first - pose_->x;
    const double dy = point.second - pose_->y;
    local.emplace_back(cosine * dx + sine * dy, -sine * dx + cosine * dy);
  }
  footprint_polygon_local_ = std::move(local);
}

void MotionCostStop::setOdometrySpeed(const double forward_speed_mps)
{
  forward_speed_mps_ = forward_speed_mps;
}

void MotionCostStop::setLocalPath(const avg_msgs::msg::AvgPath & path)
{
  local_path_ = path;
}

void MotionCostStop::setManeuverPhases(
  std::string drop_zone_phase, std::string campsite_phase)
{
  drop_zone_phase_ = normalizeLabel(std::move(drop_zone_phase));
  campsite_phase_ = normalizeLabel(std::move(campsite_phase));
}

MotionCostStopDecision MotionCostStop::evaluate(
  const avg_msgs::msg::AvgTwist & command, const double now_sec)
{
  if (!config_.enabled) {
    return {};
  }

  // HH_260728 - A stop is released only by fresh clear evidence for the exact
  // source and geometry that triggered it. The new command cannot substitute
  // a zero/rotation corridor for the original front/side/rear hazard.
  if (latch_active_) {
    const auto latch_decision = evaluateLatchedHazard(now_sec);
    if (latch_decision.blocked) {
      return latch_decision;
    }
  }

  // HH_260721 - Treat a missing or stale merged grid as a fail-closed motion condition.
  if (config_.stale_stop_enabled && config_.stale_timeout_s > 0.0) {
    if (!merged_grid_.available || !validGrid(merged_grid_.grid)) {
      markBlocked("merged_cost_grid_missing", false, now_sec);
      return {true, false, false, true, "merged_cost_grid_missing"};
    }
    const double age_s = std::max(0.0, now_sec - merged_grid_.receive_sec);
    if (age_s > config_.stale_timeout_s) {
      const std::string reason = "merged_cost_grid_stale";
      markBlocked(reason, false, now_sec);
      return {true, false, false, true, reason};
    }
  }

  if (!translational(command)) {
    const double motion_threshold = std::max(0.0, config_.min_translation_mps);
    if (std::abs(command.angular.z) > motion_threshold) {
      // HH_260727 - A rotating asymmetric body can touch the raw map boundary even while
      // robot_center_link stays clear, so evaluate the full footprint first.
      const auto lanelet_decision = evaluateLanelet(command, now_sec);
      if (lanelet_decision.blocked) {
        return lanelet_decision;
      }
      const auto rotation_decision = evaluateRotation(now_sec);
      if (rotation_decision.blocked) {
        return rotation_decision;
      }
    }
    // HH_260731 - A true zero command is stationary, not a rotation command.
    // Do not create a new radar rotation latch while stopped. An already active
    // latch was checked above and still remains fail-closed until fresh clear evidence.
    return evaluateLatch(now_sec);
  }

  const auto lanelet_decision = evaluateLanelet(command, now_sec);
  if (lanelet_decision.blocked) {
    return lanelet_decision;
  }

  const auto corridors = corridorsForCommand(command);
  if (corridors.empty()) {
    return evaluateLatch(now_sec);
  }

  const auto source_decision = evaluateDynamicSources(corridors, now_sec);
  if (source_decision.blocked) {
    return source_decision;
  }

  const auto merged_decision = evaluateMergedGrid(
    corridors, staticBypassActive(command), now_sec);
  if (merged_decision.blocked) {
    return merged_decision;
  }
  return evaluateLatch(now_sec);
}

MotionCostStopDecision MotionCostStop::evaluateLaneletRecovery(
  const avg_msgs::msg::AvgTwist & command,
  const double now_sec,
  const double pose_max_age_s)
{
  if (!config_.enabled || !config_.lanelet_enabled) {
    return {};
  }
  // HH_260729 - Recovery must not interpret absent, stale, or frame-mismatched
  // route evidence as clear. The normal command path also has merged-grid
  // fail-closed checks; this probe is intentionally self-contained.
  if (!lanelet_grid_.available || !validGrid(lanelet_grid_.grid)) {
    return {true, false, true, true, "lanelet_recovery_grid_missing"};
  }
  const double lanelet_timeout_s = config_.lanelet_recovery_stale_timeout_s > 0.0 ?
    config_.lanelet_recovery_stale_timeout_s : config_.stale_timeout_s;
  if (lanelet_timeout_s > 0.0 &&
    std::max(0.0, now_sec - lanelet_grid_.receive_sec) > lanelet_timeout_s)
  {
    return {true, false, true, true, "lanelet_recovery_grid_stale"};
  }
  if (!pose_.has_value()) {
    return {true, false, true, true, "lanelet_recovery_pose_missing"};
  }
  if (pose_max_age_s > 0.0 &&
    (pose_->observation_sec <= 0.0 ||
    std::max(0.0, now_sec - pose_->observation_sec) > pose_max_age_s))
  {
    return {true, false, true, true, "lanelet_recovery_pose_stale"};
  }
  if (pose_->frame_id.empty() || lanelet_grid_.grid.header.frame_id.empty()) {
    return {true, false, true, true, "lanelet_recovery_frame_missing"};
  }
  if (pose_->frame_id != lanelet_grid_.grid.header.frame_id)
  {
    return {true, false, true, true, "lanelet_recovery_frame_mismatch"};
  }
  return evaluateLanelet(command, now_sec, false);
}

MotionCostStopDecision MotionCostStop::evaluateRouteRecoveryCommand(
  const avg_msgs::msg::AvgTwist & command,
  const double now_sec,
  const double probe_distance_m,
  const double pose_max_age_s)
{
  if (!config_.enabled) {
    return {};
  }

  // HH_260731 - Recovery motion is safety-critical and must never interpret a
  // missing, stale, invalid, or frame-incompatible lanelet grid as projected
  // clearance. The earlier route-hold probe already failed closed, but the
  // command-specific projection previously skipped these checks.
  if (!config_.lanelet_enabled || !lanelet_grid_.available ||
    !validGrid(lanelet_grid_.grid))
  {
    return {true, false, true, true, "route_recovery_lanelet_grid_missing"};
  }
  const double lanelet_timeout_s = config_.lanelet_recovery_stale_timeout_s > 0.0 ?
    config_.lanelet_recovery_stale_timeout_s : config_.stale_timeout_s;
  if (lanelet_timeout_s > 0.0 &&
    std::max(0.0, now_sec - lanelet_grid_.receive_sec) > lanelet_timeout_s)
  {
    return {true, false, true, true, "route_recovery_lanelet_grid_stale"};
  }

  const double translation_norm = std::hypot(command.linear.x, command.linear.y);
  if (translation_norm <= std::max(0.0, config_.min_translation_mps)) {
    return {true, false, true, false, "route_recovery_translation_required"};
  }
  // HH_260805 - Route recovery may add only a small reverse-yaw correction.
  // Larger angular commands remain outside this bounded safety exception.
  constexpr double kMaximumRecoveryAngularRateRadps = 0.15;
  if (std::abs(command.angular.z) > kMaximumRecoveryAngularRateRadps) {
    return {true, false, true, false, "route_recovery_angular_rate_exceeded"};
  }
  if (!pose_.has_value()) {
    return {true, false, true, true, "route_recovery_pose_missing"};
  }
  if (pose_max_age_s > 0.0 &&
    (pose_->observation_sec <= 0.0 ||
    std::max(0.0, now_sec - pose_->observation_sec) > pose_max_age_s))
  {
    return {true, false, true, true, "route_recovery_pose_stale"};
  }
  if (pose_->frame_id.empty() || lanelet_grid_.grid.header.frame_id.empty()) {
    return {true, false, true, true, "route_recovery_frame_missing"};
  }
  if (pose_->frame_id != lanelet_grid_.grid.header.frame_id)
  {
    return {true, false, true, true, "route_recovery_frame_mismatch"};
  }

  // HH_260806 - Planning-margin contact may use a bounded escape candidate,
  // but a cost-100 cell inside the physical body is never recoverable motion.
  if (config_.lanelet_body_hard_stop_enabled) {
    const auto body_hit = samplePhysicalBody(
      lanelet_grid_.grid, config_.lanelet_body_hard_stop_threshold,
      config_.lanelet_stop_on_unknown);
    if (body_hit.blocked) {
      return {
        true, false, true, false,
        "route_recovery_physical_body_" + body_hit.detail};
    }
  }

  // HH_260729 - A dynamic-obstacle latch remains authoritative during route
  // recovery. Moving away from a map boundary never bypasses live evidence.
  if (latch_active_) {
    const auto latch_decision = evaluateLatchedHazard(now_sec);
    if (latch_decision.blocked) {
      return latch_decision;
    }
  }
  if (config_.stale_stop_enabled && config_.stale_timeout_s > 0.0) {
    if (!merged_grid_.available || !validGrid(merged_grid_.grid)) {
      markBlocked("merged_cost_grid_missing", false, now_sec);
      return {true, false, false, true, "merged_cost_grid_missing"};
    }
    if (std::max(0.0, now_sec - merged_grid_.receive_sec) > config_.stale_timeout_s) {
      markBlocked("merged_cost_grid_stale", false, now_sec);
      return {true, false, false, true, "merged_cost_grid_stale"};
    }
  }

  // Project a short body-frame twist, including the bounded reverse-yaw arc.
  // The current footprint may already touch cost 100, but the projected full
  // footprint must be clear; a command parallel to or deeper into the boundary
  // therefore remains blocked.
  const PlanarPose current_pose = *pose_;
  const double distance = std::max(0.05, probe_distance_m);
  const double duration_s = distance / translation_norm;
  const double yaw_delta = command.angular.z * duration_s;
  double body_x = distance * command.linear.x / translation_norm;
  double body_y = distance * command.linear.y / translation_norm;
  if (std::abs(command.angular.z) > 1.0e-6) {
    // Exact planar constant-twist integration prevents a yawed corner from
    // being evaluated as if the robot had translated without rotating.
    body_x =
      (command.linear.x * std::sin(yaw_delta) +
      command.linear.y * (std::cos(yaw_delta) - 1.0)) /
      command.angular.z;
    body_y =
      (command.linear.x * (1.0 - std::cos(yaw_delta)) +
      command.linear.y * std::sin(yaw_delta)) /
      command.angular.z;
  }
  const double cosine = std::cos(current_pose.yaw);
  const double sine = std::sin(current_pose.yaw);
  pose_->x += cosine * body_x - sine * body_y;
  pose_->y += sine * body_x + cosine * body_y;
  pose_->yaw += yaw_delta;
  const auto projected_lanelet_decision = evaluateLanelet(command, now_sec, false);
  *pose_ = current_pose;
  if (projected_lanelet_decision.blocked) {
    auto decision = projected_lanelet_decision;
    decision.reason = "route_recovery_predicted_" + decision.reason;
    return decision;
  }

  // Evaluate obstacle sources from the actual current pose and requested escape
  // corridor. Only the present lanelet contact receives the bounded exception.
  const auto corridors = corridorsForCommand(command);
  if (corridors.empty()) {
    return {true, false, true, false, "route_recovery_corridor_missing"};
  }
  const auto source_decision = evaluateDynamicSources(corridors, now_sec);
  if (source_decision.blocked) {
    return source_decision;
  }
  const auto merged_decision = evaluateMergedGrid(
    corridors, staticBypassActive(command), now_sec);
  if (merged_decision.blocked) {
    return merged_decision;
  }
  return evaluateLatch(now_sec);
}

bool MotionCostStop::latched() const
{
  return latch_active_;
}

double MotionCostStop::holdUntilSec() const
{
  return hold_until_sec_;
}

const std::string & MotionCostStop::latchReason() const
{
  return latch_reason_;
}

double MotionCostStop::frontLookahead() const
{
  if (!config_.use_speed_dependent_lookahead) {
    return config_.fixed_front_lookahead_m;
  }
  const double speed = std::abs(forward_speed_mps_);
  const double friction = std::max(0.05, config_.front_friction);
  const double braking_m = speed * speed / (2.0 * 9.8 * friction);
  const double requested = braking_m + config_.front_reaction_time_s * speed +
    config_.front_margin_m;
  return std::clamp(
    requested, config_.front_lookahead_min_m, config_.front_lookahead_max_m);
}

MotionCostStopDecision MotionCostStop::evaluateLanelet(
  const avg_msgs::msg::AvgTwist & command,
  const double now_sec,
  const bool update_hold)
{
  if (!config_.lanelet_enabled || !lanelet_grid_.available ||
    !validGrid(lanelet_grid_.grid) || !pose_.has_value())
  {
    return {};
  }
  if (!pose_->frame_id.empty() && !lanelet_grid_.grid.header.frame_id.empty() &&
    pose_->frame_id != lanelet_grid_.grid.header.frame_id)
  {
    return {};
  }

  const bool any_motion =
    std::abs(command.linear.x) > config_.min_translation_mps ||
    std::abs(command.linear.y) > config_.min_translation_mps ||
    std::abs(command.angular.z) > config_.min_translation_mps;

  // HH_260806 - The physical rectangle is authoritative before the larger
  // planning margin. This reason is intentionally not eligible for crab,
  // reverse, or reverse-yaw recovery.
  if (config_.lanelet_body_hard_stop_enabled && any_motion) {
    const auto body_hit = samplePhysicalBody(
      lanelet_grid_.grid, config_.lanelet_body_hard_stop_threshold,
      config_.lanelet_stop_on_unknown);
    if (body_hit.blocked) {
      const std::string reason = "lanelet_physical_body_" + body_hit.detail;
      if (update_hold) {
        markBlocked(reason, false, now_sec);
      }
      return {true, false, true, false, reason};
    }
  }

  // HH_260727 - A maneuver/static exception may skip only the legacy
  // base-link current-cell and directional corridor checks. The complete
  // planning footprint remains a fail-closed boundary for every motion.
  if (config_.lanelet_footprint_enabled && any_motion) {
    const auto footprint_hit = sampleFootprint(
      lanelet_grid_.grid, config_.lanelet_footprint_threshold,
      config_.lanelet_stop_on_unknown);
    if (footprint_hit.blocked) {
      const std::string reason = "lanelet_footprint_" + footprint_hit.detail;
      if (update_hold) {
        markBlocked(reason, false, now_sec);
      }
      return {true, false, true, false, reason};
    }
  }

  if (laneletStaticBypassActive(command)) {
    return {};
  }

  // HH_260721 - Apply current-cell lanelet cost only to explicitly enabled travel directions.
  const bool front_command = command.linear.x > config_.min_translation_mps;
  const bool reverse_command = config_.lanelet_check_reverse &&
    command.linear.x < -config_.min_translation_mps;
  const bool lateral_command = config_.lanelet_check_lateral &&
    std::abs(command.linear.y) > config_.min_translation_mps;
  // HH_260721 - A disabled rotation exception checks the occupied current cell without a corridor.
  const bool rotation_command = !config_.lanelet_allow_rotation &&
    std::abs(command.angular.z) > config_.min_translation_mps;
  if (!front_command && !reverse_command && !lateral_command && !rotation_command) {
    return {};
  }

  const auto path_distance = closestPathDistance();
  const bool route_reentry = config_.lanelet_current_allow_route_reentry &&
    path_distance.has_value() && *path_distance <= config_.lanelet_route_reentry_max_distance_m &&
    (!config_.lanelet_route_reentry_require_front_cmd || front_command);

  const int current_cost = sampleGridCost(lanelet_grid_.grid, pose_->x, pose_->y);
  if (current_cost < 0 && config_.lanelet_stop_on_unknown && !route_reentry) {
    if (update_hold) {
      markBlocked("lanelet_current_out_of_grid", false, now_sec);
    }
    return {true, false, true, false, "lanelet_current_out_of_grid"};
  }
  if (current_cost >= config_.lanelet_current_threshold && !route_reentry) {
    if (update_hold) {
      markBlocked("lanelet_current_cost", false, now_sec);
    }
    return {true, false, true, false, "lanelet_current_cost"};
  }
  if (rotation_command) {
    return {};
  }

  std::vector<Corridor> corridors;
  if (front_command) {
    corridors.push_back(
      {"FRONT", 0.0, config_.lanelet_lookahead_m, config_.lanelet_width_m,
        config_.lanelet_threshold, false});
  }
  if (reverse_command) {
    corridors.push_back(
      {"REAR", kPi, config_.lanelet_lookahead_m, config_.lanelet_width_m,
        config_.lanelet_threshold, false});
  }
  if (lateral_command && command.linear.y > config_.min_translation_mps) {
    corridors.push_back(
      {"LEFT", kPi * 0.5, config_.lanelet_lookahead_m, config_.lanelet_width_m,
        config_.lanelet_threshold, false});
  }
  if (lateral_command && command.linear.y < -config_.min_translation_mps) {
    corridors.push_back(
      {"RIGHT", -kPi * 0.5, config_.lanelet_lookahead_m, config_.lanelet_width_m,
        config_.lanelet_threshold, false});
  }

  for (const auto & corridor : corridors) {
    if (corridor.label == "FRONT" && config_.lanelet_front_use_local_path) {
      const auto path_sample = samplePathCorridor(
        lanelet_grid_.grid, corridor.lookahead_m, config_.lanelet_path_width_m,
        corridor.threshold,
        route_reentry ? config_.lanelet_route_reentry_max_distance_m :
        config_.lanelet_path_max_start_distance_m,
        config_.lanelet_stop_on_unknown, local_path_);
      if (path_sample.path_available) {
        if (!path_sample.hit.blocked) {
          continue;
        }
        const double blocked_distance = std::hypot(
          path_sample.hit.world_x - pose_->x, path_sample.hit.world_y - pose_->y);
        const bool front_path_reentry = route_reentry &&
          config_.lanelet_front_path_allow_route_reentry &&
          blocked_distance <= std::max(0.05, config_.lanelet_lookahead_m);
        if (front_path_reentry) {
          continue;
        }
        const std::string reason = "lanelet_front_path_" + path_sample.hit.detail;
        if (update_hold) {
          markBlocked(reason, false, now_sec);
        }
        return {true, false, true, false, reason};
      }
    }

    const auto hit = sampleCorridor(lanelet_grid_.grid, corridor, true);
    if (hit.blocked) {
      const std::string reason = "lanelet_" + normalizeLabel(corridor.label) + "_" + hit.detail;
      if (update_hold) {
        markBlocked(reason, false, now_sec);
      }
      return {true, false, true, false, reason};
    }
  }
  return {};
}

MotionCostStopDecision MotionCostStop::evaluateDynamicSources(
  const std::vector<Corridor> & corridors, const double now_sec)
{
  for (const auto & source : source_grids_) {
    if (!sourceIsDynamic(source.first) || !source.second.available ||
      !validGrid(source.second.grid))
    {
      continue;
    }
    if (config_.source_max_age_s > 0.0 &&
      now_sec - source.second.receive_sec > config_.source_max_age_s)
    {
      continue;
    }
    if (!pose_.has_value() ||
      (!pose_->frame_id.empty() && !source.second.grid.header.frame_id.empty() &&
      pose_->frame_id != source.second.grid.header.frame_id))
    {
      continue;
    }

    for (const auto & corridor : corridors) {
      if (corridor.label == "FRONT" && config_.dynamic_front_use_local_path) {
        const auto path_sample = samplePathCorridor(
          source.second.grid, corridor.lookahead_m, config_.dynamic_front_path_width_m,
          corridor.threshold, config_.dynamic_front_path_max_start_distance_m, false,
          local_path_);
        if (path_sample.path_available) {
          if (!path_sample.hit.blocked) {
            continue;
          }
          const std::string reason = "dynamic_front_path:" + source.first;
          LatchContext context;
          context.probe_kind = LatchProbeKind::kPath;
          context.corridor = corridor;
          context.path_snapshot = local_path_;
          context.source_label = source.first;
          context.source_receive_sec_at_trigger = source.second.receive_sec;
          context.source_stamp_sec_at_trigger = messageStampSec(source.second.grid);
          context.merged_receive_sec_at_trigger = merged_grid_.receive_sec;
          context.merged_stamp_sec_at_trigger = messageStampSec(merged_grid_.grid);
          context.path_width_m = config_.dynamic_front_path_width_m;
          context.path_max_start_distance_m =
            config_.dynamic_front_path_max_start_distance_m;
          context.reason = reason;
          activateLatch(std::move(context), now_sec);
          return {true, true, false, false, reason};
        }
      }

      const auto hit = sampleCorridor(source.second.grid, corridor, false);
      if (hit.blocked) {
        const std::string reason = "dynamic_" + normalizeLabel(corridor.label) + ":" + source.first;
        LatchContext context;
        context.probe_kind = LatchProbeKind::kCorridor;
        context.corridor = corridor;
        context.source_label = source.first;
        context.source_receive_sec_at_trigger = source.second.receive_sec;
        context.source_stamp_sec_at_trigger = messageStampSec(source.second.grid);
        context.merged_receive_sec_at_trigger = merged_grid_.receive_sec;
        context.merged_stamp_sec_at_trigger = messageStampSec(merged_grid_.grid);
        context.reason = reason;
        activateLatch(std::move(context), now_sec);
        return {true, true, false, false, reason};
      }
    }
  }
  return {};
}

MotionCostStopDecision MotionCostStop::evaluateMergedGrid(
  const std::vector<Corridor> & corridors,
  const bool static_bypass,
  const double now_sec)
{
  if (!merged_grid_.available || !validGrid(merged_grid_.grid) || !pose_.has_value()) {
    return {};
  }
  if (!pose_->frame_id.empty() && !merged_grid_.grid.header.frame_id.empty() &&
    pose_->frame_id != merged_grid_.grid.header.frame_id)
  {
    return {};
  }

  for (const auto & corridor : corridors) {
    GridHit hit;
    bool path_probe = false;
    if (corridor.label == "FRONT" && config_.dynamic_front_use_local_path) {
      const auto path_sample = samplePathCorridor(
        merged_grid_.grid, corridor.lookahead_m, config_.dynamic_front_path_width_m,
        corridor.threshold, config_.dynamic_front_path_max_start_distance_m, false,
        local_path_);
      path_probe = path_sample.path_available;
      hit = path_sample.path_available ? path_sample.hit :
        sampleCorridor(merged_grid_.grid, corridor, false);
    } else {
      hit = sampleCorridor(merged_grid_.grid, corridor, false);
    }

    if (hit.blocked) {
      const auto blocking_source = sourceGridBlockingPoint(hit, corridor.threshold, now_sec);
      const bool dynamic_at_hit = blocking_source.has_value();
      if ((config_.require_dynamic_source || static_bypass) && !dynamic_at_hit) {
        continue;
      }
      const std::string reason = "merged_" + normalizeLabel(corridor.label) +
        (blocking_source.has_value() ? ":" + *blocking_source : "");
      LatchContext context;
      context.probe_kind = path_probe ? LatchProbeKind::kPath : LatchProbeKind::kCorridor;
      context.corridor = corridor;
      context.path_snapshot = path_probe ? local_path_ : std::nullopt;
      context.source_label = blocking_source;
      if (blocking_source.has_value()) {
        context.source_receive_sec_at_trigger =
          source_grids_.at(*blocking_source).receive_sec;
        context.source_stamp_sec_at_trigger =
          messageStampSec(source_grids_.at(*blocking_source).grid);
      }
      context.merged_receive_sec_at_trigger = merged_grid_.receive_sec;
      context.merged_stamp_sec_at_trigger = messageStampSec(merged_grid_.grid);
      context.path_width_m = config_.dynamic_front_path_width_m;
      context.path_max_start_distance_m =
        config_.dynamic_front_path_max_start_distance_m;
      context.probe_merged_grid = true;
      context.reason = reason;
      activateLatch(std::move(context), now_sec);
      return {true, dynamic_at_hit, false, false, reason};
    }
    if (corridor.check_unavoidable && config_.unavoidable_stop_enabled &&
      unavoidable(hit.lethal_cells, hit.total_cells))
    {
      const std::string reason = "dynamic_front_unavoidable";
      LatchContext context;
      context.probe_kind = LatchProbeKind::kCorridor;
      context.corridor = corridor;
      context.merged_receive_sec_at_trigger = merged_grid_.receive_sec;
      context.merged_stamp_sec_at_trigger = messageStampSec(merged_grid_.grid);
      context.probe_merged_grid = true;
      context.reason = reason;
      activateLatch(std::move(context), now_sec);
      return {true, true, false, false, reason};
    }
  }
  return {};
}

MotionCostStopDecision MotionCostStop::evaluateRotation(const double now_sec)
{
  if (!config_.rotation_dynamic_stop) {
    return {};
  }
  for (const auto & source : source_grids_) {
    if (!sourceIsDynamic(source.first) || !source.second.available ||
      !validGrid(source.second.grid) ||
      (config_.source_max_age_s > 0.0 &&
      now_sec - source.second.receive_sec > config_.source_max_age_s))
    {
      continue;
    }
    const auto hit = sampleDisk(
      source.second.grid, std::max(0.05, config_.rotation_radius_m),
      config_.rotation_threshold);
    if (hit.blocked) {
      const std::string reason = "dynamic_rotation:" + source.first;
      LatchContext context;
      context.probe_kind = LatchProbeKind::kRotation;
      context.source_label = source.first;
      context.source_receive_sec_at_trigger = source.second.receive_sec;
      context.source_stamp_sec_at_trigger = messageStampSec(source.second.grid);
      context.merged_receive_sec_at_trigger = merged_grid_.receive_sec;
      context.merged_stamp_sec_at_trigger = messageStampSec(merged_grid_.grid);
      context.rotation_radius_m = config_.rotation_radius_m;
      context.rotation_threshold = config_.rotation_threshold;
      context.reason = reason;
      activateLatch(std::move(context), now_sec);
      return {true, true, false, false, reason};
    }
  }
  return {};
}

MotionCostStopDecision MotionCostStop::evaluateLatchedHazard(const double now_sec)
{
  if (!latch_context_.has_value()) {
    return keepLatch("trigger_context_missing", now_sec, true);
  }
  const LatchContext & context = *latch_context_;
  if (!latchEvidenceFresh(context, now_sec)) {
    return keepLatch("trigger_evidence_not_fresh", now_sec, true);
  }

  const TimedGrid * probe_grid = nullptr;
  if (context.probe_merged_grid) {
    probe_grid = &merged_grid_;
  } else if (context.source_label.has_value()) {
    const auto source = source_grids_.find(*context.source_label);
    if (source != source_grids_.end()) {
      probe_grid = &source->second;
    }
  }
  if (probe_grid == nullptr || !probe_grid->available || !validGrid(probe_grid->grid)) {
    return keepLatch("trigger_grid_unavailable", now_sec, true);
  }

  GridHit hit;
  if (context.probe_kind == LatchProbeKind::kRotation) {
    hit = sampleDisk(
      probe_grid->grid, std::max(0.05, context.rotation_radius_m),
      context.rotation_threshold);
  } else if (context.probe_kind == LatchProbeKind::kPath) {
    const auto path_sample = samplePathCorridor(
      probe_grid->grid, context.corridor.lookahead_m,
      context.path_width_m, context.corridor.threshold,
      context.path_max_start_distance_m, false,
      context.path_snapshot);
    if (!path_sample.path_available) {
      return keepLatch("trigger_path_unavailable", now_sec, true);
    }
    hit = path_sample.hit;
  } else {
    hit = sampleCorridor(probe_grid->grid, context.corridor, false);
  }

  if (hit.blocked) {
    // HH_260728 - Refresh the trigger timestamps only while the saved hazard is
    // still occupied. A later clear frame must be newer than this evidence.
    LatchContext refreshed = context;
    refreshed.merged_receive_sec_at_trigger = merged_grid_.receive_sec;
    refreshed.merged_stamp_sec_at_trigger = messageStampSec(merged_grid_.grid);
    if (refreshed.source_label.has_value()) {
      const auto source = source_grids_.find(*refreshed.source_label);
      if (source != source_grids_.end()) {
        refreshed.source_receive_sec_at_trigger = source->second.receive_sec;
        refreshed.source_stamp_sec_at_trigger = messageStampSec(source->second.grid);
      }
    }
    const std::string reason = refreshed.reason;
    activateLatch(std::move(refreshed), now_sec);
    return {true, true, false, false, reason};
  }

  return evaluateLatch(now_sec);
}

MotionCostStopDecision MotionCostStop::evaluateLatch(const double now_sec)
{
  if (!latch_active_) {
    return {};
  }
  if (!config_.latch_enabled || config_.clear_required_s <= 0.0) {
    hold_until_sec_ = std::max(
      hold_until_sec_, now_sec + std::max(0.0, config_.stop_hold_s));
    clearLatch();
    return {};
  }
  if (!clear_since_sec_.has_value()) {
    clear_since_sec_ = now_sec;
    const double merged_stamp_sec = messageStampSec(merged_grid_.grid);
    clear_merged_evidence_start_sec_ =
      merged_stamp_sec > 0.0 ? merged_stamp_sec : merged_grid_.receive_sec;
    clear_source_evidence_start_sec_.reset();
    if (latch_context_.has_value() && latch_context_->source_label.has_value()) {
      const auto source = source_grids_.find(*latch_context_->source_label);
      if (source != source_grids_.end()) {
        const double source_stamp_sec = messageStampSec(source->second.grid);
        clear_source_evidence_start_sec_ =
          source_stamp_sec > 0.0 ? source_stamp_sec : source->second.receive_sec;
      }
    }
  }
  if (now_sec - *clear_since_sec_ >= config_.clear_required_s) {
    const auto evidence_sec = [](const TimedGrid & timed_grid) {
        const double stamp_sec = messageStampSec(timed_grid.grid);
        return stamp_sec > 0.0 ? stamp_sec : timed_grid.receive_sec;
      };
    const bool merged_advanced =
      clear_merged_evidence_start_sec_.has_value() &&
      evidence_sec(merged_grid_) > *clear_merged_evidence_start_sec_;
    bool source_advanced = true;
    if (latch_context_.has_value() && latch_context_->source_label.has_value()) {
      const auto source = source_grids_.find(*latch_context_->source_label);
      source_advanced =
        source != source_grids_.end() &&
        clear_source_evidence_start_sec_.has_value() &&
        evidence_sec(source->second) > *clear_source_evidence_start_sec_;
    }
    // HH_260728 - A single clear grid cannot prove a continuous-clear window,
    // even when clear_required_s is tuned below the grid freshness timeout.
    if (!merged_advanced || !source_advanced) {
      return keepLatch("clear_evidence_not_advanced", now_sec, false);
    }
    // HH_260728 - Start the configured post-clear hold at confirmed release,
    // rather than inheriting a nearly expired hold from the previous callback.
    hold_until_sec_ = std::max(
      hold_until_sec_, now_sec + std::max(0.0, config_.stop_hold_s));
    clearLatch();
    return {};
  }
  return keepLatch("", now_sec, false);
}

MotionCostStopDecision MotionCostStop::keepLatch(
  const std::string & detail,
  const double now_sec,
  const bool reset_clear_timer)
{
  if (reset_clear_timer) {
    clear_since_sec_.reset();
    clear_merged_evidence_start_sec_.reset();
    clear_source_evidence_start_sec_.reset();
  }
  hold_until_sec_ = std::max(
    hold_until_sec_, now_sec + std::max(0.0, config_.stop_hold_s));
  std::string reason = "cost_stop_latched:" + latch_reason_;
  if (!detail.empty()) {
    reason += ":" + detail;
  }
  return {true, true, false, false, reason};
}

bool MotionCostStop::latchEvidenceFresh(
  const LatchContext & context,
  const double now_sec) const
{
  if (!pose_.has_value() || !merged_grid_.available || !validGrid(merged_grid_.grid)) {
    return false;
  }
  const auto frame_matches_pose = [this](const TimedGrid & timed_grid) {
      return pose_->frame_id.empty() || timed_grid.grid.header.frame_id.empty() ||
             pose_->frame_id == timed_grid.grid.header.frame_id;
    };
  const auto timestamp_is_fresh = [now_sec](
    const TimedGrid & timed_grid, const double timeout_s)
    {
      const double stamp_sec = messageStampSec(timed_grid.grid);
      return stamp_sec <= 0.0 || timeout_s <= 0.0 ||
             std::max(0.0, now_sec - stamp_sec) <= timeout_s;
    };
  const auto evidence_advanced = [](
    const TimedGrid & timed_grid,
    const double trigger_receive_sec,
    const double trigger_stamp_sec)
    {
      const double current_stamp_sec = messageStampSec(timed_grid.grid);
      if (current_stamp_sec > 0.0 && trigger_stamp_sec > 0.0) {
        return current_stamp_sec > trigger_stamp_sec;
      }
      return timed_grid.receive_sec > trigger_receive_sec;
    };
  if (!frame_matches_pose(merged_grid_)) {
    return false;
  }
  if (context.merged_stamp_sec_at_trigger > 0.0 &&
    messageStampSec(merged_grid_.grid) <= 0.0)
  {
    return false;
  }
  if (config_.stale_timeout_s > 0.0 &&
    (std::max(0.0, now_sec - merged_grid_.receive_sec) > config_.stale_timeout_s ||
    !timestamp_is_fresh(merged_grid_, config_.stale_timeout_s)))
  {
    return false;
  }
  if (!clear_since_sec_.has_value() &&
    !evidence_advanced(
      merged_grid_, context.merged_receive_sec_at_trigger,
      context.merged_stamp_sec_at_trigger))
  {
    return false;
  }

  if (!context.source_label.has_value()) {
    return true;
  }
  const auto source = source_grids_.find(*context.source_label);
  if (source == source_grids_.end() || !source->second.available ||
    !validGrid(source->second.grid) || !frame_matches_pose(source->second))
  {
    return false;
  }
  if (context.source_stamp_sec_at_trigger > 0.0 &&
    messageStampSec(source->second.grid) <= 0.0)
  {
    return false;
  }
  if (config_.source_max_age_s > 0.0 &&
    (std::max(0.0, now_sec - source->second.receive_sec) > config_.source_max_age_s ||
    !timestamp_is_fresh(source->second, config_.source_max_age_s)))
  {
    return false;
  }
  return clear_since_sec_.has_value() ||
         evidence_advanced(
    source->second, context.source_receive_sec_at_trigger,
    context.source_stamp_sec_at_trigger);
}

void MotionCostStop::activateLatch(LatchContext context, const double now_sec)
{
  hold_until_sec_ = std::max(
    hold_until_sec_, now_sec + std::max(0.0, config_.stop_hold_s));
  if (!config_.latch_enabled) {
    return;
  }
  latch_reason_ = context.reason;
  latch_context_ = std::move(context);
  latch_active_ = true;
  clear_since_sec_.reset();
  clear_merged_evidence_start_sec_.reset();
  clear_source_evidence_start_sec_.reset();
}

void MotionCostStop::clearLatch()
{
  latch_active_ = false;
  latch_context_.reset();
  clear_since_sec_.reset();
  clear_merged_evidence_start_sec_.reset();
  clear_source_evidence_start_sec_.reset();
  latch_reason_.clear();
}

std::vector<MotionCostStop::Corridor> MotionCostStop::corridorsForCommand(
  const avg_msgs::msg::AvgTwist & command) const
{
  std::vector<Corridor> corridors;
  const double minimum = std::max(0.0, config_.min_translation_mps);
  const bool maneuver = command.linear.x<-minimum || std::abs(command.linear.y)> minimum;
  const double near_side = maneuver ? config_.maneuver_body_near_side_m : config_.body_near_side_m;
  const double near_rear = maneuver ? config_.maneuver_body_near_rear_m : config_.body_near_rear_m;

  if (command.linear.x > minimum) {
    corridors.push_back(
      {"FRONT", 0.0, frontLookahead(), config_.front_width_m,
        config_.cost_stop_threshold, true});
  } else if (config_.side_rear_enabled && command.linear.x < -minimum) {
    corridors.push_back(
      {"REAR", kPi, maneuver ? near_rear : config_.rear_lookahead_m,
        config_.rear_width_m, config_.rear_threshold, false});
  }
  if (config_.side_rear_enabled && command.linear.y > minimum) {
    corridors.push_back(
      {"LEFT", kPi * 0.5, maneuver ? near_side : config_.side_lookahead_m,
        config_.side_width_m, config_.side_threshold, false});
  } else if (config_.side_rear_enabled && command.linear.y < -minimum) {
    corridors.push_back(
      {"RIGHT", -kPi * 0.5, maneuver ? near_side : config_.side_lookahead_m,
        config_.side_width_m, config_.side_threshold, false});
  }

  if (config_.side_rear_enabled && config_.body_near_enabled && translational(command)) {
    const std::vector<Corridor> near_corridors{
      {"LEFT_NEAR", kPi * 0.5, near_side, config_.side_width_m, config_.side_threshold, false},
      {"RIGHT_NEAR", -kPi * 0.5, near_side, config_.side_width_m, config_.side_threshold, false},
      {"REAR_NEAR", kPi, near_rear, config_.rear_width_m, config_.rear_threshold, false}};
    for (const auto & candidate : near_corridors) {
      const bool same_direction = std::any_of(
        corridors.begin(), corridors.end(), [&candidate](const auto & existing) {
          return std::abs(existing.yaw_offset - candidate.yaw_offset) < 1.0e-6;
        });
      if (!same_direction && candidate.lookahead_m > 0.0) {
        corridors.push_back(candidate);
      }
    }
  }
  return corridors;
}

MotionCostStop::GridHit MotionCostStop::sampleCorridor(
  const avg_msgs::msg::AvgOccupancyGrid & grid,
  const Corridor & corridor,
  const bool lanelet_mode) const
{
  GridHit hit;
  if (!pose_.has_value() || !validGrid(grid)) {
    return hit;
  }

  const double resolution = grid.info.resolution;
  const double lookahead = std::max(0.05, corridor.lookahead_m);
  const double width = std::max(0.05, corridor.width_m);
  const double heading = pose_->yaw + corridor.yaw_offset;
  const double cosine = std::cos(heading);
  const double sine = std::sin(heading);
  const int along_steps = static_cast<int>(std::floor(lookahead / resolution + 1.0e-9)) + 1;
  const int lateral_steps = static_cast<int>(std::floor(width / resolution + 1.0e-9)) + 1;

  for (int along_index = 0; along_index < along_steps; ++along_index) {
    const double along = along_index * resolution;
    for (int lateral_index = 0; lateral_index < lateral_steps; ++lateral_index) {
      const double lateral = -0.5 * width + lateral_index * resolution;
      const double world_x = pose_->x + along * cosine - lateral * sine;
      const double world_y = pose_->y + along * sine + lateral * cosine;
      int grid_x = 0;
      int grid_y = 0;
      if (!worldToGrid(grid, world_x, world_y, grid_x, grid_y)) {
        if (lanelet_mode && config_.lanelet_stop_on_unknown) {
          hit.blocked = true;
          hit.world_x = world_x;
          hit.world_y = world_y;
          hit.detail = "out_of_grid";
          return hit;
        }
        continue;
      }
      const int cost = grid.data[grid_y * static_cast<int>(grid.info.width) + grid_x];
      ++hit.total_cells;
      if (cost >= corridor.threshold) {
        const auto center = gridToWorld(grid, grid_x, grid_y);
        hit.blocked = true;
        hit.world_x = center.first;
        hit.world_y = center.second;
        hit.cost = cost;
        hit.detail = "cost";
        return hit;
      }
      if (cost >= config_.unavoidable_threshold) {
        hit.lethal_cells.emplace_back(grid_x, grid_y);
      }
    }
  }
  return hit;
}

MotionCostStop::GridHit MotionCostStop::sampleDisk(
  const avg_msgs::msg::AvgOccupancyGrid & grid,
  const double radius_m,
  const int threshold) const
{
  GridHit hit;
  if (!pose_.has_value() || !validGrid(grid)) {
    return hit;
  }
  const double resolution = grid.info.resolution;
  const int cell_radius = std::max(1, static_cast<int>(std::ceil(radius_m / resolution)));
  int center_x = 0;
  int center_y = 0;
  if (!worldToGrid(grid, pose_->x, pose_->y, center_x, center_y)) {
    return hit;
  }
  for (int delta_y = -cell_radius; delta_y <= cell_radius; ++delta_y) {
    for (int delta_x = -cell_radius; delta_x <= cell_radius; ++delta_x) {
      if (std::hypot(delta_x * resolution, delta_y * resolution) > radius_m) {
        continue;
      }
      const int grid_x = center_x + delta_x;
      const int grid_y = center_y + delta_y;
      if (grid_x < 0 || grid_y < 0 ||
        grid_x >= static_cast<int>(grid.info.width) ||
        grid_y >= static_cast<int>(grid.info.height))
      {
        continue;
      }
      const int cost = grid.data[grid_y * static_cast<int>(grid.info.width) + grid_x];
      if (cost >= threshold) {
        const auto world = gridToWorld(grid, grid_x, grid_y);
        hit.blocked = true;
        hit.world_x = world.first;
        hit.world_y = world.second;
        hit.cost = cost;
        hit.detail = "cost";
        return hit;
      }
    }
  }
  return hit;
}

MotionCostStop::GridHit MotionCostStop::sampleFootprint(
  const avg_msgs::msg::AvgOccupancyGrid & grid,
  const int threshold,
  const bool stop_on_unknown) const
{
  std::vector<std::pair<double, double>> local = footprint_polygon_local_;
  if (local.size() < 3) {
    local = {
      {config_.footprint_front_m, config_.footprint_left_m},
      {config_.footprint_front_m, -config_.footprint_right_m},
      {-config_.footprint_rear_m, -config_.footprint_right_m},
      {-config_.footprint_rear_m, config_.footprint_left_m}};
  }
  return samplePolygonFootprint(grid, threshold, stop_on_unknown, local);
}

MotionCostStop::GridHit MotionCostStop::samplePhysicalBody(
  const avg_msgs::msg::AvgOccupancyGrid & grid,
  const int threshold,
  const bool stop_on_unknown) const
{
  const std::vector<std::pair<double, double>> local{
    {config_.body_front_m, config_.body_left_m},
    {config_.body_front_m, -config_.body_right_m},
    {-config_.body_rear_m, -config_.body_right_m},
    {-config_.body_rear_m, config_.body_left_m}};
  return samplePolygonFootprint(grid, threshold, stop_on_unknown, local);
}

MotionCostStop::GridHit MotionCostStop::samplePolygonFootprint(
  const avg_msgs::msg::AvgOccupancyGrid & grid,
  const int threshold,
  const bool stop_on_unknown,
  const std::vector<std::pair<double, double>> & local_polygon) const
{
  GridHit hit;
  if (!pose_.has_value() || !validGrid(grid) || local_polygon.size() < 3) {
    return hit;
  }

  const double cosine = std::cos(pose_->yaw);
  const double sine = std::sin(pose_->yaw);
  std::vector<std::pair<double, double>> world;
  world.reserve(local_polygon.size());
  for (const auto & point : local_polygon) {
    world.emplace_back(
      pose_->x + cosine * point.first - sine * point.second,
      pose_->y + sine * point.first + cosine * point.second);
  }

  auto check_point = [&](const double world_x, const double world_y) {
      int grid_x = 0;
      int grid_y = 0;
      if (!worldToGrid(grid, world_x, world_y, grid_x, grid_y)) {
        if (stop_on_unknown) {
          hit.blocked = true;
          hit.world_x = world_x;
          hit.world_y = world_y;
          hit.detail = "out_of_grid";
        }
        return;
      }
      const int cost = grid.data[grid_y * static_cast<int>(grid.info.width) + grid_x];
      ++hit.total_cells;
      if (cost >= threshold) {
        const auto center = gridToWorld(grid, grid_x, grid_y);
        hit.blocked = true;
        hit.world_x = center.first;
        hit.world_y = center.second;
        hit.cost = cost;
        hit.detail = "cost";
      }
    };

  // HH_260727 - Sample every polygon edge at half-cell spacing. This catches contact with
  // a boundary cell even when that cell's center lies just outside the body.
  const double edge_step = std::max(0.01, grid.info.resolution * 0.5);
  for (std::size_t index = 0; index < world.size() && !hit.blocked; ++index) {
    const auto & start = world[index];
    const auto & end = world[(index + 1) % world.size()];
    const double length = std::hypot(end.first - start.first, end.second - start.second);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / edge_step)));
    for (int step = 0; step <= steps && !hit.blocked; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      check_point(
        start.first + ratio * (end.first - start.first),
        start.second + ratio * (end.second - start.second));
    }
  }
  if (hit.blocked) {
    return hit;
  }

  // HH_260727 - Check the grid-cell centers covered by the polygon so interior cost can
  // never be hidden by clear vertices or edges.
  double min_grid_x = std::numeric_limits<double>::infinity();
  double min_grid_y = std::numeric_limits<double>::infinity();
  double max_grid_x = -std::numeric_limits<double>::infinity();
  double max_grid_y = -std::numeric_limits<double>::infinity();
  const double origin_yaw = yawFromGridOrigin(grid);
  const double origin_cosine = std::cos(origin_yaw);
  const double origin_sine = std::sin(origin_yaw);
  for (const auto & point : world) {
    const double dx = point.first - grid.info.origin.position.x;
    const double dy = point.second - grid.info.origin.position.y;
    const double local_x = origin_cosine * dx + origin_sine * dy;
    const double local_y = -origin_sine * dx + origin_cosine * dy;
    min_grid_x = std::min(min_grid_x, local_x / grid.info.resolution);
    min_grid_y = std::min(min_grid_y, local_y / grid.info.resolution);
    max_grid_x = std::max(max_grid_x, local_x / grid.info.resolution);
    max_grid_y = std::max(max_grid_y, local_y / grid.info.resolution);
  }

  const int x_begin = std::max(0, static_cast<int>(std::floor(min_grid_x)) - 1);
  const int y_begin = std::max(0, static_cast<int>(std::floor(min_grid_y)) - 1);
  const int x_end = std::min(
    static_cast<int>(grid.info.width) - 1,
    static_cast<int>(std::ceil(max_grid_x)) + 1);
  const int y_end = std::min(
    static_cast<int>(grid.info.height) - 1,
    static_cast<int>(std::ceil(max_grid_y)) + 1);
  for (int grid_y = y_begin; grid_y <= y_end && !hit.blocked; ++grid_y) {
    for (int grid_x = x_begin; grid_x <= x_end && !hit.blocked; ++grid_x) {
      const auto cell = gridToWorld(grid, grid_x, grid_y);
      if (!pointInPolygon(cell.first, cell.second, world)) {
        continue;
      }
      const int cost = grid.data[grid_y * static_cast<int>(grid.info.width) + grid_x];
      ++hit.total_cells;
      if (cost >= threshold) {
        hit.blocked = true;
        hit.world_x = cell.first;
        hit.world_y = cell.second;
        hit.cost = cost;
        hit.detail = "cost";
      }
    }
  }
  return hit;
}

MotionCostStop::PathSample MotionCostStop::samplePathCorridor(
  const avg_msgs::msg::AvgOccupancyGrid & grid,
  const double lookahead_m,
  const double width_m,
  const int threshold,
  const double max_start_distance_m,
  const bool stop_on_unknown,
  const std::optional<avg_msgs::msg::AvgPath> & path) const
{
  PathSample output;
  if (!pose_.has_value() || !path.has_value() ||
    path->poses.size() < 2U || !validGrid(grid))
  {
    return output;
  }
  if (!path->header.frame_id.empty() && !grid.header.frame_id.empty() &&
    path->header.frame_id != grid.header.frame_id)
  {
    return output;
  }

  std::vector<std::pair<double, double>> points;
  points.reserve(path->poses.size());
  for (const auto & stamped_pose : path->poses) {
    const double x = stamped_pose.pose.position.x;
    const double y = stamped_pose.pose.position.y;
    if (std::isfinite(x) && std::isfinite(y)) {
      points.emplace_back(x, y);
    }
  }
  if (points.size() < 2U) {
    return output;
  }

  std::size_t closest_index = 0U;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index < points.size(); ++index) {
    const double distance = std::hypot(
      points[index].first - pose_->x,
      points[index].second - pose_->y);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = index;
    }
  }
  if (closest_distance > std::max(0.05, max_start_distance_m)) {
    return output;
  }
  output.path_available = true;

  const double resolution = grid.info.resolution;
  std::vector<double> lateral_offsets{0.0};
  const int lateral_steps = static_cast<int>(std::floor(0.5 * width_m / resolution + 1.0e-9));
  for (int index = 1; index <= lateral_steps; ++index) {
    lateral_offsets.push_back(index * resolution);
    lateral_offsets.push_back(-index * resolution);
  }

  double accumulated = 0.0;
  const double scan_lookahead = std::max(0.05, lookahead_m);
  for (std::size_t index = closest_index; index + 1U < points.size(); ++index) {
    const double dx = points[index + 1U].first - points[index].first;
    const double dy = points[index + 1U].second - points[index].second;
    const double segment_length = std::hypot(dx, dy);
    if (segment_length <= 1.0e-4) {
      continue;
    }
    const double cosine = dx / segment_length;
    const double sine = dy / segment_length;
    const int steps = std::max(1, static_cast<int>(std::ceil(segment_length / resolution)));
    for (int step = 0; step <= steps; ++step) {
      const double raw_along = std::min(segment_length, step * resolution);
      if (accumulated + raw_along > scan_lookahead + resolution * 1.0e-3) {
        return output;
      }
      // HH_260721 - Include the exact lookahead boundary despite float32 grid resolution drift.
      const double along = std::min(raw_along, std::max(0.0, scan_lookahead - accumulated));
      const double center_x = points[index].first + cosine * along;
      const double center_y = points[index].second + sine * along;
      for (const double lateral : lateral_offsets) {
        const double world_x = center_x - lateral * sine;
        const double world_y = center_y + lateral * cosine;
        int grid_x = 0;
        int grid_y = 0;
        if (!worldToGrid(grid, world_x, world_y, grid_x, grid_y)) {
          if (stop_on_unknown) {
            output.hit = {true, world_x, world_y, -1, "out_of_grid", 0, {}};
            return output;
          }
          continue;
        }
        const int cost = grid.data[grid_y * static_cast<int>(grid.info.width) + grid_x];
        if (cost >= threshold) {
          const auto world = gridToWorld(grid, grid_x, grid_y);
          output.hit = {true, world.first, world.second, cost, "cost", 0, {}};
          return output;
        }
      }
    }
    accumulated += segment_length;
    if (accumulated >= scan_lookahead) {
      break;
    }
  }
  return output;
}

std::optional<double> MotionCostStop::closestPathDistance() const
{
  if (!pose_.has_value() || !local_path_.has_value() || local_path_->poses.size() < 2U ||
    (!local_path_->header.frame_id.empty() && !pose_->frame_id.empty() &&
    local_path_->header.frame_id != pose_->frame_id))
  {
    return std::nullopt;
  }
  double closest = std::numeric_limits<double>::infinity();
  for (const auto & stamped_pose : local_path_->poses) {
    closest = std::min(
      closest,
      std::hypot(
        stamped_pose.pose.position.x - pose_->x,
        stamped_pose.pose.position.y - pose_->y));
  }
  return std::isfinite(closest) ? std::optional<double>(closest) : std::nullopt;
}

bool MotionCostStop::sourceIsDynamic(const std::string & label) const
{
  return labelMatches(normalizeLabel(label), config_.dynamic_source_labels);
}

std::optional<std::string> MotionCostStop::sourceGridBlockingPoint(
  const GridHit & hit, const int threshold, const double now_sec) const
{
  for (const auto & source : source_grids_) {
    if (!sourceIsDynamic(source.first) || !source.second.available ||
      (config_.source_max_age_s > 0.0 &&
      now_sec - source.second.receive_sec > config_.source_max_age_s))
    {
      continue;
    }
    if (sampleGridCost(source.second.grid, hit.world_x, hit.world_y) >= threshold) {
      return source.first;
    }
  }
  return std::nullopt;
}

bool MotionCostStop::staticBypassActive(const avg_msgs::msg::AvgTwist & command) const
{
  if (config_.campsite_static_bypass_phases.count(campsite_phase_) > 0U) {
    return true;
  }
  const double lateral_min = std::max(0.0, config_.static_lateral_bypass_min_mps);
  const bool pure_lateral = config_.static_lateral_bypass &&
    std::abs(command.linear.y) > lateral_min && std::abs(command.linear.x) <= lateral_min;
  const double reverse_min = std::max(0.0, config_.static_reverse_bypass_min_mps);
  const bool pure_reverse = config_.static_reverse_bypass &&
    command.linear.x < -reverse_min && std::abs(command.linear.y) <= lateral_min;
  return pure_lateral || pure_reverse;
}

bool MotionCostStop::laneletStaticBypassActive(
  const avg_msgs::msg::AvgTwist & command) const
{
  const bool forward_drop_zone_exit =
    config_.drop_zone_static_bypass_phases.count(drop_zone_phase_) > 0U &&
    command.linear.x > config_.min_translation_mps &&
    std::abs(command.linear.y) <= config_.min_translation_mps;
  return forward_drop_zone_exit ||
         (config_.campsite_static_bypass_phases.count(campsite_phase_) > 0U &&
         translational(command));
}

bool MotionCostStop::translational(const avg_msgs::msg::AvgTwist & command) const
{
  const double minimum = std::max(0.0, config_.min_translation_mps);
  return std::abs(command.linear.x) > minimum || std::abs(command.linear.y) > minimum;
}

bool MotionCostStop::unavoidable(
  const std::vector<std::pair<int, int>> & cells, const int total_cells) const
{
  std::set<std::pair<int, int>> remaining(cells.begin(), cells.end());
  int largest_cluster = 0;
  while (!remaining.empty()) {
    std::queue<std::pair<int, int>> pending;
    pending.push(*remaining.begin());
    remaining.erase(remaining.begin());
    int cluster_size = 0;
    while (!pending.empty()) {
      const auto cell = pending.front();
      pending.pop();
      ++cluster_size;
      const std::pair<int, int> neighbors[] = {
        {cell.first + 1, cell.second}, {cell.first - 1, cell.second},
        {cell.first, cell.second + 1}, {cell.first, cell.second - 1}};
      for (const auto & neighbor : neighbors) {
        const auto found = remaining.find(neighbor);
        if (found != remaining.end()) {
          pending.push(neighbor);
          remaining.erase(found);
        }
      }
    }
    largest_cluster = std::max(largest_cluster, cluster_size);
  }
  const double ratio = static_cast<double>(largest_cluster) /
    static_cast<double>(std::max(1, total_cells));
  return largest_cluster >= config_.unavoidable_min_cells && ratio >= config_.unavoidable_min_ratio;
}

void MotionCostStop::markBlocked(
  const std::string & reason, const bool latch, const double now_sec)
{
  hold_until_sec_ = std::max(hold_until_sec_, now_sec + std::max(0.0, config_.stop_hold_s));
  if (latch && config_.latch_enabled) {
    latch_active_ = true;
    clear_since_sec_.reset();
    latch_reason_ = reason;
  }
}

bool MotionCostStop::validGrid(const avg_msgs::msg::AvgOccupancyGrid & grid)
{
  return grid.info.resolution > 0.0 && grid.info.width > 0U && grid.info.height > 0U &&
         grid.data.size() >= static_cast<std::size_t>(grid.info.width) * grid.info.height;
}

bool MotionCostStop::worldToGrid(
  const avg_msgs::msg::AvgOccupancyGrid & grid,
  const double world_x,
  const double world_y,
  int & grid_x,
  int & grid_y)
{
  if (!validGrid(grid)) {
    return false;
  }
  // HH_260721 - Respect rotated OccupancyGrid origins instead of assuming map-axis alignment.
  const double dx = world_x - grid.info.origin.position.x;
  const double dy = world_y - grid.info.origin.position.y;
  const double yaw = yawFromGridOrigin(grid);
  const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  grid_x = static_cast<int>(std::floor(local_x / grid.info.resolution));
  grid_y = static_cast<int>(std::floor(local_y / grid.info.resolution));
  return grid_x >= 0 && grid_y >= 0 &&
         grid_x < static_cast<int>(grid.info.width) &&
         grid_y < static_cast<int>(grid.info.height);
}

std::pair<double, double> MotionCostStop::gridToWorld(
  const avg_msgs::msg::AvgOccupancyGrid & grid, const int grid_x, const int grid_y)
{
  const double local_x = (grid_x + 0.5) * grid.info.resolution;
  const double local_y = (grid_y + 0.5) * grid.info.resolution;
  const double yaw = yawFromGridOrigin(grid);
  return {
    grid.info.origin.position.x + std::cos(yaw) * local_x - std::sin(yaw) * local_y,
    grid.info.origin.position.y + std::sin(yaw) * local_x + std::cos(yaw) * local_y};
}

double MotionCostStop::messageStampSec(const avg_msgs::msg::AvgOccupancyGrid & grid)
{
  return static_cast<double>(grid.header.stamp.sec) +
         static_cast<double>(grid.header.stamp.nanosec) * 1.0e-9;
}

double MotionCostStop::yawFromGridOrigin(const avg_msgs::msg::AvgOccupancyGrid & grid)
{
  const auto & orientation = grid.info.origin.orientation;
  const double sine = 2.0 *
    (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosine = 1.0 - 2.0 *
    (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sine, cosine);
}

int MotionCostStop::sampleGridCost(
  const avg_msgs::msg::AvgOccupancyGrid & grid,
  const double world_x,
  const double world_y)
{
  int grid_x = 0;
  int grid_y = 0;
  if (!worldToGrid(grid, world_x, world_y, grid_x, grid_y)) {
    return -1;
  }
  return grid.data[grid_y * static_cast<int>(grid.info.width) + grid_x];
}

std::string MotionCostStop::normalizeLabel(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(), [](const unsigned char character) {
      if (character == '-' || std::isspace(character)) {
        return '_';
      }
      return static_cast<char>(std::tolower(character));
    });
  return value;
}

}  // namespace camrod_control
