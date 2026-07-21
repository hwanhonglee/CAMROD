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

}  // namespace

MotionCostStop::MotionCostStop(MotionCostStopConfig config)
: config_(std::move(config))
{
}

void MotionCostStop::setConfig(const MotionCostStopConfig & config)
{
  config_ = config;
  if (!config_.latch_enabled) {
    latch_active_ = false;
    clear_since_sec_.reset();
    latch_reason_.clear();
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
    // HH_260721 - Honor the lanelet rotation policy before checking live body obstacles.
    if (std::abs(command.angular.z) > config_.min_translation_mps &&
      !config_.lanelet_allow_rotation)
    {
      const auto lanelet_decision = evaluateLanelet(command, now_sec);
      if (lanelet_decision.blocked) {
        return lanelet_decision;
      }
    }
    const auto rotation_decision = evaluateRotation(now_sec);
    if (rotation_decision.blocked) {
      return rotation_decision;
    }
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
  const avg_msgs::msg::AvgTwist & command, const double now_sec)
{
  if (!config_.lanelet_enabled || !lanelet_grid_.available ||
    !validGrid(lanelet_grid_.grid) || !pose_.has_value() ||
    laneletStaticBypassActive(command))
  {
    return {};
  }
  if (!pose_->frame_id.empty() && !lanelet_grid_.grid.header.frame_id.empty() &&
    pose_->frame_id != lanelet_grid_.grid.header.frame_id)
  {
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
    markBlocked("lanelet_current_out_of_grid", false, now_sec);
    return {true, false, true, false, "lanelet_current_out_of_grid"};
  }
  if (current_cost >= config_.lanelet_current_threshold && !route_reentry) {
    markBlocked("lanelet_current_cost", false, now_sec);
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
        config_.lanelet_stop_on_unknown);
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
        markBlocked(reason, false, now_sec);
        return {true, false, true, false, reason};
      }
    }

    const auto hit = sampleCorridor(lanelet_grid_.grid, corridor, true);
    if (hit.blocked) {
      const std::string reason = "lanelet_" + normalizeLabel(corridor.label) + "_" + hit.detail;
      markBlocked(reason, false, now_sec);
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
          corridor.threshold, config_.dynamic_front_path_max_start_distance_m, false);
        if (path_sample.path_available) {
          if (!path_sample.hit.blocked) {
            continue;
          }
          const std::string reason = "dynamic_front_path:" + source.first;
          markBlocked(reason, true, now_sec);
          return {true, true, false, false, reason};
        }
      }

      const auto hit = sampleCorridor(source.second.grid, corridor, false);
      if (hit.blocked) {
        const std::string reason = "dynamic_" + normalizeLabel(corridor.label) + ":" + source.first;
        markBlocked(reason, true, now_sec);
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
    if (corridor.label == "FRONT" && config_.dynamic_front_use_local_path) {
      const auto path_sample = samplePathCorridor(
        merged_grid_.grid, corridor.lookahead_m, config_.dynamic_front_path_width_m,
        corridor.threshold, config_.dynamic_front_path_max_start_distance_m, false);
      hit = path_sample.path_available ? path_sample.hit :
        sampleCorridor(merged_grid_.grid, corridor, false);
    } else {
      hit = sampleCorridor(merged_grid_.grid, corridor, false);
    }

    if (hit.blocked) {
      const bool dynamic_at_hit = sourceGridBlocksPoint(hit, corridor.threshold, now_sec);
      if ((config_.require_dynamic_source || static_bypass) && !dynamic_at_hit) {
        continue;
      }
      const std::string reason = "merged_" + normalizeLabel(corridor.label);
      markBlocked(reason, true, now_sec);
      return {true, dynamic_at_hit, false, false, reason};
    }
    if (corridor.check_unavoidable && config_.unavoidable_stop_enabled &&
      unavoidable(hit.lethal_cells, hit.total_cells))
    {
      const std::string reason = "dynamic_front_unavoidable";
      markBlocked(reason, true, now_sec);
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
      markBlocked(reason, true, now_sec);
      return {true, true, false, false, reason};
    }
  }
  return {};
}

MotionCostStopDecision MotionCostStop::evaluateLatch(const double now_sec)
{
  if (!latch_active_) {
    return {};
  }
  if (!config_.latch_enabled || config_.clear_required_s <= 0.0) {
    latch_active_ = false;
    clear_since_sec_.reset();
    latch_reason_.clear();
    return {};
  }
  if (!clear_since_sec_.has_value()) {
    clear_since_sec_ = now_sec;
  }
  if (now_sec - *clear_since_sec_ >= config_.clear_required_s) {
    latch_active_ = false;
    clear_since_sec_.reset();
    latch_reason_.clear();
    return {};
  }
  hold_until_sec_ = std::max(hold_until_sec_, now_sec + std::max(0.0, config_.stop_hold_s));
  return {true, true, false, false, "cost_stop_latched:" + latch_reason_};
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

MotionCostStop::PathSample MotionCostStop::samplePathCorridor(
  const avg_msgs::msg::AvgOccupancyGrid & grid,
  const double lookahead_m,
  const double width_m,
  const int threshold,
  const double max_start_distance_m,
  const bool stop_on_unknown) const
{
  PathSample output;
  if (!pose_.has_value() || !local_path_.has_value() ||
    local_path_->poses.size() < 2U || !validGrid(grid))
  {
    return output;
  }
  if (!local_path_->header.frame_id.empty() && !grid.header.frame_id.empty() &&
    local_path_->header.frame_id != grid.header.frame_id)
  {
    return output;
  }

  std::vector<std::pair<double, double>> points;
  points.reserve(local_path_->poses.size());
  for (const auto & stamped_pose : local_path_->poses) {
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

bool MotionCostStop::sourceGridBlocksPoint(
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
      return true;
    }
  }
  return false;
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
