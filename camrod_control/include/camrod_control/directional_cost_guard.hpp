#pragma once

// HH_260721 - Define direction-aware obstacle and lanelet cost arbitration outside the ROS node.

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "avg_msgs/msg/avg_occupancy_grid.hpp"
#include "avg_msgs/msg/avg_path.hpp"
#include "avg_msgs/msg/avg_twist.hpp"

namespace camrod_control
{

struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string frame_id;
  std::string source{"unknown"};
};

struct DirectionalCostGuardConfig
{
  bool enabled{true};
  int cost_stop_threshold{85};
  double fixed_front_lookahead_m{2.0};
  double front_width_m{1.27};
  double stop_hold_s{1.0};
  bool latch_enabled{true};
  double clear_required_s{2.0};
  bool stale_stop_enabled{true};
  double stale_timeout_s{1.0};
  bool require_dynamic_source{true};
  std::set<std::string> dynamic_source_labels{"lidar", "radar"};
  double source_max_age_s{1.0};

  bool use_speed_dependent_lookahead{true};
  double front_lookahead_min_m{2.60};
  double front_lookahead_max_m{3.5};
  double front_friction{0.4};
  double front_reaction_time_s{0.20};
  double front_margin_m{0.45};

  bool side_rear_enabled{true};
  bool body_near_enabled{true};
  double body_near_side_m{1.20};
  double body_near_rear_m{0.80};
  double maneuver_body_near_side_m{1.20};
  double maneuver_body_near_rear_m{0.80};
  int side_threshold{85};
  double side_lookahead_m{1.2};
  double side_width_m{1.69160};
  int rear_threshold{85};
  double rear_lookahead_m{1.2};
  double rear_width_m{1.27};
  double min_translation_mps{0.02};

  bool static_lateral_bypass{true};
  double static_lateral_bypass_min_mps{0.02};
  bool static_reverse_bypass{true};
  double static_reverse_bypass_min_mps{0.02};
  bool rotation_dynamic_stop{true};
  double rotation_radius_m{1.5};
  int rotation_threshold{85};

  bool unavoidable_stop_enabled{true};
  int unavoidable_threshold{90};
  int unavoidable_min_cells{25};
  double unavoidable_min_ratio{0.25};

  bool dynamic_front_use_local_path{true};
  double dynamic_front_path_width_m{1.27};
  double dynamic_front_path_max_start_distance_m{1.5};

  bool lanelet_enabled{true};
  int lanelet_threshold{85};
  int lanelet_current_threshold{85};
  double lanelet_lookahead_m{1.0};
  double lanelet_width_m{0.8};
  bool lanelet_stop_on_unknown{true};
  bool lanelet_allow_rotation{true};
  bool lanelet_check_reverse{false};
  bool lanelet_check_lateral{false};
  bool lanelet_front_use_local_path{true};
  double lanelet_path_max_start_distance_m{1.5};
  double lanelet_path_width_m{0.25};
  bool lanelet_front_path_allow_route_reentry{true};
  bool lanelet_current_allow_route_reentry{true};
  double lanelet_route_reentry_max_distance_m{4.0};
  bool lanelet_route_reentry_require_front_cmd{true};

  // HH_260721 - Keep static-cost exceptions bounded to configured maneuver phases.
  std::set<std::string> drop_zone_static_bypass_phases{"exit_straight", "align_exit_yaw"};
  std::set<std::string> campsite_static_bypass_phases{
    "align_entry_yaw", "reverse_in", "crab_in", "rotate_180",
    "align_return_yaw", "reverse_out", "crab_out"};
};

struct CostGuardDecision
{
  bool blocked{false};
  bool dynamic_obstacle{false};
  bool lanelet_violation{false};
  bool stale_grid{false};
  std::string reason;
};

class DirectionalCostGuard
{
public:
  explicit DirectionalCostGuard(DirectionalCostGuardConfig config = {});

  void setConfig(const DirectionalCostGuardConfig & config);
  const DirectionalCostGuardConfig & config() const;
  void setMergedGrid(const avg_msgs::msg::AvgOccupancyGrid & grid, double receive_sec);
  void setLaneletGrid(const avg_msgs::msg::AvgOccupancyGrid & grid, double receive_sec);
  void setSourceGrid(
    const std::string & label,
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    double receive_sec);
  void setPose(const PlanarPose & pose);
  void setOdometrySpeed(double forward_speed_mps);
  void setLocalPath(const avg_msgs::msg::AvgPath & path);
  void setManeuverPhases(std::string drop_zone_phase, std::string campsite_phase);

  CostGuardDecision evaluate(const avg_msgs::msg::AvgTwist & command, double now_sec);
  bool latched() const;
  double holdUntilSec() const;
  const std::string & latchReason() const;
  double frontLookahead() const;

  // HH_260721 - Expose deterministic geometry primitives for native regression tests.
  static int sampleGridCost(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    double world_x,
    double world_y);
  static std::string normalizeLabel(std::string value);

private:
  struct TimedGrid
  {
    avg_msgs::msg::AvgOccupancyGrid grid;
    double receive_sec{0.0};
    bool available{false};
  };

  struct Corridor
  {
    std::string label;
    double yaw_offset{0.0};
    double lookahead_m{0.0};
    double width_m{0.0};
    int threshold{85};
    bool check_unavoidable{false};
  };

  struct GridHit
  {
    bool blocked{false};
    double world_x{0.0};
    double world_y{0.0};
    int cost{-1};
    std::string detail{"clear"};
    int total_cells{0};
    std::vector<std::pair<int, int>> lethal_cells;
  };

  struct PathSample
  {
    bool path_available{false};
    GridHit hit;
  };

  CostGuardDecision evaluateLanelet(const avg_msgs::msg::AvgTwist & command, double now_sec);
  CostGuardDecision evaluateDynamicSources(
    const std::vector<Corridor> & corridors,
    double now_sec);
  CostGuardDecision evaluateMergedGrid(
    const std::vector<Corridor> & corridors,
    bool static_bypass,
    double now_sec);
  CostGuardDecision evaluateRotation(double now_sec);
  CostGuardDecision evaluateLatch(double now_sec);
  std::vector<Corridor> corridorsForCommand(const avg_msgs::msg::AvgTwist & command) const;
  GridHit sampleCorridor(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    const Corridor & corridor,
    bool lanelet_mode) const;
  GridHit sampleDisk(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    double radius_m,
    int threshold) const;
  PathSample samplePathCorridor(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    double lookahead_m,
    double width_m,
    int threshold,
    double max_start_distance_m,
    bool stop_on_unknown) const;
  std::optional<double> closestPathDistance() const;
  bool sourceIsDynamic(const std::string & label) const;
  bool sourceGridBlocksPoint(const GridHit & hit, int threshold, double now_sec) const;
  bool staticBypassActive(const avg_msgs::msg::AvgTwist & command) const;
  bool laneletStaticBypassActive(const avg_msgs::msg::AvgTwist & command) const;
  bool translational(const avg_msgs::msg::AvgTwist & command) const;
  bool unavoidable(const std::vector<std::pair<int, int>> & cells, int total_cells) const;
  void markBlocked(const std::string & reason, bool latch, double now_sec);
  static bool validGrid(const avg_msgs::msg::AvgOccupancyGrid & grid);
  static bool worldToGrid(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    double world_x,
    double world_y,
    int & grid_x,
    int & grid_y);
  static std::pair<double, double> gridToWorld(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    int grid_x,
    int grid_y);
  static double yawFromGridOrigin(const avg_msgs::msg::AvgOccupancyGrid & grid);

  DirectionalCostGuardConfig config_;
  TimedGrid merged_grid_;
  TimedGrid lanelet_grid_;
  std::map<std::string, TimedGrid> source_grids_;
  std::optional<PlanarPose> pose_;
  std::optional<avg_msgs::msg::AvgPath> local_path_;
  double forward_speed_mps_{0.0};
  std::string drop_zone_phase_;
  std::string campsite_phase_;
  bool latch_active_{false};
  std::optional<double> clear_since_sec_;
  double hold_until_sec_{0.0};
  std::string latch_reason_;
};

}  // namespace camrod_control
