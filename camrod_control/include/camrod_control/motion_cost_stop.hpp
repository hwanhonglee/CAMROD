#pragma once

// HH_260721 - Stop and latch motion for obstacle or lanelet cost in every commanded direction.

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
  double observation_sec{0.0};
};

struct MotionCostStopConfig
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
  // HH_260803 - Static route raster updates are slower than dynamic obstacle
  // grids; recovery keeps a separate age without weakening dynamic fail-close.
  double lanelet_recovery_stale_timeout_s{0.0};
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
  double body_near_side_m{0.60};
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
  // HH_260806 - Cost-100 contact inside the provisional configured body is a
  // hard stop. Unlike planning-margin contact, it may never authorize
  // recovery motion.
  bool lanelet_body_hard_stop_enabled{true};
  int lanelet_body_hard_stop_threshold{100};
  double body_front_m{0.65837};
  double body_rear_m{0.63323};
  double body_left_m{0.43505};
  double body_right_m{0.43495};
  // HH_260727 - Check the complete configured planning footprint against raw
  // lanelet cost, not only the robot_center_link cell. A separate lethal
  // threshold lets narrow lanes retain their soft 98 boundary penalty.
  bool lanelet_footprint_enabled{true};
  int lanelet_footprint_threshold{100};
  // HH_260806 - Provisional body candidate plus 0.05 m clearance on all sides.
  double footprint_front_m{0.70837};
  double footprint_rear_m{0.68323};
  double footprint_left_m{0.48505};
  double footprint_right_m{0.48495};
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
  // HH_260721 - Name the campsite exception after its same-lanelet retrace behavior.
  std::set<std::string> campsite_static_bypass_phases{
    "align_entry_yaw", "reverse_in", "crab_in", "rotate_180",
    "align_retrace_yaw", "reverse_out", "crab_out"};
};

struct MotionCostStopDecision
{
  bool blocked{false};
  bool dynamic_obstacle{false};
  bool lanelet_violation{false};
  bool stale_grid{false};
  std::string reason;
};

class MotionCostStop
{
public:
  explicit MotionCostStop(MotionCostStopConfig config = {});

  void setConfig(const MotionCostStopConfig & config);
  const MotionCostStopConfig & config() const;
  void setMergedGrid(const avg_msgs::msg::AvgOccupancyGrid & grid, double receive_sec);
  void setLaneletGrid(const avg_msgs::msg::AvgOccupancyGrid & grid, double receive_sec);
  void setSourceGrid(
    const std::string & label,
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    double receive_sec);
  void setPose(const PlanarPose & pose);
  void setFootprintPolygonWorld(
    const std::vector<std::pair<double, double>> & polygon_world);
  void setOdometrySpeed(double forward_speed_mps);
  void setLocalPath(const avg_msgs::msg::AvgPath & path);
  void setManeuverPhases(std::string drop_zone_phase, std::string campsite_phase);

  MotionCostStopDecision evaluate(const avg_msgs::msg::AvgTwist & command, double now_sec);
  // HH_260729 - A stopped Nav2 action no longer emits a useful command, so the
  // gate periodically revalidates the saved route direction through this
  // fail-closed lanelet-only probe before resuming or reissuing the goal.
  MotionCostStopDecision evaluateLaneletRecovery(
    const avg_msgs::msg::AvgTwist & command,
    double now_sec,
    double pose_max_age_s);
  // HH_260731 - A bounded reverse or orthogonal-crab recovery candidate may
  // ignore only the footprint's present lanelet contact. It must put the
  // complete footprint inside the lanelet after the probe distance and still
  // pass fresh lanelet evidence, dynamic costs, and ordinary interlocks.
  MotionCostStopDecision evaluateRouteRecoveryCommand(
    const avg_msgs::msg::AvgTwist & command,
    double now_sec,
    double probe_distance_m,
    double pose_max_age_s);
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

  enum class LatchProbeKind
  {
    kCorridor,
    kPath,
    kRotation
  };

  // HH_260728 - Preserve the exact dynamic hazard that started a stop. Incoming
  // zero or changed-direction commands must not redefine what "clear" means.
  struct LatchContext
  {
    LatchProbeKind probe_kind{LatchProbeKind::kCorridor};
    Corridor corridor;
    std::optional<avg_msgs::msg::AvgPath> path_snapshot;
    std::optional<std::string> source_label;
    double source_receive_sec_at_trigger{0.0};
    double source_stamp_sec_at_trigger{0.0};
    double merged_receive_sec_at_trigger{0.0};
    double merged_stamp_sec_at_trigger{0.0};
    double path_width_m{0.0};
    double path_max_start_distance_m{0.0};
    double rotation_radius_m{0.0};
    int rotation_threshold{85};
    bool probe_merged_grid{false};
    std::string reason;
  };

  MotionCostStopDecision evaluateLanelet(
    const avg_msgs::msg::AvgTwist & command,
    double now_sec,
    bool update_hold = true);
  MotionCostStopDecision evaluateDynamicSources(
    const std::vector<Corridor> & corridors,
    double now_sec);
  MotionCostStopDecision evaluateMergedGrid(
    const std::vector<Corridor> & corridors,
    bool static_bypass,
    double now_sec);
  MotionCostStopDecision evaluateRotation(double now_sec);
  MotionCostStopDecision evaluateLatchedHazard(double now_sec);
  MotionCostStopDecision evaluateLatch(double now_sec);
  MotionCostStopDecision keepLatch(
    const std::string & detail,
    double now_sec,
    bool reset_clear_timer);
  bool latchEvidenceFresh(const LatchContext & context, double now_sec) const;
  void activateLatch(LatchContext context, double now_sec);
  void clearLatch();
  std::vector<Corridor> corridorsForCommand(const avg_msgs::msg::AvgTwist & command) const;
  GridHit sampleCorridor(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    const Corridor & corridor,
    bool lanelet_mode) const;
  GridHit sampleDisk(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    double radius_m,
    int threshold) const;
  GridHit sampleFootprint(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    int threshold,
    bool stop_on_unknown) const;
  GridHit samplePhysicalBody(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    int threshold,
    bool stop_on_unknown) const;
  GridHit samplePolygonFootprint(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    int threshold,
    bool stop_on_unknown,
    const std::vector<std::pair<double, double>> & local_polygon) const;
  PathSample samplePathCorridor(
    const avg_msgs::msg::AvgOccupancyGrid & grid,
    double lookahead_m,
    double width_m,
    int threshold,
    double max_start_distance_m,
    bool stop_on_unknown,
    const std::optional<avg_msgs::msg::AvgPath> & path) const;
  std::optional<double> closestPathDistance() const;
  bool sourceIsDynamic(const std::string & label) const;
  std::optional<std::string> sourceGridBlockingPoint(
    const GridHit & hit,
    int threshold,
    double now_sec) const;
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
  static double messageStampSec(const avg_msgs::msg::AvgOccupancyGrid & grid);
  static double yawFromGridOrigin(const avg_msgs::msg::AvgOccupancyGrid & grid);

  MotionCostStopConfig config_;
  TimedGrid merged_grid_;
  TimedGrid lanelet_grid_;
  std::map<std::string, TimedGrid> source_grids_;
  std::optional<PlanarPose> pose_;
  // HH_260727 - Stored in robot_center_link coordinates and transformed with the freshest
  // localization pose for every safety evaluation.
  std::vector<std::pair<double, double>> footprint_polygon_local_;
  std::optional<avg_msgs::msg::AvgPath> local_path_;
  double forward_speed_mps_{0.0};
  std::string drop_zone_phase_;
  std::string campsite_phase_;
  bool latch_active_{false};
  std::optional<LatchContext> latch_context_;
  std::optional<double> clear_since_sec_;
  std::optional<double> clear_merged_evidence_start_sec_;
  std::optional<double> clear_source_evidence_start_sec_;
  double hold_until_sec_{0.0};
  std::string latch_reason_;
};

}  // namespace camrod_control
