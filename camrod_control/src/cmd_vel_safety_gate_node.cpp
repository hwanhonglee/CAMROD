// HH_260721 - Assemble final cmd_vel authorization from explicit native C++ policies.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "avg_msgs/msg/avg_bool.hpp"
#include "avg_msgs/msg/avg_localization_mode.hpp"
#include "avg_msgs/msg/avg_occupancy_grid.hpp"
#include "avg_msgs/msg/avg_odometry.hpp"
#include "avg_msgs/msg/avg_path.hpp"
#include "avg_msgs/msg/avg_platform_status.hpp"
#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "avg_msgs/msg/avg_twist.hpp"
#include "avg_msgs/msg/module_state.hpp"
#include "avg_msgs/msg/planning_mission_key.hpp"
#include "camrod_control/charging_mission_override.hpp"
#include "camrod_control/cmd_vel_gate_policy.hpp"
#include "camrod_control/motion_geometry.hpp"
#include "camrod_control/motion_cost_stop.hpp"
#include "camrod_control/ros_message_conversion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "yaml-cpp/yaml.h"

namespace camrod_control
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string join(const std::vector<std::string> & values, const std::string & separator)
{
  std::ostringstream output;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0U) {
      output << separator;
    }
    output << values[index];
  }
  return output.str();
}

std::set<std::string> parseLabelSet(
  const std::string & value, const std::set<std::string> & fallback)
{
  std::set<std::string> labels;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    const auto normalized = MotionCostStop::normalizeLabel(item);
    const auto first = normalized.find_first_not_of(" \t\r\n");
    const auto last = normalized.find_last_not_of(" \t\r\n");
    if (first != std::string::npos) {
      labels.insert(normalized.substr(first, last - first + 1U));
    }
  }
  return labels.empty() ? fallback : labels;
}

std::vector<std::string> parseTopicList(const std::string & value)
{
  std::vector<std::string> topics;
  std::set<std::string> seen;
  std::stringstream stream(value);
  std::string topic;
  while (std::getline(stream, topic, ',')) {
    const auto first = topic.find_first_not_of(" \t\r\n");
    const auto last = topic.find_last_not_of(" \t\r\n");
    if (first == std::string::npos) {
      continue;
    }
    topic = topic.substr(first, last - first + 1U);
    if (seen.insert(topic).second) {
      topics.push_back(topic);
    }
  }
  return topics;
}

double quaternionYaw(const avg_msgs::msg::AvgQuaternion & orientation)
{
  const double sine = 2.0 *
    (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cosine = 1.0 - 2.0 *
    (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sine, cosine);
}

std::string phaseFromStatus(const std::string & message)
{
  std::stringstream stream(message);
  std::string token;
  while (stream >> token) {
    if (token.rfind("phase=", 0) == 0) {
      return token.substr(6U);
    }
  }
  return {};
}

}  // namespace

struct YawAlignmentZone
{
  std::string id;
  double x{0.0};
  double y{0.0};
  double target_yaw{0.0};
  double activation_radius_m{1.2};
  double lock_radius_m{0.7};
  double position_tolerance_m{0.1};
  double yaw_tolerance_rad{8.0 * kPi / 180.0};
  double yaw_tolerance_per_meter_rad{4.0 * kPi / 180.0};
  double hold_s{0.5};
  double angular_kp{1.8};
  double max_angular_z{0.8};
  double max_approach_linear_x{0.25};
};

class CmdVelSafetyGateNode final : public rclcpp::Node
{
public:
  CmdVelSafetyGateNode()
  : Node("cmd_vel_safety_gate")
  {
    // HH_260721 - Load command topology and final gate policy from one synchronized config.
    input_topic_ = declare_parameter<std::string>("input_topic", "/control/cmd_vel_raw");
    navigation_input_topic_ = declare_parameter<std::string>(
      "navigation_input_topic", "/control/nav2_cmd_vel_ros");
    output_topic_ = declare_parameter<std::string>("output_topic", "/control/cmd_vel");
    ros_output_topic_ = declare_parameter<std::string>(
      "ros_output_topic", "/control/cmd_vel_ros");
    engage_topic_ = declare_parameter<std::string>("engage_topic", "/planning/engage");
    mission_engage_topic_ = declare_parameter<std::string>(
      "mission_engage_topic", "/planning/mission_engage");
    mission_request_topic_ = declare_parameter<std::string>(
      "mission_request_topic", "/planning/mission_key");
    state_topic_ = declare_parameter<std::string>("state_topic", "/control/command_enabled");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/control/cmd_vel_safety_gate/status");
    platform_drive_enable_topic_ = declare_parameter<std::string>(
      "platform_drive_enable_topic", "/platform/drive_enable");

    if (engage_topic_ == state_topic_) {
      RCLCPP_WARN(get_logger(), "engage_topic matched state_topic; using /planning/engage");
      engage_topic_ = "/planning/engage";
    }

    loadGatePolicyConfig();
    loadChargingMissionOverrideConfig();
    loadMotionCostStopConfig();
    loadAlignmentConfig();
    loadRuntimeConfig();
    createPublishersAndSubscriptions();

    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&CmdVelSafetyGateNode::publishState, this));
    recreateCommandTimeoutTimer();
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&CmdVelSafetyGateNode::onSetParameters, this, std::placeholders::_1));

    loadYawAlignmentZones();
    publishState();
    RCLCPP_INFO(
      get_logger(),
      "cmd_vel safety gate ready: maneuver_in=%s nav2_in=%s out=%s driver_out=%s",
      input_topic_.c_str(), navigation_input_topic_.c_str(), output_topic_.c_str(),
      ros_output_topic_.c_str());
  }

private:
  void loadGatePolicyConfig()
  {
    require_platform_drive_enable_ = declare_parameter<bool>(
      "require_platform_drive_enable", true);
    const std::string platform_mode = MotionCostStop::normalizeLabel(
      declare_parameter<std::string>("platform_safety_source_mode", "platform_status"));
    platform_safety_enabled_ = platform_mode != "disabled" && platform_mode != "off" &&
      platform_mode != "none";
    platform_status_topic_ = declare_parameter<std::string>(
      "platform_status_topic", "/platform/status");

    gate_config_.require_platform_drive_enable = require_platform_drive_enable_;
    gate_config_.platform_safety_enabled = platform_safety_enabled_;
    gate_config_.platform_status_timeout_s = declare_parameter<double>(
      "platform_status_timeout_s", 0.5);
    gate_config_.block_on_platform_status_stale = declare_parameter<bool>(
      "block_on_platform_status_stale", true);
    gate_config_.block_on_charging = declare_parameter<bool>("block_on_charging", true);
    gate_config_.block_on_platform_error_code = declare_parameter<bool>(
      "block_on_platform_error_code", true);
    gate_config_.require_can_control_mode = declare_parameter<bool>(
      "require_can_control_mode", true);
    gate_config_.critical_battery_stop_enabled = declare_parameter<bool>(
      "critical_battery_stop_enabled", true);
    gate_config_.critical_battery_percentage = declare_parameter<double>(
      "critical_battery_percentage", 0.10);
    gate_policy_.setConfig(gate_config_);
  }

  void loadChargingMissionOverrideConfig()
  {
    // HH_260721 - Map existing field parameters to the explicit charging motion override model.
    charging_override_config_.allow_motion_while_charging = declare_parameter<bool>(
      "allow_mission_departure_while_charging", true);
    charging_override_config_.duration_s = declare_parameter<double>(
      "charging_departure_grace_s", 15.0);
    charging_override_config_.request_dedup_s = declare_parameter<double>(
      "mission_request_dedup_s", 1.0);
    charging_override_config_.mission_prefixes = parseLabelSet(
      declare_parameter<std::string>("charger_departure_mission_prefixes", "camping_site_"),
      {"camping_site_"});
    charging_mission_override_.setConfig(charging_override_config_);
  }

  // HH_260721 - Load all translation, crab, zero-turn, lanelet, and latch stop settings together.
  void loadMotionCostStopConfig()
  {
    // HH_260721 - Preserve every field-tuned obstacle and lanelet parameter in the native guard.
    cost_grid_topic_ = declare_parameter<std::string>(
      "cost_grid_topic", "/planning/cost_grid/inflation");
    lanelet_grid_topic_ = declare_parameter<std::string>(
      "lanelet_safety_grid_topic", "/map/cost_grid/lanelet");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/localization/pose");
    odometry_topic_ = declare_parameter<std::string>(
      "odometry_topic", "/localization/fallback/odometry");
    pose_source_preference_ = MotionCostStop::normalizeLabel(
      declare_parameter<std::string>("pose_source_preference", "odometry"));
    enable_pose_raw_fallback_ = declare_parameter<bool>("enable_pose_raw_fallback", false);
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "robot_base_link");

    motion_cost_stop_config_.enabled = declare_parameter<bool>("enable_cost_stop", true);
    motion_cost_stop_config_.cost_stop_threshold =
      declare_parameter<int>("cost_stop_threshold", 85);
    motion_cost_stop_config_.fixed_front_lookahead_m = declare_parameter<double>(
      "cost_stop_lookahead_m", 2.0);
    motion_cost_stop_config_.front_width_m = declare_parameter<double>("cost_stop_width_m", 1.27);
    motion_cost_stop_config_.stop_hold_s = declare_parameter<double>("cost_stop_hold_s", 1.0);
    motion_cost_stop_config_.latch_enabled =
      declare_parameter<bool>("cost_stop_latch_enable", true);
    motion_cost_stop_config_.clear_required_s = declare_parameter<double>(
      "cost_stop_clear_required_s", 2.0);
    cost_stop_latch_log_interval_s_ = declare_parameter<double>(
      "cost_stop_latch_log_interval_s", 1.0);
    motion_cost_stop_config_.stale_stop_enabled = declare_parameter<bool>(
      "cost_grid_stale_stop_enable", true);
    motion_cost_stop_config_.stale_timeout_s = declare_parameter<double>(
      "cost_grid_stale_timeout_s", 1.0);
    cost_grid_stale_log_interval_s_ = declare_parameter<double>(
      "cost_grid_stale_log_interval_s", 1.0);

    cost_source_debug_enable_ = declare_parameter<bool>("cost_source_debug_enable", true);
    motion_cost_stop_config_.source_max_age_s = declare_parameter<double>(
      "cost_source_debug_max_age_s", 1.0);
    cost_source_topics_ = declare_parameter<std::vector<std::string>>(
      "cost_source_debug_topics",
      {"/map/cost_grid/lanelet", "/sensing/cost_grid/lidar", "/sensing/cost_grid/radar",
        "/planning/cost_grid/global_path"});
    cost_source_labels_ = declare_parameter<std::vector<std::string>>(
      "cost_source_debug_labels", {"lanelet", "lidar", "radar", "global_path"});
    motion_cost_stop_config_.require_dynamic_source = declare_parameter<bool>(
      "cost_stop_require_dynamic_source", true);
    motion_cost_stop_config_.dynamic_source_labels = parseLabelSet(
      declare_parameter<std::string>("cost_stop_dynamic_source_labels", "lidar,radar"),
      {"lidar", "radar"});
    motion_cost_stop_config_.dynamic_front_use_local_path = declare_parameter<bool>(
      "front_dynamic_stop_use_local_path", true);
    motion_cost_stop_config_.dynamic_front_path_width_m = declare_parameter<double>(
      "front_dynamic_path_width_m", motion_cost_stop_config_.front_width_m);
    motion_cost_stop_config_.dynamic_front_path_max_start_distance_m = declare_parameter<double>(
      "front_dynamic_path_max_start_distance_m", 1.5);

    motion_cost_stop_config_.use_speed_dependent_lookahead = declare_parameter<bool>(
      "enable_speed_dependent_lookahead", true);
    motion_cost_stop_config_.front_lookahead_min_m = declare_parameter<double>(
      "front_lookahead_min_m", 2.10);
    motion_cost_stop_config_.front_lookahead_max_m = declare_parameter<double>(
      "front_lookahead_max_m", 3.0);
    motion_cost_stop_config_.front_friction = declare_parameter<double>(
      "front_lookahead_friction",
      0.4);
    motion_cost_stop_config_.front_reaction_time_s = declare_parameter<double>(
      "front_reaction_time_s", 0.15);
    motion_cost_stop_config_.front_margin_m = declare_parameter<double>(
      "front_lookahead_margin_m",
      0.3);

    motion_cost_stop_config_.side_rear_enabled = declare_parameter<bool>(
      "enable_side_rear_cost_stop", true);
    motion_cost_stop_config_.body_near_enabled = declare_parameter<bool>(
      "enable_body_near_dynamic_stop", true);
    motion_cost_stop_config_.body_near_side_m = declare_parameter<double>(
      "body_near_side_lookahead_m", 0.75);
    motion_cost_stop_config_.body_near_rear_m = declare_parameter<double>(
      "body_near_rear_lookahead_m", 0.55);
    motion_cost_stop_config_.maneuver_body_near_side_m = declare_parameter<double>(
      "body_near_maneuver_side_lookahead_m", 0.55);
    motion_cost_stop_config_.maneuver_body_near_rear_m = declare_parameter<double>(
      "body_near_maneuver_rear_lookahead_m", 0.45);
    motion_cost_stop_config_.side_threshold = declare_parameter<int>("side_cost_threshold", 85);
    motion_cost_stop_config_.side_lookahead_m = declare_parameter<double>("side_lookahead_m", 1.2);
    motion_cost_stop_config_.side_width_m = declare_parameter<double>(
      "side_corridor_width_m",
      1.69160);
    motion_cost_stop_config_.rear_threshold = declare_parameter<int>("rear_cost_threshold", 85);
    motion_cost_stop_config_.rear_lookahead_m = declare_parameter<double>("rear_lookahead_m", 1.2);
    motion_cost_stop_config_.rear_width_m =
      declare_parameter<double>("rear_corridor_width_m", 1.27);
    motion_cost_stop_config_.min_translation_mps = declare_parameter<double>(
      "lanelet_safety_min_translation_mps", 0.02);
    motion_cost_stop_config_.static_lateral_bypass = declare_parameter<bool>(
      "lateral_cmd_bypass_static_cost_stop", true);
    motion_cost_stop_config_.static_lateral_bypass_min_mps = declare_parameter<double>(
      "lateral_cmd_bypass_min_mps", 0.02);
    motion_cost_stop_config_.static_reverse_bypass = declare_parameter<bool>(
      "reverse_cmd_bypass_static_cost_stop", true);
    motion_cost_stop_config_.static_reverse_bypass_min_mps = declare_parameter<double>(
      "reverse_cmd_bypass_min_mps", 0.02);
    // HH_260721 - Use side_cost_threshold as the single lateral dynamic-cost threshold.
    motion_cost_stop_config_.rotation_dynamic_stop = declare_parameter<bool>(
      "rotation_cmd_dynamic_obstacle_stop", true);
    motion_cost_stop_config_.rotation_radius_m = declare_parameter<double>(
      "rotation_cmd_dynamic_obstacle_radius_m", 1.5);
    motion_cost_stop_config_.rotation_threshold = declare_parameter<int>(
      "rotation_cmd_dynamic_obstacle_threshold", 85);
    motion_cost_stop_config_.unavoidable_stop_enabled = declare_parameter<bool>(
      "enable_unavoidable_stop", true);
    motion_cost_stop_config_.unavoidable_threshold = declare_parameter<int>(
      "unavoidable_lethal_threshold", 90);
    motion_cost_stop_config_.unavoidable_min_cells = declare_parameter<int>(
      "unavoidable_cluster_min_cells", 25);
    motion_cost_stop_config_.unavoidable_min_ratio = declare_parameter<double>(
      "unavoidable_cluster_min_ratio", 0.25);

    motion_cost_stop_config_.lanelet_enabled =
      declare_parameter<bool>("lanelet_safety_enable", true);
    motion_cost_stop_config_.lanelet_threshold = declare_parameter<int>(
      "lanelet_safety_threshold",
      85);
    motion_cost_stop_config_.lanelet_current_threshold = declare_parameter<int>(
      "lanelet_safety_current_threshold", 85);
    motion_cost_stop_config_.lanelet_lookahead_m = declare_parameter<double>(
      "lanelet_safety_lookahead_m", 1.0);
    motion_cost_stop_config_.lanelet_width_m = declare_parameter<double>(
      "lanelet_safety_width_m",
      0.8);
    motion_cost_stop_config_.lanelet_stop_on_unknown = declare_parameter<bool>(
      "lanelet_safety_stop_on_unknown", true);
    motion_cost_stop_config_.lanelet_allow_rotation = declare_parameter<bool>(
      "lanelet_safety_allow_rotation_in_place", true);
    motion_cost_stop_config_.lanelet_check_reverse = declare_parameter<bool>(
      "lanelet_safety_check_reverse", false);
    motion_cost_stop_config_.lanelet_check_lateral = declare_parameter<bool>(
      "lanelet_safety_check_lateral", false);
    motion_cost_stop_config_.lanelet_front_use_local_path = declare_parameter<bool>(
      "lanelet_safety_front_use_local_path", true);
    motion_cost_stop_config_.lanelet_path_max_start_distance_m = declare_parameter<double>(
      "lanelet_safety_front_path_max_start_distance_m", 1.5);
    motion_cost_stop_config_.lanelet_path_width_m = declare_parameter<double>(
      "lanelet_safety_front_path_width_m", 0.25);
    motion_cost_stop_config_.lanelet_front_path_allow_route_reentry = declare_parameter<bool>(
      "lanelet_safety_front_path_allow_route_reentry", true);
    motion_cost_stop_config_.lanelet_current_allow_route_reentry = declare_parameter<bool>(
      "lanelet_safety_current_allow_route_reentry", true);
    motion_cost_stop_config_.lanelet_route_reentry_max_distance_m = declare_parameter<double>(
      "lanelet_safety_current_route_reentry_max_distance_m", 4.0);
    motion_cost_stop_config_.lanelet_route_reentry_require_front_cmd = declare_parameter<bool>(
      "lanelet_safety_current_route_reentry_require_front_cmd", true);

    drop_zone_status_topic_ = declare_parameter<std::string>(
      "drop_zone_maneuver_controller_status_topic",
      "/control/drop_zone_maneuver_controller/status");
    motion_cost_stop_config_.drop_zone_static_bypass_phases = parseLabelSet(
      declare_parameter<std::string>(
        "drop_zone_maneuver_controller_static_bypass_phases",
        "EXIT_STRAIGHT,ALIGN_EXIT_YAW"),
      {"exit_straight", "align_exit_yaw"});
    campsite_status_topic_ = declare_parameter<std::string>(
      "camping_site_maneuver_controller_status_topic",
      "/control/camping_site_maneuver_controller/status");
    motion_cost_stop_config_.campsite_static_bypass_phases = parseLabelSet(
      declare_parameter<std::string>(
        "camping_site_maneuver_controller_static_bypass_phases",
        // HH_260721 - Match the explicit same-lanelet retrace alignment phase.
        "ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,ALIGN_RETRACE_YAW,REVERSE_OUT,CRAB_OUT"),
      {"align_entry_yaw", "reverse_in", "crab_in", "rotate_180",
        "align_retrace_yaw", "reverse_out", "crab_out"});
    motion_cost_stop_.setConfig(motion_cost_stop_config_);
  }

  void loadAlignmentConfig()
  {
    enable_yaw_alignment_zone_ = declare_parameter<bool>("enable_yaw_alignment_zone", false);
    yaw_alignment_frame_id_ = declare_parameter<std::string>("yaw_alignment_frame_id", "map");
    yaw_alignment_zones_file_ = declare_parameter<std::string>("yaw_alignment_zones_file", "");
    yaw_alignment_exit_margin_m_ = declare_parameter<double>(
      "yaw_alignment_exit_margin_m", 0.3);
    enable_route_heading_alignment_ = declare_parameter<bool>(
      "enable_route_heading_alignment", true);
    route_heading_path_topic_ = declare_parameter<std::string>(
      "route_heading_path_topic", "/planning/local_path");
    route_heading_frame_id_ = declare_parameter<std::string>(
      "route_heading_frame_id", "map");
    route_heading_min_cmd_x_mps_ = declare_parameter<double>(
      "route_heading_min_cmd_x_mps", 0.03);
    route_heading_lateral_cmd_epsilon_mps_ = declare_parameter<double>(
      "route_heading_lateral_cmd_epsilon_mps", 0.02);
    route_heading_lookahead_m_ = declare_parameter<double>("route_heading_lookahead_m", 2.0);
    route_heading_error_enter_rad_ = declare_parameter<double>(
      "route_heading_error_enter_deg", 75.0) * kPi / 180.0;
    route_heading_error_exit_rad_ = declare_parameter<double>(
      "route_heading_error_exit_deg", 35.0) * kPi / 180.0;
    route_heading_angular_kp_ = declare_parameter<double>("route_heading_angular_kp", 0.8);
    route_heading_max_angular_z_ = declare_parameter<double>(
      "route_heading_max_angular_z", 0.35);
    route_heading_max_linear_x_ = declare_parameter<double>(
      "route_heading_max_linear_x", 0.0);
    route_heading_min_path_points_ = declare_parameter<int>(
      "route_heading_min_path_points", 2);
  }

  void loadRuntimeConfig()
  {
    // HH_260721 - Keep stale input, e-stop, and EKF recovery holds at the final output boundary.
    additional_estop_topics_ = parseTopicList(
      declare_parameter<std::string>(
        "additional_estop_topics", "/planning/state_machine/estop"));
    const std::string dr_mode = MotionCostStop::normalizeLabel(
      declare_parameter<std::string>("dr_timeout_source_mode", "localization_monitor"));
    dr_timeout_enabled_ = dr_mode != "disabled" && dr_mode != "off" && dr_mode != "none";
    dr_timeout_topic_ = declare_parameter<std::string>(
      "dr_timeout_topic", "/localization/state/dr_timeout");
    const bool allow_on_start = declare_parameter<bool>("allow_on_start", false);
    gate_policy_.setManualEngage(allow_on_start);
    publish_zero_when_blocked_ = declare_parameter<bool>("publish_zero_when_blocked", true);
    speed_scale_ = declare_parameter<double>("speed_scale", 1.0);
    input_timeout_s_ = declare_parameter<double>("input_timeout_s", 0.35);
    zero_publish_rate_hz_ = declare_parameter<double>("zero_publish_rate_hz", 10.0);

    enable_gnss_recovery_hold_ = declare_parameter<bool>("enable_gnss_recovery_hold", true);
    localization_mode_topic_ = declare_parameter<std::string>(
      "localization_mode_topic", "/localization/mode");
    gnss_recovery_hold_s_ = declare_parameter<double>("gnss_recovery_hold_s", 2.0);
    gnss_recovery_min_source_s_ = declare_parameter<double>(
      "gnss_recovery_min_source_s", 0.5);
    gnss_recovery_hold_cooldown_s_ = declare_parameter<double>(
      "gnss_recovery_hold_cooldown_s", 5.0);
    gnss_recovery_source_mode_min_ = declare_parameter<int>(
      "gnss_recovery_source_mode_min", avg_msgs::msg::AvgLocalizationMode::DR_ONLY);
    gnss_recovery_target_mode_ = declare_parameter<int>(
      "gnss_recovery_target_mode", avg_msgs::msg::AvgLocalizationMode::NORMAL);
  }

  void createPublishersAndSubscriptions()
  {
    // HH_260721 - Keep standard ROS Twist only at explicit Nav2 and platform driver boundaries.
    command_publisher_ = create_publisher<avg_msgs::msg::AvgTwist>(output_topic_, 10);
    ros_command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(ros_output_topic_, 10);
    rclcpp::QoS state_qos(1);
    state_qos.reliable().transient_local();
    state_publisher_ = create_publisher<avg_msgs::msg::AvgBool>(state_topic_, state_qos);
    status_publisher_ = create_publisher<avg_msgs::msg::ModuleState>(status_topic_, state_qos);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    command_subscription_ = create_subscription<avg_msgs::msg::AvgTwist>(
      input_topic_, 10, [this](const avg_msgs::msg::AvgTwist::SharedPtr message) {
        onCommand(*message);
      });
    navigation_command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      navigation_input_topic_, 10, [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        onCommand(twistFromRos(*message));
      });
    manual_engage_subscription_ = create_subscription<avg_msgs::msg::AvgBool>(
      engage_topic_, 10, [this](const avg_msgs::msg::AvgBool::SharedPtr message) {
        gate_policy_.setManualEngage(message->data);
        onAuthorizationChanged("manual_engage");
      });
    mission_engage_subscription_ = create_subscription<avg_msgs::msg::AvgBool>(
      mission_engage_topic_, 10, [this](const avg_msgs::msg::AvgBool::SharedPtr message) {
        gate_policy_.setMissionEngage(message->data);
        if (!message->data) {
          charging_mission_override_.cancel();
        }
        onAuthorizationChanged("mission_engage");
      });
    mission_request_subscription_ = create_subscription<avg_msgs::msg::PlanningMissionKey>(
      mission_request_topic_, 10,
      std::bind(&CmdVelSafetyGateNode::onMissionRequest, this, std::placeholders::_1));

    if (require_platform_drive_enable_) {
      platform_drive_subscription_ = create_subscription<avg_msgs::msg::AvgBool>(
        platform_drive_enable_topic_, 10,
        [this](const avg_msgs::msg::AvgBool::SharedPtr message) {
          gate_policy_.setPlatformDriveEnable(message->data);
          onAuthorizationChanged("platform_drive_enable");
        });
    }
    if (platform_safety_enabled_) {
      platform_status_subscription_ = create_subscription<avg_msgs::msg::AvgPlatformStatus>(
        platform_status_topic_, 20,
        std::bind(&CmdVelSafetyGateNode::onPlatformStatus, this, std::placeholders::_1));
    }

    drop_zone_status_subscription_ = create_subscription<avg_msgs::msg::ModuleState>(
      drop_zone_status_topic_, 10,
      [this](const avg_msgs::msg::ModuleState::SharedPtr message) {
        drop_zone_phase_ = phaseFromStatus(message->message);
        motion_cost_stop_.setManeuverPhases(drop_zone_phase_, campsite_phase_);
      });
    campsite_status_subscription_ = create_subscription<avg_msgs::msg::ModuleState>(
      campsite_status_topic_, 10,
      [this](const avg_msgs::msg::ModuleState::SharedPtr message) {
        campsite_phase_ = phaseFromStatus(message->message);
        motion_cost_stop_.setManeuverPhases(drop_zone_phase_, campsite_phase_);
      });

    for (const auto & topic : additional_estop_topics_) {
      gate_policy_.setEstopSource(topic, false);
      estop_subscriptions_.push_back(
        create_subscription<avg_msgs::msg::AvgBool>(
          topic, 10, [this, topic](const avg_msgs::msg::AvgBool::SharedPtr message) {
            gate_policy_.setEstopSource(topic, message->data);
            onAuthorizationChanged("estop:" + topic);
          }));
    }
    if (dr_timeout_enabled_) {
      dr_timeout_subscription_ = create_subscription<avg_msgs::msg::AvgBool>(
        dr_timeout_topic_, 10, [this](const avg_msgs::msg::AvgBool::SharedPtr message) {
          gate_policy_.setDrTimeout(message->data);
          onAuthorizationChanged("dr_timeout");
        });
    }
    if (enable_gnss_recovery_hold_) {
      localization_mode_subscription_ = create_subscription<avg_msgs::msg::AvgLocalizationMode>(
        localization_mode_topic_, 20,
        std::bind(&CmdVelSafetyGateNode::onLocalizationMode, this, std::placeholders::_1));
    }

    if (motion_cost_stop_config_.enabled) {
      rclcpp::QoS cost_qos(10);
      cost_qos.reliable().transient_local();
      merged_grid_subscription_ = create_subscription<avg_msgs::msg::AvgOccupancyGrid>(
        cost_grid_topic_, cost_qos,
        [this](const avg_msgs::msg::AvgOccupancyGrid::SharedPtr message) {
          merged_grid_frame_ = message->header.frame_id;
          motion_cost_stop_.setMergedGrid(*message, nowSec());
          refreshMotionCostStopPose();
        });
      if (motion_cost_stop_config_.lanelet_enabled) {
        lanelet_grid_subscription_ = create_subscription<avg_msgs::msg::AvgOccupancyGrid>(
          lanelet_grid_topic_, cost_qos,
          [this](const avg_msgs::msg::AvgOccupancyGrid::SharedPtr message) {
            lanelet_grid_frame_ = message->header.frame_id;
            motion_cost_stop_.setLaneletGrid(*message, nowSec());
            refreshMotionCostStopPose();
          });
      }
      if (cost_source_debug_enable_ || motion_cost_stop_config_.require_dynamic_source) {
        for (std::size_t index = 0; index < cost_source_topics_.size(); ++index) {
          const std::string label = index < cost_source_labels_.size() ?
            cost_source_labels_[index] : cost_source_topics_[index];
          source_grid_subscriptions_.push_back(
            create_subscription<avg_msgs::msg::AvgOccupancyGrid>(
              cost_source_topics_[index], cost_qos,
              [this, label](const avg_msgs::msg::AvgOccupancyGrid::SharedPtr message) {
                motion_cost_stop_.setSourceGrid(label, *message, nowSec());
              }));
        }
      }
    }

    if (motion_cost_stop_config_.enabled || enable_yaw_alignment_zone_ ||
      enable_route_heading_alignment_)
    {
      pose_subscription_ = create_subscription<avg_msgs::msg::AvgPoseStamped>(
        pose_topic_, 10, [this](const avg_msgs::msg::AvgPoseStamped::SharedPtr message) {
          latest_pose_ = *message;
          refreshMotionCostStopPose();
        });
      odometry_subscription_ = create_subscription<avg_msgs::msg::AvgOdometry>(
        odometry_topic_, 10, [this](const avg_msgs::msg::AvgOdometry::SharedPtr message) {
          latest_odometry_ = *message;
          motion_cost_stop_.setOdometrySpeed(message->twist.twist.linear.x);
          refreshMotionCostStopPose();
        });
    }
    if (enable_route_heading_alignment_ || motion_cost_stop_config_.lanelet_front_use_local_path ||
      motion_cost_stop_config_.dynamic_front_use_local_path)
    {
      route_path_subscription_ = create_subscription<avg_msgs::msg::AvgPath>(
        route_heading_path_topic_, 10, [this](const avg_msgs::msg::AvgPath::SharedPtr message) {
          latest_path_ = *message;
          motion_cost_stop_.setLocalPath(*message);
        });
    }
  }

  void onCommand(avg_msgs::msg::AvgTwist command)
  {
    const double now_sec = nowSec();
    last_input_command_sec_ = now_sec;
    command_input_stale_ = false;
    refreshMotionCostStopPose();

    // HH_260721 - Re-evaluate a latched obstacle before authorization so clear time can advance.
    if (motion_cost_stop_.latched()) {
      const auto latch_decision = motion_cost_stop_.evaluate(command, now_sec);
      updatePolicyCostState();
      if (latch_decision.blocked) {
        publishZero();
        logMotionCostStopDecision(latch_decision, now_sec);
        return;
      }
    }

    if (!effectiveEnabled(now_sec)) {
      if (publish_zero_when_blocked_) {
        publishZero();
      }
      logBlockReasons(now_sec);
      return;
    }

    if (const auto yaw_override = applyYawAlignment(command); yaw_override.has_value()) {
      command = *yaw_override;
    } else if (const auto heading_override = applyRouteHeadingAlignment(command);
      heading_override.has_value())
    {
      command = *heading_override;
    }

    const auto cost_decision = motion_cost_stop_.evaluate(command, now_sec);
    updatePolicyCostState();
    if (cost_decision.blocked) {
      if (publish_zero_when_blocked_) {
        publishZero();
      }
      logMotionCostStopDecision(cost_decision, now_sec);
      return;
    }
    publishCommand(scaleCommand(command));
  }

  void onMissionRequest(const avg_msgs::msg::PlanningMissionKey::SharedPtr message)
  {
    MissionRequestIdentity request;
    request.mission_key = message->mission_key;
    request.source = message->source;
    request.stamp_sec = message->header.stamp.sec;
    request.stamp_nanosec = message->header.stamp.nanosec;
    if (!charging_mission_override_.activateForMission(request, nowSec())) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "charging motion override activated for mission=%s duration=%.1fs",
      message->mission_key.c_str(), charging_override_config_.duration_s);
    publishState();
  }

  void onPlatformStatus(const avg_msgs::msg::AvgPlatformStatus::SharedPtr message)
  {
    // HH_260721 - Consume normalized DBC state; this node never decodes raw CAN frames.
    PlatformSafetyState state;
    state.received = true;
    state.received_sec = nowSec();
    state.vehicle_state = message->vehicle_state;
    state.control_mode = message->control_mode;
    state.error_code = message->error_code;
    if (message->battery_state_available && std::isfinite(message->battery_percentage)) {
      state.battery_percentage = std::clamp(
        static_cast<double>(message->battery_percentage), 0.0, 1.0);
    }
    gate_policy_.setPlatformState(state);
    charging_mission_override_.setCharging(message->is_charging);
    gate_policy_.setEstopSource(platform_status_topic_, message->estop);
    publishState();
    if (!effectiveEnabled(nowSec()) && publish_zero_when_blocked_) {
      publishZero();
    }
  }

  void onLocalizationMode(const avg_msgs::msg::AvgLocalizationMode::SharedPtr message)
  {
    const int mode = message->value;
    const double now_sec = nowSec();
    const bool previous_source = last_localization_mode_.has_value() &&
      *last_localization_mode_ >= gnss_recovery_source_mode_min_;
    const bool current_source = mode >= gnss_recovery_source_mode_min_;
    if (current_source && !previous_source) {
      gnss_recovery_source_enter_sec_ = now_sec;
    }
    const std::optional<int> previous_mode = last_localization_mode_;
    last_localization_mode_ = mode;
    if (!previous_mode.has_value() || !enable_gnss_recovery_hold_ ||
      gnss_recovery_hold_s_ <= 0.0)
    {
      return;
    }
    const bool recovered = previous_source && mode == gnss_recovery_target_mode_;
    if (!recovered) {
      return;
    }
    const double source_duration = gnss_recovery_source_enter_sec_.has_value() ?
      now_sec - *gnss_recovery_source_enter_sec_ : gnss_recovery_min_source_s_;
    gnss_recovery_source_enter_sec_.reset();
    if (source_duration < gnss_recovery_min_source_s_ ||
      now_sec - last_gnss_recovery_hold_sec_ < gnss_recovery_hold_cooldown_s_)
    {
      return;
    }
    last_gnss_recovery_hold_sec_ = now_sec;
    gnss_recovery_hold_until_sec_ = std::max(
      gnss_recovery_hold_until_sec_, now_sec + gnss_recovery_hold_s_);
    gate_policy_.setGnssRecoveryHoldUntil(gnss_recovery_hold_until_sec_);
    RCLCPP_WARN(
      get_logger(), "cmd_vel held %.2fs after EKF localization recovery %d->%d",
      gnss_recovery_hold_s_, *previous_mode, mode);
    publishState();
    publishZero();
  }

  void onAuthorizationChanged(const std::string & source)
  {
    publishState();
    const double now_sec = nowSec();
    if (!effectiveEnabled(now_sec) && publish_zero_when_blocked_) {
      publishZero();
    }
    // HH_260721 - Log authorization only when its effective reason set changes.
    const auto reasons = gate_policy_.blockReasons(
      now_sec, charging_mission_override_.charging(), charging_mission_override_.isActive(now_sec));
    const std::string signature = reasons.empty() ? "enabled" : join(reasons, ",");
    if (signature != last_authorization_log_signature_) {
      last_authorization_log_signature_ = signature;
      RCLCPP_INFO(
        get_logger(), "command authorization updated: source=%s state=%s",
        source.c_str(), signature.c_str());
    }
  }

  bool effectiveEnabled(const double now_sec)
  {
    updatePolicyCostState();
    return gate_policy_.enabled(
      now_sec, charging_mission_override_.charging(), charging_mission_override_.isActive(now_sec));
  }

  void updatePolicyCostState()
  {
    gate_policy_.setCostState(motion_cost_stop_.latched(), motion_cost_stop_.holdUntilSec());
    gate_policy_.setGnssRecoveryHoldUntil(gnss_recovery_hold_until_sec_);
  }

  void publishState()
  {
    const double now_sec = nowSec();
    updatePolicyCostState();
    const bool charging_motion_override_active = charging_mission_override_.isActive(now_sec);
    const auto reasons = gate_policy_.blockReasons(
      now_sec, charging_mission_override_.charging(), charging_motion_override_active);
    const bool enabled = reasons.empty();

    avg_msgs::msg::AvgBool state;
    state.data = enabled;
    state_publisher_->publish(state);

    avg_msgs::msg::ModuleState status;
    status.stamp = now();
    status.module_name = "cmd_vel_safety_gate";
    const bool hard_fault = std::any_of(
      reasons.begin(), reasons.end(), [](const std::string & reason) {
        return reason == "estop" || reason.rfind("vehicle_state=", 0) == 0 ||
        reason.rfind("platform_error=", 0) == 0 ||
        reason.rfind("platform_status_", 0) == 0;
      });
    status.level = enabled ? avg_msgs::msg::ModuleState::OK :
      hard_fault ? avg_msgs::msg::ModuleState::ERROR : avg_msgs::msg::ModuleState::WARN;
    const std::string operating_state = charging_motion_override_active && enabled ?
      "DEPARTING_CHARGER" :
      enabled ? "ENABLED" :
      std::find(reasons.begin(), reasons.end(), "charging") != reasons.end() ? "CHARGING" :
      "BLOCKED";
    status.message = "state=" + operating_state + " reasons=" +
      (reasons.empty() ? "none" : join(reasons, ",")) +
      " charging=" + std::string(charging_mission_override_.charging() ? "true" : "false") +
      " battery=" + batteryText();
    status_publisher_->publish(status);
  }

  std::string batteryText() const
  {
    const auto & battery = gate_policy_.platformState().battery_percentage;
    if (!battery.has_value() || !std::isfinite(*battery)) {
      return "unknown";
    }
    std::ostringstream output;
    output << std::fixed << std::setprecision(1) << *battery * 100.0 << "%";
    return output.str();
  }

  void publishCommand(const avg_msgs::msg::AvgTwist & command)
  {
    command_publisher_->publish(command);
    ros_command_publisher_->publish(twistToRos(command));
  }

  void publishZero()
  {
    publishCommand(avg_msgs::msg::AvgTwist{});
  }

  avg_msgs::msg::AvgTwist scaleCommand(const avg_msgs::msg::AvgTwist & input) const
  {
    avg_msgs::msg::AvgTwist output = input;
    output.linear.x *= speed_scale_;
    output.linear.y *= speed_scale_;
    output.linear.z *= speed_scale_;
    output.angular.x *= speed_scale_;
    output.angular.y *= speed_scale_;
    output.angular.z *= speed_scale_;
    return output;
  }

  void recreateCommandTimeoutTimer()
  {
    if (command_timeout_timer_) {
      command_timeout_timer_->cancel();
    }
    const double rate_hz = std::max(1.0, zero_publish_rate_hz_);
    command_timeout_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this]() {
        if (!last_input_command_sec_.has_value() || input_timeout_s_ <= 0.0) {
          return;
        }
        const double age_s = nowSec() - *last_input_command_sec_;
        if (age_s <= input_timeout_s_) {
          return;
        }
        if (publish_zero_when_blocked_) {
          publishZero();
        }
        if (!command_input_stale_) {
          command_input_stale_ = true;
          RCLCPP_WARN(
            get_logger(), "cmd_vel input stale: age=%.2fs timeout=%.2fs; publishing zero",
            age_s, input_timeout_s_);
        }
      });
  }

  void refreshMotionCostStopPose()
  {
    std::string target_frame =
      !merged_grid_frame_.empty() ? merged_grid_frame_ : lanelet_grid_frame_;
    if (target_frame.empty()) {
      target_frame = route_heading_frame_id_;
    }
    if (const auto pose = resolvePose(target_frame); pose.has_value()) {
      motion_cost_stop_.setPose(*pose);
    }
  }

  std::optional<PlanarPose> resolvePose(const std::string & target_frame)
  {
    // HH_260721 - Honor the configured exclusive pose priority before using TF fallback.
    const std::vector<std::string> priorities = pose_source_preference_ == "tf_robot_base" ?
      std::vector<std::string>{"tf_robot_base", "odometry", "pose_topic"} :
    pose_source_preference_ == "pose_topic" ?
    std::vector<std::string>{"pose_topic", "tf_robot_base", "odometry"} :
    std::vector<std::string>{"odometry", "tf_robot_base", "pose_topic"};
    for (const auto & source : priorities) {
      if (source == "pose_topic" && latest_pose_.has_value()) {
        PlanarPose pose;
        pose.x = latest_pose_->pose.position.x;
        pose.y = latest_pose_->pose.position.y;
        pose.yaw = quaternionYaw(latest_pose_->pose.orientation);
        pose.frame_id = latest_pose_->header.frame_id;
        pose.source = "pose_topic";
        if (const auto transformed = transformPose(pose, target_frame); transformed.has_value()) {
          return transformed;
        }
      }
      if (source == "odometry" && latest_odometry_.has_value()) {
        PlanarPose pose;
        pose.x = latest_odometry_->pose.pose.position.x;
        pose.y = latest_odometry_->pose.pose.position.y;
        pose.yaw = quaternionYaw(latest_odometry_->pose.pose.orientation);
        pose.frame_id = latest_odometry_->header.frame_id;
        pose.source = "odometry";
        if (const auto transformed = transformPose(pose, target_frame); transformed.has_value()) {
          return transformed;
        }
      }
      if (source == "tf_robot_base" && tf_buffer_) {
        try {
          const auto transform = tf_buffer_->lookupTransform(
            target_frame, robot_base_frame_, tf2::TimePointZero);
          PlanarPose pose;
          pose.x = transform.transform.translation.x;
          pose.y = transform.transform.translation.y;
          const auto & q = transform.transform.rotation;
          pose.yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
          pose.frame_id = target_frame;
          pose.source = "tf_robot_base";
          return pose;
        } catch (const tf2::TransformException &) {
          continue;
        }
      }
    }
    return std::nullopt;
  }

  std::optional<PlanarPose> transformPose(
    const PlanarPose & source,
    const std::string & target_frame)
  {
    if (target_frame.empty() || source.frame_id.empty() || source.frame_id == target_frame) {
      PlanarPose output = source;
      if (output.frame_id.empty()) {
        output.frame_id = target_frame;
      }
      return output;
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(
        target_frame, source.frame_id, tf2::TimePointZero);
      const auto & q = transform.transform.rotation;
      const double transform_yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      PlanarPose output;
      output.x = transform.transform.translation.x +
        std::cos(transform_yaw) * source.x - std::sin(transform_yaw) * source.y;
      output.y = transform.transform.translation.y +
        std::sin(transform_yaw) * source.x + std::cos(transform_yaw) * source.y;
      output.yaw = normalizeAngle(transform_yaw + source.yaw);
      output.frame_id = target_frame;
      output.source = source.source + "_tf";
      return output;
    } catch (const tf2::TransformException &) {
      if (enable_pose_raw_fallback_) {
        return source;
      }
      return std::nullopt;
    }
  }

  std::optional<avg_msgs::msg::AvgTwist> applyRouteHeadingAlignment(
    const avg_msgs::msg::AvgTwist & command)
  {
    if (!enable_route_heading_alignment_ || !latest_path_.has_value() ||
      motion_cost_stop_config_.drop_zone_static_bypass_phases.count(
        MotionCostStop::normalizeLabel(drop_zone_phase_)) > 0U ||
      motion_cost_stop_config_.campsite_static_bypass_phases.count(
        MotionCostStop::normalizeLabel(campsite_phase_)) > 0U ||
      std::abs(command.linear.y) > route_heading_lateral_cmd_epsilon_mps_ ||
      command.linear.x < -route_heading_min_cmd_x_mps_)
    {
      route_heading_active_ = false;
      return std::nullopt;
    }
    if (!route_heading_active_ && command.linear.x <= route_heading_min_cmd_x_mps_) {
      return std::nullopt;
    }
    if (latest_path_->poses.size() < static_cast<std::size_t>(
        std::max(2, route_heading_min_path_points_)))
    {
      route_heading_active_ = false;
      return std::nullopt;
    }
    const std::string path_frame = latest_path_->header.frame_id.empty() ?
      route_heading_frame_id_ : latest_path_->header.frame_id;
    const auto pose = resolvePose(path_frame);
    if (!pose.has_value()) {
      route_heading_active_ = false;
      return std::nullopt;
    }
    const auto heading = routeHeading(*latest_path_, *pose);
    if (!heading.has_value()) {
      route_heading_active_ = false;
      return std::nullopt;
    }
    const double yaw_error = normalizeAngle(*heading - pose->yaw);
    if (route_heading_active_) {
      if (std::abs(yaw_error) <= route_heading_error_exit_rad_) {
        route_heading_active_ = false;
        return std::nullopt;
      }
    } else if (std::abs(yaw_error) >= route_heading_error_enter_rad_) {
      route_heading_active_ = true;
    } else {
      return std::nullopt;
    }
    avg_msgs::msg::AvgTwist output;
    output.linear.x = std::clamp(
      command.linear.x, 0.0, std::max(0.0, route_heading_max_linear_x_));
    output.angular.z = std::clamp(
      route_heading_angular_kp_ * yaw_error,
      -std::abs(route_heading_max_angular_z_), std::abs(route_heading_max_angular_z_));
    return output;
  }

  std::optional<double> routeHeading(
    const avg_msgs::msg::AvgPath & path, const PlanarPose & pose) const
  {
    std::vector<std::pair<double, double>> points;
    for (const auto & stamped_pose : path.poses) {
      if (std::isfinite(stamped_pose.pose.position.x) &&
        std::isfinite(stamped_pose.pose.position.y))
      {
        points.emplace_back(stamped_pose.pose.position.x, stamped_pose.pose.position.y);
      }
    }
    if (points.size() < 2U) {
      return std::nullopt;
    }
    std::size_t closest_index = 0U;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < points.size(); ++index) {
      const double dx = points[index].first - pose.x;
      const double dy = points[index].second - pose.y;
      const double projection = std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy;
      const double distance = std::hypot(dx, dy) + (projection < -0.30 ? 1000.0 : 0.0);
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_index = index;
      }
    }
    std::size_t target_index = closest_index;
    double accumulated = 0.0;
    for (std::size_t index = closest_index; index + 1U < points.size(); ++index) {
      accumulated += std::hypot(
        points[index + 1U].first - points[index].first,
        points[index + 1U].second - points[index].second);
      target_index = index + 1U;
      if (accumulated >= std::max(0.1, route_heading_lookahead_m_)) {
        break;
      }
    }
    std::size_t start_index = closest_index;
    if (target_index == closest_index) {
      if (closest_index == 0U) {
        return std::nullopt;
      }
      start_index = closest_index - 1U;
    }
    const double dx = points[target_index].first - points[start_index].first;
    const double dy = points[target_index].second - points[start_index].second;
    return std::hypot(dx, dy) > 1.0e-4 ?
           std::optional<double>(std::atan2(dy, dx)) : std::nullopt;
  }

  void loadYawAlignmentZones()
  {
    // HH_260721 - Preserve optional per-zone heading locks using yaml-cpp.
    yaw_zones_.clear();
    active_yaw_zone_.reset();
    yaw_zone_hold_start_sec_.reset();
    yaw_zone_satisfied_ = false;
    if (!enable_yaw_alignment_zone_ || yaw_alignment_zones_file_.empty()) {
      return;
    }
    try {
      YAML::Node root = YAML::LoadFile(yaw_alignment_zones_file_);
      YAML::Node container = root["yaw_alignment_zones"] ? root["yaw_alignment_zones"] : root;
      if (container["frame_id"]) {
        yaw_alignment_frame_id_ = container["frame_id"].as<std::string>();
      }
      const YAML::Node zones = container["zones"];
      if (!zones || !zones.IsSequence()) {
        return;
      }
      for (std::size_t index = 0U; index < zones.size(); ++index) {
        const YAML::Node source = zones[index];
        if (!source["x"] || !source["y"]) {
          continue;
        }
        YawAlignmentZone zone;
        zone.id = source["id"] ? source["id"].as<std::string>() :
          "zone_" + std::to_string(index + 1U);
        zone.x = source["x"].as<double>();
        zone.y = source["y"].as<double>();
        if (source["yaw_deg"]) {
          zone.target_yaw = source["yaw_deg"].as<double>() * kPi / 180.0;
        } else if (source["next_x"] && source["next_y"]) {
          zone.target_yaw = std::atan2(
            source["next_y"].as<double>() - zone.y,
            source["next_x"].as<double>() - zone.x);
        } else {
          continue;
        }
        zone.position_tolerance_m = std::max(
          0.05, source["position_tolerance_m"] ?
          source["position_tolerance_m"].as<double>() : 0.10);
        zone.activation_radius_m = std::max(
          zone.position_tolerance_m + 0.05,
          source["activation_radius_m"] ? source["activation_radius_m"].as<double>() : 1.2);
        zone.lock_radius_m = std::max(
          zone.position_tolerance_m,
          source["lock_radius_m"] ? source["lock_radius_m"].as<double>() :
          std::max(zone.position_tolerance_m + 0.15, zone.activation_radius_m * 0.6));
        zone.yaw_tolerance_rad = std::max(
          1.0, source["yaw_tolerance_deg"] ? source["yaw_tolerance_deg"].as<double>() : 8.0) *
          kPi / 180.0;
        zone.yaw_tolerance_per_meter_rad = std::max(
          0.0, source["yaw_tolerance_per_meter_deg"] ?
          source["yaw_tolerance_per_meter_deg"].as<double>() : 4.0) * kPi / 180.0;
        zone.hold_s = std::max(0.0, source["hold_s"] ? source["hold_s"].as<double>() : 0.5);
        zone.angular_kp = std::max(
          0.1, source["angular_kp"] ? source["angular_kp"].as<double>() : 1.8);
        zone.max_angular_z = std::max(
          0.05, source["max_angular_z"] ? source["max_angular_z"].as<double>() : 0.8);
        zone.max_approach_linear_x = std::max(
          0.0, source["max_approach_linear_x"] ?
          source["max_approach_linear_x"].as<double>() : 0.25);
        yaw_zones_.push_back(zone);
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "failed to load yaw alignment zones: %s", error.what());
    }
  }

  std::optional<avg_msgs::msg::AvgTwist> applyYawAlignment(
    const avg_msgs::msg::AvgTwist & command)
  {
    if (!enable_yaw_alignment_zone_ || yaw_zones_.empty()) {
      return std::nullopt;
    }
    const auto pose = resolvePose(yaw_alignment_frame_id_);
    if (!pose.has_value()) {
      return std::nullopt;
    }
    const YawAlignmentZone * zone = selectYawZone(pose->x, pose->y);
    if (zone == nullptr) {
      active_yaw_zone_.reset();
      yaw_zone_hold_start_sec_.reset();
      yaw_zone_satisfied_ = false;
      return std::nullopt;
    }
    if (!active_yaw_zone_.has_value() || *active_yaw_zone_ != zone->id) {
      active_yaw_zone_ = zone->id;
      yaw_zone_hold_start_sec_.reset();
      yaw_zone_satisfied_ = false;
    }
    if (yaw_zone_satisfied_) {
      return std::nullopt;
    }
    const double distance = std::hypot(pose->x - zone->x, pose->y - zone->y);
    if (distance > zone->lock_radius_m) {
      yaw_zone_hold_start_sec_.reset();
      return std::nullopt;
    }
    const double yaw_error = normalizeAngle(zone->target_yaw - pose->yaw);
    const double tolerance = zone->yaw_tolerance_rad +
      zone->yaw_tolerance_per_meter_rad *
      std::max(0.0, distance - zone->position_tolerance_m);
    const bool aligned = std::abs(yaw_error) <= tolerance &&
      distance <= zone->position_tolerance_m;
    if (aligned) {
      if (!yaw_zone_hold_start_sec_.has_value()) {
        yaw_zone_hold_start_sec_ = nowSec();
      }
      if (nowSec() - *yaw_zone_hold_start_sec_ >= zone->hold_s) {
        yaw_zone_satisfied_ = true;
        return std::nullopt;
      }
    } else {
      yaw_zone_hold_start_sec_.reset();
    }
    avg_msgs::msg::AvgTwist output;
    output.angular.z = std::clamp(
      zone->angular_kp * (1.0 + std::min(1.0, distance)) * yaw_error,
      -zone->max_angular_z, zone->max_angular_z);
    if (distance > zone->position_tolerance_m) {
      const double yaw_scale = std::max(
        0.0, 1.0 - std::abs(yaw_error) / std::max(0.01, zone->yaw_tolerance_rad * 2.5));
      output.linear.x = std::clamp(
        command.linear.x, -zone->max_approach_linear_x, zone->max_approach_linear_x) * yaw_scale;
    }
    return output;
  }

  const YawAlignmentZone * selectYawZone(const double x, const double y) const
  {
    if (active_yaw_zone_.has_value()) {
      for (const auto & zone : yaw_zones_) {
        if (zone.id == *active_yaw_zone_ &&
          std::hypot(x - zone.x, y - zone.y) <=
          zone.activation_radius_m + yaw_alignment_exit_margin_m_)
        {
          return &zone;
        }
      }
    }
    const YawAlignmentZone * selected = nullptr;
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto & zone : yaw_zones_) {
      const double distance = std::hypot(x - zone.x, y - zone.y);
      if (distance <= zone.activation_radius_m && distance < nearest) {
        selected = &zone;
        nearest = distance;
      }
    }
    return selected;
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    // HH_260721 - Retain runtime tuning for safety thresholds without changing topic topology.
    bool recreate_timeout_timer = false;
    bool reload_yaw_zones = false;
    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();
      if (name == "speed_scale") {
        speed_scale_ = parameter.as_double();
      } else if (name == "input_timeout_s") {
        input_timeout_s_ = parameter.as_double();
      } else if (name == "zero_publish_rate_hz") {
        zero_publish_rate_hz_ = parameter.as_double();
        recreate_timeout_timer = true;
      } else if (name == "publish_zero_when_blocked") {
        publish_zero_when_blocked_ = parameter.as_bool();
      } else if (name == "cost_stop_threshold") {
        motion_cost_stop_config_.cost_stop_threshold = parameter.as_int();
      } else if (name == "cost_stop_hold_s") {
        motion_cost_stop_config_.stop_hold_s = parameter.as_double();
      } else if (name == "cost_stop_latch_enable") {
        motion_cost_stop_config_.latch_enabled = parameter.as_bool();
      } else if (name == "cost_stop_clear_required_s") {
        motion_cost_stop_config_.clear_required_s = parameter.as_double();
      } else if (name == "cost_grid_stale_stop_enable") {
        motion_cost_stop_config_.stale_stop_enabled = parameter.as_bool();
      } else if (name == "cost_grid_stale_timeout_s") {
        motion_cost_stop_config_.stale_timeout_s = parameter.as_double();
      } else if (name == "cost_stop_lookahead_m") {
        motion_cost_stop_config_.fixed_front_lookahead_m = parameter.as_double();
      } else if (name == "cost_stop_width_m") {
        motion_cost_stop_config_.front_width_m = parameter.as_double();
      } else if (name == "cost_stop_require_dynamic_source") {
        motion_cost_stop_config_.require_dynamic_source = parameter.as_bool();
      } else if (name == "cost_stop_dynamic_source_labels") {
        motion_cost_stop_config_.dynamic_source_labels = parseLabelSet(
          parameter.as_string(), {"lidar", "radar"});
      } else if (name == "front_dynamic_stop_use_local_path") {
        motion_cost_stop_config_.dynamic_front_use_local_path = parameter.as_bool();
      } else if (name == "front_dynamic_path_width_m") {
        motion_cost_stop_config_.dynamic_front_path_width_m = parameter.as_double();
      } else if (name == "lanelet_safety_enable") {
        motion_cost_stop_config_.lanelet_enabled = parameter.as_bool();
      } else if (name == "lanelet_safety_threshold") {
        motion_cost_stop_config_.lanelet_threshold = parameter.as_int();
      } else if (name == "lanelet_safety_current_threshold") {
        motion_cost_stop_config_.lanelet_current_threshold = parameter.as_int();
      } else if (name == "lanelet_safety_lookahead_m") {
        motion_cost_stop_config_.lanelet_lookahead_m = parameter.as_double();
      } else if (name == "lanelet_safety_width_m") {
        motion_cost_stop_config_.lanelet_width_m = parameter.as_double();
      } else if (name == "lanelet_safety_stop_on_unknown") {
        motion_cost_stop_config_.lanelet_stop_on_unknown = parameter.as_bool();
      } else if (name == "enable_speed_dependent_lookahead") {
        motion_cost_stop_config_.use_speed_dependent_lookahead = parameter.as_bool();
      } else if (name == "front_lookahead_min_m") {
        motion_cost_stop_config_.front_lookahead_min_m = parameter.as_double();
      } else if (name == "front_lookahead_max_m") {
        motion_cost_stop_config_.front_lookahead_max_m = parameter.as_double();
      } else if (name == "front_lookahead_friction") {
        motion_cost_stop_config_.front_friction = parameter.as_double();
      } else if (name == "front_reaction_time_s") {
        motion_cost_stop_config_.front_reaction_time_s = parameter.as_double();
      } else if (name == "front_lookahead_margin_m") {
        motion_cost_stop_config_.front_margin_m = parameter.as_double();
      } else if (name == "side_cost_threshold") {
        motion_cost_stop_config_.side_threshold = parameter.as_int();
      } else if (name == "rear_cost_threshold") {
        motion_cost_stop_config_.rear_threshold = parameter.as_int();
      } else if (name == "rotation_cmd_dynamic_obstacle_stop") {
        motion_cost_stop_config_.rotation_dynamic_stop = parameter.as_bool();
      } else if (name == "rotation_cmd_dynamic_obstacle_radius_m") {
        motion_cost_stop_config_.rotation_radius_m = parameter.as_double();
      } else if (name == "rotation_cmd_dynamic_obstacle_threshold") {
        motion_cost_stop_config_.rotation_threshold = parameter.as_int();
      } else if (name == "enable_unavoidable_stop") {
        motion_cost_stop_config_.unavoidable_stop_enabled = parameter.as_bool();
      } else if (name == "enable_yaw_alignment_zone") {
        enable_yaw_alignment_zone_ = parameter.as_bool();
        reload_yaw_zones = true;
      } else if (name == "yaw_alignment_zones_file") {
        yaw_alignment_zones_file_ = parameter.as_string();
        reload_yaw_zones = true;
      } else if (name == "enable_route_heading_alignment") {
        enable_route_heading_alignment_ = parameter.as_bool();
        route_heading_active_ = false;
      }
    }
    motion_cost_stop_.setConfig(motion_cost_stop_config_);
    if (recreate_timeout_timer) {
      recreateCommandTimeoutTimer();
    }
    if (reload_yaw_zones) {
      loadYawAlignmentZones();
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void logBlockReasons(const double now_sec)
  {
    if (now_sec - last_block_log_sec_ < 2.0) {
      return;
    }
    last_block_log_sec_ = now_sec;
    const auto reasons = gate_policy_.blockReasons(
      now_sec, charging_mission_override_.charging(), charging_mission_override_.isActive(now_sec));
    RCLCPP_WARN(
      get_logger(), "cmd_vel BLOCKED: %s",
      reasons.empty() ? "unknown" : join(reasons, ", ").c_str());
  }

  void logMotionCostStopDecision(
    const MotionCostStopDecision & decision, const double now_sec)
  {
    const double interval = decision.stale_grid ? cost_grid_stale_log_interval_s_ :
      cost_stop_latch_log_interval_s_;
    if (decision.reason == last_cost_reason_ && now_sec - last_cost_log_sec_ < interval) {
      return;
    }
    last_cost_reason_ = decision.reason;
    last_cost_log_sec_ = now_sec;
    RCLCPP_WARN(get_logger(), "cmd_vel cost stop: %s", decision.reason.c_str());
  }

  // HH_260721 - Read ROS time through the non-const Humble clock API.
  double nowSec()
  {
    return get_clock()->now().seconds();
  }

  CmdVelGatePolicyConfig gate_config_;
  ChargingMissionOverrideConfig charging_override_config_;
  MotionCostStopConfig motion_cost_stop_config_;
  CmdVelGatePolicy gate_policy_;
  ChargingMissionOverride charging_mission_override_;
  MotionCostStop motion_cost_stop_;

  std::string input_topic_;
  std::string navigation_input_topic_;
  std::string output_topic_;
  std::string ros_output_topic_;
  std::string engage_topic_;
  std::string mission_engage_topic_;
  std::string mission_request_topic_;
  std::string state_topic_;
  std::string status_topic_;
  std::string platform_drive_enable_topic_;
  std::string platform_status_topic_;
  std::string drop_zone_status_topic_;
  std::string campsite_status_topic_;
  std::string cost_grid_topic_;
  std::string lanelet_grid_topic_;
  std::string pose_topic_;
  std::string odometry_topic_;
  std::string pose_source_preference_;
  std::string robot_base_frame_;
  std::string route_heading_path_topic_;
  std::string route_heading_frame_id_;
  std::string yaw_alignment_frame_id_;
  std::string yaw_alignment_zones_file_;
  std::string localization_mode_topic_;
  std::string dr_timeout_topic_;
  std::string merged_grid_frame_;
  std::string lanelet_grid_frame_;
  std::string drop_zone_phase_;
  std::string campsite_phase_;
  // HH_260721 - Retain the last effective authorization state to suppress heartbeat log noise.
  std::string last_authorization_log_signature_;

  bool require_platform_drive_enable_{true};
  bool platform_safety_enabled_{true};
  bool enable_pose_raw_fallback_{false};
  bool cost_source_debug_enable_{true};
  bool publish_zero_when_blocked_{true};
  bool dr_timeout_enabled_{true};
  bool enable_gnss_recovery_hold_{true};
  bool enable_yaw_alignment_zone_{false};
  bool enable_route_heading_alignment_{true};
  bool route_heading_active_{false};
  bool command_input_stale_{false};
  bool yaw_zone_satisfied_{false};
  int route_heading_min_path_points_{2};
  int gnss_recovery_source_mode_min_{2};
  int gnss_recovery_target_mode_{0};
  double speed_scale_{1.0};
  double input_timeout_s_{0.35};
  double zero_publish_rate_hz_{10.0};
  double cost_stop_latch_log_interval_s_{1.0};
  double cost_grid_stale_log_interval_s_{1.0};
  double route_heading_min_cmd_x_mps_{0.03};
  double route_heading_lateral_cmd_epsilon_mps_{0.02};
  double route_heading_lookahead_m_{2.0};
  double route_heading_error_enter_rad_{75.0 * kPi / 180.0};
  double route_heading_error_exit_rad_{35.0 * kPi / 180.0};
  double route_heading_angular_kp_{0.8};
  double route_heading_max_angular_z_{0.35};
  double route_heading_max_linear_x_{0.0};
  double yaw_alignment_exit_margin_m_{0.3};
  double gnss_recovery_hold_s_{2.0};
  double gnss_recovery_min_source_s_{0.5};
  double gnss_recovery_hold_cooldown_s_{5.0};
  double gnss_recovery_hold_until_sec_{0.0};
  double last_gnss_recovery_hold_sec_{-1.0e9};
  double last_block_log_sec_{-1.0e9};
  double last_cost_log_sec_{-1.0e9};

  std::vector<std::string> additional_estop_topics_;
  std::vector<std::string> cost_source_topics_;
  std::vector<std::string> cost_source_labels_;
  std::vector<YawAlignmentZone> yaw_zones_;
  std::optional<std::string> active_yaw_zone_;
  std::optional<double> yaw_zone_hold_start_sec_;
  std::optional<double> last_input_command_sec_;
  std::optional<double> gnss_recovery_source_enter_sec_;
  std::optional<int> last_localization_mode_;
  std::optional<avg_msgs::msg::AvgPoseStamped> latest_pose_;
  std::optional<avg_msgs::msg::AvgOdometry> latest_odometry_;
  std::optional<avg_msgs::msg::AvgPath> latest_path_;
  std::string last_cost_reason_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr command_timeout_timer_;

  rclcpp::Publisher<avg_msgs::msg::AvgTwist>::SharedPtr command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ros_command_publisher_;
  rclcpp::Publisher<avg_msgs::msg::AvgBool>::SharedPtr state_publisher_;
  rclcpp::Publisher<avg_msgs::msg::ModuleState>::SharedPtr status_publisher_;
  rclcpp::Subscription<avg_msgs::msg::AvgTwist>::SharedPtr command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_command_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr manual_engage_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr mission_engage_subscription_;
  rclcpp::Subscription<avg_msgs::msg::PlanningMissionKey>::SharedPtr mission_request_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr platform_drive_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgPlatformStatus>::SharedPtr platform_status_subscription_;
  rclcpp::Subscription<avg_msgs::msg::ModuleState>::SharedPtr drop_zone_status_subscription_;
  rclcpp::Subscription<avg_msgs::msg::ModuleState>::SharedPtr campsite_status_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr dr_timeout_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgLocalizationMode>::SharedPtr
    localization_mode_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgOccupancyGrid>::SharedPtr merged_grid_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgOccupancyGrid>::SharedPtr lanelet_grid_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgOdometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgPath>::SharedPtr route_path_subscription_;
  std::vector<rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr> estop_subscriptions_;
  std::vector<rclcpp::Subscription<avg_msgs::msg::AvgOccupancyGrid>::SharedPtr>
  source_grid_subscriptions_;
};

}  // namespace camrod_control

int main(int argc, char ** argv)
{
  // HH_260721 - Run the final command safety gate as a conventional rclcpp node.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod_control::CmdVelSafetyGateNode>());
  rclcpp::shutdown();
  return 0;
}
