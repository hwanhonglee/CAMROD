// HH_260721 - Implement campsite crab, zero-turn, recall, and reverse-entry
// maneuvers in C++.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "action_msgs/srv/cancel_goal.hpp"
#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "avg_msgs/msg/avg_service_state.hpp"
#include "avg_msgs/msg/avg_twist.hpp"
#include "avg_msgs/msg/campsite_occupancy.hpp"
#include "avg_msgs/msg/module_state.hpp"
#include "avg_msgs/msg/motion_operation.hpp"
#include "avg_msgs/msg/planning_recall_request.hpp"
#include "avg_msgs/msg/planning_scenario.hpp"
#include "avg_msgs/msg/planning_state.hpp"
#include "avg_msgs/msg/ui_destination_command.hpp"
#include "avg_msgs/srv/request_motion_operation.hpp"
#include "camrod_control/control_diagnostics.hpp"
#include "camrod_control/motion_geometry.hpp"
#include "camrod_control/ros_message_conversion.hpp"
#include "camrod_control/yaw_alignment_settling.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

enum class CampingSiteManeuverPhase {
  kIdle,
  kAlignEntryYaw,
  kReverseIn,
  kCrabIn,
  kRotate180,
  kUnloadWait,
  kWaitReturn,
  kAlignRetraceYaw,
  // HH_260721 - Rotate at the lanelet snap pose after leaving a constrained
  // roadside stop.
  kAlignReturnRouteYaw,
  kReverseOut,
  kCrabOut,
  kDone,
  kError,
};

// HH_260721 - Separate full campsite turnaround behavior from roadside stop
// behavior.
enum class CampsiteServiceMode {
  kTurnaround,
  kRoadsideStop,
};

std::string serviceModeName(const CampsiteServiceMode mode) {
  return mode == CampsiteServiceMode::kRoadsideStop ? "roadside_stop"
                                                    : "turnaround";
}

CampsiteServiceMode serviceModeFromName(const std::string &name) {
  return name == "roadside_stop" ? CampsiteServiceMode::kRoadsideStop
                                 : CampsiteServiceMode::kTurnaround;
}

std::string phaseName(const CampingSiteManeuverPhase phase) {
  switch (phase) {
  case CampingSiteManeuverPhase::kIdle:
    return "IDLE";
  case CampingSiteManeuverPhase::kAlignEntryYaw:
    return "ALIGN_ENTRY_YAW";
  case CampingSiteManeuverPhase::kReverseIn:
    return "REVERSE_IN";
  case CampingSiteManeuverPhase::kCrabIn:
    return "CRAB_IN";
  case CampingSiteManeuverPhase::kRotate180:
    return "ROTATE_180";
  case CampingSiteManeuverPhase::kUnloadWait:
    return "UNLOAD_WAIT";
  case CampingSiteManeuverPhase::kWaitReturn:
    return "WAIT_RETURN";
  case CampingSiteManeuverPhase::kAlignRetraceYaw:
    return "ALIGN_RETRACE_YAW";
  case CampingSiteManeuverPhase::kAlignReturnRouteYaw:
    return "ALIGN_RETURN_ROUTE_YAW";
  case CampingSiteManeuverPhase::kReverseOut:
    return "REVERSE_OUT";
  case CampingSiteManeuverPhase::kCrabOut:
    return "CRAB_OUT";
  case CampingSiteManeuverPhase::kDone:
    return "DONE";
  case CampingSiteManeuverPhase::kError:
    return "ERROR";
  }
  return "ERROR";
}

std::string fixed(const double value, const int precision = 2) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string signedFixed(const double value, const int precision = 0) {
  std::ostringstream stream;
  stream << std::showpos << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

} // namespace

class CampingSiteManeuverControllerNode : public rclcpp::Node {
public:
  CampingSiteManeuverControllerNode()
      : Node("camping_site_maneuver_controller") {
    // HH_260721 - Preserve the v2.0.3 semantic interfaces without Python
    // wrappers.
    command_topic_ =
        declare_parameter<std::string>("cmd_vel_topic", "/control/cmd_vel_raw");
    pose_topic_ =
        declare_parameter<std::string>("pose_topic", "/localization/pose");
    // HH_260727 - Bare-node defaults follow the regulated UI/mission boundary;
    // RViz /goal_pose must never arm a campsite maneuver.
    site_goal_topic_ = declare_parameter<std::string>(
        "site_goal_topic", "/planning/site_goal_pose_ros");
    route_goal_topic_ = declare_parameter<std::string>(
        "route_goal_topic", "/planning/goal_pose_snapped");
    lanelet_pose_topic_ = declare_parameter<std::string>(
        "lanelet_pose_topic", "/planning/lanelet_pose");
    planning_state_topic_ = declare_parameter<std::string>(
        "planning_state_topic", "/planning/state_machine/state");
    operation_topic_ = declare_parameter<std::string>(
        "operation_topic",
        "/control/camping_site_maneuver_controller/operation");
    adopt_destination_topic_ = declare_parameter<std::string>(
        "adopt_destination_topic",
        "/control/camping_site_maneuver_controller/adopt");
    status_topic_ = declare_parameter<std::string>(
        "status_topic", "/control/camping_site_maneuver_controller/status");
    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic",
                                                        "/system/diagnostics");
    service_state_topic_ =
        declare_parameter<std::string>("service_state_topic", "/service/state");
    reverse_path_topic_ = declare_parameter<std::string>(
        "reverse_path_topic",
        "/control/camping_site_maneuver_controller/path_ros");
    campsite_occupancy_topic_ = declare_parameter<std::string>(
        "campsite_occupancy_topic", "/perception/camping_sites/occupancy");

    enable_auto_start_from_planning_state_ =
        declare_parameter<bool>("enable_auto_start_from_planning_state", true);
    site_mission_key_prefix_ = declare_parameter<std::string>(
        "site_mission_key_prefix", "camping_site_");
    use_goal_pair_for_lateral_offset_ =
        declare_parameter<bool>("use_goal_pair_for_lateral_offset", true);
    require_goal_pair_for_auto_start_ =
        declare_parameter<bool>("require_goal_pair_for_auto_start", true);
    site_entry_mode_ =
        declare_parameter<std::string>("site_entry_mode", "crab");
    if (site_entry_mode_ != "reverse" && site_entry_mode_ != "crab") {
      RCLCPP_WARN(get_logger(), "unknown site_entry_mode='%s'; using crab",
                  site_entry_mode_.c_str());
      site_entry_mode_ = "crab";
    }
    camping_sites_yaml_ =
        declare_parameter<std::string>("camping_sites_yaml", "");
    site_goal_frame_id_ =
        declare_parameter<std::string>("site_goal_frame_id", "map");
    default_lateral_offset_m_ =
        declare_parameter<double>("default_lateral_offset_m", 1.2);
    minimum_lateral_offset_m_ =
        declare_parameter<double>("min_lateral_offset_m", 0.2);
    maximum_lateral_offset_m_ =
        declare_parameter<double>("max_lateral_offset_m", 7.0);
    // HH_260806 - B11-B13 are roadside service points, not drive-in bays.
    // Keep their lateral motion shallow even when the map goal is farther away.
    roadside_maximum_lateral_offset_m_ = std::abs(
        declare_parameter<double>("roadside_max_lateral_offset_m", 0.60));
    default_lateral_direction_ =
        declare_parameter<std::string>("default_lateral_direction", "left");
    crab_speed_mps_ =
        std::abs(declare_parameter<double>("crab_speed_mps", 0.18));
    entry_position_tolerance_m_ =
        std::abs(declare_parameter<double>("entry_position_tolerance_m", 0.15));
    return_position_tolerance_m_ = std::abs(
        declare_parameter<double>("return_position_tolerance_m", 0.04));
    return_translation_gain_ = std::max(
        0.1, std::abs(declare_parameter<double>("return_translation_kp", 4.0)));
    crab_timeout_margin_s_ =
        std::abs(declare_parameter<double>("crab_timeout_margin_s", 3.0));
    reverse_return_timeout_margin_s_ = std::abs(
        declare_parameter<double>("reverse_return_timeout_margin_s", 45.0));
    reverse_return_progress_tolerance_m_ = std::abs(
        declare_parameter<double>("reverse_return_progress_tolerance_m", 0.45));
    reverse_return_lateral_tolerance_m_ = std::abs(
        declare_parameter<double>("reverse_return_lateral_tolerance_m", 0.40));
    crab_timeout_speed_scale_ = std::max(
        0.05,
        std::abs(declare_parameter<double>("crab_timeout_speed_scale", 1.0)));
    maximum_forward_residual_m_ =
        std::abs(declare_parameter<double>("max_forward_residual_m", 0.8));
    rotate_proportional_gain_ = declare_parameter<double>("rotate_kp", 1.2);
    maximum_angular_speed_radps_ =
        std::abs(declare_parameter<double>("max_angular_speed_radps", 0.35));
    rotate_yaw_tolerance_deg_ =
        std::abs(declare_parameter<double>("rotate_yaw_tolerance_deg", 4.0));
    // HH_260810 - Finish zero-turn/alignment and let angular motion settle
    // before any crab/reverse/route command starts.
    rotate_settle_hold_s_ =
        std::abs(declare_parameter<double>("rotate_settle_hold_s", 0.8));
    rotate_settle_max_rate_degps_ = std::abs(
        declare_parameter<double>("rotate_settle_max_rate_degps", 3.0));
    yaw_alignment_settling_.setConfig(
        camrod_control::YawAlignmentSettlingConfig{
            rotate_yaw_tolerance_deg_, rotate_settle_hold_s_,
            rotate_settle_max_rate_degps_});
    site_rotate_direction_policy_ = declare_parameter<std::string>(
        "site_rotate_direction_policy", "site_index_lanelet_side");
    if (site_rotate_direction_policy_ != "site_index_lanelet_side" &&
        site_rotate_direction_policy_ != "shortest") {
      RCLCPP_WARN(get_logger(),
                  "unknown site_rotate_direction_policy='%s'; using shortest",
                  site_rotate_direction_policy_.c_str());
      site_rotate_direction_policy_ = "shortest";
    }
    const std::vector<int64_t> right_indices =
        declare_parameter<std::vector<int64_t>>("right_lanelet_site_indices",
                                                {1, 3, 5, 7, 9, 11});
    right_lanelet_site_indices_.insert(right_indices.begin(),
                                       right_indices.end());
    const std::vector<int64_t> left_indices =
        declare_parameter<std::vector<int64_t>>("left_lanelet_site_indices",
                                                {2, 4, 6, 8, 10, 12, 13});
    left_lanelet_site_indices_.insert(left_indices.begin(), left_indices.end());

    reverse_entry_speed_mps_ =
        std::abs(declare_parameter<double>("reverse_entry_speed_mps", 0.16));
    reverse_entry_yaw_gain_ =
        declare_parameter<double>("reverse_entry_yaw_kp", 1.0);
    reverse_entry_lateral_gain_ =
        declare_parameter<double>("reverse_entry_lateral_kp", -0.25);
    reverse_entry_maximum_angular_speed_radps_ =
        std::abs(declare_parameter<double>(
            "reverse_entry_max_angular_speed_radps", 0.25));
    reverse_entry_use_site_goal_yaw_ =
        declare_parameter<bool>("reverse_entry_use_site_goal_yaw", true);
    reverse_entry_site_yaw_mode_ = declare_parameter<std::string>(
        "reverse_entry_site_yaw_mode", "reverse_axis");
    if (reverse_entry_site_yaw_mode_ != "reverse_axis" &&
        reverse_entry_site_yaw_mode_ != "robot_yaw") {
      RCLCPP_WARN(
          get_logger(),
          "unknown reverse_entry_site_yaw_mode='%s'; using reverse_axis",
          reverse_entry_site_yaw_mode_.c_str());
      reverse_entry_site_yaw_mode_ = "reverse_axis";
    }
    reverse_entry_auto_select_yaw_equivalent_ = declare_parameter<bool>(
        "reverse_entry_auto_select_yaw_equivalent", true);
    reverse_entry_lateral_tolerance_m_ = std::abs(
        declare_parameter<double>("reverse_entry_lateral_tolerance_m", 0.35));
    reverse_entry_debug_period_s_ =
        declare_parameter<double>("reverse_entry_debug_period_s", 1.0);
    unload_wait_s_ = declare_parameter<double>("unload_wait_s", 5.0);
    auto_return_after_unload_wait_ =
        declare_parameter<bool>("auto_return_after_unload_wait", false);
    reset_wait_return_on_site_goal_ =
        declare_parameter<bool>("reset_wait_return_on_site_goal", true);
    request_return_to_drop_zone_on_done_ =
        declare_parameter<bool>("request_return_to_drop_zone_on_done", true);
    // HH_260721 - Retracing the entry lanelet keeps the 180-degree yaw reached
    // inside the site.
    align_retrace_yaw_before_crab_out_ =
        declare_parameter<bool>("align_retrace_yaw_before_crab_out", true);
    return_request_retry_period_s_ =
        declare_parameter<double>("return_request_retry_period_s", 1.0);
    return_to_drop_zone_topic_ = declare_parameter<std::string>(
        "return_to_drop_zone_topic",
        "/planning/state_machine/return_to_drop_zone");
    cancel_nav2_on_site_phase_ =
        declare_parameter<bool>("cancel_nav2_on_site_phase", true);
    nav2_cancel_period_s_ =
        declare_parameter<double>("nav2_cancel_period_s", 0.5);
    nav2_cancel_action_topics_ = declare_parameter<std::vector<std::string>>(
        "nav2_cancel_action_topics",
        {"/planning/follow_path/_action/cancel_goal",
         "/planning/navigate_to_pose/_action/cancel_goal"});
    pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 2.0);
    route_goal_reached_distance_m_ = std::abs(
        declare_parameter<double>("route_goal_reached_distance_m", 0.9));
    adopt_site_arrival_distance_m_ = std::abs(
        declare_parameter<double>("adopt_site_arrival_distance_m", 3.0));
    goal_pair_maximum_age_s_ =
        declare_parameter<double>("goal_pair_max_age_s", 0.0);
    control_rate_hz_ =
        std::max(0.1, declare_parameter<double>("control_rate_hz", 10.0));
    idle_tick_rate_hz_ =
        std::max(0.1, declare_parameter<double>("idle_tick_rate_hz", 1.0));
    status_publish_rate_hz_ =
        std::max(0.1, declare_parameter<double>("status_publish_rate_hz", 1.0));

    command_publisher_ =
        create_publisher<avg_msgs::msg::AvgTwist>(command_topic_, 10);
    status_publisher_ =
        create_publisher<avg_msgs::msg::ModuleState>(status_topic_, 10);
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            diagnostics_topic_, 10);
    return_request_publisher_ =
        create_publisher<avg_msgs::msg::PlanningRecallRequest>(
            return_to_drop_zone_topic_, 10);
    service_state_publisher_ = create_publisher<avg_msgs::msg::AvgServiceState>(
        service_state_topic_, 10);
    reverse_path_publisher_ =
        create_publisher<nav_msgs::msg::Path>(reverse_path_topic_, 10);
    for (const std::string &topic : nav2_cancel_action_topics_) {
      nav2_cancel_clients_.push_back(
          create_client<action_msgs::srv::CancelGoal>(topic));
    }

    pose_subscription_ = create_subscription<avg_msgs::msg::AvgPoseStamped>(
        pose_topic_, 10,
        [this](const avg_msgs::msg::AvgPoseStamped::SharedPtr message) {
          last_pose_ = *message;
          last_pose_time_ = now();
        });
    site_goal_subscription_ =
        create_subscription<geometry_msgs::msg::PoseStamped>(
            site_goal_topic_, 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
              onSiteGoal(camrod_control::poseFromRos(*message));
            });
    route_goal_subscription_ =
        create_subscription<avg_msgs::msg::AvgPoseStamped>(
            route_goal_topic_, 10,
            [this](const avg_msgs::msg::AvgPoseStamped::SharedPtr message) {
              route_goal_ = *message;
              route_goal_time_ = now();
              last_auto_key_.clear();
            });
    lanelet_pose_subscription_ =
        create_subscription<avg_msgs::msg::AvgPoseStamped>(
            lanelet_pose_topic_, 10,
            [this](const avg_msgs::msg::AvgPoseStamped::SharedPtr message) {
              lanelet_pose_ = *message;
              lanelet_pose_time_ = now();
            });
    planning_state_subscription_ =
        create_subscription<avg_msgs::msg::PlanningState>(
            planning_state_topic_, 10,
            [this](const avg_msgs::msg::PlanningState::SharedPtr message) {
              onPlanningState(*message);
            });
    operation_subscription_ =
        create_subscription<avg_msgs::msg::MotionOperation>(
            operation_topic_, 10,
            [this](const avg_msgs::msg::MotionOperation::SharedPtr message) {
              applyOperation(message->operation, message->source.empty()
                                                     ? "topic"
                                                     : message->source);
            });
    adopt_destination_subscription_ =
        create_subscription<avg_msgs::msg::UiDestinationCommand>(
            adopt_destination_topic_, 10,
            [this](
                const avg_msgs::msg::UiDestinationCommand::SharedPtr message) {
              if (message->run) {
                const std::string key = missionKeyFromDestination(*message);
                if (!key.empty()) {
                  adoptWaitReturnState(key, message->source.empty()
                                                ? "adopt_destination"
                                                : message->source);
                }
              }
            });
    // HH_260723 - Reject duplicate campsite entry from the transient semantic
    // occupancy state and stop an approach if the target becomes occupied.
    campsite_occupancy_subscription_ =
        create_subscription<avg_msgs::msg::CampsiteOccupancy>(
            campsite_occupancy_topic_,
            rclcpp::QoS(1).transient_local().reliable(),
            [this](const avg_msgs::msg::CampsiteOccupancy::SharedPtr message) {
              occupied_mission_keys_.clear();
              occupied_mission_keys_.insert(
                  message->occupied_mission_keys.begin(),
                  message->occupied_mission_keys.end());
              if (isEntryPhase() && isSiteOccupied(site_goal_key_)) {
                setError("occupied campsite entry blocked: " + site_goal_key_);
              }
            });
    operation_service_ = create_service<avg_msgs::srv::RequestMotionOperation>(
        "/control/camping_site_maneuver_controller/request_operation",
        [this](const std::shared_ptr<
                   avg_msgs::srv::RequestMotionOperation::Request>
                   request,
               std::shared_ptr<avg_msgs::srv::RequestMotionOperation::Response>
                   response) {
          const auto result = applyOperation(
              request->operation,
              request->source.empty() ? "service" : request->source);
          response->accepted = result.first;
          response->message = result.second;
        });

    phase_start_time_ = now();
    loadCampingSites();
    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_hz_),
        std::bind(&CampingSiteManeuverControllerNode::onTimer, this));
    RCLCPP_INFO(get_logger(),
                "camping_site_maneuver_controller ready: cmd=%s pose=%s "
                "lanelet_pose=%s auto_start=%s entry_mode=%s adopt_topic=%s "
                "cancel_nav2=%s",
                command_topic_.c_str(), pose_topic_.c_str(),
                lanelet_pose_topic_.c_str(),
                enable_auto_start_from_planning_state_ ? "true" : "false",
                site_entry_mode_.c_str(), adopt_destination_topic_.c_str(),
                cancel_nav2_on_site_phase_ ? "true" : "false");
  }

private:
  bool isActivePhase() const {
    return phase_ != CampingSiteManeuverPhase::kIdle &&
           phase_ != CampingSiteManeuverPhase::kDone &&
           phase_ != CampingSiteManeuverPhase::kError;
  }

  bool isSiteInternalPhase() const {
    return phase_ == CampingSiteManeuverPhase::kAlignEntryYaw ||
           phase_ == CampingSiteManeuverPhase::kReverseIn ||
           phase_ == CampingSiteManeuverPhase::kCrabIn ||
           phase_ == CampingSiteManeuverPhase::kRotate180 ||
           phase_ == CampingSiteManeuverPhase::kUnloadWait ||
           phase_ == CampingSiteManeuverPhase::kWaitReturn ||
           phase_ == CampingSiteManeuverPhase::kAlignRetraceYaw ||
           phase_ == CampingSiteManeuverPhase::kAlignReturnRouteYaw ||
           phase_ == CampingSiteManeuverPhase::kReverseOut ||
           phase_ == CampingSiteManeuverPhase::kCrabOut;
  }

  bool isEntryPhase() const {
    return phase_ == CampingSiteManeuverPhase::kAlignEntryYaw ||
           phase_ == CampingSiteManeuverPhase::kReverseIn ||
           phase_ == CampingSiteManeuverPhase::kCrabIn ||
           phase_ == CampingSiteManeuverPhase::kRotate180;
  }

  bool isSiteOccupied(const std::string &mission_key) const {
    return !mission_key.empty() &&
           occupied_mission_keys_.count(mission_key) > 0U;
  }

  avg_msgs::msg::AvgPoseStamped makePose(const std::string &frame_id,
                                         const double x, const double y,
                                         const double z,
                                         const double yaw_deg) const {
    avg_msgs::msg::AvgPoseStamped pose;
    pose.header.frame_id =
        frame_id.empty()
            ? (site_goal_frame_id_.empty() ? "map" : site_goal_frame_id_)
            : frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    const geometry_msgs::msg::Quaternion orientation =
        camrod_control::quaternionFromYaw(yaw_deg * M_PI / 180.0);
    pose.pose.orientation.x = orientation.x;
    pose.pose.orientation.y = orientation.y;
    pose.pose.orientation.z = orientation.z;
    pose.pose.orientation.w = orientation.w;
    return pose;
  }

  avg_msgs::msg::AvgPoseStamped
  stampPoseNow(avg_msgs::msg::AvgPoseStamped pose) const {
    pose.header.stamp = now();
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id =
          site_goal_frame_id_.empty() ? "map" : site_goal_frame_id_;
    }
    return pose;
  }

  void loadCampingSites() {
    camping_site_goals_.clear();
    camping_site_service_modes_.clear();
    if (camping_sites_yaml_.empty()) {
      return;
    }
    try {
      const YAML::Node document = YAML::LoadFile(camping_sites_yaml_);
      const YAML::Node sites = document["camping_sites"];
      if (!sites || !sites.IsSequence()) {
        RCLCPP_WARN(get_logger(), "invalid camping_sites_yaml format: %s",
                    camping_sites_yaml_.c_str());
        return;
      }
      for (std::size_t index = 0; index < sites.size(); ++index) {
        const YAML::Node item = sites[index];
        if (!item["x"] || !item["y"]) {
          continue;
        }
        // HH_260721 - Prefer the map-authored operational pose while retaining
        // legacy YAML fallback.
        const avg_msgs::msg::AvgPoseStamped pose = makePose(
            item["frame_id"] ? item["frame_id"].as<std::string>()
                             : site_goal_frame_id_,
            item["service_x"] ? item["service_x"].as<double>()
                              : item["x"].as<double>(),
            item["service_y"] ? item["service_y"].as<double>()
                              : item["y"].as<double>(),
            item["service_z"] ? item["service_z"].as<double>()
                              : (item["z"] ? item["z"].as<double>() : 0.0),
            item["service_yaw_deg"]
                ? item["service_yaw_deg"].as<double>()
                : (item["yaw_deg"] ? item["yaw_deg"].as<double>() : 0.0));
        const std::string configured_mode =
            item["service_mode"] ? item["service_mode"].as<std::string>()
                                 : "turnaround";
        const CampsiteServiceMode service_mode =
            serviceModeFromName(configured_mode);
        if (configured_mode != "turnaround" &&
            configured_mode != "roadside_stop") {
          RCLCPP_WARN(get_logger(),
                      "unknown campsite service_mode '%s'; using turnaround",
                      configured_mode.c_str());
        }
        for (const std::string field : {"type", "id", "name"}) {
          if (item[field]) {
            const std::string key = item[field].as<std::string>();
            if (!key.empty()) {
              camping_site_goals_[key] = pose;
              camping_site_service_modes_[key] = service_mode;
            }
          }
        }
      }
      RCLCPP_INFO(get_logger(), "loaded %zu campsite goal entries from %s",
                  camping_site_goals_.size(), camping_sites_yaml_.c_str());
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "failed to load camping_sites_yaml (%s): %s",
                  camping_sites_yaml_.c_str(), error.what());
    }
  }

  static double poseDistance(const avg_msgs::msg::AvgPoseStamped &first,
                             const avg_msgs::msg::AvgPoseStamped &second) {
    return std::hypot(first.pose.position.x - second.pose.position.x,
                      first.pose.position.y - second.pose.position.y);
  }

  void onSiteGoal(const avg_msgs::msg::AvgPoseStamped &message) {
    if (isSiteInternalPhase()) {
      requestNav2CancelForSitePhase(true);
      RCLCPP_WARN(get_logger(),
                  "camping_site_maneuver_controller ignored new site_goal "
                  "while internal: phase=%s x=%.2f y=%.2f",
                  phaseName(phase_).c_str(), message.pose.position.x,
                  message.pose.position.y);
      return;
    }
    const bool had_goal = site_goal_.has_value();
    const bool goal_changed =
        had_goal && poseDistance(*site_goal_, message) > 0.2;
    site_goal_ = message;
    site_goal_time_ = now();
    site_goal_key_.clear();
    if (!had_goal || goal_changed ||
        phase_ == CampingSiteManeuverPhase::kIdle ||
        phase_ == CampingSiteManeuverPhase::kDone ||
        phase_ == CampingSiteManeuverPhase::kError) {
      last_auto_key_.clear();
    }
    if (isActivePhase() && goal_changed) {
      publishZero();
      return_requested_ = false;
      return_published_ = false;
      return_acknowledged_ = false;
      setPhase(CampingSiteManeuverPhase::kIdle,
               "new site_goal received; stale maneuver cancelled");
    }
    RCLCPP_INFO(get_logger(),
                "camping_site_maneuver_controller site_goal updated: topic=%s "
                "x=%.2f y=%.2f",
                site_goal_topic_.c_str(), message.pose.position.x,
                message.pose.position.y);
  }

  std::string missionKeyFromDestination(
      const avg_msgs::msg::UiDestinationCommand &message) const {
    if (!message.mission_key.empty()) {
      return message.mission_key;
    }
    std::string site = message.site;
    std::transform(site.begin(), site.end(), site.begin(), ::toupper);
    if (site.size() > 1U && site.front() == 'B' &&
        std::all_of(site.begin() + 1, site.end(), ::isdigit)) {
      return site_mission_key_prefix_ +
             std::to_string(std::stoi(site.substr(1)));
    }
    return site;
  }

  bool poseUsableAsAdoptRoute(
      const std::optional<avg_msgs::msg::AvgPoseStamped> &route_pose,
      const avg_msgs::msg::AvgPoseStamped &site_goal) const {
    if (!route_pose.has_value()) {
      return false;
    }
    const double distance = poseDistance(*route_pose, site_goal);
    return distance >= minimum_lateral_offset_m_ &&
           distance <= maximum_lateral_offset_m_;
  }

  bool adoptWaitReturnState(const std::string &key, const std::string &source) {
    if (isSiteOccupied(key)) {
      RCLCPP_WARN(
          get_logger(),
          "camping_site_maneuver_controller adopt blocked: occupied site %s",
          key.c_str());
      return false;
    }
    if (phase_ != CampingSiteManeuverPhase::kIdle &&
        phase_ != CampingSiteManeuverPhase::kDone &&
        phase_ != CampingSiteManeuverPhase::kError &&
        phase_ != CampingSiteManeuverPhase::kUnloadWait &&
        phase_ != CampingSiteManeuverPhase::kWaitReturn) {
      RCLCPP_WARN(
          get_logger(),
          "camping_site_maneuver_controller adopt ignored during phase=%s",
          phaseName(phase_).c_str());
      return false;
    }
    if (!poseIsFresh()) {
      RCLCPP_WARN(get_logger(), "camping_site_maneuver_controller adopt "
                                "ignored: fresh pose unavailable");
      return false;
    }
    std::optional<avg_msgs::msg::AvgPoseStamped> adopted_site_goal;
    const auto configured_goal = camping_site_goals_.find(key);
    if (configured_goal != camping_site_goals_.end()) {
      adopted_site_goal = configured_goal->second;
    } else if (site_goal_.has_value()) {
      adopted_site_goal = *site_goal_;
    }
    if (!adopted_site_goal.has_value()) {
      RCLCPP_WARN(get_logger(),
                  "camping_site_maneuver_controller adopt ignored: site goal "
                  "unavailable for %s",
                  key.c_str());
      return false;
    }
    *adopted_site_goal = stampPoseNow(*adopted_site_goal);
    const double distance_to_site =
        poseDistance(*last_pose_, *adopted_site_goal);
    if (distance_to_site > adopt_site_arrival_distance_m_) {
      RCLCPP_WARN(get_logger(),
                  "camping_site_maneuver_controller adopt ignored: key=%s "
                  "distance=%.2fm limit=%.2fm",
                  key.c_str(), distance_to_site,
                  adopt_site_arrival_distance_m_);
      return false;
    }

    avg_msgs::msg::AvgPoseStamped adopted_route_goal;
    std::string route_source = "current_pose_fallback";
    if (poseUsableAsAdoptRoute(route_goal_, *adopted_site_goal)) {
      adopted_route_goal = stampPoseNow(*route_goal_);
      route_source = "latest_route_goal";
    } else if (poseUsableAsAdoptRoute(lanelet_pose_, *adopted_site_goal)) {
      adopted_route_goal = stampPoseNow(*lanelet_pose_);
      route_source = "latest_lanelet_pose";
    } else {
      adopted_route_goal = stampPoseNow(*last_pose_);
    }

    const auto relative =
        camrod_control::relativeXy(adopted_route_goal, *adopted_site_goal);
    double offset = std::abs(relative.second);
    double direction = relative.second >= 0.0 ? 1.0 : -1.0;
    std::string source_name = "goal_pair";
    if (offset < minimum_lateral_offset_m_) {
      direction = default_lateral_direction_ == "right" ? -1.0 : 1.0;
      if (route_source != "current_pose_fallback") {
        offset = std::min(poseDistance(adopted_route_goal, *adopted_site_goal),
                          maximum_lateral_offset_m_);
        source_name = route_source + "_distance";
      } else {
        offset = 0.0;
        source_name = "adopt_current_pose";
      }
    }

    site_goal_ = *adopted_site_goal;
    route_goal_ = adopted_route_goal;
    site_goal_time_ = now();
    route_goal_time_ = now();
    site_goal_key_ = key;
    // HH_260721 - Restore the configured service policy when adopting a robot
    // already at a site.
    active_service_mode_ = serviceModeForKey(key);
    last_auto_key_ = "adopt:" + key + ":" + fixed(now().seconds(), 3);
    start_yaw_ = camrod_control::yawFromPose(adopted_route_goal);
    target_yaw_ = start_yaw_;
    rotate_direction_sign_ = 0.0;
    rotate_direction_label_ = "adopted";
    crab_direction_ = direction;
    crab_offset_m_ = offset;
    crab_source_ = source_name;
    goal_pair_forward_m_ = relative.first;
    entry_start_x_ = adopted_route_goal.pose.position.x;
    entry_start_y_ = adopted_route_goal.pose.position.y;
    return_anchor_x_ = adopted_route_goal.pose.position.x;
    return_anchor_y_ = adopted_route_goal.pose.position.y;
    return_anchor_source_ = route_source;
    return_anchor_offset_m_ = poseDistance(*last_pose_, adopted_route_goal);
    entry_reference_yaw_ = start_yaw_;
    entry_target_x_ = adopted_site_goal->pose.position.x;
    entry_target_y_ = adopted_site_goal->pose.position.y;
    entry_target_yaw_ = camrod_control::normalizeAngle(start_yaw_ + M_PI);
    entry_reverse_axis_yaw_ = start_yaw_;
    const double effective_speed =
        std::max(0.01, crab_speed_mps_ * crab_timeout_speed_scale_);
    crab_duration_s_ = offset > 0.0 ? offset / effective_speed : 0.0;
    return_requested_ = false;
    return_published_ = false;
    return_acknowledged_ = false;
    last_return_request_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    publishZero();
    setPhase(CampingSiteManeuverPhase::kWaitReturn,
             "adopt=" + source + " key=" + key + " route=" + route_source +
                 " dist=" + fixed(distance_to_site) + "m offset=" +
                 fixed(offset) + "m direction=" + signedFixed(direction) +
                 " service_mode=" + serviceModeName(active_service_mode_) +
                 " source=" + source_name);
    return true;
  }

  bool siteGoalMatchesKey(const std::string &key) const {
    if (key.empty() || !site_goal_.has_value()) {
      return false;
    }
    const auto expected = camping_site_goals_.find(key);
    return expected == camping_site_goals_.end() ||
           poseDistance(*site_goal_, expected->second) <= 0.5;
  }

  CampsiteServiceMode serviceModeForKey(const std::string &key) const {
    const auto configured_mode = camping_site_service_modes_.find(key);
    return configured_mode == camping_site_service_modes_.end()
               ? CampsiteServiceMode::kTurnaround
               : configured_mode->second;
  }

  void ensureGoalPairForAutoStart(const std::string &key) {
    // HH_260721 - Bind the mission key and service policy even when the UI pose
    // already matches YAML.
    site_goal_key_ = key;
    active_service_mode_ = serviceModeForKey(key);
    const auto configured_goal = camping_site_goals_.find(key);
    if (configured_goal != camping_site_goals_.end() &&
        (!site_goal_.has_value() || !siteGoalMatchesKey(key))) {
      site_goal_ = stampPoseNow(configured_goal->second);
      site_goal_time_ = now();
      RCLCPP_INFO(
          get_logger(),
          "restored site_goal from camping_sites_yaml: key=%s x=%.2f y=%.2f",
          key.c_str(), site_goal_->pose.position.x,
          site_goal_->pose.position.y);
    }
    if (!route_goal_.has_value() && poseIsFresh()) {
      route_goal_ = stampPoseNow(*last_pose_);
      route_goal_time_ = now();
      RCLCPP_INFO(get_logger(),
                  "restored route_goal from current pose at GOAL_REACHED");
    }
  }

  std::optional<int64_t> siteIndexFromText(const std::string &text) const {
    std::smatch match;
    if (!std::regex_search(text, match, std::regex("camping_site_([0-9]+)"))) {
      return std::nullopt;
    }
    return std::stoll(match[1].str());
  }

  std::pair<double, std::string>
  rotationDirectionForSource(const std::string &source) const {
    if (site_rotate_direction_policy_ != "site_index_lanelet_side") {
      return {0.0, "shortest"};
    }
    std::optional<int64_t> index = siteIndexFromText(source);
    if (!index.has_value()) {
      index = siteIndexFromText(site_goal_key_);
    }
    if (!index.has_value()) {
      index = siteIndexFromText(last_auto_key_);
    }
    if (!index.has_value()) {
      return {0.0, "shortest"};
    }
    if (right_lanelet_site_indices_.count(*index) > 0U) {
      return {-1.0, "right_cw_site_" + std::to_string(*index)};
    }
    if (left_lanelet_site_indices_.count(*index) > 0U) {
      return {1.0, "left_ccw_site_" + std::to_string(*index)};
    }
    return {0.0, "shortest_site_" + std::to_string(*index)};
  }

  void onPlanningState(const avg_msgs::msg::PlanningState &message) {
    if (message.scenario_id ==
            avg_msgs::msg::PlanningScenario::RETURN_TO_DROP_ZONE ||
        message.active_mission_key == "drop_zone") {
      return_acknowledged_ = true;
    }
    if (!enable_auto_start_from_planning_state_) {
      return;
    }
    if (message.state != avg_msgs::msg::PlanningState::GOAL_REACHED) {
      if (phase_ == CampingSiteManeuverPhase::kIdle ||
          phase_ == CampingSiteManeuverPhase::kDone ||
          phase_ == CampingSiteManeuverPhase::kError) {
        last_auto_key_.clear();
      }
      return;
    }
    if (message.scenario_id !=
            avg_msgs::msg::PlanningScenario::DELIVERY_TO_SITE &&
        message.scenario_id !=
            avg_msgs::msg::PlanningScenario::RECALL_TO_SITE) {
      return;
    }
    // HH_260727 - RViz/manual navigation is free-space driving only. Ignore
    // stale UI site-goal pairs so arrival cannot start a campsite maneuver.
    if (message.active_goal_source == "manual") {
      return;
    }
    std::string automatic_key;
    const std::string mission_key = message.active_mission_key;
    if (mission_key.rfind(site_mission_key_prefix_, 0) == 0) {
      if (isSiteOccupied(mission_key)) {
        publishZero();
        setPhase(CampingSiteManeuverPhase::kError,
                 "occupied campsite auto-entry blocked: " + mission_key);
        last_auto_key_ = mission_key;
        return;
      }
      ensureGoalPairForAutoStart(mission_key);
      automatic_key = mission_key;
    } else {
      // HH_260727 - A stale site/route pose pair is not mission authority.
      // Auto-entry requires an explicit regulated camping_site_* mission key.
      return;
    }
    if (automatic_key == last_auto_key_) {
      return;
    }
    if (!routeGoalReachedForAutoStart()) {
      // HH_260813 - This gate used to reject an arrival without a trace, which
      // made a missing crab entry indistinguishable from a missing mission key.
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "campsite auto-entry waiting at GOAL_REACHED: key=%s "
          "route_goal=%s pose=%s lanelet=%s limit=%.2fm",
          automatic_key.c_str(),
          route_goal_.has_value() ? "yes" : "missing",
          route_goal_.has_value() && poseIsFresh()
              ? fixed(poseDistance(*last_pose_, *route_goal_)).c_str()
              : "stale",
          route_goal_.has_value() && lanelet_pose_.has_value()
              ? fixed(poseDistance(*lanelet_pose_, *route_goal_)).c_str()
              : "stale",
          route_goal_reached_distance_m_);
      return;
    }
    const auto result = startSequence("planning_state:" + automatic_key);
    if (result.first) {
      last_auto_key_ = automatic_key;
    }
  }

  std::pair<bool, std::string> applyOperation(const uint8_t operation,
                                              const std::string &source) {
    if (operation == avg_msgs::msg::MotionOperation::START) {
      return startSequence(source);
    }
    if (operation == avg_msgs::msg::MotionOperation::RETURN) {
      return requestReturn(source);
    }
    if (operation == avg_msgs::msg::MotionOperation::CANCEL) {
      publishZero();
      setPhase(CampingSiteManeuverPhase::kIdle, "cancel=" + source);
      return {true, "camping-site maneuver cancelled"};
    }
    return {false,
            "unsupported camping-site operation=" + std::to_string(operation)};
  }

  std::tuple<double, double, std::string, double> resolveLateralMotion() const {
    double offset = default_lateral_offset_m_;
    double direction = default_lateral_direction_ == "right" ? -1.0 : 1.0;
    std::string source = "default";
    double forward = 0.0;
    if (use_goal_pair_for_lateral_offset_ && route_goal_.has_value() &&
        site_goal_.has_value() && goalPairIsFresh()) {
      const auto relative =
          camrod_control::relativeXy(*route_goal_, *site_goal_);
      forward = relative.first;
      if (std::abs(relative.second) >= minimum_lateral_offset_m_) {
        offset = std::abs(relative.second);
        direction = relative.second >= 0.0 ? 1.0 : -1.0;
        source = "goal_pair";
      }
    }
    return {camrod_control::clamp(offset, minimum_lateral_offset_m_,
                                  maximum_lateral_offset_m_),
            direction, source, forward};
  }

  std::pair<bool, std::string> startSequence(const std::string &source) {
    if (phase_ != CampingSiteManeuverPhase::kIdle &&
        phase_ != CampingSiteManeuverPhase::kDone &&
        phase_ != CampingSiteManeuverPhase::kError) {
      return {false, "site maneuver already active: " + phaseName(phase_)};
    }
    if (isSiteOccupied(site_goal_key_)) {
      publishZero();
      setPhase(CampingSiteManeuverPhase::kError,
               "occupied campsite entry blocked: " + site_goal_key_);
      return {false, "occupied campsite entry blocked: " + site_goal_key_};
    }
    if (!poseIsFresh()) {
      setError("fresh pose unavailable");
      return {false, "fresh pose unavailable"};
    }
    // HH_260721 - A direct/manual goal remains a normal turnaround unless a
    // mission key selects otherwise.
    active_service_mode_ = serviceModeForKey(site_goal_key_);
    const auto motion = resolveLateralMotion();
    const double requested_lateral_offset = std::get<0>(motion);
    const double direction = std::get<1>(motion);
    std::string source_name = std::get<2>(motion);
    const double forward_residual = std::get<3>(motion);
    if (source.rfind("planning_state:", 0) == 0 &&
        require_goal_pair_for_auto_start_ && source_name != "goal_pair") {
      setError("site/route goal pair unavailable for auto site maneuver");
      return {false, "site/route goal pair unavailable for auto site maneuver"};
    }
    // HH_260807 - Nav2 may report GOAL_REACHED before the body reaches the
    // exact snapped centerline pose. Preserve the actual pose for entry
    // progress, but return automatic services to the explicit route goal so the
    // next Nav2 command never inherits an arrival pose already touching the
    // road margin.
    captureReturnAnchor(source.rfind("planning_state:", 0) == 0 &&
                        source_name == "goal_pair");
    if (site_entry_mode_ == "reverse" &&
        active_service_mode_ != CampsiteServiceMode::kRoadsideStop) {
      return startReverseEntrySequence(source, source_name, forward_residual);
    }
    double lateral_offset = requested_lateral_offset;
    if (active_service_mode_ == CampsiteServiceMode::kRoadsideStop) {
      lateral_offset =
          std::min(lateral_offset, roadside_maximum_lateral_offset_m_);
      if (lateral_offset + 1.0e-6 < requested_lateral_offset) {
        source_name += "_roadside_cap";
      }
    }
    if (lateral_offset <= 0.0 || crab_speed_mps_ <= 0.0) {
      setError("invalid lateral motion parameters");
      return {false, "invalid lateral motion parameters"};
    }

    start_yaw_ = camrod_control::yawFromPose(*last_pose_);
    const auto rotation = rotationDirectionForSource(source);
    rotate_direction_sign_ = rotation.first;
    rotate_direction_label_ = rotation.second;
    target_yaw_ = camrod_control::normalizeAngle(
        start_yaw_ +
        (rotate_direction_sign_ == 0.0 ? M_PI : rotate_direction_sign_ * M_PI));
    crab_direction_ = direction;
    crab_offset_m_ = lateral_offset;
    crab_source_ = source_name;
    goal_pair_forward_m_ = forward_residual;
    entry_start_x_ = last_pose_->pose.position.x;
    entry_start_y_ = last_pose_->pose.position.y;
    entry_reference_yaw_ = start_yaw_;
    crab_duration_s_ =
        lateral_offset /
        std::max(0.01, crab_speed_mps_ * crab_timeout_speed_scale_);
    return_requested_ = false;
    return_published_ = false;
    return_acknowledged_ = false;
    last_return_request_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    if (source_name == "goal_pair" &&
        std::abs(forward_residual) > maximum_forward_residual_m_) {
      RCLCPP_WARN(
          get_logger(),
          "goal pair forward residual %.2fm exceeds crab-only limit %.2fm",
          forward_residual, maximum_forward_residual_m_);
    }
    setPhase(CampingSiteManeuverPhase::kCrabIn,
             "start=" + source + " offset=" + fixed(lateral_offset) +
                 "m direction=" + signedFixed(direction) +
                 " rotate=" + rotate_direction_label_ +
                 " service_mode=" + serviceModeName(active_service_mode_) +
                 " source=" + source_name +
                 " forward=" + fixed(forward_residual) +
                 "m requested_offset=" + fixed(requested_lateral_offset) +
                 "m return_anchor=" + return_anchor_source_ +
                 " anchor_offset=" + fixed(return_anchor_offset_m_) +
                 "m anchor_xy=(" + fixed(return_anchor_x_) + "," +
                 fixed(return_anchor_y_) + ")" +
                 "m timeout_speed_scale=" + fixed(crab_timeout_speed_scale_) +
                 " duration=" + fixed(crab_duration_s_, 1) + "s");
    return {true, "site maneuver started"};
  }

  std::pair<double, std::string>
  selectReverseEntryAxis(const double direct_axis_yaw, const double dx,
                         const double dy) const {
    if (!reverse_entry_use_site_goal_yaw_ || !site_goal_.has_value()) {
      return {direct_axis_yaw, "snap_to_site"};
    }
    double axis_yaw = camrod_control::yawFromPose(*site_goal_);
    std::string source = "site_reverse_axis";
    if (reverse_entry_site_yaw_mode_ == "robot_yaw") {
      axis_yaw = camrod_control::normalizeAngle(axis_yaw + M_PI);
      source = "site_robot_yaw";
    }
    if (reverse_entry_auto_select_yaw_equivalent_) {
      const double forward_projection =
          std::cos(axis_yaw) * dx + std::sin(axis_yaw) * dy;
      const double flipped_axis =
          camrod_control::normalizeAngle(axis_yaw + M_PI);
      const double flipped_projection =
          std::cos(flipped_axis) * dx + std::sin(flipped_axis) * dy;
      if (flipped_projection > forward_projection) {
        axis_yaw = flipped_axis;
        source += "_flipped";
      }
    }
    return {camrod_control::normalizeAngle(axis_yaw), source};
  }

  std::pair<bool, std::string>
  startReverseEntrySequence(const std::string &source,
                            const std::string &source_name,
                            const double forward_residual) {
    if (!site_goal_.has_value()) {
      setError("site goal unavailable for reverse site maneuver");
      return {false, "site goal unavailable for reverse site maneuver"};
    }
    if (reverse_entry_speed_mps_ <= 0.0) {
      setError("invalid reverse entry speed");
      return {false, "invalid reverse entry speed"};
    }
    entry_start_x_ = last_pose_->pose.position.x;
    entry_start_y_ = last_pose_->pose.position.y;
    entry_target_x_ = site_goal_->pose.position.x;
    entry_target_y_ = site_goal_->pose.position.y;
    const double dx = entry_target_x_ - entry_start_x_;
    const double dy = entry_target_y_ - entry_start_y_;
    const double distance = std::hypot(dx, dy);
    if (distance < minimum_lateral_offset_m_) {
      setError("reverse site target is too close to lanelet snap pose");
      return {false, "reverse site target is too close to lanelet snap pose"};
    }
    if (distance > maximum_lateral_offset_m_) {
      setError("reverse site target distance " + fixed(distance) +
               "m exceeds max " + fixed(maximum_lateral_offset_m_) + "m");
      return {false, "reverse site target exceeds max distance"};
    }
    start_yaw_ = camrod_control::yawFromPose(*last_pose_);
    const auto selected_axis =
        selectReverseEntryAxis(std::atan2(dy, dx), dx, dy);
    entry_reverse_axis_yaw_ = selected_axis.first;
    entry_target_yaw_ =
        camrod_control::normalizeAngle(entry_reverse_axis_yaw_ + M_PI);
    target_yaw_ = entry_target_yaw_;
    rotate_direction_sign_ = 0.0;
    rotate_direction_label_ = "shortest";
    entry_line_origin_x_ = entry_target_x_;
    entry_line_origin_y_ = entry_target_y_;
    entry_target_progress_m_ = std::cos(entry_reverse_axis_yaw_) * dx +
                               std::sin(entry_reverse_axis_yaw_) * dy;
    entry_reverse_distance_m_ = distance;
    crab_offset_m_ = distance;
    crab_source_ = source_name;
    goal_pair_forward_m_ = forward_residual;
    crab_duration_s_ = distance / std::max(0.01, reverse_entry_speed_mps_ *
                                                     crab_timeout_speed_scale_);
    return_requested_ = false;
    return_published_ = false;
    return_acknowledged_ = false;
    last_return_request_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    publishReversePath(entry_start_x_, entry_start_y_, entry_target_x_,
                       entry_target_y_, entry_target_yaw_);
    setPhase(CampingSiteManeuverPhase::kAlignEntryYaw,
             "start=" + source + " reverse_distance=" + fixed(distance) +
                 "m target=(" + fixed(entry_target_x_) + "," +
                 fixed(entry_target_y_) + ") source=" + source_name +
                 " forward=" + fixed(forward_residual) +
                 "m axis=" + selected_axis.second + " axis_yaw=" +
                 fixed(entry_reverse_axis_yaw_ * 180.0 / M_PI, 1) +
                 "deg body_yaw=" + fixed(entry_target_yaw_ * 180.0 / M_PI, 1) +
                 "deg");
    return {true, "site reverse maneuver started"};
  }

  std::pair<bool, std::string> requestReturn(const std::string &source) {
    if (phase_ == CampingSiteManeuverPhase::kWaitReturn) {
      return_requested_ = true;
      beginReturnExit("return=" + source);
      return {true, "site maneuver return started"};
    }
    if (phase_ == CampingSiteManeuverPhase::kIdle ||
        phase_ == CampingSiteManeuverPhase::kDone) {
      return {false,
              "site maneuver is not waiting for return: " + phaseName(phase_)};
    }
    return_requested_ = true;
    return {true, "return request latched during " + phaseName(phase_)};
  }

  void beginReturnExit(const std::string &reason) {
    if (active_service_mode_ == CampsiteServiceMode::kRoadsideStop) {
      // HH_260721 - Leave constrained terrain first; the 180-degree return
      // alignment runs on-lane.
      target_yaw_ = start_yaw_;
      setPhase(CampingSiteManeuverPhase::kCrabOut,
               reason + "; roadside exit before on-lane return alignment");
    } else if (site_entry_mode_ == "reverse") {
      setPhase(CampingSiteManeuverPhase::kReverseOut, reason);
    } else if (align_retrace_yaw_before_crab_out_) {
      // HH_260721 - Verify retrace yaw without undoing the site's required
      // 180-degree turn.
      target_yaw_ = camrod_control::normalizeAngle(start_yaw_ + M_PI);
      setPhase(CampingSiteManeuverPhase::kAlignRetraceYaw,
               reason + "; preserve_retrace_yaw");
    } else {
      setPhase(CampingSiteManeuverPhase::kCrabOut, reason);
    }
  }

  bool goalPairIsFresh() const {
    return goal_pair_maximum_age_s_ <= 0.0 ||
           ((now() - site_goal_time_).seconds() <= goal_pair_maximum_age_s_ &&
            (now() - route_goal_time_).seconds() <= goal_pair_maximum_age_s_);
  }

  bool routeGoalReachedForAutoStart() const {
    if (!route_goal_.has_value()) {
      return false;
    }
    // HH_260813 - Planning declares GOAL_REACHED from the lanelet snap while
    // this controller holds the raw vehicle pose. A body stopped off the
    // centerline is at the route goal by one measure and away from it by the
    // other; either confirmation is enough to adopt the campsite entry.
    if (poseIsFresh() &&
        poseDistance(*last_pose_, *route_goal_) <=
            route_goal_reached_distance_m_) {
      return true;
    }
    return lanelet_pose_.has_value() &&
           (now() - lanelet_pose_time_).seconds() <= pose_timeout_s_ &&
           poseDistance(*lanelet_pose_, *route_goal_) <=
               route_goal_reached_distance_m_;
  }

  bool poseIsFresh() const {
    return last_pose_.has_value() &&
           (now() - last_pose_time_).seconds() <= pose_timeout_s_;
  }

  void captureReturnAnchor(const bool use_route_goal) {
    return_anchor_x_ = last_pose_->pose.position.x;
    return_anchor_y_ = last_pose_->pose.position.y;
    return_anchor_source_ = "actual_arrival_pose";
    if (use_route_goal && route_goal_.has_value()) {
      return_anchor_x_ = route_goal_->pose.position.x;
      return_anchor_y_ = route_goal_->pose.position.y;
      return_anchor_source_ = "route_goal_snap";
    }
    return_anchor_offset_m_ = distanceTo(return_anchor_x_, return_anchor_y_);
  }

  double relativeLateralFromEntry() const {
    if (!last_pose_.has_value()) {
      return 0.0;
    }
    const double dx = last_pose_->pose.position.x - entry_start_x_;
    const double dy = last_pose_->pose.position.y - entry_start_y_;
    return -std::sin(entry_reference_yaw_) * dx +
           std::cos(entry_reference_yaw_) * dy;
  }

  bool entryReached() const {
    return poseIsFresh() &&
           crab_direction_ * relativeLateralFromEntry() >=
               std::max(0.0, crab_offset_m_ - entry_position_tolerance_m_);
  }

  bool returnReached() const {
    // HH_260807 - Require the explicit route anchor in both map axes. The entry
    // start remains separate because Nav2 can accept an arrival short of the
    // centerline target by route_goal_reached_distance_m.
    return poseIsFresh() && distanceTo(return_anchor_x_, return_anchor_y_) <=
                                return_position_tolerance_m_;
  }

  std::pair<double, double> axisProgressLateral(const double origin_x,
                                                const double origin_y,
                                                const double axis_yaw) const {
    if (!last_pose_.has_value()) {
      return {0.0, 0.0};
    }
    const double dx = last_pose_->pose.position.x - origin_x;
    const double dy = last_pose_->pose.position.y - origin_y;
    return {std::cos(axis_yaw) * dx + std::sin(axis_yaw) * dy,
            -std::sin(axis_yaw) * dx + std::cos(axis_yaw) * dy};
  }

  double distanceTo(const double x, const double y) const {
    return last_pose_.has_value() ? std::hypot(last_pose_->pose.position.x - x,
                                               last_pose_->pose.position.y - y)
                                  : std::numeric_limits<double>::infinity();
  }

  bool reverseInReached() const {
    if (!poseIsFresh()) {
      return false;
    }
    const double progress = axisProgressLateral(entry_start_x_, entry_start_y_,
                                                entry_reverse_axis_yaw_)
                                .first;
    const double target_line_lateral =
        axisProgressLateral(entry_line_origin_x_, entry_line_origin_y_,
                            entry_reverse_axis_yaw_)
            .second;
    return distanceTo(entry_target_x_, entry_target_y_) <=
               entry_position_tolerance_m_ ||
           (progress >= std::max(0.0, entry_target_progress_m_ -
                                          entry_position_tolerance_m_) &&
            std::abs(target_line_lateral) <=
                reverse_entry_lateral_tolerance_m_);
  }

  bool reverseOutReached() const {
    if (!poseIsFresh()) {
      return false;
    }
    if (distanceTo(return_anchor_x_, return_anchor_y_) <=
        return_position_tolerance_m_) {
      return true;
    }
    const double return_axis =
        camrod_control::normalizeAngle(entry_reverse_axis_yaw_ + M_PI);
    const double target_dx = return_anchor_x_ - return_start_x_;
    const double target_dy = return_anchor_y_ - return_start_y_;
    const double target_progress =
        std::cos(return_axis) * target_dx + std::sin(return_axis) * target_dy;
    const double current_progress =
        axisProgressLateral(return_start_x_, return_start_y_, return_axis)
            .first;
    const double lateral_to_snap_line =
        axisProgressLateral(return_anchor_x_, return_anchor_y_, return_axis)
            .second;
    const double progress_tolerance = std::max(
        return_position_tolerance_m_, reverse_return_progress_tolerance_m_);
    const double lateral_tolerance = std::max(
        {return_position_tolerance_m_, reverse_entry_lateral_tolerance_m_,
         reverse_return_lateral_tolerance_m_});
    if (std::abs(lateral_to_snap_line) > lateral_tolerance) {
      return false;
    }
    return target_progress >= 0.0
               ? current_progress >= target_progress - progress_tolerance
               : current_progress <= target_progress + progress_tolerance;
  }

  void setPhase(const CampingSiteManeuverPhase phase,
                const std::string &detail) {
    phase_ = phase;
    phase_start_time_ = now();
    yaw_alignment_settling_.reset();
    if (phase_ == CampingSiteManeuverPhase::kReverseOut &&
        last_pose_.has_value()) {
      return_start_x_ = last_pose_->pose.position.x;
      return_start_y_ = last_pose_->pose.position.y;
      publishReversePath(
          return_start_x_, return_start_y_, return_anchor_x_, return_anchor_y_,
          camrod_control::normalizeAngle(entry_target_yaw_ + M_PI));
    }
    RCLCPP_INFO(get_logger(), "camping_site_maneuver_controller %s: %s",
                phaseName(phase_).c_str(), detail.c_str());
    publishServiceState(detail);
    publishStatus(true);
  }

  void setError(const std::string &detail) {
    publishZero();
    setPhase(CampingSiteManeuverPhase::kError, detail);
  }

  void publishServiceState(const std::string &detail) const {
    avg_msgs::msg::AvgServiceState message;
    if (phase_ == CampingSiteManeuverPhase::kAlignEntryYaw ||
        phase_ == CampingSiteManeuverPhase::kReverseIn ||
        phase_ == CampingSiteManeuverPhase::kCrabIn ||
        phase_ == CampingSiteManeuverPhase::kRotate180) {
      message.state = avg_msgs::msg::AvgServiceState::SITE_ENTRY;
      message.state_name = "SITE_ENTRY";
    } else if (phase_ == CampingSiteManeuverPhase::kUnloadWait) {
      message.state = avg_msgs::msg::AvgServiceState::UNLOAD_WAIT;
      message.state_name = "UNLOAD_WAIT";
    } else if (phase_ == CampingSiteManeuverPhase::kWaitReturn) {
      // HH_260721 - Distinguish unloading dwell from the operator
      // return-request wait.
      message.state =
          avg_msgs::msg::AvgServiceState::WAITING_FOR_RETURN_REQUEST;
      message.state_name = "WAITING_FOR_RETURN_REQUEST";
    } else if (phase_ == CampingSiteManeuverPhase::kAlignRetraceYaw ||
               phase_ == CampingSiteManeuverPhase::kAlignReturnRouteYaw ||
               phase_ == CampingSiteManeuverPhase::kReverseOut ||
               phase_ == CampingSiteManeuverPhase::kCrabOut ||
               phase_ == CampingSiteManeuverPhase::kDone) {
      message.state = avg_msgs::msg::AvgServiceState::RETURN_WITH_CARGO;
      message.state_name = "RETURN_WITH_CARGO";
    } else {
      return;
    }
    message.description =
        "camping_site_maneuver_controller:" + phaseName(phase_) + ":" + detail;
    service_state_publisher_->publish(message);
  }

  void publishZero() const {
    command_publisher_->publish(avg_msgs::msg::AvgTwist());
  }

  void publishCrab(const double direction) const {
    avg_msgs::msg::AvgTwist command;
    command.linear.y = direction * crab_speed_mps_;
    command_publisher_->publish(command);
  }

  void publishCrabReturnToAnchor() {
    if (!poseIsFresh()) {
      setError("pose timeout while returning to lanelet snap pose");
      return;
    }
    // HH_260807 - Keep the return in parallel-motion mode while correcting
    // both lateral crab distance and the small longitudinal drift accumulated
    // during zero-turn. Automatic missions target the explicit route-goal snap.
    const double remaining_distance =
        distanceTo(return_anchor_x_, return_anchor_y_);
    // HH_260807 - Proportional slowdown prevents a 10 Hz crab command from
    // stepping across the narrow 4 cm centerline handoff tolerance.
    const double return_speed = std::min(
        crab_speed_mps_, return_translation_gain_ * remaining_distance);
    const auto velocity = camrod_control::bodyTranslationTowardTarget(
        last_pose_->pose.position.x, last_pose_->pose.position.y,
        camrod_control::yawFromPose(*last_pose_), return_anchor_x_,
        return_anchor_y_, return_speed);
    avg_msgs::msg::AvgTwist command;
    command.linear.x = velocity.first;
    command.linear.y = velocity.second;
    command_publisher_->publish(command);
  }

  void publishReversePath(const double start_x, const double start_y,
                          const double end_x, const double end_y,
                          const double robot_yaw) const {
    nav_msgs::msg::Path path;
    path.header.frame_id =
        last_pose_.has_value() && !last_pose_->header.frame_id.empty()
            ? last_pose_->header.frame_id
            : (site_goal_frame_id_.empty() ? "map" : site_goal_frame_id_);
    path.header.stamp = now();
    const double distance = std::hypot(end_x - start_x, end_y - start_y);
    const int steps = std::max(1, static_cast<int>(distance / 0.25));
    for (int index = 0; index <= steps; ++index) {
      const double ratio =
          static_cast<double>(index) / static_cast<double>(steps);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = start_x + (end_x - start_x) * ratio;
      pose.pose.position.y = start_y + (end_y - start_y) * ratio;
      pose.pose.orientation = camrod_control::quaternionFromYaw(robot_yaw);
      path.poses.push_back(pose);
    }
    reverse_path_publisher_->publish(path);
  }

  void requestNav2CancelForSitePhase(const bool force) {
    if (!cancel_nav2_on_site_phase_ || !isSiteInternalPhase()) {
      return;
    }
    const rclcpp::Time current_time = now();
    if (!force && last_nav2_cancel_request_time_.nanoseconds() > 0 &&
        (current_time - last_nav2_cancel_request_time_).seconds() <
            std::max(0.1, nav2_cancel_period_s_)) {
      return;
    }
    bool sent = false;
    for (const auto &client : nav2_cancel_clients_) {
      if (!client->service_is_ready()) {
        continue;
      }
      client->async_send_request(
          std::make_shared<action_msgs::srv::CancelGoal::Request>());
      sent = true;
    }
    if (sent) {
      last_nav2_cancel_request_time_ = current_time;
      if (last_nav2_cancel_log_time_.nanoseconds() == 0 ||
          (current_time - last_nav2_cancel_log_time_).seconds() >= 2.0) {
        last_nav2_cancel_log_time_ = current_time;
        RCLCPP_INFO(get_logger(), "requested Nav2 cancel during %s",
                    phaseName(phase_).c_str());
      }
    }
  }

  void publishReverseAlongAxis(const double target_yaw, const double origin_x,
                               const double origin_y) {
    if (!poseIsFresh()) {
      setError("pose timeout during " + phaseName(phase_));
      return;
    }
    const double heading_error = camrod_control::normalizeAngle(
        target_yaw - camrod_control::yawFromPose(*last_pose_));
    const double reverse_axis =
        camrod_control::normalizeAngle(target_yaw + M_PI);
    const double lateral_error =
        axisProgressLateral(origin_x, origin_y, reverse_axis).second;
    avg_msgs::msg::AvgTwist command;
    command.linear.x = -reverse_entry_speed_mps_;
    command.angular.z =
        camrod_control::clamp(reverse_entry_yaw_gain_ * heading_error +
                                  reverse_entry_lateral_gain_ * lateral_error,
                              -reverse_entry_maximum_angular_speed_radps_,
                              reverse_entry_maximum_angular_speed_radps_);
    command_publisher_->publish(command);
    const rclcpp::Time current_time = now();
    if (reverse_entry_debug_period_s_ > 0.0 &&
        (last_reverse_entry_debug_time_.nanoseconds() == 0 ||
         (current_time - last_reverse_entry_debug_time_).seconds() >=
             reverse_entry_debug_period_s_)) {
      last_reverse_entry_debug_time_ = current_time;
      RCLCPP_INFO(
          get_logger(),
          "reverse control: phase=%s target_yaw=%.1fdeg reverse_axis=%.1fdeg "
          "heading_err=%.1fdeg lateral_err=%.2fm angular_z=%.2f",
          phaseName(phase_).c_str(), target_yaw * 180.0 / M_PI,
          reverse_axis * 180.0 / M_PI, heading_error * 180.0 / M_PI,
          lateral_error, command.angular.z);
    }
  }

  bool publishRotate() {
    if (!poseIsFresh()) {
      setError("pose timeout during " + phaseName(phase_) + " yaw control");
      return false;
    }
    const double yaw = camrod_control::yawFromPose(*last_pose_);
    if (phase_ == CampingSiteManeuverPhase::kRotate180 &&
        rotate_direction_sign_ != 0.0) {
      const double target_delta = rotate_direction_sign_ * M_PI;
      double traveled = camrod_control::normalizeAngle(yaw - start_yaw_);
      if (rotate_direction_sign_ > 0.0 && traveled < 0.0) {
        traveled += 2.0 * M_PI;
      } else if (rotate_direction_sign_ < 0.0 && traveled > 0.0) {
        traveled -= 2.0 * M_PI;
      }
      const double remaining = target_delta - traveled;
      const bool settled = yaw_alignment_settling_.observe(
          remaining, yaw, last_pose_time_.seconds());
      if (yaw_alignment_settling_.withinTolerance()) {
        publishZero();
        return settled;
      }
      avg_msgs::msg::AvgTwist command;
      if (rotate_direction_sign_ * remaining > 0.0) {
        command.angular.z =
            rotate_direction_sign_ *
            std::min(maximum_angular_speed_radps_,
                     std::max(0.05,
                              std::abs(rotate_proportional_gain_ * remaining)));
      } else {
        command.angular.z = camrod_control::clamp(
            rotate_proportional_gain_ *
                camrod_control::normalizeAngle(target_yaw_ - yaw),
            -maximum_angular_speed_radps_, maximum_angular_speed_radps_);
      }
      command_publisher_->publish(command);
      return false;
    }
    const double error = camrod_control::normalizeAngle(target_yaw_ - yaw);
    const bool settled =
        yaw_alignment_settling_.observe(error, yaw, last_pose_time_.seconds());
    if (yaw_alignment_settling_.withinTolerance()) {
      publishZero();
      return settled;
    }
    avg_msgs::msg::AvgTwist command;
    command.angular.z = camrod_control::clamp(rotate_proportional_gain_ * error,
                                              -maximum_angular_speed_radps_,
                                              maximum_angular_speed_radps_);
    command_publisher_->publish(command);
    return false;
  }

  void publishReturnRequest(const std::string &reason) {
    if (!request_return_to_drop_zone_on_done_ || return_acknowledged_) {
      return;
    }
    const rclcpp::Time current_time = now();
    if (return_published_ && return_request_retry_period_s_ > 0.0 &&
        (current_time - last_return_request_publish_time_).seconds() <
            return_request_retry_period_s_) {
      return;
    }
    avg_msgs::msg::PlanningRecallRequest message;
    message.header.stamp = current_time;
    message.site_name = site_goal_key_;
    message.source = "camping_site_maneuver_controller:" + reason;
    return_request_publisher_->publish(message);
    return_published_ = true;
    last_return_request_publish_time_ = current_time;
    RCLCPP_INFO(get_logger(), "return_to_drop_zone requested: %s",
                reason.c_str());
  }

  void onTimer() {
    const double elapsed = (now() - phase_start_time_).seconds();
    requestNav2CancelForSitePhase(false);
    if (phase_ == CampingSiteManeuverPhase::kAlignEntryYaw) {
      if (publishRotate()) {
        setPhase(CampingSiteManeuverPhase::kReverseIn,
                 "entry yaw aligned for reverse site approach");
      }
    } else if (phase_ == CampingSiteManeuverPhase::kReverseIn) {
      if (reverseInReached()) {
        publishZero();
        target_yaw_ = camrod_control::normalizeAngle(entry_target_yaw_ + M_PI);
        setPhase(CampingSiteManeuverPhase::kRotate180,
                 "reverse entry pose reached");
      } else if (elapsed > crab_duration_s_ + crab_timeout_margin_s_) {
        setError("reverse entry timeout before reaching site target");
      } else {
        publishReverseAlongAxis(entry_target_yaw_, entry_line_origin_x_,
                                entry_line_origin_y_);
      }
    } else if (phase_ == CampingSiteManeuverPhase::kCrabIn) {
      if (entryReached()) {
        publishZero();
        // HH_260721 - Keep the route heading at constrained roadside service
        // poses.
        if (active_service_mode_ == CampsiteServiceMode::kRoadsideStop) {
          target_yaw_ = start_yaw_;
          setPhase(CampingSiteManeuverPhase::kUnloadWait,
                   "roadside service pose reached; zero-turn skipped");
        } else {
          setPhase(CampingSiteManeuverPhase::kRotate180,
                   "crab entry pose reached");
        }
      } else if (elapsed > crab_duration_s_ + crab_timeout_margin_s_) {
        setError("crab entry timeout before reaching site offset");
      } else {
        publishCrab(crab_direction_);
      }
    } else if (phase_ == CampingSiteManeuverPhase::kRotate180) {
      if (publishRotate()) {
        setPhase(CampingSiteManeuverPhase::kUnloadWait,
                 "robot yaw rotated 180deg");
      }
    } else if (phase_ == CampingSiteManeuverPhase::kUnloadWait) {
      publishZero();
      if (elapsed >= unload_wait_s_) {
        if (auto_return_after_unload_wait_ || return_requested_) {
          beginReturnExit("unload wait complete");
        } else {
          setPhase(CampingSiteManeuverPhase::kWaitReturn,
                   "waiting external return command");
        }
      }
    } else if (phase_ == CampingSiteManeuverPhase::kWaitReturn) {
      publishZero();
    } else if (phase_ == CampingSiteManeuverPhase::kAlignRetraceYaw) {
      if (publishRotate()) {
        setPhase(CampingSiteManeuverPhase::kCrabOut,
                 "entry-lanelet retrace yaw confirmed");
      }
    } else if (phase_ == CampingSiteManeuverPhase::kAlignReturnRouteYaw) {
      if (publishRotate()) {
        // HH_260721 - Publish the drop-zone request only after yaw selects the
        // reversed lanelet path.
        setPhase(CampingSiteManeuverPhase::kDone,
                 "aligned with reversed route at lanelet snap pose");
        publishReturnRequest("done_after_return_route_alignment");
      }
    } else if (phase_ == CampingSiteManeuverPhase::kReverseOut) {
      if (reverseOutReached()) {
        publishZero();
        setPhase(CampingSiteManeuverPhase::kDone,
                 "returned to lanelet snap pose");
        publishReturnRequest("done");
      } else if (elapsed >
                 crab_duration_s_ + reverse_return_timeout_margin_s_) {
        setError("reverse return timeout before reaching lanelet snap pose");
      } else {
        publishReverseAlongAxis(
            camrod_control::normalizeAngle(entry_target_yaw_ + M_PI),
            return_anchor_x_, return_anchor_y_);
      }
    } else if (phase_ == CampingSiteManeuverPhase::kCrabOut) {
      if (returnReached()) {
        publishZero();
        if (active_service_mode_ == CampsiteServiceMode::kRoadsideStop) {
          // HH_260721 - Roadside stops rotate on the lane, not beside B12/B13
          // obstacles.
          target_yaw_ = camrod_control::normalizeAngle(start_yaw_ + M_PI);
          setPhase(CampingSiteManeuverPhase::kAlignReturnRouteYaw,
                   "roadside exit reached lanelet snap pose; align with "
                   "reversed route");
        } else {
          setPhase(
              CampingSiteManeuverPhase::kDone,
              "returned to lanelet snap pose anchor=" + return_anchor_source_ +
                  " error=" +
                  fixed(distanceTo(return_anchor_x_, return_anchor_y_)) + "m");
          publishReturnRequest("done");
        }
      } else if (elapsed > crab_duration_s_ + crab_timeout_margin_s_) {
        setError("crab return timeout before reaching lanelet snap pose");
      } else {
        publishCrabReturnToAnchor();
      }
    } else if (phase_ == CampingSiteManeuverPhase::kDone) {
      publishReturnRequest("done_retry");
    }
    publishStatus(false);
  }

  void publishStatus(const bool force) {
    const rclcpp::Time current_time = now();
    if (!force && last_status_publish_time_.nanoseconds() > 0 &&
        (current_time - last_status_publish_time_).seconds() <
            1.0 / status_publish_rate_hz_) {
      return;
    }
    // HH_260721 - Entry, unload wait, return wait, and exit are normal
    // operating states.
    const uint8_t module_level = phase_ == CampingSiteManeuverPhase::kError
                                     ? avg_msgs::msg::ModuleState::ERROR
                                     : avg_msgs::msg::ModuleState::OK;
    const std::string message =
        "phase=" + phaseName(phase_) +
        " service_mode=" + serviceModeName(active_service_mode_) +
        " return_published=" + (return_published_ ? "True" : "False") +
        " return_ack=" + (return_acknowledged_ ? "True" : "False");
    status_publisher_->publish(camrod_control::makeModuleState(
        *this, "control", module_level, message, phaseName(phase_)));
    const uint8_t diagnostic_level =
        module_level == avg_msgs::msg::ModuleState::ERROR
            ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
        : module_level == avg_msgs::msg::ModuleState::WARN
            ? diagnostic_msgs::msg::DiagnosticStatus::WARN
            : diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics_publisher_->publish(camrod_control::makeDiagnostics(
        *this, "control/camping_site_maneuver_controller", "control",
        diagnostic_level, message,
        {
            {"phase", phaseName(phase_)},
            {"cmd_vel_topic", command_topic_},
            {"crab_duration_s", fixed(crab_duration_s_)},
            {"crab_offset_m", fixed(crab_offset_m_)},
            {"crab_source", crab_source_},
            {"return_anchor_source", return_anchor_source_},
            {"return_anchor_offset_m", fixed(return_anchor_offset_m_)},
        }));
    last_status_publish_time_ = current_time;
  }

  std::string command_topic_;
  std::string pose_topic_;
  std::string site_goal_topic_;
  std::string route_goal_topic_;
  std::string lanelet_pose_topic_;
  std::string planning_state_topic_;
  std::string operation_topic_;
  std::string adopt_destination_topic_;
  std::string status_topic_;
  std::string diagnostics_topic_;
  std::string service_state_topic_;
  std::string reverse_path_topic_;
  std::string campsite_occupancy_topic_;
  bool enable_auto_start_from_planning_state_{true};
  std::string site_mission_key_prefix_{"camping_site_"};
  bool use_goal_pair_for_lateral_offset_{true};
  bool require_goal_pair_for_auto_start_{true};
  std::string site_entry_mode_{"crab"};
  std::string camping_sites_yaml_;
  std::string site_goal_frame_id_{"map"};
  double default_lateral_offset_m_{1.2};
  double minimum_lateral_offset_m_{0.2};
  double maximum_lateral_offset_m_{7.0};
  double roadside_maximum_lateral_offset_m_{0.60};
  std::string default_lateral_direction_{"left"};
  double crab_speed_mps_{0.18};
  double entry_position_tolerance_m_{0.15};
  double return_position_tolerance_m_{0.04};
  double return_translation_gain_{4.0};
  double crab_timeout_margin_s_{3.0};
  double reverse_return_timeout_margin_s_{45.0};
  double reverse_return_progress_tolerance_m_{0.45};
  double reverse_return_lateral_tolerance_m_{0.40};
  double crab_timeout_speed_scale_{1.0};
  double maximum_forward_residual_m_{0.8};
  double rotate_proportional_gain_{1.2};
  double maximum_angular_speed_radps_{0.35};
  double rotate_yaw_tolerance_deg_{4.0};
  double rotate_settle_hold_s_{0.8};
  double rotate_settle_max_rate_degps_{3.0};
  std::string site_rotate_direction_policy_{"site_index_lanelet_side"};
  std::set<int64_t> right_lanelet_site_indices_;
  std::set<int64_t> left_lanelet_site_indices_;
  double reverse_entry_speed_mps_{0.16};
  double reverse_entry_yaw_gain_{1.0};
  double reverse_entry_lateral_gain_{-0.25};
  double reverse_entry_maximum_angular_speed_radps_{0.25};
  bool reverse_entry_use_site_goal_yaw_{true};
  std::string reverse_entry_site_yaw_mode_{"reverse_axis"};
  bool reverse_entry_auto_select_yaw_equivalent_{true};
  double reverse_entry_lateral_tolerance_m_{0.35};
  double reverse_entry_debug_period_s_{1.0};
  double unload_wait_s_{5.0};
  bool auto_return_after_unload_wait_{false};
  bool reset_wait_return_on_site_goal_{true};
  bool request_return_to_drop_zone_on_done_{true};
  bool align_retrace_yaw_before_crab_out_{true};
  double return_request_retry_period_s_{1.0};
  std::string return_to_drop_zone_topic_;
  bool cancel_nav2_on_site_phase_{true};
  double nav2_cancel_period_s_{0.5};
  std::vector<std::string> nav2_cancel_action_topics_;
  double pose_timeout_s_{2.0};
  double route_goal_reached_distance_m_{0.9};
  double adopt_site_arrival_distance_m_{3.0};
  double goal_pair_maximum_age_s_{0.0};
  double control_rate_hz_{10.0};
  double idle_tick_rate_hz_{1.0};
  double status_publish_rate_hz_{1.0};

  CampingSiteManeuverPhase phase_{CampingSiteManeuverPhase::kIdle};
  CampsiteServiceMode active_service_mode_{CampsiteServiceMode::kTurnaround};
  std::optional<avg_msgs::msg::AvgPoseStamped> last_pose_;
  std::optional<avg_msgs::msg::AvgPoseStamped> site_goal_;
  std::optional<avg_msgs::msg::AvgPoseStamped> route_goal_;
  std::optional<avg_msgs::msg::AvgPoseStamped> lanelet_pose_;
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time site_goal_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time route_goal_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lanelet_pose_time_{0, 0, RCL_ROS_TIME};
  std::string site_goal_key_;
  rclcpp::Time phase_start_time_{0, 0, RCL_ROS_TIME};
  camrod_control::YawAlignmentSettling yaw_alignment_settling_;
  double start_yaw_{0.0};
  double target_yaw_{0.0};
  double rotate_direction_sign_{0.0};
  std::string rotate_direction_label_{"shortest"};
  double entry_target_yaw_{0.0};
  double entry_reverse_axis_yaw_{0.0};
  double entry_line_origin_x_{0.0};
  double entry_line_origin_y_{0.0};
  double entry_target_progress_m_{0.0};
  double entry_target_x_{0.0};
  double entry_target_y_{0.0};
  double entry_reverse_distance_m_{0.0};
  double return_start_x_{0.0};
  double return_start_y_{0.0};
  double crab_direction_{1.0};
  double crab_duration_s_{0.0};
  double crab_offset_m_{0.0};
  std::string crab_source_{"default"};
  double goal_pair_forward_m_{0.0};
  double entry_start_x_{0.0};
  double entry_start_y_{0.0};
  double entry_reference_yaw_{0.0};
  double return_anchor_x_{0.0};
  double return_anchor_y_{0.0};
  double return_anchor_offset_m_{0.0};
  std::string return_anchor_source_{"actual_arrival_pose"};
  std::string last_auto_key_;
  bool return_requested_{false};
  bool return_published_{false};
  bool return_acknowledged_{false};
  rclcpp::Time last_return_request_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_nav2_cancel_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_nav2_cancel_log_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_reverse_entry_debug_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_publish_time_{0, 0, RCL_ROS_TIME};
  std::map<std::string, avg_msgs::msg::AvgPoseStamped> camping_site_goals_;
  std::map<std::string, CampsiteServiceMode> camping_site_service_modes_;
  std::set<std::string> occupied_mission_keys_;

  rclcpp::Publisher<avg_msgs::msg::AvgTwist>::SharedPtr command_publisher_;
  rclcpp::Publisher<avg_msgs::msg::ModuleState>::SharedPtr status_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::Publisher<avg_msgs::msg::PlanningRecallRequest>::SharedPtr
      return_request_publisher_;
  rclcpp::Publisher<avg_msgs::msg::AvgServiceState>::SharedPtr
      service_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reverse_path_publisher_;
  std::vector<rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr>
      nav2_cancel_clients_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr
      pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      site_goal_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr
      route_goal_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr
      lanelet_pose_subscription_;
  rclcpp::Subscription<avg_msgs::msg::PlanningState>::SharedPtr
      planning_state_subscription_;
  rclcpp::Subscription<avg_msgs::msg::MotionOperation>::SharedPtr
      operation_subscription_;
  rclcpp::Subscription<avg_msgs::msg::UiDestinationCommand>::SharedPtr
      adopt_destination_subscription_;
  rclcpp::Subscription<avg_msgs::msg::CampsiteOccupancy>::SharedPtr
      campsite_occupancy_subscription_;
  rclcpp::Service<avg_msgs::srv::RequestMotionOperation>::SharedPtr
      operation_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CampingSiteManeuverControllerNode>());
  rclcpp::shutdown();
  return 0;
}
