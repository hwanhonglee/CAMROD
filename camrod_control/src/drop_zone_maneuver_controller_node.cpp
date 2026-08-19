// HH_260721 - Implement drop-zone departure and parking-yaw alignment in C++.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "avg_msgs/msg/avg_bool.hpp"
#include "avg_msgs/msg/avg_platform_status.hpp"
#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "avg_msgs/msg/avg_service_state.hpp"
#include "avg_msgs/msg/avg_twist.hpp"
#include "avg_msgs/msg/module_state.hpp"
#include "avg_msgs/msg/motion_operation.hpp"
#include "avg_msgs/msg/planning_scenario.hpp"
#include "avg_msgs/msg/planning_state.hpp"
#include "avg_msgs/srv/request_motion_operation.hpp"
#include "camrod_control/control_diagnostics.hpp"
#include "camrod_control/drop_zone_exit_target_policy.hpp"
#include "camrod_control/drop_zone_station_pose.hpp"
#include "camrod_control/motion_geometry.hpp"
#include "camrod_control/reverse_parking_axis.hpp"
#include "camrod_control/yaw_alignment_settling.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

enum class DropZoneManeuverPhase {
  kIdle,
  kExitStraight,
  kAlignExitYaw,
  kAlignParkingYaw,
  kError,
};

std::string phaseName(const DropZoneManeuverPhase phase) {
  switch (phase) {
  case DropZoneManeuverPhase::kIdle:
    return "IDLE";
  case DropZoneManeuverPhase::kExitStraight:
    return "EXIT_STRAIGHT";
  case DropZoneManeuverPhase::kAlignExitYaw:
    return "ALIGN_EXIT_YAW";
  case DropZoneManeuverPhase::kAlignParkingYaw:
    return "ALIGN_PARKING_YAW";
  case DropZoneManeuverPhase::kError:
    return "ERROR";
  }
  return "ERROR";
}

std::string fixed(const double value, const int precision = 3) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

} // namespace

class DropZoneManeuverControllerNode : public rclcpp::Node {
public:
  DropZoneManeuverControllerNode() : Node("drop_zone_maneuver_controller") {
    // HH_260721 - Preserve explicit control-owned operation and command
    // interfaces.
    command_topic_ =
        declare_parameter<std::string>("command_topic", "/control/cmd_vel_raw");
    vehicle_pose_topic_ = declare_parameter<std::string>("vehicle_pose_topic",
                                                         "/localization/pose");
    // HH_260721 - Classify departure from live charging feedback instead of
    // inferring it from UI state.
    platform_status_topic_ = declare_parameter<std::string>(
        "platform_status_topic", "/platform/status");
    planning_state_topic_ = declare_parameter<std::string>(
        "planning_state_topic", "/planning/state_machine/state");
    lanelet_pose_topic_ = declare_parameter<std::string>(
        "lanelet_pose_topic", "/planning/lanelet_pose");
    drop_zone_goal_topic_ = declare_parameter<std::string>(
        "drop_zone_goal_topic", "/planning/drop_zone_goal_raw");
    operation_topic_ = declare_parameter<std::string>(
        "operation_topic", "/control/drop_zone_maneuver_controller/operation");
    exit_complete_topic_ = declare_parameter<std::string>(
        "exit_complete_topic", "/control/drop_zone/exit_complete");
    status_topic_ = declare_parameter<std::string>(
        "status_topic", "/control/drop_zone_maneuver_controller/status");
    exit_path_topic_ = declare_parameter<std::string>(
        "exit_path_topic",
        "/control/drop_zone_maneuver_controller/exit_path_ros");
    parking_operation_topic_ = declare_parameter<std::string>(
        "parking_operation_topic", "/parking/operation");
    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic",
                                                        "/system/diagnostics");
    service_state_topic_ =
        declare_parameter<std::string>("service_state_topic", "/service/state");

    drop_zones_yaml_ = declare_parameter<std::string>("drop_zones_yaml", "");
    drop_zone_id_ = declare_parameter<std::string>("drop_zone_id", "drop_zone");
    use_drop_zone_pose_as_station_ =
        declare_parameter<bool>("use_drop_zone_pose_as_station", true);
    station_pose_ = camrod_control::loadDropZoneStationPose(
        get_logger(), drop_zones_yaml_, drop_zone_id_,
        use_drop_zone_pose_as_station_,
        camrod_control::DropZoneStationPose{
            declare_parameter<double>("station_x_m", 0.0),
            declare_parameter<double>("station_y_m", 0.0),
            declare_parameter<double>("station_yaw_deg", 0.0) * M_PI / 180.0});

    enable_auto_alignment_ =
        declare_parameter<bool>("enable_auto_alignment", true);
    return_scenario_id_ = declare_parameter<int>(
        "return_scenario_id",
        avg_msgs::msg::PlanningScenario::RETURN_TO_DROP_ZONE);
    return_mission_key_ =
        declare_parameter<std::string>("return_mission_key", "drop_zone");
    station_yaw_represents_reverse_axis_ =
        declare_parameter<bool>("station_yaw_represents_reverse_axis", true);
    automatically_select_reverse_approach_yaw_ = declare_parameter<bool>(
        "automatically_select_reverse_approach_yaw", false);
    yaw_tolerance_deg_ =
        std::abs(declare_parameter<double>("yaw_tolerance_deg", 5.0));
    yaw_proportional_gain_ =
        declare_parameter<double>("yaw_proportional_gain", 1.2);
    maximum_angular_speed_radps_ = std::abs(
        declare_parameter<double>("maximum_angular_speed_radps", 0.35));
    // HH_260810 - One in-tolerance sample is not enough to hand a rotating
    // platform to reverse parking. Require continuous low-rate convergence.
    yaw_settle_hold_s_ =
        std::abs(declare_parameter<double>("yaw_settle_hold_s", 1.0));
    yaw_settle_max_rate_degps_ =
        std::abs(declare_parameter<double>("yaw_settle_max_rate_degps", 3.0));
    yaw_alignment_settling_.setConfig(
        camrod_control::YawAlignmentSettlingConfig{yaw_tolerance_deg_,
                                                   yaw_settle_hold_s_,
                                                   yaw_settle_max_rate_degps_});
    exit_distance_m_ =
        std::abs(declare_parameter<double>("exit_distance_m", 1.2));
    require_lanelet_exit_target_ =
        declare_parameter<bool>("require_lanelet_exit_target", true);
    allow_fixed_distance_fallback_ =
        declare_parameter<bool>("allow_fixed_distance_fallback", false);
    exit_target_min_distance_m_ =
        declare_parameter<double>("exit_target_min_distance_m", 0.3);
    exit_target_max_distance_m_ =
        declare_parameter<double>("exit_target_max_distance_m", 8.0);
    exit_target_min_forward_m_ =
        declare_parameter<double>("exit_target_min_forward_m", 0.1);
    exit_target_max_lateral_m_ =
        declare_parameter<double>("exit_target_max_lateral_m", 1.0);
    exit_position_tolerance_m_ =
        declare_parameter<double>("exit_position_tolerance_m", 0.20);
    exit_speed_mps_ =
        std::abs(declare_parameter<double>("exit_speed_mps", 0.16));
    exit_timeout_s_ =
        std::abs(declare_parameter<double>("exit_timeout_s", 30.0));
    pose_timeout_s_ =
        std::abs(declare_parameter<double>("pose_timeout_s", 2.0));
    allow_unstamped_pose_fallback_ = declare_parameter<bool>(
        "allow_unstamped_pose_fallback", false);
    exit_target_policy_config_ = camrod_control::DropZoneExitTargetPolicyConfig{
        require_lanelet_exit_target_, allow_fixed_distance_fallback_,
        pose_timeout_s_, exit_target_min_distance_m_,
        exit_target_max_distance_m_, exit_target_min_forward_m_,
        exit_target_max_lateral_m_, exit_distance_m_};
    exit_target_policy_config_.allow_unstamped_pose_fallback =
        allow_unstamped_pose_fallback_;
    if (!camrod_control::dropZoneExitTargetConfigIsValid(
            exit_target_policy_config_) ||
        !std::isfinite(exit_position_tolerance_m_) ||
        exit_position_tolerance_m_ <= 0.0) {
      throw std::invalid_argument(
          "invalid drop-zone exit-target safety configuration");
    }
    control_rate_hz_ =
        std::max(0.1, declare_parameter<double>("control_rate_hz", 10.0));
    idle_rate_hz_ =
        std::max(0.1, declare_parameter<double>("idle_rate_hz", 1.0));
    status_rate_hz_ =
        std::max(0.1, declare_parameter<double>("status_rate_hz", 1.0));

    command_publisher_ =
        create_publisher<avg_msgs::msg::AvgTwist>(command_topic_, 10);
    status_publisher_ =
        create_publisher<avg_msgs::msg::ModuleState>(status_topic_, 10);
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            diagnostics_topic_, 10);
    service_state_publisher_ = create_publisher<avg_msgs::msg::AvgServiceState>(
        service_state_topic_, 10);
    exit_path_publisher_ =
        create_publisher<nav_msgs::msg::Path>(exit_path_topic_, 10);
    exit_complete_publisher_ =
        create_publisher<avg_msgs::msg::AvgBool>(exit_complete_topic_, 10);
    parking_operation_publisher_ =
        create_publisher<avg_msgs::msg::MotionOperation>(
            parking_operation_topic_, 10);

    vehicle_pose_subscription_ =
        create_subscription<avg_msgs::msg::AvgPoseStamped>(
            vehicle_pose_topic_, 10,
            [this](const avg_msgs::msg::AvgPoseStamped::SharedPtr message) {
              last_vehicle_pose_ = *message;
              last_vehicle_pose_time_ = now();
            });
    platform_status_subscription_ =
        create_subscription<avg_msgs::msg::AvgPlatformStatus>(
            platform_status_topic_, 10,
            [this](const avg_msgs::msg::AvgPlatformStatus::SharedPtr message) {
              is_charging_ = message->is_charging;
            });
    lanelet_pose_subscription_ =
        create_subscription<avg_msgs::msg::AvgPoseStamped>(
            lanelet_pose_topic_, 10,
            [this](const avg_msgs::msg::AvgPoseStamped::SharedPtr message) {
              last_lanelet_pose_ = *message;
              last_lanelet_pose_time_ = now();
            });
    drop_zone_goal_subscription_ =
        create_subscription<avg_msgs::msg::AvgPoseStamped>(
            drop_zone_goal_topic_, 10,
            [this](const avg_msgs::msg::AvgPoseStamped::SharedPtr message) {
              onDropZoneGoal(*message);
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
    operation_service_ = create_service<avg_msgs::srv::RequestMotionOperation>(
        "/control/drop_zone_maneuver_controller/request_operation",
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

    target_body_yaw_rad_ = configuredParkingBodyYaw();
    phase_start_time_ = now();
    control_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_rate_hz_),
        std::bind(&DropZoneManeuverControllerNode::onTimer, this));
    RCLCPP_INFO(get_logger(),
                "drop_zone_maneuver_controller ready: "
                "station=(%.2f,%.2f,%.1fdeg) command=%s parking_operation=%s",
                station_pose_.x_m, station_pose_.y_m,
                station_pose_.yaw_rad * 180.0 / M_PI, command_topic_.c_str(),
                parking_operation_topic_.c_str());
  }

private:
  bool isActive() const {
    return phase_ != DropZoneManeuverPhase::kIdle &&
           phase_ != DropZoneManeuverPhase::kError;
  }

  double configuredParkingBodyYaw() const {
    return station_yaw_represents_reverse_axis_
               ? camrod_control::bodyYawForReverseAxis(station_pose_.yaw_rad)
               : camrod_control::normalizeAngle(station_pose_.yaw_rad);
  }

  void onDropZoneGoal(const avg_msgs::msg::AvgPoseStamped &message) {
    if (isActive()) {
      RCLCPP_WARN(get_logger(),
                  "ignored drop-zone goal update while maneuver is active");
      return;
    }
    station_pose_ = camrod_control::DropZoneStationPose{
        message.pose.position.x, message.pose.position.y,
        camrod_control::yawFromPose(message)};
    target_body_yaw_rad_ = configuredParkingBodyYaw();
  }

  void onPlanningState(const avg_msgs::msg::PlanningState &message) {
    if (!enable_auto_alignment_) {
      return;
    }
    const bool drop_zone_arrival =
        message.state == avg_msgs::msg::PlanningState::GOAL_REACHED &&
        message.scenario_id == return_scenario_id_ &&
        message.active_mission_key == return_mission_key_;
    if (!drop_zone_arrival) {
      if (!isActive()) {
        last_auto_alignment_key_.clear();
      }
      return;
    }
    const std::string alignment_key =
        std::to_string(message.scenario_id) + ":" + message.active_mission_key;
    if (alignment_key == last_auto_alignment_key_) {
      return;
    }
    last_auto_alignment_key_ = alignment_key;
    startParkingAlignment("planning_state:return_to_drop_zone");
  }

  std::pair<bool, std::string> applyOperation(const uint8_t operation,
                                              const std::string &source) {
    if (operation == avg_msgs::msg::MotionOperation::ALIGN_FOR_PARKING) {
      return startParkingAlignment(source);
    }
    if (operation == avg_msgs::msg::MotionOperation::EXIT) {
      return startExit(source);
    }
    if (operation == avg_msgs::msg::MotionOperation::CANCEL) {
      road_handoff_ready_ = false;
      publishZero();
      setPhase(DropZoneManeuverPhase::kIdle, "cancel=" + source);
      return {true, "drop-zone maneuver cancelled"};
    }
    return {false,
            "unsupported drop-zone operation=" + std::to_string(operation)};
  }

  bool vehiclePoseIsFresh() const {
    if (!last_vehicle_pose_.has_value() ||
        last_vehicle_pose_->header.frame_id.empty()) {
      return false;
    }
    const rclcpp::Time current_time = now();
    const bool header_stamp_available =
        last_vehicle_pose_->header.stamp.sec != 0 ||
        last_vehicle_pose_->header.stamp.nanosec != 0U;
    double header_stamp_age_s = 0.0;
    if (header_stamp_available) {
      const double header_stamp_sec =
          static_cast<double>(last_vehicle_pose_->header.stamp.sec) +
          static_cast<double>(last_vehicle_pose_->header.stamp.nanosec) *
              1.0e-9;
      header_stamp_age_s = current_time.seconds() - header_stamp_sec;
    }
    return camrod_control::dropZoneExitPoseSampleIsFresh(
        (current_time - last_vehicle_pose_time_).seconds(),
        header_stamp_available, header_stamp_age_s,
        allow_unstamped_pose_fallback_, pose_timeout_s_);
  }

  std::pair<bool, std::string>
  startParkingAlignment(const std::string &source) {
    if (isActive()) {
      return {false, "drop-zone maneuver already active: " + phaseName(phase_)};
    }
    road_handoff_ready_ = false;
    if (!vehiclePoseIsFresh()) {
      setError("fresh pose unavailable for parking alignment");
      return {false, "fresh pose unavailable for parking alignment"};
    }
    target_body_yaw_rad_ = selectParkingBodyYaw();
    setPhase(DropZoneManeuverPhase::kAlignParkingYaw, "start=" + source);
    return {true, "drop-zone parking alignment started"};
  }

  std::pair<bool, std::string> startExit(const std::string &source) {
    if (isActive()) {
      publishExitComplete(false);
      return {false, "drop-zone maneuver already active: " + phaseName(phase_)};
    }
    road_handoff_ready_ = false;
    exit_target_ = camrod_control::DropZoneExitTargetSelection{};
    exit_target_frame_id_.clear();
    if (!vehiclePoseIsFresh()) {
      publishExitComplete(false);
      setError("fresh pose unavailable for drop-zone exit");
      return {false, "fresh pose unavailable for drop-zone exit"};
    }
    const rclcpp::Time capture_time = now();
    exit_start_x_m_ = last_vehicle_pose_->pose.position.x;
    exit_start_y_m_ = last_vehicle_pose_->pose.position.y;
    const double start_yaw_rad =
        camrod_control::yawFromPose(*last_vehicle_pose_);
    camrod_control::DropZoneExitTargetPolicyInput target_input;
    target_input.vehicle_x_m = exit_start_x_m_;
    target_input.vehicle_y_m = exit_start_y_m_;
    target_input.vehicle_yaw_rad = start_yaw_rad;
    target_input.vehicle_frame_id = last_vehicle_pose_->header.frame_id;
    target_input.vehicle_age_s =
        (capture_time - last_vehicle_pose_time_).seconds();
    if (last_vehicle_pose_->header.stamp.sec != 0 ||
        last_vehicle_pose_->header.stamp.nanosec != 0U) {
      target_input.vehicle_header_stamp_available = true;
      const double header_stamp_sec =
          static_cast<double>(last_vehicle_pose_->header.stamp.sec) +
          static_cast<double>(last_vehicle_pose_->header.stamp.nanosec) *
              1.0e-9;
      target_input.vehicle_header_stamp_age_s =
          capture_time.seconds() - header_stamp_sec;
    }
    target_input.lanelet_pose_available = last_lanelet_pose_.has_value();
    if (last_lanelet_pose_.has_value()) {
      target_input.lanelet_x_m = last_lanelet_pose_->pose.position.x;
      target_input.lanelet_y_m = last_lanelet_pose_->pose.position.y;
      target_input.lanelet_yaw_rad =
          camrod_control::yawFromPose(*last_lanelet_pose_);
      target_input.lanelet_frame_id = last_lanelet_pose_->header.frame_id;
      target_input.lanelet_age_s =
          (capture_time - last_lanelet_pose_time_).seconds();
      if (last_lanelet_pose_->header.stamp.sec != 0 ||
          last_lanelet_pose_->header.stamp.nanosec != 0U) {
        target_input.lanelet_header_stamp_available = true;
        const double header_stamp_sec =
            static_cast<double>(last_lanelet_pose_->header.stamp.sec) +
            static_cast<double>(last_lanelet_pose_->header.stamp.nanosec) *
                1.0e-9;
        target_input.lanelet_header_stamp_age_s =
            capture_time.seconds() - header_stamp_sec;
      }
    }
    exit_target_ = camrod_control::makeDropZoneExitTargetSelection(
        exit_target_policy_config_, target_input);
    if (!exit_target_.valid) {
      const std::string detail =
          "drop-zone exit target rejected: " + exit_target_.detail;
      publishExitComplete(false);
      setError(detail);
      return {false, detail};
    }
    exit_target_frame_id_ = "map";
    if (exit_target_.source ==
            camrod_control::DropZoneExitTargetSource::kLaneletPose &&
        !target_input.lanelet_frame_id.empty()) {
      exit_target_frame_id_ = target_input.lanelet_frame_id;
    } else if (!target_input.vehicle_frame_id.empty()) {
      exit_target_frame_id_ = target_input.vehicle_frame_id;
    }
    exit_heading_yaw_rad_ = std::atan2(
        exit_target_.target_y_m - exit_start_y_m_,
        exit_target_.target_x_m - exit_start_x_m_);
    // HH_260819 - Final yaw belongs to the same immutable lanelet sample as
    // the exit XY target; live snap updates cannot move either target.
    target_body_yaw_rad_ = exit_target_.target_yaw_rad;
    // HH_260721 - Preserve departure origin while charger feedback clears
    // during physical exit.
    exit_started_while_charging_ = is_charging_;
    setPhase(
        DropZoneManeuverPhase::kExitStraight,
        "start=" + source + " target_source=" +
            camrod_control::dropZoneExitTargetSourceName(exit_target_.source) +
            " target_distance_m=" + fixed(exit_target_.initial_distance_m));
    return {true, "drop-zone exit started"};
  }

  void setPhase(const DropZoneManeuverPhase phase, const std::string &detail) {
    phase_ = phase;
    phase_start_time_ = now();
    yaw_alignment_settling_.reset();
    RCLCPP_INFO(get_logger(), "drop_zone_maneuver_controller %s: %s",
                phaseName(phase_).c_str(), detail.c_str());
    if (phase_ == DropZoneManeuverPhase::kExitStraight ||
        phase_ == DropZoneManeuverPhase::kAlignExitYaw) {
      publishExitPath();
    }
    publishServiceState(detail);
    publishStatus(true);
  }

  void setError(const std::string &detail) {
    publishZero();
    setPhase(DropZoneManeuverPhase::kError, detail);
  }

  void publishServiceState(const std::string &detail) const {
    avg_msgs::msg::AvgServiceState message;
    if (phase_ == DropZoneManeuverPhase::kExitStraight ||
        phase_ == DropZoneManeuverPhase::kAlignExitYaw) {
      // HH_260721 - Distinguish charger departure from an uncharged drop-zone
      // departure.
      message.state = exit_started_while_charging_
                          ? avg_msgs::msg::AvgServiceState::DEPARTING_CHARGER
                          : avg_msgs::msg::AvgServiceState::DEPARTING_DROP_ZONE;
      message.state_name = exit_started_while_charging_ ? "DEPARTING_CHARGER"
                                                        : "DEPARTING_DROP_ZONE";
    } else if (phase_ == DropZoneManeuverPhase::kAlignParkingYaw) {
      message.state = avg_msgs::msg::AvgServiceState::DROP_ZONE_PARKING;
      message.state_name = "DROP_ZONE_PARKING";
    } else {
      return;
    }
    message.description =
        "drop_zone_maneuver_controller:" + phaseName(phase_) + ":" + detail;
    service_state_publisher_->publish(message);
  }

  void publishZero() const {
    command_publisher_->publish(avg_msgs::msg::AvgTwist());
  }

  void publishExitComplete(const bool success) const {
    avg_msgs::msg::AvgBool message;
    message.data = success;
    exit_complete_publisher_->publish(message);
  }

  void requestParkingStart() const {
    // HH_260721 - Start the selected parking controller only after yaw
    // convergence.
    avg_msgs::msg::MotionOperation message;
    message.header.stamp = now();
    message.operation = avg_msgs::msg::MotionOperation::START;
    message.source = "drop_zone_maneuver_controller";
    parking_operation_publisher_->publish(message);
  }

  void publishExitPath() const {
    if (!exit_target_.valid) {
      return;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = exit_target_frame_id_;
    path.header.stamp = now();
    const int step_count =
        std::max(1, static_cast<int>(exit_target_.initial_distance_m / 0.25));
    for (int step = 0; step <= step_count; ++step) {
      const double ratio =
          static_cast<double>(step) / static_cast<double>(step_count);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x =
          exit_start_x_m_ +
          (exit_target_.target_x_m - exit_start_x_m_) * ratio;
      pose.pose.position.y =
          exit_start_y_m_ +
          (exit_target_.target_y_m - exit_start_y_m_) * ratio;
      pose.pose.orientation = camrod_control::quaternionFromYaw(
          step == step_count ? exit_target_.target_yaw_rad
                             : exit_heading_yaw_rad_);
      path.poses.push_back(pose);
    }
    exit_path_publisher_->publish(path);
  }

  double selectParkingBodyYaw() const {
    const double configured_yaw = configuredParkingBodyYaw();
    if (!automatically_select_reverse_approach_yaw_ ||
        !last_vehicle_pose_.has_value()) {
      return configured_yaw;
    }
    const double flipped_yaw =
        camrod_control::normalizeAngle(configured_yaw + M_PI);
    return stationDistanceAlongReverseAxis(flipped_yaw) >
                   stationDistanceAlongReverseAxis(configured_yaw) + 1e-3
               ? flipped_yaw
               : configured_yaw;
  }

  double stationDistanceAlongReverseAxis(const double body_yaw_rad) const {
    if (!last_vehicle_pose_.has_value()) {
      return 0.0;
    }
    return camrod_control::signedDistanceAlongAxis(
        last_vehicle_pose_->pose.position.x,
        last_vehicle_pose_->pose.position.y, station_pose_.x_m,
        station_pose_.y_m, camrod_control::reverseAxisYawForBody(body_yaw_rad));
  }

  bool publishAlignmentCommand() {
    if (!vehiclePoseIsFresh()) {
      setError("pose timeout during drop-zone yaw alignment");
      return false;
    }
    const double current_yaw = camrod_control::yawFromPose(*last_vehicle_pose_);
    const double yaw_error =
        camrod_control::normalizeAngle(target_body_yaw_rad_ - current_yaw);
    const bool settled = yaw_alignment_settling_.observe(
        yaw_error, current_yaw, last_vehicle_pose_time_.seconds());
    if (yaw_alignment_settling_.withinTolerance()) {
      publishZero();
      return settled;
    }
    avg_msgs::msg::AvgTwist command;
    command.angular.z = camrod_control::clamp(
        yaw_proportional_gain_ * yaw_error, -maximum_angular_speed_radps_,
        maximum_angular_speed_radps_);
    command_publisher_->publish(command);
    return false;
  }

  double distanceFromExitStart() const {
    if (!last_vehicle_pose_.has_value()) {
      return 0.0;
    }
    return std::hypot(last_vehicle_pose_->pose.position.x - exit_start_x_m_,
                      last_vehicle_pose_->pose.position.y - exit_start_y_m_);
  }

  double exitTargetError() const {
    if (!last_vehicle_pose_.has_value() || !exit_target_.valid) {
      return 0.0;
    }
    return camrod_control::dropZoneExitTargetError(
        exit_target_, last_vehicle_pose_->pose.position.x,
        last_vehicle_pose_->pose.position.y);
  }

  void publishExitCommand() {
    if (!vehiclePoseIsFresh()) {
      publishExitComplete(false);
      setError("pose timeout during drop-zone exit");
      return;
    }
    if (!exit_target_.valid) {
      publishExitComplete(false);
      setError("captured drop-zone exit target unavailable");
      return;
    }
    const double current_x_m = last_vehicle_pose_->pose.position.x;
    const double current_y_m = last_vehicle_pose_->pose.position.y;
    if (camrod_control::dropZoneExitTranslationComplete(
            exit_target_, current_x_m, current_y_m,
            exit_position_tolerance_m_)) {
      publishZero();
      setPhase(
          DropZoneManeuverPhase::kAlignExitYaw,
          "captured exit target reached; aligning captured lanelet yaw");
      return;
    }
    if ((now() - phase_start_time_).seconds() >= exit_timeout_s_) {
      publishExitComplete(false);
      setError("drop-zone exit timeout");
      return;
    }
    // HH_260819 - Correct the small map/tag lateral offset while translating
    // instead of holding the parking heading for a blind fixed distance.
    const double heading_error = camrod_control::dropZoneExitBearingError(
        exit_target_, current_x_m, current_y_m,
        camrod_control::yawFromPose(*last_vehicle_pose_));
    avg_msgs::msg::AvgTwist command;
    command.linear.x = exit_speed_mps_;
    command.angular.z = camrod_control::clamp(
        yaw_proportional_gain_ * heading_error, -maximum_angular_speed_radps_,
        maximum_angular_speed_radps_);
    command_publisher_->publish(command);
  }

  void onTimer() {
    if (phase_ == DropZoneManeuverPhase::kExitStraight) {
      publishExitCommand();
    } else if (phase_ == DropZoneManeuverPhase::kAlignExitYaw) {
      if (publishAlignmentCommand()) {
        road_handoff_ready_ = true;
        publishExitComplete(true);
        setPhase(DropZoneManeuverPhase::kIdle, "drop-zone exit aligned");
      }
    } else if (phase_ == DropZoneManeuverPhase::kAlignParkingYaw) {
      if (publishAlignmentCommand()) {
        requestParkingStart();
        setPhase(DropZoneManeuverPhase::kIdle,
                 "parking yaw aligned; reverse parking requested");
      }
    }
    publishStatus(false);
  }

  void publishStatus(const bool force) {
    const rclcpp::Time current_time = now();
    if (!force && last_status_time_.nanoseconds() > 0 &&
        (current_time - last_status_time_).seconds() < 1.0 / status_rate_hz_) {
      return;
    }
    // HH_260721 - Straight exit and yaw alignment are healthy operating
    // progress.
    const uint8_t module_level = phase_ == DropZoneManeuverPhase::kError
                                     ? avg_msgs::msg::ModuleState::ERROR
                                     : avg_msgs::msg::ModuleState::OK;
    const std::string operating_state =
        phase_ == DropZoneManeuverPhase::kIdle && road_handoff_ready_
            ? "ROAD_HANDOFF_READY"
            : phaseName(phase_);
    const std::string message =
        "phase=" + operating_state +
        " exit_target_source=" +
        camrod_control::dropZoneExitTargetSourceName(exit_target_.source) +
        " exit_target_error_m=" + fixed(exitTargetError()) +
        " target_yaw_deg=" + fixed(target_body_yaw_rad_ * 180.0 / M_PI, 2) +
        " settle_s=" + fixed(yaw_alignment_settling_.stableDurationS(), 2) +
        " yaw_rate_degps=" + fixed(yaw_alignment_settling_.yawRateDegps(), 2);
    status_publisher_->publish(camrod_control::makeModuleState(
        *this, "control", module_level, message, operating_state));
    if (!force) {
      // HH_260819 - Reconstruct in-progress departure after a UI restart.
      // `_on_service_state` treats identical heartbeats as edge-idempotent.
      publishServiceState("heartbeat");
    }
    const uint8_t diagnostic_level =
        module_level == avg_msgs::msg::ModuleState::ERROR
            ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
        : module_level == avg_msgs::msg::ModuleState::WARN
            ? diagnostic_msgs::msg::DiagnosticStatus::WARN
            : diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics_publisher_->publish(camrod_control::makeDiagnostics(
        *this, "control/drop_zone_maneuver_controller", "control",
        diagnostic_level, message,
        {
            {"phase", operating_state},
            {"command_topic", command_topic_},
            {"exit_travel_distance_m", fixed(distanceFromExitStart())},
            {"exit_target_source",
             camrod_control::dropZoneExitTargetSourceName(exit_target_.source)},
            {"exit_target_x_m", fixed(exit_target_.target_x_m)},
            {"exit_target_y_m", fixed(exit_target_.target_y_m)},
            {"exit_target_initial_distance_m",
             fixed(exit_target_.initial_distance_m)},
            {"exit_target_initial_forward_m",
             fixed(exit_target_.initial_forward_m)},
            {"exit_target_initial_lateral_m",
             fixed(exit_target_.initial_lateral_m)},
            {"exit_target_error_m", fixed(exitTargetError())},
            {"station_x_m", fixed(station_pose_.x_m)},
            {"station_y_m", fixed(station_pose_.y_m)},
            {"station_yaw_deg", fixed(station_pose_.yaw_rad * 180.0 / M_PI)},
        }));
    last_status_time_ = current_time;
  }

  std::string command_topic_;
  std::string vehicle_pose_topic_;
  std::string platform_status_topic_;
  std::string planning_state_topic_;
  std::string lanelet_pose_topic_;
  std::string drop_zone_goal_topic_;
  std::string operation_topic_;
  std::string exit_complete_topic_;
  std::string status_topic_;
  std::string exit_path_topic_;
  std::string parking_operation_topic_;
  std::string diagnostics_topic_;
  std::string service_state_topic_;
  std::string drop_zones_yaml_;
  std::string drop_zone_id_;
  bool use_drop_zone_pose_as_station_{true};
  bool enable_auto_alignment_{true};
  int return_scenario_id_{avg_msgs::msg::PlanningScenario::RETURN_TO_DROP_ZONE};
  std::string return_mission_key_{"drop_zone"};
  bool station_yaw_represents_reverse_axis_{true};
  bool automatically_select_reverse_approach_yaw_{false};
  double yaw_tolerance_deg_{5.0};
  double yaw_proportional_gain_{1.2};
  double maximum_angular_speed_radps_{0.35};
  double yaw_settle_hold_s_{1.0};
  double yaw_settle_max_rate_degps_{3.0};
  double exit_distance_m_{1.2};
  bool require_lanelet_exit_target_{true};
  bool allow_fixed_distance_fallback_{false};
  double exit_target_min_distance_m_{0.3};
  double exit_target_max_distance_m_{8.0};
  double exit_target_min_forward_m_{0.1};
  double exit_target_max_lateral_m_{1.0};
  double exit_position_tolerance_m_{0.20};
  double exit_speed_mps_{0.16};
  double exit_timeout_s_{30.0};
  double pose_timeout_s_{2.0};
  bool allow_unstamped_pose_fallback_{false};
  double control_rate_hz_{10.0};
  double idle_rate_hz_{1.0};
  double status_rate_hz_{1.0};

  camrod_control::DropZoneStationPose station_pose_;
  DropZoneManeuverPhase phase_{DropZoneManeuverPhase::kIdle};
  std::optional<avg_msgs::msg::AvgPoseStamped> last_vehicle_pose_;
  std::optional<avg_msgs::msg::AvgPoseStamped> last_lanelet_pose_;
  rclcpp::Time last_vehicle_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lanelet_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time phase_start_time_{0, 0, RCL_ROS_TIME};
  double exit_start_x_m_{0.0};
  double exit_start_y_m_{0.0};
  double exit_heading_yaw_rad_{0.0};
  double target_body_yaw_rad_{0.0};
  camrod_control::DropZoneExitTargetPolicyConfig exit_target_policy_config_;
  camrod_control::DropZoneExitTargetSelection exit_target_;
  std::string exit_target_frame_id_{"map"};
  camrod_control::YawAlignmentSettling yaw_alignment_settling_;
  bool is_charging_{false};
  bool exit_started_while_charging_{false};
  bool road_handoff_ready_{false};
  std::string last_auto_alignment_key_;
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<avg_msgs::msg::AvgTwist>::SharedPtr command_publisher_;
  rclcpp::Publisher<avg_msgs::msg::ModuleState>::SharedPtr status_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::Publisher<avg_msgs::msg::AvgServiceState>::SharedPtr
      service_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr exit_path_publisher_;
  rclcpp::Publisher<avg_msgs::msg::AvgBool>::SharedPtr exit_complete_publisher_;
  rclcpp::Publisher<avg_msgs::msg::MotionOperation>::SharedPtr
      parking_operation_publisher_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr
      vehicle_pose_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgPlatformStatus>::SharedPtr
      platform_status_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr
      lanelet_pose_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr
      drop_zone_goal_subscription_;
  rclcpp::Subscription<avg_msgs::msg::PlanningState>::SharedPtr
      planning_state_subscription_;
  rclcpp::Subscription<avg_msgs::msg::MotionOperation>::SharedPtr
      operation_subscription_;
  rclcpp::Service<avg_msgs::srv::RequestMotionOperation>::SharedPtr
      operation_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DropZoneManeuverControllerNode>());
  rclcpp::shutdown();
  return 0;
}
