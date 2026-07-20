// HH_260721 - Implement final reverse parking as a native C++ control node.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "avg_msgs/msg/avg_amr_service_state.hpp"
#include "avg_msgs/msg/avg_platform_status.hpp"
#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "avg_msgs/msg/avg_twist.hpp"
#include "avg_msgs/msg/module_state.hpp"
#include "avg_msgs/msg/motion_operation.hpp"
#include "avg_msgs/srv/request_motion_operation.hpp"
#include "camrod_control/control_support.hpp"
#include "camrod_control/parking_geometry.hpp"
#include "camrod_control/station_pose.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

enum class ReverseParkingPhase
{
  kIdle,
  kReverseApproach,
  kWaitForCharging,
  kParked,
  kError,
};

std::string phaseName(const ReverseParkingPhase phase)
{
  switch (phase) {
    case ReverseParkingPhase::kIdle:
      return "IDLE";
    case ReverseParkingPhase::kReverseApproach:
      return "REVERSE_APPROACH";
    case ReverseParkingPhase::kWaitForCharging:
      return "WAIT_FOR_CHARGING";
    case ReverseParkingPhase::kParked:
      return "PARKED";
    case ReverseParkingPhase::kError:
      return "ERROR";
  }
  return "ERROR";
}

std::string fixed(const double value, const int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

}  // namespace

class ReverseParkingControllerNode : public rclcpp::Node
{
public:
  ReverseParkingControllerNode()
  : Node("reverse_parking_controller")
  {
    // HH_260721 - Preserve all v2.0.3 interfaces while enforcing charging completion.
    command_topic_ = declare_parameter<std::string>("command_topic", "/control/cmd_vel_raw");
    vehicle_pose_topic_ =
      declare_parameter<std::string>("vehicle_pose_topic", "/localization/pose");
    platform_status_topic_ = declare_parameter<std::string>(
      "platform_status_topic",
      "/platform/status");
    operation_topic_ = declare_parameter<std::string>("operation_topic", "/parking/operation");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/parking/reverse_parking_controller/status");
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/parking/reverse_parking_controller/path_ros");
    drop_zone_goal_topic_ = declare_parameter<std::string>(
      "drop_zone_goal_topic", "/planning/drop_zone_goal_raw");
    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "/system/diagnostics");
    amr_service_state_topic_ = declare_parameter<std::string>(
      "amr_service_state_topic", "/AMR_service_state");

    drop_zones_yaml_ = declare_parameter<std::string>("drop_zones_yaml", "");
    drop_zone_id_ = declare_parameter<std::string>("drop_zone_id", "drop_zone");
    use_drop_zone_pose_as_station_ = declare_parameter<bool>("use_drop_zone_pose_as_station", true);
    station_pose_ = camrod_control::loadDropZoneStationPose(
      get_logger(), drop_zones_yaml_, drop_zone_id_, use_drop_zone_pose_as_station_,
      camrod_control::StationPose{
      declare_parameter<double>("station_x_m", 0.0),
      declare_parameter<double>("station_y_m", 0.0),
      declare_parameter<double>("station_yaw_deg", 0.0) * M_PI / 180.0});

    station_yaw_represents_reverse_axis_ = declare_parameter<bool>(
      "station_yaw_represents_reverse_axis", true);
    automatically_select_reverse_approach_yaw_ = declare_parameter<bool>(
      "automatically_select_reverse_approach_yaw", false);
    start_yaw_tolerance_deg_ = std::abs(declare_parameter<double>("start_yaw_tolerance_deg", 7.5));
    reverse_speed_mps_ = std::abs(declare_parameter<double>("reverse_speed_mps", 0.16));
    heading_proportional_gain_ = declare_parameter<double>("heading_proportional_gain", 0.8);
    lateral_proportional_gain_ = declare_parameter<double>("lateral_proportional_gain", -0.25);
    maximum_angular_speed_radps_ = std::abs(
      declare_parameter<double>("maximum_angular_speed_radps", 0.22));
    maximum_reverse_distance_m_ = std::abs(
      declare_parameter<double>("maximum_reverse_distance_m", 1.5));
    reverse_timeout_s_ = std::abs(declare_parameter<double>("reverse_timeout_s", 30.0));
    require_station_on_reverse_axis_ = declare_parameter<bool>(
      "require_station_on_reverse_axis", true);
    station_axis_tolerance_m_ = std::abs(
      declare_parameter<double>("station_axis_tolerance_m", 0.25));
    stop_when_charging_ = declare_parameter<bool>("stop_when_charging", true);
    complete_without_charging_ = declare_parameter<bool>("complete_without_charging", false);
    charging_wait_timeout_s_ = std::abs(
      declare_parameter<double>("charging_wait_timeout_s", 20.0));
    pose_timeout_s_ = std::abs(declare_parameter<double>("pose_timeout_s", 2.0));
    control_rate_hz_ = std::max(0.1, declare_parameter<double>("control_rate_hz", 10.0));
    idle_rate_hz_ = std::max(0.1, declare_parameter<double>("idle_rate_hz", 1.0));
    status_rate_hz_ = std::max(0.1, declare_parameter<double>("status_rate_hz", 1.0));

    command_publisher_ = create_publisher<avg_msgs::msg::AvgTwist>(command_topic_, 10);
    status_publisher_ = create_publisher<avg_msgs::msg::ModuleState>(status_topic_, 10);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10);
    service_state_publisher_ = create_publisher<avg_msgs::msg::AvgAmrServiceState>(
      amr_service_state_topic_, 10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

    vehicle_pose_subscription_ = create_subscription<avg_msgs::msg::AvgPoseStamped>(
      vehicle_pose_topic_, 10,
      [this](const avg_msgs::msg::AvgPoseStamped::SharedPtr message) {
        last_vehicle_pose_ = *message;
        last_vehicle_pose_time_ = now();
      });
    platform_status_subscription_ = create_subscription<avg_msgs::msg::AvgPlatformStatus>(
      platform_status_topic_, 10,
      [this](const avg_msgs::msg::AvgPlatformStatus::SharedPtr message) {
        is_charging_ = message->is_charging;
      });
    operation_subscription_ = create_subscription<avg_msgs::msg::MotionOperation>(
      operation_topic_, 10,
      [this](const avg_msgs::msg::MotionOperation::SharedPtr message) {
        applyOperation(message->operation, message->source.empty() ? "topic" : message->source);
      });
    drop_zone_goal_subscription_ = create_subscription<avg_msgs::msg::AvgPoseStamped>(
      drop_zone_goal_topic_, 10,
      [this](const avg_msgs::msg::AvgPoseStamped::SharedPtr message) {
        onDropZoneGoal(*message);
      });
    operation_service_ = create_service<avg_msgs::srv::RequestMotionOperation>(
      "/parking/reverse_parking_controller/request_operation",
      [this](
        const std::shared_ptr<avg_msgs::srv::RequestMotionOperation::Request> request,
        std::shared_ptr<avg_msgs::srv::RequestMotionOperation::Response> response)
      {
        const auto result = applyOperation(
          request->operation, request->source.empty() ? "service" : request->source);
        response->accepted = result.first;
        response->message = result.second;
      });

    target_body_yaw_rad_ = configuredParkingBodyYaw();
    phase_start_time_ = now();
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&ReverseParkingControllerNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "reverse_parking_controller ready: station=(%.2f,%.2f,%.1fdeg) operation=%s command=%s",
      station_pose_.x_m, station_pose_.y_m, station_pose_.yaw_rad * 180.0 / M_PI,
      operation_topic_.c_str(), command_topic_.c_str());
  }

private:
  double configuredParkingBodyYaw() const
  {
    return station_yaw_represents_reverse_axis_ ?
           camrod_control::bodyYawForReverseAxis(station_pose_.yaw_rad) :
           camrod_control::normalizeAngle(station_pose_.yaw_rad);
  }

  void onDropZoneGoal(const avg_msgs::msg::AvgPoseStamped & message)
  {
    if (phase_ == ReverseParkingPhase::kReverseApproach ||
      phase_ == ReverseParkingPhase::kWaitForCharging)
    {
      RCLCPP_WARN(get_logger(), "ignored drop-zone goal update while reverse parking is active");
      return;
    }
    station_pose_ = camrod_control::StationPose{
      message.pose.position.x,
      message.pose.position.y,
      camrod_control::yawFromPose(message)};
    target_body_yaw_rad_ = configuredParkingBodyYaw();
  }

  std::pair<bool, std::string> applyOperation(const uint8_t operation, const std::string & source)
  {
    if (operation == avg_msgs::msg::MotionOperation::START) {
      return startReverseParking(source);
    }
    if (operation == avg_msgs::msg::MotionOperation::CANCEL) {
      publishZero();
      setPhase(ReverseParkingPhase::kIdle, "cancel=" + source);
      return {true, "reverse parking cancelled"};
    }
    return {false, "unsupported reverse parking operation=" + std::to_string(operation)};
  }

  bool vehiclePoseIsFresh() const
  {
    return last_vehicle_pose_.has_value() &&
           (now() - last_vehicle_pose_time_).seconds() <= pose_timeout_s_;
  }

  std::pair<bool, std::string> startReverseParking(const std::string & source)
  {
    if (phase_ == ReverseParkingPhase::kReverseApproach ||
      phase_ == ReverseParkingPhase::kWaitForCharging)
    {
      return {false, "reverse parking already active"};
    }
    if (!vehiclePoseIsFresh()) {
      setError("fresh pose unavailable");
      return {false, "fresh pose unavailable"};
    }
    target_body_yaw_rad_ = selectParkingBodyYaw();
    const double current_yaw = camrod_control::yawFromPose(*last_vehicle_pose_);
    const double yaw_error_deg = std::abs(
      camrod_control::normalizeAngle(target_body_yaw_rad_ - current_yaw) * 180.0 / M_PI);
    if (yaw_error_deg > start_yaw_tolerance_deg_) {
      setError(
        "parking yaw is not aligned: error=" + fixed(yaw_error_deg, 1) +
        "deg tolerance=" + fixed(start_yaw_tolerance_deg_, 1) + "deg");
      return {false, "parking yaw alignment required"};
    }
    reverse_start_x_m_ = last_vehicle_pose_->pose.position.x;
    reverse_start_y_m_ = last_vehicle_pose_->pose.position.y;
    setPhase(ReverseParkingPhase::kReverseApproach, "start=" + source);
    publishPath();
    return {true, "reverse parking started"};
  }

  void setPhase(const ReverseParkingPhase phase, const std::string & detail)
  {
    phase_ = phase;
    phase_start_time_ = now();
    RCLCPP_INFO(
      get_logger(), "reverse_parking_controller %s: %s",
      phaseName(phase_).c_str(), detail.c_str());
    publishServiceState(detail);
    publishStatus(true);
  }

  void setError(const std::string & detail)
  {
    publishZero();
    setPhase(ReverseParkingPhase::kError, detail);
  }

  void publishServiceState(const std::string & detail)
  {
    avg_msgs::msg::AvgAmrServiceState message;
    if (phase_ == ReverseParkingPhase::kReverseApproach ||
      phase_ == ReverseParkingPhase::kWaitForCharging)
    {
      message.state = avg_msgs::msg::AvgAmrServiceState::DROP_ZONE_PARKING;
    } else if (phase_ == ReverseParkingPhase::kParked) {
      message.state = avg_msgs::msg::AvgAmrServiceState::DROP_ZONE_WAIT;
    } else {
      return;
    }
    message.description =
      "reverse_parking_controller:" + phaseName(phase_) + ":" + detail;
    service_state_publisher_->publish(message);
  }

  void publishZero() const
  {
    command_publisher_->publish(avg_msgs::msg::AvgTwist());
  }

  double selectParkingBodyYaw() const
  {
    const double configured_yaw = configuredParkingBodyYaw();
    if (!automatically_select_reverse_approach_yaw_ || !last_vehicle_pose_.has_value()) {
      return configured_yaw;
    }
    const double flipped_yaw = camrod_control::normalizeAngle(configured_yaw + M_PI);
    return stationDistanceAlongReverseAxis(flipped_yaw) >
           stationDistanceAlongReverseAxis(configured_yaw) + 1e-3 ?
           flipped_yaw : configured_yaw;
  }

  double stationDistanceAlongReverseAxis() const
  {
    return stationDistanceAlongReverseAxis(target_body_yaw_rad_);
  }

  double stationDistanceAlongReverseAxis(const double body_yaw_rad) const
  {
    if (!last_vehicle_pose_.has_value()) {
      return 0.0;
    }
    return camrod_control::signedDistanceAlongAxis(
      last_vehicle_pose_->pose.position.x,
      last_vehicle_pose_->pose.position.y,
      station_pose_.x_m,
      station_pose_.y_m,
      camrod_control::reverseAxisYawForBody(body_yaw_rad));
  }

  double stationLateralError() const
  {
    if (!last_vehicle_pose_.has_value()) {
      return 0.0;
    }
    const double dx = last_vehicle_pose_->pose.position.x - station_pose_.x_m;
    const double dy = last_vehicle_pose_->pose.position.y - station_pose_.y_m;
    return -std::sin(station_pose_.yaw_rad) * dx + std::cos(station_pose_.yaw_rad) * dy;
  }

  double distanceReversed() const
  {
    if (!last_vehicle_pose_.has_value()) {
      return 0.0;
    }
    return std::hypot(
      last_vehicle_pose_->pose.position.x - reverse_start_x_m_,
      last_vehicle_pose_->pose.position.y - reverse_start_y_m_);
  }

  void publishPath() const
  {
    if (!last_vehicle_pose_.has_value()) {
      return;
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = last_vehicle_pose_->header.frame_id.empty() ?
      "map" : last_vehicle_pose_->header.frame_id;
    path.header.stamp = now();
    const double reverse_axis_yaw = camrod_control::reverseAxisYawForBody(target_body_yaw_rad_);
    const double station_distance = std::max(0.0, stationDistanceAlongReverseAxis());
    const double distance = std::min(
      station_distance > 0.05 ? station_distance : maximum_reverse_distance_m_,
      maximum_reverse_distance_m_);
    const int step_count = std::max(1, static_cast<int>(std::max(distance, 0.05) / 0.25));
    for (int step = 0; step <= step_count; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(step_count);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x =
        last_vehicle_pose_->pose.position.x + std::cos(reverse_axis_yaw) * distance * ratio;
      pose.pose.position.y =
        last_vehicle_pose_->pose.position.y + std::sin(reverse_axis_yaw) * distance * ratio;
      pose.pose.orientation = camrod_control::quaternionFromYaw(target_body_yaw_rad_);
      path.poses.push_back(pose);
    }
    path_publisher_->publish(path);
  }

  void finishTravel(const std::string & detail)
  {
    publishZero();
    if (complete_without_charging_) {
      setPhase(ReverseParkingPhase::kParked, detail);
      return;
    }
    // HH_260721 - Hold zero at the contact pose until CAN confirms charging.
    setPhase(ReverseParkingPhase::kWaitForCharging, detail + "; waiting for charging");
  }

  void publishReverseCommand()
  {
    if (stop_when_charging_ && is_charging_) {
      publishZero();
      setPhase(ReverseParkingPhase::kParked, "charging detected");
      return;
    }
    if (!vehiclePoseIsFresh()) {
      setError("pose timeout during reverse parking");
      return;
    }
    const double reversed = distanceReversed();
    if (reversed >= maximum_reverse_distance_m_) {
      finishTravel("reverse distance limit reached");
      return;
    }
    if ((now() - phase_start_time_).seconds() >= reverse_timeout_s_) {
      setError("reverse parking timeout");
      return;
    }
    const double station_axis_distance = stationDistanceAlongReverseAxis();
    if (require_station_on_reverse_axis_ && reversed <= 0.05 &&
      station_axis_distance < -station_axis_tolerance_m_)
    {
      setError("station is behind the reverse axis; check drop-zone yaw");
      return;
    }
    if (reversed > 0.05 && station_axis_distance <= station_axis_tolerance_m_) {
      finishTravel("station reverse axis reached");
      return;
    }

    const double heading_error = camrod_control::normalizeAngle(
      target_body_yaw_rad_ - camrod_control::yawFromPose(*last_vehicle_pose_));
    avg_msgs::msg::AvgTwist command;
    command.linear.x = -reverse_speed_mps_;
    command.angular.z = camrod_control::clamp(
      heading_proportional_gain_ * heading_error +
      lateral_proportional_gain_ * stationLateralError(),
      -maximum_angular_speed_radps_, maximum_angular_speed_radps_);
    command_publisher_->publish(command);
  }

  void waitForCharging()
  {
    publishZero();
    if (is_charging_) {
      setPhase(ReverseParkingPhase::kParked, "charging detected");
    } else if ((now() - phase_start_time_).seconds() >= charging_wait_timeout_s_) {
      setError("charging was not detected before timeout");
    }
  }

  void onTimer()
  {
    if (phase_ == ReverseParkingPhase::kReverseApproach) {
      publishReverseCommand();
    } else if (phase_ == ReverseParkingPhase::kWaitForCharging) {
      waitForCharging();
    }
    publishStatus(false);
  }

  void publishStatus(const bool force)
  {
    const rclcpp::Time current_time = now();
    if (!force && last_status_time_.nanoseconds() > 0 &&
      (current_time - last_status_time_).seconds() < 1.0 / status_rate_hz_)
    {
      return;
    }
    uint8_t module_level = avg_msgs::msg::ModuleState::WARN;
    if (phase_ == ReverseParkingPhase::kError) {
      module_level = avg_msgs::msg::ModuleState::ERROR;
    } else if (phase_ == ReverseParkingPhase::kIdle || phase_ == ReverseParkingPhase::kParked) {
      module_level = avg_msgs::msg::ModuleState::OK;
    }
    const std::string message =
      "phase=" + phaseName(phase_) + " charging=" + (is_charging_ ? "True" : "False");
    status_publisher_->publish(
      camrod_control::makeModuleState(*this, "parking", module_level, message));
    const uint8_t diagnostic_level = module_level == avg_msgs::msg::ModuleState::ERROR ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      module_level == avg_msgs::msg::ModuleState::WARN ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics_publisher_->publish(
      camrod_control::makeDiagnostics(
        *this, "parking/reverse_parking_controller", "parking", diagnostic_level, message,
    {
      {"phase", phaseName(phase_)},
      {"command_topic", command_topic_},
      {"reverse_distance_m", fixed(distanceReversed())},
      {"station_axis_distance_m", fixed(stationDistanceAlongReverseAxis())},
      {"target_body_yaw_deg", fixed(target_body_yaw_rad_ * 180.0 / M_PI)},
    }));
    last_status_time_ = current_time;
  }

  std::string command_topic_;
  std::string vehicle_pose_topic_;
  std::string platform_status_topic_;
  std::string operation_topic_;
  std::string status_topic_;
  std::string path_topic_;
  std::string drop_zone_goal_topic_;
  std::string diagnostics_topic_;
  std::string amr_service_state_topic_;
  std::string drop_zones_yaml_;
  std::string drop_zone_id_;
  bool use_drop_zone_pose_as_station_{true};
  bool station_yaw_represents_reverse_axis_{true};
  bool automatically_select_reverse_approach_yaw_{false};
  double start_yaw_tolerance_deg_{7.5};
  double reverse_speed_mps_{0.16};
  double heading_proportional_gain_{0.8};
  double lateral_proportional_gain_{-0.25};
  double maximum_angular_speed_radps_{0.22};
  double maximum_reverse_distance_m_{1.5};
  double reverse_timeout_s_{30.0};
  bool require_station_on_reverse_axis_{true};
  double station_axis_tolerance_m_{0.25};
  bool stop_when_charging_{true};
  bool complete_without_charging_{false};
  double charging_wait_timeout_s_{20.0};
  double pose_timeout_s_{2.0};
  double control_rate_hz_{10.0};
  double idle_rate_hz_{1.0};
  double status_rate_hz_{1.0};

  camrod_control::StationPose station_pose_;
  ReverseParkingPhase phase_{ReverseParkingPhase::kIdle};
  std::optional<avg_msgs::msg::AvgPoseStamped> last_vehicle_pose_;
  rclcpp::Time last_vehicle_pose_time_{0, 0, RCL_ROS_TIME};
  bool is_charging_{false};
  double target_body_yaw_rad_{0.0};
  double reverse_start_x_m_{0.0};
  double reverse_start_y_m_{0.0};
  rclcpp::Time phase_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<avg_msgs::msg::AvgTwist>::SharedPtr command_publisher_;
  rclcpp::Publisher<avg_msgs::msg::ModuleState>::SharedPtr status_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<avg_msgs::msg::AvgAmrServiceState>::SharedPtr service_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr vehicle_pose_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgPlatformStatus>::SharedPtr platform_status_subscription_;
  rclcpp::Subscription<avg_msgs::msg::MotionOperation>::SharedPtr operation_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr drop_zone_goal_subscription_;
  rclcpp::Service<avg_msgs::srv::RequestMotionOperation>::SharedPtr operation_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReverseParkingControllerNode>());
  rclcpp::shutdown();
  return 0;
}
