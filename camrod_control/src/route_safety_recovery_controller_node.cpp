// HH_260803 / TODOLIST 12 - Execute only the safety gate's bounded and
// unambiguous reverse/crab recommendation; Nav2 retains mission ownership.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "avg_msgs/msg/avg_twist.hpp"
#include "avg_msgs/msg/module_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camrod_control
{

class RouteSafetyRecoveryControllerNode final : public rclcpp::Node
{
public:
  RouteSafetyRecoveryControllerNode()
  : Node("route_safety_recovery_controller")
  {
    enabled_ = declare_parameter<bool>("enabled", true);
    gate_status_topic_ = declare_parameter<std::string>(
      "gate_status_topic", "/control/cmd_vel_safety_gate/status");
    candidate_topic_ = declare_parameter<std::string>(
      "candidate_topic", "/control/route_safety_recovery/candidate");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/localization/pose");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/control/cmd_vel_raw");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/control/route_safety_recovery_controller/status");
    maximum_speed_mps_ = std::clamp(
      declare_parameter<double>("maximum_speed_mps", 0.10), 0.02, 0.10);
    maximum_distance_m_ = std::clamp(
      declare_parameter<double>("maximum_distance_m", 0.40), 0.05, 1.0);
    maximum_duration_s_ = std::clamp(
      declare_parameter<double>("maximum_duration_s", 10.0), 0.5, 15.0);
    pose_timeout_s_ = std::clamp(
      declare_parameter<double>("pose_timeout_s", 0.5), 0.05, 2.0);
    candidate_timeout_s_ = std::clamp(
      declare_parameter<double>("candidate_timeout_s", 0.75), 0.1, 2.0);
    status_timeout_s_ = std::clamp(
      declare_parameter<double>("status_timeout_s", 1.0), 0.5, 3.0);
    control_rate_hz_ = std::clamp(
      declare_parameter<double>("control_rate_hz", 10.0), 2.0, 30.0);

    rclcpp::QoS state_qos(1);
    state_qos.reliable().transient_local();
    command_publisher_ = create_publisher<avg_msgs::msg::AvgTwist>(command_topic_, 10);
    status_publisher_ = create_publisher<avg_msgs::msg::ModuleState>(status_topic_, state_qos);
    gate_status_subscription_ = create_subscription<avg_msgs::msg::ModuleState>(
      gate_status_topic_, state_qos,
      std::bind(&RouteSafetyRecoveryControllerNode::onGateStatus, this, std::placeholders::_1));
    candidate_subscription_ = create_subscription<avg_msgs::msg::AvgTwist>(
      candidate_topic_, state_qos,
      std::bind(&RouteSafetyRecoveryControllerNode::onCandidate, this, std::placeholders::_1));
    pose_subscription_ = create_subscription<avg_msgs::msg::AvgPoseStamped>(
      pose_topic_, 10,
      std::bind(&RouteSafetyRecoveryControllerNode::onPose, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_hz_),
      std::bind(&RouteSafetyRecoveryControllerNode::tick, this));
    publishStatus("IDLE", "waiting_for_route_safety_hold");
  }

private:
  void onGateStatus(const avg_msgs::msg::ModuleState::SharedPtr message)
  {
    const bool hold = message && message->operating_state == "ROUTE_SAFETY_HOLD";
    last_gate_status_time_ = now();
    if (hold && !hold_active_) {
      hold_start_time_ = now();
      start_pose_ = poseFresh() ? latest_pose_ : std::nullopt;
      limit_reached_ = false;
      RCLCPP_WARN(get_logger(), "bounded route recovery armed");
    } else if (!hold && hold_active_) {
      publishZero();
      start_pose_.reset();
      limit_reached_ = false;
      RCLCPP_INFO(get_logger(), "bounded route recovery released to Nav2");
    }
    hold_active_ = hold;
  }

  void onCandidate(const avg_msgs::msg::AvgTwist::SharedPtr message)
  {
    if (!message) {
      return;
    }
    candidate_ = *message;
    last_candidate_time_ = now();
  }

  void onPose(const avg_msgs::msg::AvgPoseStamped::SharedPtr message)
  {
    if (!message) {
      return;
    }
    latest_pose_ = *message;
    last_pose_time_ = now();
    if (hold_active_ && !start_pose_.has_value()) {
      start_pose_ = latest_pose_;
    }
  }

  bool poseFresh()
  {
    return latest_pose_.has_value() &&
           (now() - last_pose_time_).seconds() <= pose_timeout_s_;
  }

  double displacement()
  {
    if (!latest_pose_.has_value() || !start_pose_.has_value()) {
      return 0.0;
    }
    return std::hypot(
      latest_pose_->pose.position.x - start_pose_->pose.position.x,
      latest_pose_->pose.position.y - start_pose_->pose.position.y);
  }

  void tick()
  {
    if (!enabled_ || !hold_active_) {
      publishStatus("IDLE", enabled_ ? "waiting_for_route_safety_hold" : "disabled");
      return;
    }
    const auto current_time = now();
    if ((current_time - last_gate_status_time_).seconds() > status_timeout_s_) {
      publishZero();
      publishStatus("SAFETY_HOLD", "gate_status_stale");
      return;
    }
    if (!poseFresh()) {
      publishZero();
      publishStatus("SAFETY_HOLD", "pose_missing_or_stale");
      return;
    }
    if ((current_time - last_candidate_time_).seconds() > candidate_timeout_s_) {
      publishZero();
      publishStatus("SAFETY_HOLD", "candidate_missing_or_stale");
      return;
    }
    const double elapsed = (current_time - hold_start_time_).seconds();
    const double distance = displacement();
    if (limit_reached_ || elapsed >= maximum_duration_s_ || distance >= maximum_distance_m_) {
      limit_reached_ = true;
      publishZero();
      publishStatus("LIMIT_HOLD", "bounded_recovery_limit_reached");
      return;
    }

    const double norm = std::hypot(candidate_.linear.x, candidate_.linear.y);
    if (norm < 0.02 || std::abs(candidate_.angular.z) > 1.0e-6) {
      publishZero();
      publishStatus("SAFETY_HOLD", "no_unambiguous_translation_candidate");
      return;
    }
    avg_msgs::msg::AvgTwist command = candidate_;
    const double scale = std::min(1.0, maximum_speed_mps_ / norm);
    command.linear.x *= scale;
    command.linear.y *= scale;
    command.angular.z = 0.0;
    command_publisher_->publish(command);
    const std::string motion = std::abs(command.linear.y) > std::abs(command.linear.x) ?
      (command.linear.y > 0.0 ? "CRAB_LEFT" : "CRAB_RIGHT") : "REVERSE";
    publishStatus(
      motion,
      "distance=" + std::to_string(distance) + " elapsed=" + std::to_string(elapsed));
  }

  void publishZero()
  {
    command_publisher_->publish(avg_msgs::msg::AvgTwist{});
  }

  void publishStatus(const std::string & operating_state, const std::string & detail)
  {
    avg_msgs::msg::ModuleState status;
    status.stamp = now();
    status.module_name = "route_safety_recovery_controller";
    status.level = avg_msgs::msg::ModuleState::OK;
    status.operating_state = operating_state;
    status.message = detail;
    status_publisher_->publish(status);
  }

  bool enabled_{true};
  bool hold_active_{false};
  bool limit_reached_{false};
  double maximum_speed_mps_{0.10};
  double maximum_distance_m_{0.40};
  double maximum_duration_s_{10.0};
  double pose_timeout_s_{0.5};
  double candidate_timeout_s_{0.75};
  double status_timeout_s_{1.0};
  double control_rate_hz_{10.0};
  std::string gate_status_topic_;
  std::string candidate_topic_;
  std::string pose_topic_;
  std::string command_topic_;
  std::string status_topic_;
  avg_msgs::msg::AvgTwist candidate_;
  std::optional<avg_msgs::msg::AvgPoseStamped> latest_pose_;
  std::optional<avg_msgs::msg::AvgPoseStamped> start_pose_;
  rclcpp::Time last_gate_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_candidate_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time hold_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<avg_msgs::msg::AvgTwist>::SharedPtr command_publisher_;
  rclcpp::Publisher<avg_msgs::msg::ModuleState>::SharedPtr status_publisher_;
  rclcpp::Subscription<avg_msgs::msg::ModuleState>::SharedPtr gate_status_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgTwist>::SharedPtr candidate_subscription_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr pose_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camrod_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod_control::RouteSafetyRecoveryControllerNode>());
  rclcpp::shutdown();
  return 0;
}
