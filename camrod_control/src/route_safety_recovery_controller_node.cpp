// HH_260805 / TODOLIST 12 - Execute only the safety gate's projected staged
// crab/reverse/yaw recommendation; Nav2 retains mission ownership.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "avg_msgs/msg/avg_twist.hpp"
#include "avg_msgs/msg/module_state.hpp"
#include "camrod_control/bounded_recovery_attempt_policy.hpp"
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
    maximum_angular_speed_radps_ = std::clamp(
      declare_parameter<double>("maximum_angular_speed_radps", 0.10), 0.02, 0.15);
    maximum_yaw_change_rad_ = std::clamp(
      declare_parameter<double>("maximum_yaw_change_deg", 12.0), 2.0, 20.0) *
      M_PI / 180.0;
    maximum_distance_m_ = std::clamp(
      declare_parameter<double>("maximum_distance_m", 0.40), 0.05, 1.0);
    maximum_duration_s_ = std::clamp(
      declare_parameter<double>("maximum_duration_s", 10.0), 0.5, 15.0);
    maximum_attempts_ = static_cast<int>(std::clamp<std::int64_t>(
        declare_parameter<std::int64_t>("maximum_attempts", 50), 1, 100));
    retry_pause_s_ = std::clamp(
      declare_parameter<double>("retry_pause_s", 0.5), 0.1, 5.0);
    maximum_total_distance_m_ = std::max(
      maximum_distance_m_, std::clamp(
        declare_parameter<double>("maximum_total_distance_m", 1.50), 0.40, 3.0));
    maximum_total_duration_s_ = std::max(
      maximum_duration_s_, std::clamp(
        declare_parameter<double>("maximum_total_duration_s", 90.0), 5.0, 300.0));
    behavior_config_.zero_hold_pauses_limits = declare_parameter<bool>(
      "zero_hold_pauses_limits", false);
    behavior_config_.allow_corrective_yaw_beyond_limit = declare_parameter<bool>(
      "allow_corrective_yaw_beyond_limit", false);
    attempt_limits_ = {
      maximum_duration_s_, maximum_distance_m_, maximum_attempts_,
      maximum_total_duration_s_, maximum_total_distance_m_};
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
      episode_start_time_ = now();
      attempt_start_time_ = episode_start_time_;
      start_pose_ = poseFresh() ? latest_pose_ : std::nullopt;
      start_yaw_ = start_pose_.has_value() ?
        std::optional<double>(poseYaw(*start_pose_)) : std::nullopt;
      cumulative_distance_m_ = 0.0;
      attempt_distance_offset_m_ = 0.0;
      attempt_yaw_offset_rad_ = 0.0;
      motion_segment_observed_ = false;
      zero_hold_clock_.reset();
      current_attempt_ = 1;
      retry_waiting_ = false;
      limit_reached_ = false;
      RCLCPP_WARN(
        get_logger(),
        "bounded route recovery armed: attempts=%d total=%.1fs/%.2fm",
        maximum_attempts_, maximum_total_duration_s_, maximum_total_distance_m_);
    } else if (!hold && hold_active_) {
      publishZero();
      start_pose_.reset();
      start_yaw_.reset();
      current_attempt_ = 0;
      cumulative_distance_m_ = 0.0;
      attempt_distance_offset_m_ = 0.0;
      attempt_yaw_offset_rad_ = 0.0;
      motion_segment_observed_ = false;
      zero_hold_clock_.reset();
      retry_waiting_ = false;
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
      start_yaw_ = poseYaw(*message);
    }
  }

  bool poseFresh()
  {
    return latest_pose_.has_value() &&
           (now() - last_pose_time_).seconds() <= pose_timeout_s_;
  }

  double displacement() const
  {
    if (!latest_pose_.has_value() || !start_pose_.has_value()) {
      return 0.0;
    }
    return std::hypot(
      latest_pose_->pose.position.x - start_pose_->pose.position.x,
      latest_pose_->pose.position.y - start_pose_->pose.position.y);
  }

  static double poseYaw(const avg_msgs::msg::AvgPoseStamped & pose)
  {
    const auto & q = pose.pose.orientation;
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  double signedYawDisplacement() const
  {
    if (!latest_pose_.has_value() || !start_yaw_.has_value()) {
      return std::atan2(
        std::sin(attempt_yaw_offset_rad_),
        std::cos(attempt_yaw_offset_rad_));
    }
    const double delta = poseYaw(*latest_pose_) - *start_yaw_;
    const double total = attempt_yaw_offset_rad_ +
      std::atan2(std::sin(delta), std::cos(delta));
    return std::atan2(std::sin(total), std::cos(total));
  }

  double yawDisplacement() const
  {
    return std::abs(signedYawDisplacement());
  }

  double attemptDistance() const
  {
    return attempt_distance_offset_m_ + displacement();
  }

  void beginZeroHold(const rclcpp::Time & current_time)
  {
    if (zero_hold_clock_.active()) {
      return;
    }
    if (motion_segment_observed_ && latest_pose_.has_value()) {
      attempt_distance_offset_m_ += displacement();
      if (start_yaw_.has_value()) {
        const double delta = poseYaw(*latest_pose_) - *start_yaw_;
        attempt_yaw_offset_rad_ = std::atan2(
          std::sin(attempt_yaw_offset_rad_ + delta),
          std::cos(attempt_yaw_offset_rad_ + delta));
      }
    }
    if (latest_pose_.has_value()) {
      start_pose_ = latest_pose_;
      start_yaw_ = poseYaw(*latest_pose_);
    }
    motion_segment_observed_ = false;
    zero_hold_clock_.begin(current_time.seconds(), retry_waiting_);
  }

  void resumeZeroHold(const rclcpp::Time & current_time)
  {
    if (!zero_hold_clock_.active()) {
      return;
    }
    zero_hold_clock_.resume(current_time.seconds());
    // Ignore passive terrain settling while the exact-zero sentinel owned the
    // output, but preserve distance/yaw accumulated by earlier authorized
    // motion in this bounded attempt.
    if (latest_pose_.has_value()) {
      start_pose_ = latest_pose_;
      start_yaw_ = poseYaw(*latest_pose_);
    }
    motion_segment_observed_ = false;
  }

  double episodeElapsed(const rclcpp::Time & current_time) const
  {
    if (!behavior_config_.zero_hold_pauses_limits) {
      return (current_time - episode_start_time_).seconds();
    }
    return zero_hold_clock_.episodeElapsed(
      current_time.seconds(), episode_start_time_.seconds());
  }

  double attemptElapsed(const rclcpp::Time & current_time) const
  {
    if (!behavior_config_.zero_hold_pauses_limits) {
      return (current_time - attempt_start_time_).seconds();
    }
    return zero_hold_clock_.attemptElapsed(
      current_time.seconds(), attempt_start_time_.seconds());
  }

  double retryElapsed(const rclcpp::Time & current_time) const
  {
    if (!behavior_config_.zero_hold_pauses_limits) {
      return (current_time - retry_wait_start_time_).seconds();
    }
    return zero_hold_clock_.retryElapsed(
      current_time.seconds(), retry_wait_start_time_.seconds());
  }

  void tick()
  {
    if (!enabled_ || !hold_active_) {
      publishStatus("IDLE", enabled_ ? "waiting_for_route_safety_hold" : "disabled");
      return;
    }
    const auto current_time = now();
    if ((current_time - last_gate_status_time_).seconds() > status_timeout_s_) {
      resumeZeroHold(current_time);
      publishZero();
      publishStatus("SAFETY_HOLD", "gate_status_stale");
      return;
    }
    if (!poseFresh()) {
      resumeZeroHold(current_time);
      publishZero();
      publishStatus("SAFETY_HOLD", "pose_missing_or_stale");
      return;
    }
    if ((current_time - last_candidate_time_).seconds() > candidate_timeout_s_) {
      resumeZeroHold(current_time);
      publishZero();
      publishStatus("SAFETY_HOLD", "candidate_missing_or_stale");
      return;
    }
    if (limit_reached_ && behavior_config_.zero_hold_pauses_limits) {
      resumeZeroHold(current_time);
      publishZero();
      publishStatus("LIMIT_HOLD", recoveryDetail("bounded_recovery_limit_reached"));
      return;
    }

    const auto candidate_disposition =
      ClassifyBoundedRecoveryCandidateForConfig(candidate_, behavior_config_);
    if (BoundedRecoveryCandidatePausesLimits(
        candidate_disposition, behavior_config_))
    {
      beginZeroHold(current_time);
      publishZero();
      publishStatus(
        "SAFETY_HOLD", recoveryDetail("gate_candidate_zero_hold"));
      return;
    }
    resumeZeroHold(current_time);

    const double total_elapsed = episodeElapsed(current_time);
    if (retry_waiting_) {
      if (total_elapsed >= maximum_total_duration_s_ ||
        cumulative_distance_m_ >= maximum_total_distance_m_)
      {
        limit_reached_ = true;
        publishZero();
        publishStatus("LIMIT_HOLD", recoveryDetail("total_recovery_limit_reached"));
        return;
      }
      if (retryElapsed(current_time) < retry_pause_s_)
      {
        publishZero();
        publishStatus("RETRY_WAIT", recoveryDetail("waiting_for_fresh_candidate"));
        return;
      }
      ++current_attempt_;
      attempt_start_time_ = current_time;
      start_pose_ = latest_pose_;
      start_yaw_ = poseYaw(*latest_pose_);
      attempt_distance_offset_m_ = 0.0;
      attempt_yaw_offset_rad_ = 0.0;
      motion_segment_observed_ = false;
      zero_hold_clock_.resetAttempt();
      zero_hold_clock_.resetRetry();
      retry_waiting_ = false;
      RCLCPP_WARN(
        get_logger(), "bounded route recovery retry %d/%d",
        current_attempt_, maximum_attempts_);
    }

    const double elapsed = attemptElapsed(current_time);
    const double distance = attemptDistance();
    // HH_260818 - Sum one net displacement per bounded attempt instead of
    // every 20 Hz pose segment. Segment summation turns harmless localization
    // jitter into artificial travel and can exhaust the episode while stopped.
    const double total_distance = cumulative_distance_m_ + distance;
    const auto limit_action = EvaluateBoundedRecoveryAttempt(
      attempt_limits_, current_attempt_, elapsed, distance, total_elapsed,
      total_distance);
    if (limit_reached_ || limit_action == BoundedRecoveryAction::kFinalHold) {
      limit_reached_ = true;
      publishZero();
      publishStatus("LIMIT_HOLD", recoveryDetail("bounded_recovery_limit_reached"));
      return;
    }
    if (limit_action == BoundedRecoveryAction::kRetry) {
      cumulative_distance_m_ = total_distance;
      attempt_distance_offset_m_ = 0.0;
      attempt_yaw_offset_rad_ = 0.0;
      motion_segment_observed_ = false;
      if (latest_pose_.has_value()) {
        start_pose_ = latest_pose_;
        start_yaw_ = poseYaw(*latest_pose_);
      }
      retry_waiting_ = true;
      retry_wait_start_time_ = current_time;
      zero_hold_clock_.resetRetry();
      publishZero();
      publishStatus("RETRY_WAIT", recoveryDetail("attempt_limit_reached"));
      return;
    }

    if (candidate_disposition == BoundedRecoveryCandidateDisposition::kInvalid ||
      (!behavior_config_.zero_hold_pauses_limits && candidate_disposition ==
      BoundedRecoveryCandidateDisposition::kZeroHold))
    {
      publishZero();
      publishStatus("SAFETY_HOLD", "candidate_outside_bounded_twist");
      return;
    }
    const double norm = std::hypot(candidate_.linear.x, candidate_.linear.y);
    avg_msgs::msg::AvgTwist command = candidate_;
    const double scale = std::min(1.0, maximum_speed_mps_ / norm);
    command.linear.x *= scale;
    command.linear.y *= scale;
    command.angular.z = std::clamp(
      command.angular.z, -maximum_angular_speed_radps_, maximum_angular_speed_radps_);
    // Keep the whole-attempt yaw envelope, while allowing a correction that
    // reduces an offset already created by a preceding crab stage.  The old
    // absolute-only clamp deleted that safe correction and converted it into
    // a straight reverse command, which the gate then held at exact zero.
    if (!BoundedRecoveryYawCommandPermittedForConfig(
        signedYawDisplacement(), command.angular.z,
        maximum_yaw_change_rad_, behavior_config_))
    {
      command.angular.z = 0.0;
    }
    command_publisher_->publish(command);
    motion_segment_observed_ = true;
    const std::string motion = std::abs(command.linear.y) > std::abs(command.linear.x) ?
      (command.linear.y > 0.0 ? "CRAB_LEFT" : "CRAB_RIGHT") :
      command.angular.z > 1.0e-6 ? "REVERSE_YAW_LEFT" :
      command.angular.z < -1.0e-6 ? "REVERSE_YAW_RIGHT" : "REVERSE";
    publishStatus(
      motion,
      recoveryDetail("moving") +
      " distance=" + std::to_string(distance) +
      " yaw_deg=" + std::to_string(yawDisplacement() * 180.0 / M_PI) +
      " elapsed=" + std::to_string(elapsed));
  }

  std::string recoveryDetail(const std::string & reason) const
  {
    const auto current_time = now();
    std::string detail =
      reason +
      " attempt=" + std::to_string(current_attempt_) + "/" +
      std::to_string(maximum_attempts_) +
      " total_distance=" +
      std::to_string(cumulative_distance_m_ + attemptDistance()) +
      " total_elapsed=" + std::to_string(episodeElapsed(current_time));
    if (behavior_config_.zero_hold_pauses_limits) {
      detail += " zero_hold_elapsed=" +
        std::to_string(zero_hold_clock_.zeroHoldElapsed(current_time.seconds()));
    }
    return detail;
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
  double maximum_angular_speed_radps_{0.10};
  double maximum_yaw_change_rad_{12.0 * M_PI / 180.0};
  double maximum_distance_m_{0.40};
  double maximum_duration_s_{10.0};
  int maximum_attempts_{50};
  double retry_pause_s_{0.5};
  double maximum_total_distance_m_{1.50};
  double maximum_total_duration_s_{90.0};
  BoundedRecoveryBehaviorConfig behavior_config_;
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
  std::optional<double> start_yaw_;
  BoundedRecoveryAttemptLimits attempt_limits_;
  int current_attempt_{0};
  double cumulative_distance_m_{0.0};
  double attempt_distance_offset_m_{0.0};
  double attempt_yaw_offset_rad_{0.0};
  bool motion_segment_observed_{false};
  BoundedRecoveryZeroHoldClock zero_hold_clock_;
  bool retry_waiting_{false};
  rclcpp::Time last_gate_status_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_candidate_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time episode_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time attempt_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time retry_wait_start_time_{0, 0, RCL_ROS_TIME};
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
