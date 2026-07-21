// 260708 - Exterior light controller.
// Decides the lamp state (headlight relay + WS2815 turn indicators) from:
//   priority 1: hazard  — /platform/status OR /planning/state_machine/estop
//               (the state machine mirrors ERROR_STOP onto its estop topic, HH_260701)
//   priority 2: crab    — camping_site_maneuver_controller active phase + |cmd_vel.linear.y|
//   priority 3: turn    — lanelet turn_direction windows from /planning/route_turn_segments
// HH_260720 - Read the generated global-path mirror instead of the Nav2 ROS boundary.
//               matched against the robot arc length on /planning/global_path_avg
//   priority 4: off
// Headlight is an independent pass-through channel from the UI button.
// Pure decision logic lives in include/camrod_platform/light_decision.hpp (gtest).
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "avg_msgs/msg/avg_light_command.hpp"
#include "avg_msgs/msg/avg_bool.hpp"
#include "avg_msgs/msg/avg_path.hpp"
#include "avg_msgs/msg/avg_platform_status.hpp"
#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "avg_msgs/msg/avg_twist.hpp"
#include "avg_msgs/msg/module_state.hpp"
#include "avg_msgs/msg/route_turn_segment_array.hpp"
#include "rclcpp/rclcpp.hpp"

#include "camrod_platform/light_decision.hpp"

namespace camrod_platform
{

class LightControllerNode : public rclcpp::Node
{
public:
  LightControllerNode()
  : rclcpp::Node("light_controller")
  {
    platform_status_topic_ = declare_parameter<std::string>(
      "platform_status_topic", "/platform/status");
    estop_state_machine_topic_ = declare_parameter<std::string>(
      "estop_state_machine_topic", "/planning/state_machine/estop");
    headlight_command_topic_ = declare_parameter<std::string>(
      "headlight_command_topic", "/platform/headlight/command");
    camping_site_maneuver_controller_status_topic_ = declare_parameter<std::string>(
      // HH_260720 - Observe the control-owned campsite maneuver phase.
      "camping_site_maneuver_controller_status_topic", "/control/camping_site_maneuver_controller/status");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/control/cmd_vel");
    // HH_260720 - Subscribe to the generated CAMROD path mirror, not Nav2's ROS boundary.
    global_path_topic_ = declare_parameter<std::string>(
      "global_path_topic", "/planning/global_path_avg");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/localization/pose");
    route_turn_segments_topic_ = declare_parameter<std::string>(
      "route_turn_segments_topic", "/planning/route_turn_segments");
    lights_command_topic_ = declare_parameter<std::string>(
      "lights_command_topic", "/platform/lights/command");

    params_.turn_pre_distance_m = declare_parameter<double>("turn_pre_distance_m", 12.0);
    params_.crab_lateral_threshold_mps =
      declare_parameter<double>("crab_lateral_threshold_mps", 0.05);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
    pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 2.5);
    site_status_timeout_s_ = declare_parameter<double>("site_status_timeout_s", 3.0);
    cmd_vel_timeout_s_ = declare_parameter<double>("cmd_vel_timeout_s", 1.0);
    route_length_tolerance_m_ = declare_parameter<double>("route_length_tolerance_m", 1.0);

    publish_rate_hz_ = std::clamp(publish_rate_hz_, 0.5, 50.0);

    // HH_260720 - Hardware e-stop comes from the canonical generated platform status.
    sub_platform_status_ = create_subscription<avg_msgs::msg::AvgPlatformStatus>(
      platform_status_topic_, 10,
      [this](avg_msgs::msg::AvgPlatformStatus::ConstSharedPtr msg) {
        estop_platform_ = msg->estop;
      });
    sub_estop_state_machine_ = create_subscription<avg_msgs::msg::AvgBool>(
      estop_state_machine_topic_, 10,
      [this](avg_msgs::msg::AvgBool::ConstSharedPtr msg) {estop_state_machine_ = msg->data;});
    sub_headlight_ = create_subscription<avg_msgs::msg::AvgBool>(
      headlight_command_topic_, 10,
      [this](avg_msgs::msg::AvgBool::ConstSharedPtr msg) {headlight_on_ = msg->data;});
    sub_site_status_ = create_subscription<avg_msgs::msg::ModuleState>(
      camping_site_maneuver_controller_status_topic_, 10,
      [this](avg_msgs::msg::ModuleState::ConstSharedPtr msg) {
        // HH_260721 - Indicator behavior follows the explicit phase, never a health severity.
        camping_site_maneuver_controller_active_ =
          isCampingSiteManeuverActive(msg->operating_state);
        site_status_stamp_ = now();
      });
    sub_cmd_vel_ = create_subscription<avg_msgs::msg::AvgTwist>(
      cmd_vel_topic_, 10,
      [this](avg_msgs::msg::AvgTwist::ConstSharedPtr msg) {
        cmd_vel_lateral_mps_ = msg->linear.y;
        cmd_vel_stamp_ = now();
      });
    sub_pose_ = create_subscription<avg_msgs::msg::AvgPoseStamped>(
      pose_topic_, 10,
      [this](avg_msgs::msg::AvgPoseStamped::ConstSharedPtr msg) {
        pose_x_ = msg->pose.position.x;
        pose_y_ = msg->pose.position.y;
        pose_stamp_ = now();
      });

    // Planner publishes both route topics reliable + transient_local so a late
    // subscriber immediately receives the active route (HH_260629 pattern).
    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    sub_global_path_ = create_subscription<avg_msgs::msg::AvgPath>(
      global_path_topic_, latched_qos,
      [this](avg_msgs::msg::AvgPath::ConstSharedPtr msg) {onGlobalPath(*msg);});
    // HH_260720 - Consume named turn-segment fields instead of packed float triples.
    sub_turn_segments_ = create_subscription<avg_msgs::msg::RouteTurnSegmentArray>(
      route_turn_segments_topic_, latched_qos,
      [this](avg_msgs::msg::RouteTurnSegmentArray::ConstSharedPtr msg) {onTurnSegments(*msg);});

    pub_lights_ = create_publisher<avg_msgs::msg::AvgLightCommand>(lights_command_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(),
      "light_controller ready: pre_distance=%.1fm crab_threshold=%.2fm/s rate=%.1fHz",
      params_.turn_pre_distance_m, params_.crab_lateral_threshold_mps, publish_rate_hz_);
  }

private:
  void onGlobalPath(const avg_msgs::msg::AvgPath & msg)
  {
    path_points_.clear();
    path_cumulative_s_.clear();
    path_points_.reserve(msg.poses.size());
    path_cumulative_s_.reserve(msg.poses.size());
    double cumulative = 0.0;
    for (std::size_t i = 0; i < msg.poses.size(); ++i) {
      const auto & p = msg.poses[i].pose.position;
      if (i > 0U) {
        const double dx = p.x - path_points_[i - 1U].first;
        const double dy = p.y - path_points_[i - 1U].second;
        cumulative += std::hypot(dx, dy);
      }
      path_points_.emplace_back(p.x, p.y);
      path_cumulative_s_.push_back(cumulative);
    }
    path_total_length_m_ = cumulative;
  }

  void onTurnSegments(const avg_msgs::msg::RouteTurnSegmentArray & msg)
  {
    turn_segments_.clear();
    segments_total_length_m_ = static_cast<double>(msg.total_route_length_m);
    turn_segments_.reserve(msg.segments.size());
    for (const auto & typed_segment : msg.segments) {
      TurnSegment segment;
      segment.direction = typed_segment.direction;
      segment.start_s = typed_segment.start_distance_m;
      segment.end_s = typed_segment.end_distance_m;
      turn_segments_.push_back(segment);
    }
  }

  bool isFresh(const rclcpp::Time & stamp, double timeout_s) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (now() - stamp).seconds() <= timeout_s;
  }

  // Nearest path point by full scan; at 0.2 m resolution and 5 Hz this stays
  // cheap for the route lengths CAMROD drives (hundreds of meters).
  double computeRouteS() const
  {
    if (path_points_.size() < 2U) {
      return -1.0;
    }
    double best_distance_sq = std::numeric_limits<double>::max();
    std::size_t best_index = 0U;
    for (std::size_t i = 0; i < path_points_.size(); ++i) {
      const double dx = path_points_[i].first - pose_x_;
      const double dy = path_points_[i].second - pose_y_;
      const double distance_sq = dx * dx + dy * dy;
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_index = i;
      }
    }
    return path_cumulative_s_[best_index];
  }

  void onTimer()
  {
    LightDecisionInput input;
    input.estop_platform = estop_platform_;
    input.estop_state_machine = estop_state_machine_;
    input.camping_site_maneuver_controller_active =
      camping_site_maneuver_controller_active_ && isFresh(site_status_stamp_, site_status_timeout_s_);
    input.cmd_vel_lateral_mps =
      isFresh(cmd_vel_stamp_, cmd_vel_timeout_s_) ? cmd_vel_lateral_mps_ : 0.0;

    const bool pose_fresh = isFresh(pose_stamp_, pose_timeout_s_);
    const bool sync_ok = segments_total_length_m_ >= 0.0 &&
      routeLengthMatches(
      path_total_length_m_, segments_total_length_m_, route_length_tolerance_m_);
    input.route_context_valid = pose_fresh && sync_ok;
    input.route_s = pose_fresh ? computeRouteS() : -1.0;
    input.turn_segments = turn_segments_;

    const LightDecisionResult result = decideIndicator(input, params_);

    avg_msgs::msg::AvgLightCommand command;
    command.header.stamp = now();
    command.headlight = headlight_on_;
    command.indicator = static_cast<uint8_t>(result.mode);
    command.source = result.source;
    pub_lights_->publish(command);

    if (result.mode != last_mode_ || headlight_on_ != last_headlight_) {
      RCLCPP_INFO(
        get_logger(), "lights: indicator=%u headlight=%s source=%s",
        static_cast<unsigned>(result.mode), headlight_on_ ? "on" : "off",
        result.source.c_str());
      last_mode_ = result.mode;
      last_headlight_ = headlight_on_;
    }
  }

  std::string platform_status_topic_;
  std::string estop_state_machine_topic_;
  std::string headlight_command_topic_;
  std::string camping_site_maneuver_controller_status_topic_;
  std::string cmd_vel_topic_;
  std::string global_path_topic_;
  std::string pose_topic_;
  std::string route_turn_segments_topic_;
  std::string lights_command_topic_;

  LightDecisionParams params_;
  double publish_rate_hz_{5.0};
  double pose_timeout_s_{2.5};
  double site_status_timeout_s_{3.0};
  double cmd_vel_timeout_s_{1.0};
  double route_length_tolerance_m_{1.0};

  bool estop_platform_{false};
  bool estop_state_machine_{false};
  bool headlight_on_{false};
  bool camping_site_maneuver_controller_active_{false};
  double cmd_vel_lateral_mps_{0.0};
  double pose_x_{0.0};
  double pose_y_{0.0};
  rclcpp::Time site_status_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cmd_vel_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pose_stamp_{0, 0, RCL_ROS_TIME};

  std::vector<std::pair<double, double>> path_points_;
  std::vector<double> path_cumulative_s_;
  double path_total_length_m_{-1.0};
  std::vector<TurnSegment> turn_segments_;
  double segments_total_length_m_{-1.0};

  IndicatorMode last_mode_{IndicatorMode::kOff};
  bool last_headlight_{false};

  rclcpp::Subscription<avg_msgs::msg::AvgPlatformStatus>::SharedPtr sub_platform_status_;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr sub_estop_state_machine_;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr sub_headlight_;
  rclcpp::Subscription<avg_msgs::msg::ModuleState>::SharedPtr sub_site_status_;
  rclcpp::Subscription<avg_msgs::msg::AvgTwist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<avg_msgs::msg::AvgPath>::SharedPtr sub_global_path_;
  rclcpp::Subscription<avg_msgs::msg::RouteTurnSegmentArray>::SharedPtr sub_turn_segments_;
  rclcpp::Publisher<avg_msgs::msg::AvgLightCommand>::SharedPtr pub_lights_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camrod_platform

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod_platform::LightControllerNode>());
  rclcpp::shutdown();
  return 0;
}
