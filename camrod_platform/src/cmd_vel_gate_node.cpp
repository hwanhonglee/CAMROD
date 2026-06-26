#include <algorithm>
#include <cctype>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace camrod_platform
{

class CmdVelGateNode : public rclcpp::Node
{
public:
  CmdVelGateNode()
  : Node("cmd_vel_gate")
  {
    input_cmd_vel_topic_ =
      declare_parameter<std::string>("input_cmd_vel_topic", "/planning/cmd_vel");
    output_cmd_vel_topic_ =
      declare_parameter<std::string>("output_cmd_vel_topic", "/platform/cmd_vel");
    enable_topic_ =
      declare_parameter<std::string>("enable_topic", "/platform/drive_enable");
    engage_topic_ =
      declare_parameter<std::string>("engage_topic", "/planning/engaged");
    // HH_260522: unified source selector for engage signal.
    // HH_260624 - Default to /planning/engaged so manual 2D-goal engage and
    // UI mission engage remain independent before reaching the final platform gate.
    //   planning_engage/planning_engaged/topic/enabled/on: subscribe
    //   disabled/off/none: ignore
    engage_source_mode_ =
      declare_parameter<std::string>("engage_source_mode", "planning_engaged");
    std::transform(
      engage_source_mode_.begin(),
      engage_source_mode_.end(),
      engage_source_mode_.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    if (
      engage_source_mode_ == "disabled" || engage_source_mode_ == "off" ||
      engage_source_mode_ == "none")
    {
      enable_engage_topic_sub_ = false;
    } else if (
      engage_source_mode_ == "planning_engage" || engage_source_mode_ == "planning_engaged" ||
      engage_source_mode_ == "topic" || engage_source_mode_ == "enabled" ||
      engage_source_mode_ == "on")
    {
      enable_engage_topic_sub_ = true;
    } else {
      enable_engage_topic_sub_ = true;
      RCLCPP_WARN(
        get_logger(),
        "Unknown engage_source_mode '%s'. Using planning_engaged mode.",
        engage_source_mode_.c_str());
    }
    state_topic_ =
      declare_parameter<std::string>("state_topic", "/platform/drive_enabled");
    // HH_260522: unified source selector for e-stop signal.
    //   platform_status/topic/enabled/on: subscribe
    //   disabled/off/none: ignore
    estop_source_mode_ =
      declare_parameter<std::string>("estop_source_mode", "platform_status");
    std::transform(
      estop_source_mode_.begin(),
      estop_source_mode_.end(),
      estop_source_mode_.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    if (
      estop_source_mode_ == "disabled" || estop_source_mode_ == "off" ||
      estop_source_mode_ == "none")
    {
      enable_estop_topic_sub_ = false;
    } else if (
      estop_source_mode_ == "platform_status" || estop_source_mode_ == "topic" ||
      estop_source_mode_ == "enabled" || estop_source_mode_ == "on")
    {
      enable_estop_topic_sub_ = true;
    } else {
      enable_estop_topic_sub_ = true;
      RCLCPP_WARN(
        get_logger(),
        "Unknown estop_source_mode '%s'. Using platform_status mode.",
        estop_source_mode_.c_str());
    }
    estop_topic_ =
      declare_parameter<std::string>("estop_topic", "/platform/status/estop");
    allow_on_start_ =
      declare_parameter<bool>("allow_on_start", false);
    publish_zero_when_blocked_ =
      declare_parameter<bool>("publish_zero_when_blocked", true);
    // HH_260626: If planning stops publishing, keep sending zero so Ranger
    // never continues with a stale platform command.
    input_timeout_s_ =
      declare_parameter<double>("input_timeout_s", 0.50);
    zero_publish_rate_hz_ =
      declare_parameter<double>("zero_publish_rate_hz", 10.0);

    drive_enabled_ = allow_on_start_;
    planning_engaged_ = !enable_engage_topic_sub_ || allow_on_start_;
    estop_ = false;

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 10);
    pub_state_ = create_publisher<std_msgs::msg::Bool>(state_topic_, 10);

    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      input_cmd_vel_topic_, 10,
      std::bind(&CmdVelGateNode::on_cmd_vel, this, std::placeholders::_1));
    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, 10,
      std::bind(&CmdVelGateNode::on_enable, this, std::placeholders::_1));
    if (enable_engage_topic_sub_) {
      sub_engage_ = create_subscription<std_msgs::msg::Bool>(
        engage_topic_, 10,
        std::bind(&CmdVelGateNode::on_engage, this, std::placeholders::_1));
    }
    if (enable_estop_topic_sub_) {
      sub_estop_ = create_subscription<std_msgs::msg::Bool>(
        estop_topic_, 10,
        std::bind(&CmdVelGateNode::on_estop, this, std::placeholders::_1));
    }

    srv_set_enabled_ = create_service<std_srvs::srv::SetBool>(
      "set_enabled",
      std::bind(
        &CmdVelGateNode::on_set_enabled, this, std::placeholders::_1,
        std::placeholders::_2));

    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&CmdVelGateNode::publish_state, this));
    const auto zero_period_ms = static_cast<int>(
      1000.0 / std::max(1.0, zero_publish_rate_hz_));
    cmd_timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(zero_period_ms),
      std::bind(&CmdVelGateNode::on_cmd_timeout_timer, this));

    publish_state();
    RCLCPP_INFO(
      get_logger(),
      "cmd_vel_gate ready: in=%s out=%s enable_topic=%s engage_topic=%s "
      "estop_topic=%s allow_on_start=%s drive_enabled=%s planning_engaged=%s "
      "input_timeout_s=%.2f zero_rate_hz=%.1f",
      input_cmd_vel_topic_.c_str(), output_cmd_vel_topic_.c_str(),
      enable_topic_.c_str(),
      enable_engage_topic_sub_ ? engage_topic_.c_str() : "(disabled)",
      enable_estop_topic_sub_ ? estop_topic_.c_str() : "(disabled)",
      allow_on_start_ ? "true" : "false",
      drive_enabled_ ? "true" : "false",
      planning_engaged_ ? "true" : "false",
      input_timeout_s_, zero_publish_rate_hz_);
  }

private:
  // HH_260625: Platform drive-enable and planning engage are independent latches.
  // The final platform cmd_vel opens only when both latches are true and e-stop is clear.
  bool effective_enabled() const
  {
    return drive_enabled_ && planning_engaged_ && !estop_;
  }

  // Publishes effective drive state for downstream modules.
  void publish_state()
  {
    std_msgs::msg::Bool msg;
    msg.data = effective_enabled();
    pub_state_->publish(msg);
  }

  // Emits explicit zero Twist while the gate is blocking commands.
  void publish_zero()
  {
    geometry_msgs::msg::Twist zero;
    pub_cmd_->publish(zero);
  }

  // Passes through or blocks cmd_vel based on gate/e-stop state.
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = now();
    has_cmd_ = true;
    cmd_input_stale_ = false;

    if (effective_enabled()) {
      pub_cmd_->publish(*msg);
      return;
    }

    if (publish_zero_when_blocked_) {
      publish_zero();
    }
    RCLCPP_DEBUG(
      get_logger(), "cmd_vel blocked: drive_enabled=%s planning_engaged=%s estop=%s",
      drive_enabled_ ? "true" : "false",
      planning_engaged_ ? "true" : "false",
      estop_ ? "true" : "false");
  }

  // Forces zero output when the upstream planning command stream stalls.
  void on_cmd_timeout_timer()
  {
    if (input_timeout_s_ <= 0.0 || !has_cmd_) {
      return;
    }

    const double age_s = (now() - last_cmd_time_).seconds();
    if (age_s <= input_timeout_s_) {
      return;
    }

    if (publish_zero_when_blocked_) {
      publish_zero();
    }
    if (cmd_input_stale_) {
      return;
    }

    cmd_input_stale_ = true;
    RCLCPP_WARN(
      get_logger(),
      "cmd_vel input stale: last_cmd_age=%.2fs timeout=%.2fs; publishing zero",
      age_s, input_timeout_s_);
  }

  // Updates gate state from /platform/drive_enable.
  void on_enable(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool new_enabled = static_cast<bool>(msg->data);
    if (new_enabled == drive_enabled_) {
      return;
    }
    drive_enabled_ = new_enabled;
    publish_state();
    if (!effective_enabled() && publish_zero_when_blocked_) {
      publish_zero();
    }
    RCLCPP_INFO(
      get_logger(),
      "drive enable topic update: drive_enabled=%s planning_engaged=%s estop=%s effective=%s",
      drive_enabled_ ? "true" : "false",
      planning_engaged_ ? "true" : "false",
      estop_ ? "true" : "false",
      effective_enabled() ? "true" : "false");
  }

  // Mirrors the configured planning engage-state topic into the planning latch.
  void on_engage(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool new_enabled = static_cast<bool>(msg->data);
    if (new_enabled == planning_engaged_) {
      return;
    }
    planning_engaged_ = new_enabled;
    publish_state();
    if (!effective_enabled() && publish_zero_when_blocked_) {
      publish_zero();
    }
    RCLCPP_INFO(
      get_logger(),
      "planning engage-state update: drive_enabled=%s planning_engaged=%s estop=%s effective=%s",
      drive_enabled_ ? "true" : "false",
      planning_engaged_ ? "true" : "false",
      estop_ ? "true" : "false",
      effective_enabled() ? "true" : "false");
  }

  // Applies e-stop override and forces zero command when active.
  void on_estop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool new_estop = static_cast<bool>(msg->data);
    if (new_estop == estop_) {
      return;
    }
    estop_ = new_estop;
    publish_state();
    if (estop_ && publish_zero_when_blocked_) {
      publish_zero();
    }
    RCLCPP_WARN(
      get_logger(),
      "estop update: estop=%s effective_enabled=%s",
      estop_ ? "true" : "false",
      effective_enabled() ? "true" : "false");
  }

  // Service API to toggle gate state at runtime.
  void on_set_enabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    drive_enabled_ = static_cast<bool>(request->data);
    publish_state();
    if (!effective_enabled() && publish_zero_when_blocked_) {
      publish_zero();
    }
    response->success = true;
    response->message =
      "drive_enabled=" + std::string(drive_enabled_ ? "true" : "false") +
      " planning_engaged=" + std::string(planning_engaged_ ? "true" : "false") +
      " estop=" + std::string(estop_ ? "true" : "false") +
      " effective=" + std::string(effective_enabled() ? "true" : "false");
  }

private:
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string enable_topic_;
  std::string engage_topic_;
  std::string engage_source_mode_;
  std::string state_topic_;
  std::string estop_source_mode_;
  std::string estop_topic_;
  bool enable_engage_topic_sub_{true};
  bool enable_estop_topic_sub_{true};
  bool allow_on_start_{false};
  bool publish_zero_when_blocked_{true};
  double input_timeout_s_{0.50};
  double zero_publish_rate_hz_{10.0};
  bool drive_enabled_{false};
  bool planning_engaged_{false};
  bool estop_{false};
  bool has_cmd_{false};
  bool cmd_input_stale_{false};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_state_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_engage_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_set_enabled_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr cmd_timeout_timer_;
};

}  // namespace camrod_platform

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod_platform::CmdVelGateNode>());
  rclcpp::shutdown();
  return 0;
}
