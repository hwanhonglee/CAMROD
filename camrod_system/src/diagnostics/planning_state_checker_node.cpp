#include <chrono>
#include <cstdio>
#include <string>

#include "avg_msgs/msg/planning_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_diagnostics_base/base_checker.hpp"

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using diagnostic_updater::DiagnosticStatusWrapper;
using robot_diagnostics_base::BaseChecker;

namespace camrod_system
{

class PlanningStateChecker : public BaseChecker
{
public:
  PlanningStateChecker()
  : BaseChecker("planning_state_checker", "planning")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("state_topic", std::string("/planning/state_machine/state"));
    declare_parameter("stale_timeout_s", 3.0);
  }

  void load_parameters_() override
  {
    state_topic_ = get_param<std::string>("state_topic", state_topic_);
    stale_timeout_s_ = get_param<double>("stale_timeout_s", stale_timeout_s_);

    state_sub_ = create_subscription<avg_msgs::msg::PlanningState>(
      state_topic_, rclcpp::QoS(10),
      [this](avg_msgs::msg::PlanningState::ConstSharedPtr msg) {
        latest_state_ = *msg;
        have_state_ = true;
        last_state_time_ = now();
      });
  }

  void setup_tasks_() override
  {
    // HH_260617: Validate CAMROD semantic planning-state output, not only Nav2 action topics.
    add_task(
      "/planning/state_machine/state",
      [this](DiagnosticStatusWrapper & stat) { check_state(stat); });
  }

private:
  void check_state(DiagnosticStatusWrapper & stat)
  {
    if (!have_state_) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "no planning state received: " + state_topic_);
      stat.add("topic", state_topic_);
      return;
    }

    const double age_s = (now() - last_state_time_).seconds();
    if (age_s > stale_timeout_s_) {
      char buf[128];
      std::snprintf(buf, sizeof(buf), "planning state stale %.2fs", age_s);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string(buf));
      stat.add("topic", state_topic_);
      stat.add("last_msg_sec_ago", age_s);
      return;
    }

    int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "planning state ok";
    if (latest_state_.state == avg_msgs::msg::PlanningState::ERROR_STOP) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "planning state ERROR_STOP";
    } else if (latest_state_.state == avg_msgs::msg::PlanningState::WARN_RECOVERY) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "planning state WARN_RECOVERY";
    } else if (latest_state_.estop) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message = "planning estop asserted";
    }

    stat.summary(level, message);
    stat.add("topic", state_topic_);
    stat.add("state", static_cast<int>(latest_state_.state));
    stat.add("label", latest_state_.label);
    stat.add("scenario_id", static_cast<int>(latest_state_.scenario_id));
    stat.add("scenario_label", latest_state_.scenario_label);
    stat.add("active_mission_key", latest_state_.active_mission_key);
    stat.add("active_goal_source", latest_state_.active_goal_source);
    stat.add("estop", latest_state_.estop ? "true" : "false");
    stat.add("return_requested", latest_state_.return_requested ? "true" : "false");
    stat.add("recall_requested", latest_state_.recall_requested ? "true" : "false");
    stat.add("last_msg_sec_ago", age_s);
  }

  std::string state_topic_{"/planning/state_machine/state"};
  double stale_timeout_s_{3.0};
  bool have_state_{false};
  avg_msgs::msg::PlanningState latest_state_;
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<avg_msgs::msg::PlanningState>::SharedPtr state_sub_;
};

}  // namespace camrod_system

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camrod_system::PlanningStateChecker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
