/**
 * Planning Nav Status Checker Node
 *
 * Subscribes to the navigate_to_pose GoalStatusArray topic and publishes
 * advisory Nav2 navigation status diagnostics.
 *
 * Diagnostic item
 * ---------------
 *   /planning/nav_status
 *     - Freshness    : disabled by default because action status is advisory
 *     - Status       : EXECUTING/ACCEPTED -> OK
 *                      ABORTED once      -> WARN
 *                      repeated ABORTED  -> ERROR when above abort_error
 *     - Abort count  : 60s rolling window
 *                      > abort_warn  -> WARN
 *                      > abort_error -> ERROR
 *
 * Parameters
 * ----------
 *   nav_status_topic:  "/planning/navigate_to_pose/_action/status"
 *   stale_timeout_s:   0.0   # <=0 disables action-status freshness timeout
 *   abort_warn:        2     # abort count in 60s > this value -> WARN
 *   abort_error:       5     # abort count in 60s > this value -> ERROR
 *   terminal_status_stale_ok: true
 *                     # quiet status after SUCCEEDED/CANCELED is normal idle
 */

#include <cstdio>
#include <deque>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/msg/goal_status.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;
using GoalStatusArray  = action_msgs::msg::GoalStatusArray;
using GoalStatus       = action_msgs::msg::GoalStatus;

struct NavStatusState
{
  std::mutex mtx;

  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};

  int8_t current_status{GoalStatus::STATUS_UNKNOWN};

  std::deque<rclcpp::Time> abort_times;

  rclcpp::Subscription<GoalStatusArray>::SharedPtr sub;
};

class PlanningNavStatusCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  PlanningNavStatusCheckerNode()
  : robot_diagnostics_base::BaseChecker(
      "planning_nav_status_checker", "planning_nav_status_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("nav_status_topic",
      std::string("/planning/navigate_to_pose/_action/status"));
    declare_parameter("stale_timeout_s", 0.0);
    declare_parameter("idle_ok_without_status", true);
    declare_parameter("terminal_status_stale_ok", true);
    declare_parameter("abort_warn",    2);
    declare_parameter("abort_error",   5);
  }

  void load_parameters_() override
  {
    nav_status_topic_ = get_parameter("nav_status_topic").as_string();
    stale_timeout_ = get_param<double>("stale_timeout_s", stale_timeout_);
    idle_ok_without_status_ = get_parameter("idle_ok_without_status").as_bool();
    terminal_status_stale_ok_ = get_parameter("terminal_status_stale_ok").as_bool();
    abort_warn_       = get_parameter("abort_warn").as_int();
    abort_error_      = get_parameter("abort_error").as_int();
  }

  void setup_tasks_() override
  {
    state_.sub = create_subscription<GoalStatusArray>(
      nav_status_topic_, rclcpp::QoS(10),
      [this](const GoalStatusArray::ConstSharedPtr msg) { onNavStatus(msg); });

    add_task("/planning/nav_status",
      [this](StatusWrapper & stat) { checkNavStatus(stat); });

    RCLCPP_INFO(get_logger(),
      "Planning nav status checker started "
      "(topic=%s, stale=%.1fs, terminal_stale_ok=%s, abort_warn=%d, abort_error=%d)",
      nav_status_topic_.c_str(), stale_timeout_,
      terminal_status_stale_ok_ ? "true" : "false", abort_warn_, abort_error_);
  }

private:
  void onNavStatus(const GoalStatusArray::ConstSharedPtr msg)
  {
    auto now = this->now();

    // Select the dominant status from the status list.
    // Priority: EXECUTING > ACCEPTED > CANCELING > terminal states.
    int8_t dominant = GoalStatus::STATUS_UNKNOWN;
    bool   aborted  = false;

    for (const auto & s : msg->status_list) {
      if (s.status == GoalStatus::STATUS_EXECUTING) {
        dominant = GoalStatus::STATUS_EXECUTING;
      } else if (s.status == GoalStatus::STATUS_ACCEPTED &&
                 dominant != GoalStatus::STATUS_EXECUTING) {
        dominant = GoalStatus::STATUS_ACCEPTED;
      } else if (s.status == GoalStatus::STATUS_ABORTED) {
        aborted = true;
        if (dominant == GoalStatus::STATUS_UNKNOWN) {
          dominant = GoalStatus::STATUS_ABORTED;
        }
      } else if (dominant == GoalStatus::STATUS_UNKNOWN) {
        dominant = s.status;
      }
    }

    std::lock_guard<std::mutex> lock(state_.mtx);
    state_.last_msg_time  = now;
    state_.has_msg        = true;
    state_.current_status = dominant;

    if (aborted) {
      state_.abort_times.push_back(now);
    }

    // Remove abort samples older than the 60s rolling window.
    while (!state_.abort_times.empty() &&
           (now - state_.abort_times.front()).seconds() > 60.0)
    {
      state_.abort_times.pop_front();
    }
  }

  static const char * statusLabel(int8_t s)
  {
    switch (s) {
      case GoalStatus::STATUS_UNKNOWN:   return "UNKNOWN";
      case GoalStatus::STATUS_ACCEPTED:  return "ACCEPTED";
      case GoalStatus::STATUS_EXECUTING: return "EXECUTING";
      case GoalStatus::STATUS_CANCELING: return "CANCELING";
      case GoalStatus::STATUS_SUCCEEDED: return "SUCCEEDED";
      case GoalStatus::STATUS_CANCELED:  return "CANCELED";
      case GoalStatus::STATUS_ABORTED:   return "ABORTED";
      default:                           return "UNKNOWN";
    }
  }

  void checkNavStatus(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    if (!state_.has_msg) {
      // HH_260617: Before the first Nav2 goal, the action status topic may not
      // publish anything. Lifecycle checks cover server liveness; this checker
      // should report abort/status quality, not force ERROR while idle.
      if (idle_ok_without_status_) {
        stat.summary(DiagnosticStatus::OK, "idle (no nav status yet)");
        stat.add("topic", nav_status_topic_);
        stat.add("idle_ok_without_status", "true");
        return;
      }
      stat.summary(DiagnosticStatus::WARN, "No topic messages: " + nav_status_topic_);
      stat.add("topic", nav_status_topic_);
      return;
    }

    double elapsed = (this->now() - state_.last_msg_time).seconds();
    if (stale_timeout_ > 0.0 && elapsed > stale_timeout_) {
      // HH_260618: Nav2 action status is event-driven enough that it can stop
      // publishing after SUCCEEDED/CANCELED. Treat terminal stale as normal idle;
      // lifecycle checkers still verify server liveness, and abort history remains
      // handled below when new status samples arrive.
      if (
        terminal_status_stale_ok_ &&
        (state_.current_status == GoalStatus::STATUS_SUCCEEDED ||
         state_.current_status == GoalStatus::STATUS_CANCELED))
      {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
          "idle: %s (last status %.1fs ago)",
          statusLabel(state_.current_status), elapsed);
        stat.summary(DiagnosticStatus::OK, std::string(buf));
        stat.add("current_status", std::string(statusLabel(state_.current_status)));
        stat.add("terminal_status_stale_ok", "true");
        stat.add("last_msg_sec_ago", elapsed);
        return;
      }
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "No nav status for %.1fs (timeout=%.1fs)", elapsed, stale_timeout_);
      stat.summary(DiagnosticStatus::WARN, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      stat.add("status_freshness_critical", "false");
      return;
    }

    int abort_count = static_cast<int>(state_.abort_times.size());

    int8_t     lvl = DiagnosticStatus::OK;
    std::string msg_str;

    if (abort_count > abort_error_) {
      lvl     = DiagnosticStatus::ERROR;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "Repeated ABORTED (%d in 60s > %d)", abort_count, abort_error_);
      msg_str = buf;
    } else if (abort_count > abort_warn_) {
      lvl     = DiagnosticStatus::WARN;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "High ABORTED frequency (%d in 60s > %d)", abort_count, abort_warn_);
      msg_str = buf;
    }

    if (lvl == DiagnosticStatus::OK) {
      switch (state_.current_status) {
        case GoalStatus::STATUS_EXECUTING:
        case GoalStatus::STATUS_ACCEPTED:
          msg_str = std::string("navigation active: ") + statusLabel(state_.current_status);
          break;
        case GoalStatus::STATUS_ABORTED:
          lvl     = DiagnosticStatus::WARN;
          msg_str = "ABORTED - recent path planning failed";
          break;
        case GoalStatus::STATUS_SUCCEEDED:
        case GoalStatus::STATUS_CANCELED:
          msg_str = std::string("idle: ") + statusLabel(state_.current_status);
          break;
        case GoalStatus::STATUS_UNKNOWN:
        default:
          msg_str = "idle (no status)";
          break;
      }
    }

    stat.summary(lvl, msg_str);

    stat.add("current_status",     std::string(statusLabel(state_.current_status)));

    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%d", abort_count);
    stat.add("abort_count_60s",    std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%d / %d", abort_warn_, abort_error_);
    stat.add("abort_warn/error",   std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago",   std::string(tmp));
  }

  std::string nav_status_topic_;
  double      stale_timeout_{5.0};
  bool        idle_ok_without_status_{true};
  bool        terminal_status_stale_ok_{true};
  int         abort_warn_{2};
  int         abort_error_{5};

  NavStatusState state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningNavStatusCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
