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
 *   service_state_topic: "/service/state"
 *                     # suppress expected Nav2 abort/cancel noise during site maneuvers
 */

#include <cstdio>
#include <deque>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <avg_msgs/msg/avg_service_state.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;
// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.

struct NavStatusState
{
  std::mutex mtx;

  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};

  int8_t current_status{action_msgs::msg::GoalStatus::STATUS_UNKNOWN};

  std::deque<rclcpp::Time> abort_times;
  int32_t latest_service_state{-1};
  bool has_service_state{false};

  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr sub;
  rclcpp::Subscription<avg_msgs::msg::AvgServiceState>::SharedPtr service_state_sub;
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
    declare_parameter("service_state_topic", std::string("/service/state"));
    declare_parameter("suppress_abort_during_service_maneuver", true);
  }

  void load_parameters_() override
  {
    nav_status_topic_ = get_parameter("nav_status_topic").as_string();
    stale_timeout_ = get_param<double>("stale_timeout_s", stale_timeout_);
    idle_ok_without_status_ = get_parameter("idle_ok_without_status").as_bool();
    terminal_status_stale_ok_ = get_parameter("terminal_status_stale_ok").as_bool();
    abort_warn_       = get_parameter("abort_warn").as_int();
    abort_error_      = get_parameter("abort_error").as_int();
    service_state_topic_ = get_parameter("service_state_topic").as_string();
    suppress_abort_during_service_maneuver_ =
      get_parameter("suppress_abort_during_service_maneuver").as_bool();
  }

  void setup_tasks_() override
  {
    state_.sub = create_subscription<action_msgs::msg::GoalStatusArray>(
      nav_status_topic_, rclcpp::QoS(10),
      [this](const action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) { onNavStatus(msg); });
    state_.service_state_sub = create_subscription<avg_msgs::msg::AvgServiceState>(
      service_state_topic_, rclcpp::QoS(10),
      [this](const avg_msgs::msg::AvgServiceState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.latest_service_state = msg->state;
        state_.has_service_state = true;
        if (serviceManeuverSuppressActiveLocked()) {
          state_.abort_times.clear();
        }
      });

    add_task("/planning/nav_status",
      [this](StatusWrapper & stat) { checkNavStatus(stat); });

    RCLCPP_INFO(get_logger(),
      "Planning nav status checker started "
      "(topic=%s, service_state=%s, stale=%.1fs, terminal_stale_ok=%s, "
      "suppress_service_maneuver=%s, abort_warn=%d, abort_error=%d)",
      nav_status_topic_.c_str(), service_state_topic_.c_str(), stale_timeout_,
      terminal_status_stale_ok_ ? "true" : "false",
      suppress_abort_during_service_maneuver_ ? "true" : "false", abort_warn_, abort_error_);
  }

private:
  void onNavStatus(const action_msgs::msg::GoalStatusArray::ConstSharedPtr msg)
  {
    auto now = this->now();

    // Select the dominant status from the status list.
    // Priority: EXECUTING > ACCEPTED > CANCELING > terminal states.
    int8_t dominant = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
    bool   aborted  = false;

    for (const auto & s : msg->status_list) {
      if (s.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        dominant = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
      } else if (s.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED &&
                 dominant != action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        dominant = action_msgs::msg::GoalStatus::STATUS_ACCEPTED;
      } else if (s.status == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
        aborted = true;
        if (dominant == action_msgs::msg::GoalStatus::STATUS_UNKNOWN) {
          dominant = action_msgs::msg::GoalStatus::STATUS_ABORTED;
        }
      } else if (dominant == action_msgs::msg::GoalStatus::STATUS_UNKNOWN) {
        dominant = s.status;
      }
    }

    std::lock_guard<std::mutex> lock(state_.mtx);
    state_.last_msg_time  = now;
    state_.has_msg        = true;
    state_.current_status = dominant;

    if (aborted) {
      if (serviceManeuverSuppressActiveLocked()) {
        state_.abort_times.clear();
      } else {
        state_.abort_times.push_back(now);
      }
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
      case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:   return "UNKNOWN";
      case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:  return "ACCEPTED";
      case action_msgs::msg::GoalStatus::STATUS_EXECUTING: return "EXECUTING";
      case action_msgs::msg::GoalStatus::STATUS_CANCELING: return "CANCELING";
      case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED: return "SUCCEEDED";
      case action_msgs::msg::GoalStatus::STATUS_CANCELED:  return "CANCELED";
      case action_msgs::msg::GoalStatus::STATUS_ABORTED:   return "ABORTED";
      default:                           return "UNKNOWN";
    }
  }

  bool serviceManeuverSuppressActiveLocked() const
  {
    if (!suppress_abort_during_service_maneuver_ || !state_.has_service_state) {
      return false;
    }
    switch (state_.latest_service_state) {
      case avg_msgs::msg::AvgServiceState::SITE_ENTRY:
      case avg_msgs::msg::AvgServiceState::UNLOAD_WAIT:
      case avg_msgs::msg::AvgServiceState::GUEST_LOADING_WAIT:
      case avg_msgs::msg::AvgServiceState::WAITING_FOR_RETURN_REQUEST:
      case avg_msgs::msg::AvgServiceState::RETURN_WITH_CARGO:
      case avg_msgs::msg::AvgServiceState::DROP_ZONE_PARKING:
      case avg_msgs::msg::AvgServiceState::WAITING_FOR_CHARGING:
      case avg_msgs::msg::AvgServiceState::OPERATOR_STOPPED:
        return true;
      default:
        return false;
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
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "idle (no nav status yet)");
        stat.add("topic", nav_status_topic_);
        stat.add("idle_ok_without_status", "true");
        return;
      }
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No topic messages: " + nav_status_topic_);
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
        (state_.current_status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED ||
         state_.current_status == action_msgs::msg::GoalStatus::STATUS_CANCELED))
      {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
          "idle: %s (last status %.1fs ago)",
          statusLabel(state_.current_status), elapsed);
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, std::string(buf));
        stat.add("current_status", std::string(statusLabel(state_.current_status)));
        stat.add("terminal_status_stale_ok", "true");
        stat.add("last_msg_sec_ago", elapsed);
        return;
      }
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "No nav status for %.1fs (timeout=%.1fs)", elapsed, stale_timeout_);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      stat.add("status_freshness_critical", "false");
      return;
    }

    const bool service_maneuver_suppressed = serviceManeuverSuppressActiveLocked();
    int abort_count = static_cast<int>(state_.abort_times.size());
    int effective_abort_count = service_maneuver_suppressed ? 0 : abort_count;

    int8_t     lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg_str;

    if (effective_abort_count > abort_error_) {
      lvl     = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "Repeated ABORTED (%d in 60s > %d)", effective_abort_count, abort_error_);
      msg_str = buf;
    } else if (effective_abort_count > abort_warn_) {
      lvl     = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "High ABORTED frequency (%d in 60s > %d)", effective_abort_count, abort_warn_);
      msg_str = buf;
    }

    if (lvl == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      switch (state_.current_status) {
        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
          msg_str = std::string("navigation active: ") + statusLabel(state_.current_status);
          break;
        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
          if (service_maneuver_suppressed) {
            msg_str = "service maneuver owns motion; Nav2 abort/cancel ignored";
          } else {
            lvl     = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            msg_str = "ABORTED - recent path planning failed";
          }
          break;
        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
          msg_str = std::string("idle: ") + statusLabel(state_.current_status);
          break;
        case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
        default:
          msg_str = "idle (no status)";
          break;
      }
    }

    stat.summary(lvl, msg_str);

    stat.add("current_status",     std::string(statusLabel(state_.current_status)));
    stat.add("service_maneuver_suppressed", service_maneuver_suppressed ? "true" : "false");
    stat.add("latest_service_state", static_cast<int>(state_.latest_service_state));

    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%d", abort_count);
    stat.add("abort_count_60s",    std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%d", effective_abort_count);
    stat.add("effective_abort_count_60s", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%d / %d", abort_warn_, abort_error_);
    stat.add("abort_warn/error",   std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago",   std::string(tmp));
  }

  std::string nav_status_topic_;
  std::string service_state_topic_;
  double      stale_timeout_{5.0};
  bool        idle_ok_without_status_{true};
  bool        terminal_status_stale_ok_{true};
  bool        suppress_abort_during_service_maneuver_{true};
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
