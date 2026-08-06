/**
 * Planning Path Checker Node
 *
 * navigation 실행 중 경로 토픽들(global/local)의 품질을 /diagnostics 로 발행한다.
 * path_names 리스트로 경로 소스를 동적으로 추가할 수 있다.
 *
 * navigation idle 상태에서는 staleness/point count 판정을 하지 않는다.
 *
 * 진단 항목 (소스별)
 * -----------------
 *   - Staleness    : navigation 중 path 미갱신 경과 시간
 *   - Point count  : poses.size() < min_points_warn  → WARN
 *                    poses.size() < min_points_error → ERROR
 *
 * 파라미터 구성 예시
 * -----------------
 *   path_names: ["global_path", "local_path"]
 *   nav_status_topic: "/planning/navigate_to_pose/_action/status"
 *
 *   global_path:
 *     topic:            "/planning/global_path_avg"
 *     stale_timeout:    3.0
 *     min_points_warn:  5
 *     min_points_error: 2
 *
 *   local_path:
 *     topic:            "/planning/local_path"
 *     stale_timeout:    2.0
 *     min_points_warn:  8
 *     min_points_error: 3
 */

#include <cstdio>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// HH_260720 - Diagnose generated CAMROD global/local path mirrors.
#include <avg_msgs/msg/avg_path.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <avg_msgs/msg/avg_service_state.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;
// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.

// ── 경로 소스별 상태 ──────────────────────────────────────────────────────

struct PathSource
{
  // 설정
  std::string name;
  std::string topic;
  double      stale_timeout{3.0};
  double      point_count_grace{0.0};
  std::size_t min_points_warn{5};
  std::size_t min_points_error{2};

  // 런타임 (mutex 보호)
  std::mutex   mtx;
  rclcpp::Time last_path_time{0, 0, RCL_ROS_TIME};
  bool         has_path{false};
  std::size_t  point_count{0};
  bool         low_point_count_active{false};
  rclcpp::Time low_point_count_since{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<avg_msgs::msg::AvgPath>::SharedPtr sub;
};

// ── PlanningPathCheckerNode ───────────────────────────────────────────────

class PlanningPathCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  explicit PlanningPathCheckerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : robot_diagnostics_base::BaseChecker(
      "planning_path_checker", "planning_path_checker", options)
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("path_names",       std::vector<std::string>{});
    declare_parameter("nav_status_topic",
      std::string("/planning/navigate_to_pose/_action/status"));
    declare_parameter("service_state_topic", std::string("/service/state"));
    // HH_260807 - A local maneuver or stationary service phase intentionally
    // clears Nav2's local path. Do not report that ownership handoff as a fault.
    declare_parameter(
      "local_path_suppressed_service_states",
      std::vector<int64_t>{0, 2, 5, 6, 8, 10, 11, 12, 13, 14, 15, 16});
  }

  void load_parameters_() override
  {
    path_names_       = get_parameter("path_names").as_string_array();
    nav_status_topic_ = get_parameter("nav_status_topic").as_string();
    service_state_topic_ = get_parameter("service_state_topic").as_string();
    for (const auto state :
      get_parameter("local_path_suppressed_service_states").as_integer_array())
    {
      local_path_suppressed_service_states_.insert(static_cast<int32_t>(state));
    }

    for (const auto & name : path_names_) {
      auto src = std::make_shared<PathSource>();
      src->name = name;

      declare_parameter(name + ".topic",            std::string(""));
      declare_parameter(name + ".stale_timeout_s",    3.0);
      declare_parameter(name + ".point_count_grace_s", 0.0);
      declare_parameter(name + ".min_points_warn",  int64_t(5));
      declare_parameter(name + ".min_points_error", int64_t(2));

      src->topic            = get_parameter(name + ".topic").as_string();
      src->stale_timeout = get_param<double>(name + ".stale_timeout_s", src->stale_timeout);
      src->point_count_grace = get_param<double>(
        name + ".point_count_grace_s", src->point_count_grace);
      src->min_points_warn  = static_cast<std::size_t>(
        get_parameter(name + ".min_points_warn").as_int());
      src->min_points_error = static_cast<std::size_t>(
        get_parameter(name + ".min_points_error").as_int());

      sources_.push_back(src);
    }
  }

  void setup_tasks_() override
  {
    // nav status 구독 (모든 경로 소스 공용)
    nav_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      nav_status_topic_, rclcpp::QoS(10),
      [this](const action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) {
        bool active = false;
        for (const auto & s : msg->status_list) {
          if (s.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
              s.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED)
          {
            active = true;
            break;
          }
        }
        std::lock_guard<std::mutex> lock(nav_mtx_);
        nav_active_ = active;
      });

    service_state_sub_ = create_subscription<avg_msgs::msg::AvgServiceState>(
      service_state_topic_, rclcpp::QoS(10),
      [this](const avg_msgs::msg::AvgServiceState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(service_state_mtx_);
        service_state_ = msg->state;
        has_service_state_ = true;
      });

    for (auto & src : sources_) {
      src->sub = create_subscription<avg_msgs::msg::AvgPath>(
        src->topic, rclcpp::QoS(1).reliable(),
        [this, src](const avg_msgs::msg::AvgPath::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(src->mtx);
          src->last_path_time = this->now();
          src->has_path       = true;
          src->point_count    = msg->poses.size();
          if (src->point_count < src->min_points_warn) {
            if (!src->low_point_count_active) {
              src->low_point_count_active = true;
              src->low_point_count_since = src->last_path_time;
            }
          } else {
            src->low_point_count_active = false;
          }
        });

      const std::string diag_name = "/planning/path/" + src->name;
      add_task(diag_name,
        [this, src](StatusWrapper & stat) { checkPath(stat, *src); });

      RCLCPP_INFO(get_logger(),
        "[%s] Path checker started (topic=%s, stale=%.1fs, warn=%zu, error=%zu)",
        src->name.c_str(), src->topic.c_str(),
        src->stale_timeout, src->min_points_warn, src->min_points_error);
    }
  }

private:
  void checkPath(StatusWrapper & stat, PathSource & src)
  {
    bool nav_active;
    {
      std::lock_guard<std::mutex> lock(nav_mtx_);
      nav_active = nav_active_;
    }

    bool has_service_state;
    int32_t service_state;
    {
      std::lock_guard<std::mutex> lock(service_state_mtx_);
      has_service_state = has_service_state_;
      service_state = service_state_;
    }

    std::lock_guard<std::mutex> lock(src.mtx);

    // navigation idle 이면 판정 없이 OK
    if (!nav_active) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "navigation idle");
      stat.add("nav_active",  "false");
      stat.add("point_count", static_cast<int>(src.point_count));
      return;
    }

    // HH_260807 - Nav2 may retain an EXECUTING status for one diagnostic tick
    // after a campsite/drop-zone controller takes ownership and clears the
    // local path. The service phase is authoritative during that handoff.
    if (
      src.name == "local_path" && has_service_state &&
      local_path_suppressed_service_states_.count(service_state) != 0U)
    {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "service state does not require a Nav2 local path");
      stat.add("nav_active", "true");
      stat.add("service_state", service_state);
      stat.add("point_count", static_cast<int>(src.point_count));
      return;
    }

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!src.has_path) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "No path while navigating: " + src.topic);
      stat.add("nav_active", "true");
      stat.add("topic",      src.topic);
      return;
    }

    const double elapsed = (this->now() - src.last_path_time).seconds();
    // HH_260702 - A non-positive timeout disables staleness for event-style
    // HH_260720 - Event-style paths such as /planning/global_path_avg do not expire.
    // Point-count checks still run.
    if (src.stale_timeout > 0.0 && elapsed > src.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "Path stale while navigating (%.1fs > %.1fs)",
        elapsed, src.stale_timeout);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string(buf));
      char tmp[32];
      std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
      stat.add("last_path_sec_ago", std::string(tmp));
      stat.add("nav_active",        "true");
      return;
    }

    // HH_260807 - Service state and Nav2 terminal callbacks can be queued in
    // either order in a serialized component container. Give the local path a
    // bounded transition window; a persistent low-count route still faults.
    const double low_point_count_elapsed = src.low_point_count_active ?
      (this->now() - src.low_point_count_since).seconds() : 0.0;
    if (
      src.low_point_count_active && src.point_count_grace > 0.0 &&
      low_point_count_elapsed < src.point_count_grace)
    {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "path point count transition grace");
      stat.add("point_count", static_cast<int>(src.point_count));
      stat.add("point_count_grace_s", src.point_count_grace);
      stat.add("low_point_count_elapsed_s", low_point_count_elapsed);
      stat.add("nav_active", "true");
      return;
    }

    // ── Point count 체크 ────────────────────────────────────────────────
    int8_t      lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg_str;

    if (src.point_count < src.min_points_error) {
      lvl = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "Path point count too low (%zu < %zu)",
        src.point_count, src.min_points_error);
      msg_str = buf;
    } else if (src.point_count < src.min_points_warn) {
      lvl = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "Path point count low (%zu < %zu)",
        src.point_count, src.min_points_warn);
      msg_str = buf;
    } else {
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "OK (%zu points, %.1fs ago)",
        src.point_count, elapsed);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%d", static_cast<int>(src.point_count));
    stat.add("point_count",      std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%d", static_cast<int>(src.min_points_warn));
    stat.add("min_points_warn",  std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%d", static_cast<int>(src.min_points_error));
    stat.add("min_points_error", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_path_sec_ago", std::string(tmp));
    stat.add("nav_active",        "true");
    stat.add("point_count_grace_s", src.point_count_grace);
    stat.add("low_point_count_elapsed_s", low_point_count_elapsed);
  }

  std::vector<std::string>                  path_names_;
  std::string                               nav_status_topic_;
  std::string                               service_state_topic_;
  std::unordered_set<int32_t>               local_path_suppressed_service_states_;
  std::vector<std::shared_ptr<PathSource>>  sources_;

  std::mutex                                            nav_mtx_;
  bool                                                  nav_active_{false};
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr      nav_sub_;
  std::mutex                                            service_state_mtx_;
  bool                                                  has_service_state_{false};
  int32_t                                               service_state_{-1};
  rclcpp::Subscription<avg_msgs::msg::AvgServiceState>::SharedPtr
    service_state_sub_;
};

#include "camrod_system/checker_entrypoint.hpp"
CAMROD_SYSTEM_CHECKER_ENTRYPOINT(PlanningPathCheckerNode)
