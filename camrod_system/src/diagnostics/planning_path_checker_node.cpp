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
 *     topic:            "/planning/global_path"
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
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/msg/goal_status.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;
using Path             = nav_msgs::msg::Path;
using GoalStatusArray  = action_msgs::msg::GoalStatusArray;
using GoalStatus       = action_msgs::msg::GoalStatus;

// ── 경로 소스별 상태 ──────────────────────────────────────────────────────

struct PathSource
{
  // 설정
  std::string name;
  std::string topic;
  double      stale_timeout{3.0};
  std::size_t min_points_warn{5};
  std::size_t min_points_error{2};

  // 런타임 (mutex 보호)
  std::mutex   mtx;
  rclcpp::Time last_path_time{0, 0, RCL_ROS_TIME};
  bool         has_path{false};
  std::size_t  point_count{0};

  rclcpp::Subscription<Path>::SharedPtr sub;
};

// ── PlanningPathCheckerNode ───────────────────────────────────────────────

class PlanningPathCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  PlanningPathCheckerNode()
  : robot_diagnostics_base::BaseChecker(
      "planning_path_checker", "planning_path_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("path_names",       std::vector<std::string>{});
    declare_parameter("nav_status_topic",
      std::string("/planning/navigate_to_pose/_action/status"));
  }

  void load_parameters_() override
  {
    path_names_       = get_parameter("path_names").as_string_array();
    nav_status_topic_ = get_parameter("nav_status_topic").as_string();

    for (const auto & name : path_names_) {
      auto src = std::make_shared<PathSource>();
      src->name = name;

      declare_parameter(name + ".topic",            std::string(""));
      declare_parameter(name + ".stale_timeout_s",    3.0);
      declare_parameter(name + ".min_points_warn",  int64_t(5));
      declare_parameter(name + ".min_points_error", int64_t(2));

      src->topic            = get_parameter(name + ".topic").as_string();
      src->stale_timeout = get_param<double>(name + ".stale_timeout_s", src->stale_timeout);
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
    nav_sub_ = create_subscription<GoalStatusArray>(
      nav_status_topic_, rclcpp::QoS(10),
      [this](const GoalStatusArray::ConstSharedPtr msg) {
        bool active = false;
        for (const auto & s : msg->status_list) {
          if (s.status == GoalStatus::STATUS_EXECUTING ||
              s.status == GoalStatus::STATUS_ACCEPTED)
          {
            active = true;
            break;
          }
        }
        std::lock_guard<std::mutex> lock(nav_mtx_);
        nav_active_ = active;
      });

    for (auto & src : sources_) {
      src->sub = create_subscription<Path>(
        src->topic, rclcpp::QoS(1).reliable(),
        [this, src](const Path::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(src->mtx);
          src->last_path_time = this->now();
          src->has_path       = true;
          src->point_count    = msg->poses.size();
        });

      const std::string diag_name = "/planning/path/" + src->name;
      add_task(diag_name,
        [this, src](StatusWrapper & stat) { checkPath(stat, *src); });

      RCLCPP_INFO(get_logger(),
        "[%s] 경로 모니터링 시작 (topic=%s, stale=%.1fs, warn=%zu, error=%zu)",
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

    std::lock_guard<std::mutex> lock(src.mtx);

    // navigation idle 이면 판정 없이 OK
    if (!nav_active) {
      stat.summary(DiagnosticStatus::OK, "navigation idle");
      stat.add("nav_active",  "false");
      stat.add("point_count", static_cast<int>(src.point_count));
      return;
    }

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!src.has_path) {
      stat.summary(DiagnosticStatus::ERROR,
        "navigation 중 경로 미수신: " + src.topic);
      stat.add("nav_active", "true");
      stat.add("topic",      src.topic);
      return;
    }

    const double elapsed = (this->now() - src.last_path_time).seconds();
    if (elapsed > src.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "navigation 중 경로 stale (%.1fs > %.1fs)",
        elapsed, src.stale_timeout);
      stat.summary(DiagnosticStatus::ERROR, std::string(buf));
      char tmp[32];
      std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
      stat.add("last_path_sec_ago", std::string(tmp));
      stat.add("nav_active",        "true");
      return;
    }

    // ── Point count 체크 ────────────────────────────────────────────────
    int8_t      lvl = DiagnosticStatus::OK;
    std::string msg_str;

    if (src.point_count < src.min_points_error) {
      lvl = DiagnosticStatus::ERROR;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "경로 포인트 부족 (%zu < %zu)",
        src.point_count, src.min_points_error);
      msg_str = buf;
    } else if (src.point_count < src.min_points_warn) {
      lvl = DiagnosticStatus::WARN;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "경로 포인트 적음 (%zu < %zu)",
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
  }

  std::vector<std::string>                  path_names_;
  std::string                               nav_status_topic_;
  std::vector<std::shared_ptr<PathSource>>  sources_;

  std::mutex                                            nav_mtx_;
  bool                                                  nav_active_{false};
  rclcpp::Subscription<GoalStatusArray>::SharedPtr      nav_sub_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningPathCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
