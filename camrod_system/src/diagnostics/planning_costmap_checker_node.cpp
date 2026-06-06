/**
 * Planning Costmap Checker Node
 *
 * Nav2 global/local costmap 토픽의 staleness 를 /diagnostics 로 발행한다.
 *
 * 진단 항목
 * ---------
 *   /planning/global_costmap
 *     - Staleness : global costmap 미갱신 경과 시간
 *
 *   /planning/local_costmap
 *     - Staleness : local costmap 미갱신 경과 시간
 *
 * 파라미터 구성
 * -------------
 *   global_costmap_topic:  "/planning/global_costmap/costmap"
 *   local_costmap_topic:   "/planning/local_costmap/costmap"
 *   stale_timeout:         5.0   # 이 시간(초) 이상 미갱신이면 ERROR
 */

#include <cstdio>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;
using OccupancyGrid    = nav_msgs::msg::OccupancyGrid;

// ── Costmap 상태 구조체 ───────────────────────────────────────────────────

struct CostmapState
{
  std::string  name;
  std::string  topic;
  double       stale_timeout{5.0};

  std::mutex   mtx;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool         has_msg{false};

  rclcpp::Subscription<OccupancyGrid>::SharedPtr sub;
};

// ── PlanningCostmapCheckerNode ────────────────────────────────────────────

class PlanningCostmapCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  PlanningCostmapCheckerNode()
  : robot_diagnostics_base::BaseChecker(
      "planning_costmap_checker", "planning_costmap_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("global_costmap_topic",
      std::string("/planning/global_costmap/costmap"));
    declare_parameter("local_costmap_topic",
      std::string("/planning/local_costmap/costmap"));
    declare_parameter("global_stale_timeout_s", 3.0);
    declare_parameter("local_stale_timeout_s",  1.0);
  }

  void load_parameters_() override
  {
    global_.name          = "global";
    global_.topic         = get_parameter("global_costmap_topic").as_string();
    global_.stale_timeout = get_param<double>("global_stale_timeout_s", global_.stale_timeout);

    local_.name          = "local";
    local_.topic         = get_parameter("local_costmap_topic").as_string();
    local_.stale_timeout = get_param<double>("local_stale_timeout_s", local_.stale_timeout);
  }

  void setup_tasks_() override
  {
    // global costmap — transient_local QoS (Nav2 기본값)
    auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    global_.sub = create_subscription<OccupancyGrid>(
      global_.topic, costmap_qos,
      [this](const OccupancyGrid::ConstSharedPtr /*msg*/) {
        std::lock_guard<std::mutex> lock(global_.mtx);
        global_.last_msg_time = this->now();
        global_.has_msg       = true;
      });

    local_.sub = create_subscription<OccupancyGrid>(
      local_.topic, costmap_qos,
      [this](const OccupancyGrid::ConstSharedPtr /*msg*/) {
        std::lock_guard<std::mutex> lock(local_.mtx);
        local_.last_msg_time = this->now();
        local_.has_msg       = true;
      });

    add_task("/planning/global_costmap",
      [this](StatusWrapper & stat) { checkCostmap(stat, global_); });

    add_task("/planning/local_costmap",
      [this](StatusWrapper & stat) { checkCostmap(stat, local_); });

    RCLCPP_INFO(get_logger(),
      "Planning Costmap 모니터링 시작 "
      "(global=%s stale=%.1fs, local=%s stale=%.1fs)",
      global_.topic.c_str(), global_.stale_timeout,
      local_.topic.c_str(),  local_.stale_timeout);
  }

private:
  void checkCostmap(StatusWrapper & stat, CostmapState & costmap)
  {
    std::lock_guard<std::mutex> lock(costmap.mtx);

    if (!costmap.has_msg) {
      stat.summary(DiagnosticStatus::STALE,
        "토픽 수신 없음: " + costmap.topic);
      stat.add("topic", costmap.topic);
      return;
    }

    double elapsed = (this->now() - costmap.last_msg_time).seconds();
    if (elapsed > costmap.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "%.1fs 동안 costmap 미갱신 (timeout=%.1fs)",
        elapsed, costmap.stale_timeout);
      stat.summary(DiagnosticStatus::ERROR, std::string(buf));
      char tmp[32];
      std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
      stat.add("last_msg_sec_ago", std::string(tmp));
      return;
    }

    char buf[64];
    std::snprintf(buf, sizeof(buf), "OK (%.1fs ago)", elapsed);
    stat.summary(DiagnosticStatus::OK, std::string(buf));
    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago", std::string(tmp));
  }

  CostmapState global_;
  CostmapState local_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningCostmapCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
