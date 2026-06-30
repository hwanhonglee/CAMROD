#include <cstdio>
#include <deque>
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

// HH_260630 - Track Nav2 costmap freshness and publish rate with the same
// diagnostic style used by sensing/preprocessing checkers.

struct CostmapState
{
  std::string  name;
  std::string  topic;
  double       expected_hz{1.0};
  double       hz_warn_ratio{0.7};
  double       hz_error_ratio{0.4};
  double       stale_timeout{5.0};
  double       hz_window_s{6.0};

  std::mutex   mtx;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool         has_msg{false};
  uint32_t     width{0};
  uint32_t     height{0};
  double       resolution{0.0};
  std::deque<rclcpp::Time> timestamps;

  rclcpp::Subscription<OccupancyGrid>::SharedPtr sub;
};

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
    declare_parameter("global_expected_hz", 0.5);
    declare_parameter("local_expected_hz", 2.0);
    declare_parameter("hz_warn_ratio", 0.7);
    declare_parameter("hz_error_ratio", 0.4);
    declare_parameter("global_stale_timeout_s", 3.0);
    declare_parameter("local_stale_timeout_s", 1.5);
    declare_parameter("hz_window_s", 6.0);
  }

  void load_parameters_() override
  {
    global_.name          = "global";
    global_.topic         = get_parameter("global_costmap_topic").as_string();
    global_.expected_hz   = get_parameter("global_expected_hz").as_double();
    global_.hz_warn_ratio = get_parameter("hz_warn_ratio").as_double();
    global_.hz_error_ratio = get_parameter("hz_error_ratio").as_double();
    global_.stale_timeout = get_param<double>("global_stale_timeout_s", global_.stale_timeout);
    global_.hz_window_s = get_parameter("hz_window_s").as_double();

    local_.name          = "local";
    local_.topic         = get_parameter("local_costmap_topic").as_string();
    local_.expected_hz   = get_parameter("local_expected_hz").as_double();
    local_.hz_warn_ratio = get_parameter("hz_warn_ratio").as_double();
    local_.hz_error_ratio = get_parameter("hz_error_ratio").as_double();
    local_.stale_timeout = get_param<double>("local_stale_timeout_s", local_.stale_timeout);
    local_.hz_window_s = get_parameter("hz_window_s").as_double();
  }

  void setup_tasks_() override
  {
    auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    global_.sub = create_subscription<OccupancyGrid>(
      global_.topic, costmap_qos,
      [this](const OccupancyGrid::ConstSharedPtr msg) { onCostmap(msg, global_); });

    local_.sub = create_subscription<OccupancyGrid>(
      local_.topic, costmap_qos,
      [this](const OccupancyGrid::ConstSharedPtr msg) { onCostmap(msg, local_); });

    add_task("/planning/global_costmap",
      [this](StatusWrapper & stat) { checkCostmap(stat, global_); });

    add_task("/planning/local_costmap",
      [this](StatusWrapper & stat) { checkCostmap(stat, local_); });

    RCLCPP_INFO(get_logger(),
      "planning costmap monitoring: global=%s expected_hz=%.1f stale=%.1fs, "
      "local=%s expected_hz=%.1f stale=%.1fs",
      global_.topic.c_str(), global_.expected_hz, global_.stale_timeout,
      local_.topic.c_str(), local_.expected_hz, local_.stale_timeout);
  }

private:
  void onCostmap(const OccupancyGrid::ConstSharedPtr msg, CostmapState & costmap)
  {
    std::lock_guard<std::mutex> lock(costmap.mtx);
    const auto now = this->now();
    costmap.last_msg_time = now;
    costmap.has_msg = true;
    costmap.width = msg->info.width;
    costmap.height = msg->info.height;
    costmap.resolution = msg->info.resolution;
    costmap.timestamps.push_back(now);
    while (!costmap.timestamps.empty() &&
           (now - costmap.timestamps.front()).seconds() > costmap.hz_window_s)
    {
      costmap.timestamps.pop_front();
    }
  }

  void checkCostmap(StatusWrapper & stat, CostmapState & costmap)
  {
    std::lock_guard<std::mutex> lock(costmap.mtx);

    if (!costmap.has_msg) {
      stat.summary(DiagnosticStatus::STALE,
        "no topic messages: " + costmap.topic);
      stat.add("topic", costmap.topic);
      stat.add("costmap", costmap.name);
      return;
    }

    double elapsed = (this->now() - costmap.last_msg_time).seconds();
    if (elapsed > costmap.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "no costmap update for %.1fs (timeout=%.1fs)",
        elapsed, costmap.stale_timeout);
      stat.summary(DiagnosticStatus::ERROR, std::string(buf));
      char tmp[32];
      std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
      stat.add("last_msg_sec_ago", std::string(tmp));
      stat.add("topic", costmap.topic);
      stat.add("costmap", costmap.name);
      return;
    }

    double actual_hz = 0.0;
    if (costmap.timestamps.size() >= 2) {
      const double window = (costmap.timestamps.back() - costmap.timestamps.front()).seconds();
      if (window > 0.0) {
        actual_hz = static_cast<double>(costmap.timestamps.size() - 1) / window;
      }
    }

    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str = "OK";
    if (costmap.expected_hz > 0.0) {
      const double ratio = actual_hz / costmap.expected_hz;
      const int8_t hz_lvl = check_low(ratio, costmap.hz_warn_ratio, costmap.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl = hz_lvl;
        msg_str = (hz_lvl == S::ERROR) ? "publish rate critically low" : "publish rate low";
      }
    }

    if (lvl == DiagnosticStatus::OK) {
      char buf[80];
      std::snprintf(buf, sizeof(buf), "OK (%.1f Hz, %.1fs ago)", actual_hz, elapsed);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_hz);
    stat.add("actual_hz", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", costmap.expected_hz);
    stat.add("expected_hz", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago", std::string(tmp));
    stat.add("topic", costmap.topic);
    stat.add("costmap", costmap.name);
    stat.add("width", static_cast<int>(costmap.width));
    stat.add("height", static_cast<int>(costmap.height));
    std::snprintf(tmp, sizeof(tmp), "%.3f", costmap.resolution);
    stat.add("resolution_m", std::string(tmp));
  }

  CostmapState global_;
  CostmapState local_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningCostmapCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
