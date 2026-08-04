#include <cstdio>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// HH_260720 - Diagnose generated CAMROD sensing cost grids.
#include <avg_msgs/msg/avg_occupancy_grid.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// HH_260630 - Cost-grid checker covers sensor-derived and merged grid outputs.

class CostGridCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  explicit CostGridCheckerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : robot_diagnostics_base::BaseChecker("cost_grid_checker", "cost_grid_checker", options)
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("grid_names", std::vector<std::string>{});

    // Legacy single-grid parameters remain supported for older profiles.
    declare_parameter("diagnostic_name", std::string("/sensing/cost_grid/lidar"));
    declare_parameter("output_topic", std::string("/sensing/cost_grid/lidar"));
    declare_parameter("expected_hz", 10.0);
    declare_parameter("hz_warn_ratio", 0.7);
    declare_parameter("hz_error_ratio", 0.4);
    declare_parameter("stale_timeout_s", 2.0);
    declare_parameter("unknown_ratio_warn", 0.9);
    declare_parameter("unknown_ratio_error", 1.0);
  }

  void load_parameters_() override
  {
    auto names = get_parameter("grid_names").as_string_array();

    if (names.empty()) {
      auto grid = std::make_shared<GridState>();
      grid->name = "default";
      grid->diagnostic_name = get_parameter("diagnostic_name").as_string();
      grid->output_topic = get_parameter("output_topic").as_string();
      grid->expected_hz = get_parameter("expected_hz").as_double();
      grid->hz_warn_ratio = get_parameter("hz_warn_ratio").as_double();
      grid->hz_error_ratio = get_parameter("hz_error_ratio").as_double();
      grid->stale_timeout = get_param<double>("stale_timeout_s", grid->stale_timeout);
      grid->unknown_ratio_warn = get_parameter("unknown_ratio_warn").as_double();
      grid->unknown_ratio_error = get_parameter("unknown_ratio_error").as_double();
      grids_.push_back(grid);
      return;
    }

    for (const auto & name : names) {
      auto grid = std::make_shared<GridState>();
      grid->name = name;

      declare_parameter(name + ".diagnostic_name", "/sensing/cost_grid/" + name);
      declare_parameter(name + ".output_topic", "/sensing/cost_grid/" + name);
      declare_parameter(name + ".expected_hz", get_parameter("expected_hz").as_double());
      declare_parameter(name + ".hz_warn_ratio", get_parameter("hz_warn_ratio").as_double());
      declare_parameter(name + ".hz_error_ratio", get_parameter("hz_error_ratio").as_double());
      declare_parameter(name + ".stale_timeout_s", get_parameter("stale_timeout_s").as_double());
      declare_parameter(name + ".unknown_ratio_warn", get_parameter("unknown_ratio_warn").as_double());
      declare_parameter(name + ".unknown_ratio_error", get_parameter("unknown_ratio_error").as_double());

      grid->diagnostic_name = get_parameter(name + ".diagnostic_name").as_string();
      grid->output_topic = get_parameter(name + ".output_topic").as_string();
      grid->expected_hz = get_parameter(name + ".expected_hz").as_double();
      grid->hz_warn_ratio = get_parameter(name + ".hz_warn_ratio").as_double();
      grid->hz_error_ratio = get_parameter(name + ".hz_error_ratio").as_double();
      grid->stale_timeout = get_param<double>(name + ".stale_timeout_s", grid->stale_timeout);
      grid->unknown_ratio_warn = get_parameter(name + ".unknown_ratio_warn").as_double();
      grid->unknown_ratio_error = get_parameter(name + ".unknown_ratio_error").as_double();

      grids_.push_back(grid);
    }
  }

  void setup_tasks_() override
  {
    for (auto & grid : grids_) {
      // HH_260630 - Use volatile/reliable subscription so it connects to both
      // volatile path grids and transient-local sensor/merged grid publishers.
      grid->sub = create_subscription<avg_msgs::msg::AvgOccupancyGrid>(
        grid->output_topic, rclcpp::QoS(10).reliable(),
        [this, grid](const avg_msgs::msg::AvgOccupancyGrid::ConstSharedPtr msg) {
          onGrid(msg, grid);
        });

      add_task(grid->diagnostic_name,
        [this, grid](StatusWrapper & stat) { checkGrid(stat, *grid); });

      RCLCPP_INFO(get_logger(),
        "cost_grid monitoring: name=%s diagnostic=%s topic=%s expected_hz=%.1f",
        grid->name.c_str(), grid->diagnostic_name.c_str(),
        grid->output_topic.c_str(), grid->expected_hz);
    }
  }

private:
  struct GridState
  {
    std::string name;
    std::string diagnostic_name;
    std::string output_topic;
    double expected_hz{10.0};
    double hz_warn_ratio{0.7};
    double hz_error_ratio{0.4};
    double stale_timeout{2.0};
    double unknown_ratio_warn{0.9};
    double unknown_ratio_error{1.0};

    std::mutex mtx;
    rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
    bool has_msg{false};
    double actual_unknown_ratio{1.0};
    uint32_t width{0};
    uint32_t height{0};
    double resolution{0.0};
    std::deque<rclcpp::Time> timestamps;
    rclcpp::Subscription<avg_msgs::msg::AvgOccupancyGrid>::SharedPtr sub;
  };

  void onGrid(
    const avg_msgs::msg::AvgOccupancyGrid::ConstSharedPtr msg,
    const std::shared_ptr<GridState> & grid)
  {
    std::lock_guard<std::mutex> lock(grid->mtx);
    auto now = this->now();
    grid->last_msg_time = now;
    grid->has_msg = true;
    grid->width = msg->info.width;
    grid->height = msg->info.height;
    grid->resolution = msg->info.resolution;

    const auto total = msg->data.size();
    if (total > 0) {
      size_t unknown_count = 0;
      for (const auto & cell : msg->data) {
          if (cell < 0) ++unknown_count;
      }
      grid->actual_unknown_ratio =
        static_cast<double>(unknown_count) / static_cast<double>(total);
    } else {
      grid->actual_unknown_ratio = 1.0;
    }

    grid->timestamps.push_back(now);
    while (!grid->timestamps.empty() &&
           (now - grid->timestamps.front()).seconds() > 2.0)
    {
      grid->timestamps.pop_front();
    }
  }

  void checkGrid(StatusWrapper & stat, GridState & grid)
  {
    std::lock_guard<std::mutex> lock(grid.mtx);

    if (!grid.has_msg) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "no topic messages: " + grid.output_topic);
      stat.add("topic", grid.output_topic);
      stat.add("grid_name", grid.name);
      return;
    }

    double elapsed = (this->now() - grid.last_msg_time).seconds();
    if (elapsed > grid.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "no messages for %.1fs (timeout=%.1fs)", elapsed, grid.stale_timeout);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      stat.add("topic", grid.output_topic);
      stat.add("grid_name", grid.name);
      return;
    }

    double actual_hz = 0.0;
    if (grid.timestamps.size() >= 2) {
      double window = (grid.timestamps.back() - grid.timestamps.front()).seconds();
      if (window > 0.0) {
        actual_hz = static_cast<double>(grid.timestamps.size() - 1) / window;
      }
    }

    int8_t lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg_str = "OK";

    if (grid.expected_hz > 0.0) {
      double ratio = actual_hz / grid.expected_hz;
      int8_t hz_lvl = check_low(ratio, grid.hz_warn_ratio, grid.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl = hz_lvl;
        msg_str = (hz_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) ? "publish rate critically low" : "publish rate low";
      }
    }

    int8_t unk_lvl = check_high(
      grid.actual_unknown_ratio, grid.unknown_ratio_warn, grid.unknown_ratio_error);
    if (unk_lvl > lvl) {
      lvl = unk_lvl;
      msg_str = (unk_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) ? "grid fully unknown" : "grid unknown ratio high";
    }

    if (lvl == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "OK (%.1f Hz, unknown=%.0f%%)",
        actual_hz, grid.actual_unknown_ratio * 100.0);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_hz);
    stat.add("actual_hz", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", grid.expected_hz);
    stat.add("expected_hz", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", grid.actual_unknown_ratio * 100.0);
    stat.add("unknown_ratio_pct", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago", std::string(tmp));
    stat.add("topic", grid.output_topic);
    stat.add("grid_name", grid.name);
    stat.add("width", static_cast<int>(grid.width));
    stat.add("height", static_cast<int>(grid.height));
    std::snprintf(tmp, sizeof(tmp), "%.3f", grid.resolution);
    stat.add("resolution_m", std::string(tmp));
  }

  std::vector<std::shared_ptr<GridState>> grids_;
};

#include "camrod_system/checker_entrypoint.hpp"
CAMROD_SYSTEM_CHECKER_ENTRYPOINT(CostGridCheckerNode)
