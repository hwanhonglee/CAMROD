#include <chrono>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "camrod_system/diagnostic_detail.hpp"

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using SteadyClock      = std::chrono::steady_clock;
using TimePoint        = std::chrono::steady_clock::time_point;

// ── 유틸리티 ──────────────────────────────────────────────────────────────

static const std::map<uint8_t, std::string> LEVEL_NAMES = {
  {diagnostic_msgs::msg::DiagnosticStatus::OK,    "OK"},
  {diagnostic_msgs::msg::DiagnosticStatus::WARN,  "WARN"},
  {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "ERROR"},
  {diagnostic_msgs::msg::DiagnosticStatus::STALE, "STALE"},
};

static std::string level_name(uint8_t lvl)
{
  auto it = LEVEL_NAMES.find(lvl);
  return it != LEVEL_NAMES.end() ? it->second : "?";
}

// ── TopicConfig ───────────────────────────────────────────────────────────

struct TopicConfig {
  std::string name;
  std::string group;
  double      timeout_s{5.0};
  // HH_260728 - The registry owns physical identity so every checker, stale
  // conversion, UI detail, and SYSTEM log uses one consistent mount label.
  std::string component_id;
  std::string location;
  std::string frame_id;
  std::string mount_xyz_m;
  std::string mount_rpy_deg;
  std::string pose_verified;
};

// ── DiagnosticsAggregator ─────────────────────────────────────────────────

class DiagnosticsAggregator : public rclcpp::Node
{
public:
  DiagnosticsAggregator()
  : rclcpp::Node("diagnostics_agg")
  {
    declare_parameter("config_file", std::string(""));
    // HH_260408: Keep aggregated status in topic by default, without periodic console spam.
    declare_parameter("enable_summary_log", false);
    std::string config_path = get_parameter("config_file").as_string();
    enable_summary_log_ = get_parameter("enable_summary_log").as_bool();

    double publish_rate_hz = 1.0;

    if (!config_path.empty()) {
      publish_rate_hz = load_config(config_path);
    } else {
      RCLCPP_WARN(get_logger(), "config_file parameter is not set. Using defaults.");
    }

    // HH_260617: Use relative diagnostics topics so the system namespace owns the
    // public `/system/diagnostics*` API instead of relying on absolute-topic remaps.
    sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "diagnostics", 10,
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) { diagnostics_callback(msg); });

    pub_   = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics_agg", 10);
    timer_ = create_timer(
      this, get_clock(),
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      [this]() { publish_aggregated(); });

    RCLCPP_INFO(
      get_logger(),
      "DiagnosticsAggregator started. monitored topics: %zu, default timeout: %.1fs",
      topic_configs_.size(), default_timeout_);
  }

private:
  double load_config(const std::string & config_path)
  {
    std::ifstream f(config_path);
    if (!f.good()) {
      RCLCPP_ERROR(get_logger(), "config file not found: %s", config_path.c_str());
      return 1.0;
    }

    YAML::Node cfg = YAML::LoadFile(config_path);

    double publish_rate_hz = 1.0;
    if (cfg["global"]) {
      auto g = cfg["global"];
      if (g["timeout_s"])  default_timeout_ = g["timeout_s"].as<double>();
      if (g["publish_rate_hz"]) {
        publish_rate_hz = g["publish_rate_hz"].as<double>();
      }
      if (g["ignored_names"]) {
        for (const auto & name : g["ignored_names"]) {
          ignored_names_.insert(name.as<std::string>());
        }
      }
    }

    if (cfg["topics"]) {
      for (const auto & t : cfg["topics"]) {
        TopicConfig tc;
        tc.name        = t["name"].as<std::string>();
        tc.group       = t["group"] ? t["group"].as<std::string>() : "unknown";
        tc.timeout_s = t["timeout_s"] ? t["timeout_s"].as<double>() : default_timeout_;
        if (t["metadata"]) {
          const auto metadata = t["metadata"];
          tc.component_id =
            metadata["component_id"] ? metadata["component_id"].as<std::string>() : "";
          tc.location =
            metadata["location"] ? metadata["location"].as<std::string>() : "";
          tc.frame_id =
            metadata["frame_id"] ? metadata["frame_id"].as<std::string>() : "";
          tc.mount_xyz_m =
            metadata["mount_xyz_m"] ? metadata["mount_xyz_m"].as<std::string>() : "";
          tc.mount_rpy_deg =
            metadata["mount_rpy_deg"] ? metadata["mount_rpy_deg"].as<std::string>() : "";
          tc.pose_verified =
            metadata["pose_verified"] ? metadata["pose_verified"].as<std::string>() : "";
        }
        topic_configs_[tc.name] = tc;
      }
    }

    RCLCPP_INFO(get_logger(), "config loaded: %s", config_path.c_str());
    // HH_260617: Drop explicitly ignored diagnostics from the state-machine
    // aggregate stream. This prevents planning/system summary self-loops while
    // keeping raw checker statuses available on /system/diagnostics.
    for (const auto & name : ignored_names_) {
      RCLCPP_INFO(get_logger(), "  [ignored] %s", name.c_str());
    }
    for (const auto & [name, tc] : topic_configs_) {
      RCLCPP_INFO(get_logger(), "  [%s] %s  timeout=%.1fs",
        tc.group.c_str(), name.c_str(), tc.timeout_s);
    }

    if (publish_rate_hz <= 1e-6) {
      RCLCPP_WARN(get_logger(), "publish_rate_hz must be > 0. Clamping to 1.0 Hz.");
      publish_rate_hz = 1.0;
    }
    return publish_rate_hz;
  }

  void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    auto now = SteadyClock::now();
    for (const auto & status : msg->status) {
      const std::string key = !status.name.empty() ? status.name : status.hardware_id;
      if (key.empty()) {
        continue;
      }
      if (ignored_names_.find(key) != ignored_names_.end()) {
        status_map_.erase(key);
        continue;
      }
      if (!topic_configs_.empty() && topic_configs_.find(key) == topic_configs_.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "Received diagnostic item not listed in config: \"%s\"; routing to unknown group",
          key.c_str());
      }
      status_map_[key] = {status, now};
    }
  }

  double get_timeout(const std::string & name) const
  {
    auto it = topic_configs_.find(name);
    return it != topic_configs_.end() ? it->second.timeout_s : default_timeout_;
  }

  std::string get_group(const std::string & name) const
  {
    auto it = topic_configs_.find(name);
    return it != topic_configs_.end() ? it->second.group : "unknown";
  }

  diagnostic_msgs::msg::DiagnosticStatus check_stale(const std::string & name, const diagnostic_msgs::msg::DiagnosticStatus & status,
    TimePoint last_seen)
  {
    double elapsed =
      std::chrono::duration<double>(SteadyClock::now() - last_seen).count();

    if (elapsed > get_timeout(name)) {
      // HH_260728 - Preserve checker values and registry mount metadata when a
      // stream becomes stale; losing them made the fault location disappear
      // at the exact moment an operator needed it.
      auto stale = status;
      stale.level       = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      std::ostringstream oss;
      oss << "STALE (last seen " << std::fixed;
      oss.precision(1);
      oss << elapsed << "s ago)";
      stale.message = oss.str();
      return stale;
    }
    return status;
  }

  void add_registry_metadata(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & name) const
  {
    const auto config = topic_configs_.find(name);
    if (config == topic_configs_.end()) {
      return;
    }
    const auto & topic = config->second;
    camrod_system::diagnostic_detail::upsertValue(
      status, "component_id", topic.component_id);
    camrod_system::diagnostic_detail::upsertValue(
      status, "sensor_location", topic.location);
    camrod_system::diagnostic_detail::upsertValue(
      status, "sensor_frame", topic.frame_id);
    camrod_system::diagnostic_detail::upsertValue(
      status, "mount_xyz_m", topic.mount_xyz_m);
    camrod_system::diagnostic_detail::upsertValue(
      status, "mount_rpy_deg", topic.mount_rpy_deg);
    camrod_system::diagnostic_detail::upsertValue(
      status, "pose_verified", topic.pose_verified);
  }

  void publish_aggregated()
  {
    if (status_map_.empty()) {
      return;
    }

    diagnostic_msgs::msg::DiagnosticArray agg_msg;
    agg_msg.header.stamp = get_clock()->now();

    std::map<std::string, uint8_t> group_worst;

    for (auto & [name, entry] : status_map_) {
      diagnostic_msgs::msg::DiagnosticStatus s = check_stale(name, entry.status, entry.last_seen);
      add_registry_metadata(s, name);
      agg_msg.status.push_back(s);

      std::string group = get_group(s.name);
      auto it = group_worst.find(group);
      if (it == group_worst.end()) {
        group_worst[group] = static_cast<uint8_t>(s.level);
      } else {
        it->second = static_cast<uint8_t>(std::max<unsigned>(it->second, s.level));
      }
    }

    pub_->publish(agg_msg);

    if (enable_summary_log_) {
      // 그룹별 요약 로그 (optional)
      std::ostringstream summary;
      bool first = true;
      for (const auto & [g, lvl] : group_worst) {
        if (!first) summary << " | ";
        summary << g << "=" << level_name(lvl);
        first = false;
      }
      RCLCPP_INFO(get_logger(), "[AGG] total=%zu | %s",
        agg_msg.status.size(), summary.str().c_str());
    }
  }

  // 상태 엔트리
  struct StatusEntry {
    diagnostic_msgs::msg::DiagnosticStatus status;
    TimePoint        last_seen;
  };

  std::unordered_map<std::string, TopicConfig>    topic_configs_;
  std::unordered_map<std::string, StatusEntry>    status_map_;
  std::unordered_set<std::string>                  ignored_names_;
  double                                           default_timeout_{5.0};
  bool                                             enable_summary_log_{false};

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr    pub_;
  rclcpp::TimerBase::SharedPtr                     timer_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DiagnosticsAggregator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
