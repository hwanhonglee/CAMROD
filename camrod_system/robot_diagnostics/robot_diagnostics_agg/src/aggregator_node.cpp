#include <chrono>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

using DiagnosticArray  = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using SteadyClock      = std::chrono::steady_clock;
using TimePoint        = std::chrono::steady_clock::time_point;

// ── 유틸리티 ──────────────────────────────────────────────────────────────

static const std::map<uint8_t, std::string> LEVEL_NAMES = {
  {DiagnosticStatus::OK,    "OK"},
  {DiagnosticStatus::WARN,  "WARN"},
  {DiagnosticStatus::ERROR, "ERROR"},
  {DiagnosticStatus::STALE, "STALE"},
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
  double      timeout_sec{5.0};
};

// ── DiagnosticsAggregator ─────────────────────────────────────────────────

class DiagnosticsAggregator : public rclcpp::Node
{
public:
  DiagnosticsAggregator()
  : rclcpp::Node("diagnostics_agg")
  {
    declare_parameter("config_file", std::string(""));
    std::string config_path = get_parameter("config_file").as_string();

    double publish_rate = 1.0;

    if (!config_path.empty()) {
      publish_rate = load_config(config_path);
    } else {
      RCLCPP_WARN(get_logger(), "config_file 파라미터가 설정되지 않았습니다. 기본값으로 동작합니다.");
    }

    sub_ = create_subscription<DiagnosticArray>(
      "/diagnostics", 10,
      [this](const DiagnosticArray::SharedPtr msg) { diagnostics_callback(msg); });

    pub_   = create_publisher<DiagnosticArray>("/diagnostics_agg", 10);
    timer_ = create_timer(
      this, get_clock(),
      std::chrono::duration<double>(1.0 / publish_rate),
      [this]() { publish_aggregated(); });

    RCLCPP_INFO(
      get_logger(),
      "DiagnosticsAggregator 시작. 모니터링 토픽 수: %zu, 기본 timeout: %.1fs",
      topic_configs_.size(), default_timeout_);
  }

private:
  double load_config(const std::string & config_path)
  {
    std::ifstream f(config_path);
    if (!f.good()) {
      RCLCPP_ERROR(get_logger(), "config 파일을 찾을 수 없습니다: %s", config_path.c_str());
      return 1.0;
    }

    YAML::Node cfg = YAML::LoadFile(config_path);

    double publish_rate = 1.0;
    if (cfg["global"]) {
      auto g = cfg["global"];
      if (g["timeout_sec"])  default_timeout_ = g["timeout_sec"].as<double>();
      if (g["publish_rate"]) publish_rate      = g["publish_rate"].as<double>();
    }

    if (cfg["topics"]) {
      for (const auto & t : cfg["topics"]) {
        TopicConfig tc;
        tc.name        = t["name"].as<std::string>();
        tc.group       = t["group"] ? t["group"].as<std::string>() : "unknown";
        tc.timeout_sec = t["timeout_sec"] ? t["timeout_sec"].as<double>() : default_timeout_;
        topic_configs_[tc.name] = tc;
      }
    }

    RCLCPP_INFO(get_logger(), "config 로드 완료: %s", config_path.c_str());
    for (const auto & [name, tc] : topic_configs_) {
      RCLCPP_INFO(get_logger(), "  [%s] %s  timeout=%.1fs",
        tc.group.c_str(), name.c_str(), tc.timeout_sec);
    }

    return publish_rate;
  }

  void diagnostics_callback(const DiagnosticArray::SharedPtr msg)
  {
    auto now = SteadyClock::now();
    for (const auto & status : msg->status) {
      if (!topic_configs_.empty() && topic_configs_.find(status.name) == topic_configs_.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "config에 없는 토픽 수신: \"%s\" → unknown 그룹으로 처리", status.name.c_str());
      }
      status_map_[status.name] = {status, now};
    }
  }

  double get_timeout(const std::string & name) const
  {
    auto it = topic_configs_.find(name);
    return it != topic_configs_.end() ? it->second.timeout_sec : default_timeout_;
  }

  std::string get_group(const std::string & name) const
  {
    auto it = topic_configs_.find(name);
    return it != topic_configs_.end() ? it->second.group : "unknown";
  }

  DiagnosticStatus check_stale(const std::string & name, const DiagnosticStatus & status,
    TimePoint last_seen)
  {
    double elapsed =
      std::chrono::duration<double>(SteadyClock::now() - last_seen).count();

    if (elapsed > get_timeout(name)) {
      DiagnosticStatus stale;
      stale.name        = status.name;
      stale.hardware_id = status.hardware_id;
      stale.level       = DiagnosticStatus::STALE;
      std::ostringstream oss;
      oss << "STALE (last seen " << std::fixed;
      oss.precision(1);
      oss << elapsed << "s ago)";
      stale.message = oss.str();
      return stale;
    }
    return status;
  }

  void publish_aggregated()
  {
    if (status_map_.empty()) {
      return;
    }

    DiagnosticArray agg_msg;
    agg_msg.header.stamp = get_clock()->now();

    std::map<std::string, uint8_t> group_worst;

    for (auto & [name, entry] : status_map_) {
      DiagnosticStatus s = check_stale(name, entry.status, entry.last_seen);
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

    // 그룹별 요약 로그
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

  // 상태 엔트리
  struct StatusEntry {
    DiagnosticStatus status;
    TimePoint        last_seen;
  };

  std::unordered_map<std::string, TopicConfig>    topic_configs_;
  std::unordered_map<std::string, StatusEntry>    status_map_;
  double                                           default_timeout_{5.0};

  rclcpp::Subscription<DiagnosticArray>::SharedPtr sub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr    pub_;
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
