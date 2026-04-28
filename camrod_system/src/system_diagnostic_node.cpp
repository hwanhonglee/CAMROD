#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "avg_msgs/msg/module_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camrod_system
{

struct ModuleSnapshot
{
  int level{static_cast<int>(avg_msgs::msg::ModuleState::OK)};
  std::string message{"no status yet"};
  double stamp_sec{0.0};
};

class SystemDiagnosticNode : public rclcpp::Node
{
public:
  SystemDiagnosticNode()
  : Node("system_diagnostic")
  {
    diagnostic_topic_ = declare_parameter<std::string>("diagnostic_topic", "/diagnostics");
    publish_period_s_ = declare_parameter<double>("publish_period_s", 0.5);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 2.0);
    source_diagnostic_topic_ =
      declare_parameter<std::string>("source_diagnostic_topic", "/diagnostics");
    known_modules_ = declare_parameter<std::vector<std::string>>(
      "known_modules",
      std::vector<std::string>{
        "map",
        "sensing",
        "localization",
        "planning",
        "platform",
        "perception",
        "sensor_kit",
        "bringup",
        "system",
      });

    for (const auto & module : known_modules_) {
      snapshots_[module] = ModuleSnapshot{
        static_cast<int>(avg_msgs::msg::ModuleState::WARN),
        "no status yet",
        0.0,
      };
    }

    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostic_topic_, 10);
    diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      source_diagnostic_topic_, 10,
      std::bind(&SystemDiagnosticNode::on_diagnostic, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_s_),
      std::bind(&SystemDiagnosticNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "system_diagnostic: diagnostic=%s period=%.2fs",
      diagnostic_topic_.c_str(), publish_period_s_);
  }

private:
  // Extracts module category from diagnostic key-values or fallback name/hardware_id.
  static std::string extract_category(const diagnostic_msgs::msg::DiagnosticStatus & status)
  {
    for (const auto & kv : status.values) {
      if (kv.key == "category" && !kv.value.empty()) {
        return kv.value;
      }
    }
    if (!status.name.empty()) {
      const auto slash = status.name.find('/');
      return slash == std::string::npos ? status.name : status.name.substr(0, slash);
    }
    return status.hardware_id;
  }

  // Consumes incoming module diagnostics and updates latest snapshot per category.
  void on_diagnostic(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    const double now_sec = now().seconds();
    for (const auto & status : msg->status) {
      const std::string category = extract_category(status);
      if (category.empty()) {
        continue;
      }
      auto it = snapshots_.find(category);
      if (it == snapshots_.end()) {
        it = snapshots_.insert(
          std::make_pair(
            category,
            ModuleSnapshot{
              static_cast<int>(avg_msgs::msg::ModuleState::WARN),
              "discovered dynamically",
              0.0,
            })).first;
      }
      it->second.level = static_cast<int>(status.level);
      it->second.message = status.message;
      it->second.stamp_sec = now_sec;
    }
  }

  // Builds and publishes one system-level diagnostic snapshot.
  void on_timer()
  {
    const auto stamp = now();
    const double now_sec = stamp.seconds();

    std::vector<std::string> warn_modules;
    std::vector<std::string> error_modules;

    for (const auto & kv : snapshots_) {
      const auto & module_name = kv.first;
      const auto & snap = kv.second;

      if (snap.stamp_sec <= 0.0) {
        warn_modules.push_back(module_name);
        continue;
      }

      const double age = now_sec - snap.stamp_sec;
      if (age > stale_timeout_s_) {
        warn_modules.push_back(module_name);
        continue;
      }

      if (snap.level >= static_cast<int>(avg_msgs::msg::ModuleState::ERROR)) {
        error_modules.push_back(module_name);
      } else if (snap.level == static_cast<int>(avg_msgs::msg::ModuleState::WARN)) {
        warn_modules.push_back(module_name);
      }
    }

    diagnostic_msgs::msg::DiagnosticArray diag;
    diag.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "system/diagnostic";
    st.hardware_id = "system";

    if (!error_modules.empty()) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      st.message = "one or more modules in ERROR";
    } else if (!warn_modules.empty()) {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      st.message = "one or more modules in WARN";
    } else {
      st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      st.message = "system healthy";
    }

    st.values.push_back(make_kv("category", "system"));
    st.values.push_back(make_kv("status_count", std::to_string(snapshots_.size())));
    st.values.push_back(make_kv("active_modules", join_sorted_keys(snapshots_)));
    st.values.push_back(make_kv("warn_modules", join_vector(warn_modules)));
    st.values.push_back(make_kv("error_modules", join_vector(error_modules)));
    diag.status.push_back(st);

    diag_pub_->publish(diag);
  }

  // Helper to create diagnostic key-value fields with consistent style.
  static diagnostic_msgs::msg::KeyValue make_kv(const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
  }

  // Joins map keys sorted by std::map order (module name ascending).
  static std::string join_sorted_keys(const std::map<std::string, ModuleSnapshot> & values)
  {
    std::vector<std::string> keys;
    keys.reserve(values.size());
    for (const auto & kv : values) {
      keys.push_back(kv.first);
    }
    return join_vector(keys);
  }

  // Joins vector values into a comma-separated string.
  static std::string join_vector(const std::vector<std::string> & values)
  {
    std::string out;
    for (size_t i = 0; i < values.size(); ++i) {
      out += values[i];
      if (i + 1 < values.size()) {
        out += ",";
      }
    }
    return out;
  }

private:
  std::string diagnostic_topic_;
  std::string source_diagnostic_topic_;
  double publish_period_s_{0.5};
  double stale_timeout_s_{2.0};
  std::vector<std::string> known_modules_;
  std::map<std::string, ModuleSnapshot> snapshots_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camrod_system

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camrod_system::SystemDiagnosticNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

