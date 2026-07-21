#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "avg_msgs/msg/avg_system_msgs.hpp"
#include "avg_msgs/msg/module_state.hpp"
#include "avg_msgs/msg/system_status.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camrod_system
{

struct ModuleSnapshot
{
  std::string category{"system"};
  int level{static_cast<int>(avg_msgs::msg::ModuleState::OK)};
  std::string message{"no status yet"};
  double stamp_sec{0.0};
  std::vector<diagnostic_msgs::msg::KeyValue> values;
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
    system_status_topic_ = declare_parameter<std::string>("system_status_topic", "status");
    avg_system_msgs_topic_ = declare_parameter<std::string>("avg_system_msgs_topic", "msgs");
    // HH_260702 - Keep the operator console focused on one system-level line;
    // detailed checker data remains available on /system/status and diagnostics topics.
    log_status_summary_ = declare_parameter<bool>("log_status_summary", true);
    log_status_summary_period_s_ = declare_parameter<double>("log_status_summary_period_s", 5.0);
    known_modules_ = declare_parameter<std::vector<std::string>>(
      "known_modules",
      std::vector<std::string>{
        "hardware",
        "map",
        "sensing",
        "localization",
        "planning",
        // HH_260720 - Control is a first-class runtime module after command/maneuver extraction.
        "control",
        // HH_260720 - Include control-owned parking status in semantic /system/status.
        "parking",
        "platform",
        "perception",
        "system",
      });

    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostic_topic_, 10);
    system_status_pub_ =
      create_publisher<avg_msgs::msg::SystemStatus>(system_status_topic_, 10);
    avg_system_msgs_pub_ =
      create_publisher<avg_msgs::msg::AvgSystemMsgs>(avg_system_msgs_topic_, 10);
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
    return infer_category_from_name(status.name, status.hardware_id);
  }

  // HH_260617: Most checker statuses are named "node: /domain/item".
  // Infer the owning CAMROD module so module readiness is not hidden by checker names.
  static std::string infer_category_from_name(
    const std::string & status_name,
    const std::string & hardware_id)
  {
    const std::string key = !status_name.empty() ? status_name : hardware_id;
    const auto colon = key.find(':');
    const std::string task = colon == std::string::npos ? key : key.substr(colon + 1);
    const std::string normalized = trim(task);

    if (starts_with(normalized, "/hardware/") || starts_with(key, "hw_checker") ||
      starts_with(key, "gpu_checker") || starts_with(key, "network_checker"))
    {
      return "hardware";
    }
    if (starts_with(normalized, "/sensor/") || starts_with(normalized, "/sensing/") ||
      starts_with(key, "gnss_checker") || starts_with(key, "imu_checker") ||
      starts_with(key, "lidar_checker") || starts_with(key, "radar_checker") ||
      starts_with(key, "camera_checker") || starts_with(key, "wheel_odometry_checker") ||
      starts_with(key, "velocity_converter_checker"))
    {
      return "sensing";
    }
    if (starts_with(normalized, "/localization/") || starts_with(key, "localization_")) {
      return "localization";
    }
    if (starts_with(normalized, "/planning/") || starts_with(key, "planning_")) {
      return "planning";
    }
    // HH_260720 - Classify control gate and maneuver diagnostics independently from planning.
    if (starts_with(normalized, "/control/") || starts_with(key, "control_")) {
      return "control";
    }
    if (starts_with(normalized, "/perception/") || starts_with(key, "perception_")) {
      return "perception";
    }
    if (starts_with(normalized, "/map/") || starts_with(key, "map_")) {
      return "map";
    }
    if (starts_with(normalized, "/platform/") || starts_with(key, "ranger_platform")) {
      return "platform";
    }
    if (starts_with(normalized, "/ui/") || starts_with(key, "ui_")) {
      return "ui";
    }
    if (starts_with(normalized, "/parking/") || starts_with(key, "parking_")) {
      return "parking";
    }
    if (starts_with(normalized, "/system/") || starts_with(key, "system_") ||
      starts_with(key, "system/"))
    {
      return "system";
    }
    return hardware_id;
  }

  static bool starts_with(const std::string & value, const std::string & prefix)
  {
    return value.rfind(prefix, 0) == 0;
  }

  static std::string trim(const std::string & value)
  {
    const auto first = value.find_first_not_of(" \t");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = value.find_last_not_of(" \t");
    return value.substr(first, last - first + 1);
  }

  static std::string collapse_spaces(std::string value)
  {
    std::string out;
    out.reserve(value.size());
    bool previous_space = false;
    for (const char ch : value) {
      const bool is_space = ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r';
      if (is_space) {
        if (!previous_space) {
          out.push_back(' ');
        }
        previous_space = true;
      } else {
        out.push_back(ch);
        previous_space = false;
      }
    }
    return trim(out);
  }

  static std::string console_safe_ascii(std::string value)
  {
    for (auto & ch : value) {
      const auto byte = static_cast<unsigned char>(ch);
      if (byte < 32 || byte > 126) {
        ch = ' ';
      }
    }
    return collapse_spaces(value);
  }

  // Consumes incoming diagnostics and stores latest snapshot by stable status key.
  void on_diagnostic(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    const double now_sec = now().seconds();
    for (const auto & status : msg->status) {
      const std::string category = extract_category(status);
      if (category.empty()) {
        continue;
      }
      const std::string status_key = !status.name.empty() ? status.name : status.hardware_id;
      if (status_key.empty()) {
        continue;
      }
      // HH_260617: system_diagnostic publishes `system/diagnostic` to the same
      // diagnostics bus it reads. Ignore its own summary to avoid recursively
      // turning a previous WARN/ERROR into a persistent system-module fault.
      if (status_key == "system/diagnostic") {
        snapshots_.erase(status_key);
        continue;
      }
      snapshots_[status_key] = ModuleSnapshot{
        category,
        static_cast<int>(status.level),
        status.message,
        now_sec,
        status.values,
      };
    }
  }

  // Builds and publishes one system-level diagnostic snapshot.
  void on_timer()
  {
    const auto stamp = now();
    const double now_sec = stamp.seconds();

    auto module_states = build_module_states(stamp, now_sec);

    std::vector<std::string> warn_modules;
    std::vector<std::string> error_modules;
    for (const auto & module : module_states) {
      if (module.level >= avg_msgs::msg::ModuleState::ERROR) {
        error_modules.push_back(module.module_name);
      } else if (module.level == avg_msgs::msg::ModuleState::WARN) {
        warn_modules.push_back(module.module_name);
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
    st.values.push_back(make_kv("status_count", std::to_string(module_states.size())));
    st.values.push_back(make_kv("active_modules", module_names(module_states)));
    st.values.push_back(make_kv("warn_modules", join_vector(warn_modules)));
    st.values.push_back(make_kv("error_modules", join_vector(error_modules)));
    diag.status.push_back(st);

    diag_pub_->publish(diag);
    publish_system_status(stamp, module_states, warn_modules, error_modules);
    maybe_log_system_summary(module_states, warn_modules, error_modules, now_sec);
  }

  std::vector<avg_msgs::msg::ModuleState> build_module_states(
    const rclcpp::Time & stamp,
    double now_sec)
  {
    std::map<std::string, avg_msgs::msg::ModuleState> modules;
    for (const auto & name : known_modules_) {
      avg_msgs::msg::ModuleState module;
      module.stamp = stamp;
      module.module_name = name;
      module.level = avg_msgs::msg::ModuleState::WARN;
      module.message = "no status yet";
      modules[name] = module;
    }

    for (const auto & kv : snapshots_) {
      const auto & snap = kv.second;
      const std::string category = snap.category.empty() ? "system" : snap.category;
      if (modules.find(category) == modules.end()) {
        avg_msgs::msg::ModuleState module;
        module.stamp = stamp;
        module.module_name = category;
        // HH_260721 - Dynamic alternative groups are not degraded by default.
        // Their first real checker result must be able to establish OK instead
        // of being masked forever by a synthetic WARN seed.
        module.level = avg_msgs::msg::ModuleState::OK;
        module.message = "discovered dynamically";
        modules[category] = module;
      }

      auto candidate = make_module_state(stamp, category, kv.first, snap, now_sec);
      auto & aggregate = modules[category];
      merge_module_state(aggregate, candidate);
    }

    std::vector<avg_msgs::msg::ModuleState> out;
    out.reserve(modules.size());
    for (const auto & kv : modules) {
      out.push_back(kv.second);
    }
    return out;
  }

  avg_msgs::msg::ModuleState make_module_state(
    const rclcpp::Time & stamp,
    const std::string & category,
    const std::string & status_key,
    const ModuleSnapshot & snap,
    double now_sec) const
  {
    avg_msgs::msg::ModuleState module;
    module.stamp = stamp;
    module.module_name = category;
    module.level = map_diagnostic_level(snap.level);
    module.message = status_key + ": " + snap.message;
    if (snap.stamp_sec <= 0.0 || now_sec - snap.stamp_sec > stale_timeout_s_) {
      module.level = std::max<uint8_t>(module.level, avg_msgs::msg::ModuleState::WARN);
      module.message = status_key + ": stale";
    }
    module.missing_nodes = split_csv(value_for(snap.values, "missing_nodes"));
    module.missing_topics = split_csv(value_for(snap.values, "missing_topics"));
    const auto publisher_missing = split_csv(value_for(snap.values, "publisher_missing"));
    module.missing_topics.insert(
      module.missing_topics.end(), publisher_missing.begin(), publisher_missing.end());
    const auto type_mismatches = split_csv(value_for(snap.values, "type_mismatches"));
    module.missing_topics.insert(
      module.missing_topics.end(), type_mismatches.begin(), type_mismatches.end());
    return module;
  }

  static uint8_t map_diagnostic_level(int level)
  {
    if (level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      return avg_msgs::msg::ModuleState::ERROR;
    }
    if (level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      return avg_msgs::msg::ModuleState::WARN;
    }
    return avg_msgs::msg::ModuleState::OK;
  }

  static void merge_module_state(
    avg_msgs::msg::ModuleState & aggregate,
    const avg_msgs::msg::ModuleState & candidate)
  {
    if (aggregate.message == "no status yet" || candidate.level >= aggregate.level) {
      aggregate.level = candidate.level;
      aggregate.message = candidate.message;
    }
    append_unique(aggregate.missing_nodes, candidate.missing_nodes);
    append_unique(aggregate.missing_topics, candidate.missing_topics);
    append_unique(aggregate.missing_lifecycle_nodes, candidate.missing_lifecycle_nodes);
  }

  static void append_unique(std::vector<std::string> & dst, const std::vector<std::string> & src)
  {
    for (const auto & item : src) {
      if (!item.empty() && std::find(dst.begin(), dst.end(), item) == dst.end()) {
        dst.push_back(item);
      }
    }
  }

  void publish_system_status(
    const rclcpp::Time & stamp,
    const std::vector<avg_msgs::msg::ModuleState> & modules,
    const std::vector<std::string> & warn_modules,
    const std::vector<std::string> & error_modules)
  {
    avg_msgs::msg::SystemStatus system_status;
    system_status.stamp = stamp;
    system_status.system_ok = error_modules.empty() && warn_modules.empty();
    if (!error_modules.empty()) {
      system_status.message = "one or more modules in ERROR";
    } else if (!warn_modules.empty()) {
      system_status.message = "one or more modules in WARN";
    } else {
      system_status.message = "system healthy";
    }
    system_status.modules = modules;
    system_status_pub_->publish(system_status);

    avg_msgs::msg::AvgSystemMsgs avg_msg;
    avg_msg.stamp = stamp;
    avg_msg.state.stamp = stamp;
    avg_msg.state.module_name = "system";
    avg_msg.state.level = system_status.system_ok ?
      avg_msgs::msg::ModuleState::OK : avg_msgs::msg::ModuleState::WARN;
    if (!error_modules.empty()) {
      avg_msg.state.level = avg_msgs::msg::ModuleState::ERROR;
    }
    avg_msg.state.message = system_status.message;
    avg_msg.system_status = system_status;
    for (const auto & module : modules) {
      avg_msg.active_modules.push_back(module.module_name);
    }
    avg_msg.status_count = static_cast<uint32_t>(modules.size());
    avg_system_msgs_pub_->publish(avg_msg);
  }

  void maybe_log_system_summary(
    const std::vector<avg_msgs::msg::ModuleState> & modules,
    const std::vector<std::string> & warn_modules,
    const std::vector<std::string> & error_modules,
    double now_sec)
  {
    if (!log_status_summary_) {
      return;
    }

    const bool has_error = !error_modules.empty();
    const bool has_warn = !warn_modules.empty();
    const std::string summary = build_console_summary(modules, has_error, has_warn);
    const bool changed = summary != last_console_summary_;
    const bool periodic_due =
      (has_error || has_warn) &&
      log_status_summary_period_s_ > 0.0 &&
      (last_console_summary_log_sec_ <= 0.0 ||
       now_sec - last_console_summary_log_sec_ >= log_status_summary_period_s_);

    if (!changed && !periodic_due) {
      return;
    }

    last_console_summary_ = summary;
    last_console_summary_log_sec_ = now_sec;
    if (has_error) {
      RCLCPP_ERROR(get_logger(), "%s", summary.c_str());
    } else if (has_warn) {
      RCLCPP_WARN(get_logger(), "%s", summary.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "%s", summary.c_str());
    }
  }

  static std::string build_console_summary(
    const std::vector<avg_msgs::msg::ModuleState> & modules,
    bool has_error,
    bool has_warn)
  {
    if (has_error) {
      std::string summary = "[SYSTEM] ERROR";
      summary += "\n  ERROR:\n";
      summary += describe_modules(modules, avg_msgs::msg::ModuleState::ERROR, true);
      const auto warn = describe_modules(modules, avg_msgs::msg::ModuleState::WARN, false);
      if (!warn.empty()) {
        summary += "\n  WARN:\n" + warn;
      }
      return summary;
    }
    if (has_warn) {
      return "[SYSTEM] WARN\n  WARN:\n" +
        describe_modules(modules, avg_msgs::msg::ModuleState::WARN, false);
    }
    return "[SYSTEM] OK\n  all modules healthy";
  }

  static std::string describe_modules(
    const std::vector<avg_msgs::msg::ModuleState> & modules,
    uint8_t level,
    bool include_higher)
  {
    std::string out;
    for (const auto & module : modules) {
      const bool match = include_higher ? module.level >= level : module.level == level;
      if (!match) {
        continue;
      }
      out += "    - " + format_module_line(module) + "\n";
    }
    if (!out.empty()) {
      out.pop_back();
    }
    return out;
  }

  static std::string format_module_line(const avg_msgs::msg::ModuleState & module)
  {
    std::string message = console_safe_ascii(module.message);
    std::string source;
    std::string detail = message;

    const auto first_colon = message.find(':');
    if (first_colon != std::string::npos) {
      source = trim(message.substr(0, first_colon));
      const auto rest = trim(message.substr(first_colon + 1));
      const auto second_colon = rest.find(':');
      if (second_colon != std::string::npos) {
        source += " " + trim(rest.substr(0, second_colon));
        detail = trim(rest.substr(second_colon + 1));
      } else {
        detail = rest;
      }
    }

    std::string item = console_safe_ascii(module.module_name) + ": " +
      compact_message(detail, 112);
    if (!source.empty()) {
      item += " [" + compact_message(source, 88) + "]";
    }
    return item;
  }

  static std::string compact_message(std::string value, size_t limit)
  {
    value = console_safe_ascii(value);
    if (value.size() <= limit) {
      return value;
    }
    if (limit <= 3) {
      return value.substr(0, limit);
    }
    return value.substr(0, limit - 3) + "...";
  }

  // Helper to create diagnostic key-value fields with consistent style.
  static diagnostic_msgs::msg::KeyValue make_kv(const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
  }

  static std::string module_names(const std::vector<avg_msgs::msg::ModuleState> & values)
  {
    std::vector<std::string> keys;
    keys.reserve(values.size());
    for (const auto & module : values) {
      keys.push_back(module.module_name);
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
        out += ", ";
      }
    }
    return out;
  }

  static std::string value_for(
    const std::vector<diagnostic_msgs::msg::KeyValue> & values,
    const std::string & key)
  {
    for (const auto & kv : values) {
      if (kv.key == key) {
        return kv.value;
      }
    }
    return "";
  }

  static std::vector<std::string> split_csv(const std::string & value)
  {
    std::vector<std::string> out;
    std::stringstream stream(value);
    std::string item;
    while (std::getline(stream, item, ',')) {
      item = trim(item);
      if (!item.empty()) {
        out.push_back(item);
      }
    }
    return out;
  }

private:
  std::string diagnostic_topic_;
  std::string source_diagnostic_topic_;
  std::string system_status_topic_;
  std::string avg_system_msgs_topic_;
  double publish_period_s_{0.5};
  double stale_timeout_s_{2.0};
  bool log_status_summary_{true};
  double log_status_summary_period_s_{5.0};
  std::string last_console_summary_;
  double last_console_summary_log_sec_{0.0};
  std::vector<std::string> known_modules_;
  std::map<std::string, ModuleSnapshot> snapshots_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<avg_msgs::msg::SystemStatus>::SharedPtr system_status_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgSystemMsgs>::SharedPtr avg_system_msgs_pub_;
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
