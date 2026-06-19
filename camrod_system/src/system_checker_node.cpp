#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <sstream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camrod_system
{

struct RequiredTopicSpec
{
  std::string name;
  std::string type;
  int min_publishers{1};
};

struct RequiredModuleSpec
{
  std::string name;
  std::vector<std::string> required_nodes;
  std::vector<RequiredTopicSpec> required_topics;
};

struct RequiredAlternativeGroupSpec
{
  std::string name;
  std::vector<RequiredModuleSpec> alternatives;
};

class SystemCheckerNode : public rclcpp::Node
{
public:
  SystemCheckerNode()
  : Node("system_checker")
  {
    check_period_s_ = declare_parameter<double>("check_period_s", 1.0);

    startup_grace_s_ = declare_parameter<double>("startup_grace_s", 6.0);

    declare_parameter<std::vector<std::string>>(
      "required_nodes", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>(
      "required_topics", std::vector<std::string>{});
    required_nodes_ = normalize_names(
      get_parameter("required_nodes").as_string_array());
    required_topics_ = normalize_names(
      get_parameter("required_topics").as_string_array());

    // HH_260617: Add module-level graph manifests so system health can report
    // which package domain is missing required ROS nodes/topics, instead of one
    // ambiguous global "topics missing" list.
    declare_parameter<std::vector<std::string>>(
      "required_modules", std::vector<std::string>{});
    // HH_260618: Keep a debug-only exclusion hook without editing the shared
    // system_checker.yaml manifest. Normal bringup still requires final_parking.
    declare_parameter<std::vector<std::string>>(
      "disabled_modules", std::vector<std::string>{});
    declare_parameter<std::string>("disabled_modules_csv", "");
    std::set<std::string> disabled_modules;
    const auto disabled_modules_param = get_parameter("disabled_modules").as_string_array();
    for (const auto & name : disabled_modules_param) {
      if (!name.empty()) {
        disabled_modules.insert(name);
      }
    }
    for (const auto & name : split(get_parameter("disabled_modules_csv").as_string(), ',')) {
      const auto trimmed = trim(name);
      if (!trimmed.empty()) {
        disabled_modules.insert(trimmed);
      }
    }
    const auto required_modules = get_parameter("required_modules").as_string_array();
    for (const auto & module_name : required_modules) {
      if (module_name.empty()) {
        continue;
      }
      if (disabled_modules.count(module_name) > 0) {
        continue;
      }
      required_module_specs_.push_back(load_module_spec(module_name));
    }

    // HH_260618: Some runtime capabilities have multiple valid implementations.
    // Final parking must have exactly one healthy graph: camrod_parking OR camrod_docking.
    declare_parameter<std::vector<std::string>>(
      "required_alternative_groups", std::vector<std::string>{});
    const auto required_alternative_groups =
      get_parameter("required_alternative_groups").as_string_array();
    for (const auto & group_name : required_alternative_groups) {
      if (group_name.empty()) {
        continue;
      }
      if (disabled_modules.count(group_name) > 0) {
        continue;
      }
      const std::string alternatives_param = group_name + ".alternatives";
      if (!has_parameter(alternatives_param)) {
        declare_parameter<std::vector<std::string>>(
          alternatives_param, std::vector<std::string>{});
      }

      RequiredAlternativeGroupSpec group;
      group.name = group_name;
      const auto alternatives = get_parameter(alternatives_param).as_string_array();
      for (const auto & alternative_name : alternatives) {
        if (alternative_name.empty() || disabled_modules.count(alternative_name) > 0) {
          continue;
        }
        group.alternatives.push_back(load_module_spec(alternative_name));
      }
      required_alternative_group_specs_.push_back(group);
    }

    diagnostic_topic_ = declare_parameter<std::string>("diagnostic_topic", "/diagnostics");
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostic_topic_, 5);

    start_steady_ = std::chrono::steady_clock::now();
    last_report_steady_ = start_steady_;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(check_period_s_),
      std::bind(&SystemCheckerNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(), "system checker started: diagnostic=%s",
      diagnostic_topic_.c_str());
  }

private:
  // Normalizes names so every required node/topic is compared as an absolute graph name.
  static std::vector<std::string> normalize_names(const std::vector<std::string> & names)
  {
    std::vector<std::string> out;
    out.reserve(names.size());
    for (const auto & name : names) {
      if (name.empty()) {
        continue;
      }
      out.push_back(name.front() == '/' ? name : "/" + name);
    }
    return out;
  }

  static std::vector<std::string> split(const std::string & text, char delimiter)
  {
    std::vector<std::string> parts;
    std::stringstream stream(text);
    std::string part;
    while (std::getline(stream, part, delimiter)) {
      parts.push_back(part);
    }
    return parts;
  }

  static std::string trim(const std::string & text)
  {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
      return "";
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
  }

  // HH_260617: Topic spec format is "topic|type|min_publishers".
  // type and min_publishers are optional; examples:
  //   /localization/pose|avg_msgs/msg/PoseStamped|1
  //   /planning/engage|std_msgs/msg/Bool|0
  static std::vector<RequiredTopicSpec> parse_topic_specs(
    const std::vector<std::string> & specs)
  {
    std::vector<RequiredTopicSpec> out;
    out.reserve(specs.size());
    for (const auto & spec : specs) {
      if (spec.empty()) {
        continue;
      }
      const auto parts = split(spec, '|');
      RequiredTopicSpec topic;
      topic.name = parts.empty() ? "" : normalize_name(parts[0]);
      topic.type = parts.size() > 1 ? parts[1] : "";
      if (parts.size() > 2 && !parts[2].empty()) {
        try {
          topic.min_publishers = std::max(0, std::stoi(parts[2]));
        } catch (const std::exception &) {
          topic.min_publishers = 1;
        }
      }
      if (!topic.name.empty()) {
        out.push_back(topic);
      }
    }
    return out;
  }

  static std::string normalize_name(const std::string & name)
  {
    if (name.empty()) {
      return name;
    }
    return name.front() == '/' ? name : "/" + name;
  }

  static bool has_type(
    const std::vector<std::string> & actual_types,
    const std::string & expected_type)
  {
    if (expected_type.empty()) {
      return true;
    }
    return std::find(actual_types.begin(), actual_types.end(), expected_type) !=
           actual_types.end();
  }

  RequiredModuleSpec load_module_spec(const std::string & module_name)
  {
    const std::string node_param = module_name + ".required_nodes";
    const std::string topic_param = module_name + ".required_topics";
    if (!has_parameter(node_param)) {
      declare_parameter<std::vector<std::string>>(node_param, std::vector<std::string>{});
    }
    if (!has_parameter(topic_param)) {
      declare_parameter<std::vector<std::string>>(topic_param, std::vector<std::string>{});
    }

    RequiredModuleSpec module;
    module.name = module_name;
    module.required_nodes = normalize_names(get_parameter(node_param).as_string_array());
    module.required_topics = parse_topic_specs(get_parameter(topic_param).as_string_array());
    return module;
  }

  // Builds one diagnostic status for either node-missing or topic-missing checks.
  static diagnostic_msgs::msg::DiagnosticStatus build_status(
    const std::string & name,
    const std::vector<std::string> & missing,
    const std::string & category = "system",
    const std::string & missing_key = "missing")
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.values.push_back(make_kv("category", category));
    if (!missing.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "missing";
      status.values.push_back(make_kv(missing_key, join_vector(missing)));
      status.values.push_back(make_kv("missing", join_vector(missing)));
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "ok";
    }
    return status;
  }

  static std::string get_status_value(
    const diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key)
  {
    for (const auto & kv : status.values) {
      if (kv.key == key) {
        return kv.value;
      }
    }
    return "";
  }

  static std::string module_failure_summary(
    const diagnostic_msgs::msg::DiagnosticStatus & status)
  {
    std::vector<std::string> parts;
    const auto missing_nodes = get_status_value(status, "missing_nodes");
    const auto missing_topics = get_status_value(status, "missing_topics");
    const auto type_mismatches = get_status_value(status, "type_mismatches");
    const auto publisher_missing = get_status_value(status, "publisher_missing");
    if (!missing_nodes.empty()) {
      parts.push_back("nodes=" + missing_nodes);
    }
    if (!missing_topics.empty()) {
      parts.push_back("topics=" + missing_topics);
    }
    if (!type_mismatches.empty()) {
      parts.push_back("types=" + type_mismatches);
    }
    if (!publisher_missing.empty()) {
      parts.push_back("publishers=" + publisher_missing);
    }
    return join_vector(parts);
  }

  diagnostic_msgs::msg::DiagnosticStatus build_module_status(
    const RequiredModuleSpec & module,
    const std::set<std::string> & node_names,
    const std::map<std::string, std::vector<std::string>> & topic_names_and_types)
  {
    std::vector<std::string> missing_nodes;
    std::vector<std::string> missing_topics;
    std::vector<std::string> type_mismatches;
    std::vector<std::string> publisher_missing;

    for (const auto & need : module.required_nodes) {
      if (node_names.find(need) == node_names.end()) {
        missing_nodes.push_back(need);
      }
    }

    for (const auto & topic : module.required_topics) {
      auto topic_it = topic_names_and_types.find(topic.name);
      if (topic_it == topic_names_and_types.end()) {
        missing_topics.push_back(topic.name);
        continue;
      }
      if (!has_type(topic_it->second, topic.type)) {
        type_mismatches.push_back(topic.name + " expected=" + topic.type);
      }
      if (topic.min_publishers > 0) {
        const auto publisher_count = static_cast<int>(
          get_publishers_info_by_topic(topic.name).size());
        if (publisher_count < topic.min_publishers) {
          publisher_missing.push_back(
            topic.name + " publishers=" + std::to_string(publisher_count));
        }
      }
    }

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "system_checker/" + module.name;
    status.hardware_id = "system_checker";
    status.values.push_back(make_kv("category", module.name));
    status.values.push_back(make_kv("module", module.name));
    status.values.push_back(make_kv("required_nodes", std::to_string(module.required_nodes.size())));
    status.values.push_back(
      make_kv("required_topics", std::to_string(module.required_topics.size())));
    status.values.push_back(make_kv("missing_nodes", join_vector(missing_nodes)));
    status.values.push_back(make_kv("missing_topics", join_vector(missing_topics)));
    status.values.push_back(make_kv("type_mismatches", join_vector(type_mismatches)));
    status.values.push_back(make_kv("publisher_missing", join_vector(publisher_missing)));

    if (!missing_nodes.empty() || !missing_topics.empty() ||
      !type_mismatches.empty() || !publisher_missing.empty())
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "module graph incomplete";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "module graph ok";
    }
    return status;
  }

  diagnostic_msgs::msg::DiagnosticStatus build_alternative_group_status(
    const RequiredAlternativeGroupSpec & group,
    const std::set<std::string> & node_names,
    const std::map<std::string, std::vector<std::string>> & topic_names_and_types)
  {
    std::vector<std::string> checked_alternatives;
    std::vector<std::string> healthy_alternatives;
    std::vector<std::string> failed_alternatives;

    for (const auto & alternative : group.alternatives) {
      checked_alternatives.push_back(alternative.name);
      const auto alternative_status =
        build_module_status(alternative, node_names, topic_names_and_types);
      if (alternative_status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
        healthy_alternatives.push_back(alternative.name);
      } else {
        failed_alternatives.push_back(
          alternative.name + "(" + module_failure_summary(alternative_status) + ")");
      }
    }

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "system_checker/" + group.name;
    status.hardware_id = "system_checker";
    status.values.push_back(make_kv("category", group.name));
    status.values.push_back(make_kv("module", group.name));
    status.values.push_back(make_kv("checked_alternatives", join_vector(checked_alternatives)));
    status.values.push_back(make_kv("healthy_alternatives", join_vector(healthy_alternatives)));
    status.values.push_back(make_kv("failed_alternatives", join_vector(failed_alternatives)));

    if (healthy_alternatives.size() == 1U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "exactly one alternative graph ok";
      status.values.push_back(make_kv("selected_alternative", healthy_alternatives.front()));
    } else if (healthy_alternatives.empty()) {
      // HH_260618: A required capability with no healthy implementation is an
      // autonomy-blocking fault, not a degraded warning. Final parking cannot
      // be skipped in normal operation.
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "no required alternative graph ok";
      status.values.push_back(make_kv("selected_alternative", ""));
    } else {
      // HH_260618: Multiple healthy final-parking implementations are also an
      // invalid authority split because both can target the same motion command path.
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "multiple required alternative graphs ok";
      status.values.push_back(make_kv("selected_alternative", ""));
    }
    return status;
  }

  // Periodically checks required nodes/topics and publishes DiagnosticArray.
  void on_timer()
  {
    std::set<std::string> node_names;
    for (const auto & item : get_node_graph_interface()->get_node_names_and_namespaces()) {
      const auto & name = item.first;
      const auto & ns = item.second;
      if (ns == "/") {
        node_names.insert("/" + name);
      } else {
        node_names.insert(ns + "/" + name);
      }
    }

    std::map<std::string, std::vector<std::string>> topic_names_and_types;
    const auto topics = get_topic_names_and_types();
    for (const auto & kv : topics) {
      topic_names_and_types[kv.first] = kv.second;
    }

    std::vector<std::string> missing_nodes;
    for (const auto & need : required_nodes_) {
      if (node_names.find(need) == node_names.end()) {
        missing_nodes.push_back(need);
      }
    }
    std::vector<std::string> missing_topics;
    for (const auto & need : required_topics_) {
      if (topic_names_and_types.find(need) == topic_names_and_types.end()) {
        missing_topics.push_back(need);
      }
    }

    const auto now_steady = std::chrono::steady_clock::now();
    const double startup_elapsed_s =
      std::chrono::duration<double>(now_steady - start_steady_).count();
    const bool in_startup_grace = startup_elapsed_s < startup_grace_s_;

    if ((!missing_nodes.empty() || !missing_topics.empty()) && !in_startup_grace) {
      const double report_elapsed_s =
        std::chrono::duration<double>(now_steady - last_report_steady_).count();
      if (report_elapsed_s > 2.0) {
        RCLCPP_WARN(
          get_logger(),
          "system check missing nodes=%s topics=%s",
          join_or_dash(missing_nodes).c_str(),
          join_or_dash(missing_topics).c_str());
        last_report_steady_ = now_steady;
      }
    }

    diagnostic_msgs::msg::DiagnosticArray diag;
    diag.header.stamp = now();
    diag.status.push_back(
      build_status("system_checker/nodes", missing_nodes, "system", "missing_nodes"));
    diag.status.push_back(
      build_status("system_checker/topics", missing_topics, "system", "missing_topics"));
    for (const auto & module : required_module_specs_) {
      diag.status.push_back(build_module_status(module, node_names, topic_names_and_types));
    }
    for (const auto & group : required_alternative_group_specs_) {
      diag.status.push_back(
        build_alternative_group_status(group, node_names, topic_names_and_types));
    }
    pub_diag_->publish(diag);
  }

  // Joins list for warn logs while preserving legacy "-" output on empty list.
  static std::string join_or_dash(const std::vector<std::string> & values)
  {
    if (values.empty()) {
      return "-";
    }
    std::string merged;
    for (size_t i = 0; i < values.size(); ++i) {
      merged += values[i];
      if (i + 1 < values.size()) {
        merged += ",";
      }
    }
    return merged;
  }

  static diagnostic_msgs::msg::KeyValue make_kv(
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
  }

  static std::string join_vector(const std::vector<std::string> & values)
  {
    std::string merged;
    for (size_t i = 0; i < values.size(); ++i) {
      merged += values[i];
      if (i + 1 < values.size()) {
        merged += ",";
      }
    }
    return merged;
  }

private:
  double check_period_s_{1.0};
  double startup_grace_s_{6.0};
  std::string diagnostic_topic_;
  std::vector<std::string> required_nodes_;
  std::vector<std::string> required_topics_;
  std::vector<RequiredModuleSpec> required_module_specs_;
  std::vector<RequiredAlternativeGroupSpec> required_alternative_group_specs_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::chrono::steady_clock::time_point start_steady_;
  std::chrono::steady_clock::time_point last_report_steady_;
};

}  // namespace camrod_system

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camrod_system::SystemCheckerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
