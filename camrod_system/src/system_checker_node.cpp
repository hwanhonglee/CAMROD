#include <algorithm>
#include <chrono>
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

class SystemCheckerNode : public rclcpp::Node
{
public:
  SystemCheckerNode()
  : Node("system_checker")
  {
    check_period_s_ = declare_parameter<double>("check_period_s", 1.0);

    // Canonical: startup_grace_s / Legacy: startup_grace_sec.
    double startup_grace_s = declare_parameter<double>("startup_grace_s", 6.0);
    const double startup_grace_sec_legacy =
      declare_parameter<double>("startup_grace_sec", 6.0);
    if (startup_grace_s == 6.0 && startup_grace_sec_legacy != 6.0) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter 'startup_grace_sec' is deprecated. Use 'startup_grace_s'.");
      startup_grace_s = startup_grace_sec_legacy;
    }
    startup_grace_s_ = startup_grace_s;

    declare_parameter<std::vector<std::string>>(
      "required_nodes", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>(
      "required_topics", std::vector<std::string>{});
    required_nodes_ = normalize_names(
      get_parameter("required_nodes").as_string_array());
    required_topics_ = normalize_names(
      get_parameter("required_topics").as_string_array());

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

  // Builds one diagnostic status for either node-missing or topic-missing checks.
  static diagnostic_msgs::msg::DiagnosticStatus build_status(
    const std::string & name, const std::vector<std::string> & missing)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    if (!missing.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "missing";
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "missing";
      std::string merged;
      for (size_t i = 0; i < missing.size(); ++i) {
        merged += missing[i];
        if (i + 1 < missing.size()) {
          merged += ",";
        }
      }
      kv.value = merged;
      status.values.push_back(kv);
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "ok";
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

    std::set<std::string> topic_names;
    const auto topics = get_topic_names_and_types();
    for (const auto & kv : topics) {
      topic_names.insert(kv.first);
    }

    std::vector<std::string> missing_nodes;
    for (const auto & need : required_nodes_) {
      if (node_names.find(need) == node_names.end()) {
        missing_nodes.push_back(need);
      }
    }
    std::vector<std::string> missing_topics;
    for (const auto & need : required_topics_) {
      if (topic_names.find(need) == topic_names.end()) {
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
    diag.status.push_back(build_status("system_checker/nodes", missing_nodes));
    diag.status.push_back(build_status("system_checker/topics", missing_topics));
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

private:
  double check_period_s_{1.0};
  double startup_grace_s_{6.0};
  std::string diagnostic_topic_;
  std::vector<std::string> required_nodes_;
  std::vector<std::string> required_topics_;

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
