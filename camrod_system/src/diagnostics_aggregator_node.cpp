#include <algorithm>
#include <cstdio>
#include <map>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camrod_system
{

struct Snapshot
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  double stamp_sec{0.0};
};

class DiagnosticsAggregatorNode : public rclcpp::Node
{
public:
  DiagnosticsAggregatorNode()
  : Node("diagnostics_aggregator")
  {
    source_topic_ = declare_parameter<std::string>("source_topic", "/diagnostics");
    // HH_260617: Default to a relative topic; system.launch.py namespaces it to
    // `/system/diagnostics_agg_tools` when used for lightweight system tools.
    output_topic_ = declare_parameter<std::string>("output_topic", "diagnostics_agg");
    publish_period_s_ = declare_parameter<double>("publish_period_s", 1.0);
    stale_timeout_s_ = declare_parameter<double>("stale_timeout_s", 3.0);

    sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      source_topic_, 20,
      std::bind(&DiagnosticsAggregatorNode::on_diag, this, std::placeholders::_1));
    pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(output_topic_, 10);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_s_),
      std::bind(&DiagnosticsAggregatorNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "diagnostics_aggregator ready: source=%s output=%s stale_timeout_s=%.1f",
      source_topic_.c_str(), output_topic_.c_str(), stale_timeout_s_);
  }

private:
  // Copies only status payload fields used by downstream diagnostics consumers.
  static diagnostic_msgs::msg::DiagnosticStatus copy_status(
    const diagnostic_msgs::msg::DiagnosticStatus & src)
  {
    diagnostic_msgs::msg::DiagnosticStatus dst;
    dst.level = src.level;
    dst.name = src.name;
    dst.message = src.message;
    dst.hardware_id = src.hardware_id;
    dst.values = src.values;
    return dst;
  }

  // Stores latest status by stable key(name or hardware_id).
  void on_diag(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    const double now_sec = now().seconds();
    for (const auto & status : msg->status) {
      const std::string key = !status.name.empty() ? status.name : status.hardware_id;
      if (key.empty()) {
        continue;
      }
      snapshots_[key] = Snapshot{copy_status(status), now_sec};
    }
  }

  // Publishes sorted aggregated status list with stale-state marking.
  void on_timer()
  {
    if (snapshots_.empty()) {
      return;
    }

    const auto stamp = now();
    const double now_sec = stamp.seconds();

    diagnostic_msgs::msg::DiagnosticArray out;
    out.header.stamp = stamp;

    for (const auto & kv : snapshots_) {
      const auto & snap = kv.second;
      const double age = now_sec - snap.stamp_sec;
      auto status = copy_status(snap.status);
      if (age > stale_timeout_s_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        status.message = "stale (" + format_age(age) + "s)";
      }
      out.status.push_back(status);
    }

    pub_->publish(out);
  }

  // Formats age into fixed one-decimal text to keep legacy log style.
  static std::string format_age(double age_s)
  {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f", age_s);
    return std::string(buf);
  }

private:
  std::string source_topic_;
  std::string output_topic_;
  double publish_period_s_{1.0};
  double stale_timeout_s_{3.0};
  std::map<std::string, Snapshot> snapshots_;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camrod_system

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camrod_system::DiagnosticsAggregatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
