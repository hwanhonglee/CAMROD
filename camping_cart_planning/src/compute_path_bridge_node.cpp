#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>

namespace camping_cart_planning
{

class ComputePathBridgeNode : public rclcpp::Node
{
public:
  ComputePathBridgeNode()
  : rclcpp::Node("compute_path_bridge")
  {
    // 2026-02-02: Bridge planner_server output to global_path for RViz.
    // Prefer unsmoothed_plan (planner_server publishes it by default in Humble).
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/planning/unsmoothed_plan");
    fallback_input_topic_ = declare_parameter<std::string>(
      "fallback_input_topic", "/planning/plan");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/planning/global_path");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    // 2026-02-03: Latch last path for RViz and diagnostics.
    pub_path_ = create_publisher<nav_msgs::msg::Path>(
      output_topic_, rclcpp::QoS(1).transient_local().reliable());

    const double republish_rate = declare_parameter<double>("republish_rate_hz", 1.0);
    if (republish_rate > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / republish_rate);
      republish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ComputePathBridgeNode::republishLast, this));
    }

    sub_path_ = create_subscription<nav_msgs::msg::Path>(
      input_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&ComputePathBridgeNode::on_path, this, std::placeholders::_1));
    if (!fallback_input_topic_.empty()) {
      sub_path_fallback_ = create_subscription<nav_msgs::msg::Path>(
        fallback_input_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&ComputePathBridgeNode::on_path, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      get_logger(),
      "ComputePath bridge listening on %s -> %s",
      input_topic_.c_str(), output_topic_.c_str());
    if (!fallback_input_topic_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "ComputePath bridge fallback on %s",
        fallback_input_topic_.c_str());
    }
  }

private:
  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    nav_msgs::msg::Path path = *msg;
    if (path.poses.empty()) {
      return;
    }

    if (!frame_id_.empty()) {
      path.header.frame_id = frame_id_;
    }
    if (path.header.stamp.sec == 0 && path.header.stamp.nanosec == 0) {
      path.header.stamp = now();
    }

    last_path_ = path;
    has_path_ = true;
    pub_path_->publish(path);
  }

  void republishLast()
  {
    if (!has_path_) {
      return;
    }
    last_path_.header.stamp = now();
    pub_path_->publish(last_path_);
  }

  std::string input_topic_;
  std::string fallback_input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_fallback_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  nav_msgs::msg::Path last_path_;
  bool has_path_{false};
};

}  // namespace camping_cart_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camping_cart_planning::ComputePathBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
