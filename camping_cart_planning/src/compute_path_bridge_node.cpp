#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
    enforce_forward_from_start_ = declare_parameter<bool>("enforce_forward_from_start", true);
    start_pose_topic_ = declare_parameter<std::string>("start_pose_topic", "/localization/pose");
    goal_pose_topic_ = declare_parameter<std::string>("goal_pose_topic", "/planning/goal_pose");
    start_anchor_distance_ = declare_parameter<double>("start_anchor_distance", 1.0);
    goal_anchor_distance_ = declare_parameter<double>("goal_anchor_distance", 1.0);
    anchor_path_endpoints_ = declare_parameter<bool>("anchor_path_endpoints", false);

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
    if (enforce_forward_from_start_ && !start_pose_topic_.empty()) {
      sub_start_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        start_pose_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&ComputePathBridgeNode::on_start_pose, this, std::placeholders::_1));
    }
    if (!goal_pose_topic_.empty()) {
      sub_goal_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_pose_topic_, rclcpp::SystemDefaultsQoS(),
        std::bind(&ComputePathBridgeNode::on_goal_pose, this, std::placeholders::_1));
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
  static double dist2d(
    const geometry_msgs::msg::Point & a,
    const geometry_msgs::msg::Point & b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::hypot(dx, dy);
  }

  void on_start_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_start_pose_ = *msg;
    has_start_pose_ = true;
  }

  void on_goal_pose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_goal_pose_ = *msg;
    has_goal_pose_ = true;
  }

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

    if (enforce_forward_from_start_ && path.poses.size() >= 2) {
      const auto & first = path.poses.front().pose.position;
      const auto & last = path.poses.back().pose.position;
      bool should_reverse = false;

      if (has_start_pose_ && has_goal_pose_) {
        const auto & start = latest_start_pose_.pose.position;
        const auto & goal = latest_goal_pose_.pose.position;
        const double normal_score = dist2d(first, start) + dist2d(last, goal);
        const double reversed_score = dist2d(last, start) + dist2d(first, goal);
        should_reverse = reversed_score + 1e-3 < normal_score;
      } else if (has_start_pose_) {
        const auto & start = latest_start_pose_.pose.position;
        should_reverse = dist2d(last, start) + 1e-3 < dist2d(first, start);
      }

      if (should_reverse) {
        std::reverse(path.poses.begin(), path.poses.end());
      }
    }

    if (anchor_path_endpoints_ && has_start_pose_ && !path.poses.empty()) {
      const auto dist_start = dist2d(path.poses.front().pose.position, latest_start_pose_.pose.position);
      if (dist_start > start_anchor_distance_) {
        auto start = latest_start_pose_;
        start.header.frame_id = path.header.frame_id;
        path.poses.insert(path.poses.begin(), start);
      }
    }

    if (anchor_path_endpoints_ && has_goal_pose_ && !path.poses.empty()) {
      const auto dist_goal = dist2d(path.poses.back().pose.position, latest_goal_pose_.pose.position);
      if (dist_goal > goal_anchor_distance_) {
        auto goal = latest_goal_pose_;
        goal.header.frame_id = path.header.frame_id;
        path.poses.push_back(goal);
      }
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
  std::string start_pose_topic_;
  std::string goal_pose_topic_;
  bool enforce_forward_from_start_{true};
  double start_anchor_distance_{1.0};
  double goal_anchor_distance_{1.0};
  bool anchor_path_endpoints_{false};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_fallback_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_start_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  nav_msgs::msg::Path last_path_;
  geometry_msgs::msg::PoseStamped latest_start_pose_;
  geometry_msgs::msg::PoseStamped latest_goal_pose_;
  bool has_start_pose_{false};
  bool has_goal_pose_{false};
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
