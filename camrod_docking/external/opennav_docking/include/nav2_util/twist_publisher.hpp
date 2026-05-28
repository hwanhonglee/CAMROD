// Minimal TwistPublisher shim for nav2_util compatibility.
// nav2_util 1.1.20 (apt) does not ship twist_publisher.hpp.
// Provides the same interface used by opennav_docking: publishes TwistStamped
// and extracts the Twist field to the standard cmd_vel topic.
#ifndef NAV2_UTIL__TWIST_PUBLISHER_HPP_
#define NAV2_UTIL__TWIST_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace nav2_util
{

class TwistPublisher
{
public:
  TwistPublisher(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & topic,
    size_t qos_depth)
  {
    pub_ = node->create_publisher<geometry_msgs::msg::Twist>(topic, qos_depth);
  }

  void on_activate() { pub_->on_activate(); }
  void on_deactivate() { pub_->on_deactivate(); }

  void publish(std::unique_ptr<geometry_msgs::msg::TwistStamped> msg)
  {
    auto twist = std::make_unique<geometry_msgs::msg::Twist>(msg->twist);
    pub_->publish(std::move(twist));
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__TWIST_PUBLISHER_HPP_
