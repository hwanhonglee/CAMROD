#pragma once

// HH_260721 - Make ROS compatibility conversions explicit at the control package boundary.

#include "avg_msgs/msg/avg_pose_stamped.hpp"
#include "avg_msgs/msg/avg_twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace camrod_control
{

inline avg_msgs::msg::AvgPoseStamped poseFromRos(
  const geometry_msgs::msg::PoseStamped & input)
{
  avg_msgs::msg::AvgPoseStamped output;
  output.header.stamp = input.header.stamp;
  output.header.frame_id = input.header.frame_id;
  output.pose.position.x = input.pose.position.x;
  output.pose.position.y = input.pose.position.y;
  output.pose.position.z = input.pose.position.z;
  output.pose.orientation.x = input.pose.orientation.x;
  output.pose.orientation.y = input.pose.orientation.y;
  output.pose.orientation.z = input.pose.orientation.z;
  output.pose.orientation.w = input.pose.orientation.w;
  return output;
}

inline avg_msgs::msg::AvgTwist twistFromRos(const geometry_msgs::msg::Twist & input)
{
  avg_msgs::msg::AvgTwist output;
  output.linear.x = input.linear.x;
  output.linear.y = input.linear.y;
  output.linear.z = input.linear.z;
  output.angular.x = input.angular.x;
  output.angular.y = input.angular.y;
  output.angular.z = input.angular.z;
  return output;
}

inline geometry_msgs::msg::Twist twistToRos(const avg_msgs::msg::AvgTwist & input)
{
  geometry_msgs::msg::Twist output;
  output.linear.x = input.linear.x;
  output.linear.y = input.linear.y;
  output.linear.z = input.linear.z;
  output.angular.x = input.angular.x;
  output.angular.y = input.angular.y;
  output.angular.z = input.angular.z;
  return output;
}

}  // namespace camrod_control
