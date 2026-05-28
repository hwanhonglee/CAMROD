// Copyright (c) 2024 CAMROD Project
//
// Licensed under the Apache License, Version 2.0 (the "License").
// See LICENSE for details.

#include "camrod_docking/camrod_docking_plugin.hpp"

#include <cmath>
#include <functional>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace camrod_docking
{

void CamrodDockingPlugin::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name,
  std::shared_ptr<tf2_ros::Buffer> tf)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;

  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("CamrodDockingPlugin: parent lifecycle node expired during configure");
  }

  detected_dock_pose_topic_ = node->declare_parameter(
    name + ".detected_dock_pose_topic", "/docking/detected_dock_pose");
  base_frame_ = node->declare_parameter(name + ".base_frame", "robot_base_link");
  staging_x_offset_ = node->declare_parameter(name + ".staging_x_offset", -0.70);
  staging_yaw_offset_ = node->declare_parameter(name + ".staging_yaw_offset", 0.0);
  docking_threshold_ = node->declare_parameter(name + ".docking_threshold", 0.05);
  external_detection_timeout_ =
    node->declare_parameter(name + ".external_detection_timeout", 1.0);
  charging_current_threshold_ = static_cast<float>(
    node->declare_parameter(name + ".charging_current_threshold", 0.5));
  ext_translation_x_ = node->declare_parameter(name + ".external_detection_translation_x", -0.20);
  ext_translation_y_ = node->declare_parameter(name + ".external_detection_translation_y", 0.0);
  filter_coef_ = node->declare_parameter(name + ".filter_coef", 0.1);
  dock_backwards_ = node->get_parameter("dock_backwards").as_bool();

  dock_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    detected_dock_pose_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&CamrodDockingPlugin::onDockPose, this, std::placeholders::_1));

  battery_sub_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery_state",
    rclcpp::SensorDataQoS(),
    std::bind(&CamrodDockingPlugin::onBatteryState, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(),
    "[%s] CamrodDockingPlugin configured: pose_topic=%s threshold=%.3f staging_x=%.2f "
    "ext_translation=(%.3f, %.3f) filter_coef=%.2f charging_current_threshold=%.2fA",
    name_.c_str(), detected_dock_pose_topic_.c_str(),
    docking_threshold_, staging_x_offset_,
    ext_translation_x_, ext_translation_y_, filter_coef_, charging_current_threshold_);
}

void CamrodDockingPlugin::onDockPose(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  latest_pose_ = *msg;
  pose_received_ = true;
}

void CamrodDockingPlugin::onBatteryState(
  const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(battery_mutex_);
  battery_current_ = msg->current;
  battery_received_ = true;
}

geometry_msgs::msg::PoseStamped CamrodDockingPlugin::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  // Reset per-attempt state.
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    filter_initialized_ = false;
    approach_yaw_locked_ = false;
  }

  // Staging pose: offset behind the dock in its forward direction.
  const double dock_yaw = tf2::getYaw(pose.orientation) + staging_yaw_offset_;

  geometry_msgs::msg::PoseStamped staging;
  staging.header.frame_id = frame;
  staging.header.stamp = rclcpp::Clock().now();
  staging.pose.position.x = pose.position.x + staging_x_offset_ * std::cos(dock_yaw);
  staging.pose.position.y = pose.position.y + staging_x_offset_ * std::sin(dock_yaw);
  staging.pose.position.z = pose.position.z;

  // 후진 도킹: 로봇 후면이 도크를 향하도록 +π / 전진 도킹: dock 방향 그대로
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, dock_backwards_ ? dock_yaw + M_PI : dock_yaw);
  staging.pose.orientation = tf2::toMsg(q);
  return staging;
}

bool CamrodDockingPlugin::getRefinedPose(
  geometry_msgs::msg::PoseStamped & pose, std::string /*id*/)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!pose_received_) {
    return false;
  }

  auto node = node_.lock();
  if (!node) { return false; }

  const double age =
    (node->get_clock()->now() - rclcpp::Time(latest_pose_.header.stamp)).seconds();
  if (age > external_detection_timeout_) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000,
      "[%s] dock pose stale (%.2fs > %.2fs)", name_.c_str(), age, external_detection_timeout_);
    return false;
  }

  // Fix #1: PoseFilter with RESET on timeout gap (mirrors SimpleChargingDock).
  // Prevents EMA lock-in at stale position when detection recovers after a gap.
  if (!filter_initialized_) {
    filtered_pose_ = latest_pose_;
    filter_initialized_ = true;
  } else {
    const double gap =
      (rclcpp::Time(latest_pose_.header.stamp) -
       rclcpp::Time(filtered_pose_.header.stamp)).seconds();
    if (gap > external_detection_timeout_) {
      // Detection gap exceeded timeout — reset filter to avoid jumping from stale value.
      filtered_pose_ = latest_pose_;
      RCLCPP_INFO(node->get_logger(),
        "[%s] PoseFilter RESET (gap=%.2fs)", name_.c_str(), gap);
    } else {
      filtered_pose_.header = latest_pose_.header;
      const double a = filter_coef_;
      filtered_pose_.pose.position.x =
        a * latest_pose_.pose.position.x + (1.0 - a) * filtered_pose_.pose.position.x;
      filtered_pose_.pose.position.y =
        a * latest_pose_.pose.position.y + (1.0 - a) * filtered_pose_.pose.position.y;
      filtered_pose_.pose.position.z =
        a * latest_pose_.pose.position.z + (1.0 - a) * filtered_pose_.pose.position.z;
      // Orientation: slerp
      tf2::Quaternion q_old, q_new, q_slerp;
      tf2::fromMsg(filtered_pose_.pose.orientation, q_old);
      tf2::fromMsg(latest_pose_.pose.orientation, q_new);
      q_slerp = q_old.slerp(q_new, a);
      q_slerp.normalize();
      filtered_pose_.pose.orientation = tf2::toMsg(q_slerp);
    }
  }

  // Use latest filtered dock position as target (dynamic — updates each 20Hz cycle).
  pose = filtered_pose_;

  // approach_yaw locked from robot odom heading on first call per docking attempt.
  // Camera-derived yaw (position or z-axis) drifts with PnP viewing-angle errors.
  // Odom heading is stable and is correctly set toward the dock by getStagingPose().
  if (!approach_yaw_locked_) {
    try {
      geometry_msgs::msg::TransformStamped robot_tf = tf_->lookupTransform(
        filtered_pose_.header.frame_id, base_frame_, tf2::TimePointZero);
      const double robot_heading = tf2::getYaw(robot_tf.transform.rotation);
      // 후진: 로봇이 dock 반대를 향하므로 +π 보정 / 전진: heading 그대로
      locked_approach_yaw_ = dock_backwards_ ? robot_heading + M_PI : robot_heading;
      approach_yaw_locked_ = true;
      RCLCPP_INFO(node->get_logger(),
        "[%s] approach_yaw LOCKED: robot_heading=%.1f° → approach_yaw=%.1f° (dock_backwards=%s)",
        name_.c_str(), robot_heading * 180.0 / M_PI, locked_approach_yaw_ * 180.0 / M_PI,
        dock_backwards_ ? "true" : "false");
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(node->get_logger(), "[%s] TF lookup failed for approach_yaw lock: %s",
        name_.c_str(), e.what());
      return false;
    }
  }
  const double approach_yaw = locked_approach_yaw_;

  RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
    "[%s] getRefinedPose: dock_pos=(%.3f,%.3f) approach_yaw=%.1f° (odom-locked)",
    name_.c_str(), filtered_pose_.pose.position.x, filtered_pose_.pose.position.y,
    approach_yaw * 180.0 / M_PI);

  // Shift position from tag toward docking target (ext_translation_x < 0 = toward robot).
  pose.pose.position.x += std::cos(approach_yaw) * ext_translation_x_
                        - std::sin(approach_yaw) * ext_translation_y_;
  pose.pose.position.y += std::sin(approach_yaw) * ext_translation_x_
                        + std::cos(approach_yaw) * ext_translation_y_;
  pose.pose.position.z = 0.0;

  // Replace orientation with a clean 2D yaw (roll=0, pitch=0).
  // DockingServer가 dock_backwards=true 시 +π를 자동 적용하므로 approach_yaw 그대로 반환.
  tf2::Quaternion qout;
  qout.setRPY(0.0, 0.0, approach_yaw);
  pose.pose.orientation = tf2::toMsg(qout);

  return true;
}

bool CamrodDockingPlugin::isDocked()
{
  geometry_msgs::msg::PoseStamped dock_pose;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!pose_received_ || !filter_initialized_) {
      return false;
    }
    dock_pose = filtered_pose_;
  }

  try {
    // Compare base_link distance to tag position (not approach_yaw-dependent target).
    // Docked when dist ≈ |ext_translation_x_| (robot stopped at expected offset from tag).
    geometry_msgs::msg::TransformStamped t = tf_->lookupTransform(
      dock_pose.header.frame_id, base_frame_, tf2::TimePointZero);
    const double dx = t.transform.translation.x - dock_pose.pose.position.x;
    const double dy = t.transform.translation.y - dock_pose.pose.position.y;
    const double dist = std::hypot(dx, dy);
    const double expected = std::abs(ext_translation_x_);
    bool result = std::abs(dist - expected) < docking_threshold_;
    docked_.store(result);
    auto node = node_.lock();
    if (node) {
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
        "[%s] isDocked dist=%.4fm expected=%.4fm threshold=%.4fm → %s",
        name_.c_str(), dist, expected, docking_threshold_, result ? "DOCKED" : "not yet");
    }
    return result;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

bool CamrodDockingPlugin::isCharging()
{
  // 하드웨어 없는 테스트: isDocked() 확인 후에만 true 반환
  // 실제 충전 하드웨어 연동 시 아래로 교체:
  // std::lock_guard<std::mutex> lock(battery_mutex_);
  // if (!battery_received_) { return false; }
  // return battery_current_ > charging_current_threshold_;
  return docked_.load();
}

bool CamrodDockingPlugin::hasStoppedCharging()
{
  std::lock_guard<std::mutex> lock(battery_mutex_);
  if (!battery_received_) {
    return true;
  }
  return battery_current_ <= charging_current_threshold_;
}

}  // namespace camrod_docking

PLUGINLIB_EXPORT_CLASS(
  camrod_docking::CamrodDockingPlugin,
  opennav_docking_core::ChargingDock)
