// Copyright (c) 2024 CAMROD Project
//
// Licensed under the Apache License, Version 2.0 (the "License").
// See LICENSE for details.

#ifndef CAMROD_DOCKING__CAMROD_DOCKING_PLUGIN_HPP_
#define CAMROD_DOCKING__CAMROD_DOCKING_PLUGIN_HPP_

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "opennav_docking_core/charging_dock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "tf2_ros/buffer.h"

namespace camrod_docking
{

/**
 * ChargingDock plugin for CAMROD that uses AprilTag-based pose detection.
 *
 * Subscribes to a PoseStamped topic published by docking_apriltag_bridge
 * (odom frame) to refine the dock pose during docking approach.
 * isDocked() uses TF distance threshold.
 *
 * isCharging() subscribes to /battery_state (sensor_msgs/BatteryState)
 * published by ranger_ros2 and checks current > charging_current_threshold.
 * The Ranger BMS does not expose a dedicated charging-status flag via ugv_sdk,
 * so positive current is the only reliable indicator of active charging.
 *
 * Tune charging_current_threshold (param, default 0.5 A) after measuring
 * actual charging current on real hardware.
 */
class CamrodDockingPlugin : public opennav_docking_core::ChargingDock
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name,
    std::shared_ptr<tf2_ros::Buffer> tf) override;

  void cleanup() override {}
  void activate() override {}
  void deactivate() override
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_received_ = false;
    filter_initialized_ = false;
    approach_yaw_locked_ = false;
  }

  geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & pose,
    const std::string & frame) override;

  bool getRefinedPose(geometry_msgs::msg::PoseStamped & pose, std::string id) override;

  bool isDocked() override;

  bool isCharging() override;
  bool disableCharging() override { return true; }
  bool hasStoppedCharging() override;

private:
  void onDockPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onBatteryState(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  mutable std::mutex pose_mutex_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  bool pose_received_{false};

  mutable std::mutex battery_mutex_;
  float battery_current_{0.0f};
  bool battery_received_{false};

  std::atomic<bool> docked_{false};

  std::string name_;
  std::string detected_dock_pose_topic_;
  std::string base_frame_;
  double staging_x_offset_{-0.70};
  double staging_yaw_offset_{0.0};
  double docking_threshold_{0.05};
  double external_detection_timeout_{1.0};
  float charging_current_threshold_{0.5f};  // A — tune on real hardware
  // Offset applied in getRefinedPose(): shifts tag pose → base_link target pose.
  // x < 0 moves target backward from tag (prevents robot body from hitting dock).
  double ext_translation_x_{-0.20};
  double ext_translation_y_{0.0};
  bool dock_backwards_{false};

  // PoseFilter state (EMA with RESET on timeout gap — mirrors SimpleChargingDock)
  double filter_coef_{0.1};
  geometry_msgs::msg::PoseStamped filtered_pose_;
  bool filter_initialized_{false};

  // approach_yaw locked from robot odom heading on first getRefinedPose() call.
  // Camera-derived yaw drifts with PnP viewing-angle errors; odom heading is stable.
  bool approach_yaw_locked_{false};
  double locked_approach_yaw_{0.0};
};

}  // namespace camrod_docking

#endif  // CAMROD_DOCKING__CAMROD_DOCKING_PLUGIN_HPP_
