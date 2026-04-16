//  Copyright 2023 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef ROUTE_PANEL_HPP_
#define ROUTE_PANEL_HPP_

#include <QButtonGroup>
#include <QCheckBox>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <autoware/adapi_specs/routing.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>

namespace tier4_adapi_rviz_plugins
{

class RoutePanel : public rviz_common::Panel
{
  Q_OBJECT
  using ClearRoute = autoware::adapi_specs::routing::ClearRoute;
  using SetRoutePoints = autoware::adapi_specs::routing::SetRoutePoints;
  using ChangeRoutePoints = autoware::adapi_specs::routing::ChangeRoutePoints;
  using PoseStamped = geometry_msgs::msg::PoseStamped;

public:
  explicit RoutePanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  QPushButton * waypoints_mode_;
  QPushButton * waypoints_reset_;
  QPushButton * waypoints_apply_;
  QPushButton * adapi_clear_;
  QPushButton * adapi_set_;
  QPushButton * adapi_change_;
  QLabel * adapi_response_;
  QCheckBox * adapi_auto_clear_;
  QGroupBox * waypoints_group_;
  QCheckBox * allow_goal_modification_;
  QPushButton * planning_enable_;
  QLabel * planning_enable_state_;
  QLabel * planning_estop_state_;
  QLabel * planning_cmd_vel_state_;

  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_engaged_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_engage_;
  rclcpp::TimerBase::SharedPtr planning_status_timer_;

  bool planning_engaged_{false};
  bool planning_estop_{false};
  bool planning_has_cmd_vel_{false};
  rclcpp::Time planning_last_cmd_vel_stamp_{0, 0, RCL_ROS_TIME};

  std::vector<PoseStamped> waypoints_;
  void onPose(const PoseStamped::ConstSharedPtr msg);
  void onPlanningEngaged(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void onPlanningEstop(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void onPlanningCmdVel(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void updatePlanningStateLabels();

  enum AdapiMode { Set, Change };
  AdapiMode adapi_mode_;

  autoware::component_interface_utils::Client<ClearRoute>::SharedPtr cli_clear_;
  autoware::component_interface_utils::Client<SetRoutePoints>::SharedPtr cli_set_;
  autoware::component_interface_utils::Client<ChangeRoutePoints>::SharedPtr cli_change_;
  void requestRoute(const PoseStamped & pose);
  void asyncSendRequest(SetRoutePoints::Service::Request::SharedPtr req);

private slots:
  void clearRoute();
  void onWaypointsMode(bool clicked);
  void onWaypointsReset();
  void onWaypointsApply();
  void onPlanningEnableToggled(bool enabled);
};

}  // namespace tier4_adapi_rviz_plugins

#endif  // ROUTE_PANEL_HPP_
