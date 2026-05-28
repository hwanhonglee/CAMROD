#ifndef CAMROD_DOCKING__MANUAL_DOCK_SERVER_NODE_HPP_
#define CAMROD_DOCKING__MANUAL_DOCK_SERVER_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "avg_msgs/action/manual_dock.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"

namespace camrod_docking
{

/**
 * ManualDockServerNode
 *
 * avg_msgs/action/ManualDock Action Server.
 * UI 버튼 요청을 받아 opennav_docking의 DockRobot action 하나에 위임한다.
 *
 * navigate_to_staging_pose 파라미터(기본 true):
 *   true  — docking_server가 Nav2를 통해 staging 위치까지 자율 이동 후 도킹
 *   false — 로봇이 이미 staging 위치에 있다고 가정, Nav2 호출 생략 후 바로 도킹 접근
 *           (Nav2·SLAM 없이 카메라+로봇만으로 도킹 단계 단독 테스트 가능)
 *
 * 상태머신(planning_state_machine)과 완전히 독립적으로 동작한다.
 * E-Stop 발생 시 cmd_vel_gate 차단 → docking_server 내부 타임아웃 →
 * DockRobot result FAILED → abort() 자동 전파.
 */
class ManualDockServerNode : public rclcpp::Node
{
public:
  using ManualDock = avg_msgs::action::ManualDock;
  using DockRobot  = opennav_docking_msgs::action::DockRobot;
  using ManualDockGoalHandle = rclcpp_action::ServerGoalHandle<ManualDock>;
  using DockRobotGoalHandle  = rclcpp_action::ClientGoalHandle<DockRobot>;

  explicit ManualDockServerNode();

private:
  // ── Action Server 콜백 ────────────────────────────────────────────────────
  rclcpp_action::GoalResponse _handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ManualDock::Goal> goal);

  rclcpp_action::CancelResponse _handle_cancel(
    std::shared_ptr<ManualDockGoalHandle> goal_handle);

  void _handle_accepted(
    std::shared_ptr<ManualDockGoalHandle> goal_handle);

  // ── 시퀀스 실행 (별도 스레드) ─────────────────────────────────────────────
  void _execute(std::shared_ptr<ManualDockGoalHandle> goal_handle);

  // ── 파라미터 ──────────────────────────────────────────────────────────────
  std::string action_server_name_;
  std::string dock_action_name_;
  bool navigate_to_staging_pose_;

  // ── ROS 인터페이스 ────────────────────────────────────────────────────────
  rclcpp_action::Server<ManualDock>::SharedPtr action_server_;
  rclcpp_action::Client<DockRobot>::SharedPtr  dock_client_;

  // ── 상태 ──────────────────────────────────────────────────────────────────
  std::atomic<bool> active_{false};
  std::mutex dock_gh_mutex_;
  DockRobotGoalHandle::SharedPtr dock_goal_handle_;
};

}  // namespace camrod_docking

#endif  // CAMROD_DOCKING__MANUAL_DOCK_SERVER_NODE_HPP_
