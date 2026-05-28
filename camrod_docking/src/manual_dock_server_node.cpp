#include "camrod_docking/manual_dock_server_node.hpp"

#include <chrono>
#include <future>
#include <string>

namespace camrod_docking
{

ManualDockServerNode::ManualDockServerNode()
: rclcpp::Node("manual_dock_server")
{
  action_server_name_ = declare_parameter<std::string>(
    "action_server_name", "/docking/manual_dock");
  dock_action_name_ = declare_parameter<std::string>(
    "dock_action_name", "/docking/dock_robot");
  navigate_to_staging_pose_ = declare_parameter<bool>(
    "navigate_to_staging_pose", true);

  // Action Server
  action_server_ = rclcpp_action::create_server<ManualDock>(
    this,
    action_server_name_,
    [this](const rclcpp_action::GoalUUID & uuid,
           std::shared_ptr<const ManualDock::Goal> goal) {
      return _handle_goal(uuid, goal);
    },
    [this](std::shared_ptr<ManualDockGoalHandle> gh) {
      return _handle_cancel(gh);
    },
    [this](std::shared_ptr<ManualDockGoalHandle> gh) {
      _handle_accepted(gh);
    });

  // DockRobot Action Client
  dock_client_ = rclcpp_action::create_client<DockRobot>(this, dock_action_name_);

  RCLCPP_INFO(get_logger(),
    "ManualDockServerNode ready (server=%s, dock_action=%s, navigate_to_staging=%s)",
    action_server_name_.c_str(), dock_action_name_.c_str(),
    navigate_to_staging_pose_ ? "true" : "false");
}

// ── Goal 수락/거부 ────────────────────────────────────────────────────────────

rclcpp_action::GoalResponse ManualDockServerNode::_handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const ManualDock::Goal> goal)
{
  if (active_.load()) {
    RCLCPP_WARN(get_logger(),
      "manual dock already in progress — rejecting new goal (dock_id=%s)",
      goal->dock_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->dock_id.empty()) {
    RCLCPP_WARN(get_logger(), "dock_id is empty — rejecting goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "manual dock goal accepted (dock_id=%s)", goal->dock_id.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// ── Cancel 요청 ───────────────────────────────────────────────────────────────

rclcpp_action::CancelResponse ManualDockServerNode::_handle_cancel(
  std::shared_ptr<ManualDockGoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "manual dock cancel requested");
  std::lock_guard<std::mutex> lock(dock_gh_mutex_);
  if (dock_goal_handle_) {
    dock_client_->async_cancel_goal(dock_goal_handle_);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// ── 실행 스레드 시작 ──────────────────────────────────────────────────────────

void ManualDockServerNode::_handle_accepted(
  std::shared_ptr<ManualDockGoalHandle> goal_handle)
{
  active_ = true;
  std::thread([this, goal_handle]() { _execute(goal_handle); }).detach();
}

// ── 시퀀스 실행 ───────────────────────────────────────────────────────────────

void ManualDockServerNode::_execute(
  std::shared_ptr<ManualDockGoalHandle> goal_handle)
{
  auto result = std::make_shared<ManualDock::Result>();
  const auto goal = goal_handle->get_goal();
  const auto start_time = this->now();

  auto finish = [&](bool success, uint16_t error_code, const std::string & msg) {
    result->success = success;
    result->error_code = error_code;
    result->message = msg;
    result->total_elapsed_sec =
      static_cast<float>((this->now() - start_time).seconds());
    active_ = false;
    {
      std::lock_guard<std::mutex> lock(dock_gh_mutex_);
      dock_goal_handle_.reset();
    }
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else if (success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
    RCLCPP_INFO(get_logger(), "manual dock finished: %s (%.1fs)",
      msg.c_str(), result->total_elapsed_sec);
  };

  // docking_server action 서버 대기
  if (!dock_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "docking_server action not available: %s",
      dock_action_name_.c_str());
    finish(false, 0, "docking_server action not available: " + dock_action_name_);
    return;
  }

  // DockRobot goal 구성
  auto dock_goal = DockRobot::Goal();
  dock_goal.use_dock_id = true;
  dock_goal.dock_id = goal->dock_id;
  dock_goal.navigate_to_staging_pose = navigate_to_staging_pose_;
  dock_goal.max_staging_time = goal->max_staging_time;

  // Feedback relay: DockRobot → ManualDock
  auto feedback_cb = [this, &goal_handle](
    DockRobotGoalHandle::SharedPtr,
    const std::shared_ptr<const DockRobot::Feedback> fb)
  {
    if (goal_handle->is_canceling()) {return;}
    auto feedback = std::make_shared<ManualDock::Feedback>();
    feedback->state = fb->state;
    feedback->docking_time = fb->docking_time;
    feedback->num_retries = fb->num_retries;
    switch (fb->state) {
      case DockRobot::Feedback::NAV_TO_STAGING_POSE:
        feedback->phase_label = "NAVIGATING_TO_STAGING";
        break;
      case DockRobot::Feedback::INITIAL_PERCEPTION:
        feedback->phase_label = "INITIAL_PERCEPTION";
        break;
      case DockRobot::Feedback::CONTROLLING:
        feedback->phase_label = "DOCKING";
        break;
      case DockRobot::Feedback::WAIT_FOR_CHARGE:
        feedback->phase_label = "WAIT_FOR_CHARGE";
        break;
      case DockRobot::Feedback::RETRY:
        feedback->phase_label = "RETRY";
        break;
      default:
        feedback->phase_label = "UNKNOWN";
        break;
    }
    goal_handle->publish_feedback(feedback);
  };

  // DockRobot goal 전송 (goal_response + result 비동기)
  std::promise<DockRobotGoalHandle::SharedPtr> goal_accept_promise;
  auto goal_accept_future = goal_accept_promise.get_future();

  std::promise<DockRobotGoalHandle::WrappedResult> result_promise;
  auto result_future = result_promise.get_future();

  auto send_options = rclcpp_action::Client<DockRobot>::SendGoalOptions();
  send_options.goal_response_callback =
    [&goal_accept_promise](const DockRobotGoalHandle::SharedPtr & gh) {
      goal_accept_promise.set_value(gh);
    };
  send_options.feedback_callback = feedback_cb;
  send_options.result_callback =
    [&result_promise](const DockRobotGoalHandle::WrappedResult & wr) {
      result_promise.set_value(wr);
    };

  dock_client_->async_send_goal(dock_goal, send_options);

  // goal 수락 대기 (최대 10초)
  if (goal_accept_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
    finish(false, 0, "timed out waiting for DockRobot goal acceptance");
    return;
  }

  auto dock_gh = goal_accept_future.get();
  if (!dock_gh) {
    finish(false, 0, "DockRobot goal rejected by docking_server");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(dock_gh_mutex_);
    dock_goal_handle_ = dock_gh;
  }

  RCLCPP_INFO(get_logger(), "DockRobot goal accepted, docking started (dock_id=%s)",
    goal->dock_id.c_str());

  // result 대기 — cancel 요청 100ms 주기로 폴링
  while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(get_logger(), "cancel detected — cancelling DockRobot goal");
      {
        std::lock_guard<std::mutex> lock(dock_gh_mutex_);
        if (dock_goal_handle_) {
          dock_client_->async_cancel_goal(dock_goal_handle_);
        }
      }
      result_future.wait();
      break;
    }
  }

  // cancel된 경우
  if (goal_handle->is_canceling()) {
    finish(false, 0, "cancelled by user");
    return;
  }

  // result 처리
  const auto wrapped = result_future.get();
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped.result->success)
  {
    finish(true, 0, "docked successfully at " + goal->dock_id);
  } else {
    const uint16_t ec = wrapped.result ? wrapped.result->error_code : 0;
    finish(false, ec,
      "docking failed (dock_id=" + goal->dock_id +
      ", error_code=" + std::to_string(ec) + ")");
  }
}

}  // namespace camrod_docking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camrod_docking::ManualDockServerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
