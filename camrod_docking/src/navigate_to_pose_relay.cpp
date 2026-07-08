// YH_260706: NavigateToPose action relay for opennav_docking Phase 1 staging.
//
// 문제: opennav_docking 의 내부 navigator 는 상대명 "navigate_to_pose" 로 액션
//   클라이언트를 생성한다. docking 네임스페이스(/docking) 하에서 이는
//   /docking/navigate_to_pose 로 해석되는데, launch 의 노드 레벨 remap
//   (-r navigate_to_pose:=/planning/navigate_to_pose) 이 opennav 내부 클라이언트에는
//   적용되지 않아(apt 바이너리, 액션명 파라미터 없음) 서버를 못 찾고 Phase 1 이
//   즉시 FailedToStage(error 903) 로 실패한다.
//
// 해결: /docking/navigate_to_pose 에 서버를 제공하고 실제 Nav2 서버
//   (/planning/navigate_to_pose)로 goal/feedback/result/cancel 을 그대로 중계한다.
//   sim(docking_phase1_sim.launch.py)·실차(docking.launch.py) 모두 동일 opennav
//   바이너리를 쓰므로 두 런치에서 이 노드를 함께 띄운다.

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace camrod_docking
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class NavigateToPoseRelay : public rclcpp::Node
{
public:
  NavigateToPoseRelay()
  : rclcpp::Node("navigate_to_pose_relay")
  {
    // 상대명. docking 네임스페이스 하에서 /docking/navigate_to_pose 로 해석되어
    // opennav 내부 클라이언트가 찾는 이름과 일치한다.
    input_action_ = declare_parameter<std::string>("input_action", "navigate_to_pose");
    // 실제 Nav2 서버(절대명). CAMROD 풀스택은 /planning/navigate_to_pose.
    output_action_ = declare_parameter<std::string>("output_action", "/planning/navigate_to_pose");
    server_wait_timeout_ = declare_parameter<double>("server_wait_timeout", 5.0);

    client_ = rclcpp_action::create_client<NavigateToPose>(this, output_action_);

    server_ = rclcpp_action::create_server<NavigateToPose>(
      this, input_action_,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<ServerGoalHandle>) {
        std::lock_guard<std::mutex> lock(gh_mutex_);
        if (down_gh_) {client_->async_cancel_goal(down_gh_);}
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<ServerGoalHandle> gh) {
        std::thread([this, gh]() {execute(gh);}).detach();
      });

    RCLCPP_INFO(get_logger(),
      "NavigateToPoseRelay ready: %s (server) -> %s (client)",
      input_action_.c_str(), output_action_.c_str());
  }

private:
  void execute(std::shared_ptr<ServerGoalHandle> gh)
  {
    auto result = std::make_shared<NavigateToPose::Result>();

    if (!client_->wait_for_action_server(
        std::chrono::duration<double>(server_wait_timeout_)))
    {
      RCLCPP_ERROR(get_logger(), "downstream server unavailable: %s", output_action_.c_str());
      gh->abort(result);
      return;
    }

    // downstream goal 구성 (upstream goal 그대로 전달)
    auto down_goal = *gh->get_goal();

    std::promise<ClientGoalHandle::SharedPtr> accept_promise;
    auto accept_future = accept_promise.get_future();
    std::promise<ClientGoalHandle::WrappedResult> result_promise;
    auto result_future = result_promise.get_future();

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback =
      [&accept_promise](const ClientGoalHandle::SharedPtr & h) {accept_promise.set_value(h);};
    // feedback 을 upstream 으로 그대로 중계
    opts.feedback_callback =
      [this, &gh](ClientGoalHandle::SharedPtr,
                  const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        if (gh->is_canceling()) {return;}
        gh->publish_feedback(std::make_shared<NavigateToPose::Feedback>(*fb));
      };
    opts.result_callback =
      [&result_promise](const ClientGoalHandle::WrappedResult & wr) {result_promise.set_value(wr);};

    client_->async_send_goal(down_goal, opts);

    if (accept_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "timed out waiting for downstream goal acceptance");
      gh->abort(result);
      return;
    }
    auto down_gh = accept_future.get();
    if (!down_gh) {
      RCLCPP_WARN(get_logger(), "downstream goal rejected");
      gh->abort(result);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(gh_mutex_);
      down_gh_ = down_gh;
    }

    // result 대기 (cancel 요청 100ms 주기로 폴링)
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      if (gh->is_canceling()) {
        std::lock_guard<std::mutex> lock(gh_mutex_);
        if (down_gh_) {client_->async_cancel_goal(down_gh_);}
        result_future.wait();
        break;
      }
    }

    const auto wrapped = result_future.get();
    {
      std::lock_guard<std::mutex> lock(gh_mutex_);
      down_gh_.reset();
    }
    *result = *wrapped.result;

    switch (wrapped.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        gh->succeed(result);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        gh->canceled(result);
        break;
      default:
        gh->abort(result);
        break;
    }
  }

  std::string input_action_;
  std::string output_action_;
  double server_wait_timeout_{5.0};

  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  std::mutex gh_mutex_;
  ClientGoalHandle::SharedPtr down_gh_;
};

}  // namespace camrod_docking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camrod_docking::NavigateToPoseRelay>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
