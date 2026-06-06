/**
 * Planning Lifecycle Checker Node
 *
 * Nav2 lifecycle 노드들의 /get_state 서비스를 비동기 폴링하여
 * 각 노드가 ACTIVE 상태인지 /diagnostics 로 발행한다.
 *
 * 진단 항목 (node_names 목록의 각 노드)
 * --------------------------------------
 *   /planning/lifecycle/{node_name}
 *     - Service ready : /get_state 서비스 미응답 → ERROR
 *     - State         : ACTIVE(3)      → OK
 *                       INACTIVE(2)    → WARN
 *                       UNCONFIGURED(1)/FINALIZED(4) → ERROR
 *
 * 파라미터 구성
 * -------------
 *   node_names:
 *     - "/planning/planner_server"
 *     - "/planning/controller_server"
 *     - "/planning/bt_navigator"
 *     - "/planning/behavior_server"
 *   poll_rate_hz:    2.0  # lifecycle state 폴링 주기 (Hz)
 *   stale_timeout_s: 5.0  # 마지막 응답 후 이 시간 초과 시 ERROR
 */

#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;
using GetState         = lifecycle_msgs::srv::GetState;
using LifecycleState   = lifecycle_msgs::msg::State;

// ── lifecycle 노드 1개의 상태 ─────────────────────────────────────────────

struct LifecycleNodeState
{
  std::string name;

  // 폴링 클라이언트
  rclcpp::Client<GetState>::SharedPtr client;

  // 캐시된 상태 (mutex 보호)
  std::mutex  mtx;
  bool        has_response{false};
  uint8_t     state_id{LifecycleState::PRIMARY_STATE_UNKNOWN};
  std::string state_label{"UNKNOWN"};
  rclcpp::Time last_response_time{0, 0, RCL_ROS_TIME};
};

// ── PlanningLifecycleCheckerNode ──────────────────────────────────────────

class PlanningLifecycleCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  PlanningLifecycleCheckerNode()
  : robot_diagnostics_base::BaseChecker(
      "planning_lifecycle_checker", "planning_lifecycle_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("node_names",   std::vector<std::string>{
      "/planning/planner_server",
      "/planning/controller_server",
      "/planning/bt_navigator",
      "/planning/behavior_server",
    });
    declare_parameter("poll_rate_hz",    2.0);
    declare_parameter("stale_timeout_s", 5.0);
  }

  void load_parameters_() override
  {
    node_names_    = get_parameter("node_names").as_string_array();
    poll_rate_ = get_param<double>("poll_rate_hz", poll_rate_);
    stale_timeout_ = get_param<double>("stale_timeout_s", stale_timeout_);
  }

  void setup_tasks_() override
  {
    for (const auto & name : node_names_) {
      auto state = std::make_shared<LifecycleNodeState>();
      state->name   = name;
      state->client = create_client<GetState>(name + "/get_state");

      // 진단 task 등록 (태스크 이름에서 슬래시를 그대로 사용)
      add_task("/planning/lifecycle" + name,
        [this, state](StatusWrapper & stat) { checkNode(stat, state); });

      nodes_.push_back(state);

      RCLCPP_INFO(get_logger(),
        "Planning Lifecycle 모니터링 등록: %s", name.c_str());
    }

    // 폴링 타이머 (diagnostic updater 와 별도)
    auto poll_period = std::chrono::duration<double>(1.0 / poll_rate_);
    poll_timer_ = create_wall_timer(poll_period,
      [this]() { pollAllNodes(); });
  }

private:
  // ── 비동기 폴링 ────────────────────────────────────────────────────────

  void pollAllNodes()
  {
    for (auto & node : nodes_) {
      if (!node->client->service_is_ready()) {
        // 서비스 미응답 → 캐시 유지, has_response 는 false 유지
        continue;
      }

      auto req = std::make_shared<GetState::Request>();
      node->client->async_send_request(req,
        [this, node](rclcpp::Client<GetState>::SharedFuture future) {
          auto response = future.get();
          std::lock_guard<std::mutex> lock(node->mtx);
          node->state_id        = response->current_state.id;
          node->state_label     = response->current_state.label;
          node->has_response    = true;
          node->last_response_time = this->now();
        });
    }
  }

  // ── 진단 체크 (캐시 읽기) ─────────────────────────────────────────────

  void checkNode(
    StatusWrapper & stat,
    const std::shared_ptr<LifecycleNodeState> & node)
  {
    std::lock_guard<std::mutex> lock(node->mtx);

    // 서비스가 아직 한번도 응답 안 한 경우
    if (!node->has_response) {
      if (!node->client->service_is_ready()) {
        stat.summary(DiagnosticStatus::ERROR,
          node->name + " — get_state 서비스 없음 (노드 미기동)");
      } else {
        stat.summary(DiagnosticStatus::STALE,
          node->name + " — 응답 대기 중");
      }
      stat.add("node", node->name);
      return;
    }

    // Stale 체크 (마지막 응답 후 일정 시간 초과)
    double elapsed = (this->now() - node->last_response_time).seconds();
    if (elapsed > stale_timeout_) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "%s — 폴링 응답 없음 (%.1fs > %.1fs)",
        node->name.c_str(), elapsed, stale_timeout_);
      stat.summary(DiagnosticStatus::ERROR, std::string(buf));
      stat.add("node",              node->name);
      stat.add("last_response_sec", elapsed);
      return;
    }

    // State 판정
    int8_t     lvl;
    std::string msg_str;

    switch (node->state_id) {
      case LifecycleState::PRIMARY_STATE_ACTIVE:
        lvl     = DiagnosticStatus::OK;
        msg_str = node->name + " ACTIVE";
        break;
      case LifecycleState::PRIMARY_STATE_INACTIVE:
        lvl     = DiagnosticStatus::WARN;
        msg_str = node->name + " INACTIVE — 전환 중이거나 정지됨";
        break;
      case LifecycleState::PRIMARY_STATE_UNCONFIGURED:
        lvl     = DiagnosticStatus::ERROR;
        msg_str = node->name + " UNCONFIGURED — 초기화 안 됨";
        break;
      case LifecycleState::PRIMARY_STATE_FINALIZED:
        lvl     = DiagnosticStatus::ERROR;
        msg_str = node->name + " FINALIZED — 종료됨";
        break;
      default:
        lvl     = DiagnosticStatus::ERROR;
        msg_str = node->name + " UNKNOWN state";
        break;
    }

    stat.summary(lvl, msg_str);

    stat.add("node",       node->name);
    stat.add("state",      node->state_label);

    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%d", static_cast<int>(node->state_id));
    stat.add("state_id",   std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("polled_sec_ago", std::string(tmp));
  }

  // 파라미터
  std::vector<std::string> node_names_;
  double                   poll_rate_{2.0};
  double                   stale_timeout_{5.0};

  std::vector<std::shared_ptr<LifecycleNodeState>> nodes_;
  rclcpp::TimerBase::SharedPtr                     poll_timer_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningLifecycleCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
