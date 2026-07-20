/**
 * Localization Init Checker Node
 *
 * drop_zone_matcher_node 가 발행하는 초기화 상태 토픽들을 구독하여
 * 로봇이 올바른 위치에서 시작했는지 /diagnostics 토픽으로 발행한다.
 *
 * 로컬라이제이션이 올바른 drop zone 에서 초기화되지 않으면
 * 이후 모든 위치 추정이 잘못된 기준점에서 출발한다.
 *
 * 진단 항목
 * ---------
 *   /localization/init
 *     - Staleness         : 초기화 상태 토픽 미수신 경과 시간
 *     - Match OK          : drop_zone/match_ok = false 일 때
 *                           → grace_period 이내  : WARN (아직 초기화 중)
 *                           → grace_period 초과  : ERROR (drop zone 매칭 실패)
 *     - Match distance    : 가장 가까운 zone 까지의 거리
 *                           → dist_warn_m 초과   : WARN
 *                           → dist_error_m 초과  : ERROR (zone 범위 밖)
 *
 * 파라미터 구성
 * -------------
 *   ok_topic:         "/localization/drop_zone/match_ok"
 *   distance_topic:   "/localization/drop_zone/match_distance"
 *   id_topic:         "/localization/drop_zone/match_id"
 *   stale_timeout:    5.0    # drop_zone_matcher 가 늦게 뜰 수 있으므로 여유있게
 *   grace_period_s:   30.0   # 시동 후 이 시간 이내 미매칭 → WARN (이후 → ERROR)
 *   dist_warn_m:      3.0    # 거리 > 이 값 → WARN
 *   dist_error_m:     8.0    # 거리 > 이 값 → ERROR (zone 밖으로 봄)
 */

#include <cmath>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
// HH_260720 - Diagnose generated CAMROD initialization state values.
#include <avg_msgs/msg/avg_bool.hpp>
#include <avg_msgs/msg/avg_float32.hpp>
#include <avg_msgs/msg/avg_string.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── 초기화 상태 구조체 ────────────────────────────────────────────────────

struct InitState
{
  std::mutex mtx;

  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};

  bool   match_ok{false};
  float  match_distance{std::numeric_limits<float>::infinity()};
  std::string match_id;

  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr    ok_sub;
  rclcpp::Subscription<avg_msgs::msg::AvgFloat32>::SharedPtr dist_sub;
  rclcpp::Subscription<avg_msgs::msg::AvgString>::SharedPtr  id_sub;
};

// ── LocalizationInitCheckerNode ───────────────────────────────────────────

class LocalizationInitCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  LocalizationInitCheckerNode()
  : robot_diagnostics_base::BaseChecker(
      "localization_init_checker", "localization_init_checker"),
    node_start_time_(rclcpp::Clock(RCL_ROS_TIME).now())
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("enabled", true);
    declare_parameter("ok_topic",        std::string("/localization/drop_zone/match_ok"));
    declare_parameter("distance_topic",  std::string("/localization/drop_zone/match_distance"));
    declare_parameter("id_topic",        std::string("/localization/drop_zone/match_id"));
    declare_parameter("stale_timeout_s",   5.0);
    declare_parameter("grace_period_s", 30.0);
    declare_parameter("dist_warn_m",     3.0);
    declare_parameter("dist_error_m",    8.0);
    declare_parameter("warn_during_grace", true);
  }

  void load_parameters_() override
  {
    enabled_         = get_parameter("enabled").as_bool();
    ok_topic_         = get_parameter("ok_topic").as_string();
    distance_topic_   = get_parameter("distance_topic").as_string();
    id_topic_         = get_parameter("id_topic").as_string();
    stale_timeout_ = get_param<double>("stale_timeout_s", stale_timeout_);
    grace_period_s_ = get_param<double>(
      "grace_period_s", grace_period_s_);
    dist_warn_m_      = get_parameter("dist_warn_m").as_double();
    dist_error_m_     = get_parameter("dist_error_m").as_double();
    warn_during_grace_ = get_parameter("warn_during_grace").as_bool();
  }

  void setup_tasks_() override
  {
    state_.ok_sub = create_subscription<avg_msgs::msg::AvgBool>(
      ok_topic_, rclcpp::QoS(1),
      [this](const avg_msgs::msg::AvgBool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.last_msg_time = this->now();
        state_.has_msg       = true;
        state_.match_ok      = msg->data;
      });

    state_.dist_sub = create_subscription<avg_msgs::msg::AvgFloat32>(
      distance_topic_, rclcpp::QoS(1),
      [this](const avg_msgs::msg::AvgFloat32::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.match_distance = msg->data;
      });

    state_.id_sub = create_subscription<avg_msgs::msg::AvgString>(
      id_topic_, rclcpp::QoS(1),
      [this](const avg_msgs::msg::AvgString::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.match_id = msg->data;
      });

    add_task("/localization/init",
      [this](StatusWrapper & stat) { checkInit(stat); });

    RCLCPP_INFO(get_logger(),
      "Localization init checker started "
      "(enabled=%s, grace=%.0fs, dist_warn=%.1fm, dist_error=%.1fm)",
      enabled_ ? "true" : "false", grace_period_s_, dist_warn_m_, dist_error_m_);
  }

private:
  void checkInit(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    if (!enabled_) {
      stat.summary(DiagnosticStatus::OK, "Drop zone init check disabled");
      stat.add("enabled", "false");
      return;
    }

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!state_.has_msg) {
      double since_start =
        (rclcpp::Clock(RCL_ROS_TIME).now() - node_start_time_).seconds();
      if (since_start > stale_timeout_) {
        stat.summary(DiagnosticStatus::STALE,
          "No topic messages: " + ok_topic_);
        stat.add("topic", ok_topic_);
      } else {
        // 노드 시작 직후 drop_zone_matcher 가 아직 안 뜬 경우
        stat.summary(DiagnosticStatus::WARN,
          "Waiting for init status before drop_zone_matcher starts");
        stat.add("since_node_start_s", since_start);
      }
      return;
    }

    double elapsed_since_msg =
      (this->now() - state_.last_msg_time).seconds();

    if (elapsed_since_msg > stale_timeout_) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "No messages for %.1fs (timeout=%.1fs)",
        elapsed_since_msg, stale_timeout_);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_age_s", elapsed_since_msg);
      return;
    }

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    double since_start =
      (rclcpp::Clock(RCL_ROS_TIME).now() - node_start_time_).seconds();

    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str;

    if (state_.match_ok) {
      // 매칭 성공 → OK. 거리 품질도 추가로 체크
      lvl     = DiagnosticStatus::OK;
      msg_str = "Drop zone matching complete";
      if (!state_.match_id.empty()) {
        msg_str += " (id=" + state_.match_id + ")";
      }
    } else {
      // 미매칭: grace period 내 → WARN, 초과 → ERROR
      if (since_start < grace_period_s_) {
        // HH_260617: Sim profile can keep startup grace at OK so localization
        // dummy data does not block planning/control smoke tests.
        lvl = warn_during_grace_ ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
        char buf[80];
        std::snprintf(buf, sizeof(buf),
          "Initializing (%.0fs / grace=%.0fs)",
          since_start, grace_period_s_);
        msg_str = buf;
      } else {
        lvl     = DiagnosticStatus::ERROR;
        msg_str = "Drop zone matching failed: grace period exceeded";
      }
    }

    // 거리 체크 (매칭 여부와 무관)
    if (std::isfinite(state_.match_distance)) {
      if (state_.match_distance > static_cast<float>(dist_error_m_) &&
          lvl < DiagnosticStatus::ERROR)
      {
        lvl     = DiagnosticStatus::ERROR;
        msg_str = "Drop zone distance exceeded: outside zone";
      } else if (state_.match_distance > static_cast<float>(dist_warn_m_) &&
                 lvl < DiagnosticStatus::WARN)
      {
        lvl     = DiagnosticStatus::WARN;
        msg_str = "Drop zone distance high: near zone boundary";
      }
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    stat.add("enabled",         "true");
    stat.add("match_ok",        state_.match_ok ? "true" : "false");
    stat.add("match_id",        state_.match_id.empty() ? "(none)" : state_.match_id);

    char tmp[48];
    if (std::isfinite(state_.match_distance)) {
      std::snprintf(tmp, sizeof(tmp), "%.3f", state_.match_distance);
    } else {
      std::snprintf(tmp, sizeof(tmp), "inf");
    }
    stat.add("match_distance_m",  std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", dist_warn_m_);
    stat.add("dist_warn_m",       std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", dist_error_m_);
    stat.add("dist_error_m",      std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", since_start);
    stat.add("since_node_start_s", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", grace_period_s_);
    // HH_260617: Publish diagnostic detail keys with canonical `_s` time suffix.
    stat.add("grace_period_s",  std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed_since_msg);
    stat.add("last_msg_age_s",  std::string(tmp));
  }

  // 파라미터
  std::string ok_topic_;
  std::string distance_topic_;
  std::string id_topic_;
  bool enabled_{true};
  double stale_timeout_{5.0};
  double grace_period_s_{30.0};
  double dist_warn_m_{3.0};
  double dist_error_m_{8.0};
  bool warn_during_grace_{true};

  rclcpp::Time node_start_time_;
  InitState    state_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationInitCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
