/**
 * Localization Source Checker Node
 *
 * localization_pose_selector_node 가 발행하는 소스 토픽을 구독하여
 * 현재 로컬라이제이션 소스가 정상 경로(ESKF)인지 폴백 소스인지를
 * /diagnostics 토픽으로 발행한다.
 *
 * Primary = ESKF (정상), Fallback = fallback source (폴백)
 * 폴백 활성화는 곧 ESKF 이상 또는 GNSS/IMU 품질 저하를 의미한다.
 *
 * 진단 항목
 * ---------
 *   /localization/source
 *     - Staleness            : 토픽 미수신 경과 시간
 *     - Source OK            : "primary_eskf" → OK
 *                              "fallback_source" → WARN (폴백 활성)
 *                              그 외 값       → ERROR (알 수 없는 소스)
 *     - Fallback duration    : 폴백 활성 지속 시간
 *                              > fallback_warn_sec  → WARN
 *                              > fallback_error_sec → ERROR
 *     - Switch count         : 분당 소스 전환 횟수
 *                              > switch_warn  → WARN  (불안정 전환)
 *                              > switch_error → ERROR (과도한 전환)
 *
 * 파라미터 구성
 * -------------
 *   source_topic:          "/localization/pose_source"
 *   primary_source:        "primary_eskf"
 *   fallback_source:       "fallback_source"
 *   stale_timeout:         3.0
 *   fallback_warn_sec:     10.0   # 폴백 지속 > 이 값 → WARN
 *   fallback_error_sec:    60.0   # 폴백 지속 > 이 값 → ERROR
 *   switch_warn:           3      # 최근 60s 내 전환 횟수 > 이 값 → WARN
 *   switch_error:          10     # 최근 60s 내 전환 횟수 > 이 값 → ERROR
 */

#include <cmath>
#include <deque>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── 소스 상태 구조체 ─────────────────────────────────────────────────────

struct SourceState
{
  std::mutex mtx;

  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};

  std::string current_source;
  std::string prev_source;

  // 폴백 시작 시점 (폴백 전환 시 갱신)
  rclcpp::Time fallback_start_time{0, 0, RCL_ROS_TIME};
  bool in_fallback{false};

  // 최근 60s 내 전환 시각 기록
  std::deque<rclcpp::Time> switch_times;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
};

// ── LocalizationSourceCheckerNode ─────────────────────────────────────────

class LocalizationSourceCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  LocalizationSourceCheckerNode()
  : robot_diagnostics_base::BaseChecker(
      "localization_source_checker", "localization_source_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("source_topic",       std::string("/localization/pose_source"));
    declare_parameter("primary_source",     std::string("primary_eskf"));
    declare_parameter("fallback_source",    std::string("fallback_source"));
    declare_parameter("stale_timeout_s",      3.0);
    declare_parameter("fallback_warn_s",  10.0);
    declare_parameter("fallback_error_s", 60.0);
    declare_parameter("switch_warn",        3);
    declare_parameter("switch_error",       10);
  }

  void load_parameters_() override
  {
    source_topic_       = get_parameter("source_topic").as_string();
    primary_source_     = get_parameter("primary_source").as_string();
    fallback_source_    = get_parameter("fallback_source").as_string();
    stale_timeout_ = get_param_with_alias<double>("stale_timeout_s", stale_timeout_, {"stale_timeout"});
    fallback_warn_sec_  = get_param_with_alias<double>(
      "fallback_warn_s", fallback_warn_sec_, {"fallback_warn_sec"});
    fallback_error_sec_ = get_param_with_alias<double>(
      "fallback_error_s", fallback_error_sec_, {"fallback_error_sec"});
    switch_warn_        = get_parameter("switch_warn").as_int();
    switch_error_       = get_parameter("switch_error").as_int();
  }

  void setup_tasks_() override
  {
    // latched (transient_local) QoS — pose_selector 와 동일하게
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    state_.sub = create_subscription<std_msgs::msg::String>(
      source_topic_, qos,
      [this](const std_msgs::msg::String::ConstSharedPtr msg) {
        onSource(msg);
      });

    add_task("/localization/source",
      [this](StatusWrapper & stat) { checkSource(stat); });

    RCLCPP_INFO(get_logger(),
      "Localization Source 모니터링 시작 "
      "(primary=%s, fallback=%s, fallback_error=%.0fs)",
      primary_source_.c_str(), fallback_source_.c_str(), fallback_error_sec_);
  }

private:
  void onSource(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);
    auto now = this->now();

    const bool changed = (state_.has_msg && msg->data != state_.current_source);

    if (changed) {
      state_.switch_times.push_back(now);
      // 60s 이전 기록 제거
      while (!state_.switch_times.empty() &&
             (now - state_.switch_times.front()).seconds() > 60.0) {
        state_.switch_times.pop_front();
      }
    }

    const bool entering_fallback =
      (msg->data == fallback_source_) &&
      (state_.current_source != fallback_source_ || !state_.has_msg);

    if (entering_fallback) {
      state_.fallback_start_time = now;
      state_.in_fallback         = true;
    } else if (msg->data == primary_source_) {
      state_.in_fallback = false;
    }

    state_.prev_source    = state_.current_source;
    state_.current_source = msg->data;
    state_.last_msg_time  = now;
    state_.has_msg        = true;
  }

  void checkSource(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!state_.has_msg) {
      stat.summary(DiagnosticStatus::STALE, "토픽 수신 없음: " + source_topic_);
      stat.add("topic", source_topic_);
      return;
    }

    double elapsed = (this->now() - state_.last_msg_time).seconds();
    if (elapsed > stale_timeout_) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "%.1fs 동안 메시지 없음 (timeout=%.1fs)", elapsed, stale_timeout_);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── 소스 판정 ───────────────────────────────────────────────────────
    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str;

    if (state_.current_source == primary_source_) {
      lvl     = DiagnosticStatus::OK;
      msg_str = "Primary ESKF 사용 중";
    } else if (state_.current_source == fallback_source_) {
      lvl     = DiagnosticStatus::WARN;
      msg_str = "폴백(Fallback source) 활성화 — ESKF 이상";
    } else {
      lvl     = DiagnosticStatus::ERROR;
      msg_str = "알 수 없는 소스: " + state_.current_source;
    }

    // ── 폴백 지속 시간 체크 ─────────────────────────────────────────────
    double fallback_duration = 0.0;
    if (state_.in_fallback) {
      fallback_duration = (this->now() - state_.fallback_start_time).seconds();
      if (fallback_duration > fallback_error_sec_ && lvl < DiagnosticStatus::ERROR) {
        lvl     = DiagnosticStatus::ERROR;
        char buf[80];
        std::snprintf(buf, sizeof(buf),
          "폴백 장기 지속 (%.0fs > %.0fs)", fallback_duration, fallback_error_sec_);
        msg_str = buf;
      } else if (fallback_duration > fallback_warn_sec_ && lvl < DiagnosticStatus::WARN) {
        lvl     = DiagnosticStatus::WARN;
        char buf[80];
        std::snprintf(buf, sizeof(buf),
          "폴백 활성 (%.0fs / warn=%.0fs)", fallback_duration, fallback_warn_sec_);
        msg_str = buf;
      }
    }

    // ── 전환 횟수 체크 ──────────────────────────────────────────────────
    int switch_count = static_cast<int>(state_.switch_times.size());
    if (switch_count > switch_error_ && lvl < DiagnosticStatus::ERROR) {
      lvl     = DiagnosticStatus::ERROR;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "소스 과도 전환 (60s내 %d회 > %d)", switch_count, switch_error_);
      msg_str = buf;
    } else if (switch_count > switch_warn_ && lvl < DiagnosticStatus::WARN) {
      lvl     = DiagnosticStatus::WARN;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "소스 불안정 전환 (60s내 %d회 > %d)", switch_count, switch_warn_);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    stat.add("current_source",    state_.current_source);
    stat.add("primary_source",    primary_source_);
    stat.add("fallback_source",   fallback_source_);

    char tmp[48];
    std::snprintf(tmp, sizeof(tmp), "%d", switch_count);
    stat.add("switch_count_60s",  std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%d / %d", switch_warn_, switch_error_);
    stat.add("switch_warn/error", std::string(tmp));

    std::snprintf(tmp, sizeof(tmp), "%.1f", fallback_duration);
    stat.add("fallback_duration_sec", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.0f / %.0f", fallback_warn_sec_, fallback_error_sec_);
    stat.add("fallback_warn/error_sec", std::string(tmp));

    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago",  std::string(tmp));
  }

  // 파라미터
  std::string source_topic_;
  std::string primary_source_{"primary_eskf"};
  std::string fallback_source_{"fallback_source"};
  double stale_timeout_{3.0};
  double fallback_warn_sec_{10.0};
  double fallback_error_sec_{60.0};
  int    switch_warn_{3};
  int    switch_error_{10};

  SourceState state_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationSourceCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
