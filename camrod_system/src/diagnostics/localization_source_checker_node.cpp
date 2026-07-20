/**
 * HH_260721 - Diagnose whether the pose selector uses the primary EKF output
 * or its configured dead-reckoning fallback, including staleness and flapping.
 */

#include <cmath>
#include <deque>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
// HH_260720 - Diagnose the generated CAMROD localization-source label.
#include <avg_msgs/msg/avg_string.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

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

  rclcpp::Subscription<avg_msgs::msg::AvgString>::SharedPtr sub;
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
    declare_parameter("source_topic", std::string("/localization/pose_source"));
    // HH_260721 - Match the filter-neutral label published by the pose selector.
    declare_parameter("primary_source", std::string("primary_filter"));
    declare_parameter("fallback_source", std::string("fallback_source"));
    declare_parameter("stale_timeout_s", 3.0);
    declare_parameter("fallback_warn_s", 10.0);
    declare_parameter("fallback_error_s", 60.0);
    declare_parameter("switch_warn", 3);
    declare_parameter("switch_error", 10);
  }

  void load_parameters_() override
  {
    source_topic_ = get_parameter("source_topic").as_string();
    primary_source_ = get_parameter("primary_source").as_string();
    fallback_source_ = get_parameter("fallback_source").as_string();
    stale_timeout_ = get_param<double>("stale_timeout_s", stale_timeout_);
    fallback_warn_sec_ = get_param<double>(
      "fallback_warn_s", fallback_warn_sec_);
    fallback_error_sec_ = get_param<double>(
      "fallback_error_s", fallback_error_sec_);
    switch_warn_ = get_parameter("switch_warn").as_int();
    switch_error_ = get_parameter("switch_error").as_int();
  }

  void setup_tasks_() override
  {
    // latched (transient_local) QoS — pose_selector 와 동일하게
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

    state_.sub = create_subscription<avg_msgs::msg::AvgString>(
      source_topic_, qos,
      [this](const avg_msgs::msg::AvgString::ConstSharedPtr msg) {
        onSource(msg);
      });

    add_task(
      "/localization/source",
      [this](diagnostic_updater::DiagnosticStatusWrapper & stat) {checkSource(stat);});

    RCLCPP_INFO(
      get_logger(),
      "Localization source checker started "
      "(primary=%s, fallback=%s, fallback_error=%.0fs)",
      primary_source_.c_str(), fallback_source_.c_str(), fallback_error_sec_);
  }

private:
  void onSource(const avg_msgs::msg::AvgString::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);
    auto now = this->now();

    const bool changed = (state_.has_msg && msg->data != state_.current_source);

    if (changed) {
      state_.switch_times.push_back(now);
      // 60s 이전 기록 제거
      while (!state_.switch_times.empty() &&
        (now - state_.switch_times.front()).seconds() > 60.0)
      {
        state_.switch_times.pop_front();
      }
    }

    const bool entering_fallback =
      (msg->data == fallback_source_) &&
      (state_.current_source != fallback_source_ || !state_.has_msg);

    if (entering_fallback) {
      state_.fallback_start_time = now;
      state_.in_fallback = true;
    } else if (msg->data == primary_source_) {
      state_.in_fallback = false;
    }

    state_.prev_source = state_.current_source;
    state_.current_source = msg->data;
    state_.last_msg_time = now;
    state_.has_msg = true;
  }

  void checkSource(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!state_.has_msg) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::STALE,
        "No topic messages: " + source_topic_);
      stat.add("topic", source_topic_);
      return;
    }

    double elapsed = (this->now() - state_.last_msg_time).seconds();
    if (elapsed > stale_timeout_) {
      char buf[96];
      std::snprintf(
        buf, sizeof(buf),
        "No messages for %.1fs (timeout=%.1fs)", elapsed, stale_timeout_);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── 소스 판정 ───────────────────────────────────────────────────────
    int8_t lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg_str;

    if (state_.current_source == primary_source_) {
      lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
      msg_str = "Primary EKF filter active";
    } else if (state_.current_source == fallback_source_) {
      lvl = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg_str = "Fallback source active - primary EKF unavailable";
    } else {
      lvl = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg_str = "Unknown source: " + state_.current_source;
    }

    // ── 폴백 지속 시간 체크 ─────────────────────────────────────────────
    double fallback_duration = 0.0;
    if (state_.in_fallback) {
      fallback_duration = (this->now() - state_.fallback_start_time).seconds();
      if (fallback_duration > fallback_error_sec_ &&
        lvl < diagnostic_msgs::msg::DiagnosticStatus::ERROR)
      {
        lvl = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        char buf[80];
        std::snprintf(
          buf, sizeof(buf),
          "Fallback active too long (%.0fs > %.0fs)", fallback_duration, fallback_error_sec_);
        msg_str = buf;
      } else if (fallback_duration > fallback_warn_sec_ &&
        lvl < diagnostic_msgs::msg::DiagnosticStatus::WARN)
      {
        lvl = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        char buf[80];
        std::snprintf(
          buf, sizeof(buf),
          "Fallback active (%.0fs / warn=%.0fs)", fallback_duration, fallback_warn_sec_);
        msg_str = buf;
      }
    }

    // ── 전환 횟수 체크 ──────────────────────────────────────────────────
    int switch_count = static_cast<int>(state_.switch_times.size());
    if (switch_count > switch_error_ &&
      lvl < diagnostic_msgs::msg::DiagnosticStatus::ERROR)
    {
      lvl = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      char buf[80];
      std::snprintf(
        buf, sizeof(buf),
        "Excessive source switching (%d in 60s > %d)", switch_count, switch_error_);
      msg_str = buf;
    } else if (switch_count > switch_warn_ &&
      lvl < diagnostic_msgs::msg::DiagnosticStatus::WARN)
    {
      lvl = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      char buf[80];
      std::snprintf(
        buf, sizeof(buf),
        "Unstable source switching (%d in 60s > %d)", switch_count, switch_warn_);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    stat.add("current_source", state_.current_source);
    stat.add("primary_source", primary_source_);
    stat.add("fallback_source", fallback_source_);

    char tmp[48];
    std::snprintf(tmp, sizeof(tmp), "%d", switch_count);
    stat.add("switch_count_60s", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%d / %d", switch_warn_, switch_error_);
    stat.add("switch_warn/error", std::string(tmp));

    std::snprintf(tmp, sizeof(tmp), "%.1f", fallback_duration);
    stat.add("fallback_duration_sec", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.0f / %.0f", fallback_warn_sec_, fallback_error_sec_);
    stat.add("fallback_warn/error_sec", std::string(tmp));

    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago", std::string(tmp));
  }

  // 파라미터
  std::string source_topic_;
  std::string primary_source_{"primary_filter"};
  std::string fallback_source_{"fallback_source"};
  double stale_timeout_{3.0};
  double fallback_warn_sec_{10.0};
  double fallback_error_sec_{60.0};
  int switch_warn_{3};
  int switch_error_{10};

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
