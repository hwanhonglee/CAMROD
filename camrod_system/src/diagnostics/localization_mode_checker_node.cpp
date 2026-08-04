/**
 * Localization Mode Checker Node
 *
 * avg_msgs/AvgLocalizationStatus 토픽을 구독하여
 * 로컬라이제이션 전체 상태를 /diagnostics 토픽으로 발행한다.
 *
 * supervisor 가 이미 계산한 시스템 자가진단(mode·confidence·sensor 상태)을
 * Diagnostic 레벨로 노출한다.
 *
 * 진단 항목
 * ---------
 *   /localization/mode
 *     - Staleness         : status 토픽 미수신 경과 시간
 *     - Mode level        : NORMAL(0)=OK, DEGRADED(1)=WARN,
 *                           DR_ONLY(2)=WARN, INVALID(3)=ERROR
 *     - Confidence        : 신뢰도 [0.0, 1.0] 하한 체크
 *                           confidence < conf_warn → WARN
 *                           confidence < conf_error → ERROR
 *     - Sensor flags      : gnss_ok, imu_ok, wheel_ok false → WARN
 *     - Innovation norm   : GNSS/Wheel innovation norm 상한 체크
 *                           norm > innov_warn → WARN
 *                           norm > innov_error → ERROR
 *
 * 파라미터 구성
 * -------------
 *   status_topic:      "/localization/status"
 *   health_topic:      "/localization/state"
 *   stale_timeout:     2.0
 *   conf_warn:         0.6    # confidence < 이 값 → WARN
 *   conf_error:        0.3    # confidence < 이 값 → ERROR
 *   innov_warn:        3.0    # innovation norm > 이 값 → WARN
 *   innov_error:       6.0    # innovation norm > 이 값 → ERROR
 */

#include <cmath>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
// HH_260720 - Diagnose generated CAMROD localization health values.
#include <avg_msgs/msg/avg_bool.hpp>
#include <avg_msgs/msg/avg_localization_status.hpp>
#include <avg_msgs/msg/avg_localization_mode.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using StatusWrapper     = diagnostic_updater::DiagnosticStatusWrapper;
// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.

// ── 로컬라이제이션 모드 상태 구조체 ──────────────────────────────────────

struct ModeState
{
  std::mutex mtx;

  // status 토픽 수신 상태
  rclcpp::Time last_status_time{0, 0, RCL_ROS_TIME};
  bool has_status{false};

  // 최근 상태값
  uint8_t mode_value{avg_msgs::msg::AvgLocalizationMode::INVALID};
  std::string mode_label{"INVALID"};
  float confidence{0.0f};
  bool gnss_ok{false};
  bool imu_ok{false};
  bool wheel_ok{false};
  float gnss_innovation_norm{0.0f};
  float wheel_innovation_norm{0.0f};
  bool health{false};

  // 구독자
  rclcpp::Subscription<avg_msgs::msg::AvgLocalizationStatus>::SharedPtr status_sub;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr health_sub;
};

// ── LocalizationModeCheckerNode ───────────────────────────────────────────

class LocalizationModeCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  explicit LocalizationModeCheckerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : robot_diagnostics_base::BaseChecker(
      "localization_mode_checker", "localization_mode_checker", options)
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("status_topic",  std::string("/localization/status"));
    declare_parameter("health_topic",  std::string("/localization/state"));
    declare_parameter("stale_timeout_s", 2.0);
    declare_parameter("conf_warn",     0.6);
    declare_parameter("conf_error",    0.3);
    declare_parameter("innov_warn",    3.0);
    declare_parameter("innov_error",   6.0);
    declare_parameter("require_sensor_flags", true);
  }

  void load_parameters_() override
  {
    status_topic_  = get_parameter("status_topic").as_string();
    health_topic_  = get_parameter("health_topic").as_string();
    stale_timeout_ = get_param<double>("stale_timeout_s", stale_timeout_);
    conf_warn_     = get_parameter("conf_warn").as_double();
    conf_error_    = get_parameter("conf_error").as_double();
    innov_warn_    = get_parameter("innov_warn").as_double();
    innov_error_   = get_parameter("innov_error").as_double();
    require_sensor_flags_ = get_parameter("require_sensor_flags").as_bool();
  }

  void setup_tasks_() override
  {
    // ── Status 구독 ─────────────────────────────────────────────────────
    state_.status_sub = create_subscription<avg_msgs::msg::AvgLocalizationStatus>(
      status_topic_, rclcpp::QoS(10),
      [this](const avg_msgs::msg::AvgLocalizationStatus::ConstSharedPtr msg) { onStatus(msg); });

    // ── Health 구독 (transient_local QoS — supervisor 기본값) ───────────
    auto health_qos = rclcpp::QoS(1).transient_local();
    state_.health_sub = create_subscription<avg_msgs::msg::AvgBool>(
      health_topic_, health_qos,
      [this](const avg_msgs::msg::AvgBool::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.health = msg->data;
      });

    add_task("/localization/mode",
      [this](StatusWrapper & stat) { checkMode(stat); });

    RCLCPP_INFO(get_logger(),
      "Localization mode checker started (status=%s, conf_warn=%.2f, conf_error=%.2f)",
      status_topic_.c_str(), conf_warn_, conf_error_);
  }

private:
  void onStatus(const avg_msgs::msg::AvgLocalizationStatus::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);
    state_.last_status_time      = this->now();
    state_.has_status            = true;
    state_.mode_value            = msg->mode.value;
    state_.mode_label            = msg->mode.label.empty()
                                     ? modeLabel(msg->mode.value)
                                     : msg->mode.label;
    state_.confidence            = msg->confidence;
    state_.gnss_ok               = msg->gnss_ok;
    state_.imu_ok                = msg->imu_ok;
    state_.wheel_ok              = msg->wheel_ok;
    state_.gnss_innovation_norm  = msg->gnss_innovation_norm;
    state_.wheel_innovation_norm = msg->wheel_innovation_norm;
  }

  static std::string modeLabel(uint8_t v)
  {
    switch (v) {
      case avg_msgs::msg::AvgLocalizationMode::NORMAL:   return "NORMAL";
      case avg_msgs::msg::AvgLocalizationMode::DEGRADED: return "DEGRADED";
      case avg_msgs::msg::AvgLocalizationMode::DR_ONLY:  return "DR_ONLY";
      case avg_msgs::msg::AvgLocalizationMode::INVALID:  return "INVALID";
      default:                return "UNKNOWN";
    }
  }

  void checkMode(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!state_.has_status) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No topic messages: " + status_topic_);
      stat.add("topic", status_topic_);
      return;
    }

    double elapsed = (this->now() - state_.last_status_time).seconds();
    if (elapsed > stale_timeout_) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "No status for %.1fs (timeout=%.1fs)", elapsed, stale_timeout_);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    int8_t lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg_str;

    // 1. Mode 레벨 (최우선)
    switch (state_.mode_value) {
      case avg_msgs::msg::AvgLocalizationMode::NORMAL:
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::OK;
        msg_str = "NORMAL - localization healthy";
        break;
      case avg_msgs::msg::AvgLocalizationMode::DEGRADED:
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        msg_str = "DEGRADED - limited performance (GNSS/Wheel missing)";
        break;
      case avg_msgs::msg::AvgLocalizationMode::DR_ONLY:
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        msg_str = "DR_ONLY - dead reckoning only (GNSS missing)";
        break;
      case avg_msgs::msg::AvgLocalizationMode::INVALID:
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        msg_str = "INVALID - localization unavailable (IMU missing or all sensors failed)";
        break;
      default:
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        msg_str = "UNKNOWN mode value";
        break;
    }

    // 2. Confidence 체크
    if (state_.confidence < static_cast<float>(conf_error_) &&
        lvl < diagnostic_msgs::msg::DiagnosticStatus::ERROR)
    {
      lvl     = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg_str = "Confidence critically low (conf < " + std::to_string(conf_error_) + ")";
    } else if (state_.confidence < static_cast<float>(conf_warn_) &&
               lvl < diagnostic_msgs::msg::DiagnosticStatus::WARN)
    {
      lvl     = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg_str = "Confidence low (conf < " + std::to_string(conf_warn_) + ")";
    }

    // 3. Sensor flag 체크 (WARN 레벨)
    // HH_260617: Sim profiles can disable sensor flag enforcement because the
    // localization monitor may publish NORMAL confidence while hardware-specific
    // gnss/imu/wheel flags are intentionally synthetic or unavailable.
    if (require_sensor_flags_ && lvl < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      if (!state_.gnss_ok) {
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        msg_str = "GNSS input missing or invalid";
      } else if (!state_.wheel_ok) {
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        msg_str = "Wheel odometry input missing";
      } else if (!state_.imu_ok) {
        // IMU 없으면 supervisor에서 INVALID로 이미 처리됨
        // 여기선 다른 조건이 OK인 경우에만 추가 WARN
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        msg_str = "IMU input missing";
      }
    }

    // 4. Innovation norm 체크
    float max_innov = std::max(
      state_.gnss_innovation_norm, state_.wheel_innovation_norm);
    if (max_innov > static_cast<float>(innov_error_) &&
        lvl < diagnostic_msgs::msg::DiagnosticStatus::ERROR)
    {
      lvl     = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg_str = "Innovation norm critical (measurement rejection likely)";
    } else if (max_innov > static_cast<float>(innov_warn_) &&
               lvl < diagnostic_msgs::msg::DiagnosticStatus::WARN)
    {
      lvl     = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg_str = "Innovation norm high (fusion quality degraded)";
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    stat.add("mode",                   state_.mode_label);
    stat.add("mode_value",             static_cast<int>(state_.mode_value));

    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.3f", state_.confidence);
    stat.add("confidence",             std::string(tmp));

    stat.add("gnss_ok",                state_.gnss_ok  ? "true" : "false");
    stat.add("imu_ok",                 state_.imu_ok   ? "true" : "false");
    stat.add("wheel_ok",               state_.wheel_ok ? "true" : "false");
    stat.add("health",                 state_.health   ? "true" : "false");

    std::snprintf(tmp, sizeof(tmp), "%.3f", state_.gnss_innovation_norm);
    stat.add("gnss_innovation_norm",   std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", state_.wheel_innovation_norm);
    stat.add("wheel_innovation_norm",  std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago",       std::string(tmp));
  }

  // 파라미터
  std::string status_topic_;
  std::string health_topic_;
  double stale_timeout_{2.0};
  double conf_warn_{0.6};
  double conf_error_{0.3};
  double innov_warn_{3.0};
  double innov_error_{6.0};
  bool require_sensor_flags_{true};

  ModeState state_;
};

#include "camrod_system/checker_entrypoint.hpp"
CAMROD_SYSTEM_CHECKER_ENTRYPOINT(LocalizationModeCheckerNode)
