/**
 * Wheel Odometry Checker Node
 *
 * nav_msgs/Odometry 토픽을 구독하여 휠 오도메트리 상태를 /diagnostics 토픽으로 발행한다.
 * 센서 레벨 관제: 휠 오도메트리가 살아있는가? 속도 데이터가 유효한가?
 *
 * 진단 항목 (휠별 단일 태스크)
 * --------------------------------
 *   /sensor/wheel/{name}
 *     - Staleness       : 마지막 메시지 수신 후 경과 시간
 *     - Rate            : 2 초 rolling window 기반 실제 Hz
 *     - Velocity NaN    : twist.linear/angular 성분 NaN/Inf 여부
 *     - Max speed       : |vx| > max_speed_ms → WARN/ERROR (과속 또는 센서 이상)
 *
 * 파라미터 구성
 * -------------
 *   wheel_names: ["main"]
 *
 *   main:
 *     topic:              "/localization/input/wheel_odometry"
 *     expected_hz:        20.0
 *     hz_warn_ratio:      0.7
 *     hz_error_ratio:     0.4
 *     stale_timeout:      1.0
 *     max_speed_warn_ms:  3.0    # |vx| > 이 값 (m/s) → WARN  (0.0 = 비활성화)
 *     max_speed_error_ms: 5.0    # |vx| > 이 값 (m/s) → ERROR (0.0 = 비활성화)
 */

#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// HH_260720 - Diagnose generated CAMROD wheel odometry.
#include <avg_msgs/msg/avg_bool.hpp>
#include <avg_msgs/msg/avg_odometry.hpp>

#include <camrod_system/dummy_source_monitor.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── 휠 오도메트리 상태 구조체 ─────────────────────────────────────────────

struct WheelState
{
  // 설정
  std::string name;
  std::string topic;
  double expected_hz{20.0};
  double hz_warn_ratio{0.7};
  double hz_error_ratio{0.4};
  double stale_timeout{1.0};
  double max_speed_warn_ms{3.0};
  double max_speed_error_ms{5.0};
  std::string dummy_active_topic;
  double dummy_active_timeout{1.0};

  // 런타임 상태 (mutex 보호)
  std::mutex mtx;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};

  // 최근 측정값
  double vx{0.0}, vy{0.0}, vz{0.0};
  double wx{0.0}, wy{0.0}, wz{0.0};
  bool vel_nan{false};

  // Hz 계산용 rolling window (2s)
  std::deque<rclcpp::Time> timestamps;

  // 구독자
  rclcpp::Subscription<avg_msgs::msg::AvgOdometry>::SharedPtr sub;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr dummy_active_sub;
  camrod_system::DummySourceMonitor dummy_monitor;
};

static bool has_nan6(double a, double b, double c, double d, double e, double f)
{
  return !std::isfinite(a) || !std::isfinite(b) || !std::isfinite(c) ||
         !std::isfinite(d) || !std::isfinite(e) || !std::isfinite(f);
}

// ── WheelOdometryCheckerNode ──────────────────────────────────────────────

class WheelOdometryCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  explicit WheelOdometryCheckerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : robot_diagnostics_base::BaseChecker(
      "wheel_odometry_checker", "wheel_odometry_checker", options)
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("wheel_names", std::vector<std::string>{});
  }

  void load_parameters_() override
  {
    auto names = get_parameter("wheel_names").as_string_array();

    for (const auto & name : names) {
      auto wheel = std::make_shared<WheelState>();
      wheel->name = name;

      declare_parameter(name + ".topic",               std::string("/localization/input/wheel_odometry"));
      declare_parameter(name + ".expected_hz",         20.0);
      declare_parameter(name + ".hz_warn_ratio",       0.7);
      declare_parameter(name + ".hz_error_ratio",      0.4);
      declare_parameter(name + ".stale_timeout_s",       1.0);
      declare_parameter(name + ".max_speed_warn_ms",   3.0);
      declare_parameter(name + ".max_speed_error_ms",  5.0);
      declare_parameter(
        name + ".dummy_active_topic",
        std::string("/platform/dummy_active"));
      declare_parameter(name + ".dummy_active_timeout_s", 1.0);

      wheel->topic              = get_parameter(name + ".topic").as_string();
      wheel->expected_hz        = get_parameter(name + ".expected_hz").as_double();
      wheel->hz_warn_ratio      = get_parameter(name + ".hz_warn_ratio").as_double();
      wheel->hz_error_ratio     = get_parameter(name + ".hz_error_ratio").as_double();
      wheel->stale_timeout = get_param<double>(name + ".stale_timeout_s", wheel->stale_timeout);
      wheel->max_speed_warn_ms  = get_parameter(name + ".max_speed_warn_ms").as_double();
      wheel->max_speed_error_ms = get_parameter(name + ".max_speed_error_ms").as_double();
      wheel->dummy_active_topic =
        get_parameter(name + ".dummy_active_topic").as_string();
      wheel->dummy_active_timeout =
        get_parameter(name + ".dummy_active_timeout_s").as_double();

      if (wheel->dummy_active_topic.empty()) {
        throw std::runtime_error(name + ".dummy_active_topic must not be empty");
      }
      if (!std::isfinite(wheel->dummy_active_timeout) ||
        wheel->dummy_active_timeout <= 0.0)
      {
        throw std::runtime_error(
                name + ".dummy_active_timeout_s must be finite and > 0");
      }

      wheel_list_.push_back(wheel);
    }
  }

  void setup_tasks_() override
  {
    for (auto & wheel : wheel_list_) {
      wheel->sub = create_subscription<avg_msgs::msg::AvgOdometry>(
        wheel->topic, rclcpp::SensorDataQoS(),
        [this, wheel](const avg_msgs::msg::AvgOdometry::ConstSharedPtr msg) {
          onOdometry(msg, wheel);
        });

      // HH_260729 - Platform dummy feedback is intentionally degraded. Its
      // heartbeat must remain fresh; stale/false restores normal wheel errors.
      wheel->dummy_active_sub = create_subscription<avg_msgs::msg::AvgBool>(
        wheel->dummy_active_topic, rclcpp::QoS(10),
        [this, wheel](const avg_msgs::msg::AvgBool::ConstSharedPtr msg) {
          wheel->dummy_monitor.update(msg->data, this->now());
        });

      add_task("/sensor/wheel/" + wheel->name,
        [this, wheel](StatusWrapper & stat) { checkWheel(stat, *wheel); });

      RCLCPP_INFO(get_logger(),
        "Wheel odometry checker started: %s "
        "(topic=%s, expected_hz=%.0f, dummy=%s timeout=%.2fs)",
        wheel->name.c_str(), wheel->topic.c_str(), wheel->expected_hz,
        wheel->dummy_active_topic.c_str(), wheel->dummy_active_timeout);
    }
  }

private:
  void onOdometry(
    const avg_msgs::msg::AvgOdometry::ConstSharedPtr msg,
    const std::shared_ptr<WheelState> & wheel)
  {
    std::lock_guard<std::mutex> lock(wheel->mtx);
    auto now = this->now();
    wheel->last_msg_time = now;
    wheel->has_msg       = true;

    wheel->vx = msg->twist.twist.linear.x;
    wheel->vy = msg->twist.twist.linear.y;
    wheel->vz = msg->twist.twist.linear.z;
    wheel->wx = msg->twist.twist.angular.x;
    wheel->wy = msg->twist.twist.angular.y;
    wheel->wz = msg->twist.twist.angular.z;
    wheel->vel_nan = has_nan6(
      wheel->vx, wheel->vy, wheel->vz,
      wheel->wx, wheel->wy, wheel->wz);

    wheel->timestamps.push_back(now);
    while (!wheel->timestamps.empty() &&
           (now - wheel->timestamps.front()).seconds() > 2.0)
    {
      wheel->timestamps.pop_front();
    }
  }

  void checkWheel(StatusWrapper & stat, WheelState & wheel)
  {
    std::lock_guard<std::mutex> lock(wheel.mtx);
    const auto now = this->now();

    // HH_260729 - Preserve source provenance even when the platform dummy has
    // not yet produced derived wheel odometry. Never report dummy data as OK.
    double dummy_age_s = -1.0;
    if (wheel.dummy_monitor.isActive(
        now, wheel.dummy_active_timeout, dummy_age_s))
    {
      const double data_age_s =
        wheel.has_msg ? (now - wheel.last_msg_time).seconds() : -1.0;
      const bool data_fresh =
        wheel.has_msg && data_age_s >= 0.0 && data_age_s <= wheel.stale_timeout;
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "DUMMY DATA (platform hardware disabled): sensor=" + wheel.name +
        " topic=" + wheel.topic +
        (data_fresh ? " dummy_odometry=fresh" :
        " dummy_odometry=pending_or_stale"));
      stat.add("data_source", "dummy");
      stat.add("dummy_source", "platform");
      stat.add("hardware_enabled", "false");
      stat.add("dummy_active_topic", wheel.dummy_active_topic);
      stat.add("dummy_active_age_s", dummy_age_s);
      stat.add("dummy_data_received", wheel.has_msg ? "true" : "false");
      if (wheel.has_msg) {
        stat.add("dummy_data_age_s", data_age_s);
      }
      return;
    }

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!wheel.has_msg) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No topic messages: " + wheel.topic);
      stat.add("topic", wheel.topic);
      return;
    }

    double elapsed = (now - wheel.last_msg_time).seconds();
    if (elapsed > wheel.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "No messages for %.1fs (timeout=%.1fs)", elapsed, wheel.stale_timeout);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── Rate 계산 (rolling 2s window) ───────────────────────────────────
    double actual_hz = 0.0;
    if (wheel.timestamps.size() >= 2) {
      double window = (wheel.timestamps.back() - wheel.timestamps.front()).seconds();
      if (window > 0.0) {
        actual_hz = static_cast<double>(wheel.timestamps.size() - 1) / window;
      }
    }

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    int8_t lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // NaN/Inf 체크 (최우선)
    if (wheel.vel_nan) {
      lvl     = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg_str = "twist velocity contains NaN/Inf";
    }

    // Rate 체크
    if (lvl < diagnostic_msgs::msg::DiagnosticStatus::ERROR && wheel.expected_hz > 0.0) {
      double ratio = actual_hz / wheel.expected_hz;
      int8_t hz_lvl = check_low(ratio, wheel.hz_warn_ratio, wheel.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl     = hz_lvl;
        msg_str = (hz_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) ?
          "Input rate critically low" : "Input rate low";
      }
    }

    // 최대 속도 체크
    double speed = std::abs(wheel.vx);
    if (lvl < diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      if (wheel.max_speed_error_ms > 0.0 && speed > wheel.max_speed_error_ms) {
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        msg_str = "Velocity abnormal (above ERROR threshold)";
      } else if (wheel.max_speed_warn_ms > 0.0 && speed > wheel.max_speed_warn_ms &&
                 lvl < diagnostic_msgs::msg::DiagnosticStatus::WARN)
      {
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        msg_str = "Velocity abnormal (above WARN threshold)";
      }
    }

    if (lvl == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "OK (%.0f Hz, vx=%.2f m/s)", actual_hz, wheel.vx);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    char tmp[48];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_hz);
    stat.add("actual_hz",        std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.0f", wheel.expected_hz);
    stat.add("expected_hz",      std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", wheel.vx);
    stat.add("vx (m/s)",         std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", wheel.vy);
    stat.add("vy (m/s)",         std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.4f", wheel.wz);
    stat.add("wz (rad/s)",       std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago", std::string(tmp));
  }

  std::vector<std::shared_ptr<WheelState>> wheel_list_;
};

#include "camrod_system/checker_entrypoint.hpp"
CAMROD_SYSTEM_CHECKER_ENTRYPOINT(WheelOdometryCheckerNode)
