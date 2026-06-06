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
 *     topic:              "/platform/status/wheel_odometry"
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
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
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
  WheelOdometryCheckerNode()
  : robot_diagnostics_base::BaseChecker("wheel_odometry_checker", "wheel_odometry_checker")
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

      declare_parameter(name + ".topic",               std::string("/platform/status/wheel_odometry"));
      declare_parameter(name + ".expected_hz",         20.0);
      declare_parameter(name + ".hz_warn_ratio",       0.7);
      declare_parameter(name + ".hz_error_ratio",      0.4);
      declare_parameter(name + ".stale_timeout_s",       1.0);
      declare_parameter(name + ".max_speed_warn_ms",   3.0);
      declare_parameter(name + ".max_speed_error_ms",  5.0);

      wheel->topic              = get_parameter(name + ".topic").as_string();
      wheel->expected_hz        = get_parameter(name + ".expected_hz").as_double();
      wheel->hz_warn_ratio      = get_parameter(name + ".hz_warn_ratio").as_double();
      wheel->hz_error_ratio     = get_parameter(name + ".hz_error_ratio").as_double();
      wheel->stale_timeout = get_param<double>(name + ".stale_timeout_s", wheel->stale_timeout);
      wheel->max_speed_warn_ms  = get_parameter(name + ".max_speed_warn_ms").as_double();
      wheel->max_speed_error_ms = get_parameter(name + ".max_speed_error_ms").as_double();

      wheel_list_.push_back(wheel);
    }
  }

  void setup_tasks_() override
  {
    for (auto & wheel : wheel_list_) {
      wheel->sub = create_subscription<nav_msgs::msg::Odometry>(
        wheel->topic, rclcpp::SensorDataQoS(),
        [this, wheel](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          onOdometry(msg, wheel);
        });

      add_task("/sensor/wheel/" + wheel->name,
        [this, wheel](StatusWrapper & stat) { checkWheel(stat, *wheel); });

      RCLCPP_INFO(get_logger(),
        "휠 오도메트리 모니터링 시작: %s (topic=%s, expected_hz=%.0f)",
        wheel->name.c_str(), wheel->topic.c_str(), wheel->expected_hz);
    }
  }

private:
  void onOdometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg,
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

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!wheel.has_msg) {
      stat.summary(DiagnosticStatus::STALE, "토픽 수신 없음: " + wheel.topic);
      stat.add("topic", wheel.topic);
      return;
    }

    double elapsed = (this->now() - wheel.last_msg_time).seconds();
    if (elapsed > wheel.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "%.2fs 동안 메시지 없음 (timeout=%.1fs)", elapsed, wheel.stale_timeout);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
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
    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // NaN/Inf 체크 (최우선)
    if (wheel.vel_nan) {
      lvl     = DiagnosticStatus::ERROR;
      msg_str = "twist velocity에 NaN/Inf 포함";
    }

    // Rate 체크
    if (lvl < DiagnosticStatus::ERROR && wheel.expected_hz > 0.0) {
      double ratio = actual_hz / wheel.expected_hz;
      int8_t hz_lvl = check_low(ratio, wheel.hz_warn_ratio, wheel.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl     = hz_lvl;
        msg_str = (hz_lvl == DiagnosticStatus::ERROR) ?
          "수신 속도 심각 저하" : "수신 속도 저하";
      }
    }

    // 최대 속도 체크
    double speed = std::abs(wheel.vx);
    if (lvl < DiagnosticStatus::ERROR) {
      if (wheel.max_speed_error_ms > 0.0 && speed > wheel.max_speed_error_ms) {
        lvl     = DiagnosticStatus::ERROR;
        msg_str = "속도 이상 (ERROR 임계값 초과)";
      } else if (wheel.max_speed_warn_ms > 0.0 && speed > wheel.max_speed_warn_ms &&
                 lvl < DiagnosticStatus::WARN)
      {
        lvl     = DiagnosticStatus::WARN;
        msg_str = "속도 이상 (WARN 임계값 초과)";
      }
    }

    if (lvl == DiagnosticStatus::OK) {
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

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometryCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
