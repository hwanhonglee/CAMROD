/**
 * IMU Checker Node
 *
 * sensor_msgs/Imu 토픽을 구독하여 IMU 센서 상태를 /diagnostics 토픽으로 발행한다.
 * 센서 레벨 관제: IMU가 살아있는가? 데이터 유효성은 문제없는가?
 *
 * 진단 항목 (IMU별 단일 태스크)
 * --------------------------------
 *   /sensor/imu/{name}
 *     - Staleness         : 마지막 메시지 수신 후 경과 시간
 *     - Rate              : 2 초 rolling window 기반 실제 Hz
 *     - Gyro validity     : angular_velocity 성분 NaN/Inf 여부
 *     - Accel validity    : linear_acceleration 성분 NaN/Inf 여부
 *     - Accel magnitude   : ||a|| 이 정적 중력(~9.8 m/s²) 대비 크게 벗어나면 WARN
 *                           (이동 중에는 무시 가능하도록 warn/error 임계값으로 별도 설정)
 *
 * 파라미터 구성
 * -------------
 *   imu_names: ["main"]
 *
 *   main:
 *     topic:                  "/sensing/imu/data"
 *     expected_hz:            100.0
 *     hz_warn_ratio:          0.7
 *     hz_error_ratio:         0.4
 *     stale_timeout:          0.5
 *     accel_magnitude_warn:   30.0   # ||a|| > 이 값 (m/s²) → WARN  (0.0 = 비활성화)
 *     accel_magnitude_error:  50.0   # ||a|| > 이 값 (m/s²) → ERROR (0.0 = 비활성화)
 */

#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// HH_260720 - Diagnose the canonical generated CAMROD IMU stream.
#include <avg_msgs/msg/avg_bool.hpp>
#include <avg_msgs/msg/avg_imu.hpp>

#include <camrod_system/dummy_source_monitor.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── IMU 상태 구조체 ───────────────────────────────────────────────────────

struct ImuState
{
  // 설정
  std::string name;
  std::string topic;
  double expected_hz{100.0};
  double hz_warn_ratio{0.7};
  double hz_error_ratio{0.4};
  double stale_timeout{0.5};
  double accel_magnitude_warn{30.0};
  double accel_magnitude_error{50.0};
  std::string dummy_active_topic;
  double dummy_active_timeout{1.0};

  // 런타임 상태 (mutex 보호)
  std::mutex mtx;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};

  // 최근 측정값
  double gyro_x{0.0}, gyro_y{0.0}, gyro_z{0.0};
  double accel_x{0.0}, accel_y{0.0}, accel_z{0.0};
  bool gyro_nan{false};
  bool accel_nan{false};
  double accel_magnitude{0.0};

  // Hz 계산용 rolling window (2s)
  std::deque<rclcpp::Time> timestamps;

  // 구독자
  rclcpp::Subscription<avg_msgs::msg::AvgImu>::SharedPtr sub;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr dummy_active_sub;
  camrod_system::DummySourceMonitor dummy_monitor;
};

static bool has_nan(double x, double y, double z)
{
  return !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z);
}

// ── ImuCheckerNode ────────────────────────────────────────────────────────

class ImuCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  explicit ImuCheckerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : robot_diagnostics_base::BaseChecker("imu_checker", "imu_checker", options)
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("imu_names", std::vector<std::string>{});
  }

  void load_parameters_() override
  {
    auto names = get_parameter("imu_names").as_string_array();

    for (const auto & name : names) {
      auto imu = std::make_shared<ImuState>();
      imu->name = name;

      declare_parameter(name + ".topic",                  std::string("/sensing/imu/data"));
      declare_parameter(name + ".expected_hz",            100.0);
      declare_parameter(name + ".hz_warn_ratio",          0.7);
      declare_parameter(name + ".hz_error_ratio",         0.4);
      declare_parameter(name + ".stale_timeout_s",          0.5);
      declare_parameter(name + ".accel_magnitude_warn",   30.0);
      declare_parameter(name + ".accel_magnitude_error",  50.0);
      declare_parameter(
        name + ".dummy_active_topic",
        std::string("/sensing/imu/dummy_active"));
      declare_parameter(name + ".dummy_active_timeout_s", 1.0);

      imu->topic                 = get_parameter(name + ".topic").as_string();
      imu->expected_hz           = get_parameter(name + ".expected_hz").as_double();
      imu->hz_warn_ratio         = get_parameter(name + ".hz_warn_ratio").as_double();
      imu->hz_error_ratio        = get_parameter(name + ".hz_error_ratio").as_double();
      imu->stale_timeout = get_param<double>(name + ".stale_timeout_s", imu->stale_timeout);
      imu->accel_magnitude_warn  = get_parameter(name + ".accel_magnitude_warn").as_double();
      imu->accel_magnitude_error = get_parameter(name + ".accel_magnitude_error").as_double();
      imu->dummy_active_topic =
        get_parameter(name + ".dummy_active_topic").as_string();
      imu->dummy_active_timeout =
        get_parameter(name + ".dummy_active_timeout_s").as_double();

      if (imu->dummy_active_topic.empty()) {
        throw std::runtime_error(name + ".dummy_active_topic must not be empty");
      }
      if (!std::isfinite(imu->dummy_active_timeout) ||
        imu->dummy_active_timeout <= 0.0)
      {
        throw std::runtime_error(
                name + ".dummy_active_timeout_s must be finite and > 0");
      }

      imu_list_.push_back(imu);
    }
  }

  void setup_tasks_() override
  {
    for (auto & imu : imu_list_) {
      imu->sub = create_subscription<avg_msgs::msg::AvgImu>(
        imu->topic, rclcpp::SensorDataQoS(),
        [this, imu](const avg_msgs::msg::AvgImu::ConstSharedPtr msg) {
          onImu(msg, imu);
        });

      // HH_260729 - A fresh explicit true heartbeat marks an intentional
      // dummy source; false/stale continues through normal physical checks.
      imu->dummy_active_sub = create_subscription<avg_msgs::msg::AvgBool>(
        imu->dummy_active_topic, rclcpp::QoS(10),
        [this, imu](const avg_msgs::msg::AvgBool::ConstSharedPtr msg) {
          imu->dummy_monitor.update(msg->data, this->now());
        });

      add_task("/sensor/imu/" + imu->name,
        [this, imu](StatusWrapper & stat) { checkImu(stat, *imu); });

      RCLCPP_INFO(get_logger(),
        "IMU checker started: %s (topic=%s, expected_hz=%.0f, dummy=%s timeout=%.2fs)",
        imu->name.c_str(), imu->topic.c_str(), imu->expected_hz,
        imu->dummy_active_topic.c_str(), imu->dummy_active_timeout);
    }
  }

private:
  void onImu(
    const avg_msgs::msg::AvgImu::ConstSharedPtr msg,
    const std::shared_ptr<ImuState> & imu)
  {
    std::lock_guard<std::mutex> lock(imu->mtx);
    auto now = this->now();
    imu->last_msg_time = now;
    imu->has_msg       = true;

    imu->gyro_x = msg->angular_velocity.x;
    imu->gyro_y = msg->angular_velocity.y;
    imu->gyro_z = msg->angular_velocity.z;
    imu->accel_x = msg->linear_acceleration.x;
    imu->accel_y = msg->linear_acceleration.y;
    imu->accel_z = msg->linear_acceleration.z;

    imu->gyro_nan  = has_nan(imu->gyro_x,  imu->gyro_y,  imu->gyro_z);
    imu->accel_nan = has_nan(imu->accel_x, imu->accel_y, imu->accel_z);
    imu->accel_magnitude = std::sqrt(
      imu->accel_x * imu->accel_x +
      imu->accel_y * imu->accel_y +
      imu->accel_z * imu->accel_z);

    imu->timestamps.push_back(now);
    while (!imu->timestamps.empty() &&
           (now - imu->timestamps.front()).seconds() > 2.0)
    {
      imu->timestamps.pop_front();
    }
  }

  void checkImu(StatusWrapper & stat, ImuState & imu)
  {
    std::lock_guard<std::mutex> lock(imu.mtx);
    const auto now = this->now();

    // HH_260729 - Dummy data remains operator-visible WARN and cannot be
    // mistaken for healthy physical IMU data. Heartbeat expiry restores the
    // missing/stale/rate/data-quality diagnosis below.
    double dummy_age_s = -1.0;
    if (imu.dummy_monitor.isActive(
        now, imu.dummy_active_timeout, dummy_age_s))
    {
      const double data_age_s =
        imu.has_msg ? (now - imu.last_msg_time).seconds() : -1.0;
      const bool data_fresh =
        imu.has_msg && data_age_s >= 0.0 && data_age_s <= imu.stale_timeout;
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "DUMMY DATA (hardware disabled): sensor=" + imu.name +
        " topic=" + imu.topic +
        (data_fresh ? " dummy_imu=fresh" : " dummy_imu=pending_or_stale"));
      stat.add("data_source", "dummy");
      stat.add("hardware_enabled", "false");
      stat.add("dummy_active_topic", imu.dummy_active_topic);
      stat.add("dummy_active_age_s", dummy_age_s);
      stat.add("dummy_data_received", imu.has_msg ? "true" : "false");
      if (imu.has_msg) {
        stat.add("dummy_data_age_s", data_age_s);
      }
      return;
    }

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!imu.has_msg) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No topic messages: " + imu.topic);
      stat.add("topic", imu.topic);
      return;
    }

    double elapsed = (now - imu.last_msg_time).seconds();
    if (elapsed > imu.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "No messages for %.1fs (timeout=%.1fs)", elapsed, imu.stale_timeout);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── Rate 계산 (rolling 2s window) ───────────────────────────────────
    double actual_hz = 0.0;
    if (imu.timestamps.size() >= 2) {
      double window = (imu.timestamps.back() - imu.timestamps.front()).seconds();
      if (window > 0.0) {
        actual_hz = static_cast<double>(imu.timestamps.size() - 1) / window;
      }
    }

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    int8_t lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // NaN/Inf 체크 (최우선)
    if (imu.gyro_nan) {
      lvl     = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg_str = "angular_velocity contains NaN/Inf";
    } else if (imu.accel_nan) {
      lvl     = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg_str = "linear_acceleration contains NaN/Inf";
    }

    // Rate 체크
    if (lvl < diagnostic_msgs::msg::DiagnosticStatus::ERROR && imu.expected_hz > 0.0) {
      double ratio = actual_hz / imu.expected_hz;
      int8_t hz_lvl = check_low(ratio, imu.hz_warn_ratio, imu.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl     = hz_lvl;
        msg_str = (hz_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) ?
          "Input rate critically low" : "Input rate low";
      }
    }

    // Accel magnitude 체크
    if (lvl < diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      if (imu.accel_magnitude_error > 0.0 &&
          imu.accel_magnitude > imu.accel_magnitude_error)
      {
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        msg_str = "Acceleration magnitude abnormal (above ERROR threshold)";
      } else if (imu.accel_magnitude_warn > 0.0 &&
                 imu.accel_magnitude > imu.accel_magnitude_warn &&
                 lvl < diagnostic_msgs::msg::DiagnosticStatus::WARN)
      {
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        msg_str = "Acceleration magnitude abnormal (above WARN threshold)";
      }
    }

    if (lvl == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "OK (%.0f Hz)", actual_hz);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    char tmp[48];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_hz);
    stat.add("actual_hz",         std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.0f", imu.expected_hz);
    stat.add("expected_hz",       std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", imu.gyro_x);
    stat.add("gyro_x (rad/s)",    std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", imu.gyro_y);
    stat.add("gyro_y (rad/s)",    std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", imu.gyro_z);
    stat.add("gyro_z (rad/s)",    std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", imu.accel_x);
    stat.add("accel_x (m/s^2)",    std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", imu.accel_y);
    stat.add("accel_y (m/s^2)",    std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", imu.accel_z);
    stat.add("accel_z (m/s^2)",    std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", imu.accel_magnitude);
    stat.add("accel_magnitude (m/s^2)", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago",  std::string(tmp));
  }

  std::vector<std::shared_ptr<ImuState>> imu_list_;
};

#include "camrod_system/checker_entrypoint.hpp"
CAMROD_SYSTEM_CHECKER_ENTRYPOINT(ImuCheckerNode)
