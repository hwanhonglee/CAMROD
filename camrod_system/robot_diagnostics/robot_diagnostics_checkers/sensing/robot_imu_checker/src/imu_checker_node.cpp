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
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
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
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub;
};

static bool has_nan(double x, double y, double z)
{
  return !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z);
}

// ── ImuCheckerNode ────────────────────────────────────────────────────────

class ImuCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  ImuCheckerNode()
  : robot_diagnostics_base::BaseChecker("imu_checker", "imu_checker")
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
      declare_parameter(name + ".stale_timeout",          0.5);
      declare_parameter(name + ".accel_magnitude_warn",   30.0);
      declare_parameter(name + ".accel_magnitude_error",  50.0);

      imu->topic                 = get_parameter(name + ".topic").as_string();
      imu->expected_hz           = get_parameter(name + ".expected_hz").as_double();
      imu->hz_warn_ratio         = get_parameter(name + ".hz_warn_ratio").as_double();
      imu->hz_error_ratio        = get_parameter(name + ".hz_error_ratio").as_double();
      imu->stale_timeout         = get_parameter(name + ".stale_timeout").as_double();
      imu->accel_magnitude_warn  = get_parameter(name + ".accel_magnitude_warn").as_double();
      imu->accel_magnitude_error = get_parameter(name + ".accel_magnitude_error").as_double();

      imu_list_.push_back(imu);
    }
  }

  void setup_tasks_() override
  {
    for (auto & imu : imu_list_) {
      imu->sub = create_subscription<sensor_msgs::msg::Imu>(
        imu->topic, rclcpp::SensorDataQoS(),
        [this, imu](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          onImu(msg, imu);
        });

      add_task("/sensor/imu/" + imu->name,
        [this, imu](StatusWrapper & stat) { checkImu(stat, *imu); });

      RCLCPP_INFO(get_logger(),
        "IMU 모니터링 시작: %s (topic=%s, expected_hz=%.0f)",
        imu->name.c_str(), imu->topic.c_str(), imu->expected_hz);
    }
  }

private:
  void onImu(
    const sensor_msgs::msg::Imu::ConstSharedPtr msg,
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

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!imu.has_msg) {
      stat.summary(DiagnosticStatus::STALE, "토픽 수신 없음: " + imu.topic);
      stat.add("topic", imu.topic);
      return;
    }

    double elapsed = (this->now() - imu.last_msg_time).seconds();
    if (elapsed > imu.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "%.2fs 동안 메시지 없음 (timeout=%.1fs)", elapsed, imu.stale_timeout);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
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
    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // NaN/Inf 체크 (최우선)
    if (imu.gyro_nan) {
      lvl     = DiagnosticStatus::ERROR;
      msg_str = "angular_velocity에 NaN/Inf 포함";
    } else if (imu.accel_nan) {
      lvl     = DiagnosticStatus::ERROR;
      msg_str = "linear_acceleration에 NaN/Inf 포함";
    }

    // Rate 체크
    if (lvl < DiagnosticStatus::ERROR && imu.expected_hz > 0.0) {
      double ratio = actual_hz / imu.expected_hz;
      int8_t hz_lvl = check_low(ratio, imu.hz_warn_ratio, imu.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl     = hz_lvl;
        msg_str = (hz_lvl == DiagnosticStatus::ERROR) ?
          "수신 속도 심각 저하" : "수신 속도 저하";
      }
    }

    // Accel magnitude 체크
    if (lvl < DiagnosticStatus::ERROR) {
      if (imu.accel_magnitude_error > 0.0 &&
          imu.accel_magnitude > imu.accel_magnitude_error)
      {
        lvl     = DiagnosticStatus::ERROR;
        msg_str = "가속도 크기 이상 (ERROR 임계값 초과)";
      } else if (imu.accel_magnitude_warn > 0.0 &&
                 imu.accel_magnitude > imu.accel_magnitude_warn &&
                 lvl < DiagnosticStatus::WARN)
      {
        lvl     = DiagnosticStatus::WARN;
        msg_str = "가속도 크기 이상 (WARN 임계값 초과)";
      }
    }

    if (lvl == DiagnosticStatus::OK) {
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
    stat.add("accel_x (m/s²)",    std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", imu.accel_y);
    stat.add("accel_y (m/s²)",    std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", imu.accel_z);
    stat.add("accel_z (m/s²)",    std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", imu.accel_magnitude);
    stat.add("accel_magnitude (m/s²)", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago",  std::string(tmp));
  }

  std::vector<std::shared_ptr<ImuState>> imu_list_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
