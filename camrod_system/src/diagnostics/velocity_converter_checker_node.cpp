/**
 * Velocity Converter Checker Node
 *
 * PlatformVelocityConverterNode 의 입·출력 토픽을 감시하여
 * 파이프라인 건강 상태를 /diagnostics 토픽으로 발행한다.
 *
 * 핵심 감시 항목: "Silent Drop" 감지
 *   require_imu=true 상태에서 IMU 가 오지 않으면 velocity 가 조용히 드롭됨.
 *   → velocity 입력은 정상인데 출력이 없으면 이 상황을 ERROR 로 노출.
 *
 * 진단 항목
 * ---------
 *   /sensing/velocity_converter
 *     - Velocity input  : /platform/status/velocity 수신 여부
 *     - IMU input       : /sensing/imu/data 수신 여부
 *     - Output          : TwistWithCovarianceStamped 출력 여부 및 Hz
 *     - Silent drop     : velocity 정상 + 출력 unavailable → IMU 대기로 드롭 의심
 *
 * 파라미터 구성
 * -------------
 *   velocity_topic:          "/platform/status/velocity"
 *   imu_topic:               "/sensing/imu/data"
 *   output_topic:            "/sensing/platform_velocity_converter/twist_with_covariance"
 *   velocity_stale_timeout_s: 1.0
 *   imu_stale_timeout_s:      1.0
 *   output_stale_timeout_s:   1.0
 *   expected_output_hz:      10.0
 *   hz_warn_ratio:           0.7
 *   hz_error_ratio:          0.4
 */

#include <deque>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
// HH_260720 - Diagnose generated CAMROD velocity-converter contracts.
#include <avg_msgs/msg/avg_twist_stamped.hpp>
#include <avg_msgs/msg/avg_twist_with_covariance_stamped.hpp>
#include <avg_msgs/msg/avg_imu.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── VelocityConverterCheckerNode ──────────────────────────────────────────

class VelocityConverterCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  VelocityConverterCheckerNode()
  : robot_diagnostics_base::BaseChecker(
      "velocity_converter_checker", "velocity_converter_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("velocity_topic",
      std::string("/platform/status/velocity"));
    declare_parameter("imu_topic",
      std::string("/sensing/imu/data"));
    declare_parameter("output_topic",
      std::string("/sensing/platform_velocity_converter/twist_with_covariance"));
    declare_parameter("velocity_stale_timeout_s", 1.0);
    declare_parameter("imu_stale_timeout_s",      1.0);
    declare_parameter("output_stale_timeout_s",   1.0);
    declare_parameter("expected_output_hz",     10.0);
    declare_parameter("hz_warn_ratio",          0.7);
    declare_parameter("hz_error_ratio",         0.4);
  }

  void load_parameters_() override
  {
    velocity_topic_          = get_parameter("velocity_topic").as_string();
    imu_topic_               = get_parameter("imu_topic").as_string();
    output_topic_            = get_parameter("output_topic").as_string();
    velocity_stale_timeout_ = get_param<double>("velocity_stale_timeout_s", velocity_stale_timeout_);
    imu_stale_timeout_ = get_param<double>("imu_stale_timeout_s", imu_stale_timeout_);
    output_stale_timeout_ = get_param<double>("output_stale_timeout_s", output_stale_timeout_);
    expected_output_hz_      = get_parameter("expected_output_hz").as_double();
    hz_warn_ratio_           = get_parameter("hz_warn_ratio").as_double();
    hz_error_ratio_          = get_parameter("hz_error_ratio").as_double();
  }

  void setup_tasks_() override
  {
    vel_sub_ = create_subscription<avg_msgs::msg::AvgTwistStamped>(
      velocity_topic_, rclcpp::SensorDataQoS(),
      [this](const avg_msgs::msg::AvgTwistStamped::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(mtx_);
        vel_last_time_ = this->now();
        vel_has_msg_   = true;
      });

    imu_sub_ = create_subscription<avg_msgs::msg::AvgImu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](const avg_msgs::msg::AvgImu::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(mtx_);
        imu_last_time_ = this->now();
        imu_has_msg_   = true;
      });

    out_sub_ = create_subscription<avg_msgs::msg::AvgTwistWithCovarianceStamped>(
      output_topic_, rclcpp::QoS(10),
      [this](const avg_msgs::msg::AvgTwistWithCovarianceStamped::ConstSharedPtr) {
        std::lock_guard<std::mutex> lock(mtx_);
        auto now = this->now();
        out_last_time_ = now;
        out_has_msg_   = true;
        out_timestamps_.push_back(now);
        while (!out_timestamps_.empty() &&
               (now - out_timestamps_.front()).seconds() > 2.0)
        {
          out_timestamps_.pop_front();
        }
      });

    add_task("/sensing/velocity_converter",
      [this](StatusWrapper & stat) { checkConverter(stat); });

    RCLCPP_INFO(get_logger(),
      "velocity_converter checker started (vel=%s, imu=%s, out=%s)",
      velocity_topic_.c_str(), imu_topic_.c_str(), output_topic_.c_str());
  }

private:
  void checkConverter(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    auto now = this->now();

    // ── 경과 시간 계산 ──────────────────────────────────────────────────
    double vel_elapsed = vel_has_msg_ ? (now - vel_last_time_).seconds() : -1.0;
    double imu_elapsed = imu_has_msg_ ? (now - imu_last_time_).seconds() : -1.0;
    double out_elapsed = out_has_msg_ ? (now - out_last_time_).seconds() : -1.0;

    bool vel_stale = !vel_has_msg_ || (vel_elapsed > velocity_stale_timeout_);
    bool imu_stale = !imu_has_msg_ || (imu_elapsed > imu_stale_timeout_);
    bool out_stale = !out_has_msg_ || (out_elapsed > output_stale_timeout_);

    // ── 출력 Hz 계산 ────────────────────────────────────────────────────
    double actual_hz = 0.0;
    if (out_timestamps_.size() >= 2) {
      double window = (out_timestamps_.back() - out_timestamps_.front()).seconds();
      if (window > 0.0) {
        actual_hz = static_cast<double>(out_timestamps_.size() - 1) / window;
      }
    }

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // velocity 입력 stale → 운동 정보 unavailable → ERROR
    if (vel_stale) {
      lvl = S::ERROR;
      msg_str = vel_has_msg_ ?
        "velocity input timeout" : "velocity input missing";
    }

    // IMU 입력 stale → angular velocity 부정확 → WARN
    if (imu_stale && lvl < S::WARN) {
      lvl = S::WARN;
      msg_str = imu_has_msg_ ?
        "IMU input timeout (angular velocity unreliable)" : "IMU input missing";
    }

    // Silent drop 감지: velocity 정상인데 출력 unavailable → require_imu 로 드롭 의심 → ERROR
    if (!vel_stale && out_stale && lvl < S::ERROR) {
      lvl = S::ERROR;
      msg_str = "Velocity received but output blocked (suspected IMU wait drop)";
    }

    // 출력 Hz 체크
    if (!out_stale && expected_output_hz_ > 0.0) {
      double ratio = actual_hz / expected_output_hz_;
      int8_t hz_lvl = check_low(ratio, hz_warn_ratio_, hz_error_ratio_);
      if (hz_lvl > lvl) {
        lvl = hz_lvl;
        msg_str = (hz_lvl == S::ERROR) ? "Output rate critically low" : "Output rate low";
      }
    }

    if (lvl == DiagnosticStatus::OK) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "OK (%.1f Hz)", actual_hz);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    stat.add("velocity_input", std::string(vel_stale ? "STALE" : "OK"));
    stat.add("imu_input",      std::string(imu_stale ? "STALE" : "OK"));
    stat.add("output",         std::string(out_stale ? "STALE" : "OK"));

    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_hz);
    stat.add("output_hz",   std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", vel_elapsed < 0.0 ? 99.99 : vel_elapsed);
    stat.add("vel_age_sec", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", imu_elapsed < 0.0 ? 99.99 : imu_elapsed);
    stat.add("imu_age_sec", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", out_elapsed < 0.0 ? 99.99 : out_elapsed);
    stat.add("out_age_sec", std::string(tmp));
  }

  // 파라미터
  std::string velocity_topic_;
  std::string imu_topic_;
  std::string output_topic_;
  double velocity_stale_timeout_{1.0};
  double imu_stale_timeout_{1.0};
  double output_stale_timeout_{1.0};
  double expected_output_hz_{10.0};
  double hz_warn_ratio_{0.7};
  double hz_error_ratio_{0.4};

  // 런타임 상태
  std::mutex mtx_;
  rclcpp::Time vel_last_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time imu_last_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time out_last_time_{0, 0, RCL_ROS_TIME};
  bool vel_has_msg_{false};
  bool imu_has_msg_{false};
  bool out_has_msg_{false};
  std::deque<rclcpp::Time> out_timestamps_;

  rclcpp::Subscription<avg_msgs::msg::AvgTwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgImu>::SharedPtr imu_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgTwistWithCovarianceStamped>::SharedPtr out_sub_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityConverterCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
