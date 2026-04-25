/**
 * GNSS Checker Node
 *
 * sensor_msgs/NavSatFix 토픽을 구독하여 GNSS 센서 상태를 /diagnostics 토픽으로 발행한다.
 * 센서 레벨 관제: 수신기가 살아있는가? fix를 잡고 있는가?
 *
 * 진단 항목 (GNSS별 단일 태스크)
 * --------------------------------
 *   /sensor/gnss/{name}
 *     - Staleness   : 마지막 메시지 수신 후 경과 시간
 *     - Rate        : 2 초 rolling window 기반 실제 Hz
 *     - Fix status  : NO_FIX(-1) → ERROR, FIX(0) / SBAS_FIX(1) / GBAS_FIX(2) → OK
 *     - Cov type    : UNKNOWN(0) / APPROXIMATED(1) → WARN, DIAGONAL_KNOWN(2) / KNOWN(3) → OK
 *
 * 파라미터 구성
 * -------------
 *   gnss_names: ["main"]
 *
 *   main:
 *     topic:          "/sensing/gnss/navsatfix"
 *     expected_hz:    5.0
 *     hz_warn_ratio:  0.8
 *     hz_error_ratio: 0.5
 *     stale_timeout:  2.0
 */

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── GNSS 상태 구조체 ──────────────────────────────────────────────────────

struct GnssState
{
  // 설정
  std::string name;
  std::string topic;
  double expected_hz{5.0};
  double hz_warn_ratio{0.8};
  double hz_error_ratio{0.5};
  double stale_timeout{2.0};

  // 런타임 상태 (mutex 보호)
  std::mutex mtx;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};
  int8_t fix_status{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
  uint8_t covariance_type{sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN};
  std::deque<rclcpp::Time> timestamps;  // Hz 계산용 rolling window (2s)

  // 구독자
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub;
};

// ── GnssCheckerNode ───────────────────────────────────────────────────────

class GnssCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  GnssCheckerNode()
  : robot_diagnostics_base::BaseChecker("gnss_checker", "gnss_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("gnss_names", std::vector<std::string>{});
  }

  void load_parameters_() override
  {
    auto names = get_parameter("gnss_names").as_string_array();

    for (const auto & name : names) {
      auto gnss = std::make_shared<GnssState>();
      gnss->name = name;

      declare_parameter(name + ".topic",
        std::string("/sensing/gnss/navsatfix"));
      declare_parameter(name + ".expected_hz",    5.0);
      declare_parameter(name + ".hz_warn_ratio",  0.8);
      declare_parameter(name + ".hz_error_ratio", 0.5);
      declare_parameter(name + ".stale_timeout_s",  2.0);

      gnss->topic          = get_parameter(name + ".topic").as_string();
      gnss->expected_hz    = get_parameter(name + ".expected_hz").as_double();
      gnss->hz_warn_ratio  = get_parameter(name + ".hz_warn_ratio").as_double();
      gnss->hz_error_ratio = get_parameter(name + ".hz_error_ratio").as_double();
      gnss->stale_timeout = get_param_with_alias<double>(name + ".stale_timeout_s", gnss->stale_timeout, {name + ".stale_timeout"});

      gnss_list_.push_back(gnss);
    }
  }

  void setup_tasks_() override
  {
    for (auto & gnss : gnss_list_) {
      gnss->sub = create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss->topic, rclcpp::SensorDataQoS(),
        [this, gnss](const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
          onNavSatFix(msg, gnss);
        });

      add_task("/sensor/gnss/" + gnss->name,
        [this, gnss](StatusWrapper & stat) { checkGnss(stat, *gnss); });

      RCLCPP_INFO(get_logger(),
        "GNSS 모니터링 시작: %s (topic=%s, expected_hz=%.1f)",
        gnss->name.c_str(), gnss->topic.c_str(), gnss->expected_hz);
    }
  }

private:
  void onNavSatFix(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg,
    const std::shared_ptr<GnssState> & gnss)
  {
    std::lock_guard<std::mutex> lock(gnss->mtx);
    auto now = this->now();
    gnss->last_msg_time   = now;
    gnss->has_msg         = true;
    gnss->fix_status      = msg->status.status;
    gnss->covariance_type = msg->position_covariance_type;

    gnss->timestamps.push_back(now);
    while (!gnss->timestamps.empty() &&
           (now - gnss->timestamps.front()).seconds() > 2.0)
    {
      gnss->timestamps.pop_front();
    }
  }

  void checkGnss(StatusWrapper & stat, GnssState & gnss)
  {
    std::lock_guard<std::mutex> lock(gnss.mtx);

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!gnss.has_msg) {
      stat.summary(DiagnosticStatus::STALE, "토픽 수신 없음: " + gnss.topic);
      stat.add("topic", gnss.topic);
      return;
    }

    double elapsed = (this->now() - gnss.last_msg_time).seconds();
    if (elapsed > gnss.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "%.1fs 동안 메시지 없음 (timeout=%.1fs)", elapsed, gnss.stale_timeout);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── Rate 계산 (rolling 2s window) ───────────────────────────────────
    double actual_hz = 0.0;
    if (gnss.timestamps.size() >= 2) {
      double window = (gnss.timestamps.back() - gnss.timestamps.front()).seconds();
      if (window > 0.0) {
        actual_hz = static_cast<double>(gnss.timestamps.size() - 1) / window;
      }
    }

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // Fix status 체크 (최우선)
    if (gnss.fix_status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
      lvl     = DiagnosticStatus::ERROR;
      msg_str = "GNSS fix 없음 (NO_FIX)";
    }

    // Rate 체크 (fix 있어도 느리면 경고)
    if (gnss.expected_hz > 0.0) {
      double ratio = actual_hz / gnss.expected_hz;
      int8_t hz_lvl = check_low(ratio, gnss.hz_warn_ratio, gnss.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl     = hz_lvl;
        msg_str = (hz_lvl == S::ERROR) ? "수신 속도 심각 저하" : "수신 속도 저하";
      }
    }

    // Covariance type 체크 (UNKNOWN / APPROXIMATED → WARN)
    if (lvl < DiagnosticStatus::WARN) {
      if (gnss.covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN ||
          gnss.covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED)
      {
        lvl     = DiagnosticStatus::WARN;
        msg_str = "공분산 타입 불확실 (UNKNOWN/APPROXIMATED)";
      }
    }

    if (lvl == DiagnosticStatus::OK) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "OK (%.1f Hz, fix=%d)", actual_hz, gnss.fix_status);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_hz);
    stat.add("actual_hz",        std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", gnss.expected_hz);
    stat.add("expected_hz",      std::string(tmp));
    stat.add("fix_status",       static_cast<int>(gnss.fix_status));
    stat.add("covariance_type",  static_cast<int>(gnss.covariance_type));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago", std::string(tmp));
  }

  std::vector<std::shared_ptr<GnssState>> gnss_list_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GnssCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
