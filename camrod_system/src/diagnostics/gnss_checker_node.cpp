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

#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <avg_msgs/msg/avg_bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <camrod_system/dummy_source_monitor.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
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
  std::string dummy_active_topic;
  double dummy_active_timeout{1.0};

  // 런타임 상태 (mutex 보호)
  std::mutex mtx;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};
  int8_t fix_status{sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX};
  uint8_t covariance_type{sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN};
  std::deque<rclcpp::Time> timestamps;  // Hz 계산용 rolling window (2s)

  // 구독자
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr dummy_active_sub;
  camrod_system::DummySourceMonitor dummy_monitor;
};

// ── GnssCheckerNode ───────────────────────────────────────────────────────

class GnssCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  explicit GnssCheckerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : robot_diagnostics_base::BaseChecker("gnss_checker", "gnss_checker", options)
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
      declare_parameter(
        name + ".dummy_active_topic",
        std::string("/sensing/gnss/dummy_active"));
      declare_parameter(name + ".dummy_active_timeout_s", 1.0);

      gnss->topic          = get_parameter(name + ".topic").as_string();
      gnss->expected_hz    = get_parameter(name + ".expected_hz").as_double();
      gnss->hz_warn_ratio  = get_parameter(name + ".hz_warn_ratio").as_double();
      gnss->hz_error_ratio = get_parameter(name + ".hz_error_ratio").as_double();
      gnss->stale_timeout = get_param<double>(name + ".stale_timeout_s", gnss->stale_timeout);
      gnss->dummy_active_topic =
        get_parameter(name + ".dummy_active_topic").as_string();
      gnss->dummy_active_timeout =
        get_parameter(name + ".dummy_active_timeout_s").as_double();

      if (gnss->dummy_active_topic.empty()) {
        throw std::runtime_error(name + ".dummy_active_topic must not be empty");
      }
      if (!std::isfinite(gnss->dummy_active_timeout) ||
        gnss->dummy_active_timeout <= 0.0)
      {
        throw std::runtime_error(
                name + ".dummy_active_timeout_s must be finite and > 0");
      }

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

      // HH_260729 - Only a fresh explicit true heartbeat authorizes dummy
      // diagnostic mode. False or stale heartbeats retain physical failures.
      gnss->dummy_active_sub = create_subscription<avg_msgs::msg::AvgBool>(
        gnss->dummy_active_topic, rclcpp::QoS(10),
        [this, gnss](const avg_msgs::msg::AvgBool::ConstSharedPtr msg) {
          gnss->dummy_monitor.update(msg->data, this->now());
        });

      add_task("/sensor/gnss/" + gnss->name,
        [this, gnss](StatusWrapper & stat) { checkGnss(stat, *gnss); });

      RCLCPP_INFO(get_logger(),
        "GNSS checker started: %s (topic=%s, expected_hz=%.1f, dummy=%s timeout=%.2fs)",
        gnss->name.c_str(), gnss->topic.c_str(), gnss->expected_hz,
        gnss->dummy_active_topic.c_str(), gnss->dummy_active_timeout);
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
    const auto now = this->now();

    // HH_260729 - An intentional dummy is always degraded WARN, never
    // hardware OK. Its explicit fresh heartbeat suppresses only physical
    // missing/rate/fix errors; once stale, the checks below resume.
    double dummy_age_s = -1.0;
    if (gnss.dummy_monitor.isActive(
        now, gnss.dummy_active_timeout, dummy_age_s))
    {
      const double data_age_s =
        gnss.has_msg ? (now - gnss.last_msg_time).seconds() : -1.0;
      const bool data_fresh =
        gnss.has_msg && data_age_s >= 0.0 && data_age_s <= gnss.stale_timeout;
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "DUMMY DATA (hardware disabled): sensor=" + gnss.name +
        " topic=" + gnss.topic +
        (data_fresh ? " dummy_fix=fresh" : " dummy_fix=pending_or_stale"));
      stat.add("data_source", "dummy");
      stat.add("hardware_enabled", "false");
      stat.add("dummy_active_topic", gnss.dummy_active_topic);
      stat.add("dummy_active_age_s", dummy_age_s);
      stat.add("dummy_data_received", gnss.has_msg ? "true" : "false");
      if (gnss.has_msg) {
        stat.add("dummy_data_age_s", data_age_s);
      }
      return;
    }

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!gnss.has_msg) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No topic messages: " + gnss.topic);
      stat.add("topic", gnss.topic);
      return;
    }

    double elapsed = (now - gnss.last_msg_time).seconds();
    if (elapsed > gnss.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "No messages for %.1fs (timeout=%.1fs)", elapsed, gnss.stale_timeout);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string(buf));
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
    int8_t lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // Fix status 체크 (최우선)
    if (gnss.fix_status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
      lvl     = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg_str = "GNSS fix unavailable (NO_FIX)";
    }

    // Rate 체크 (fix 있어도 느리면 경고)
    if (gnss.expected_hz > 0.0) {
      double ratio = actual_hz / gnss.expected_hz;
      int8_t hz_lvl = check_low(ratio, gnss.hz_warn_ratio, gnss.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl     = hz_lvl;
        msg_str = (hz_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) ? "Input rate critically low" : "Input rate low";
      }
    }

    // Covariance type 체크 (UNKNOWN / APPROXIMATED → WARN)
    if (lvl < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      if (gnss.covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN ||
          gnss.covariance_type == sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED)
      {
        lvl     = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        msg_str = "Covariance type uncertain (UNKNOWN/APPROXIMATED)";
      }
    }

    if (lvl == diagnostic_msgs::msg::DiagnosticStatus::OK) {
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

#include "camrod_system/checker_entrypoint.hpp"
CAMROD_SYSTEM_CHECKER_ENTRYPOINT(GnssCheckerNode)
