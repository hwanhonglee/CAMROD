/**
 * Localization GNSS Checker Node
 *
 * geometry_msgs/PoseWithCovarianceStamped 토픽을 구독하여
 * GNSS 데이터가 로컬라이제이션 fusion에 사용 가능한 품질인지 /diagnostics 로 발행한다.
 * 로컬라이제이션 레벨 관제: 센서가 살아있는 것과 별개로, fusion에 쓸 수 있는가?
 *
 * 진단 항목 (GNSS 소스별 단일 태스크)
 * ----------------------------------------
 *   /localization/gnss/{name}
 *     - Staleness      : 마지막 메시지 수신 후 경과 시간
 *     - Rate           : 2 초 rolling window 기반 실제 Hz
 *     - Covariance     : XY 위치 공분산 trace (cov[0]+cov[7]) 상한 체크
 *                        → WARN: cov_warn_threshold, ERROR: cov_error_threshold
 *     - Position jump  : 연속 수신 간 XY 위치 변화량 상한 체크
 *                        → ERROR: max_jump_m 초과
 *
 * 파라미터 구성
 * -------------
 *   gnss_names: ["main"]
 *
 *   main:
 *     topic:                "/sensing/gnss/pose_with_covariance"
 *     expected_hz:          5.0
 *     hz_warn_ratio:        0.8
 *     hz_error_ratio:       0.5
 *     stale_timeout:        2.0
 *     cov_warn_threshold:   4.0   # XY trace 상한 (WARN)  ← σ ≈ 1.4m
 *     cov_error_threshold:  25.0  # XY trace 상한 (ERROR) ← σ ≈ 3.5m
 *     max_jump_m:           5.0   # 연속 수신 간 최대 XY 점프 (ERROR)
 */

#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// HH_260720 - Diagnose generated CAMROD GNSS localization poses.
#include <avg_msgs/msg/avg_bool.hpp>
#include <avg_msgs/msg/avg_pose_with_covariance_stamped.hpp>

#include <camrod_system/dummy_source_monitor.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using StatusWrapper     = diagnostic_updater::DiagnosticStatusWrapper;
// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.

// ── GNSS 로컬라이제이션 품질 상태 구조체 ─────────────────────────────────

struct LocalizationGnssState
{
  // 설정
  std::string name;
  std::string topic;
  double expected_hz{5.0};
  double hz_warn_ratio{0.8};
  double hz_error_ratio{0.5};
  double stale_timeout{2.0};
  double cov_warn_threshold{4.0};
  double cov_error_threshold{25.0};
  double max_jump_m{5.0};
  std::string dummy_active_topic;
  double dummy_active_timeout{1.0};

  // 런타임 상태 (mutex 보호)
  std::mutex mtx;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};
  double xy_cov_trace{0.0};    // cov[0] + cov[7]
  double last_x{0.0};
  double last_y{0.0};
  bool has_prev_pos{false};
  double last_jump_m{0.0};
  std::deque<rclcpp::Time> timestamps;  // Hz 계산용 rolling window (2s)

  // 구독자
  rclcpp::Subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>::SharedPtr sub;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr dummy_active_sub;
  camrod_system::DummySourceMonitor dummy_monitor;
};

// ── LocalizationGnssCheckerNode ───────────────────────────────────────────

class LocalizationGnssCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  LocalizationGnssCheckerNode()
  : robot_diagnostics_base::BaseChecker(
      "localization_gnss_checker", "localization_gnss_checker")
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
      auto gnss = std::make_shared<LocalizationGnssState>();
      gnss->name = name;

      declare_parameter(name + ".topic",
        std::string("/sensing/gnss/pose_with_covariance"));
      declare_parameter(name + ".expected_hz",          5.0);
      declare_parameter(name + ".hz_warn_ratio",        0.8);
      declare_parameter(name + ".hz_error_ratio",       0.5);
      declare_parameter(name + ".stale_timeout_s",        2.0);
      declare_parameter(name + ".cov_warn_threshold",   4.0);
      declare_parameter(name + ".cov_error_threshold",  25.0);
      declare_parameter(name + ".max_jump_m",           5.0);
      declare_parameter(
        name + ".dummy_active_topic",
        std::string("/sensing/gnss/dummy_active"));
      declare_parameter(name + ".dummy_active_timeout_s", 1.0);

      gnss->topic               = get_parameter(name + ".topic").as_string();
      gnss->expected_hz         = get_parameter(name + ".expected_hz").as_double();
      gnss->hz_warn_ratio       = get_parameter(name + ".hz_warn_ratio").as_double();
      gnss->hz_error_ratio      = get_parameter(name + ".hz_error_ratio").as_double();
      gnss->stale_timeout = get_param<double>(name + ".stale_timeout_s", gnss->stale_timeout);
      gnss->cov_warn_threshold  = get_parameter(name + ".cov_warn_threshold").as_double();
      gnss->cov_error_threshold = get_parameter(name + ".cov_error_threshold").as_double();
      gnss->max_jump_m          = get_parameter(name + ".max_jump_m").as_double();
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
      gnss->sub = create_subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
        gnss->topic, rclcpp::SensorDataQoS(),
        [this, gnss](const avg_msgs::msg::AvgPoseWithCovarianceStamped::ConstSharedPtr msg) {
          onPose(msg, gnss);
        });

      // HH_260729 - NO_FIX dummy input is intentionally rejected by the
      // localization adapter, so this pose checker sees no output. A fresh
      // source heartbeat converts that expected absence to explicit WARN.
      gnss->dummy_active_sub = create_subscription<avg_msgs::msg::AvgBool>(
        gnss->dummy_active_topic, rclcpp::QoS(10),
        [this, gnss](const avg_msgs::msg::AvgBool::ConstSharedPtr msg) {
          gnss->dummy_monitor.update(msg->data, this->now());
        });

      add_task("/localization/gnss/" + gnss->name,
        [this, gnss](StatusWrapper & stat) { checkGnss(stat, *gnss); });

      RCLCPP_INFO(get_logger(),
        "Localization GNSS checker started: %s "
        "(topic=%s, cov_error=%.1f, max_jump=%.1fm, dummy=%s timeout=%.2fs)",
        gnss->name.c_str(), gnss->topic.c_str(),
        gnss->cov_error_threshold, gnss->max_jump_m,
        gnss->dummy_active_topic.c_str(), gnss->dummy_active_timeout);
    }
  }

private:
  void onPose(
    const avg_msgs::msg::AvgPoseWithCovarianceStamped::ConstSharedPtr msg,
    const std::shared_ptr<LocalizationGnssState> & gnss)
  {
    const double cur_x = msg->pose.pose.position.x;
    const double cur_y = msg->pose.pose.position.y;

    // XY 위치 공분산 trace: cov[0]=var_x, cov[7]=var_y
    const double cov_trace = msg->pose.covariance[0] + msg->pose.covariance[7];

    std::lock_guard<std::mutex> lock(gnss->mtx);
    auto now = this->now();

    if (gnss->has_prev_pos) {
      double dx = cur_x - gnss->last_x;
      double dy = cur_y - gnss->last_y;
      gnss->last_jump_m = std::sqrt(dx * dx + dy * dy);
    }

    gnss->last_msg_time = now;
    gnss->has_msg       = true;
    gnss->xy_cov_trace  = cov_trace;
    gnss->last_x        = cur_x;
    gnss->last_y        = cur_y;
    gnss->has_prev_pos  = true;

    gnss->timestamps.push_back(now);
    while (!gnss->timestamps.empty() &&
           (now - gnss->timestamps.front()).seconds() > 2.0)
    {
      gnss->timestamps.pop_front();
    }
  }

  void checkGnss(StatusWrapper & stat, LocalizationGnssState & gnss)
  {
    std::lock_guard<std::mutex> lock(gnss.mtx);
    const auto now = this->now();

    // HH_260729 - A fresh GNSS dummy heartbeat explicitly explains the
    // intentionally absent localization pose. It remains WARN and never
    // claims that dummy GNSS is fusion-ready.
    double dummy_age_s = -1.0;
    if (gnss.dummy_monitor.isActive(
        now, gnss.dummy_active_timeout, dummy_age_s))
    {
      const double pose_age_s =
        gnss.has_msg ? (now - gnss.last_msg_time).seconds() : -1.0;
      const bool pose_fresh =
        gnss.has_msg && pose_age_s >= 0.0 && pose_age_s <= gnss.stale_timeout;
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "DUMMY DATA (GNSS hardware disabled; pose intentionally unavailable): sensor=" +
        gnss.name + " topic=" + gnss.topic +
        (pose_fresh ? " prior_pose=fresh" : " pose=absent_or_stale"));
      stat.add("data_source", "dummy");
      stat.add("hardware_enabled", "false");
      stat.add("fusion_available", "false");
      stat.add("dummy_active_topic", gnss.dummy_active_topic);
      stat.add("dummy_active_age_s", dummy_age_s);
      stat.add("pose_received", gnss.has_msg ? "true" : "false");
      if (gnss.has_msg) {
        stat.add("pose_age_s", pose_age_s);
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

    // Position jump 체크 (최우선: 이상 위치 점프)
    if (gnss.has_prev_pos && gnss.max_jump_m > 0.0 &&
        gnss.last_jump_m > gnss.max_jump_m)
    {
      lvl = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "Position jump detected (%.2fm > %.2fm)", gnss.last_jump_m, gnss.max_jump_m);
      msg_str = buf;
    }

    // Covariance trace 체크 (높을수록 위험 → check_high)
    int8_t cov_lvl = check_high(
      gnss.xy_cov_trace,
      gnss.cov_warn_threshold,
      gnss.cov_error_threshold);
    if (cov_lvl > lvl) {
      lvl = cov_lvl;
      if      (cov_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) msg_str = "GNSS covariance critical (fusion unavailable)";
      else if (cov_lvl == diagnostic_msgs::msg::DiagnosticStatus::WARN)  msg_str = "GNSS covariance high (fusion quality degraded)";
    }

    // Rate 체크
    if (gnss.expected_hz > 0.0) {
      double ratio  = actual_hz / gnss.expected_hz;
      int8_t hz_lvl = check_low(ratio, gnss.hz_warn_ratio, gnss.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl     = hz_lvl;
        msg_str = (hz_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) ? "Input rate critically low" : "Input rate low";
      }
    }

    if (lvl == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "OK (%.1f Hz, cov_trace=%.3f)", actual_hz, gnss.xy_cov_trace);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_hz);
    stat.add("actual_hz",          std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", gnss.expected_hz);
    stat.add("expected_hz",        std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.4f", gnss.xy_cov_trace);
    stat.add("xy_cov_trace",       std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.4f", gnss.cov_warn_threshold);
    stat.add("cov_warn_threshold", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.4f", gnss.cov_error_threshold);
    stat.add("cov_error_threshold", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f", gnss.last_jump_m);
    stat.add("last_jump_m",        std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago",   std::string(tmp));
  }

  std::vector<std::shared_ptr<LocalizationGnssState>> gnss_list_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationGnssCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
