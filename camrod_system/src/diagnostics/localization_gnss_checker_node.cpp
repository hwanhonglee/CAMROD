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
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus  = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper     = diagnostic_updater::DiagnosticStatusWrapper;
using PoseWithCovStamped =
  geometry_msgs::msg::PoseWithCovarianceStamped;

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
  rclcpp::Subscription<PoseWithCovStamped>::SharedPtr sub;
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
      declare_parameter(name + ".stale_timeout",        2.0);
      declare_parameter(name + ".cov_warn_threshold",   4.0);
      declare_parameter(name + ".cov_error_threshold",  25.0);
      declare_parameter(name + ".max_jump_m",           5.0);

      gnss->topic               = get_parameter(name + ".topic").as_string();
      gnss->expected_hz         = get_parameter(name + ".expected_hz").as_double();
      gnss->hz_warn_ratio       = get_parameter(name + ".hz_warn_ratio").as_double();
      gnss->hz_error_ratio      = get_parameter(name + ".hz_error_ratio").as_double();
      gnss->stale_timeout       = get_parameter(name + ".stale_timeout").as_double();
      gnss->cov_warn_threshold  = get_parameter(name + ".cov_warn_threshold").as_double();
      gnss->cov_error_threshold = get_parameter(name + ".cov_error_threshold").as_double();
      gnss->max_jump_m          = get_parameter(name + ".max_jump_m").as_double();

      gnss_list_.push_back(gnss);
    }
  }

  void setup_tasks_() override
  {
    for (auto & gnss : gnss_list_) {
      gnss->sub = create_subscription<PoseWithCovStamped>(
        gnss->topic, rclcpp::SensorDataQoS(),
        [this, gnss](const PoseWithCovStamped::ConstSharedPtr msg) {
          onPose(msg, gnss);
        });

      add_task("/localization/gnss/" + gnss->name,
        [this, gnss](StatusWrapper & stat) { checkGnss(stat, *gnss); });

      RCLCPP_INFO(get_logger(),
        "Localization GNSS 모니터링 시작: %s (topic=%s, cov_error=%.1f, max_jump=%.1fm)",
        gnss->name.c_str(), gnss->topic.c_str(),
        gnss->cov_error_threshold, gnss->max_jump_m);
    }
  }

private:
  void onPose(
    const PoseWithCovStamped::ConstSharedPtr msg,
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

    // Position jump 체크 (최우선: 이상 위치 점프)
    if (gnss.has_prev_pos && gnss.max_jump_m > 0.0 &&
        gnss.last_jump_m > gnss.max_jump_m)
    {
      lvl = DiagnosticStatus::ERROR;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "위치 점프 감지 (%.2fm > %.2fm)", gnss.last_jump_m, gnss.max_jump_m);
      msg_str = buf;
    }

    // Covariance trace 체크 (높을수록 위험 → check_high)
    int8_t cov_lvl = check_high(
      gnss.xy_cov_trace,
      gnss.cov_warn_threshold,
      gnss.cov_error_threshold);
    if (cov_lvl > lvl) {
      lvl = cov_lvl;
      if      (cov_lvl == S::ERROR) msg_str = "GNSS 공분산 심각 (fusion 불가)";
      else if (cov_lvl == S::WARN)  msg_str = "GNSS 공분산 높음 (fusion 품질 저하)";
    }

    // Rate 체크
    if (gnss.expected_hz > 0.0) {
      double ratio  = actual_hz / gnss.expected_hz;
      int8_t hz_lvl = check_low(ratio, gnss.hz_warn_ratio, gnss.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl     = hz_lvl;
        msg_str = (hz_lvl == S::ERROR) ? "수신 속도 심각 저하" : "수신 속도 저하";
      }
    }

    if (lvl == DiagnosticStatus::OK) {
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
