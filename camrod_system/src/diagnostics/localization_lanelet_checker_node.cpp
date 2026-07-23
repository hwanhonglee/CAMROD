/**
 * Localization Lanelet Checker Node
 *
 * centerline_snapper_node 가 발행하는 /localization/centerline_pose 를 구독하여
 * 로봇이 lanelet 맵 범위 안에 있는지를 /diagnostics 토픽으로 발행한다.
 *
 * centerline_snapper_node 는 로봇 위치가 max_search_radius 밖에 있으면
 * 토픽 발행을 **멈춘다** (silent failure).
 * 이 체커는 그 침묵을 STALE 로 드러내는 것이 핵심이다.
 *
 * 진단 항목
 * ---------
 *   /localization/lanelet
 *     - Staleness      : centerline_pose 미수신 경과 시간
 *                        → stale_timeout 초과 시 STALE
 *                        (= 로봇이 맵 밖으로 벗어났을 가능성)
 *     - Rate           : 2초 rolling window 기반 실제 Hz
 *                        → expected_hz × hz_warn_ratio 미만 → WARN
 *                        → expected_hz × hz_error_ratio 미만 → ERROR
 *     - XY Cov trace   : cov[0] + cov[7] (snapper 가 채워주는 XY 공분산)
 *                        → cov_warn_threshold 초과 → WARN
 *                        → cov_error_threshold 초과 → ERROR
 *
 * 파라미터 구성
 * -------------
 *   lanelet_topic:        "/localization/centerline_pose"
 *   expected_hz:          20.0
 *   hz_warn_ratio:        0.7
 *   hz_error_ratio:       0.4
 *   stale_timeout:        1.0
 *   cov_warn_threshold:   1.0
 *   cov_error_threshold:  9.0
 */

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
// HH_260720 - Diagnose the generated CAMROD centerline pose.
#include <avg_msgs/msg/avg_pose_stamped.hpp>
#include <avg_msgs/msg/avg_pose_with_covariance_stamped.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using StatusWrapper      = diagnostic_updater::DiagnosticStatusWrapper;
// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.

// ── 상태 구조체 ──────────────────────────────────────────────────────────

struct LaneletState
{
  std::mutex mtx;

  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};

  double xy_cov_trace{0.0};
  double pos_x{0.0};
  double pos_y{0.0};

  std::deque<rclcpp::Time> output_timestamps;  // Hz 계산용 rolling window (2s)
  std::deque<rclcpp::Time> input_timestamps;

  rclcpp::Subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>::SharedPtr sub;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr input_sub;
};

// ── LocalizationLaneletCheckerNode ───────────────────────────────────────

class LocalizationLaneletCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  LocalizationLaneletCheckerNode()
  : robot_diagnostics_base::BaseChecker(
      "localization_lanelet_checker", "localization_lanelet_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("lanelet_topic",       std::string("/localization/centerline_pose"));
    declare_parameter("input_pose_topic",    std::string("/localization/pose"));
    declare_parameter("expected_hz",         20.0);
    declare_parameter("hz_warn_ratio",       0.7);
    declare_parameter("hz_error_ratio",      0.4);
    declare_parameter("stale_timeout_s",       1.0);
    declare_parameter("cov_warn_threshold",  1.0);
    declare_parameter("cov_error_threshold", 9.0);
  }

  void load_parameters_() override
  {
    lanelet_topic_         = get_parameter("lanelet_topic").as_string();
    input_pose_topic_      = get_parameter("input_pose_topic").as_string();
    expected_hz_           = get_parameter("expected_hz").as_double();
    hz_warn_ratio_         = get_parameter("hz_warn_ratio").as_double();
    hz_error_ratio_        = get_parameter("hz_error_ratio").as_double();
    stale_timeout_ = get_param<double>("stale_timeout_s", stale_timeout_);
    cov_warn_threshold_    = get_parameter("cov_warn_threshold").as_double();
    cov_error_threshold_   = get_parameter("cov_error_threshold").as_double();
  }

  void setup_tasks_() override
  {
    state_.sub = create_subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
      lanelet_topic_, rclcpp::QoS(10),
      [this](const avg_msgs::msg::AvgPoseWithCovarianceStamped::ConstSharedPtr msg) { onPose(msg); });
    // HH_260723 - Compare snapped output cadence with its real upstream pose
    // cadence so CPU-wide scheduling slowdown is not mislabeled as map loss.
    state_.input_sub = create_subscription<avg_msgs::msg::AvgPoseStamped>(
      input_pose_topic_, rclcpp::QoS(10).reliable(),
      [this](const avg_msgs::msg::AvgPoseStamped::ConstSharedPtr) {onInputPose();});

    add_task("/localization/lanelet",
      [this](StatusWrapper & stat) { checkLanelet(stat); });

    RCLCPP_INFO(get_logger(),
      "Localization lanelet checker started (topic=%s, stale=%.1fs)",
      lanelet_topic_.c_str(), stale_timeout_);
  }

private:
  void onPose(const avg_msgs::msg::AvgPoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    // covariance[14] = z not observed (9999). XY 만 사용.
    const double cov_trace = msg->pose.covariance[0] + msg->pose.covariance[7];

    std::lock_guard<std::mutex> lock(state_.mtx);
    auto now = this->now();

    state_.last_msg_time = now;
    state_.has_msg       = true;
    state_.xy_cov_trace  = cov_trace;
    state_.pos_x         = msg->pose.pose.position.x;
    state_.pos_y         = msg->pose.pose.position.y;

    state_.output_timestamps.push_back(now);
    while (!state_.output_timestamps.empty() &&
           (now - state_.output_timestamps.front()).seconds() > 2.0)
    {
      state_.output_timestamps.pop_front();
    }
  }

  void onInputPose()
  {
    std::lock_guard<std::mutex> lock(state_.mtx);
    const auto now = this->now();
    state_.input_timestamps.push_back(now);
    while (!state_.input_timestamps.empty() &&
      (now - state_.input_timestamps.front()).seconds() > 2.0)
    {
      state_.input_timestamps.pop_front();
    }
  }

  static double calculateRate(const std::deque<rclcpp::Time> & timestamps)
  {
    if (timestamps.size() < 2) {
      return 0.0;
    }
    const double window = (timestamps.back() - timestamps.front()).seconds();
    return window > 0.0 ?
      static_cast<double>(timestamps.size() - 1) / window :
      0.0;
  }

  void checkLanelet(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    // ── Staleness 체크 (핵심: 맵 밖 이탈 = 발행 중단) ─────────────────
    if (!state_.has_msg) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE,
        "No topic messages: " + lanelet_topic_ +
        " (centerline_snapper not started or outside map)");
      stat.add("topic", lanelet_topic_);
      return;
    }

    double elapsed = (this->now() - state_.last_msg_time).seconds();
    if (elapsed > stale_timeout_) {
      char buf[120];
      std::snprintf(buf, sizeof(buf),
        "No messages for %.1fs - suspected outside map (timeout=%.1fs)",
        elapsed, stale_timeout_);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── Rate 계산 (rolling 2s window) ───────────────────────────────────
    const double actual_hz = calculateRate(state_.output_timestamps);
    const double input_hz = calculateRate(state_.input_timestamps);
    // HH_260723 - Judge the snapper against the cadence it actually receives.
    // The localization pose checker remains responsible for an upstream EKF
    // slowdown, while this checker reports only dropped snapping outputs.
    const double evaluated_hz =
      input_hz > 0.0 ? std::min(expected_hz_, input_hz) : expected_hz_;

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    int8_t lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // 1. Covariance trace (snapping 불확실도)
    int8_t cov_lvl = check_high(
      state_.xy_cov_trace,
      cov_warn_threshold_,
      cov_error_threshold_);
    if (cov_lvl > lvl) {
      lvl = cov_lvl;
      if      (cov_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) msg_str = "Lanelet covariance critical (snapping unusable)";
      else if (cov_lvl == diagnostic_msgs::msg::DiagnosticStatus::WARN)  msg_str = "Lanelet covariance high (snapping uncertain)";
    }

    // 2. Rate 체크
    if (expected_hz_ > 0.0) {
      double ratio  = evaluated_hz > 0.0 ? actual_hz / evaluated_hz : 1.0;
      int8_t hz_lvl = check_low(ratio, hz_warn_ratio_, hz_error_ratio_);
      if (hz_lvl > lvl) {
        lvl     = hz_lvl;
        msg_str = (hz_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) ?
          "Lanelet output drops critically below localization input" :
          "Lanelet output drops below localization input";
      }
    }

    if (lvl == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "OK (%.0f Hz, cov_trace=%.4f)", actual_hz, state_.xy_cov_trace);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    char tmp[48];
    std::snprintf(tmp, sizeof(tmp), "%.1f",  actual_hz);
    stat.add("actual_hz",           std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.0f",  expected_hz_);
    stat.add("expected_hz",         std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f",  input_hz);
    stat.add("input_pose_hz",       std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f",  evaluated_hz);
    stat.add("evaluated_hz",        std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.4f",  state_.xy_cov_trace);
    stat.add("xy_cov_trace",        std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.4f",  cov_warn_threshold_);
    stat.add("cov_warn_threshold",  std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.4f",  cov_error_threshold_);
    stat.add("cov_error_threshold", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f",  state_.pos_x);
    stat.add("pos_x (m)",           std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f",  state_.pos_y);
    stat.add("pos_y (m)",           std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f",  elapsed);
    stat.add("last_msg_sec_ago",    std::string(tmp));
  }

  // 파라미터
  std::string lanelet_topic_;
  std::string input_pose_topic_;
  double expected_hz_{20.0};
  double hz_warn_ratio_{0.7};
  double hz_error_ratio_{0.4};
  double stale_timeout_{1.0};
  double cov_warn_threshold_{1.0};
  double cov_error_threshold_{9.0};

  LaneletState state_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationLaneletCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
