/**
 * Localization Pose Checker Node
 *
 * geometry_msgs/PoseWithCovarianceStamped 토픽을 구독하여
 * 로컬라이제이션 최종 출력 품질을 /diagnostics 토픽으로 발행한다.
 *
 * GNSS 입력 품질을 체크하는 robot_gnss_quality_checker 와 달리,
 * 이 노드는 ESKF/Fusion 이후의 최종 위치 추정 출력이
 * Planning/Control 에 공급하기에 충분한 품질인지 확인한다.
 *
 * 진단 항목
 * ---------
 *   /localization/pose
 *     - Staleness      : 마지막 메시지 수신 후 경과 시간
 *     - Rate           : 2 초 rolling window 기반 실제 Hz
 *     - Position jump  : 연속 수신 간 XY 위치 변화량 상한 체크
 *                        → ERROR: max_jump_m 초과
 *     - Cov trace      : XY 위치 공분산 trace (cov[0]+cov[7]) 상한 체크
 *                        → WARN: cov_warn_threshold
 *                        → ERROR: cov_error_threshold
 *
 * 파라미터 구성
 * -------------
 *   pose_topic:           "/localization/pose_with_covariance"
 *   expected_hz:          30.0
 *   hz_warn_ratio:        0.7
 *   hz_error_ratio:       0.4
 *   stale_timeout:        1.0
 *   cov_warn_threshold:   1.0    # XY trace 상한 (WARN)  ← σ ≈ 0.7m (출력 불확실)
 *   cov_error_threshold:  9.0    # XY trace 상한 (ERROR) ← σ ≈ 2.0m (출력 신뢰불가)
 *   max_jump_m:           2.0    # 연속 수신 간 최대 XY 점프 (ERROR)
 */

#include <cmath>
#include <deque>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus   = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper      = diagnostic_updater::DiagnosticStatusWrapper;
using PoseWithCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

// ── 로컬라이제이션 포즈 출력 상태 구조체 ─────────────────────────────────

struct PoseState
{
  // 설정
  std::string pose_topic;
  double expected_hz{30.0};
  double hz_warn_ratio{0.7};
  double hz_error_ratio{0.4};
  double stale_timeout{1.0};
  double cov_warn_threshold{1.0};
  double cov_error_threshold{9.0};
  double max_jump_m{2.0};

  // 런타임 상태 (mutex 보호)
  std::mutex mtx;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};

  double xy_cov_trace{0.0};
  double last_x{0.0};
  double last_y{0.0};
  bool has_prev_pos{false};
  double last_jump_m{0.0};

  std::deque<rclcpp::Time> timestamps;  // Hz 계산용 rolling window (2s)

  rclcpp::Subscription<PoseWithCovStamped>::SharedPtr sub;
};

// ── LocalizationPoseCheckerNode ───────────────────────────────────────────

class LocalizationPoseCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  LocalizationPoseCheckerNode()
  : robot_diagnostics_base::BaseChecker(
      "localization_pose_checker", "localization_pose_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("pose_topic",          std::string("/localization/pose_with_covariance"));
    declare_parameter("expected_hz",         30.0);
    declare_parameter("hz_warn_ratio",       0.7);
    declare_parameter("hz_error_ratio",      0.4);
    declare_parameter("stale_timeout_s",       1.0);
    declare_parameter("cov_warn_threshold",  1.0);
    declare_parameter("cov_error_threshold", 9.0);
    declare_parameter("max_jump_m",          2.0);
  }

  void load_parameters_() override
  {
    state_.pose_topic          = get_parameter("pose_topic").as_string();
    state_.expected_hz         = get_parameter("expected_hz").as_double();
    state_.hz_warn_ratio       = get_parameter("hz_warn_ratio").as_double();
    state_.hz_error_ratio      = get_parameter("hz_error_ratio").as_double();
    state_.stale_timeout = get_param<double>("stale_timeout_s", state_.stale_timeout);
    state_.cov_warn_threshold  = get_parameter("cov_warn_threshold").as_double();
    state_.cov_error_threshold = get_parameter("cov_error_threshold").as_double();
    state_.max_jump_m          = get_parameter("max_jump_m").as_double();
  }

  void setup_tasks_() override
  {
    state_.sub = create_subscription<PoseWithCovStamped>(
      state_.pose_topic, rclcpp::QoS(10),
      [this](const PoseWithCovStamped::ConstSharedPtr msg) { onPose(msg); });

    add_task("/localization/pose",
      [this](StatusWrapper & stat) { checkPose(stat); });

    RCLCPP_INFO(get_logger(),
      "Localization pose checker started (topic=%s, max_jump=%.1fm, cov_error=%.1f)",
      state_.pose_topic.c_str(), state_.max_jump_m, state_.cov_error_threshold);
  }

private:
  void onPose(const PoseWithCovStamped::ConstSharedPtr msg)
  {
    const double cur_x     = msg->pose.pose.position.x;
    const double cur_y     = msg->pose.pose.position.y;
    const double cov_trace = msg->pose.covariance[0] + msg->pose.covariance[7];

    std::lock_guard<std::mutex> lock(state_.mtx);
    auto now = this->now();

    if (state_.has_prev_pos) {
      double dx = cur_x - state_.last_x;
      double dy = cur_y - state_.last_y;
      state_.last_jump_m = std::sqrt(dx * dx + dy * dy);
    }

    state_.last_msg_time = now;
    state_.has_msg       = true;
    state_.xy_cov_trace  = cov_trace;
    state_.last_x        = cur_x;
    state_.last_y        = cur_y;
    state_.has_prev_pos  = true;

    state_.timestamps.push_back(now);
    while (!state_.timestamps.empty() &&
           (now - state_.timestamps.front()).seconds() > 2.0)
    {
      state_.timestamps.pop_front();
    }
  }

  void checkPose(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!state_.has_msg) {
      stat.summary(DiagnosticStatus::STALE, "No topic messages: " + state_.pose_topic);
      stat.add("topic", state_.pose_topic);
      return;
    }

    double elapsed = (this->now() - state_.last_msg_time).seconds();
    if (elapsed > state_.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "No messages for %.1fs (timeout=%.1fs)", elapsed, state_.stale_timeout);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── Rate 계산 (rolling 2s window) ───────────────────────────────────
    double actual_hz = 0.0;
    if (state_.timestamps.size() >= 2) {
      double window =
        (state_.timestamps.back() - state_.timestamps.front()).seconds();
      if (window > 0.0) {
        actual_hz =
          static_cast<double>(state_.timestamps.size() - 1) / window;
      }
    }

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // 1. Position jump (최우선 — 갑작스러운 위치 점프)
    if (state_.has_prev_pos &&
        state_.max_jump_m > 0.0 &&
        state_.last_jump_m > state_.max_jump_m)
    {
      lvl = DiagnosticStatus::ERROR;
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "Position jump detected (%.2fm > %.2fm)", state_.last_jump_m, state_.max_jump_m);
      msg_str = buf;
    }

    // 2. Covariance trace (출력 불확실도)
    int8_t cov_lvl = check_high(
      state_.xy_cov_trace,
      state_.cov_warn_threshold,
      state_.cov_error_threshold);
    if (cov_lvl > lvl) {
      lvl = cov_lvl;
      if      (cov_lvl == S::ERROR) msg_str = "Output covariance critical (pose unusable)";
      else if (cov_lvl == S::WARN)  msg_str = "Output covariance high (pose uncertain)";
    }

    // 3. Rate 체크
    if (state_.expected_hz > 0.0) {
      double ratio  = actual_hz / state_.expected_hz;
      int8_t hz_lvl = check_low(ratio, state_.hz_warn_ratio, state_.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl     = hz_lvl;
        msg_str = (hz_lvl == S::ERROR) ? "Output rate critically low" : "Output rate low";
      }
    }

    if (lvl == DiagnosticStatus::OK) {
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
    std::snprintf(tmp, sizeof(tmp), "%.0f",  state_.expected_hz);
    stat.add("expected_hz",         std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.4f",  state_.xy_cov_trace);
    stat.add("xy_cov_trace",        std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.4f",  state_.cov_warn_threshold);
    stat.add("cov_warn_threshold",  std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.4f",  state_.cov_error_threshold);
    stat.add("cov_error_threshold", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f",  state_.last_jump_m);
    stat.add("last_jump_m",         std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f",  state_.max_jump_m);
    stat.add("max_jump_m",          std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f",  state_.last_x);
    stat.add("pos_x (m)",           std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.3f",  state_.last_y);
    stat.add("pos_y (m)",           std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f",  elapsed);
    stat.add("last_msg_sec_ago",    std::string(tmp));
  }

  PoseState state_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationPoseCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
