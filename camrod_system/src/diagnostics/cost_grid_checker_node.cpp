/**
 * Cost Grid Checker Node
 *
 * LidarCostGridNode 가 발행하는 nav_msgs/OccupancyGrid 를 감시하여
 * 파이프라인 출력 상태를 /diagnostics 토픽으로 발행한다.
 *
 * 진단 항목
 * ---------
 *   /perception/lidar/cost_grid
 *     - Staleness       : 마지막 메시지 수신 후 경과 시간
 *     - Publish rate    : 2 초 rolling window 기반 실제 Hz
 *     - Unknown ratio   : unknown(-1) 셀 비율
 *                         → 입력 클라우드 없거나 TF 실패 시 100% 에 수렴
 *
 * 파라미터 구성
 * -------------
 *   output_topic:         "/sensing/lidar/near_cost_grid"
 *   expected_hz:          10.0
 *   hz_warn_ratio:        0.7
 *   hz_error_ratio:       0.4
 *   stale_timeout:        2.0
 *   unknown_ratio_warn:   0.9   # unknown 90% 이상 → WARN  (입력 부족 의심)
 *   unknown_ratio_error:  1.0   # unknown 100%     → ERROR (입력 없음 또는 TF 실패)
 */

#include <deque>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── CostGridCheckerNode ────────────────────────────────────────────────────

class CostGridCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  CostGridCheckerNode()
  : robot_diagnostics_base::BaseChecker("cost_grid_checker", "cost_grid_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("output_topic",        std::string("/sensing/lidar/near_cost_grid"));
    declare_parameter("expected_hz",         10.0);
    declare_parameter("hz_warn_ratio",       0.7);
    declare_parameter("hz_error_ratio",      0.4);
    declare_parameter("stale_timeout_s",       2.0);
    declare_parameter("unknown_ratio_warn",  0.9);
    declare_parameter("unknown_ratio_error", 1.0);
  }

  void load_parameters_() override
  {
    output_topic_         = get_parameter("output_topic").as_string();
    expected_hz_          = get_parameter("expected_hz").as_double();
    hz_warn_ratio_        = get_parameter("hz_warn_ratio").as_double();
    hz_error_ratio_       = get_parameter("hz_error_ratio").as_double();
    stale_timeout_ = get_param<double>("stale_timeout_s", stale_timeout_);
    unknown_ratio_warn_   = get_parameter("unknown_ratio_warn").as_double();
    unknown_ratio_error_  = get_parameter("unknown_ratio_error").as_double();
  }

  void setup_tasks_() override
  {
    // LidarCostGridNode 발행 QoS 와 동일하게 맞춤 (transient_local + reliable)
    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      output_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { onGrid(msg); });

    add_task("/perception/lidar/cost_grid",
      [this](StatusWrapper & stat) { checkGrid(stat); });

    RCLCPP_INFO(get_logger(), "cost_grid 모니터링 시작: topic=%s", output_topic_.c_str());
  }

private:
  void onGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    auto now = this->now();
    last_msg_time_ = now;
    has_msg_       = true;

    // unknown(-1) 비율 계산: 입력 없거나 TF 실패 시 모든 셀이 unknown 상태가 됨
    const auto total = msg->data.size();
    if (total > 0) {
      size_t unknown_count = 0;
      for (const auto & cell : msg->data) {
        if (cell < 0) ++unknown_count;
      }
      actual_unknown_ratio_ =
        static_cast<double>(unknown_count) / static_cast<double>(total);
    } else {
      actual_unknown_ratio_ = 1.0;
    }

    timestamps_.push_back(now);
    while (!timestamps_.empty() &&
           (now - timestamps_.front()).seconds() > 2.0)
    {
      timestamps_.pop_front();
    }
  }

  void checkGrid(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!has_msg_) {
      stat.summary(DiagnosticStatus::STALE, "토픽 수신 없음: " + output_topic_);
      stat.add("topic", output_topic_);
      return;
    }

    double elapsed = (this->now() - last_msg_time_).seconds();
    if (elapsed > stale_timeout_) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "%.1fs 동안 메시지 없음 (timeout=%.1fs)", elapsed, stale_timeout_);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── 발행 속도 계산 (rolling 2s window) ─────────────────────────────
    double actual_hz = 0.0;
    if (timestamps_.size() >= 2) {
      double window = (timestamps_.back() - timestamps_.front()).seconds();
      if (window > 0.0) {
        actual_hz = static_cast<double>(timestamps_.size() - 1) / window;
      }
    }

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // 발행 속도 체크
    if (expected_hz_ > 0.0) {
      double ratio = actual_hz / expected_hz_;
      int8_t hz_lvl = check_low(ratio, hz_warn_ratio_, hz_error_ratio_);
      if (hz_lvl > lvl) {
        lvl = hz_lvl;
        msg_str = (hz_lvl == S::ERROR) ? "발행 속도 심각 저하" : "발행 속도 저하";
      }
    }

    // unknown 비율 체크 (높을수록 위험 → check_high)
    int8_t unk_lvl = check_high(
      actual_unknown_ratio_, unknown_ratio_warn_, unknown_ratio_error_);
    if (unk_lvl > lvl) {
      lvl = unk_lvl;
      msg_str = (unk_lvl == S::ERROR) ?
        "그리드 전체 unknown (입력 없음 또는 TF 실패)" : "그리드 unknown 비율 높음";
    }

    if (lvl == DiagnosticStatus::OK) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "OK (%.1f Hz, unknown=%.0f%%)",
        actual_hz, actual_unknown_ratio_ * 100.0);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_hz);
    stat.add("actual_hz",         std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", expected_hz_);
    stat.add("expected_hz",       std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_unknown_ratio_ * 100.0);
    stat.add("unknown_ratio_pct", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago",  std::string(tmp));
  }

  // 파라미터
  std::string output_topic_;
  double expected_hz_{10.0};
  double hz_warn_ratio_{0.7};
  double hz_error_ratio_{0.4};
  double stale_timeout_{2.0};
  double unknown_ratio_warn_{0.9};
  double unknown_ratio_error_{1.0};

  // 런타임 상태
  std::mutex mtx_;
  rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};
  bool has_msg_{false};
  double actual_unknown_ratio_{1.0};
  std::deque<rclcpp::Time> timestamps_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostGridCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
