/**
 * Map Cost Grid Checker Node
 *
 * lanelet_cost_grid_node 가 발행하는 /map/cost_grid/lanelet 을 감시하여
 * Planning에 공급되는 레인 제약 격자의 상태를 /diagnostics 로 발행한다.
 *
 * 이 토픽이 가장 중요한 이유:
 *   - lanelet_cost_grid_node 는 map 파일 로드 실패 시 노드가 살아있지만
 *     토픽을 발행하지 않는다 (silent failure).
 *   - /localization/pose 또는 /planning/global_path 가 없으면
 *     격자 재생성 트리거가 없어 발행이 중단된다.
 *   - 이 토픽이 멈추면 Nav2 costmap 의 레인 제약이 사라진다.
 *
 * 진단 항목
 * ---------
 *   /map/cost_grid
 *     - Staleness     : 마지막 메시지 수신 후 경과 시간
 *                       → stale_timeout 초과 시 STALE
 *                       (= 맵 로드 실패 or pose/path 입력 없음)
 *     - Rate          : 2초 rolling window 기반 실제 Hz
 *                       (이벤트 기반 발행이므로 expected_hz 는 낮게 설정)
 *     - Unknown ratio : unknown(-1) 셀 비율
 *                       → 경로 없거나 맵 커버리지 밖이면 높아짐
 *
 * 파라미터 구성
 * -------------
 *   cost_grid_topic:      "/map/cost_grid/lanelet"
 *   expected_hz:          1.0    # 이벤트 기반 (포즈/경로 갱신 시 재생성)
 *   hz_warn_ratio:        0.4    # 실제/기대 비율 < 이 값 → WARN
 *   hz_error_ratio:       0.1    # 실제/기대 비율 < 이 값 → ERROR
 *   stale_timeout:        3.0
 *   unknown_ratio_warn:   0.8    # 80% unknown → WARN (경로 없거나 맵 밖)
 *   unknown_ratio_error:  0.98   # 98% unknown → ERROR (사실상 빈 격자)
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

// ── MapCostGridCheckerNode ─────────────────────────────────────────────────

class MapCostGridCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  MapCostGridCheckerNode()
  : robot_diagnostics_base::BaseChecker("map_cost_grid_checker", "map_cost_grid_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("cost_grid_topic",    std::string("/map/cost_grid/lanelet"));
    declare_parameter("expected_hz",        1.0);
    declare_parameter("hz_warn_ratio",      0.4);
    declare_parameter("hz_error_ratio",     0.1);
    declare_parameter("stale_timeout_s",      3.0);
    declare_parameter("unknown_ratio_warn", 0.8);
    declare_parameter("unknown_ratio_error",0.98);
  }

  void load_parameters_() override
  {
    cost_grid_topic_      = get_parameter("cost_grid_topic").as_string();
    expected_hz_          = get_parameter("expected_hz").as_double();
    hz_warn_ratio_        = get_parameter("hz_warn_ratio").as_double();
    hz_error_ratio_       = get_parameter("hz_error_ratio").as_double();
    stale_timeout_ = get_param<double>("stale_timeout_s", stale_timeout_);
    unknown_ratio_warn_   = get_parameter("unknown_ratio_warn").as_double();
    unknown_ratio_error_  = get_parameter("unknown_ratio_error").as_double();
  }

  void setup_tasks_() override
  {
    // lanelet_cost_grid_node 와 동일한 QoS (transient_local + reliable)
    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      cost_grid_topic_,
      rclcpp::QoS(1).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { onGrid(msg); });

    add_task("/map/cost_grid",
      [this](StatusWrapper & stat) { checkGrid(stat); });

    RCLCPP_INFO(get_logger(),
      "Map Cost Grid 모니터링 시작 (topic=%s, stale=%.1fs)",
      cost_grid_topic_.c_str(), stale_timeout_);
  }

private:
  void onGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    auto now = this->now();

    last_msg_time_ = now;
    has_msg_       = true;
    grid_width_    = msg->info.width;
    grid_height_   = msg->info.height;
    grid_res_      = msg->info.resolution;

    // unknown(-1) 비율 계산
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

    // rolling 2s window for Hz
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
      stat.summary(DiagnosticStatus::STALE,
        "토픽 수신 없음: " + cost_grid_topic_ +
        " (맵 로드 실패 또는 pose/path 입력 없음)");
      stat.add("topic", cost_grid_topic_);
      return;
    }

    double elapsed = (this->now() - last_msg_time_).seconds();
    if (elapsed > stale_timeout_) {
      char buf[120];
      std::snprintf(buf, sizeof(buf),
        "%.1fs 동안 메시지 없음 (timeout=%.1fs) — pose/path 입력 확인 필요",
        elapsed, stale_timeout_);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── Rate 계산 (rolling 2s window) ───────────────────────────────────
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

    // 1. Unknown ratio (경로 없거나 맵 범위 밖 → 격자 의미 없음)
    int8_t unk_lvl = check_high(
      actual_unknown_ratio_, unknown_ratio_warn_, unknown_ratio_error_);
    if (unk_lvl > lvl) {
      lvl = unk_lvl;
      msg_str = (unk_lvl == S::ERROR) ?
        "레인 격자 전체 unknown — 맵 로드 실패 또는 경로 없음" :
        "레인 격자 unknown 높음 — 경로 없거나 맵 커버리지 밖";
    }

    // 2. Rate 체크 (이벤트 기반이므로 임계값 낮게 설정)
    if (expected_hz_ > 0.0) {
      double ratio  = actual_hz / expected_hz_;
      int8_t hz_lvl = check_low(ratio, hz_warn_ratio_, hz_error_ratio_);
      if (hz_lvl > lvl) {
        lvl     = hz_lvl;
        msg_str = (hz_lvl == S::ERROR) ?
          "레인 격자 발행 심각 저하 — pose/path 트리거 확인 필요" :
          "레인 격자 발행 저하";
      }
    }

    if (lvl == DiagnosticStatus::OK) {
      char buf[80];
      std::snprintf(buf, sizeof(buf),
        "OK (%.1f Hz, unknown=%.0f%%)",
        actual_hz, actual_unknown_ratio_ * 100.0);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    char tmp[48];
    std::snprintf(tmp, sizeof(tmp), "%.2f", actual_hz);
    stat.add("actual_hz",           std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", expected_hz_);
    stat.add("expected_hz",         std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_unknown_ratio_ * 100.0);
    stat.add("unknown_ratio_pct",   std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.0f", unknown_ratio_warn_ * 100.0);
    stat.add("unknown_warn_pct",    std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.0f", unknown_ratio_error_ * 100.0);
    stat.add("unknown_error_pct",   std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%u x %u", grid_width_, grid_height_);
    stat.add("grid_size",           std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", grid_res_);
    stat.add("grid_resolution_m",   std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago",    std::string(tmp));
  }

  // 파라미터
  std::string cost_grid_topic_;
  double expected_hz_{1.0};
  double hz_warn_ratio_{0.4};
  double hz_error_ratio_{0.1};
  double stale_timeout_{3.0};
  double unknown_ratio_warn_{0.8};
  double unknown_ratio_error_{0.98};

  // 런타임 상태
  std::mutex mtx_;
  rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};
  bool has_msg_{false};
  double actual_unknown_ratio_{1.0};
  uint32_t grid_width_{0};
  uint32_t grid_height_{0};
  float grid_res_{0.0f};
  std::deque<rclcpp::Time> timestamps_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapCostGridCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
