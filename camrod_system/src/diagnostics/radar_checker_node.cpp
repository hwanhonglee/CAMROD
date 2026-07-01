/**
 * Radar Checker Node
 *
 * sensor_msgs/Range 토픽을 구독하여 레이다(SEN0592) 상태를 /diagnostics 토픽으로 발행한다.
 *
 * 진단 항목 (센서별 단일 태스크)
 * --------------------------------
 *   /sensor/radar/{name}
 *     - Staleness      : 마지막 메시지 수신 후 경과 시간
 *     - Publish rate   : 2 초 rolling window 기반 실제 Hz
 *     - Range validity : range 가 NaN/Inf 이거나 센서 [min_range, max_range] 범위 밖인지
 *     - Stuck at min   : range 가 최솟값 근처에 고착 → 센서 앞 차폐 의심 (0.0 = 비활성화)
 *     - Stuck at max   : range 가 최댓값 근처에 고착 → 센서 무감지·고장 의심 (0.0 = 비활성화)
 *
 * 파라미터 구성
 * -------------
 *   radar_names: ["FRONT1", "FRONT2", "REAR"]   ← 모니터링할 레이다 이름 목록
 *
 *   FRONT1:
 *     topic:              "/sensing/radar/front1/range"
 *     expected_hz:        16.0    # SEN0592 기본 poll 60 ms → ~16.7 Hz
 *     hz_warn_ratio:      0.7     # actual_hz / expected_hz 비율 미만이면 WARN
 *     hz_error_ratio:     0.4     # actual_hz / expected_hz 비율 미만이면 ERROR
 *     stale_timeout:      1.0     # 이 시간(초) 이상 메시지 없으면 STALE
 *     min_range_m:        0.02    # 센서 측정 하한 (Range.min_range 와 일치)
 *     max_range_m:        4.5     # 센서 측정 상한 (Range.max_range 와 일치)
 *     stuck_min_warn_m:   0.0     # 0.0 = 비활성화: range ≤ 이 값이면 WARN  (전방 차폐 의심)
 *     stuck_min_error_m:  0.0     # 0.0 = 비활성화: range ≤ 이 값이면 ERROR (전방 차폐 확실)
 *     stuck_max_warn_m:   0.0     # 0.0 = 비활성화: range ≥ 이 값이면 WARN  (무감지 의심)
 *     stuck_max_error_m:  0.0     # 0.0 = 비활성화: range ≥ 이 값이면 ERROR (무감지 확실)
 *
 * HH_260623 - Documentation example updated for the latest 7-channel radar layout.
 */

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── 레이다 상태 구조체 ─────────────────────────────────────────────────────

struct RadarState
{
  // 설정
  std::string name;
  std::string topic;
  double expected_hz{16.0};
  double hz_warn_ratio{0.7};
  double hz_error_ratio{0.4};
  double stale_timeout{1.0};
  float  min_range_m{0.02f};       // 센서 측정 하한
  float  max_range_m{4.5f};        // 센서 측정 상한
  double stuck_min_warn_m{0.0};    // 0.0 = 비활성화
  double stuck_min_error_m{0.0};
  double stuck_max_warn_m{0.0};    // 0.0 = 비활성화
  double stuck_max_error_m{0.0};

  // 런타임 상태 (mutex 보호)
  std::mutex mtx;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};
  float actual_range_m{0.0f};
  bool range_is_invalid{false};    // NaN / Inf / 음수
  bool range_no_target{false};      // Range heartbeat above max_range
  bool range_out_of_bounds{false}; // 센서 측정 범위 이탈
  std::deque<rclcpp::Time> timestamps;  // Hz 계산용 rolling window (2s)

  // 구독자
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub;
};

// ── RadarCheckerNode ──────────────────────────────────────────────────────

class RadarCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  RadarCheckerNode()
  : robot_diagnostics_base::BaseChecker("radar_checker", "radar_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("radar_names", std::vector<std::string>{});
  }

  void load_parameters_() override
  {
    auto names = get_parameter("radar_names").as_string_array();

    for (const auto & name : names) {
      auto radar = std::make_shared<RadarState>();
      radar->name = name;

      // 토픽 기본값: 이름을 소문자로 변환 → /sensing/radar/<name>/range
      std::string lower_name = name;
      std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);

      declare_parameter(name + ".topic",
        std::string("/sensing/radar/" + lower_name + "/range"));
      declare_parameter(name + ".expected_hz",       16.0);
      declare_parameter(name + ".hz_warn_ratio",     0.7);
      declare_parameter(name + ".hz_error_ratio",    0.4);
      declare_parameter(name + ".stale_timeout_s",     1.0);
      declare_parameter(name + ".min_range_m",       0.02);
      declare_parameter(name + ".max_range_m",       4.5);
      declare_parameter(name + ".stuck_min_warn_m",  0.0);
      declare_parameter(name + ".stuck_min_error_m", 0.0);
      declare_parameter(name + ".stuck_max_warn_m",  0.0);
      declare_parameter(name + ".stuck_max_error_m", 0.0);

      radar->topic             = get_parameter(name + ".topic").as_string();
      radar->expected_hz       = get_parameter(name + ".expected_hz").as_double();
      radar->hz_warn_ratio     = get_parameter(name + ".hz_warn_ratio").as_double();
      radar->hz_error_ratio    = get_parameter(name + ".hz_error_ratio").as_double();
      radar->stale_timeout = get_param<double>(name + ".stale_timeout_s", radar->stale_timeout);
      radar->min_range_m       = static_cast<float>(get_parameter(name + ".min_range_m").as_double());
      radar->max_range_m       = static_cast<float>(get_parameter(name + ".max_range_m").as_double());
      radar->stuck_min_warn_m  = get_parameter(name + ".stuck_min_warn_m").as_double();
      radar->stuck_min_error_m = get_parameter(name + ".stuck_min_error_m").as_double();
      radar->stuck_max_warn_m  = get_parameter(name + ".stuck_max_warn_m").as_double();
      radar->stuck_max_error_m = get_parameter(name + ".stuck_max_error_m").as_double();

      radars_.push_back(radar);
    }
  }

  void setup_tasks_() override
  {
    for (auto & radar : radars_) {
      radar->sub = create_subscription<sensor_msgs::msg::Range>(
        radar->topic, rclcpp::SensorDataQoS(),
        [this, radar](const sensor_msgs::msg::Range::ConstSharedPtr msg) {
          onRange(msg, radar);
        });

      add_task("/sensor/radar/" + radar->name,
        [this, radar](StatusWrapper & stat) { checkRadar(stat, *radar); });

      RCLCPP_INFO(get_logger(),
        "레이다 모니터링 시작: %s (topic=%s, expected_hz=%.1f)",
        radar->name.c_str(), radar->topic.c_str(), radar->expected_hz);
    }
  }

private:
  void onRange(
    const sensor_msgs::msg::Range::ConstSharedPtr msg,
    const std::shared_ptr<RadarState> & radar)
  {
    std::lock_guard<std::mutex> lock(radar->mtx);
    auto now = this->now();

    radar->last_msg_time   = now;
    radar->has_msg         = true;
    radar->actual_range_m  = msg->range;

    // NaN / Inf / 음수 → 쓰레기값
    radar->range_is_invalid =
      (!std::isfinite(msg->range) || msg->range < 0.0f);

    // HH_260701 - The SEN0592 driver publishes range > max_range as a
    // no-target heartbeat. It proves serial freshness but must not become a
    // diagnostic warning or a cost-grid obstacle.
    radar->range_no_target = !radar->range_is_invalid && msg->range > msg->max_range;

    // 센서 측정 범위 이탈 (쓰레기값이 아닌 경우만 의미 있음)
    radar->range_out_of_bounds = !radar->range_is_invalid &&
      !radar->range_no_target && msg->range < msg->min_range;

    // Hz 계산용 rolling window (2s)
    radar->timestamps.push_back(now);
    while (!radar->timestamps.empty() &&
           (now - radar->timestamps.front()).seconds() > 2.0)
    {
      radar->timestamps.pop_front();
    }
  }

  void checkRadar(StatusWrapper & stat, RadarState & radar)
  {
    std::lock_guard<std::mutex> lock(radar.mtx);

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!radar.has_msg) {
      stat.summary(DiagnosticStatus::STALE, "토픽 수신 없음: " + radar.topic);
      stat.add("topic", radar.topic);
      return;
    }

    double elapsed = (this->now() - radar.last_msg_time).seconds();
    if (elapsed > radar.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "%.1fs 동안 메시지 없음 (timeout=%.1fs)", elapsed, radar.stale_timeout);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── 발행 속도 계산 (rolling 2s window) ─────────────────────────────
    double actual_hz = 0.0;
    if (radar.timestamps.size() >= 2) {
      double window =
        (radar.timestamps.back() - radar.timestamps.front()).seconds();
      if (window > 0.0) {
        actual_hz = static_cast<double>(radar.timestamps.size() - 1) / window;
      }
    }

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // 발행 속도 체크 (낮을수록 위험 → check_low)
    if (radar.expected_hz > 0.0) {
      double ratio = actual_hz / radar.expected_hz;
      int8_t hz_lvl = check_low(ratio, radar.hz_warn_ratio, radar.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl = hz_lvl;
        msg_str = (hz_lvl == S::ERROR) ? "발행 속도 심각 저하" : "발행 속도 저하";
      }
    }

    // NaN / Inf / 음수 체크 → 항상 ERROR (쓰레기값)
    if (radar.range_is_invalid) {
      lvl = S::ERROR;
      msg_str = "range 쓰레기값 (NaN/Inf/음수)";
    }

    // range 유효 범위 이탈 체크 → WARN (쓰레기값이 아닌 경우만)
    if (radar.range_out_of_bounds && S::WARN > lvl) {
      lvl = S::WARN;
      msg_str = "range 측정 범위 이탈";
    }

    // 최솟값 근처 고착 체크: 센서 전방 차폐 의심
    if (!radar.range_is_invalid && !radar.range_no_target &&
        (radar.stuck_min_warn_m > 0.0 || radar.stuck_min_error_m > 0.0))
    {
      double r = static_cast<double>(radar.actual_range_m);
      int8_t stuck_lvl = S::OK;
      if (radar.stuck_min_error_m > 0.0 && r <= radar.stuck_min_error_m) {
        stuck_lvl = S::ERROR;
      } else if (radar.stuck_min_warn_m > 0.0 && r <= radar.stuck_min_warn_m) {
        stuck_lvl = S::WARN;
      }
      if (stuck_lvl > lvl) {
        lvl = stuck_lvl;
        msg_str = (stuck_lvl == S::ERROR) ? "센서 전방 차폐 의심 (ERROR)" : "센서 전방 차폐 의심 (WARN)";
      }
    }

    // 최댓값 근처 고착 체크: 센서 무감지·고장 의심
    if (!radar.range_is_invalid && !radar.range_no_target &&
        (radar.stuck_max_warn_m > 0.0 || radar.stuck_max_error_m > 0.0))
    {
      double r = static_cast<double>(radar.actual_range_m);
      int8_t stuck_lvl = S::OK;
      if (radar.stuck_max_error_m > 0.0 && r >= radar.stuck_max_error_m) {
        stuck_lvl = S::ERROR;
      } else if (radar.stuck_max_warn_m > 0.0 && r >= radar.stuck_max_warn_m) {
        stuck_lvl = S::WARN;
      }
      if (stuck_lvl > lvl) {
        lvl = stuck_lvl;
        msg_str = (stuck_lvl == S::ERROR) ? "센서 무감지 고착 의심 (ERROR)" : "센서 무감지 고착 의심 (WARN)";
      }
    }

    if (lvl == DiagnosticStatus::OK) {
      char buf[64];
      if (radar.range_no_target) {
        std::snprintf(buf, sizeof(buf), "OK (%.1f Hz, no target)", actual_hz);
      } else {
        std::snprintf(buf, sizeof(buf), "OK (%.1f Hz, %.3f m)",
          actual_hz, radar.actual_range_m);
      }
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_hz);
    stat.add("actual_hz",        std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", radar.expected_hz);
    stat.add("expected_hz",      std::string(tmp));
    stat.add("range_valid",      std::string(radar.range_is_invalid ? "false(NaN/Inf)" :
                                              radar.range_no_target ? "true(no_target)" :
                                              radar.range_out_of_bounds ? "false(OOB)" : "true"));
    stat.add("no_target",        std::string(radar.range_no_target ? "true" : "false"));
    if (!radar.range_is_invalid) {
      std::snprintf(tmp, sizeof(tmp), "%.3f", static_cast<double>(radar.actual_range_m));
      stat.add("range_m", std::string(tmp));
    }
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago", std::string(tmp));
  }

  std::vector<std::shared_ptr<RadarState>> radars_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
