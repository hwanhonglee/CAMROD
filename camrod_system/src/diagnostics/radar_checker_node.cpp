/**
 * Radar Checker Node
 *
 * avg_msgs/AvgRange 토픽을 구독하여 레이다(SEN0592) 상태를 /diagnostics 토픽으로 발행한다.
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
 *   radar_names: ["FRONT1", "FRONT2"]       ← 실제 구독/검사할 레이다
 *   disabled_radar_names: ["RIGHT1", "REAR"] ← 의도적으로 제외한 레이다
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
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// HH_260729 - Distinguish an intentional hardware-off dummy stream from a
// physical radar failure without reporting the dummy stream as healthy.
#include <avg_msgs/msg/avg_bool.hpp>
// HH_260720 - Diagnose generated CAMROD radar range streams.
#include <avg_msgs/msg/avg_range.hpp>

#include <camrod_system/dummy_source_monitor.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── 레이다 상태 구조체 ─────────────────────────────────────────────────────

struct RadarState
{
  // 설정
  std::string name;
  std::string topic;
  std::string location{"unknown"};
  std::string frame_id{"unknown"};
  std::string port{"unknown"};
  bool enabled{true};
  std::string disabled_reason;
  double measured_body_echo_min_m{-1.0};
  double measured_body_echo_max_m{-1.0};
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
  std::string dummy_active_topic;
  double dummy_active_timeout_s{1.0};

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
  rclcpp::Subscription<avg_msgs::msg::AvgRange>::SharedPtr sub;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr dummy_active_sub;
  camrod_system::DummySourceMonitor dummy_monitor;
};

// ── RadarCheckerNode ──────────────────────────────────────────────────────

class RadarCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  explicit RadarCheckerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : robot_diagnostics_base::BaseChecker("radar_checker", "radar_checker", options)
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("radar_names", std::vector<std::string>{});
    // HH_260729 - An intentionally disabled field channel is a known degraded
    // configuration, not a missing-topic hardware failure.
    declare_parameter("disabled_radar_names", std::vector<std::string>{});
    // HH_260729 - A fresh true heartbeat means the radar driver is
    // intentionally replaced by the test dummy publisher. Stale/false always
    // falls back to the normal fail-visible physical radar diagnosis.
    declare_parameter(
      "dummy_active_topic", std::string("/sensing/radar/dummy_active"));
    declare_parameter("dummy_active_timeout_s", 1.0);
  }

  void load_parameters_() override
  {
    auto names = get_parameter("radar_names").as_string_array();
    auto disabled_names = get_parameter("disabled_radar_names").as_string_array();
    global_dummy_active_topic_ = get_parameter("dummy_active_topic").as_string();
    global_dummy_active_timeout_s_ = get_parameter("dummy_active_timeout_s").as_double();

    if (global_dummy_active_topic_.empty()) {
      throw std::runtime_error("dummy_active_topic must not be empty");
    }
    if (!std::isfinite(global_dummy_active_timeout_s_) ||
      global_dummy_active_timeout_s_ <= 0.0)
    {
      throw std::runtime_error("dummy_active_timeout_s must be finite and > 0");
    }

    const auto declare_and_load_identity =
      [this](RadarState & radar, const std::string & lower_name) {
        declare_parameter(
          radar.name + ".topic",
          std::string("/sensing/radar/" + lower_name + "/range"));
        declare_parameter(radar.name + ".location", std::string("unknown"));
        declare_parameter(
          radar.name + ".frame_id",
          std::string("radar_" + lower_name + "_link"));
        declare_parameter(radar.name + ".port", std::string("unknown"));

        radar.topic = get_parameter(radar.name + ".topic").as_string();
        radar.location = get_parameter(radar.name + ".location").as_string();
        radar.frame_id = get_parameter(radar.name + ".frame_id").as_string();
        radar.port = get_parameter(radar.name + ".port").as_string();
      };

    for (const auto & name : names) {
      auto radar = std::make_shared<RadarState>();
      radar->name = name;
      radar->enabled = true;

      // 토픽 기본값: 이름을 소문자로 변환 → /sensing/radar/<name>/range
      std::string lower_name = name;
      std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);

      declare_and_load_identity(*radar, lower_name);
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
      // HH_260729 - sensor_enabled[i]:=false keeps the other six physical
      // channels live and marks only this channel as intentional dummy data.
      declare_parameter(
        name + ".dummy_active_topic",
        std::string("/sensing/radar/" + lower_name + "/dummy_active"));
      declare_parameter(name + ".dummy_active_timeout_s", 1.0);

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
      radar->dummy_active_topic =
        get_parameter(name + ".dummy_active_topic").as_string();
      radar->dummy_active_timeout_s =
        get_parameter(name + ".dummy_active_timeout_s").as_double();

      if (radar->dummy_active_topic.empty()) {
        throw std::runtime_error(name + ".dummy_active_topic must not be empty");
      }
      if (!std::isfinite(radar->dummy_active_timeout_s) ||
        radar->dummy_active_timeout_s <= 0.0)
      {
        throw std::runtime_error(
                name + ".dummy_active_timeout_s must be finite and > 0");
      }

      radars_.push_back(radar);
    }

    for (const auto & name : disabled_names) {
      auto radar = std::make_shared<RadarState>();
      radar->name = name;
      radar->enabled = false;

      std::string lower_name = name;
      std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);
      declare_and_load_identity(*radar, lower_name);
      declare_parameter(
        name + ".disabled_reason",
        std::string("intentionally disabled for field test"));
      declare_parameter(name + ".measured_body_echo_min_m", -1.0);
      declare_parameter(name + ".measured_body_echo_max_m", -1.0);
      radar->disabled_reason =
        get_parameter(name + ".disabled_reason").as_string();
      radar->measured_body_echo_min_m =
        get_parameter(name + ".measured_body_echo_min_m").as_double();
      radar->measured_body_echo_max_m =
        get_parameter(name + ".measured_body_echo_max_m").as_double();
      disabled_radars_.push_back(radar);
    }
  }

  void setup_tasks_() override
  {
    // HH_260729 - Volatile subscriber remains compatible with either a
    // continuously published volatile heartbeat or a transient-local dummy
    // publisher. Freshness, rather than durability alone, authorizes dummy
    // diagnostic mode.
    global_dummy_active_sub_ = create_subscription<avg_msgs::msg::AvgBool>(
      global_dummy_active_topic_, rclcpp::QoS(10),
      [this](const avg_msgs::msg::AvgBool::ConstSharedPtr msg) {
        global_dummy_monitor_.update(msg->data, this->now());
      });

    RCLCPP_INFO(
      get_logger(),
      "Radar global dummy-state monitor: topic=%s timeout=%.2fs "
      "(fresh true => WARN DUMMY DATA; false/stale => physical diagnosis)",
      global_dummy_active_topic_.c_str(), global_dummy_active_timeout_s_);

    for (auto & radar : radars_) {
      radar->sub = create_subscription<avg_msgs::msg::AvgRange>(
        radar->topic, rclcpp::SensorDataQoS(),
        [this, radar](const avg_msgs::msg::AvgRange::ConstSharedPtr msg) {
          onRange(msg, radar);
        });

      // HH_260729 - Per-channel markers isolate sensor_enabled[i]:=false
      // without hiding missing/stale failures from any other radar.
      radar->dummy_active_sub = create_subscription<avg_msgs::msg::AvgBool>(
        radar->dummy_active_topic, rclcpp::QoS(10),
        [this, radar](const avg_msgs::msg::AvgBool::ConstSharedPtr msg) {
          radar->dummy_monitor.update(msg->data, this->now());
        });

      add_task("/sensor/radar/" + radar->name,
        [this, radar](StatusWrapper & stat) { checkRadar(stat, *radar); });

      RCLCPP_INFO(get_logger(),
        "Radar checker started: %s "
        "(location=%s frame=%s port=%s topic=%s expected_hz=%.1f "
        "channel_dummy=%s timeout=%.2fs)",
        radar->name.c_str(), radar->location.c_str(), radar->frame_id.c_str(),
        radar->port.c_str(), radar->topic.c_str(), radar->expected_hz,
        radar->dummy_active_topic.c_str(), radar->dummy_active_timeout_s);
    }

    for (auto & radar : disabled_radars_) {
      add_task(
        "/sensor/radar/" + radar->name,
        [this, radar](StatusWrapper & stat) { checkDisabledRadar(stat, *radar); });
      RCLCPP_WARN(
        get_logger(),
        "Radar checker marks %s intentionally disabled "
        "(location=%s frame=%s port=%s reason=%s)",
        radar->name.c_str(), radar->location.c_str(), radar->frame_id.c_str(),
        radar->port.c_str(), radar->disabled_reason.c_str());
    }
  }

private:
  static void addRadarIdentity(StatusWrapper & stat, const RadarState & radar)
  {
    stat.add("sensor_name", radar.name);
    stat.add("sensor_location", radar.location);
    stat.add("sensor_frame", radar.frame_id);
    stat.add("port", radar.port);
    stat.add("topic", radar.topic);
    stat.add("enabled", radar.enabled ? "true" : "false");
  }

  static std::string measuredBodyEchoText(const RadarState & radar)
  {
    const bool has_min =
      std::isfinite(radar.measured_body_echo_min_m) &&
      radar.measured_body_echo_min_m >= 0.0;
    const bool has_max =
      std::isfinite(radar.measured_body_echo_max_m) &&
      radar.measured_body_echo_max_m >= 0.0;
    if (!has_min && !has_max) {
      return "not_recorded";
    }

    const double minimum = has_min ?
      radar.measured_body_echo_min_m : radar.measured_body_echo_max_m;
    const double maximum = has_max ?
      radar.measured_body_echo_max_m : radar.measured_body_echo_min_m;
    char text[64];
    if (std::abs(maximum - minimum) < 1.0e-6) {
      std::snprintf(text, sizeof(text), "%.3f m", minimum);
    } else {
      std::snprintf(text, sizeof(text), "%.3f..%.3f m", minimum, maximum);
    }
    return text;
  }

  void checkDisabledRadar(StatusWrapper & stat, const RadarState & radar)
  {
    // HH_260729 - WARN keeps the known coverage loss operator-visible without
    // misclassifying an intentionally absent publisher as STALE/ERROR.
    const std::string echo = measuredBodyEchoText(radar);
    const std::string message =
      "INTENTIONALLY DISABLED: " + radar.name +
      " location=" + radar.location +
      " frame=" + radar.frame_id +
      " port=" + radar.port +
      " reason=" + radar.disabled_reason +
      " measured_body_echo=" + echo;
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
    addRadarIdentity(stat, radar);
    stat.add("disabled_reason", radar.disabled_reason);
    stat.add("measured_body_echo_m", echo);
  }

  void onRange(
    const avg_msgs::msg::AvgRange::ConstSharedPtr msg,
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
    const auto now = this->now();
    // HH_260729 - Preserve physical identity even on first-message and stale
    // returns so the operator never has to translate a topic back to a mount.
    addRadarIdentity(stat, radar);

    // HH_260729 - Test dummy data is intentionally degraded, never hardware
    // OK. While its explicit heartbeat is fresh, suppress physical
    // missing/stale/range errors and expose all channel identity in the WARN
    // summary. When the heartbeat expires, normal fail-visible checks below
    // resume automatically.
    double global_dummy_age_s = -1.0;
    double channel_dummy_age_s = -1.0;
    const bool global_dummy_active = global_dummy_monitor_.isActive(
      now, global_dummy_active_timeout_s_, global_dummy_age_s);
    const bool channel_dummy_active = radar.dummy_monitor.isActive(
      now, radar.dummy_active_timeout_s, channel_dummy_age_s);
    if (global_dummy_active || channel_dummy_active) {
      const double range_age_s = radar.has_msg ?
        (now - radar.last_msg_time).seconds() : -1.0;
      const bool range_fresh =
        radar.has_msg && range_age_s >= 0.0 && range_age_s <= radar.stale_timeout;
      const std::string dummy_scope =
        global_dummy_active && channel_dummy_active ? "global+channel" :
        (global_dummy_active ? "global" : "channel");
      const std::string message =
        "DUMMY DATA (hardware disabled, scope=" + dummy_scope + "): sensor=" + radar.name +
        " location=" + radar.location +
        " frame=" + radar.frame_id +
        " port=" + radar.port +
        " topic=" + radar.topic +
        (range_fresh ? " dummy_range=fresh" : " dummy_range=pending_or_stale");
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, message);
      stat.add("data_source", "dummy");
      stat.add("hardware_enabled", "false");
      stat.add("dummy_scope", dummy_scope);
      stat.add(
        "global_dummy_active", global_dummy_active ? "true" : "false");
      stat.add("global_dummy_active_topic", global_dummy_active_topic_);
      if (global_dummy_active) {
        stat.add("global_dummy_active_age_s", global_dummy_age_s);
      }
      stat.add(
        "channel_dummy_active", channel_dummy_active ? "true" : "false");
      stat.add("channel_dummy_active_topic", radar.dummy_active_topic);
      if (channel_dummy_active) {
        stat.add("channel_dummy_active_age_s", channel_dummy_age_s);
      }
      stat.add("dummy_range_received", radar.has_msg ? "true" : "false");
      if (radar.has_msg) {
        stat.add("dummy_range_age_s", range_age_s);
      }
      return;
    }

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!radar.has_msg) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No topic messages: " + radar.topic);
      return;
    }

    double elapsed = (now - radar.last_msg_time).seconds();
    if (elapsed > radar.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "No messages for %.1fs (timeout=%.1fs)", elapsed, radar.stale_timeout);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string(buf));
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
    int8_t lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // 발행 속도 체크 (낮을수록 위험 → check_low)
    if (radar.expected_hz > 0.0) {
      double ratio = actual_hz / radar.expected_hz;
      int8_t hz_lvl = check_low(ratio, radar.hz_warn_ratio, radar.hz_error_ratio);
      if (hz_lvl > lvl) {
        lvl = hz_lvl;
        msg_str = (hz_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) ? "Publish rate critically low" : "Publish rate low";
      }
    }

    // NaN / Inf / 음수 체크 → 항상 ERROR (쓰레기값)
    if (radar.range_is_invalid) {
      lvl = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      msg_str = "Invalid range value (NaN/Inf/negative)";
    }

    // range 유효 범위 이탈 체크 → WARN (쓰레기값이 아닌 경우만)
    if (radar.range_out_of_bounds && diagnostic_msgs::msg::DiagnosticStatus::WARN > lvl) {
      lvl = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      msg_str = "Range outside measurement limits";
    }

    // 최솟값 근처 고착 체크: 센서 전방 차폐 의심
    if (!radar.range_is_invalid && !radar.range_no_target &&
        (radar.stuck_min_warn_m > 0.0 || radar.stuck_min_error_m > 0.0))
    {
      double r = static_cast<double>(radar.actual_range_m);
      int8_t stuck_lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
      if (radar.stuck_min_error_m > 0.0 && r <= radar.stuck_min_error_m) {
        stuck_lvl = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      } else if (radar.stuck_min_warn_m > 0.0 && r <= radar.stuck_min_warn_m) {
        stuck_lvl = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      if (stuck_lvl > lvl) {
        lvl = stuck_lvl;
        // HH_260702 - This checker is shared by front/side/rear sensors; keep
        // the diagnostic message direction-neutral and rely on the task name
        // (/sensor/radar/<NAME>) for the physical sensor position.
        msg_str = (stuck_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) ? "Sensor near-field blockage suspected (ERROR)" : "Sensor near-field blockage suspected (WARN)";
      }
    }

    // 최댓값 근처 고착 체크: 센서 무감지·고장 의심
    if (!radar.range_is_invalid && !radar.range_no_target &&
        (radar.stuck_max_warn_m > 0.0 || radar.stuck_max_error_m > 0.0))
    {
      double r = static_cast<double>(radar.actual_range_m);
      int8_t stuck_lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
      if (radar.stuck_max_error_m > 0.0 && r >= radar.stuck_max_error_m) {
        stuck_lvl = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      } else if (radar.stuck_max_warn_m > 0.0 && r >= radar.stuck_max_warn_m) {
        stuck_lvl = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
      if (stuck_lvl > lvl) {
        lvl = stuck_lvl;
        msg_str = (stuck_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) ? "Sensor no-target stuck suspected (ERROR)" : "Sensor no-target stuck suspected (WARN)";
      }
    }

    if (lvl == diagnostic_msgs::msg::DiagnosticStatus::OK) {
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
  std::vector<std::shared_ptr<RadarState>> disabled_radars_;
  std::string global_dummy_active_topic_{"/sensing/radar/dummy_active"};
  double global_dummy_active_timeout_s_{1.0};
  camrod_system::DummySourceMonitor global_dummy_monitor_;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr global_dummy_active_sub_;
};

#include "camrod_system/checker_entrypoint.hpp"
CAMROD_SYSTEM_CHECKER_ENTRYPOINT(RadarCheckerNode)
