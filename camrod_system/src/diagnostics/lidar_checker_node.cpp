/**
 * LiDAR Checker Node
 *
 * sensor_msgs/PointCloud2 토픽을 구독하여 LiDAR 상태를 /diagnostics 토픽으로 발행한다.
 *
 * 진단 항목 (센서별 단일 태스크)
 * --------------------------------
 *   /sensor/lidar/{name}
 *     - Staleness   : 마지막 메시지 수신 후 경과 시간
 *     - Scan rate   : 2 초 rolling window 기반 실제 Hz
 *     - Point count : 스캔 당 포인트 수 (min / max 임계값)
 *     - NaN ratio   : NaN·Inf 포인트 비율 (max 임계값)
 *
 * 파라미터 구성
 * -------------
 *   lidar_names: ["front"]   ← 모니터링할 LiDAR 이름 목록
 *
 *   front:
 *     topic:           "/sensing/lidar/points"
 *     expected_hz:     10.0
 *     hz_warn_ratio:   0.8   # actual_hz / expected_hz 비율이 이 값 미만이면 WARN
 *     hz_error_ratio:  0.5   # actual_hz / expected_hz 비율이 이 값 미만이면 ERROR
 *     stale_timeout:   2.0   # 이 시간(초) 이상 메시지 없으면 STALE
 *     min_point_count: 100   # 0 = 체크 안 함
 *     max_point_count: 0     # 0 = 체크 안 함
 *     max_nan_ratio:   0.1   # 0.0 = 체크 안 함 (NaN·Inf 포인트 비율 상한)
 */

#include <cmath>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <avg_msgs/msg/avg_bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <camrod_system/dummy_source_monitor.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── LiDAR 상태 구조체 ─────────────────────────────────────────────────────

struct LidarState
{
  // 설정
  std::string name;
  std::string topic;
  double expected_hz{10.0};
  double hz_warn_ratio{0.8};
  double hz_error_ratio{0.5};
  double stale_timeout{2.0};
  int64_t min_point_count{0};
  int64_t max_point_count{0};
  double max_nan_ratio{0.0};
  std::string dummy_active_topic;
  double dummy_active_timeout{1.0};

  // 런타임 상태 (mutex 보호)
  std::mutex mtx;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};
  uint32_t actual_point_count{0};
  double actual_nan_ratio{0.0};
  std::deque<rclcpp::Time> timestamps;  // Hz 계산용 rolling window (2s)

  // 구독자
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr dummy_active_sub;
  camrod_system::DummySourceMonitor dummy_monitor;
};

// ── LidarCheckerNode ──────────────────────────────────────────────────────

class LidarCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  LidarCheckerNode()
  : robot_diagnostics_base::BaseChecker("lidar_checker", "lidar_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("lidar_names", std::vector<std::string>{});
  }

  void load_parameters_() override
  {
    auto names = get_parameter("lidar_names").as_string_array();

    for (const auto & name : names) {
      auto lidar = std::make_shared<LidarState>();
      lidar->name = name;

      // LiDAR별 파라미터 선언 (lidar_names 로드 후 동적으로 선언)
      declare_parameter(name + ".topic",
        std::string("/sensing/lidar/" + name + "/points"));
      declare_parameter(name + ".expected_hz",     10.0);
      declare_parameter(name + ".hz_warn_ratio",   0.8);
      declare_parameter(name + ".hz_error_ratio",  0.5);
      declare_parameter(name + ".stale_timeout_s",   2.0);
      declare_parameter(name + ".min_point_count", int64_t(0));
      declare_parameter(name + ".max_point_count", int64_t(0));
      declare_parameter(name + ".max_nan_ratio",   0.0);
      declare_parameter(
        name + ".dummy_active_topic",
        std::string("/sensing/lidar/dummy_active"));
      declare_parameter(name + ".dummy_active_timeout_s", 1.0);

      lidar->topic           = get_parameter(name + ".topic").as_string();
      lidar->expected_hz     = get_parameter(name + ".expected_hz").as_double();
      lidar->hz_warn_ratio   = get_parameter(name + ".hz_warn_ratio").as_double();
      lidar->hz_error_ratio  = get_parameter(name + ".hz_error_ratio").as_double();
      lidar->stale_timeout = get_param<double>(name + ".stale_timeout_s", lidar->stale_timeout);
      lidar->min_point_count = get_parameter(name + ".min_point_count").as_int();
      lidar->max_point_count = get_parameter(name + ".max_point_count").as_int();
      lidar->max_nan_ratio   = get_parameter(name + ".max_nan_ratio").as_double();
      lidar->dummy_active_topic =
        get_parameter(name + ".dummy_active_topic").as_string();
      lidar->dummy_active_timeout =
        get_parameter(name + ".dummy_active_timeout_s").as_double();

      if (lidar->dummy_active_topic.empty()) {
        throw std::runtime_error(name + ".dummy_active_topic must not be empty");
      }
      if (!std::isfinite(lidar->dummy_active_timeout) ||
        lidar->dummy_active_timeout <= 0.0)
      {
        throw std::runtime_error(
                name + ".dummy_active_timeout_s must be finite and > 0");
      }

      lidars_.push_back(lidar);
    }
  }

  void setup_tasks_() override
  {
    for (auto & lidar : lidars_) {
      lidar->sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar->topic, rclcpp::SensorDataQoS(),
        [this, lidar](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          onCloud(msg, lidar);
        });

      // HH_260729 - A dummy publisher must continuously assert its source
      // identity. False/stale heartbeat keeps physical LiDAR failures visible.
      lidar->dummy_active_sub = create_subscription<avg_msgs::msg::AvgBool>(
        lidar->dummy_active_topic, rclcpp::QoS(10),
        [this, lidar](const avg_msgs::msg::AvgBool::ConstSharedPtr msg) {
          lidar->dummy_monitor.update(msg->data, this->now());
        });

      add_task("/sensor/lidar/" + lidar->name,
        [this, lidar](StatusWrapper & stat) { checkLidar(stat, *lidar); });

      RCLCPP_INFO(get_logger(),
        "LiDAR checker started: %s (topic=%s, expected_hz=%.1f, dummy=%s timeout=%.2fs)",
        lidar->name.c_str(), lidar->topic.c_str(), lidar->expected_hz,
        lidar->dummy_active_topic.c_str(), lidar->dummy_active_timeout);
    }
  }

private:
  void onCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
    const std::shared_ptr<LidarState> & lidar)
  {
    const uint32_t total = msg->width * msg->height;

    // NaN·Inf 비율 계산 (x 채널만 검사)
    uint32_t nan_count = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      for (; iter_x != iter_x.end(); ++iter_x) {
        if (!std::isfinite(*iter_x)) {
          ++nan_count;
        }
      }
    } catch (const std::runtime_error &) {
      // x 필드가 없으면 NaN 체크 생략
    }

    std::lock_guard<std::mutex> lock(lidar->mtx);
    auto now = this->now();
    lidar->last_msg_time     = now;
    lidar->has_msg           = true;
    lidar->actual_point_count = total;
    lidar->actual_nan_ratio  =
      (total > 0) ? static_cast<double>(nan_count) / static_cast<double>(total) : 0.0;

    // Hz 계산용 rolling window (2s)
    lidar->timestamps.push_back(now);
    while (!lidar->timestamps.empty() &&
           (now - lidar->timestamps.front()).seconds() > 2.0)
    {
      lidar->timestamps.pop_front();
    }
  }

  void checkLidar(StatusWrapper & stat, LidarState & lidar)
  {
    std::lock_guard<std::mutex> lock(lidar.mtx);
    const auto now = this->now();

    // HH_260729 - Empty/low-rate dummy clouds are an intentional degraded
    // source. Report WARN before physical rate/point-count checks, and resume
    // those checks immediately when the explicit heartbeat expires.
    double dummy_age_s = -1.0;
    if (lidar.dummy_monitor.isActive(
        now, lidar.dummy_active_timeout, dummy_age_s))
    {
      const double data_age_s =
        lidar.has_msg ? (now - lidar.last_msg_time).seconds() : -1.0;
      const bool data_fresh =
        lidar.has_msg && data_age_s >= 0.0 && data_age_s <= lidar.stale_timeout;
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "DUMMY DATA (hardware disabled): sensor=" + lidar.name +
        " topic=" + lidar.topic +
        (data_fresh ? " dummy_cloud=fresh" : " dummy_cloud=pending_or_stale"));
      stat.add("data_source", "dummy");
      stat.add("hardware_enabled", "false");
      stat.add("dummy_active_topic", lidar.dummy_active_topic);
      stat.add("dummy_active_age_s", dummy_age_s);
      stat.add("dummy_data_received", lidar.has_msg ? "true" : "false");
      if (lidar.has_msg) {
        stat.add("dummy_data_age_s", data_age_s);
        stat.add("dummy_point_count", lidar.actual_point_count);
      }
      return;
    }

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!lidar.has_msg) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No topic messages: " + lidar.topic);
      stat.add("topic", lidar.topic);
      return;
    }

    double elapsed = (now - lidar.last_msg_time).seconds();
    if (elapsed > lidar.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "No messages for %.1fs (timeout=%.1fs)", elapsed, lidar.stale_timeout);
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── Scan rate 계산 (rolling 2s window) ─────────────────────────────
    double actual_hz = 0.0;
    if (lidar.timestamps.size() >= 2) {
      double window =
        (lidar.timestamps.back() - lidar.timestamps.front()).seconds();
      if (window > 0.0) {
        actual_hz = static_cast<double>(lidar.timestamps.size() - 1) / window;
      }
    }

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    int8_t lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // Scan rate 체크 (비율 낮을수록 위험 → check_low)
    if (lidar.expected_hz > 0.0) {
      double ratio = actual_hz / lidar.expected_hz;
      lvl = check_low(ratio, lidar.hz_warn_ratio, lidar.hz_error_ratio);
      if      (lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) msg_str = "Scan rate critically low";
      else if (lvl == diagnostic_msgs::msg::DiagnosticStatus::WARN)  msg_str = "Scan rate low";
    }

    // Point count 하한 체크 (낮을수록 위험 → check_low)
    if (lidar.min_point_count > 0) {
      const auto cnt = static_cast<double>(lidar.actual_point_count);
      const auto min_d = static_cast<double>(lidar.min_point_count);
      // 임계 아래면 ERROR, 임계의 절반이면 ERROR 유지
      int8_t cnt_lvl = (cnt < min_d) ? diagnostic_msgs::msg::DiagnosticStatus::ERROR : diagnostic_msgs::msg::DiagnosticStatus::OK;
      if (cnt_lvl > lvl) {
        lvl = cnt_lvl;
        msg_str = "Point count too low";
      }
    }

    // Point count 상한 체크 (높을수록 위험 → check_high)
    if (lidar.max_point_count > 0) {
      const auto cnt = static_cast<double>(lidar.actual_point_count);
      const auto max_d = static_cast<double>(lidar.max_point_count);
      int8_t cnt_lvl = (cnt > max_d) ? diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
      if (cnt_lvl > lvl) {
        lvl = cnt_lvl;
        msg_str = "Point count too high";
      }
    }

    // NaN ratio 체크 (높을수록 위험 → check_high)
    if (lidar.max_nan_ratio > 0.0) {
      // warn: max_nan_ratio * 0.5, error: max_nan_ratio
      int8_t nan_lvl = check_high(
        lidar.actual_nan_ratio,
        lidar.max_nan_ratio * 0.5,
        lidar.max_nan_ratio);
      if (nan_lvl > lvl) {
        lvl = nan_lvl;
        if      (nan_lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) msg_str = "NaN ratio critical";
        else if (nan_lvl == diagnostic_msgs::msg::DiagnosticStatus::WARN)  msg_str = "NaN ratio high";
      }
    }

    if (lvl == diagnostic_msgs::msg::DiagnosticStatus::OK) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "OK (%.1f Hz, %u pts)",
        actual_hz, lidar.actual_point_count);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_hz);
    stat.add("actual_hz",        std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", lidar.expected_hz);
    stat.add("expected_hz",      std::string(tmp));
    stat.add("point_count",      lidar.actual_point_count);
    std::snprintf(tmp, sizeof(tmp), "%.3f", lidar.actual_nan_ratio);
    stat.add("nan_ratio",        std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago", std::string(tmp));
  }

  std::vector<std::shared_ptr<LidarState>> lidars_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
