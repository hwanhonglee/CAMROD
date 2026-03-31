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

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
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

  // 런타임 상태 (mutex 보호)
  std::mutex mtx;
  rclcpp::Time last_msg_time{0, 0, RCL_ROS_TIME};
  bool has_msg{false};
  uint32_t actual_point_count{0};
  double actual_nan_ratio{0.0};
  std::deque<rclcpp::Time> timestamps;  // Hz 계산용 rolling window (2s)

  // 구독자
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
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
      declare_parameter(name + ".stale_timeout",   2.0);
      declare_parameter(name + ".min_point_count", int64_t(0));
      declare_parameter(name + ".max_point_count", int64_t(0));
      declare_parameter(name + ".max_nan_ratio",   0.0);

      lidar->topic           = get_parameter(name + ".topic").as_string();
      lidar->expected_hz     = get_parameter(name + ".expected_hz").as_double();
      lidar->hz_warn_ratio   = get_parameter(name + ".hz_warn_ratio").as_double();
      lidar->hz_error_ratio  = get_parameter(name + ".hz_error_ratio").as_double();
      lidar->stale_timeout   = get_parameter(name + ".stale_timeout").as_double();
      lidar->min_point_count = get_parameter(name + ".min_point_count").as_int();
      lidar->max_point_count = get_parameter(name + ".max_point_count").as_int();
      lidar->max_nan_ratio   = get_parameter(name + ".max_nan_ratio").as_double();

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

      add_task("/sensor/lidar/" + lidar->name,
        [this, lidar](StatusWrapper & stat) { checkLidar(stat, *lidar); });

      RCLCPP_INFO(get_logger(),
        "LiDAR 모니터링 시작: %s (topic=%s, expected_hz=%.1f)",
        lidar->name.c_str(), lidar->topic.c_str(), lidar->expected_hz);
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

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!lidar.has_msg) {
      stat.summary(DiagnosticStatus::STALE, "토픽 수신 없음: " + lidar.topic);
      stat.add("topic", lidar.topic);
      return;
    }

    double elapsed = (this->now() - lidar.last_msg_time).seconds();
    if (elapsed > lidar.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "%.1fs 동안 메시지 없음 (timeout=%.1fs)", elapsed, lidar.stale_timeout);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
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
    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // Scan rate 체크 (비율 낮을수록 위험 → check_low)
    if (lidar.expected_hz > 0.0) {
      double ratio = actual_hz / lidar.expected_hz;
      lvl = check_low(ratio, lidar.hz_warn_ratio, lidar.hz_error_ratio);
      if      (lvl == S::ERROR) msg_str = "스캔 속도 심각 저하";
      else if (lvl == S::WARN)  msg_str = "스캔 속도 저하";
    }

    // Point count 하한 체크 (낮을수록 위험 → check_low)
    if (lidar.min_point_count > 0) {
      const auto cnt = static_cast<double>(lidar.actual_point_count);
      const auto min_d = static_cast<double>(lidar.min_point_count);
      // 임계 아래면 ERROR, 임계의 절반이면 ERROR 유지
      int8_t cnt_lvl = (cnt < min_d) ? S::ERROR : S::OK;
      if (cnt_lvl > lvl) {
        lvl = cnt_lvl;
        msg_str = "포인트 수 부족";
      }
    }

    // Point count 상한 체크 (높을수록 위험 → check_high)
    if (lidar.max_point_count > 0) {
      const auto cnt = static_cast<double>(lidar.actual_point_count);
      const auto max_d = static_cast<double>(lidar.max_point_count);
      int8_t cnt_lvl = (cnt > max_d) ? S::WARN : S::OK;
      if (cnt_lvl > lvl) {
        lvl = cnt_lvl;
        msg_str = "포인트 수 초과";
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
        if      (nan_lvl == S::ERROR) msg_str = "NaN 비율 심각";
        else if (nan_lvl == S::WARN)  msg_str = "NaN 비율 높음";
      }
    }

    if (lvl == DiagnosticStatus::OK) {
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
