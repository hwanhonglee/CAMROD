/**
 * Perception Obstacle Checker Node
 *
 * 장애물 감지 파이프라인의 출력 토픽들을 통합 모니터링한다.
 *
 * 지원 토픽 타입
 * --------------
 *   PointCloud2      : sensor_msgs/PointCloud2    (obstacle_fusion_node 출력)
 *   Detection2DArray : vision_msgs/Detection2DArray (camera detection 출력)
 *
 * 진단 항목 (소스별)
 * -----------------
 *   - Staleness    : 마지막 메시지 수신 후 경과 시간
 *   - Output rate  : 2초 rolling window 기반 실제 Hz
 *   - Count        : 포인트 수 또는 검출 수 (min / max 임계값)
 *
 * 파라미터 구성 예시
 * -----------------
 *   obstacle_names: ["camera_detections", "fused_obstacles"]
 *
 *   camera_detections:
 *     topic:          "/perception/camera/detections_2d"
 *     type:           "Detection2DArray"
 *     expected_hz:    10.0
 *     hz_warn_ratio:  0.8
 *     hz_error_ratio: 0.5
 *     stale_timeout:  2.0
 *     min_count:      0    # 0 = 체크 안 함
 *     max_count:      0    # 0 = 체크 안 함
 *
 *   fused_obstacles:
 *     topic:          "/perception/obstacles"
 *     type:           "PointCloud2"
 *     expected_hz:    10.0
 *     hz_warn_ratio:  0.8
 *     hz_error_ratio: 0.5
 *     stale_timeout:  2.0
 *     min_count:      0
 *     max_count:      0
 */

#include <cstdio>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;
using PointCloud2      = sensor_msgs::msg::PointCloud2;
using Detection2DArray = vision_msgs::msg::Detection2DArray;

// ── 소스별 런타임 상태 ────────────────────────────────────────────────────

struct ObstacleSource
{
  // 설정
  std::string name;
  std::string topic;
  std::string type;            // "PointCloud2" or "Detection2DArray"
  double      expected_hz{10.0};
  double      hz_warn_ratio{0.8};
  double      hz_error_ratio{0.5};
  double      stale_timeout{2.0};
  int64_t     min_count{0};
  int64_t     max_count{0};

  // 런타임 (mutex 보호)
  std::mutex               mtx;
  rclcpp::Time             last_msg_time{0, 0, RCL_ROS_TIME};
  bool                     has_msg{false};
  int64_t                  actual_count{0};
  std::deque<rclcpp::Time> timestamps;

  // 구독 (타입에 따라 하나만 사용)
  rclcpp::Subscription<PointCloud2>::SharedPtr      pc2_sub;
  rclcpp::Subscription<Detection2DArray>::SharedPtr det_sub;
};

// ── PerceptionObstacleCheckerNode ─────────────────────────────────────────

class PerceptionObstacleCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  PerceptionObstacleCheckerNode()
  : robot_diagnostics_base::BaseChecker(
      "perception_obstacle_checker", "perception_obstacle_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("obstacle_names", std::vector<std::string>{});
  }

  void load_parameters_() override
  {
    obstacle_names_ = get_parameter("obstacle_names").as_string_array();

    for (const auto & name : obstacle_names_) {
      auto src = std::make_shared<ObstacleSource>();
      src->name = name;

      declare_parameter(name + ".topic",          std::string(""));
      declare_parameter(name + ".type",           std::string("PointCloud2"));
      declare_parameter(name + ".expected_hz",    10.0);
      declare_parameter(name + ".hz_warn_ratio",  0.8);
      declare_parameter(name + ".hz_error_ratio", 0.5);
      declare_parameter(name + ".stale_timeout_s",  2.0);
      declare_parameter(name + ".min_count",      int64_t(0));
      declare_parameter(name + ".max_count",      int64_t(0));

      src->topic          = get_parameter(name + ".topic").as_string();
      src->type           = get_parameter(name + ".type").as_string();
      src->expected_hz    = get_parameter(name + ".expected_hz").as_double();
      src->hz_warn_ratio  = get_parameter(name + ".hz_warn_ratio").as_double();
      src->hz_error_ratio = get_parameter(name + ".hz_error_ratio").as_double();
      src->stale_timeout = get_param<double>(name + ".stale_timeout_s", src->stale_timeout);
      src->min_count      = get_parameter(name + ".min_count").as_int();
      src->max_count      = get_parameter(name + ".max_count").as_int();

      sources_.push_back(src);
    }
  }

  void setup_tasks_() override
  {
    for (auto & src : sources_) {
      if (src->type == "Detection2DArray") {
        src->det_sub = create_subscription<Detection2DArray>(
          src->topic, rclcpp::QoS(10),
          [this, src](const Detection2DArray::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(src->mtx);
            const auto now     = this->now();
            src->last_msg_time = now;
            src->has_msg       = true;
            src->actual_count  = static_cast<int64_t>(msg->detections.size());
            src->timestamps.push_back(now);
            while (!src->timestamps.empty() &&
                   (now - src->timestamps.front()).seconds() > 2.0)
            {
              src->timestamps.pop_front();
            }
          });

        RCLCPP_INFO(get_logger(),
          "[%s] Detection2DArray 모니터링 시작 (topic=%s, expected_hz=%.1f)",
          src->name.c_str(), src->topic.c_str(), src->expected_hz);

      } else {
        // PointCloud2 (기본)
        src->pc2_sub = create_subscription<PointCloud2>(
          src->topic, rclcpp::QoS(10),
          [this, src](const PointCloud2::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(src->mtx);
            const auto now     = this->now();
            src->last_msg_time = now;
            src->has_msg       = true;
            src->actual_count  = static_cast<int64_t>(msg->width * msg->height);
            src->timestamps.push_back(now);
            while (!src->timestamps.empty() &&
                   (now - src->timestamps.front()).seconds() > 2.0)
            {
              src->timestamps.pop_front();
            }
          });

        RCLCPP_INFO(get_logger(),
          "[%s] PointCloud2 모니터링 시작 (topic=%s, expected_hz=%.1f)",
          src->name.c_str(), src->topic.c_str(), src->expected_hz);
      }

      const std::string diag_name = "/perception/obstacles/" + src->name;
      add_task(diag_name,
        [this, src](StatusWrapper & stat) { checkSource(stat, *src); });
    }
  }

private:
  void checkSource(StatusWrapper & stat, ObstacleSource & src)
  {
    std::lock_guard<std::mutex> lock(src.mtx);

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!src.has_msg) {
      stat.summary(DiagnosticStatus::STALE, "토픽 수신 없음: " + src.topic);
      stat.add("topic", src.topic);
      return;
    }

    const double elapsed = (this->now() - src.last_msg_time).seconds();
    if (elapsed > src.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "%.1fs 동안 메시지 없음 (timeout=%.1fs)", elapsed, src.stale_timeout);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── Output rate 계산 (rolling 2s window) ────────────────────────────
    double actual_hz = 0.0;
    if (src.timestamps.size() >= 2) {
      const double window =
        (src.timestamps.back() - src.timestamps.front()).seconds();
      if (window > 0.0) {
        actual_hz = static_cast<double>(src.timestamps.size() - 1) / window;
      }
    }

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str = "OK";

    if (src.expected_hz > 0.0) {
      const double ratio = actual_hz / src.expected_hz;
      lvl = check_low(ratio, src.hz_warn_ratio, src.hz_error_ratio);
      if      (lvl == S::ERROR) msg_str = "출력 속도 심각 저하";
      else if (lvl == S::WARN)  msg_str = "출력 속도 저하";
    }

    // Count 하한 체크
    if (src.min_count > 0 && src.actual_count < src.min_count) {
      if (S::ERROR > lvl) {
        lvl     = S::ERROR;
        msg_str = (src.type == "Detection2DArray") ? "검출 수 부족" : "포인트 수 부족";
      }
    }

    // Count 상한 체크
    if (src.max_count > 0 && src.actual_count > src.max_count) {
      if (S::WARN > lvl) {
        lvl     = S::WARN;
        msg_str = (src.type == "Detection2DArray") ? "검출 수 초과" : "포인트 수 초과";
      }
    }

    if (lvl == DiagnosticStatus::OK) {
      char buf[64];
      const char * unit = (src.type == "Detection2DArray") ? "det" : "pts";
      std::snprintf(buf, sizeof(buf), "OK (%.1f Hz, %ld %s)",
        actual_hz, src.actual_count, unit);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_hz);
    stat.add("actual_hz",        std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", src.expected_hz);
    stat.add("expected_hz",      std::string(tmp));
    stat.add("count",            src.actual_count);
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago", std::string(tmp));
  }

  std::vector<std::string>                     obstacle_names_;
  std::vector<std::shared_ptr<ObstacleSource>> sources_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionObstacleCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
