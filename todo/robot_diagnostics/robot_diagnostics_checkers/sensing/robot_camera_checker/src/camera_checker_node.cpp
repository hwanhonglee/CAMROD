/**
 * Camera Checker Node
 *
 * sensor_msgs/Image 및 sensor_msgs/CameraInfo 토픽을 구독하여
 * 카메라 상태(staleness, FPS, 해상도, 인코딩, camera_info 유무)를
 * /diagnostics 토픽으로 발행한다.
 *
 * 파라미터 구성
 * -------------
 *   camera_names: ["front", "rear"]   ← 모니터링할 카메라 이름 목록
 *
 *   front:
 *     image_topic:       "/sensing/camera/front/image_raw"
 *     camera_info_topic: "/sensing/camera/front/camera_info"
 *     expected_fps:      30.0
 *     fps_warn_ratio:    0.8    # actual_fps / expected_fps 비율
 *     fps_error_ratio:   0.5
 *     stale_timeout:     2.0    # 초 이상 메시지 없으면 STALE
 *     expected_width:    0      # 0 = 체크 안 함
 *     expected_height:   0
 *     expected_encoding: ""     # "" = 체크 안 함
 */

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── 카메라 상태 구조체 ────────────────────────────────────────────────────

struct CameraState
{
  // 설정
  std::string name;
  std::string image_topic;
  std::string camera_info_topic;
  double expected_fps{30.0};
  double fps_warn_ratio{0.8};
  double fps_error_ratio{0.5};
  double stale_timeout{2.0};
  uint32_t expected_width{0};
  uint32_t expected_height{0};
  std::string expected_encoding{};

  // 런타임 상태 (mutex 보호)
  std::mutex mtx;
  rclcpp::Time last_image_time{0, 0, RCL_ROS_TIME};
  bool has_image{false};
  bool has_camera_info{false};
  uint32_t actual_width{0};
  uint32_t actual_height{0};
  std::string actual_encoding{};
  std::deque<rclcpp::Time> image_timestamps;  // FPS 계산용 rolling window (2s)

  // 구독자
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
};

// ── CameraCheckerNode ─────────────────────────────────────────────────────

class CameraCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  CameraCheckerNode()
  : robot_diagnostics_base::BaseChecker("camera_checker", "camera_checker")
  {
    base_init();
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("camera_names", std::vector<std::string>{});
  }

  void load_parameters_() override
  {
    auto names = get_parameter("camera_names").as_string_array();

    for (const auto & name : names) {
      auto cam = std::make_shared<CameraState>();
      cam->name = name;

      // 카메라별 파라미터 선언 (camera_names 로드 후 동적으로 선언)
      declare_parameter(name + ".image_topic",
        std::string("/sensing/camera/" + name + "/image_raw"));
      declare_parameter(name + ".camera_info_topic",
        std::string("/sensing/camera/" + name + "/camera_info"));
      declare_parameter(name + ".expected_fps",      30.0);
      declare_parameter(name + ".fps_warn_ratio",    0.8);
      declare_parameter(name + ".fps_error_ratio",   0.5);
      declare_parameter(name + ".stale_timeout",     2.0);
      declare_parameter(name + ".expected_width",    int64_t(0));
      declare_parameter(name + ".expected_height",   int64_t(0));
      declare_parameter(name + ".expected_encoding", std::string(""));

      cam->image_topic       = get_parameter(name + ".image_topic").as_string();
      cam->camera_info_topic = get_parameter(name + ".camera_info_topic").as_string();
      cam->expected_fps      = get_parameter(name + ".expected_fps").as_double();
      cam->fps_warn_ratio    = get_parameter(name + ".fps_warn_ratio").as_double();
      cam->fps_error_ratio   = get_parameter(name + ".fps_error_ratio").as_double();
      cam->stale_timeout     = get_parameter(name + ".stale_timeout").as_double();
      cam->expected_width    = static_cast<uint32_t>(
        get_parameter(name + ".expected_width").as_int());
      cam->expected_height   = static_cast<uint32_t>(
        get_parameter(name + ".expected_height").as_int());
      cam->expected_encoding = get_parameter(name + ".expected_encoding").as_string();

      cameras_.push_back(cam);
    }
  }

  void setup_tasks_() override
  {
    for (auto & cam : cameras_) {
      // Image 구독
      cam->image_sub = create_subscription<sensor_msgs::msg::Image>(
        cam->image_topic, rclcpp::SensorDataQoS(),
        [this, cam](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(cam->mtx);
          auto now = this->now();
          cam->last_image_time = now;
          cam->has_image       = true;
          cam->actual_width    = msg->width;
          cam->actual_height   = msg->height;
          cam->actual_encoding = msg->encoding;

          // FPS 계산용 rolling window (2s)
          cam->image_timestamps.push_back(now);
          while (!cam->image_timestamps.empty() &&
                 (now - cam->image_timestamps.front()).seconds() > 2.0)
          {
            cam->image_timestamps.pop_front();
          }
        });

      // CameraInfo 구독
      cam->info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
        cam->camera_info_topic, rclcpp::SensorDataQoS(),
        [cam](const sensor_msgs::msg::CameraInfo::ConstSharedPtr /*msg*/) {
          std::lock_guard<std::mutex> lock(cam->mtx);
          cam->has_camera_info = true;
        });

      // 진단 태스크 등록
      add_task("/sensor/camera/" + cam->name,
        [this, cam](StatusWrapper & stat) { check_camera(stat, *cam); });

      RCLCPP_INFO(get_logger(),
        "카메라 모니터링 시작: %s (image=%s, expected_fps=%.0f)",
        cam->name.c_str(), cam->image_topic.c_str(), cam->expected_fps);
    }
  }

private:
  void check_camera(StatusWrapper & stat, CameraState & cam)
  {
    std::lock_guard<std::mutex> lock(cam.mtx);

    // ── Staleness 체크 ──────────────────────────────────────────────────
    if (!cam.has_image) {
      stat.summary(DiagnosticStatus::STALE, "토픽 수신 없음: " + cam.image_topic);
      stat.add("topic", cam.image_topic);
      return;
    }

    double elapsed = (this->now() - cam.last_image_time).seconds();
    if (elapsed > cam.stale_timeout) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "%.1fs 동안 메시지 없음 (timeout=%.1fs)", elapsed, cam.stale_timeout);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
      stat.add("last_msg_sec_ago", elapsed);
      return;
    }

    // ── FPS 계산 (rolling 2s window) ───────────────────────────────────
    double actual_fps = 0.0;
    if (cam.image_timestamps.size() >= 2) {
      double window =
        (cam.image_timestamps.back() - cam.image_timestamps.front()).seconds();
      if (window > 0.0) {
        actual_fps = static_cast<double>(cam.image_timestamps.size() - 1) / window;
      }
    }

    // ── 레벨 판정 ───────────────────────────────────────────────────────
    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str = "OK";

    // FPS 체크 (ratio 낮을수록 위험 → check_low)
    if (cam.expected_fps > 0.0) {
      double ratio = actual_fps / cam.expected_fps;
      lvl = check_low(ratio, cam.fps_warn_ratio, cam.fps_error_ratio);
      if      (lvl == S::ERROR) msg_str = "FPS 심각 저하";
      else if (lvl == S::WARN)  msg_str = "FPS 저하";
    }

    // 해상도 체크 (불일치 시 WARN → check_flag)
    if (cam.expected_width > 0) {
      lvl = std::max(lvl, check_flag(cam.actual_width == cam.expected_width, S::WARN));
      if (lvl == S::WARN) msg_str = "해상도 불일치";
    }
    if (cam.expected_height > 0) {
      lvl = std::max(lvl, check_flag(cam.actual_height == cam.expected_height, S::WARN));
      if (lvl == S::WARN) msg_str = "해상도 불일치";
    }

    // 인코딩 체크 (불일치 시 WARN → check_flag)
    if (!cam.expected_encoding.empty()) {
      lvl = std::max(lvl, check_flag(cam.actual_encoding == cam.expected_encoding, S::WARN));
      if (lvl == S::WARN) msg_str = "인코딩 불일치";
    }

    if (lvl == DiagnosticStatus::OK) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "OK (%.1f fps)", actual_fps);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    // ── 상세 값 추가 ────────────────────────────────────────────────────
    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_fps);
    stat.add("actual_fps",       std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", cam.expected_fps);
    stat.add("expected_fps",     std::string(tmp));
    stat.add("width",            cam.actual_width);
    stat.add("height",           cam.actual_height);
    stat.add("encoding",         cam.actual_encoding);
    stat.add("camera_info",      cam.has_camera_info ? "OK" : "없음");
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago", std::string(tmp));
  }

  std::vector<std::shared_ptr<CameraState>> cameras_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
