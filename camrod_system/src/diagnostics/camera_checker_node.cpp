/**
 * Camera Checker Node
 *
 * HH_260630 - Checks raw Image or CompressedImage streams plus CameraInfo.
 * Use `image_type: raw` for sensor_msgs/Image and `image_type: compressed`
 * for sensor_msgs/CompressedImage. Compressed streams use CameraInfo to fill
 * width/height because CompressedImage does not carry image dimensions.
 */

#include <algorithm>
#include <cctype>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

struct CameraState
{
  std::string name;
  std::string image_topic;
  std::string image_type{"raw"};
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
  std::deque<rclcpp::Time> image_timestamps;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
};

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

      // HH_260630 - Camera sections are declared dynamically from camera_names.
      declare_parameter(name + ".image_topic",
        std::string("/sensing/camera/" + name + "/image_raw"));
      declare_parameter(name + ".image_type", std::string("raw"));
      declare_parameter(name + ".camera_info_topic",
        std::string("/sensing/camera/" + name + "/camera_info"));
      declare_parameter(name + ".expected_fps",      30.0);
      declare_parameter(name + ".fps_warn_ratio",    0.8);
      declare_parameter(name + ".fps_error_ratio",   0.5);
      declare_parameter(name + ".stale_timeout_s",     2.0);
      declare_parameter(name + ".expected_width",    int64_t(0));
      declare_parameter(name + ".expected_height",   int64_t(0));
      declare_parameter(name + ".expected_encoding", std::string(""));

      cam->image_topic       = get_parameter(name + ".image_topic").as_string();
      cam->image_type        = get_parameter(name + ".image_type").as_string();
      std::transform(
        cam->image_type.begin(), cam->image_type.end(), cam->image_type.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (cam->image_type != "raw" && cam->image_type != "image" &&
        cam->image_type != "compressed")
      {
        RCLCPP_WARN(
          get_logger(), "Unknown camera image_type '%s' for %s; using raw",
          cam->image_type.c_str(), name.c_str());
        cam->image_type = "raw";
      }
      cam->camera_info_topic = get_parameter(name + ".camera_info_topic").as_string();
      cam->expected_fps      = get_parameter(name + ".expected_fps").as_double();
      cam->fps_warn_ratio    = get_parameter(name + ".fps_warn_ratio").as_double();
      cam->fps_error_ratio   = get_parameter(name + ".fps_error_ratio").as_double();
      cam->stale_timeout = get_param<double>(name + ".stale_timeout_s", cam->stale_timeout);
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
      // HH_260630 - Accept raw and compressed camera streams so checker targets
      // match the operational topic contracts used by sensing/perception.
      if (cam->image_type == "compressed") {
        cam->compressed_image_sub = create_subscription<sensor_msgs::msg::CompressedImage>(
          cam->image_topic, rclcpp::SensorDataQoS(),
          [this, cam](const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(cam->mtx);
            const auto now = this->now();
            record_image_sample(*cam, now);
            cam->actual_encoding = msg->format.empty() ? "compressed" : msg->format;
          });
      } else {
        cam->image_sub = create_subscription<sensor_msgs::msg::Image>(
          cam->image_topic, rclcpp::SensorDataQoS(),
          [this, cam](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(cam->mtx);
            const auto now = this->now();
            record_image_sample(*cam, now);
            cam->actual_width    = msg->width;
            cam->actual_height   = msg->height;
            cam->actual_encoding = msg->encoding;
          });
      }

      cam->info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
        cam->camera_info_topic, rclcpp::SensorDataQoS(),
        [cam](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(cam->mtx);
          cam->has_camera_info = true;
          if (cam->actual_width == 0 && msg->width > 0) {
            cam->actual_width = msg->width;
          }
          if (cam->actual_height == 0 && msg->height > 0) {
            cam->actual_height = msg->height;
          }
        });

      add_task("/sensor/camera/" + cam->name,
        [this, cam](StatusWrapper & stat) { check_camera(stat, *cam); });

      RCLCPP_INFO(get_logger(),
        "카메라 모니터링 시작: %s (image=%s, type=%s, expected_fps=%.0f)",
        cam->name.c_str(), cam->image_topic.c_str(), cam->image_type.c_str(),
        cam->expected_fps);
    }
  }

private:
  static void record_image_sample(CameraState & cam, const rclcpp::Time & now)
  {
    cam.last_image_time = now;
    cam.has_image       = true;

    cam.image_timestamps.push_back(now);
    while (!cam.image_timestamps.empty() &&
           (now - cam.image_timestamps.front()).seconds() > 2.0)
    {
      cam.image_timestamps.pop_front();
    }
  }

  void check_camera(StatusWrapper & stat, CameraState & cam)
  {
    std::lock_guard<std::mutex> lock(cam.mtx);

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

    double actual_fps = 0.0;
    if (cam.image_timestamps.size() >= 2) {
      double window =
        (cam.image_timestamps.back() - cam.image_timestamps.front()).seconds();
      if (window > 0.0) {
        actual_fps = static_cast<double>(cam.image_timestamps.size() - 1) / window;
      }
    }

    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str = "OK";

    if (cam.expected_fps > 0.0) {
      double ratio = actual_fps / cam.expected_fps;
      lvl = check_low(ratio, cam.fps_warn_ratio, cam.fps_error_ratio);
      if      (lvl == S::ERROR) msg_str = "FPS 심각 저하";
      else if (lvl == S::WARN)  msg_str = "FPS 저하";
    }

    if (cam.expected_width > 0) {
      lvl = std::max(lvl, check_flag(cam.actual_width == cam.expected_width, S::WARN));
      if (lvl == S::WARN) msg_str = "해상도 불일치";
    }
    if (cam.expected_height > 0) {
      lvl = std::max(lvl, check_flag(cam.actual_height == cam.expected_height, S::WARN));
      if (lvl == S::WARN) msg_str = "해상도 불일치";
    }

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

    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_fps);
    stat.add("actual_fps",       std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", cam.expected_fps);
    stat.add("expected_fps",     std::string(tmp));
    stat.add("image_type",       cam.image_type);
    stat.add("width",            cam.actual_width);
    stat.add("height",           cam.actual_height);
    stat.add("encoding",         cam.actual_encoding);
    stat.add("camera_info",      cam.has_camera_info ? "OK" : "없음");
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago", std::string(tmp));
  }

  std::vector<std::shared_ptr<CameraState>> cameras_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
