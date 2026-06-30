// HH_260528 - camera_rear_publisher_node — rear econ camera publisher.
// Based on todo/econ_camera/src/econ_camera_node.cpp.
// Uses OpenCV GStreamer pipeline + CPU JPEG encoding (no GPU dependency).
// Publishes image_raw (uncompressed) required by Isaac ROS AprilTag in docking.
//
// HH_260601 - replace camera_info_url file loading with inline ROS parameter calibration
//            (camera_matrix, distortion_coefficients, rectification_matrix, projection_matrix).
//            Removed yaml-cpp and fstream dependencies.
// HH_260630 - Rate-limit rear CPU JPEG publishing and avoid frame deep-copy handoff.
//
// Publishes:
//   ~/image_raw            (sensor_msgs/Image,           on demand)
//   ~/image_raw/compressed (sensor_msgs/CompressedImage, rate-limited CPU JPEG)
//   ~/camera_info          (sensor_msgs/CameraInfo)

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraRearPublisherNode : public rclcpp::Node
{
public:
  CameraRearPublisherNode() : Node("camera_rear_publisher")
  {
    declare_parameter("device",          std::string("/dev/video1"));
    declare_parameter("width",           1920);
    declare_parameter("height",          1080);
    declare_parameter("publish_width",   0);
    declare_parameter("publish_height",  0);
    declare_parameter("fps",             30);
    declare_parameter("jpeg_quality",    80);
    declare_parameter("compressed_publish_rate_hz", 10.0);
    declare_parameter("publish_compressed_without_subscribers", false);
    declare_parameter("frame_id",        std::string("camera_rear"));
    declare_parameter("camera_matrix",           std::vector<double>{});
    declare_parameter("distortion_coefficients", std::vector<double>{});
    declare_parameter("rectification_matrix",    std::vector<double>{});
    declare_parameter("projection_matrix",       std::vector<double>{});

    device_       = get_parameter("device").as_string();
    cap_w_        = get_parameter("width").as_int();
    cap_h_        = get_parameter("height").as_int();
    int pw        = get_parameter("publish_width").as_int();
    int ph        = get_parameter("publish_height").as_int();
    pub_w_        = (pw > 0) ? pw : cap_w_;
    pub_h_        = (ph > 0) ? ph : cap_h_;
    fps_          = get_parameter("fps").as_int();
    jpeg_quality_ = get_parameter("jpeg_quality").as_int();
    compressed_publish_rate_hz_ =
      get_parameter("compressed_publish_rate_hz").as_double();
    publish_compressed_without_subscribers_ =
      get_parameter("publish_compressed_without_subscribers").as_bool();
    if (compressed_publish_rate_hz_ < 0.0) {
      RCLCPP_WARN(get_logger(),
        "compressed_publish_rate_hz %.2f is invalid; using 0.0 (disabled)",
        compressed_publish_rate_hz_);
      compressed_publish_rate_hz_ = 0.0;
    }
    compressed_period_ns_ =
      compressed_publish_rate_hz_ > 0.0
        ? static_cast<int64_t>(1.0e9 / compressed_publish_rate_hz_)
        : 0;
    frame_id_     = get_parameter("frame_id").as_string();
    load_calibration_from_params();
    camera_info_msg_ = build_camera_info();
    camera_info_msg_.header.frame_id = frame_id_;

    image_pub_      = create_publisher<sensor_msgs::msg::Image>("~/image_raw", 2);
    compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                        "~/image_raw/compressed", 2);
    cinfo_pub_      = create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 2);

    if (!open_camera()) {
      RCLCPP_FATAL(get_logger(), "Failed to open camera %s", device_.c_str());
      return;
    }

    running_ = true;
    capture_thread_ = std::thread(&CameraRearPublisherNode::capture_loop, this);
    publish_thread_ = std::thread(&CameraRearPublisherNode::publish_loop, this);

    RCLCPP_INFO(get_logger(),
      "CameraRearPublisher ready  capture=%dx%d  publish=%dx%d  @%dfps  jpeg_q=%d  compressed_rate=%.2fHz  device=%s",
      cap_w_, cap_h_, pub_w_, pub_h_, fps_, jpeg_quality_,
      compressed_publish_rate_hz_, device_.c_str());
  }

  ~CameraRearPublisherNode()
  {
    running_ = false;
    cv_.notify_all();
    if (capture_thread_.joinable()) capture_thread_.join();
    if (publish_thread_.joinable())  publish_thread_.join();
    if (cap_.isOpened()) cap_.release();
  }

private:
  bool open_camera()
  {
    // nvvidconv handles UYVY→NV12 in NVMM (hardware VIC) and optional scale in one pass
    std::string nv12_caps = "video/x-raw(memory:NVMM), format=NV12";
    if (pub_w_ != cap_w_ || pub_h_ != cap_h_) {
      nv12_caps += ", width=" + std::to_string(pub_w_) +
                   ", height=" + std::to_string(pub_h_);
    }

    // HH_260630 - Match the front camera capture pattern. The ISX031 sensor is
    // stable at 30 fps; videorate drops frames before VIC conversion so rear
    // does not fall back to CPU V4L2 when the requested publish fps is lower.
    std::string gst =
      "v4l2src device=" + device_ + " ! "
      "video/x-raw, format=UYVY"
      ", width=" + std::to_string(cap_w_) +
      ", height=" + std::to_string(cap_h_) +
      ", framerate=30/1 ! "
      "videorate ! video/x-raw, format=UYVY"
      ", framerate=" + std::to_string(fps_) + "/1 ! "
      "nvvidconv ! " + nv12_caps + " ! "
      "nvvidconv ! video/x-raw, format=BGRx ! "
      "videoconvert ! video/x-raw, format=BGR ! "
      "appsink drop=true max-buffers=1 sync=false";

    RCLCPP_INFO(get_logger(), "GStreamer: %s", gst.c_str());
    cap_.open(gst, cv::CAP_GSTREAMER);

    if (!cap_.isOpened()) {
      RCLCPP_WARN(get_logger(), "GStreamer failed, falling back to V4L2 direct");
      cap_.open(device_, cv::CAP_V4L2);
      cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('U','Y','V','Y'));
      cap_.set(cv::CAP_PROP_FRAME_WIDTH,  cap_w_);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cap_h_);
      cap_.set(cv::CAP_PROP_FPS,          fps_);
      cap_.set(cv::CAP_PROP_BUFFERSIZE,   1);
    }
    return cap_.isOpened();
  }

  void capture_loop()
  {
    cv::Mat frame;
    while (running_ && rclcpp::ok()) {
      if (cap_.read(frame) && !frame.empty()) {
        {
          std::lock_guard<std::mutex> lk(frame_mutex_);
          latest_frame_ = std::move(frame);
          frame_ready_  = true;
        }
        cv_.notify_one();
      }
    }
  }

  void publish_loop()
  {
    while (running_ && rclcpp::ok()) {
      cv::Mat frame;
      {
        std::unique_lock<std::mutex> lk(frame_mutex_);
        cv_.wait(lk, [this] { return frame_ready_ || !running_; });
        if (!running_) break;
        frame        = std::move(latest_frame_);
        frame_ready_ = false;
      }

      const auto stamp = now();
      const int64_t stamp_ns = stamp.nanoseconds();

      // HH_260630 - CPU JPEG is the rear camera's dominant load, so keep the
      // monitoring topic name stable but publish it only when needed and due.
      const bool compressed_has_subscribers =
        compressed_pub_->get_subscription_count() > 0;
      const bool compressed_enabled =
        compressed_publish_rate_hz_ > 0.0 &&
        (publish_compressed_without_subscribers_ || compressed_has_subscribers);
      const bool compressed_due =
        compressed_enabled &&
        (last_compressed_publish_ns_ == 0 ||
         stamp_ns < last_compressed_publish_ns_ ||
         stamp_ns - last_compressed_publish_ns_ >= compressed_period_ns_);

      if (compressed_due) {
        std::vector<uchar> buf;
        cv::imencode(".jpg", frame, buf,
                     {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});

        sensor_msgs::msg::CompressedImage comp;
        comp.header.stamp    = stamp;
        comp.header.frame_id = frame_id_;
        comp.format          = "jpeg";
        comp.data            = std::move(buf);
        compressed_pub_->publish(std::move(comp));
        last_compressed_publish_ns_ = stamp_ns;
      }

      // Raw image (only if someone subscribed — heavy ~1.5 MB, required by Isaac ROS AprilTag)
      if (image_pub_->get_subscription_count() > 0) {
        auto img_msg = cv_bridge::CvImage(
          std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        img_msg->header.stamp    = stamp;
        img_msg->header.frame_id = frame_id_;
        image_pub_->publish(std::move(*img_msg));
      }

      // CameraInfo
      camera_info_msg_.header.stamp = stamp;
      cinfo_pub_->publish(camera_info_msg_);
    }
  }

  sensor_msgs::msg::CameraInfo build_camera_info()
  {
    sensor_msgs::msg::CameraInfo ci;
    ci.width  = pub_w_;
    ci.height = pub_h_;

    if (calib_loaded_) {
      double sx = static_cast<double>(pub_w_) / calib_w_;
      double sy = static_cast<double>(pub_h_) / calib_h_;

      ci.distortion_model = "plumb_bob";
      ci.d = d_;
      std::copy(k_.begin(), k_.end(), ci.k.begin());
      std::copy(r_.begin(), r_.end(), ci.r.begin());
      std::copy(p_.begin(), p_.end(), ci.p.begin());

      ci.k[0] *= sx; ci.k[2] *= sx;
      ci.k[4] *= sy; ci.k[5] *= sy;
      ci.p[0] *= sx; ci.p[2] *= sx;
      ci.p[5] *= sy; ci.p[6] *= sy;
    } else {
      double fx = pub_w_;
      ci.distortion_model = "plumb_bob";
      ci.d = {0, 0, 0, 0, 0};
      ci.k = {fx, 0, pub_w_/2.0, 0, fx, pub_h_/2.0, 0, 0, 1};
      ci.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
      ci.p = {fx, 0, pub_w_/2.0, 0, 0, fx, pub_h_/2.0, 0, 0, 0, 1, 0};
    }
    return ci;
  }

  void load_calibration_from_params()
  {
    auto km = get_parameter("camera_matrix").as_double_array();
    auto dm = get_parameter("distortion_coefficients").as_double_array();
    auto rm = get_parameter("rectification_matrix").as_double_array();
    auto pm = get_parameter("projection_matrix").as_double_array();

    if (km.size() != 9) {
      RCLCPP_WARN(get_logger(), "camera_matrix must have 9 elements — using identity fallback");
      return;
    }
    std::copy(km.begin(), km.end(), k_.begin());
    d_ = dm;

    if (rm.size() == 9) {
      std::copy(rm.begin(), rm.end(), r_.begin());
    } else {
      r_.fill(0.0);
      r_[0] = r_[4] = r_[8] = 1.0;
    }

    if (pm.size() == 12) {
      std::copy(pm.begin(), pm.end(), p_.begin());
    } else {
      p_.fill(0.0);
      p_[0] = k_[0]; p_[2] = k_[2];
      p_[5] = k_[4]; p_[6] = k_[5];
      p_[10] = 1.0;
    }

    calib_w_      = cap_w_;
    calib_h_      = cap_h_;
    calib_loaded_ = true;
    RCLCPP_INFO(get_logger(), "Calibration loaded from ROS parameters (%dx%d)", calib_w_, calib_h_);
  }

  std::string device_, frame_id_;
  int cap_w_, cap_h_, pub_w_, pub_h_, fps_, jpeg_quality_;
  double compressed_publish_rate_hz_{10.0};
  bool publish_compressed_without_subscribers_{false};
  int64_t compressed_period_ns_{100000000};
  int64_t last_compressed_publish_ns_{0};

  cv::VideoCapture  cap_;
  cv::Mat           latest_frame_;
  std::mutex        frame_mutex_;
  std::condition_variable cv_;
  bool              frame_ready_{false};
  std::atomic<bool> running_{false};
  std::thread       capture_thread_;
  std::thread       publish_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr            image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr  compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr       cinfo_pub_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  bool calib_loaded_{false};
  int  calib_w_{1920}, calib_h_{1080};
  std::array<double, 9>  k_{}, r_{};
  std::array<double, 12> p_{};
  std::vector<double>    d_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraRearPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
