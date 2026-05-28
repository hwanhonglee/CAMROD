// HH_260528: camera_rear_publisher_node — rear econ camera publisher.
// Based on todo/econ_camera/src/econ_camera_node.cpp.
// Uses OpenCV GStreamer pipeline + CPU JPEG encoding (no GPU dependency).
// Publishes image_raw (uncompressed) required by Isaac ROS AprilTag in docking.
//
// Publishes:
//   ~/image_raw            (sensor_msgs/Image,           on demand)
//   ~/image_raw/compressed (sensor_msgs/CompressedImage, always — lightweight JPEG)
//   ~/camera_info          (sensor_msgs/CameraInfo)

#include <atomic>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <yaml-cpp/yaml.h>

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
    declare_parameter("frame_id",        std::string("camera_rear"));
    declare_parameter("camera_info_url", std::string(""));

    device_       = get_parameter("device").as_string();
    cap_w_        = get_parameter("width").as_int();
    cap_h_        = get_parameter("height").as_int();
    int pw        = get_parameter("publish_width").as_int();
    int ph        = get_parameter("publish_height").as_int();
    pub_w_        = (pw > 0) ? pw : cap_w_;
    pub_h_        = (ph > 0) ? ph : cap_h_;
    fps_          = get_parameter("fps").as_int();
    jpeg_quality_ = get_parameter("jpeg_quality").as_int();
    frame_id_     = get_parameter("frame_id").as_string();
    load_calibration(get_parameter("camera_info_url").as_string());

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
      "CameraRearPublisher ready  capture=%dx%d  publish=%dx%d  @%dfps  jpeg_q=%d  device=%s",
      cap_w_, cap_h_, pub_w_, pub_h_, fps_, jpeg_quality_, device_.c_str());
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

    std::string gst =
      "v4l2src device=" + device_ + " ! "
      "video/x-raw, format=UYVY"
      ", width=" + std::to_string(cap_w_) +
      ", height=" + std::to_string(cap_h_) +
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
          latest_frame_ = frame.clone();
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
        frame        = latest_frame_;
        frame_ready_ = false;
      }

      const auto stamp = now();

      // Compressed image (always publish — lightweight ~100 KB)
      {
        std::vector<uchar> buf;
        cv::imencode(".jpg", frame, buf,
                     {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});

        sensor_msgs::msg::CompressedImage comp;
        comp.header.stamp    = stamp;
        comp.header.frame_id = frame_id_;
        comp.format          = "jpeg";
        comp.data            = std::move(buf);
        compressed_pub_->publish(std::move(comp));
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
      auto ci          = build_camera_info();
      ci.header.stamp    = stamp;
      ci.header.frame_id = frame_id_;
      cinfo_pub_->publish(std::move(ci));
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

  void load_calibration(const std::string & url)
  {
    std::string path = url;
    if (path.rfind("file://", 0) == 0) path = path.substr(7);
    if (path.empty()) return;

    std::ifstream f(path);
    if (!f.good()) {
      RCLCPP_WARN(get_logger(), "Calibration file not found: %s", path.c_str());
      return;
    }
    try {
      YAML::Node root = YAML::LoadFile(path);
      calib_w_ = root["image_width"].as<int>();
      calib_h_ = root["image_height"].as<int>();

      auto fill_arr = [](const YAML::Node & n, auto & arr) {
        size_t i = 0;
        for (const auto & e : n["data"]) { if (i < arr.size()) arr[i++] = e.as<double>(); }
      };
      auto fill_vec = [](const YAML::Node & n, std::vector<double> & v) {
        v.clear();
        for (const auto & e : n["data"]) v.push_back(e.as<double>());
      };

      fill_arr(root["camera_matrix"],            k_);
      fill_vec(root["distortion_coefficients"],  d_);
      fill_arr(root["rectification_matrix"],     r_);
      fill_arr(root["projection_matrix"],        p_);
      calib_loaded_ = true;
      RCLCPP_INFO(get_logger(), "Calibration loaded: %s  (%dx%d)",
        path.c_str(), calib_w_, calib_h_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Calibration parse error: %s", e.what());
    }
  }

  std::string device_, frame_id_;
  int cap_w_, cap_h_, pub_w_, pub_h_, fps_, jpeg_quality_;

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
