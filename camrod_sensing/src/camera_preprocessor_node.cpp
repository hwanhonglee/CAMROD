#include <mutex>
#include <string>

#include <avg_msgs/msg/avg_sensing_camera.hpp>
#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/msg/camera_info.hpp>
#include <avg_msgs/msg/image.hpp>

namespace camping_cart::sensing
{

class CameraPreprocessorNode : public rclcpp::Node
{
public:
  using AvgSensingCamera = avg_msgs::msg::AvgSensingCamera;

  CameraPreprocessorNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : rclcpp::Node("camera_preprocessor")
  {
    // HH_260109 Use sensing-prefixed camera topics by default.
    input_image_topic_ = declare_parameter<std::string>("input_image_topic", "/sensing/camera/image_raw");
    input_camera_info_topic_ =
      declare_parameter<std::string>("input_camera_info_topic", "/sensing/camera/camera_info");
    output_image_topic_ =
      declare_parameter<std::string>("output_image_topic", "/sensing/camera/processed/image");
    output_camera_info_topic_ =
      declare_parameter<std::string>("output_camera_info_topic", "/sensing/camera/processed/camera_info");
    camera_diagnostic_topic_ = declare_parameter<std::string>(
      "camera_diagnostic_topic", "/sensing/camera/diagnostic");
    publish_camera_diagnostic_ = declare_parameter<bool>("publish_camera_diagnostic", false);
    // HH_260220: Keep camera messages on the sensor_kit TF frame by default.
    frame_id_override_ = declare_parameter<std::string>("frame_id_override", "camera_front_link");
    require_camera_info_ = declare_parameter<bool>("require_camera_info", false);

    using std::placeholders::_1;
    image_sub_ = create_subscription<avg_msgs::msg::Image>(
      input_image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CameraPreprocessorNode::onImage, this, _1));
    camera_info_sub_ = create_subscription<avg_msgs::msg::CameraInfo>(
      input_camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CameraPreprocessorNode::onCameraInfo, this, _1));

    image_pub_ = create_publisher<avg_msgs::msg::Image>(output_image_topic_, rclcpp::QoS(10));
    camera_info_pub_ = create_publisher<avg_msgs::msg::CameraInfo>(
      output_camera_info_topic_, rclcpp::QoS(10));
    avg_camera_pub_ = create_publisher<AvgSensingCamera>(camera_diagnostic_topic_, rclcpp::QoS(10));

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(
      get_logger(),
      "Camera preprocessor ready. image=%s info=%s output=%s",
      input_image_topic_.c_str(), input_camera_info_topic_.c_str(), output_image_topic_.c_str());
  }

private:
  // Handles the `onCameraInfo` callback.
  void onCameraInfo(const avg_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_camera_info_ = *msg;
    if (!frame_id_override_.empty()) {
      last_camera_info_.header.frame_id = frame_id_override_;
    }
    camera_info_ready_ = true;
    camera_info_pub_->publish(last_camera_info_);
    if (publish_camera_diagnostic_ && avg_camera_pub_) {
      AvgSensingCamera avg_msg;
      avg_msg.camera_info = last_camera_info_;
      avg_camera_pub_->publish(avg_msg);
    }
  }

  // Handles the `onImage` callback.
  void onImage(const avg_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (require_camera_info_ && !camera_info_ready_) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Camera info not ready; skipping image.");
      return;
    }

    avg_msgs::msg::Image out = *msg;
    if (!frame_id_override_.empty()) {
      out.header.frame_id = frame_id_override_;
    }
    image_pub_->publish(out);

    if (camera_info_ready_) {
      std::lock_guard<std::mutex> lock(mutex_);
      camera_info_pub_->publish(last_camera_info_);
      if (publish_camera_diagnostic_ && avg_camera_pub_) {
        AvgSensingCamera avg_msg;
        avg_msg.image = out;
        avg_msg.camera_info = last_camera_info_;
        avg_camera_pub_->publish(avg_msg);
      }
    } else if (publish_camera_diagnostic_ && avg_camera_pub_) {
      AvgSensingCamera avg_msg;
      avg_msg.image = out;
      avg_camera_pub_->publish(avg_msg);
    }
  }

  std::string input_image_topic_;
  std::string input_camera_info_topic_;
  std::string output_image_topic_;
  std::string output_camera_info_topic_;
  std::string camera_diagnostic_topic_;
  std::string frame_id_override_;
  bool require_camera_info_{false};
  bool publish_camera_diagnostic_{false};

  rclcpp::Subscription<avg_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<avg_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<avg_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<avg_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<AvgSensingCamera>::SharedPtr avg_camera_pub_;

  std::mutex mutex_;
  avg_msgs::msg::CameraInfo last_camera_info_;
  bool camera_info_ready_{false};
};

}  // namespace camping_cart::sensing

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camping_cart::sensing::CameraPreprocessorNode>());
  rclcpp::shutdown();
  return 0;
}
