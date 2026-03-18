#include <cmath>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>

#include <avg_msgs/msg/avg_perception_msgs.hpp>
#include <avg_msgs/msg/module_health.hpp>
#include <avg_msgs/msg/camera_info.hpp>
#include <avg_msgs/msg/point_cloud2.hpp>
#include <avg_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/msg/detection2_d_array.hpp>
#include <avg_msgs/msg/transform_stamped.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace camping_cart::perception
{

namespace
{
// HH_260109 Support multiple BoundingBox2D center definitions across vision_msgs variants.
template <typename T, typename = void>
struct HasMemberX : std::false_type {};

template <typename T>
struct HasMemberX<T, std::void_t<decltype(std::declval<T>().x)>> : std::true_type {};

template <typename T, typename = void>
struct HasMemberPosition : std::false_type {};

template <typename T>
struct HasMemberPosition<T, std::void_t<decltype(std::declval<T>().position.x)>> : std::true_type {};

template <typename CenterT>
// Implements `centerX` behavior.
double centerX(const CenterT & center)
{
  // Implements `constexpr` behavior.
  if constexpr (HasMemberX<CenterT>::value) {
    return center.x;
  // Implements `constexpr` behavior.
  } else if constexpr (HasMemberPosition<CenterT>::value) {
    return center.position.x;
  }
  return 0.0;
}

template <typename CenterT>
// Implements `centerY` behavior.
double centerY(const CenterT & center)
{
  // Implements `constexpr` behavior.
  if constexpr (HasMemberX<CenterT>::value) {
    return center.y;
  // Implements `constexpr` behavior.
  } else if constexpr (HasMemberPosition<CenterT>::value) {
    return center.position.y;
  }
  return 0.0;
}

template <typename CenterT>
// Implements `centerX` behavior.
double centerX(const CenterT * center)
{
  if (!center) {
    return 0.0;
  }
  return centerX(*center);
}

template <typename CenterT>
// Implements `centerY` behavior.
double centerY(const CenterT * center)
{
  if (!center) {
    return 0.0;
  }
  return centerY(*center);
}
}  // namespace

struct BBox2D
{
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
};

class ObstacleFusionNode : public rclcpp::Node
{
public:
  using AvgPerceptionMsgs = avg_msgs::msg::AvgPerceptionMsgs;
  using ModuleHealth = avg_msgs::msg::ModuleHealth;

  ObstacleFusionNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : rclcpp::Node("obstacle_fusion"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_cloud_topic_ = declare_parameter<std::string>(
      "input_cloud_topic", "/sensing/lidar/points_filtered");
    // HH_260109 Perception-level camera detections (e.g., YOLO) gate LiDAR obstacles.
    detection_topic_ = declare_parameter<std::string>(
      "detection_topic", "/perception/camera/detections_2d");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/sensing/camera/processed/camera_info");
    output_topic_ = declare_parameter<std::string>("output_topic", "/perception/obstacles");
    publish_perception_diagnostic_ = declare_parameter<bool>("publish_perception_diagnostic", false);
    perception_diagnostic_topic_ =
      declare_parameter<std::string>("perception_diagnostic_topic", "/perception/diagnostic");
    // HH_260220: camera_front_link is provided by sensor_kit URDF (/tf_static).
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_front_link");
    use_camera_filter_ = declare_parameter<bool>("use_camera_filter", true);
    keep_lidar_when_no_detections_ =
      declare_parameter<bool>("keep_lidar_when_no_detections", true);

    using std::placeholders::_1;
    cloud_sub_ = create_subscription<avg_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleFusionNode::onCloud, this, _1));
    detection_sub_ = create_subscription<avg_msgs::msg::Detection2DArray>(
      detection_topic_, rclcpp::QoS(5),
      std::bind(&ObstacleFusionNode::onDetections, this, _1));
    camera_info_sub_ = create_subscription<avg_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::QoS(5),
      std::bind(&ObstacleFusionNode::onCameraInfo, this, _1));
    out_pub_ = create_publisher<avg_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS(10));
    if (publish_perception_diagnostic_) {
      avg_pub_ = create_publisher<AvgPerceptionMsgs>(perception_diagnostic_topic_, rclcpp::QoS(10));
    }

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(get_logger(),
      "Obstacle fusion ready. cloud=%s detections=%s output=%s",
      input_cloud_topic_.c_str(), detection_topic_.c_str(), output_topic_.c_str());
  }

private:
  // Handles the `onCameraInfo` callback.
  void onCameraInfo(const avg_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_model_.fromCameraInfo(msg);
    latest_camera_info_ = *msg;
    camera_info_ready_ = true;
  }

  // Handles the `onDetections` callback.
  void onDetections(const avg_msgs::msg::Detection2DArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // HH_260109 Cache 2D detection boxes for LiDAR gating.
    detections_.clear();
    detections_.reserve(msg->detections.size());
    latest_detections_ = *msg;
    for (const auto & det : msg->detections) {
      const auto & bbox = det.bbox;
      const auto & center = bbox.center;
      const double center_x = centerX(center);
      const double center_y = centerY(center);
      const double half_w = bbox.size_x * 0.5;
      const double half_h = bbox.size_y * 0.5;
      BBox2D box;
      box.min_x = center_x - half_w;
      box.max_x = center_x + half_w;
      box.min_y = center_y - half_h;
      box.max_y = center_y + half_h;
      detections_.push_back(box);
    }
  }

  // Handles the `onCloud` callback.
  void onCloud(const avg_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    std::vector<BBox2D> boxes;
    image_geometry::PinholeCameraModel camera_model;
    bool camera_ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      boxes = detections_;
      camera_model = camera_model_;
      camera_ready = camera_info_ready_;
    }

    // HH_260109 When no camera constraints are available, fall back to LiDAR-only obstacles.
    if (!use_camera_filter_ || !camera_ready || (boxes.empty() && keep_lidar_when_no_detections_)) {
      out_pub_->publish(*msg);
      publishAvgPerception(*msg);
      return;
    }
    if (boxes.empty() && !keep_lidar_when_no_detections_) {
      publishAvgPerception(avg_msgs::msg::PointCloud2{});
      return;
    }

    avg_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(
        camera_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup failed (%s->%s): %s",
        msg->header.frame_id.c_str(), camera_frame_.c_str(), ex.what());
      out_pub_->publish(*msg);
      publishAvgPerception(*msg);
      return;
    }

    tf2::Transform tf_cam_from_lidar;
    tf2::fromMsg(tf_msg.transform, tf_cam_from_lidar);

    std::vector<std::array<float, 3>> kept;
    kept.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const tf2::Vector3 p_lidar(x, y, z);
      const tf2::Vector3 p_cam = tf_cam_from_lidar * p_lidar;
      if (p_cam.z() <= 0.01) {
        continue;
      }

      // HH_260109 Project LiDAR points into the camera image and keep points inside detections.
      const auto uv = camera_model.project3dToPixel(
        cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
      const double u = uv.x;
      const double v = uv.y;
      bool in_box = false;
      for (const auto & box : boxes) {
        if (u >= box.min_x && u <= box.max_x && v >= box.min_y && v <= box.max_y) {
          in_box = true;
          break;
        }
      }
      if (in_box) {
        kept.push_back({x, y, z});
      }
    }

    avg_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(kept.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");
    for (const auto & p : kept) {
      *out_x = p[0];
      *out_y = p[1];
      *out_z = p[2];
      ++out_x; ++out_y; ++out_z;
    }

    out_pub_->publish(out);
    publishAvgPerception(out);
  }

  // Publishes `AvgPerception` output.
  void publishAvgPerception(const avg_msgs::msg::PointCloud2 & obstacles)
  {
    if (!publish_perception_diagnostic_ || !avg_pub_) {
      return;
    }

    AvgPerceptionMsgs msg;
    msg.stamp = now();
    msg.health.stamp = msg.stamp;
    msg.health.module_name = "perception";
    msg.health.level = ModuleHealth::OK;
    msg.health.message = "obstacle_fusion";
    msg.obstacles = obstacles;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      msg.camera_info = latest_camera_info_;
      msg.detections = latest_detections_;
    }
    avg_pub_->publish(msg);
  }

  std::string input_cloud_topic_;
  std::string detection_topic_;
  std::string camera_info_topic_;
  std::string output_topic_;
  std::string perception_diagnostic_topic_;
  std::string camera_frame_;
  bool use_camera_filter_{true};
  bool keep_lidar_when_no_detections_{true};
  bool publish_perception_diagnostic_{false};

  rclcpp::Subscription<avg_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<avg_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<avg_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<avg_msgs::msg::PointCloud2>::SharedPtr out_pub_;
  rclcpp::Publisher<AvgPerceptionMsgs>::SharedPtr avg_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex mutex_;
  image_geometry::PinholeCameraModel camera_model_;
  bool camera_info_ready_{false};
  std::vector<BBox2D> detections_;
  avg_msgs::msg::CameraInfo latest_camera_info_;
  avg_msgs::msg::Detection2DArray latest_detections_;
};

}  // namespace camping_cart::perception

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camping_cart::perception::ObstacleFusionNode>());
  rclcpp::shutdown();
  return 0;
}
