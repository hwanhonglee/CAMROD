#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace camping_cart::sensing
{

struct PointXYZ
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

class LidarPreprocessorNode : public rclcpp::Node
{
public:
  LidarPreprocessorNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : rclcpp::Node("lidar_preprocessor")
  {
    // HH_260109 Use sensing-prefixed LiDAR topic by default.
    input_topic_ = declare_parameter<std::string>("input_topic", "/sensing/lidar/points");
    // HH_260109 Publish preprocessed LiDAR points only; perception handles obstacles/objects.
    filtered_topic_ = declare_parameter<std::string>(
      "filtered_topic", "/sensing/lidar/points_filtered");
    min_range_ = declare_parameter<double>("min_range", 0.3);
    max_range_ = declare_parameter<double>("max_range", 35.0);
    min_z_ = declare_parameter<double>("min_z", -1.0);
    max_z_ = declare_parameter<double>("max_z", 2.0);
    // HH_260220: Keep point cloud frame on the sensor_kit TF tree by default.
    frame_id_override_ = declare_parameter<std::string>("frame_id_override", "lidar_link");
    publish_filtered_ = declare_parameter<bool>("publish_filtered", true);

    using std::placeholders::_1;
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarPreprocessorNode::onCloud, this, _1));
    filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(filtered_topic_, rclcpp::QoS(10));
    // HH_260109 No obstacle/object output in sensing stage.

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(get_logger(),
      "LiDAR preprocessor ready. input=%s filtered=%s",
      input_topic_.c_str(), filtered_topic_.c_str());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    std::vector<PointXYZ> points;
    points.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      const float x = *iter_x;
      const float y = *iter_y;
      const float z = *iter_z;
      const double range = std::hypot(x, y);

      // HH_260109 Apply range/Z filtering for obstacle extraction in forest paths.
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (range < min_range_ || range > max_range_) {
        continue;
      }
      if (z < min_z_ || z > max_z_) {
        continue;
      }
      points.push_back(PointXYZ{x, y, z});
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    if (!frame_id_override_.empty()) {
      out.header.frame_id = frame_id_override_;
    }

    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");
    for (const auto & p : points) {
      *out_x = p.x;
      *out_y = p.y;
      *out_z = p.z;
      ++out_x; ++out_y; ++out_z;
    }

    if (publish_filtered_) {
      filtered_pub_->publish(out);
    }
    // HH_260109 Obstacle/object extraction moves to perception.
  }

  std::string input_topic_;
  std::string filtered_topic_;
  std::string frame_id_override_;
  double min_range_{0.3};
  double max_range_{35.0};
  double min_z_{-1.0};
  double max_z_{2.0};
  bool publish_filtered_{true};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
};

}  // namespace camping_cart::sensing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camping_cart::sensing::LidarPreprocessorNode>());
  rclcpp::shutdown();
  return 0;
}
