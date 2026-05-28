#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class TagDistanceNode : public rclcpp::Node
{
public:
  TagDistanceNode()
  : Node("tag_distance_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    camera_frame_ = declare_parameter<std::string>(
      "camera_frame", "camera_rear");
    tag_frame_ = declare_parameter<std::string>(
      "tag_frame", "dock_tag");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);

    distance_pub_ = create_publisher<std_msgs::msg::Float32>("/docking/tag_distance", 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&TagDistanceNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
      "TagDistanceNode started  camera_frame=%s  tag_frame=%s  rate=%.1fHz",
      camera_frame_.c_str(), tag_frame_.c_str(), publish_rate_hz_);
  }

private:
  void timerCallback()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(camera_frame_, tag_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }

    const auto & t = tf.transform.translation;
    float dist = std::sqrt(t.x * t.x + t.y * t.y + t.z * t.z);

    std_msgs::msg::Float32 msg;
    msg.data = dist;
    distance_pub_->publish(msg);
  }

  std::string camera_frame_;
  std::string tag_frame_;
  double publish_rate_hz_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TagDistanceNode>());
  rclcpp::shutdown();
  return 0;
}
