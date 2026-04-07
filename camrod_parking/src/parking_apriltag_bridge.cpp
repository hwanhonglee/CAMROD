#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "avg_msgs/msg/avg_april_tag_detection.hpp"
#include "avg_msgs/msg/avg_april_tag_detection_array.hpp"
#include "avg_msgs/msg/avg_april_tag_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;

class ParkingAprilTagBridge : public rclcpp::Node
{
public:
  ParkingAprilTagBridge()
  : Node("parking_apriltag_bridge"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_detection_topic_ = declare_parameter<std::string>(
      "input_detection_topic", "/parking/docking/apriltag/detections_raw");

    output_avg_detection_topic_ = declare_parameter<std::string>(
      "output_avg_detection_topic", "/parking/docking/apriltag/detections");

    output_avg_pose_topic_ = declare_parameter<std::string>(
      "output_avg_pose_topic", "/parking/docking/apriltag/pose");

    output_detected_dock_pose_topic_ = declare_parameter<std::string>(
      "output_detected_dock_pose_topic", "/parking/docking/detected_dock_pose");

    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    tag_frame_ = declare_parameter<std::string>("tag_frame", "dock_tag");
    family_ = declare_parameter<std::string>("family", "36h11");
    target_tag_id_ = declare_parameter<int>("target_tag_id", 0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);

    detection_sub_ = create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      input_detection_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ParkingAprilTagBridge::detectionCallback, this, _1));

    avg_detection_pub_ =
      create_publisher<avg_msgs::msg::AvgAprilTagDetectionArray>(output_avg_detection_topic_, 10);

    avg_pose_pub_ =
      create_publisher<avg_msgs::msg::AvgAprilTagPose>(output_avg_pose_topic_, 10);

    dock_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(output_detected_dock_pose_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&ParkingAprilTagBridge::tfTimerCallback, this));
  }

private:
  void detectionCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    avg_msgs::msg::AvgAprilTagDetectionArray out;
    out.header = msg->header;

    for (const auto & det : msg->detections) {
      avg_msgs::msg::AvgAprilTagDetection item;

      item.family = det.family;
      item.id = det.id;
      item.hamming = det.hamming;
      item.goodness = det.goodness;
      item.decision_margin = det.decision_margin;

      item.centre.x = det.centre.x;
      item.centre.y = det.centre.y;

      for (size_t i = 0; i < det.corners.size(); ++i) {
        item.corners[i].x = det.corners[i].x;
        item.corners[i].y = det.corners[i].y;
      }

      item.homography = det.homography;

      out.detections.push_back(item);
    }

    avg_detection_pub_->publish(out);
  }

  void tfTimerCallback()
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(
        fixed_frame_, tag_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to lookup TF %s -> %s: %s",
        fixed_frame_.c_str(), tag_frame_.c_str(), ex.what());
      return;
    }

    avg_msgs::msg::AvgAprilTagPose avg_pose_msg;
    avg_pose_msg.header = tf_msg.header;
    avg_pose_msg.family = family_;
    avg_pose_msg.id = target_tag_id_;
    avg_pose_msg.tag_frame = tag_frame_;

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = tf_msg.header;
    pose_msg.pose.position.x = tf_msg.transform.translation.x;
    pose_msg.pose.position.y = tf_msg.transform.translation.y;
    pose_msg.pose.position.z = tf_msg.transform.translation.z;
    pose_msg.pose.orientation = tf_msg.transform.rotation;

    avg_pose_msg.pose = pose_msg;

    avg_pose_pub_->publish(avg_pose_msg);
    dock_pose_pub_->publish(pose_msg);
  }

  std::string input_detection_topic_;
  std::string output_avg_detection_topic_;
  std::string output_avg_pose_topic_;
  std::string output_detected_dock_pose_topic_;
  std::string fixed_frame_;
  std::string tag_frame_;
  std::string family_;
  int target_tag_id_;
  double publish_rate_hz_;

  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<avg_msgs::msg::AvgAprilTagDetectionArray>::SharedPtr avg_detection_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgAprilTagPose>::SharedPtr avg_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParkingAprilTagBridge>());
  rclcpp::shutdown();
  return 0;
}
