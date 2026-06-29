// Nova Carter 패턴 적용: dock_pose_publisher 스타일
// TF 변환, EMA 필터, outlier rejection, 타이머 제거.
// AprilTag 검출 → target ID 선택 → camera frame PoseStamped 즉시 발행.
// SimpleChargingDock (use_external_detection_pose=true) 이 TF 변환 + EMA 를 내부 처리.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "avg_msgs/conversions.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp"
#include "avg_msgs/msg/avg_april_tag_detection.hpp"
#include "avg_msgs/msg/avg_april_tag_detection_array.hpp"

using std::placeholders::_1;

class DockingAprilTagBridge : public rclcpp::Node
{
public:
  DockingAprilTagBridge()
  : Node("docking_apriltag_bridge")
  {
    input_detection_topic_ = declare_parameter<std::string>(
      "input_detection_topic", "/docking/apriltag/detections_raw");
    output_avg_detection_topic_ = declare_parameter<std::string>(
      "output_avg_detection_topic", "/docking/apriltag/detections");
    output_detected_dock_pose_topic_ = declare_parameter<std::string>(
      "output_detected_dock_pose_topic", "/docking/detected_dock_pose");
    target_tag_id_ = declare_parameter<int>("target_tag_id", 0);

    detection_sub_ = create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      input_detection_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DockingAprilTagBridge::detectionCallback, this, _1));

    avg_detection_pub_ =
      create_publisher<avg_msgs::msg::AvgAprilTagDetectionArray>(output_avg_detection_topic_, 10);
    dock_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(output_detected_dock_pose_topic_, 10);

    RCLCPP_INFO(get_logger(),
      "DockingAprilTagBridge: target_tag_id=%d input=%s output=%s",
      target_tag_id_, input_detection_topic_.c_str(), output_detected_dock_pose_topic_.c_str());
  }

private:
  void detectionCallback(
    const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    // avg_detection 발행 (진단용)
    avg_msgs::msg::AvgAprilTagDetectionArray out;
    out.header = avg_msgs::conversions::fromRos(msg->header);
    for (const auto & det : msg->detections) {
      avg_msgs::msg::AvgAprilTagDetection item;
      item.family = det.family;
      item.id = det.id;
      item.hamming = 0;
      item.goodness = 0.0f;
      item.decision_margin = 0.0f;
      item.centre.x = det.center.x;
      item.centre.y = det.center.y;
      for (size_t i = 0; i < det.corners.size(); ++i) {
        item.corners[i].x = det.corners[i].x;
        item.corners[i].y = det.corners[i].y;
      }
      item.homography = {};
      out.detections.push_back(item);
    }
    avg_detection_pub_->publish(out);

    // target tag 검출 시 camera frame pose 즉시 발행
    // SimpleChargingDock 이 external_detection_rotation 파라미터로 optical→nav 변환 수행
    for (const auto & det : msg->detections) {
      if (static_cast<int>(det.id) == target_tag_id_) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header;  // camera optical frame
        pose_msg.pose = det.pose.pose.pose;
        dock_pose_pub_->publish(pose_msg);
        return;
      }
    }
  }

  std::string input_detection_topic_;
  std::string output_avg_detection_topic_;
  std::string output_detected_dock_pose_topic_;
  int target_tag_id_;

  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr
    detection_sub_;
  rclcpp::Publisher<avg_msgs::msg::AvgAprilTagDetectionArray>::SharedPtr avg_detection_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockingAprilTagBridge>());
  rclcpp::shutdown();
  return 0;
}
