// HH_260528: Renamed from parking_apriltag_bridge.cpp (camrod_parking) to
//            docking_apriltag_bridge.cpp (camrod_docking).
//            Class, node name, and default topic prefixes updated from
//            "parking" to "docking" throughout.
#include <cmath>
#include <memory>
#include <string>
#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp"
#include "avg_msgs/msg/avg_april_tag_detection.hpp"
#include "avg_msgs/msg/avg_april_tag_detection_array.hpp"
#include "avg_msgs/msg/avg_april_tag_pose.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;

class DockingAprilTagBridge : public rclcpp::Node
{
public:
  DockingAprilTagBridge()
  : Node("docking_apriltag_bridge"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_detection_topic_ = declare_parameter<std::string>(
      "input_detection_topic", "/docking/apriltag/detections_raw");
    output_avg_detection_topic_ = declare_parameter<std::string>(
      "output_avg_detection_topic", "/docking/apriltag/detections");
    output_avg_pose_topic_ = declare_parameter<std::string>(
      "output_avg_pose_topic", "/docking/apriltag/pose");
    output_detected_dock_pose_topic_ = declare_parameter<std::string>(
      "output_detected_dock_pose_topic", "/docking/detected_dock_pose");

    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    tag_frame_ = declare_parameter<std::string>("tag_frame", "dock_tag");
    family_ = declare_parameter<std::string>("family", "36h11");
    target_tag_id_ = declare_parameter<int>("target_tag_id", 0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    // EMA alpha: 0~1. 낮을수록 강한 필터(느린 반응). 기본값 0.4.
    ema_alpha_ = declare_parameter<double>("ema_alpha", 0.4);
    // 이 시간(초) 이내 검출이 없으면 dock_pose 발행 중단.
    detection_timeout_ = declare_parameter<double>("detection_timeout", 0.5);
    // 이전 EMA 위치 대비 이 거리(m) 이상 급변 시 outlier로 판단하여 무시.
    max_jump_m_ = declare_parameter<double>("max_jump_m", 0.5);
    // eCon 후면 카메라 센서 리드아웃이 yaw=π TF에도 불구하고 이미지 오른쪽=로봇 오른쪽으로
    // 동작하여 Y 부호가 반전됨. 후면 카메라 사용 시 true로 설정.
    negate_y_ = declare_parameter<bool>("negate_y", false);

    detection_sub_ = create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      input_detection_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DockingAprilTagBridge::detectionCallback, this, _1));

    avg_detection_pub_ =
      create_publisher<avg_msgs::msg::AvgAprilTagDetectionArray>(output_avg_detection_topic_, 10);
    avg_pose_pub_ =
      create_publisher<avg_msgs::msg::AvgAprilTagPose>(output_avg_pose_topic_, 10);
    dock_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(output_detected_dock_pose_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&DockingAprilTagBridge::tfTimerCallback, this));

    RCLCPP_INFO(get_logger(),
      "DockingAprilTagBridge: ema_alpha=%.2f detection_timeout=%.2fs tag_id=%d",
      ema_alpha_, detection_timeout_, target_tag_id_);
  }

private:
  void detectionCallback(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    avg_msgs::msg::AvgAprilTagDetectionArray out;
    out.header = msg->header;

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

      if (static_cast<int>(det.id) == target_tag_id_) {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        // msg->header.stamp = image capture time. Using this->now() would cause
        // TF lookup to mix current odom→base_link with stale camera→dock_tag,
        // making the dock_tag position in odom drift as the robot moves.
        last_detection_time_ = msg->header.stamp;
        detection_received_ = true;
      }
    }

    avg_detection_pub_->publish(out);
  }

  void tfTimerCallback()
  {
    {
      std::lock_guard<std::mutex> lock(detection_mutex_);
      if (!detection_received_) {
        return;
      }
      const double age = (this->now() - last_detection_time_).seconds();
      if (age > detection_timeout_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "tag %d detection stale (%.2fs > %.2fs), dock_pose publish suspended",
          target_tag_id_, age, detection_timeout_);
        return;
      }
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      // Use TimePointZero (latest available TF) because isaac_ros_apriltag publishes
      // tag36h11:3 TF at GPU processing completion time (~257ms after image capture),
      // not at image capture time. Requesting at image capture time causes
      // "extrapolation into the past" errors since the TF buffer has no data that old.
      tf_msg = tf_buffer_.lookupTransform(
        fixed_frame_, tag_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF lookup %s -> %s failed: %s",
        fixed_frame_.c_str(), tag_frame_.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = tf_msg.header;
    pose_msg.pose.position.x = tf_msg.transform.translation.x;
    pose_msg.pose.position.y = tf_msg.transform.translation.y;
    pose_msg.pose.position.z = tf_msg.transform.translation.z;
    pose_msg.pose.orientation = tf_msg.transform.rotation;

    // EMA: position만 필터링, orientation은 getRefinedPose()에서 approach_yaw로 대체됨.
    if (!ema_initialized_) {
      ema_pose_ = pose_msg;
      ema_initialized_ = true;
    } else {
      // Outlier rejection: 급격한 위치 점프 무시.
      const double jump = std::hypot(
        pose_msg.pose.position.x - ema_pose_.pose.position.x,
        pose_msg.pose.position.y - ema_pose_.pose.position.y);
      if (jump > max_jump_m_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "tag %d position jump %.3fm > %.3fm — skipped",
          target_tag_id_, jump, max_jump_m_);
        return;
      }
      const double a = ema_alpha_;
      ema_pose_.pose.position.x = a * pose_msg.pose.position.x + (1.0 - a) * ema_pose_.pose.position.x;
      ema_pose_.pose.position.y = a * pose_msg.pose.position.y + (1.0 - a) * ema_pose_.pose.position.y;
      ema_pose_.pose.position.z = a * pose_msg.pose.position.z + (1.0 - a) * ema_pose_.pose.position.z;
      ema_pose_.header = pose_msg.header;
      ema_pose_.pose.orientation = pose_msg.pose.orientation;
    }

    avg_msgs::msg::AvgAprilTagPose avg_pose_msg;
    avg_pose_msg.header = ema_pose_.header;
    avg_pose_msg.family = family_;
    avg_pose_msg.id = target_tag_id_;
    avg_pose_msg.tag_frame = tag_frame_;
    avg_pose_msg.pose = ema_pose_;

    auto out_pose = ema_pose_;
    if (negate_y_) {
      out_pose.pose.position.y = -out_pose.pose.position.y;
    }

    avg_pose_pub_->publish(avg_pose_msg);
    dock_pose_pub_->publish(out_pose);
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
  double ema_alpha_;
  double max_jump_m_;
  bool negate_y_{false};
  double detection_timeout_;

  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<avg_msgs::msg::AvgAprilTagDetectionArray>::SharedPtr avg_detection_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgAprilTagPose>::SharedPtr avg_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex detection_mutex_;
  rclcpp::Time last_detection_time_;
  bool detection_received_{false};

  geometry_msgs::msg::PoseStamped ema_pose_;
  bool ema_initialized_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockingAprilTagBridge>());
  rclcpp::shutdown();
  return 0;
}
