#include <string>

#include <avg_msgs/msg/pose_stamped.hpp>
#include <avg_msgs/msg/pose_with_covariance_stamped.hpp>
#include <avg_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/module_health.hpp>

// HH_260109 Convert filtered odometry into Pose/PoseWithCovariance for consumers expecting pose topics.
class OdometryToPoseNode : public rclcpp::Node
{
public:
  OdometryToPoseNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : Node("odometry_to_pose")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/localization/odometry/filtered");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/localization/pose");
    pose_cov_topic_ = declare_parameter<std::string>("pose_cov_topic", "/localization/pose_with_covariance");
    publish_localization_diagnostic_ = declare_parameter<bool>("publish_localization_diagnostic", false);
    localization_diagnostic_topic_ = declare_parameter<std::string>(
      "localization_diagnostic_topic", "/localization/diagnostic");

    using std::placeholders::_1;
    sub_ = create_subscription<avg_msgs::msg::Odometry>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OdometryToPoseNode::onOdom, this, _1));
    pose_pub_ = create_publisher<avg_msgs::msg::PoseStamped>(pose_topic_, rclcpp::QoS(10));
    pose_cov_pub_ = create_publisher<avg_msgs::msg::PoseWithCovarianceStamped>(
      pose_cov_topic_, rclcpp::QoS(10));
    if (publish_localization_diagnostic_) {
      avg_localization_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_diagnostic_topic_, rclcpp::QoS(10));
    }

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(get_logger(),
      "Odometry->Pose bridge ready. input=%s pose=%s pose_cov=%s",
      input_topic_.c_str(), pose_topic_.c_str(), pose_cov_topic_.c_str());
  }

private:
  // Handles the `onOdom` callback.
  void onOdom(const avg_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    avg_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    pose_pub_->publish(pose);

    avg_msgs::msg::PoseWithCovarianceStamped pose_cov;
    pose_cov.header = msg->header;
    pose_cov.pose = msg->pose;
    pose_cov_pub_->publish(pose_cov);

    if (publish_localization_diagnostic_ && avg_localization_pub_) {
      avg_msgs::msg::AvgLocalizationMsgs avg_msg;
      avg_msg.stamp = msg->header.stamp;
      avg_msg.localization_odom = *msg;
      avg_msg.localization_pose = pose;
      avg_msg.localization_pose_cov = pose_cov;
      avg_msg.health.stamp = msg->header.stamp;
      avg_msg.health.module_name = "localization";
      avg_msg.health.level = avg_msgs::msg::ModuleHealth::OK;
      avg_msg.health.message = "odometry_to_pose";
      avg_localization_pub_->publish(avg_msg);
    }
  }

  std::string input_topic_;
  std::string pose_topic_;
  std::string pose_cov_topic_;
  std::string localization_diagnostic_topic_;
  bool publish_localization_diagnostic_{false};
  rclcpp::Subscription<avg_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<avg_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<avg_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;
};

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryToPoseNode>());
  rclcpp::shutdown();
  return 0;
}
