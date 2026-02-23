#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>

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

    using std::placeholders::_1;
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&OdometryToPoseNode::onOdom, this, _1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, rclcpp::QoS(10));
    pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_cov_topic_, rclcpp::QoS(10));

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(get_logger(),
      "Odometry->Pose bridge ready. input=%s pose=%s pose_cov=%s",
      input_topic_.c_str(), pose_topic_.c_str(), pose_cov_topic_.c_str());
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    pose_pub_->publish(pose);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
    pose_cov.header = msg->header;
    pose_cov.pose = msg->pose;
    pose_cov_pub_->publish(pose_cov);
  }

  std::string input_topic_;
  std::string pose_topic_;
  std::string pose_cov_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryToPoseNode>());
  rclcpp::shutdown();
  return 0;
}
