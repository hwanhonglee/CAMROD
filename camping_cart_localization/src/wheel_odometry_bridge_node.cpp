#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <string>

class WheelOdometryBridgeNode : public rclcpp::Node
{
public:
  WheelOdometryBridgeNode()
  // HH_260123 Bridge /platform/status/wheel -> /platform/wheel/odometry (twist → odom).
  : Node("wheel_odometry_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/platform/status/wheel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/platform/wheel/odometry");
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame_id", "robot_base_link");
    input_type_ = declare_parameter<std::string>("input_type", "twist");  // twist | odom

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(50));

    using std::placeholders::_1;
    if (input_type_ == "odom") {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WheelOdometryBridgeNode::onOdomIn, this, _1));
      // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
      RCLCPP_DEBUG(get_logger(), "Wheel bridge listening (odom) %s", input_topic_.c_str());
    } else {
      twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WheelOdometryBridgeNode::onTwistIn, this, _1));
      RCLCPP_DEBUG(get_logger(), "Wheel bridge listening (twist) %s", input_topic_.c_str());
    }
  }

private:
  void onTwistIn(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.twist.twist = msg->twist;
    // Pose unknown -> leave zeros, covariances minimal.
    for (double & c : odom.twist.covariance) c = 0.0;
    odom.twist.covariance[0] = 0.05;
    odom.twist.covariance[7] = 0.05;
    odom.twist.covariance[35] = 0.1;
    odom_pub_->publish(odom);
  }

  void onOdomIn(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    nav_msgs::msg::Odometry odom = *msg;
    if (odom.header.frame_id.empty()) {
      odom.header.frame_id = odom_frame_;
    }
    if (odom.child_frame_id.empty()) {
      odom.child_frame_id = base_frame_;
    }
    odom_pub_->publish(odom);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string input_type_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometryBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
