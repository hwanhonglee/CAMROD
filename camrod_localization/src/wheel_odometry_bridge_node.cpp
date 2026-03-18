#include <string>

#include <avg_msgs/msg/twist_stamped.hpp>
#include <avg_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/module_health.hpp>

class WheelOdometryBridgeNode : public rclcpp::Node
{
public:
  // Implements `WheelOdometryBridgeNode` behavior.
  WheelOdometryBridgeNode()
  // HH_260123 Bridge /platform/status/wheel -> /platform/wheel/odometry (twist → odom).
  : Node("wheel_odometry_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/platform/status/wheel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/platform/wheel/odometry");
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame_id", "robot_base_link");
    input_type_ = declare_parameter<std::string>("input_type", "twist");  // twist | odom
    publish_localization_diagnostic_ = declare_parameter<bool>("publish_localization_diagnostic", false);
    localization_diagnostic_topic_ = declare_parameter<std::string>(
      "localization_diagnostic_topic", "/localization/diagnostic");

    odom_pub_ = create_publisher<avg_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(50));
    if (publish_localization_diagnostic_) {
      avg_localization_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_diagnostic_topic_, rclcpp::QoS(10));
    }

    using std::placeholders::_1;
    if (input_type_ == "odom") {
      odom_sub_ = create_subscription<avg_msgs::msg::Odometry>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WheelOdometryBridgeNode::onOdomIn, this, _1));
      // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
      RCLCPP_DEBUG(get_logger(), "Wheel bridge listening (odom) %s", input_topic_.c_str());
    } else {
      twist_sub_ = create_subscription<avg_msgs::msg::TwistStamped>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WheelOdometryBridgeNode::onTwistIn, this, _1));
      RCLCPP_DEBUG(get_logger(), "Wheel bridge listening (twist) %s", input_topic_.c_str());
    }
  }

private:
  // Handles the `onTwistIn` callback.
  void onTwistIn(const avg_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    avg_msgs::msg::Odometry odom;
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
    publishAvg(odom);
  }

  // Handles the `onOdomIn` callback.
  void onOdomIn(const avg_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    avg_msgs::msg::Odometry odom = *msg;
    if (odom.header.frame_id.empty()) {
      odom.header.frame_id = odom_frame_;
    }
    if (odom.child_frame_id.empty()) {
      odom.child_frame_id = base_frame_;
    }
    odom_pub_->publish(odom);
    publishAvg(odom);
  }

  // Publishes `Avg` output.
  void publishAvg(const avg_msgs::msg::Odometry & odom)
  {
    if (!publish_localization_diagnostic_ || !avg_localization_pub_) {
      return;
    }
    avg_msgs::msg::AvgLocalizationMsgs avg_msg;
    avg_msg.stamp = odom.header.stamp;
    avg_msg.wheel_odometry = odom;
    avg_msg.health.stamp = odom.header.stamp;
    avg_msg.health.module_name = "localization";
    avg_msg.health.level = avg_msgs::msg::ModuleHealth::OK;
    avg_msg.health.message = "wheel_odometry_bridge";
    avg_localization_pub_->publish(avg_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string input_type_;
  bool publish_localization_diagnostic_{false};
  std::string localization_diagnostic_topic_{"/localization/diagnostic"};

  rclcpp::Publisher<avg_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;
  rclcpp::Subscription<avg_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<avg_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometryBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
