#include <string>

#include <avg_msgs/msg/twist_stamped.hpp>
#include <avg_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/module_state.hpp>

class WheelOdometryBridgeNode : public rclcpp::Node
{
public:
  // Implements `WheelOdometryBridgeNode` behavior.
  WheelOdometryBridgeNode()
  // HH_260123 Bridge wheel motion inputs into unified avg odometry.
  : Node("wheel_odometry_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/platform/status/wheel");
    output_topic_ = declare_parameter<std::string>("output_topic", "/platform/wheel/odometry");
    // HH_260326: Add nav_msgs output topic for EKF compatibility.
    nav_output_topic_ = declare_parameter<std::string>(
      "nav_output_topic", "/platform/wheel/nav_odometry");
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame_id", "robot_base_link");
    // HH_260326: Extend wheel bridge input selection to twist, avg_odom, and nav_odom.
    input_type_ = declare_parameter<std::string>("input_type", "twist");  // twist | avg_odom | nav_odom
    publish_localization_status_ = declare_parameter<bool>("publish_localization_status", false);
    localization_status_topic_ = declare_parameter<std::string>(
      "localization_status_topic", "/localization/status");

    // HH_260326: Keep legacy "odom" parameter value as avg_msgs/Odometry alias.
    if (input_type_ == "odom") {
      input_type_ = "avg_odom";
    }

    odom_pub_ = create_publisher<avg_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(50));
    nav_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(nav_output_topic_, rclcpp::QoS(50));
    if (publish_localization_status_) {
      avg_localization_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_status_topic_, rclcpp::QoS(10));
    }

    using std::placeholders::_1;
    if (input_type_ == "twist") {
      twist_sub_ = create_subscription<avg_msgs::msg::TwistStamped>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WheelOdometryBridgeNode::onTwistIn, this, _1));
      RCLCPP_INFO(
        get_logger(), "Wheel bridge listening for avg_msgs/TwistStamped on %s",
        input_topic_.c_str());
    } else if (input_type_ == "avg_odom") {
      avg_odom_sub_ = create_subscription<avg_msgs::msg::Odometry>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WheelOdometryBridgeNode::onAvgOdomIn, this, _1));
      RCLCPP_INFO(
        get_logger(), "Wheel bridge listening for avg_msgs/Odometry on %s",
        input_topic_.c_str());
    } else if (input_type_ == "nav_odom") {
      nav_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&WheelOdometryBridgeNode::onNavOdomIn, this, _1));
      RCLCPP_INFO(
        get_logger(), "Wheel bridge listening for nav_msgs/Odometry on %s",
        input_topic_.c_str());
    } else {
      RCLCPP_FATAL(
        get_logger(),
        "Unsupported input_type '%s'. Supported values: twist, avg_odom, nav_odom",
        input_type_.c_str());
      throw std::runtime_error("unsupported wheel_odometry_bridge input_type");
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
    publishUnified(odom);
  }

  // HH_260326: Forward avg_msgs/Odometry into unified wheel odometry output.
  void onAvgOdomIn(const avg_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    avg_msgs::msg::Odometry odom = *msg;
    applyDefaultFrames(odom);
    publishUnified(odom);
  }

  // HH_260326: Convert nav_msgs/Odometry into avg_msgs/Odometry for shared consumers.
  void onNavOdomIn(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    avg_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.child_frame_id = msg->child_frame_id;
    odom.pose = msg->pose;
    odom.twist = msg->twist;
    applyDefaultFrames(odom);
    publishUnified(odom);
  }

  void applyDefaultFrames(avg_msgs::msg::Odometry & odom)
  {
    // HH_260326: Preserve incoming frames when present, otherwise apply configured defaults.
    if (odom.header.frame_id.empty()) {
      odom.header.frame_id = odom_frame_;
    }
    if (odom.child_frame_id.empty()) {
      odom.child_frame_id = base_frame_;
    }
  }

  // Publishes `Avg` output.
  void publishAvg(const avg_msgs::msg::Odometry & odom)
  {
    if (!publish_localization_status_ || !avg_localization_pub_) {
      return;
    }
    avg_msgs::msg::AvgLocalizationMsgs avg_msg;
    avg_msg.stamp = odom.header.stamp;
    avg_msg.wheel_odometry = odom;
    avg_msg.state.stamp = odom.header.stamp;
    avg_msg.state.module_name = "localization";
    avg_msg.state.level = avg_msgs::msg::ModuleState::OK;
    avg_msg.state.message = "wheel_odometry_bridge";
    avg_localization_pub_->publish(avg_msg);
  }

  void publishUnified(const avg_msgs::msg::Odometry & odom)
  {
    odom_pub_->publish(odom);
    nav_odom_pub_->publish(toNavOdometry(odom));
    publishAvg(odom);
  }

  nav_msgs::msg::Odometry toNavOdometry(const avg_msgs::msg::Odometry & msg) const
  {
    nav_msgs::msg::Odometry out;
    out.header = msg.header;
    out.child_frame_id = msg.child_frame_id;
    out.pose = msg.pose;
    out.twist = msg.twist;
    return out;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string nav_output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string input_type_;
  bool publish_localization_status_{false};
  std::string localization_status_topic_{"/localization/status"};

  rclcpp::Publisher<avg_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr nav_odom_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;
  rclcpp::Subscription<avg_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<avg_msgs::msg::Odometry>::SharedPtr avg_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nav_odom_sub_;
};

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometryBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
