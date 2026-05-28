#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomYawCorrector : public rclcpp::Node
{
public:
  OdomYawCorrector()
  : Node("odom_yaw_corrector")
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/rmp401/odom", 10,
      std::bind(&OdomYawCorrector::onOdom, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "odom_yaw_corrector started: /rmp401/odom → TF odom→base_link (qz negated, corrects segwayrmp sign)");
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header = msg->header;
    t.child_frame_id = "base_link";
    t.transform.translation.x =  msg->pose.pose.position.x;
    // segwayrmp integrates position with wrong heading sign (θ_driver = -θ_actual).
    // sin is odd: sin(-θ) = -sin(θ), so y_driver = -y_actual. Negate to correct.
    // x is unaffected: cos(-θ) = cos(θ).
    t.transform.translation.y = -msg->pose.pose.position.y;
    t.transform.translation.z =  msg->pose.pose.position.z;
    t.transform.rotation.x =  msg->pose.pose.orientation.x;
    t.transform.rotation.y =  msg->pose.pose.orientation.y;
    t.transform.rotation.z = -msg->pose.pose.orientation.z;  // segwayrmp qz sign correction
    t.transform.rotation.w =  msg->pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomYawCorrector>());
  rclcpp::shutdown();
  return 0;
}
