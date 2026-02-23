#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>

namespace camping_cart::localization
{

class LocalizationHealthMonitor : public rclcpp::Node
{
public:
  // HH_260112 Use short node name; namespace applies the module prefix.
  LocalizationHealthMonitor() : Node("health_monitor")
  {
    gnss_timeout_sec_ = declare_parameter<double>("gnss_timeout_sec", 1.0);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.5);

    // HH_251231 Publish health flag (true=healthy) and degraded flag (true=fall back)
    status_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/localization/health", rclcpp::QoS(1).transient_local());
    degraded_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/localization/health/degraded", rclcpp::QoS(1).transient_local());

    // HH_260109 Monitor sensing-prefixed GNSS/IMU topics.
    gnss_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/sensing/gnss/pose", rclcpp::SensorDataQoS(),
      std::bind(&LocalizationHealthMonitor::onGnss, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/sensing/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&LocalizationHealthMonitor::onImu, this, std::placeholders::_1));

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(200ms, std::bind(&LocalizationHealthMonitor::onTimer, this));

    RCLCPP_INFO(get_logger(), "Localization health monitor started.");
  }

private:
  void onGnss(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    last_gnss_time_ = msg->header.stamp;
  }

  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    last_imu_time_ = msg->header.stamp;
  }

  void onTimer()
  {
    const rclcpp::Time now = this->now();
    const bool gnss_ok = (now - last_gnss_time_).seconds() <= gnss_timeout_sec_;
    const bool imu_ok = (now - last_imu_time_).seconds() <= imu_timeout_sec_;
    std_msgs::msg::Bool msg;
    msg.data = gnss_ok && imu_ok;
    status_pub_->publish(msg);
    std_msgs::msg::Bool degraded;
    degraded.data = !msg.data;
    degraded_pub_->publish(degraded);
  }

  double gnss_timeout_sec_{1.0};
  double imu_timeout_sec_{0.5};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr degraded_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_gnss_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace camping_cart::localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camping_cart::localization::LocalizationHealthMonitor>());
  rclcpp::shutdown();
  return 0;
}
