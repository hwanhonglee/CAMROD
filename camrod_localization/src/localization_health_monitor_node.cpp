#include <chrono>

#include <avg_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/msg/imu.hpp>
#include <avg_msgs/msg/bool.hpp>

#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/module_health.hpp>

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
    publish_localization_diagnostic_ = declare_parameter<bool>("publish_localization_diagnostic", false);
    localization_diagnostic_topic_ = declare_parameter<std::string>(
      "localization_diagnostic_topic", "/localization/diagnostic");

    // HH_251231 Publish health flag (true=healthy) and degraded flag (true=fall back)
    status_pub_ = create_publisher<avg_msgs::msg::Bool>(
      "/localization/health", rclcpp::QoS(1).transient_local());
    degraded_pub_ = create_publisher<avg_msgs::msg::Bool>(
      "/localization/health/degraded", rclcpp::QoS(1).transient_local());
    if (publish_localization_diagnostic_) {
      avg_localization_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_diagnostic_topic_, rclcpp::QoS(10));
    }

    // HH_260109 Monitor sensing-prefixed GNSS/IMU topics.
    gnss_sub_ = create_subscription<avg_msgs::msg::PoseStamped>(
      "/sensing/gnss/pose", rclcpp::SensorDataQoS(),
      std::bind(&LocalizationHealthMonitor::onGnss, this, std::placeholders::_1));
    imu_sub_ = create_subscription<avg_msgs::msg::Imu>(
      "/sensing/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&LocalizationHealthMonitor::onImu, this, std::placeholders::_1));

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(200ms, std::bind(&LocalizationHealthMonitor::onTimer, this));

    RCLCPP_INFO(get_logger(), "Localization health monitor started.");
  }

private:
  // Handles the `onGnss` callback.
  void onGnss(const avg_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    last_gnss_time_ = msg->header.stamp;
  }

  // Handles the `onImu` callback.
  void onImu(const avg_msgs::msg::Imu::ConstSharedPtr msg)
  {
    last_imu_time_ = msg->header.stamp;
  }

  // Handles the `onTimer` callback.
  void onTimer()
  {
    const rclcpp::Time now = this->now();
    const bool gnss_ok = (now - last_gnss_time_).seconds() <= gnss_timeout_sec_;
    const bool imu_ok = (now - last_imu_time_).seconds() <= imu_timeout_sec_;
    const bool healthy = gnss_ok && imu_ok;

    avg_msgs::msg::Bool msg;
    msg.data = healthy;
    status_pub_->publish(msg);
    avg_msgs::msg::Bool degraded;
    degraded.data = !msg.data;
    degraded_pub_->publish(degraded);

    if (publish_localization_diagnostic_ && avg_localization_pub_) {
      avg_msgs::msg::AvgLocalizationMsgs avg_msg;
      avg_msg.stamp = now;
      avg_msg.health.stamp = now;
      avg_msg.health.module_name = "localization";
      avg_msg.health.level = healthy ?
        avg_msgs::msg::ModuleHealth::OK :
        avg_msgs::msg::ModuleHealth::WARN;
      avg_msg.health.message = healthy ? "health_monitor_ok" : "health_monitor_degraded";
      if (!gnss_ok) {
        avg_msg.health.missing_topics.push_back("/sensing/gnss/pose");
      }
      if (!imu_ok) {
        avg_msg.health.missing_topics.push_back("/sensing/imu/data");
      }
      avg_localization_pub_->publish(avg_msg);
    }
  }

  double gnss_timeout_sec_{1.0};
  double imu_timeout_sec_{0.5};
  bool publish_localization_diagnostic_{false};
  std::string localization_diagnostic_topic_{"/localization/diagnostic"};
  rclcpp::Publisher<avg_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Publisher<avg_msgs::msg::Bool>::SharedPtr degraded_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr avg_localization_pub_;
  rclcpp::Subscription<avg_msgs::msg::PoseStamped>::SharedPtr gnss_sub_;
  rclcpp::Subscription<avg_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_gnss_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace camping_cart::localization

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camping_cart::localization::LocalizationHealthMonitor>());
  rclcpp::shutdown();
  return 0;
}
