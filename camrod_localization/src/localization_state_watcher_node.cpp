#include <chrono>

#include <avg_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/msg/imu.hpp>
#include <avg_msgs/msg/bool.hpp>

#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/module_state.hpp>

namespace camping_cart::localization
{

class LocalizationStateWatcher : public rclcpp::Node
{
public:
  // HH_260112 Use short node name; namespace applies the module prefix.
  LocalizationStateWatcher() : Node("state_watcher")
  {
    gnss_timeout_sec_ = declare_parameter<double>("gnss_timeout_sec", 1.0);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.5);
    publish_localization_status_ = declare_parameter<bool>("publish_localization_status", false);
    localization_status_topic_ = declare_parameter<std::string>(
      "localization_status_topic", "/localization/status");

    // HH_251231 Publish state flag (true=statey) and degraded flag (true=fall back)
    status_pub_ = create_publisher<avg_msgs::msg::Bool>(
      "/localization/state", rclcpp::QoS(1).transient_local());
    degraded_pub_ = create_publisher<avg_msgs::msg::Bool>(
      "/localization/state/degraded", rclcpp::QoS(1).transient_local());
    if (publish_localization_status_) {
      avg_localization_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_status_topic_, rclcpp::QoS(10));
    }

    // HH_260109 Watcher sensing-prefixed GNSS/IMU topics.
    gnss_sub_ = create_subscription<avg_msgs::msg::PoseStamped>(
      "/sensing/gnss/pose", rclcpp::SensorDataQoS(),
      std::bind(&LocalizationStateWatcher::onGnss, this, std::placeholders::_1));
    imu_sub_ = create_subscription<avg_msgs::msg::Imu>(
      "/sensing/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&LocalizationStateWatcher::onImu, this, std::placeholders::_1));

    using namespace std::chrono_literals;
    timer_ = create_wall_timer(200ms, std::bind(&LocalizationStateWatcher::onTimer, this));

    RCLCPP_INFO(get_logger(), "Localization state watcher started.");
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
    const bool statey = gnss_ok && imu_ok;

    avg_msgs::msg::Bool msg;
    msg.data = statey;
    status_pub_->publish(msg);
    avg_msgs::msg::Bool degraded;
    degraded.data = !msg.data;
    degraded_pub_->publish(degraded);

    if (publish_localization_status_ && avg_localization_pub_) {
      avg_msgs::msg::AvgLocalizationMsgs avg_msg;
      avg_msg.stamp = now;
      avg_msg.state.stamp = now;
      avg_msg.state.module_name = "localization";
      avg_msg.state.level = statey ?
        avg_msgs::msg::ModuleState::OK :
        avg_msgs::msg::ModuleState::WARN;
      avg_msg.state.message = statey ? "state_watcher_ok" : "state_watcher_degraded";
      if (!gnss_ok) {
        avg_msg.state.missing_topics.push_back("/sensing/gnss/pose");
      }
      if (!imu_ok) {
        avg_msg.state.missing_topics.push_back("/sensing/imu/data");
      }
      avg_localization_pub_->publish(avg_msg);
    }
  }

  double gnss_timeout_sec_{1.0};
  double imu_timeout_sec_{0.5};
  bool publish_localization_status_{false};
  std::string localization_status_topic_{"/localization/status"};
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
  rclcpp::spin(std::make_shared<camping_cart::localization::LocalizationStateWatcher>());
  rclcpp::shutdown();
  return 0;
}
