#include <algorithm>
#include <array>
#include <mutex>
#include <string>

#include <avg_msgs/conversions.hpp>
#include <avg_msgs/msg/avg_imu.hpp>
#include <avg_msgs/msg/avg_sensing_imu.hpp>
#include <avg_msgs/msg/avg_twist_stamped.hpp>
#include <avg_msgs/msg/avg_twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace camrod::sensing
{

class PlatformVelocityConverterNode : public rclcpp::Node
{
public:
  // HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.

  PlatformVelocityConverterNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : rclcpp::Node("platform_velocity_converter")
  {
    // HH_260109 Use platform velocity + IMU to publish twist_with_covariance.
    velocity_topic_ = declare_parameter<std::string>("velocity_topic", "/platform/status/velocity");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/sensing/imu/data_ros");
    imu_output_topic_ = declare_parameter<std::string>(
      "imu_output_topic", "/sensing/imu/data");
    output_topic_ = declare_parameter<std::string>(
      // HH_260331: Keep IMU-derived platform velocity topic under /sensing/imu/*.
      "output_topic", "/sensing/platform_velocity_converter/twist_with_covariance");
    imu_status_topic_ = declare_parameter<std::string>(
      "imu_status_topic", "/sensing/imu/status");
    publish_imu_status_ = declare_parameter<bool>("publish_imu_status", false);
    require_imu_ = declare_parameter<bool>("require_imu", true);
    linear_variance_ = declare_parameter<std::vector<double>>(
      "linear_variance", std::vector<double>{0.05, 0.05, 0.1});
    angular_variance_ = declare_parameter<std::vector<double>>(
      "angular_variance", std::vector<double>{0.2, 0.2, 0.2});

    using std::placeholders::_1;
    // HH_260720 - Consume the platform bridge's generated internal velocity contract.
    velocity_sub_ = create_subscription<avg_msgs::msg::AvgTwistStamped>(
      velocity_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PlatformVelocityConverterNode::onVelocity, this, _1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PlatformVelocityConverterNode::onImu, this, _1));
    output_pub_ = create_publisher<avg_msgs::msg::AvgTwistWithCovarianceStamped>(
      output_topic_, rclcpp::QoS(10));
    // HH_260720 - Publish the canonical generated IMU stream for localization.
    imu_data_pub_ = create_publisher<avg_msgs::msg::AvgImu>(imu_output_topic_, rclcpp::QoS(50));
    avg_imu_pub_ = create_publisher<avg_msgs::msg::AvgSensingImu>(imu_status_topic_, rclcpp::QoS(10));

    // 2026-01-27 17:45: Remove HH tags and keep startup logs quiet by default.
    RCLCPP_DEBUG(
      get_logger(),
      "Platform velocity converter ready. in=%s imu=%s out=%s",
      velocity_topic_.c_str(), imu_topic_.c_str(), output_topic_.c_str());
  }

private:
  // Handles the `onImu` callback.
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_imu_ = *msg;
    imu_ready_ = true;
    imu_data_pub_->publish(avg_msgs::conversions::fromRos(*msg));
  }

  // Handles the `onVelocity` callback.
  void onVelocity(const avg_msgs::msg::AvgTwistStamped::ConstSharedPtr msg)
  {
    if (require_imu_ && !imu_ready_) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU not ready; skipping twist_with_covariance.");
      return;
    }

    avg_msgs::msg::AvgTwistWithCovarianceStamped out;
    // HH_260720 - Generated Avg messages can be copied without a compatibility alias.
    out.header = msg->header;
    out.twist.twist = msg->twist;

    std::array<double, 36> cov{};
    cov.fill(0.0);

    const auto linear_var = padVariance(linear_variance_, 0.1);
    cov[0] = linear_var[0];
    cov[7] = linear_var[1];
    cov[14] = linear_var[2];

    if (imu_ready_) {
      std::lock_guard<std::mutex> lock(mutex_);
      // HH_260720 - Convert the driver vector into the generated AvgVector3 field.
      out.twist.twist.angular = avg_msgs::conversions::fromRos(last_imu_.angular_velocity);
      const auto imu_cov = last_imu_.angular_velocity_covariance;
      if (imu_cov[0] >= 0.0) {
        cov[21] = imu_cov[0];
        cov[28] = imu_cov[4];
        cov[35] = imu_cov[8];
      } else {
        const auto ang_var = padVariance(angular_variance_, 0.2);
        cov[21] = ang_var[0];
        cov[28] = ang_var[1];
        cov[35] = ang_var[2];
      }
    } else {
      const auto ang_var = padVariance(angular_variance_, 0.2);
      cov[21] = ang_var[0];
      cov[28] = ang_var[1];
      cov[35] = ang_var[2];
    }

    std::copy(cov.begin(), cov.end(), out.twist.covariance.begin());
    output_pub_->publish(out);
    if (publish_imu_status_ && avg_imu_pub_) {
      avg_msgs::msg::AvgSensingImu avg_msg;
      avg_msg.platform_twist_cov = out;
      if (imu_ready_) {
        std::lock_guard<std::mutex> lock(mutex_);
        avg_msg.imu_data = avg_msgs::conversions::fromRos(last_imu_);
      }
      avg_imu_pub_->publish(avg_msg);
    }
  }

  // Implements `padVariance` behavior.
  static std::array<double, 3> padVariance(const std::vector<double> & values, double fallback)
  {
    std::array<double, 3> out{fallback, fallback, fallback};
    for (size_t i = 0; i < std::min<size_t>(values.size(), 3); ++i) {
      out[i] = values[i];
    }
    return out;
  }

  std::string velocity_topic_;
  std::string imu_topic_;
  std::string output_topic_;
  std::string imu_output_topic_;
  std::string imu_status_topic_;
  bool require_imu_{true};
  bool publish_imu_status_{false};
  std::vector<double> linear_variance_;
  std::vector<double> angular_variance_;

  rclcpp::Subscription<avg_msgs::msg::AvgTwistStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<avg_msgs::msg::AvgTwistWithCovarianceStamped>::SharedPtr output_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgImu>::SharedPtr imu_data_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgSensingImu>::SharedPtr avg_imu_pub_;

  std::mutex mutex_;
  sensor_msgs::msg::Imu last_imu_;
  bool imu_ready_{false};
};

}  // namespace camrod::sensing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod::sensing::PlatformVelocityConverterNode>());
  rclcpp::shutdown();
  return 0;
}
