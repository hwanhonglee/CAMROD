#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <array>
#include <string>
#include <vector>

class PoseCovBridgeNode : public rclcpp::Node
{
public:
  PoseCovBridgeNode()
  // HH_260112 Use short node name; namespace applies the module prefix.
  : Node("pose_cov_bridge")
  {
    // HH_260109 GNSS topics use /sensing prefix.
    input_topic_ = declare_parameter<std::string>("input_topic", "/sensing/gnss/pose");
    output_topic_ = declare_parameter<std::string>("output_topic", "/sensing/gnss/pose_with_covariance");
    const auto cov = declare_parameter<std::vector<double>>(
      "position_covariance_diagonal", std::vector<double>{1.0, 1.0, 1.0, 999.0, 999.0, 1.0});

    // Fill 6x6 covariance with provided diagonal.
    covariance_.fill(0.0);
    for (size_t i = 0; i < std::min<size_t>(cov.size(), 6); ++i) {
      covariance_[i + i * 6] = cov[i];
    }

    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic_, rclcpp::QoS(10));
    subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PoseCovBridgeNode::onPose, this, std::placeholders::_1));

    // 2026-01-27 17:45: Demote startup log to DEBUG and remove HH tag from runtime output.
    RCLCPP_DEBUG(
      get_logger(), "pose_cov_bridge ready. in: %s out: %s",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void onPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header = msg->header;
    out.pose.pose = msg->pose;
    std::copy(covariance_.begin(), covariance_.end(), out.pose.covariance.begin());
    publisher_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::array<double, 36> covariance_{};

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseCovBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
