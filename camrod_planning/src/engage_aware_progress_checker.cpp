#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/progress_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/bool.hpp"

namespace camrod_planning
{

class EngageAwareProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override
  {
    plugin_name_ = plugin_name;
    node_ = parent.lock();
    if (!node_) {
      throw std::runtime_error("EngageAwareProgressChecker failed to lock lifecycle node");
    }

    clock_ = node_->get_clock();
    declare_if_missing(plugin_name_ + ".required_movement_radius", 0.03);
    declare_if_missing(plugin_name_ + ".movement_time_allowance", 120.0);
    declare_if_missing(plugin_name_ + ".engaged_topic", std::string("/planning/engaged"));
    declare_if_missing(plugin_name_ + ".default_engaged", false);

    node_->get_parameter(plugin_name_ + ".required_movement_radius", required_movement_radius_);
    double movement_time_allowance_s = 120.0;
    node_->get_parameter(plugin_name_ + ".movement_time_allowance", movement_time_allowance_s);
    movement_time_allowance_ = rclcpp::Duration::from_seconds(movement_time_allowance_s);
    node_->get_parameter(plugin_name_ + ".engaged_topic", engaged_topic_);
    node_->get_parameter(plugin_name_ + ".default_engaged", engaged_);

    // HH_260618 - Reset Nav2 progress timing while CAMROD drive engage is false.
    // This lets operators set/revise goals and inspect paths without causing a
    // FollowPath abort before they explicitly publish /planning/engage=true.
    engaged_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      engaged_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
        engaged_ = msg->data;
        if (!engaged_) {
          baseline_pose_set_ = false;
        }
      });

    dyn_params_handler_ = node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        return on_parameters(parameters);
      });
  }

  bool check(geometry_msgs::msg::PoseStamped & current_pose) override
  {
    const Pose2D current = to_pose2d(current_pose);
    if (!engaged_) {
      reset_baseline(current);
      return true;
    }

    if (!baseline_pose_set_ || distance(current, baseline_pose_) > required_movement_radius_) {
      reset_baseline(current);
      return true;
    }
    return !((clock_->now() - baseline_time_) > movement_time_allowance_);
  }

  void reset() override
  {
    baseline_pose_set_ = false;
  }

private:
  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
  };

  template<typename ParameterT>
  void declare_if_missing(const std::string & name, const ParameterT & value)
  {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, value);
    }
  }

  static Pose2D to_pose2d(const geometry_msgs::msg::PoseStamped & pose)
  {
    return Pose2D{pose.pose.position.x, pose.pose.position.y};
  }

  static double distance(const Pose2D & lhs, const Pose2D & rhs)
  {
    return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
  }

  void reset_baseline(const Pose2D & pose)
  {
    baseline_pose_ = pose;
    baseline_time_ = clock_->now();
    baseline_pose_set_ = true;
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == plugin_name_ + ".required_movement_radius" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        required_movement_radius_ = parameter.as_double();
      } else if (name == plugin_name_ + ".movement_time_allowance" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        movement_time_allowance_ = rclcpp::Duration::from_seconds(parameter.as_double());
      } else if (name == plugin_name_ + ".default_engaged" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        engaged_ = parameter.as_bool();
      }
    }
    return result;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engaged_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  std::string plugin_name_;
  std::string engaged_topic_{"/planning/engaged"};
  bool engaged_{false};
  double required_movement_radius_{0.03};
  rclcpp::Duration movement_time_allowance_{0, 0};
  Pose2D baseline_pose_;
  rclcpp::Time baseline_time_;
  bool baseline_pose_set_{false};
};

}  // namespace camrod_planning

PLUGINLIB_EXPORT_CLASS(camrod_planning::EngageAwareProgressChecker, nav2_core::ProgressChecker)
