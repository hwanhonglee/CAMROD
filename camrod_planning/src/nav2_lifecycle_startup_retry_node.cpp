#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <optional>
#include <string>
#include <utility>

// HH_260720 - Use generated localization contracts for internal Nav2 startup gating.
#include "avg_msgs/msg/avg_bool.hpp"
#include "avg_msgs/msg/avg_pose_with_covariance_stamped.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace camrod_planning
{

class Nav2LifecycleStartupRetryNode : public rclcpp::Node
{
public:
  Nav2LifecycleStartupRetryNode()
  : Node("nav2_lifecycle_startup_retry")
  {
    manager_service_ = declare_parameter<std::string>(
      "manager_service", "/planning/lifecycle_manager_planning/manage_nodes");
    is_active_service_ = declare_parameter<std::string>(
      "is_active_service", "/planning/lifecycle_manager_planning/is_active");
    retry_period_s_ = declare_parameter<double>("retry_period_s", 1.0);
    startup_cooldown_s_ = declare_parameter<double>("startup_cooldown_s", 2.0);

    require_localization_ready_ = declare_parameter<bool>("require_localization_ready", false);
    localization_ready_topic_ = declare_parameter<std::string>(
      "localization_ready_topic", "/localization/drop_zone/match_ok");
    localization_pose_cov_topic_ = declare_parameter<std::string>(
      "localization_pose_cov_topic", "/localization/pose_with_covariance");
    localization_pose_timeout_s_ = declare_parameter<double>("localization_pose_timeout_s", 1.0);
    max_position_variance_ = declare_parameter<double>("max_position_variance", 9.0);
    max_yaw_variance_ = declare_parameter<double>("max_yaw_variance", 1.0);
    require_covariance_sanity_ = declare_parameter<bool>("require_covariance_sanity", true);

    manager_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>(manager_service_);
    is_active_client_ = create_client<std_srvs::srv::Trigger>(is_active_service_);

    next_startup_time_ = now();
    last_wait_log_time_ = now();

    timer_ = create_wall_timer(
      std::chrono::duration<double>(retry_period_s_),
      std::bind(&Nav2LifecycleStartupRetryNode::on_timer, this));

    if (require_localization_ready_) {
      // HH_260720 - Subscribe directly to the generated drop-zone readiness contract.
      localization_ready_sub_ = create_subscription<avg_msgs::msg::AvgBool>(
        localization_ready_topic_, 10,
        std::bind(&Nav2LifecycleStartupRetryNode::on_localization_ready, this, std::placeholders::_1));
      // HH_260720 - Subscribe directly to the generated localization covariance contract.
      pose_cov_sub_ = create_subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>(
        localization_pose_cov_topic_, 10,
        std::bind(&Nav2LifecycleStartupRetryNode::on_localization_pose_cov, this, std::placeholders::_1));
    }

    RCLCPP_INFO(
      get_logger(),
      "nav2_lifecycle_startup_retry: is_active=%s manage=%s period=%.2fs cooldown=%.2fs "
      "require_localization_ready=%s",
      is_active_service_.c_str(), manager_service_.c_str(),
      retry_period_s_, startup_cooldown_s_,
      require_localization_ready_ ? "true" : "false");
  }

private:
  // HH_260527: Updates drop-zone match latch used by startup gating.
  // HH_260720 - Accept immutable generated messages to use the current rclcpp callback API.
  void on_localization_ready(const avg_msgs::msg::AvgBool::ConstSharedPtr msg)
  {
    localization_ready_ = static_cast<bool>(msg->data);
  }

  // Updates covariance sanity latch for localization readiness gating.
  void on_localization_pose_cov(
    const avg_msgs::msg::AvgPoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    pose_cov_received_ = true;
    last_pose_cov_rx_time_ = now();

    const auto & cov = msg->pose.covariance;
    const double pos_var = std::max(static_cast<double>(cov[0]), static_cast<double>(cov[7]));
    const double yaw_var = static_cast<double>(cov[35]);
    const bool finite = std::isfinite(pos_var) && std::isfinite(yaw_var);

    pose_cov_sane_ = finite &&
      pos_var >= 0.0 && yaw_var >= 0.0 &&
      pos_var <= max_position_variance_ &&
      yaw_var <= max_yaw_variance_;
  }

  // Evaluates localization readiness and returns (ready, reason).
  std::pair<bool, std::string> is_localization_ready(const rclcpp::Time & now_time) const
  {
    if (!require_localization_ready_) {
      return {true, "disabled"};
    }
    if (!localization_ready_) {
      return {false, "waiting_drop_zone_match_ok"};
    }
    if (!pose_cov_received_ || !last_pose_cov_rx_time_.has_value()) {
      return {false, "waiting_pose_with_covariance"};
    }

    const double age_s = (now_time - *last_pose_cov_rx_time_).seconds();
    if (age_s > localization_pose_timeout_s_) {
      return {false, "pose_with_covariance_stale(" + to_fixed(age_s, 2) + "s)"};
    }
    if (require_covariance_sanity_ && !pose_cov_sane_) {
      return {false, "pose_covariance_out_of_range"};
    }
    return {true, "ready"};
  }

  // Logs startup wait reason with throttling to avoid spam.
  void log_wait_reason(const std::string & reason, const rclcpp::Time & now_time)
  {
    const double elapsed = (now_time - last_wait_log_time_).seconds();
    if (reason != last_wait_reason_ || elapsed >= 2.0) {
      RCLCPP_INFO(get_logger(), "Nav2 STARTUP gated: %s", reason.c_str());
      last_wait_reason_ = reason;
      last_wait_log_time_ = now_time;
    }
  }

  // Periodic retry state machine for is_active + STARTUP calls.
  void on_timer()
  {
    if (done_) {
      return;
    }

    if (active_future_pending_ &&
      active_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      active_future_pending_ = false;
      std::shared_ptr<std_srvs::srv::Trigger::Response> response;
      try {
        response = active_future_.get();
      } catch (const std::exception & exc) {
        RCLCPP_WARN(get_logger(), "is_active call failed: %s", exc.what());
        return;
      }

      if (response->success) {
        done_ = true;
        RCLCPP_INFO(get_logger(), "Nav2 lifecycle is active. Startup retry node idling.");
        return;
      }

      const auto now_time = now();
      if (now_time >= next_startup_time_ && !startup_future_pending_) {
        const auto ready = is_localization_ready(now_time);
        if (!ready.first) {
          log_wait_reason(ready.second, now_time);
          return;
        }
        auto req = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        req->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
        // HH_260720 - Store the explicit shared future without deprecated implicit conversion.
        startup_future_ = manager_client_->async_send_request(req).future.share();
        startup_future_pending_ = true;
        next_startup_time_ =
          now_time + rclcpp::Duration::from_seconds(startup_cooldown_s_);
      }
      return;
    }

    if (startup_future_pending_ &&
      startup_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      startup_future_pending_ = false;
      std::shared_ptr<nav2_msgs::srv::ManageLifecycleNodes::Response> response;
      try {
        response = startup_future_.get();
      } catch (const std::exception & exc) {
        RCLCPP_WARN(get_logger(), "STARTUP call failed: %s", exc.what());
        return;
      }
      RCLCPP_INFO(
        get_logger(), "STARTUP request result: success=%s",
        response->success ? "true" : "false");
      return;
    }

    if (active_future_pending_ || startup_future_pending_) {
      return;
    }

    if (!is_active_client_->service_is_ready()) {
      return;
    }
    if (!manager_client_->service_is_ready()) {
      return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    // HH_260720 - Store the explicit shared future without deprecated implicit conversion.
    active_future_ = is_active_client_->async_send_request(req).future.share();
    active_future_pending_ = true;
  }

  // Formats a floating number with fixed precision.
  static std::string to_fixed(double value, int precision)
  {
    char buf[64];
    std::snprintf(buf, sizeof(buf), ("%." + std::to_string(precision) + "f").c_str(), value);
    return std::string(buf);
  }

private:
  std::string manager_service_;
  std::string is_active_service_;
  double retry_period_s_{1.0};
  double startup_cooldown_s_{2.0};

  bool require_localization_ready_{false};
  std::string localization_ready_topic_;
  std::string localization_pose_cov_topic_;
  double localization_pose_timeout_s_{1.0};
  double max_position_variance_{9.0};
  double max_yaw_variance_{1.0};
  bool require_covariance_sanity_{true};

  bool localization_ready_{false};
  bool pose_cov_received_{false};
  bool pose_cov_sane_{false};
  std::optional<rclcpp::Time> last_pose_cov_rx_time_;

  bool done_{false};
  std::string last_wait_reason_;
  rclcpp::Time last_wait_log_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_startup_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr manager_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr is_active_client_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture active_future_;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedFuture startup_future_;
  bool active_future_pending_{false};
  bool startup_future_pending_{false};

  rclcpp::Subscription<avg_msgs::msg::AvgBool>::SharedPtr localization_ready_sub_;
  rclcpp::Subscription<avg_msgs::msg::AvgPoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camrod_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camrod_planning::Nav2LifecycleStartupRetryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
