#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace camping_cart_planning
{

class GoalReplannerNode : public rclcpp::Node
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePathToPose = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

  GoalReplannerNode()
  : rclcpp::Node("goal_replanner")
  {
    enabled_ = declare_parameter<bool>("enabled", true);
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/planning/goal_pose");
    start_topic_ = declare_parameter<std::string>("start_topic", "/localization/pose");
    action_name_ = declare_parameter<std::string>("action_name", "/planning/compute_path_to_pose");
    planner_id_ = declare_parameter<std::string>("planner_id", "Smac2D");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    replan_rate_hz_ = declare_parameter<double>("replan_rate_hz", 2.0);
    request_timeout_sec_ = declare_parameter<double>("request_timeout_sec", 2.0);
    start_replan_distance_ = declare_parameter<double>("start_replan_distance", 0.4);
    goal_replan_distance_ = declare_parameter<double>("goal_replan_distance", 0.1);
    start_replan_yaw_deg_ = declare_parameter<double>("start_replan_yaw_deg", 10.0);
    replan_on_start_change_ = declare_parameter<bool>("replan_on_start_change", false);

    action_client_ = rclcpp_action::create_client<ComputePathToPose>(this, action_name_);

    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&GoalReplannerNode::onGoal, this, std::placeholders::_1));
    sub_start_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      start_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&GoalReplannerNode::onStart, this, std::placeholders::_1));

    if (replan_rate_hz_ > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / replan_rate_hz_);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&GoalReplannerNode::onTimer, this));
    } else {
      RCLCPP_WARN(get_logger(), "replan_rate_hz <= 0.0, auto replanning timer disabled");
    }

    RCLCPP_INFO(
      get_logger(),
      "Goal replanner: enabled=%s, goal=%s, start=%s, action=%s, planner_id=%s",
      enabled_ ? "true" : "false", goal_topic_.c_str(), start_topic_.c_str(),
      action_name_.c_str(), planner_id_.c_str());
  }

private:
  static double normalizeAngle(double a)
  {
    while (a > M_PI) {
      a -= 2.0 * M_PI;
    }
    while (a < -M_PI) {
      a += 2.0 * M_PI;
    }
    return a;
  }

  static double distance2D(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b)
  {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::hypot(dx, dy);
  }

  static double yawDiffRad(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b)
  {
    const auto & qa = a.pose.orientation;
    const auto & qb = b.pose.orientation;
    const double ya = std::atan2(
      2.0 * (qa.w * qa.z + qa.x * qa.y),
      1.0 - 2.0 * (qa.y * qa.y + qa.z * qa.z));
    const double yb = std::atan2(
      2.0 * (qb.w * qb.z + qb.x * qb.y),
      1.0 - 2.0 * (qb.y * qb.y + qb.z * qb.z));
    return std::abs(normalizeAngle(ya - yb));
  }

  bool shouldReplan() const
  {
    if (!has_goal_ || !has_start_) {
      return false;
    }
    if (!has_last_submitted_) {
      return true;
    }
    if (goal_msg_counter_ != last_submitted_goal_counter_) {
      return true;
    }

    if (distance2D(latest_goal_, last_submitted_goal_) > goal_replan_distance_) {
      return true;
    }
    if (replan_on_start_change_) {
      if (distance2D(latest_start_, last_submitted_start_) > start_replan_distance_) {
        return true;
      }
      const double yaw_thresh = start_replan_yaw_deg_ * M_PI / 180.0;
      if (yawDiffRad(latest_start_, last_submitted_start_) > yaw_thresh) {
        return true;
      }
    }
    return false;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_goal_ = *msg;
    if (latest_goal_.header.frame_id.empty()) {
      latest_goal_.header.frame_id = frame_id_;
    }
    has_goal_ = true;
    ++goal_msg_counter_;
  }

  void onStart(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_start_ = *msg;
    if (latest_start_.header.frame_id.empty()) {
      latest_start_.header.frame_id = frame_id_;
    }
    has_start_ = true;
  }

  void onTimer()
  {
    if (!enabled_) {
      return;
    }
    if (!has_goal_ || !has_start_) {
      return;
    }

    if (in_flight_) {
      if (shouldReplan()) {
        if (active_goal_handle_) {
          (void)action_client_->async_cancel_goal(active_goal_handle_);
        } else {
          (void)action_client_->async_cancel_all_goals();
        }
        in_flight_ = false;
        active_goal_handle_.reset();
        RCLCPP_INFO(get_logger(), "Canceled in-flight plan request due to updated goal/start");
      } else {
        const auto dt = (now() - request_sent_time_).seconds();
        if (dt <= request_timeout_sec_) {
          return;
        }

        if (active_goal_handle_) {
          (void)action_client_->async_cancel_goal(active_goal_handle_);
        } else {
          (void)action_client_->async_cancel_all_goals();
        }
        in_flight_ = false;
        active_goal_handle_.reset();
        RCLCPP_WARN(get_logger(), "Replan request timed out (%.2fs), canceled pending goal", dt);
      }
    }

    if (!shouldReplan()) {
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(0))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for action server %s", action_name_.c_str());
      return;
    }

    ComputePathToPose::Goal goal_msg;
    goal_msg.use_start = true;
    goal_msg.start = latest_start_;
    goal_msg.goal = latest_goal_;
    goal_msg.planner_id = planner_id_;
    goal_msg.start.header.frame_id = goal_msg.start.header.frame_id.empty()
      ? frame_id_ : goal_msg.start.header.frame_id;
    goal_msg.goal.header.frame_id = goal_msg.goal.header.frame_id.empty()
      ? frame_id_ : goal_msg.goal.header.frame_id;

    auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](GoalHandleComputePathToPose::SharedPtr goal_handle) {
        if (!goal_handle) {
          in_flight_ = false;
          active_goal_handle_.reset();
          RCLCPP_WARN(get_logger(), "ComputePathToPose goal rejected");
          return;
        }
        active_goal_handle_ = goal_handle;
      };
    send_goal_options.result_callback =
      [this](const GoalHandleComputePathToPose::WrappedResult & result) {
        in_flight_ = false;
        active_goal_handle_.reset();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          return;
        }
        RCLCPP_WARN(
          get_logger(), "ComputePathToPose finished with code %d",
          static_cast<int>(result.code));
      };

    request_sent_time_ = now();
    in_flight_ = true;
    last_submitted_start_ = latest_start_;
    last_submitted_goal_ = latest_goal_;
    has_last_submitted_ = true;
    last_submitted_goal_counter_ = goal_msg_counter_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Replan request: start(%.2f, %.2f) -> goal(%.2f, %.2f), planner=%s",
      goal_msg.start.pose.position.x, goal_msg.start.pose.position.y,
      goal_msg.goal.pose.position.x, goal_msg.goal.pose.position.y,
      goal_msg.planner_id.c_str());
    (void)action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  bool enabled_{true};
  bool has_goal_{false};
  bool has_start_{false};
  bool has_last_submitted_{false};
  bool in_flight_{false};
  uint64_t goal_msg_counter_{0};
  uint64_t last_submitted_goal_counter_{0};

  std::string goal_topic_;
  std::string start_topic_;
  std::string action_name_;
  std::string planner_id_;
  std::string frame_id_;

  double replan_rate_hz_{2.0};
  double request_timeout_sec_{2.0};
  double start_replan_distance_{0.4};
  double goal_replan_distance_{0.1};
  double start_replan_yaw_deg_{10.0};
  bool replan_on_start_change_{false};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_start_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_client_;
  GoalHandleComputePathToPose::SharedPtr active_goal_handle_;

  rclcpp::Time request_sent_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped latest_goal_;
  geometry_msgs::msg::PoseStamped latest_start_;
  geometry_msgs::msg::PoseStamped last_submitted_goal_;
  geometry_msgs::msg::PoseStamped last_submitted_start_;
};

}  // namespace camping_cart_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camping_cart_planning::GoalReplannerNode>());
  rclcpp::shutdown();
  return 0;
}
