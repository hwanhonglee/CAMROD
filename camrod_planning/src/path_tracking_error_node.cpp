#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <avg_msgs/msg/avg_tracking_error.hpp>
// HH_260720 - Use generated CAMROD pose/local-path contracts and an explicit Nav2 path boundary.
#include <avg_msgs/conversions.hpp>
#include <avg_msgs/msg/avg_path.hpp>
#include <avg_msgs/msg/avg_pose_stamped.hpp>
#include <avg_msgs/msg/avg_quaternion.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace camrod_planning
{

class PathTrackingErrorNode : public rclcpp::Node
{
public:
  // Initializes subscriptions/publishers for tracking-error estimation.
  PathTrackingErrorNode()
  : rclcpp::Node("path_tracking_error")
  {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/planning/lanelet_pose");
    local_path_topic_ = declare_parameter<std::string>("local_path_topic", "/planning/local_path");
    global_path_topic_ =
      declare_parameter<std::string>("global_path_topic", "/planning/global_path");
    // HH_260720 - Correct the public topic name so its purpose is immediately readable.
    output_topic_ = declare_parameter<std::string>("output_topic", "/planning/tracking_error");
    prefer_local_path_ = declare_parameter<bool>("prefer_local_path", true);
    pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 1.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 15.0);
    publish_on_input_update_ = declare_parameter<bool>("publish_on_input_update", true);

    sub_pose_ = create_subscription<avg_msgs::msg::AvgPoseStamped>(
      pose_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&PathTrackingErrorNode::onPose, this, std::placeholders::_1));
    sub_local_path_ = create_subscription<avg_msgs::msg::AvgPath>(
      local_path_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&PathTrackingErrorNode::onLocalPath, this, std::placeholders::_1));
    // HH_260720 - Convert the Nav2 global path once at its ROS boundary.
    sub_global_path_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic_, rclcpp::QoS(1).reliable(),
      [this](const nav_msgs::msg::Path::ConstSharedPtr msg) {
        onGlobalPath(
          std::make_shared<avg_msgs::msg::AvgPath>(
            avg_msgs::conversions::fromRos(*msg)));
      });

    pub_tracking_error_ = create_publisher<avg_msgs::msg::AvgTrackingError>(
      output_topic_, rclcpp::QoS(10));

    if (publish_rate_hz_ > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
      timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&PathTrackingErrorNode::onTimer, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "path_tracking_error: pose=%s local_path=%s global_path=%s output=%s prefer_local=%s",
      pose_topic_.c_str(), local_path_topic_.c_str(),
      global_path_topic_.c_str(), output_topic_.c_str(),
      prefer_local_path_ ? "true" : "false");
  }

private:
  struct ErrorSample
  {
    bool valid{false};
    double lateral{0.0};   // signed [m], +left / -right relative to path heading
    double heading{0.0};   // signed [rad], pose_yaw - path_tangent_yaw
    double distance{0.0};  // unsigned [m]
  };

  // Normalizes angle to [-pi, pi].
  static double normalizeAngle(const double angle)
  {
    double out = angle;
    while (out > M_PI) {
      out -= 2.0 * M_PI;
    }
    while (out < -M_PI) {
      out += 2.0 * M_PI;
    }
    return out;
  }

  // Extracts yaw from quaternion.
  static double yawFromQuaternion(const avg_msgs::msg::AvgQuaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  // Computes signed lateral and heading error against a single path.
  ErrorSample computeErrorForPath(
    const avg_msgs::msg::AvgPath & path,
    const avg_msgs::msg::AvgPoseStamped & pose) const
  {
    ErrorSample out;
    if (path.poses.empty()) {
      return out;
    }

    const double px = pose.pose.position.x;
    const double py = pose.pose.position.y;
    const double pose_yaw = yawFromQuaternion(pose.pose.orientation);

    double best_dist = std::numeric_limits<double>::infinity();
    double best_lateral = 0.0;
    double best_heading = 0.0;
    bool found_segment = false;

    if (path.poses.size() >= 2) {
      for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
        const auto & a = path.poses[i].pose.position;
        const auto & b = path.poses[i + 1].pose.position;

        const double vx = b.x - a.x;
        const double vy = b.y - a.y;
        const double seg_len2 = vx * vx + vy * vy;
        if (seg_len2 < 1e-9) {
          continue;
        }

        const double wx = px - a.x;
        const double wy = py - a.y;
        const double t = std::clamp((wx * vx + wy * vy) / seg_len2, 0.0, 1.0);
        const double proj_x = a.x + t * vx;
        const double proj_y = a.y + t * vy;
        const double dx = px - proj_x;
        const double dy = py - proj_y;
        const double dist = std::hypot(dx, dy);
        if (dist >= best_dist) {
          continue;
        }

        const double cross = vx * (py - a.y) - vy * (px - a.x);
        const double sign = (std::fabs(cross) < 1e-9) ? 0.0 : (cross > 0.0 ? 1.0 : -1.0);
        const double tangent_yaw = std::atan2(vy, vx);

        best_dist = dist;
        best_lateral = sign * dist;
        best_heading = normalizeAngle(pose_yaw - tangent_yaw);
        found_segment = true;
      }
    }

    // Fallback for one-point paths or degenerate duplicate-point paths.
    if (!found_segment) {
      size_t nearest_idx = 0;
      double nearest_d2 = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < path.poses.size(); ++i) {
        const auto & p = path.poses[i].pose.position;
        const double dx = px - p.x;
        const double dy = py - p.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < nearest_d2) {
          nearest_d2 = d2;
          nearest_idx = i;
        }
      }

      const auto & ref_pose = path.poses[nearest_idx].pose;
      const double ref_yaw = yawFromQuaternion(ref_pose.orientation);
      const double nx = -std::sin(ref_yaw);
      const double ny = std::cos(ref_yaw);
      const double rx = px - ref_pose.position.x;
      const double ry = py - ref_pose.position.y;
      best_dist = std::hypot(rx, ry);
      best_lateral = rx * nx + ry * ny;
      best_heading = normalizeAngle(pose_yaw - ref_yaw);
    }

    out.valid = std::isfinite(best_dist);
    out.lateral = best_lateral;
    out.heading = best_heading;
    out.distance = out.valid ? best_dist : 0.0;
    return out;
  }

  // Chooses which error stream should be treated as active.
  static void chooseActiveSample(
    const bool prefer_local,
    const ErrorSample & local_sample,
    const ErrorSample & global_sample,
    std::string & active_source,
    ErrorSample & active_sample)
  {
    if (prefer_local) {
      if (local_sample.valid) {
        active_source = "local";
        active_sample = local_sample;
        return;
      }
      if (global_sample.valid) {
        active_source = "global";
        active_sample = global_sample;
        return;
      }
    } else {
      if (global_sample.valid) {
        active_source = "global";
        active_sample = global_sample;
        return;
      }
      if (local_sample.valid) {
        active_source = "local";
        active_sample = local_sample;
        return;
      }
    }

    active_source = "none";
    active_sample = ErrorSample{};
  }

  // Handles pose update.
  void onPose(const avg_msgs::msg::AvgPoseStamped::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_pose_ = *msg;
    has_pose_ = true;
    last_pose_rx_ = now();
    if (publish_on_input_update_) {
      publishTrackingError();
    }
  }

  // Handles local-path update.
  void onLocalPath(const avg_msgs::msg::AvgPath::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_local_path_ = *msg;
    has_local_path_ = !msg->poses.empty();
    if (publish_on_input_update_) {
      publishTrackingError();
    }
  }

  // Handles global-path update.
  void onGlobalPath(const avg_msgs::msg::AvgPath::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_global_path_ = *msg;
    has_global_path_ = !msg->poses.empty();
    if (publish_on_input_update_) {
      publishTrackingError();
    }
  }

  // Timer callback for periodic publishing.
  void onTimer()
  {
    publishTrackingError();
  }

  // Publishes local/global path-tracking error in one message.
  void publishTrackingError()
  {
    if (!has_pose_) {
      return;
    }

    if (pose_timeout_s_ > 0.0) {
      const double age = (now() - last_pose_rx_).seconds();
      if (age > pose_timeout_s_) {
        return;
      }
    }

    const auto local_sample = has_local_path_ ?
      computeErrorForPath(latest_local_path_, latest_pose_) : ErrorSample{};
    const auto global_sample = has_global_path_ ?
      computeErrorForPath(latest_global_path_, latest_pose_) : ErrorSample{};

    std::string active_source;
    ErrorSample active_sample;
    chooseActiveSample(
      prefer_local_path_, local_sample, global_sample, active_source,
      active_sample);

    avg_msgs::msg::AvgTrackingError msg;
    const auto stamp_now = now();
    const auto ns = stamp_now.nanoseconds();
    msg.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
    msg.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    msg.frame_id = latest_pose_.header.frame_id;

    msg.local_valid = local_sample.valid;
    msg.local_lateral_deviation = local_sample.lateral;
    msg.local_heading_error = local_sample.heading;
    msg.local_distance_to_path = local_sample.distance;

    msg.global_valid = global_sample.valid;
    msg.global_lateral_deviation = global_sample.lateral;
    msg.global_heading_error = global_sample.heading;
    msg.global_distance_to_path = global_sample.distance;

    msg.active_path_source = active_source;
    msg.active_lateral_deviation = active_sample.lateral;
    msg.active_heading_error = active_sample.heading;
    msg.active_distance_to_path = active_sample.distance;

    pub_tracking_error_->publish(msg);
  }

  std::string pose_topic_;
  std::string local_path_topic_;
  std::string global_path_topic_;
  std::string output_topic_;

  bool prefer_local_path_{true};
  double pose_timeout_s_{1.0};
  double publish_rate_hz_{15.0};
  bool publish_on_input_update_{true};

  bool has_pose_{false};
  bool has_local_path_{false};
  bool has_global_path_{false};

  rclcpp::Time last_pose_rx_{0, 0, RCL_ROS_TIME};

  avg_msgs::msg::AvgPoseStamped latest_pose_;
  avg_msgs::msg::AvgPath latest_local_path_;
  avg_msgs::msg::AvgPath latest_global_path_;

  rclcpp::Subscription<avg_msgs::msg::AvgPoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<avg_msgs::msg::AvgPath>::SharedPtr sub_local_path_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_path_;
  rclcpp::Publisher<avg_msgs::msg::AvgTrackingError>::SharedPtr pub_tracking_error_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camrod_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod_planning::PathTrackingErrorNode>());
  rclcpp::shutdown();
  return 0;
}
