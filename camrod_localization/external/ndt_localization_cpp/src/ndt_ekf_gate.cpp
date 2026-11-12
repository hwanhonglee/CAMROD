// NDT pose -> EKF input, only while the NDT match is trustworthy.
//   NDT pose and its NVTL (nearest voxel transformation likelihood) carry the same stamp; they are paired by stamp and
//   the pose is republished with an EKF covariance when NVTL >= min_nvtl. Below that nothing is sent, so the EKF falls
//   back to GNSS + wheel/IMU and a diverged NDT cannot pull the EKF (whose output is the NDT's next initial guess).
//   covariance: x/y std xy_std [m], yaw std yaw_std_deg, scaled up linearly from good_nvtl (x1) to min_nvtl (x max_scale)
//   in  ~/ndt_pose (PoseWithCovarianceStamped)   ~/nvtl (autoware_internal_debug_msgs/Float32Stamped)
//   out ~/output   (PoseWithCovarianceStamped, same frame/stamp as the NDT pose)
#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>

namespace ndt_localization_cpp
{

class NdtEkfGate : public rclcpp::Node
{
  using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Nvtl = autoware_internal_debug_msgs::msg::Float32Stamped;

public:
  explicit NdtEkfGate(const rclcpp::NodeOptions & options)
  : Node("ndt_ekf_gate", options)
  {
    min_nvtl_ = declare_parameter<double>("min_nvtl", 1.2);
    good_nvtl_ = declare_parameter<double>("good_nvtl", 1.35);
    xy_std_ = declare_parameter<double>("xy_std", 0.10);
    yaw_std_ = declare_parameter<double>("yaw_std_deg", 2.0) * M_PI / 180.0;
    max_scale_ = declare_parameter<double>("max_scale", 4.0);
    pub_ = create_publisher<Pose>("~/output", 10);
    sub_pose_ = create_subscription<Pose>("~/ndt_pose", 10, [this](Pose::ConstSharedPtr m) {
      poses_.push_back(*m);
      if (poses_.size() > 10) {
        poses_.pop_front();
      }
      match();
    });
    sub_nvtl_ = create_subscription<Nvtl>("~/nvtl", 10, [this](Nvtl::ConstSharedPtr m) {
      nvtls_.push_back(*m);
      if (nvtls_.size() > 10) {
        nvtls_.pop_front();
      }
      match();
    });
    timer_ = create_wall_timer(std::chrono::seconds(10), [this]() {
      if (n_pass_ + n_block_) {
        RCLCPP_INFO(get_logger(), "last 10 s: %ld NDT poses to EKF, %ld blocked (NVTL < %.2f), NVTL min %.2f",
                    n_pass_, n_block_, min_nvtl_, nvtl_min_);
      }
      n_pass_ = n_block_ = 0;
      nvtl_min_ = 1e9;
    });
    RCLCPP_INFO(get_logger(), "NDT -> EKF when NVTL >= %.2f, xy std %.2f m, yaw std %.1f deg", min_nvtl_, xy_std_,
                yaw_std_ * 180.0 / M_PI);
  }

private:
  static bool same(const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b)
  {
    return a.sec == b.sec && a.nanosec == b.nanosec;
  }

  // pair every pose with the NVTL of the same stamp (either may arrive first)
  void match()
  {
    for (auto p = poses_.begin(); p != poses_.end();) {
      auto n = std::find_if(nvtls_.begin(), nvtls_.end(), [&](const Nvtl & v) { return same(v.stamp, p->header.stamp); });
      if (n == nvtls_.end()) {
        ++p;
        continue;
      }
      emit(*p, n->data);
      nvtls_.erase(n);
      p = poses_.erase(p);
    }
  }

  void emit(const Pose & pose, double nvtl)
  {
    nvtl_min_ = std::min(nvtl_min_, nvtl);
    if (!(nvtl >= min_nvtl_)) {
      ++n_block_;
      return;
    }
    const double w = good_nvtl_ > min_nvtl_ ? std::clamp((good_nvtl_ - nvtl) / (good_nvtl_ - min_nvtl_), 0.0, 1.0) : 0.0;
    const double s = 1.0 + w * (max_scale_ - 1.0);
    auto o = std::make_unique<Pose>(pose);
    o->pose.covariance.fill(0.0);
    o->pose.covariance[0] = o->pose.covariance[7] = std::pow(xy_std_ * s, 2);
    o->pose.covariance[14] = 1.0;                                   // z, roll, pitch: unused (2D EKF)
    o->pose.covariance[21] = o->pose.covariance[28] = 1.0;
    o->pose.covariance[35] = std::pow(yaw_std_ * s, 2);
    pub_->publish(std::move(o));
    ++n_pass_;
  }

  double min_nvtl_, good_nvtl_, xy_std_, yaw_std_, max_scale_;
  double nvtl_min_{1e9};
  long n_pass_{0}, n_block_{0};
  std::deque<Pose> poses_;
  std::deque<Nvtl> nvtls_;
  rclcpp::Publisher<Pose>::SharedPtr pub_;
  rclcpp::Subscription<Pose>::SharedPtr sub_pose_;
  rclcpp::Subscription<Nvtl>::SharedPtr sub_nvtl_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ndt_localization_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ndt_localization_cpp::NdtEkfGate)
