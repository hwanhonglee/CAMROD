#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <string>

class OdomYawCorrector : public rclcpp::Node
{
public:
  OdomYawCorrector()
  : Node("odom_yaw_corrector"), yaw_(0.0), initialized_(false)
  {
    // ── 모드 선택 ─────────────────────────────────────────────────────────────
    // use_firmware_yaw = true  : 펌웨어 euler_z 직접 사용 (정상 섀시)
    // use_firmware_yaw = false : angular.z 자체 적분 + 드리프트 보정 (rmp401 등 비정상 섀시)
    this->declare_parameter("use_firmware_yaw", false);
    use_firmware_yaw_ = this->get_parameter("use_firmware_yaw").as_bool();

    // ── 드리프트 보정 계수 (use_firmware_yaw=false 일 때만 사용) ───────────────
    // k = angular.z_측정 / linear.x  [rad/m]
    // 정상 섀시: 0.0 (보정 없음)
    this->declare_parameter("yaw_drift_coeff", 0.0);
    k_ = this->get_parameter("yaw_drift_coeff").as_double();

    // ── position.y 부호 보정 ────────────────────────────────────────────────────
    // rmp401 펌웨어 버그: 전진 기준 우측 이동 시 y 증가 (ROS 표준 좌측=+Y 와 반대)
    // 정상 섀시: false
    this->declare_parameter("invert_position_y", false);
    invert_y_ = this->get_parameter("invert_position_y").as_bool();

    // 플랫폼별 odom 토픽 설정
    // rmp401: "/rmp401/odom" / Ranger: "/odom"
    this->declare_parameter<std::string>("odom_input_topic", "/odom");
    std::string odom_topic = this->get_parameter("odom_input_topic").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      std::bind(&OdomYawCorrector::onOdom, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "odom_yaw_corrector: subscribing to '%s'", odom_topic.c_str());

    if (use_firmware_yaw_) {
      RCLCPP_INFO(get_logger(),
        "odom_yaw_corrector: MODE=firmware_yaw (정상 섀시 — euler_z 직접 사용)");
    } else {
      RCLCPP_INFO(get_logger(),
        "odom_yaw_corrector: MODE=drift_correction (yaw_drift_coeff=%.4f rad/m)", k_);
    }
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    rclcpp::Time now = this->now();

    geometry_msgs::msg::TransformStamped t;
    // msg->header.stamp은 드라이버 하드웨어 시간 기준으로 최대 66ms 지연됨.
    // controller가 clock_->now()로 TF를 조회하므로 this->now()를 사용해야
    // "extrapolation into the future" 오류를 방지할 수 있음.
    t.header.stamp        = now;
    t.header.frame_id     = msg->header.frame_id;
    t.child_frame_id      = "base_link";
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = invert_y_ ? -msg->pose.pose.position.y
                                           : msg->pose.pose.position.y;
    t.transform.translation.z = 0.0;

    if (use_firmware_yaw_) {
      // ── 정상 섀시 모드: 펌웨어 euler_z 직접 사용 ──────────────────────────
      // 펌웨어가 더 높은 주파수로 적분하므로 장기 운용 시 더 정확
      t.transform.rotation = msg->pose.pose.orientation;
    } else {
      // ── 드리프트 보정 모드: angular.z 자체 적분 ────────────────────────────
      if (!initialized_) {
        // 초기 yaw는 펌웨어 quaternion에서 추출 (정지 상태 기준)
        const auto & q = msg->pose.pose.orientation;
        yaw_ = 2.0 * std::atan2(q.z, q.w);  // roll/pitch=0 가정
        last_stamp_ = now;
        initialized_ = true;
        RCLCPP_INFO(get_logger(), "Initial yaw=%.4f rad", yaw_);
      } else {
        double dt = (now - last_stamp_).seconds();
        last_stamp_ = now;

        if (dt > 0.0 && dt < 0.5) {
          // 실시간 파라미터 업데이트 (ros2 param set 으로 현장 튜닝 가능)
          k_ = this->get_parameter("yaw_drift_coeff").as_double();

          double corrected = msg->twist.twist.angular.z - k_ * msg->twist.twist.linear.x;
          yaw_ += corrected * dt;
        }
      }

      tf2::Quaternion q_out;
      q_out.setRPY(0.0, 0.0, yaw_);
      t.transform.rotation.x = q_out.x();
      t.transform.rotation.y = q_out.y();
      t.transform.rotation.z = q_out.z();
      t.transform.rotation.w = q_out.w();
    }

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>           tf_broadcaster_;
  double          yaw_;
  double          k_;
  bool            use_firmware_yaw_;
  bool            invert_y_;
  rclcpp::Time    last_stamp_;
  bool            initialized_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomYawCorrector>());
  rclcpp::shutdown();
  return 0;
}
