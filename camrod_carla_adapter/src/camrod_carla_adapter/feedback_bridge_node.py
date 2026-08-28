"""Bridge CARLA truth/sensor feedback into CAMROD external-simulator inputs."""

import copy
import math
import time

from camrod_carla_adapter.feedback_mapping import (
    diagonal_covariance,
    multiply_quaternions,
    PlanarTransform,
    quaternion_from_yaw,
    transform_pose,
    validate_motion_values,
)
from camrod_carla_adapter.gnss_compat import (
    build_navcov,
    build_navpvt,
    build_relposned,
    quaternion_yaw_rad,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from ublox_msgs.msg import NavCOV, NavPVT, NavRELPOSNED9


class CarlaFeedbackBridgeNode(Node):
    """Expose CARLA feedback without starting CAMROD's fake integrator."""

    def __init__(self, **node_kwargs):
        super().__init__("carla_feedback_bridge", **node_kwargs)

        self.input_odometry_topic = self.declare_parameter(
            "input_odometry_topic", "/carla/ego_vehicle/odometry").value
        self.platform_odometry_topic = self.declare_parameter(
            "platform_odometry_topic", "/platform/status/odometry").value
        self.localization_odometry_topic = self.declare_parameter(
            "localization_odometry_topic", "/localization/odometry_ros").value
        self.localization_pose_topic = self.declare_parameter(
            "localization_pose_topic", "/localization/pose_ros").value
        self.metric_pose_topic = self.declare_parameter(
            "metric_pose_topic", "/camrod_carla/metric_pose").value
        self.input_imu_topic = self.declare_parameter(
            "input_imu_topic", "/carla/ego_vehicle/imu").value
        self.output_imu_topic = self.declare_parameter(
            "output_imu_topic", "/sensing/imu/data_ros").value
        self.input_gnss_topic = self.declare_parameter(
            "input_gnss_topic", "/carla/ego_vehicle/gnss").value
        self.input_gnss_right_topic = self.declare_parameter(
            "input_gnss_right_topic", "/carla/ego_vehicle/gnss_right"
        ).value
        self.output_gnss_topic = self.declare_parameter(
            "output_gnss_topic", "/sensing/gnss/ublox_gps_node/fix").value
        self.output_gnss_navpvt_topic = self.declare_parameter(
            "output_gnss_navpvt_topic",
            "/sensing/gnss/ublox_gps_node/navpvt",
        ).value
        self.output_gnss_navcov_topic = self.declare_parameter(
            "output_gnss_navcov_topic", "/sensing/gnss/navcov"
        ).value
        self.output_gnss_relpos_topic = self.declare_parameter(
            "output_gnss_relpos_topic", "/sensing/gnss/navrelposned"
        ).value
        self.output_gnss_heading_topic = self.declare_parameter(
            "output_gnss_heading_topic", "/sensing/gnss/navheading"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/camrod_carla/feedback_bridge/status").value

        self.map_frame_id = self.declare_parameter(
            "map_frame_id", "map").value
        self.odom_frame_id = self.declare_parameter(
            "odom_frame_id", "odom").value
        self.base_frame_id = self.declare_parameter(
            "base_frame_id", "robot_center_link").value
        self.imu_frame_id = self.declare_parameter(
            "imu_frame_id", "robot_center_link").value
        self.gnss_frame_id = self.declare_parameter(
            "gnss_frame_id", "robot_center_link").value
        self.gnss_heading_frame_id = self.declare_parameter(
            "gnss_heading_frame_id", "gnss_link"
        ).value

        self.publish_platform_odometry = bool(self.declare_parameter(
            "publish_platform_odometry", True).value)
        self.publish_ground_truth_localization = bool(self.declare_parameter(
            "publish_ground_truth_localization", True).value)
        self.publish_metric_pose = bool(self.declare_parameter(
            "publish_metric_pose", True).value)
        self.publish_ground_truth_tf = bool(self.declare_parameter(
            "publish_ground_truth_tf", True).value)
        self.relay_imu = bool(self.declare_parameter(
            "relay_imu", True).value)
        self.relay_gnss = bool(self.declare_parameter(
            "relay_gnss", True).value)
        self.publish_gnss_ui_compat = bool(self.declare_parameter(
            "publish_gnss_ui_compat", True
        ).value)
        self.gnss_pair_timeout_sec = float(self.declare_parameter(
            "gnss_pair_timeout_sec", 0.5
        ).value)
        self.gnss_pair_position_std_m = float(self.declare_parameter(
            "gnss_pair_position_std_m", 0.1
        ).value)
        self.gnss_moving_speed_threshold_mps = float(self.declare_parameter(
            "gnss_moving_speed_threshold_mps", 0.02
        ).value)
        self.publish_gnss_heading_from_imu = bool(self.declare_parameter(
            "publish_gnss_heading_from_imu", False
        ).value)
        self.gnss_heading_yaw_bias_rad = float(self.declare_parameter(
            "gnss_heading_yaw_bias_rad", math.pi / 2.0
        ).value)
        self.stamp_with_reception_time = bool(self.declare_parameter(
            "stamp_with_reception_time", True).value)
        self.feedback_timeout_sec = float(self.declare_parameter(
            "feedback_timeout_sec", 0.5).value)

        self.odometry_pose_covariance = diagonal_covariance(
            self.declare_parameter(
                "odometry_pose_covariance_diagonal",
                [0.0025, 0.0025, 0.01, 0.0025, 0.0025, 0.0025],
            ).value
        )
        self.odometry_twist_covariance = diagonal_covariance(
            self.declare_parameter(
                "odometry_twist_covariance_diagonal",
                [0.01, 0.01, 0.04, 0.0025, 0.0025, 0.0025],
            ).value
        )
        self.imu_orientation_covariance = diagonal_covariance(
            self.declare_parameter(
                "imu_orientation_covariance_diagonal",
                [0.0025, 0.0025, 0.0025],
            ).value
        )
        self.imu_angular_velocity_covariance = diagonal_covariance(
            self.declare_parameter(
                "imu_angular_velocity_covariance_diagonal",
                [0.0004, 0.0004, 0.0004],
            ).value
        )
        self.imu_linear_acceleration_covariance = diagonal_covariance(
            self.declare_parameter(
                "imu_linear_acceleration_covariance_diagonal",
                [0.01, 0.01, 0.01],
            ).value
        )
        self.gnss_position_covariance = diagonal_covariance(
            self.declare_parameter(
                "gnss_position_covariance_diagonal",
                [0.01, 0.01, 0.04],
            ).value
        )

        self.map_transform = PlanarTransform(
            x_m=float(self.declare_parameter("map_offset_x_m", 0.0).value),
            y_m=float(self.declare_parameter("map_offset_y_m", 0.0).value),
            yaw_rad=float(self.declare_parameter(
                "map_yaw_offset_rad", 0.0).value),
        )
        if (
            not math.isfinite(self.feedback_timeout_sec)
            or self.feedback_timeout_sec <= 0.0
        ):
            raise ValueError("feedback_timeout_sec must be finite and > 0")
        if not math.isfinite(self.gnss_heading_yaw_bias_rad):
            raise ValueError("gnss_heading_yaw_bias_rad must be finite")
        if (
            not math.isfinite(self.gnss_pair_timeout_sec)
            or self.gnss_pair_timeout_sec <= 0.0
        ):
            raise ValueError("gnss_pair_timeout_sec must be finite and > 0")
        if (
            not math.isfinite(self.gnss_pair_position_std_m)
            or self.gnss_pair_position_std_m < 0.0
        ):
            raise ValueError(
                "gnss_pair_position_std_m must be finite and non-negative"
            )
        if (
            not math.isfinite(self.gnss_moving_speed_threshold_mps)
            or self.gnss_moving_speed_threshold_mps < 0.0
        ):
            raise ValueError(
                "gnss_moving_speed_threshold_mps must be finite and non-negative"
            )

        self.platform_odometry_publisher = None
        if self.publish_platform_odometry:
            self.platform_odometry_publisher = self.create_publisher(
                Odometry, self.platform_odometry_topic, 20)
        self.localization_odometry_publisher = None
        self.localization_pose_publisher = None
        if self.publish_ground_truth_localization:
            self.localization_odometry_publisher = self.create_publisher(
                Odometry, self.localization_odometry_topic, 20)
            self.localization_pose_publisher = self.create_publisher(
                PoseStamped, self.localization_pose_topic, 20)
        self.metric_pose_publisher = None
        if self.publish_metric_pose:
            self.metric_pose_publisher = self.create_publisher(
                PoseStamped, self.metric_pose_topic, 20)

        self.imu_publisher = None
        self.gnss_heading_publisher = None
        self.imu_subscription = None
        if self.relay_imu:
            self.imu_publisher = self.create_publisher(
                Imu, self.output_imu_topic, qos_profile_sensor_data)
            if self.publish_gnss_heading_from_imu:
                self.gnss_heading_publisher = self.create_publisher(
                    Imu,
                    self.output_gnss_heading_topic,
                    qos_profile_sensor_data,
                )
            self.imu_subscription = self.create_subscription(
                Imu,
                self.input_imu_topic,
                self._on_imu,
                qos_profile_sensor_data,
            )
        self.gnss_publisher = None
        self.gnss_navpvt_publisher = None
        self.gnss_navcov_publisher = None
        self.gnss_relpos_publisher = None
        self.gnss_subscription = None
        self.gnss_right_subscription = None
        if self.relay_gnss:
            self.gnss_publisher = self.create_publisher(
                NavSatFix, self.output_gnss_topic, qos_profile_sensor_data)
            self.gnss_subscription = self.create_subscription(
                NavSatFix,
                self.input_gnss_topic,
                self._on_gnss,
                qos_profile_sensor_data,
            )
            if self.publish_gnss_ui_compat:
                # These are representation adapters over actual CARLA actors,
                # not simulation fixtures. Unknown u-blox-only concepts such
                # as RTK carrier state and satellite count stay unavailable.
                self.gnss_navpvt_publisher = self.create_publisher(
                    NavPVT,
                    self.output_gnss_navpvt_topic,
                    qos_profile_sensor_data,
                )
                self.gnss_navcov_publisher = self.create_publisher(
                    NavCOV,
                    self.output_gnss_navcov_topic,
                    qos_profile_sensor_data,
                )
                self.gnss_relpos_publisher = self.create_publisher(
                    NavRELPOSNED9,
                    self.output_gnss_relpos_topic,
                    qos_profile_sensor_data,
                )
                self.gnss_right_subscription = self.create_subscription(
                    NavSatFix,
                    self.input_gnss_right_topic,
                    self._on_gnss_right,
                    qos_profile_sensor_data,
                )

        self.status_publisher = self.create_publisher(
            DiagnosticArray, self.status_topic, 10)
        self.odometry_subscription = self.create_subscription(
            Odometry,
            self.input_odometry_topic,
            self._on_odometry,
            qos_profile_sensor_data,
        )
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        if self.publish_ground_truth_tf:
            self._publish_map_to_odom_static_transform()

        self._last_odometry_monotonic = None
        self._last_imu_monotonic = None
        self._last_gnss_monotonic = None
        self._last_gnss_right_monotonic = None
        self._latest_primary_gnss = None
        self._latest_yaw_rad = None
        self._latest_velocity_ned_mps = (0.0, 0.0, 0.0)
        self._latest_velocity_covariance_ned = (0.0, 0.0, 0.0)
        self._stream_errors = {
            "odometry": "",
            "imu": "",
            "gnss": "",
            "gnss_right": "",
        }
        self.status_timer = self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            "CARLA feedback bridge ready: odom=%s, map offset=(%.3f, %.3f, %.3f)"
            % (
                self.input_odometry_topic,
                self.map_transform.x_m,
                self.map_transform.y_m,
                self.map_transform.yaw_rad,
            )
        )

    def _output_stamp(self, source_stamp):
        if self.stamp_with_reception_time:
            return self.get_clock().now().to_msg()
        return copy.deepcopy(source_stamp)

    def _on_odometry(self, source):
        try:
            position, orientation = transform_pose(
                (
                    source.pose.pose.position.x,
                    source.pose.pose.position.y,
                    source.pose.pose.position.z,
                ),
                (
                    source.pose.pose.orientation.x,
                    source.pose.pose.orientation.y,
                    source.pose.pose.orientation.z,
                    source.pose.pose.orientation.w,
                ),
                self.map_transform,
            )
            validate_motion_values(
                (
                    source.twist.twist.linear.x,
                    source.twist.twist.linear.y,
                    source.twist.twist.linear.z,
                ),
                (
                    source.twist.twist.angular.x,
                    source.twist.twist.angular.y,
                    source.twist.twist.angular.z,
                ),
            )
        except (TypeError, ValueError) as exc:
            self._stream_errors["odometry"] = str(exc)
            self.get_logger().error("Rejected CARLA odometry: %s" % exc)
            return

        stamp = self._output_stamp(source.header.stamp)
        mapped = copy.deepcopy(source)
        mapped.header.stamp = stamp
        mapped.header.frame_id = self.map_frame_id
        mapped.child_frame_id = self.base_frame_id
        mapped.pose.pose.position.x = position[0]
        mapped.pose.pose.position.y = position[1]
        mapped.pose.pose.position.z = position[2]
        mapped.pose.pose.orientation.x = orientation[0]
        mapped.pose.pose.orientation.y = orientation[1]
        mapped.pose.pose.orientation.z = orientation[2]
        mapped.pose.pose.orientation.w = orientation[3]
        mapped.pose.covariance = list(self.odometry_pose_covariance)
        mapped.twist.covariance = list(self.odometry_twist_covariance)

        yaw_rad = quaternion_yaw_rad(mapped.pose.pose.orientation)
        longitudinal = float(mapped.twist.twist.linear.x)
        lateral = float(mapped.twist.twist.linear.y)
        vertical_up = float(mapped.twist.twist.linear.z)
        east = longitudinal * math.cos(yaw_rad) - lateral * math.sin(yaw_rad)
        north = longitudinal * math.sin(yaw_rad) + lateral * math.cos(yaw_rad)
        self._latest_yaw_rad = yaw_rad
        self._latest_velocity_ned_mps = (north, east, -vertical_up)
        # The configured deterministic covariance is isotropic in horizontal
        # velocity; retain its N/E/D diagonal for the UI NAV-COV adapter.
        self._latest_velocity_covariance_ned = (
            float(mapped.twist.covariance[7]),
            float(mapped.twist.covariance[0]),
            float(mapped.twist.covariance[14]),
        )

        if self.platform_odometry_publisher is not None:
            platform = copy.deepcopy(mapped)
            platform.header.frame_id = self.odom_frame_id
            self.platform_odometry_publisher.publish(platform)

        pose = PoseStamped()
        pose.header = copy.deepcopy(mapped.header)
        pose.pose = copy.deepcopy(mapped.pose.pose)
        if self.metric_pose_publisher is not None:
            self.metric_pose_publisher.publish(pose)
        if self.localization_odometry_publisher is not None:
            self.localization_odometry_publisher.publish(mapped)
            self.localization_pose_publisher.publish(pose)

        if self.publish_ground_truth_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = position[0]
            transform.transform.translation.y = position[1]
            transform.transform.translation.z = position[2]
            transform.transform.rotation = copy.deepcopy(
                mapped.pose.pose.orientation)
            self.dynamic_tf_broadcaster.sendTransform(transform)

        self._last_odometry_monotonic = time.monotonic()
        self._stream_errors["odometry"] = ""

    def _on_imu(self, source):
        output = copy.deepcopy(source)
        output.header.stamp = self._output_stamp(source.header.stamp)
        output.header.frame_id = self.imu_frame_id
        try:
            orientation = multiply_quaternions(
                quaternion_from_yaw(self.map_transform.yaw_rad),
                (
                    source.orientation.x,
                    source.orientation.y,
                    source.orientation.z,
                    source.orientation.w,
                ),
            )
            validate_motion_values(
                (
                    source.angular_velocity.x,
                    source.angular_velocity.y,
                    source.angular_velocity.z,
                ),
                (
                    source.linear_acceleration.x,
                    source.linear_acceleration.y,
                    source.linear_acceleration.z,
                ),
            )
        except (TypeError, ValueError) as exc:
            self._stream_errors["imu"] = str(exc)
            self.get_logger().error("Rejected CARLA IMU: %s" % exc)
            return
        output.orientation.x = orientation[0]
        output.orientation.y = orientation[1]
        output.orientation.z = orientation[2]
        output.orientation.w = orientation[3]
        self._latest_yaw_rad = quaternion_yaw_rad(output.orientation)
        output.orientation_covariance = list(self.imu_orientation_covariance)
        output.angular_velocity_covariance = list(
            self.imu_angular_velocity_covariance)
        output.linear_acceleration_covariance = list(
            self.imu_linear_acceleration_covariance)
        self.imu_publisher.publish(output)
        if self.gnss_heading_publisher is not None:
            # CARLA has no dual-antenna heading message. Preserve CAMROD's
            # existing navheading interface by deriving it exclusively from
            # the actual CARLA IMU orientation and applying the inverse of the
            # production receiver's configured -90 degree correction. This is
            # a source-labelled bridge conversion, never a synthetic pose.
            heading = copy.deepcopy(output)
            heading.header.frame_id = self.gnss_heading_frame_id
            heading_orientation = multiply_quaternions(
                quaternion_from_yaw(self.gnss_heading_yaw_bias_rad),
                orientation,
            )
            heading.orientation.x = heading_orientation[0]
            heading.orientation.y = heading_orientation[1]
            heading.orientation.z = heading_orientation[2]
            heading.orientation.w = heading_orientation[3]
            heading.orientation_covariance = [
                1.0e6, 0.0, 0.0,
                0.0, 1.0e6, 0.0,
                0.0, 0.0, self.imu_orientation_covariance[8],
            ]
            self.gnss_heading_publisher.publish(heading)
        self._last_imu_monotonic = time.monotonic()
        self._stream_errors["imu"] = ""

    def _normalized_gnss(self, source, frame_id):
        values = (source.latitude, source.longitude, source.altitude)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("position is non-finite")
        output = copy.deepcopy(source)
        output.header.stamp = self._output_stamp(source.header.stamp)
        output.header.frame_id = frame_id
        output.position_covariance = list(self.gnss_position_covariance)
        output.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return output

    def _on_gnss(self, source):
        try:
            output = self._normalized_gnss(source, self.gnss_frame_id)
        except ValueError as exc:
            self._stream_errors["gnss"] = str(exc)
            self.get_logger().error("Rejected CARLA GNSS: %s" % exc)
            return
        self.gnss_publisher.publish(output)
        self._latest_primary_gnss = output
        if self.publish_gnss_ui_compat:
            self.gnss_navpvt_publisher.publish(build_navpvt(
                output,
                yaw_rad=self._latest_yaw_rad,
                velocity_ned_mps=self._latest_velocity_ned_mps,
                speed_accuracy_mps=math.sqrt(
                    max(self._latest_velocity_covariance_ned[:2])
                ),
                heading_accuracy_rad=math.sqrt(
                    max(0.0, self.imu_orientation_covariance[8])
                ),
            ))
            self.gnss_navcov_publisher.publish(build_navcov(
                output,
                velocity_covariance=self._latest_velocity_covariance_ned,
            ))
        self._last_gnss_monotonic = time.monotonic()
        self._stream_errors["gnss"] = ""

    def _on_gnss_right(self, source):
        try:
            right = self._normalized_gnss(source, "gnss_right_link")
            left = self._latest_primary_gnss
            if left is None or self._last_gnss_monotonic is None:
                raise ValueError("left GNSS sample is unavailable")
            pair_age = time.monotonic() - self._last_gnss_monotonic
            if pair_age < 0.0 or pair_age > self.gnss_pair_timeout_sec:
                raise ValueError(
                    "left/right GNSS pair is stale: %.3fs" % pair_age
                )
            if (
                int(left.status.status) == int(NavSatStatus.STATUS_NO_FIX)
                or int(right.status.status) == int(NavSatStatus.STATUS_NO_FIX)
            ):
                raise ValueError("left/right GNSS pair has no fix")
            speed = math.hypot(
                self._latest_velocity_ned_mps[0],
                self._latest_velocity_ned_mps[1],
            )
            relpos = build_relposned(
                right,
                left,
                position_std_m=self.gnss_pair_position_std_m,
                moving=speed >= self.gnss_moving_speed_threshold_mps,
            )
        except ValueError as exc:
            self._stream_errors["gnss_right"] = str(exc)
            self.get_logger().error("Rejected CARLA right GNSS: %s" % exc)
            return
        self.gnss_relpos_publisher.publish(relpos)
        self._last_gnss_right_monotonic = time.monotonic()
        self._stream_errors["gnss_right"] = ""

    def _publish_map_to_odom_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame_id
        transform.child_frame_id = self.odom_frame_id
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

    def _stream_health(self, now_monotonic):
        last_received = {
            "odometry": self._last_odometry_monotonic,
            "imu": self._last_imu_monotonic,
            "gnss": self._last_gnss_monotonic,
            "gnss_right": self._last_gnss_right_monotonic,
        }
        enabled = {
            "odometry": True,
            "imu": self.relay_imu,
            "gnss": self.relay_gnss,
            "gnss_right": self.relay_gnss and self.publish_gnss_ui_compat,
        }
        ages = {
            name: (
                math.inf
                if received is None
                else float(now_monotonic) - received
            )
            for name, received in last_received.items()
        }
        errors = [
            "%s: %s" % (name, self._stream_errors[name])
            for name in ("odometry", "imu", "gnss", "gnss_right")
            if enabled[name] and self._stream_errors[name]
        ]
        stale = [
            name
            for name in ("odometry", "imu", "gnss", "gnss_right")
            if enabled[name]
            and (ages[name] < 0.0 or ages[name] > self.feedback_timeout_sec)
        ]
        return ages, errors, stale

    def _publish_status(self):
        ages, errors, stale = self._stream_health(time.monotonic())
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "camrod_carla/feedback_bridge"
        status.hardware_id = "vehicle.ranger.default"
        if errors:
            status.level = DiagnosticStatus.ERROR
            status.message = "; ".join(errors)
        elif stale:
            status.level = DiagnosticStatus.ERROR
            status.message = "CARLA feedback missing or stale: %s" % ",".join(
                stale)
        else:
            status.level = DiagnosticStatus.OK
            status.message = "CARLA feedback active"
        status.values = [
            KeyValue(key="odometry_age_sec", value=str(ages["odometry"])),
            KeyValue(key="imu_age_sec", value=str(ages["imu"])),
            KeyValue(key="gnss_age_sec", value=str(ages["gnss"])),
            KeyValue(
                key="gnss_right_age_sec", value=str(ages["gnss_right"])
            ),
            KeyValue(
                key="imu_seen", value=str(
                    self._last_imu_monotonic is not None).lower()
            ),
            KeyValue(
                key="gnss_seen", value=str(
                    self._last_gnss_monotonic is not None).lower()
            ),
            KeyValue(
                key="gnss_right_seen", value=str(
                    self._last_gnss_right_monotonic is not None).lower()
            ),
            KeyValue(
                key="stream_errors",
                value="; ".join(errors),
            ),
            KeyValue(
                key="ground_truth_localization",
                value=str(self.publish_ground_truth_localization).lower(),
            ),
            KeyValue(
                key="map_offset_xy_yaw",
                value="%.6f,%.6f,%.6f" % (
                    self.map_transform.x_m,
                    self.map_transform.y_m,
                    self.map_transform.yaw_rad,
                ),
            ),
        ]
        array.status = [status]
        self.status_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CarlaFeedbackBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
