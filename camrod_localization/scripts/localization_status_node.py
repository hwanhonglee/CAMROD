#!/usr/bin/env python3
# HH_260310-00:00 Module-level status_stream publisher for localization outputs.

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from avg_msgs.msg import AvgLocalizationStatusStream, AvgLocalizationMsgs, ModuleState
from status_msgs.msg import StatusArray, StatusStatus, KeyValue
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix


# Implements `_diag_level` behavior.
def _diag_level(value: object) -> bytes:
    # HH_260311-00:00 Humble StatusStatus constants/values may be bytes.
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 1:
            return bytes(value)
        if len(value) > 1:
            return bytes([value[0]])
        return b"\x00"
    return bytes([int(value) & 0xFF])


@dataclass
class StampState:
    stamp: Optional[Time] = None


class LocalizationStatusNode(Node):
    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("localization_status")

        self.status_topic = str(
            # HH_260311-00:00 Single consolidated ROS status stream.
            self.declare_parameter("status_topic", "status").value
        )
        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.2).value)
        self.stale_timeout_s = float(self.declare_parameter("stale_timeout_s", 1.5).value)

        self.topic_pose = str(self.declare_parameter("topic_pose", "/localization/pose").value)
        self.topic_pose_cov = str(
            self.declare_parameter("topic_pose_cov", "/localization/pose_with_covariance").value
        )
        self.topic_odom = str(
            self.declare_parameter("topic_odom", "/localization/odometry/filtered").value
        )
        self.topic_twist = str(
            self.declare_parameter("topic_twist", "/localization/twist").value
        )
        self.topic_gnss_navsat = str(
            self.declare_parameter("topic_gnss_navsat", "/sensing/gnss/ublox_gps_node/fix").value
        )
        self.topic_gnss_pose_cov = str(
            self.declare_parameter(
                "topic_gnss_pose_cov", "/sensing/gnss/pose_with_covariance"
            ).value
        )
        self.topic_imu = str(
            self.declare_parameter("topic_imu", "/sensing/imu/data").value
        )
        self.topic_wheel_odom = str(
            self.declare_parameter("topic_wheel_odom", "/platform/wheel/odometry").value
        )
        self.topic_diag = str(
            self.declare_parameter("topic_diag", "/localization/eskf/status").value
        )

        self._diag_pub = self.create_publisher(StatusArray, self.status_topic, 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        default_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._msg_pose = PoseStamped()
        self._msg_pose_cov = PoseWithCovarianceStamped()
        self._msg_odom = Odometry()
        self._msg_twist = TwistStamped()
        self._msg_navsat = NavSatFix()
        self._msg_gnss_pose_cov = PoseWithCovarianceStamped()
        self._msg_imu = Imu()
        self._msg_wheel_odom = Odometry()
        self._msg_diag = AvgLocalizationStatusStream()

        self._state: Dict[str, StampState] = {
            "pose": StampState(),
            "pose_cov": StampState(),
            "odom": StampState(),
            "diag": StampState(),
        }

        self.create_subscription(PoseStamped, self.topic_pose, self._on_pose, default_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, self.topic_pose_cov, self._on_pose_cov, default_qos
        )
        self.create_subscription(Odometry, self.topic_odom, self._on_odom, default_qos)
        self.create_subscription(TwistStamped, self.topic_twist, self._on_twist, default_qos)
        self.create_subscription(
            NavSatFix, self.topic_gnss_navsat, self._on_gnss_navsat, sensor_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.topic_gnss_pose_cov,
            self._on_gnss_pose_cov,
            sensor_qos,
        )
        self.create_subscription(
            Imu, self.topic_imu, self._on_imu, sensor_qos
        )
        self.create_subscription(Odometry, self.topic_wheel_odom, self._on_wheel_odom, sensor_qos)
        self.create_subscription(
            AvgLocalizationStatusStream, self.topic_diag, self._on_diag, default_qos
        )

        self.create_timer(self.publish_period_s, self._on_timer)
        self.get_logger().info(
            "localization_status: status=%s period=%.2fs"
            % (self.status_topic, self.publish_period_s)
        )

    # Implements `_set_stamp` behavior.
    def _set_stamp(self, key: str) -> None:
        self._state[key].stamp = self.get_clock().now()

    # Implements `_on_pose` behavior.
    def _on_pose(self, msg: PoseStamped) -> None:
        self._msg_pose = msg
        self._set_stamp("pose")

    # Implements `_on_pose_cov` behavior.
    def _on_pose_cov(self, msg: PoseWithCovarianceStamped) -> None:
        self._msg_pose_cov = msg
        self._set_stamp("pose_cov")

    # Implements `_on_odom` behavior.
    def _on_odom(self, msg: Odometry) -> None:
        self._msg_odom = msg
        self._set_stamp("odom")

    # Implements `_on_twist` behavior.
    def _on_twist(self, msg: TwistStamped) -> None:
        self._msg_twist = msg

    # Implements `_on_gnss_navsat` behavior.
    def _on_gnss_navsat(self, msg: NavSatFix) -> None:
        self._msg_navsat = msg

    # Implements `_on_gnss_pose_cov` behavior.
    def _on_gnss_pose_cov(self, msg: PoseWithCovarianceStamped) -> None:
        self._msg_gnss_pose_cov = msg

    # Implements `_on_imu` behavior.
    def _on_imu(self, msg: Imu) -> None:
        self._msg_imu = msg

    # Implements `_on_wheel_odom` behavior.
    def _on_wheel_odom(self, msg: Odometry) -> None:
        self._msg_wheel_odom = msg

    # Implements `_on_diag` behavior.
    def _on_diag(self, msg: AvgLocalizationStatusStream) -> None:
        self._msg_diag = msg
        self._set_stamp("diag")

    # Implements `_state` behavior.
    def _state(self, now: Time) -> ModuleState:
        required = ["pose", "pose_cov", "diag"]
        missing = []
        for key in required:
            st = self._state[key].stamp
            if st is None:
                missing.append(key)
                continue
            age = (now - st).nanoseconds * 1e-9
            if age > self.stale_timeout_s:
                missing.append(f"{key}(stale:{age:.2f}s)")

        out = ModuleState()
        out.stamp = now.to_msg()
        out.module_name = "localization"
        if missing:
            out.level = ModuleState.ERROR
            out.message = "missing/stale required localization streams"
            out.missing_topics = missing
        else:
            out.level = ModuleState.OK
            out.message = "localization streams statey"
        return out

    # Implements `_on_timer` behavior.
    def _on_timer(self) -> None:
        now = self.get_clock().now()
        msg = AvgLocalizationMsgs()
        msg.stamp = now.to_msg()
        msg.state = self._state(now)

        msg.localization_pose = self._msg_pose
        msg.localization_pose_cov = self._msg_pose_cov
        msg.localization_odom = self._msg_odom
        msg.localization_twist = self._msg_twist
        msg.gnss_navsatfix = self._msg_navsat
        msg.gnss_pose_cov = self._msg_gnss_pose_cov
        msg.imu_data = self._msg_imu
        msg.wheel_odometry = self._msg_wheel_odom

        msg.gnss_update_accepted = self._msg_diag.gnss_update_accepted
        msg.gnss_innovation_norm = float(self._msg_diag.gnss_innovation_norm)
        msg.wheel_update_accepted = self._msg_diag.wheel_update_accepted
        msg.wheel_innovation_norm = float(self._msg_diag.wheel_innovation_norm)
        msg.covariance_trace = float(self._msg_diag.covariance_trace)

        diag = StatusArray()
        diag.header.stamp = msg.stamp
        st = StatusStatus()
        st.name = "localization/status"
        st.hardware_id = "localization"
        st.level = _diag_level(msg.state.level)
        st.message = msg.state.message
        st.values.append(KeyValue(key="category", value="localization"))
        st.values.append(KeyValue(key="missing_topics", value=",".join(msg.state.missing_topics)))
        st.values.append(KeyValue(key="missing_nodes", value=",".join(msg.state.missing_nodes)))
        st.values.append(
            KeyValue(
                key="missing_lifecycle_nodes",
                value=",".join(msg.state.missing_lifecycle_nodes),
            )
        )
        diag.status.append(st)
        self._diag_pub.publish(diag)


# Entry point for this executable.
def main() -> None:
    rclpy.init()
    node = LocalizationStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"localization_status runtime exception: {e}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
