#!/usr/bin/env python3
# HH_260310-00:00 Module-level diagnostics publisher for sensing outputs.

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

from avg_msgs.msg import AvgSensingMsgs, ModuleHealth
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, Range


@dataclass
class StampState:
    stamp: Optional[Time] = None


class SensingDiagnosticNode(Node):
    def __init__(self) -> None:
        super().__init__("sensing_diagnostic")

        self.msgs_topic = str(self.declare_parameter("msgs_topic", "/sensing/messages").value)
        self.diagnostic_topic = str(
            self.declare_parameter("diagnostic_topic", "/sensing/diagnostic").value
        )
        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.2).value)
        self.stale_timeout_s = float(self.declare_parameter("stale_timeout_s", 1.5).value)
        self.require_radar_cost_grid = bool(
            self.declare_parameter("require_radar_cost_grid", True).value
        )

        self.topic_lidar_filtered = str(
            self.declare_parameter("topic_lidar_filtered", "/sensing/lidar/points_filtered").value
        )
        self.topic_camera_image = str(
            self.declare_parameter("topic_camera_image", "/sensing/camera/processed/image").value
        )
        self.topic_camera_info = str(
            self.declare_parameter(
                "topic_camera_info", "/sensing/camera/processed/camera_info"
            ).value
        )
        self.topic_twist_cov = str(
            self.declare_parameter(
                "topic_twist_cov", "/sensing/platform_velocity_converter/twist_with_covariance"
            ).value
        )
        self.topic_imu = str(self.declare_parameter("topic_imu", "/sensing/imu/data").value)
        self.topic_lidar_cost = str(
            self.declare_parameter("topic_lidar_cost", "/sensing/lidar/near_cost_grid").value
        )
        self.topic_radar_cost = str(
            self.declare_parameter("topic_radar_cost", "/sensing/radar/near_cost_grid").value
        )

        self.topic_radar_front = str(
            self.declare_parameter("topic_radar_front", "/sensing/radar/front/range").value
        )
        self.topic_radar_right1 = str(
            self.declare_parameter("topic_radar_right1", "/sensing/radar/right1/range").value
        )
        self.topic_radar_right2 = str(
            self.declare_parameter("topic_radar_right2", "/sensing/radar/right2/range").value
        )
        self.topic_radar_left1 = str(
            self.declare_parameter("topic_radar_left1", "/sensing/radar/left1/range").value
        )
        self.topic_radar_left2 = str(
            self.declare_parameter("topic_radar_left2", "/sensing/radar/left2/range").value
        )
        self.topic_radar_rear = str(
            self.declare_parameter("topic_radar_rear", "/sensing/radar/rear/range").value
        )

        self._msgs_pub = self.create_publisher(AvgSensingMsgs, self.msgs_topic, 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, self.diagnostic_topic, 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        grid_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._msg_lidar_filtered = PointCloud2()
        self._msg_camera_image = Image()
        self._msg_camera_info = CameraInfo()
        self._msg_imu = Imu()
        self._msg_twist_cov = TwistWithCovarianceStamped()
        self._msg_lidar_cost = OccupancyGrid()
        self._msg_radar_cost = OccupancyGrid()
        self._msg_radar_front = Range()
        self._msg_radar_right1 = Range()
        self._msg_radar_right2 = Range()
        self._msg_radar_left1 = Range()
        self._msg_radar_left2 = Range()
        self._msg_radar_rear = Range()

        self._state: Dict[str, StampState] = {
            "lidar_filtered": StampState(),
            "camera_image": StampState(),
            "camera_info": StampState(),
            "imu": StampState(),
            "twist_cov": StampState(),
            "lidar_cost": StampState(),
            "radar_cost": StampState(),
        }

        self.create_subscription(
            PointCloud2,
            self.topic_lidar_filtered,
            self._on_lidar_filtered,
            sensor_qos,
        )
        self.create_subscription(Image, self.topic_camera_image, self._on_camera_image, sensor_qos)
        self.create_subscription(CameraInfo, self.topic_camera_info, self._on_camera_info, sensor_qos)
        self.create_subscription(Imu, self.topic_imu, self._on_imu, sensor_qos)
        self.create_subscription(
            TwistWithCovarianceStamped, self.topic_twist_cov, self._on_twist_cov, sensor_qos
        )
        self.create_subscription(OccupancyGrid, self.topic_lidar_cost, self._on_lidar_cost, grid_qos)
        self.create_subscription(OccupancyGrid, self.topic_radar_cost, self._on_radar_cost, grid_qos)

        self.create_subscription(Range, self.topic_radar_front, self._on_radar_front, sensor_qos)
        self.create_subscription(Range, self.topic_radar_right1, self._on_radar_right1, sensor_qos)
        self.create_subscription(Range, self.topic_radar_right2, self._on_radar_right2, sensor_qos)
        self.create_subscription(Range, self.topic_radar_left1, self._on_radar_left1, sensor_qos)
        self.create_subscription(Range, self.topic_radar_left2, self._on_radar_left2, sensor_qos)
        self.create_subscription(Range, self.topic_radar_rear, self._on_radar_rear, sensor_qos)

        self.create_timer(self.publish_period_s, self._on_timer)
        self.get_logger().info(
            "sensing_diagnostic: messages=%s diagnostic=%s period=%.2fs"
            % (self.msgs_topic, self.diagnostic_topic, self.publish_period_s)
        )

    def _set_stamp(self, key: str) -> None:
        self._state[key].stamp = self.get_clock().now()

    def _on_lidar_filtered(self, msg: PointCloud2) -> None:
        self._msg_lidar_filtered = msg
        self._set_stamp("lidar_filtered")

    def _on_camera_image(self, msg: Image) -> None:
        self._msg_camera_image = msg
        self._set_stamp("camera_image")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._msg_camera_info = msg
        self._set_stamp("camera_info")

    def _on_imu(self, msg: Imu) -> None:
        self._msg_imu = msg
        self._set_stamp("imu")

    def _on_twist_cov(self, msg: TwistWithCovarianceStamped) -> None:
        self._msg_twist_cov = msg
        self._set_stamp("twist_cov")

    def _on_lidar_cost(self, msg: OccupancyGrid) -> None:
        self._msg_lidar_cost = msg
        self._set_stamp("lidar_cost")

    def _on_radar_cost(self, msg: OccupancyGrid) -> None:
        self._msg_radar_cost = msg
        self._set_stamp("radar_cost")

    def _on_radar_front(self, msg: Range) -> None:
        self._msg_radar_front = msg

    def _on_radar_right1(self, msg: Range) -> None:
        self._msg_radar_right1 = msg

    def _on_radar_right2(self, msg: Range) -> None:
        self._msg_radar_right2 = msg

    def _on_radar_left1(self, msg: Range) -> None:
        self._msg_radar_left1 = msg

    def _on_radar_left2(self, msg: Range) -> None:
        self._msg_radar_left2 = msg

    def _on_radar_rear(self, msg: Range) -> None:
        self._msg_radar_rear = msg

    def _health(self, now: Time) -> ModuleHealth:
        required = ["lidar_filtered", "twist_cov", "lidar_cost"]
        if self.require_radar_cost_grid:
            required.append("radar_cost")

        missing = []
        for key in required:
            st = self._state[key].stamp
            if st is None:
                missing.append(key)
                continue
            age = (now - st).nanoseconds * 1e-9
            if age > self.stale_timeout_s:
                missing.append(f"{key}(stale:{age:.2f}s)")

        out = ModuleHealth()
        out.stamp = now.to_msg()
        out.module_name = "sensing"
        if missing:
            out.level = ModuleHealth.ERROR
            out.message = "missing/stale required sensing streams"
            out.missing_topics = missing
        else:
            out.level = ModuleHealth.OK
            out.message = "sensing streams healthy"
        return out

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        msg = AvgSensingMsgs()
        msg.stamp = now.to_msg()
        msg.health = self._health(now)
        msg.lidar_filtered = self._msg_lidar_filtered
        msg.camera_image = self._msg_camera_image
        msg.camera_info = self._msg_camera_info
        msg.imu_data = self._msg_imu
        msg.platform_twist_cov = self._msg_twist_cov
        msg.radar_front = self._msg_radar_front
        msg.radar_right1 = self._msg_radar_right1
        msg.radar_right2 = self._msg_radar_right2
        msg.radar_left1 = self._msg_radar_left1
        msg.radar_left2 = self._msg_radar_left2
        msg.radar_rear = self._msg_radar_rear
        msg.lidar_near_cost_grid = self._msg_lidar_cost
        msg.radar_near_cost_grid = self._msg_radar_cost
        self._msgs_pub.publish(msg)

        diag = DiagnosticArray()
        diag.header.stamp = msg.stamp
        st = DiagnosticStatus()
        st.name = "sensing/diagnostic"
        st.hardware_id = "sensing"
        st.level = bytes([int(msg.health.level)])
        st.message = msg.health.message
        st.values.append(KeyValue(key="missing_topics", value=",".join(msg.health.missing_topics)))
        st.values.append(KeyValue(key="missing_nodes", value=",".join(msg.health.missing_nodes)))
        st.values.append(
            KeyValue(
                key="missing_lifecycle_nodes",
                value=",".join(msg.health.missing_lifecycle_nodes),
            )
        )
        diag.status.append(st)
        self._diag_pub.publish(diag)


def main() -> None:
    rclpy.init()
    node = SensingDiagnosticNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
