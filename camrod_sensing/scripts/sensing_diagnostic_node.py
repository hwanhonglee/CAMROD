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

from avg_msgs.msg import (
    AvgSensingCamera,
    AvgSensingGnss,
    AvgSensingImu,
    AvgSensingLidar,
    AvgSensingMsgs,
    AvgSensingRadar,
    ModuleHealth,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix, PointCloud2, Range


# Implements `_diag_level` behavior.
def _diag_level(value: object) -> bytes:
    # HH_260311-00:00 Humble DiagnosticStatus constants/values may be bytes.
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


class SensingDiagnosticNode(Node):
    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("sensing_diagnostic")

        self.diagnostic_topic = str(
            # HH_260311-00:00 Single consolidated ROS diagnostic stream.
            self.declare_parameter("diagnostic_topic", "diagnostic").value
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
        self.topic_gnss_navsat = str(
            self.declare_parameter("topic_gnss_navsat", "/sensing/gnss/navsatfix").value
        )
        self.topic_gnss_pose_cov = str(
            self.declare_parameter(
                "topic_gnss_pose_cov", "/sensing/gnss/pose_with_covariance"
            ).value
        )
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
        self._msg_gnss_navsat = NavSatFix()
        self._msg_gnss_pose_cov = PoseWithCovarianceStamped()
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
            NavSatFix, self.topic_gnss_navsat, self._on_gnss_navsat, sensor_qos
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.topic_gnss_pose_cov,
            self._on_gnss_pose_cov,
            sensor_qos,
        )
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
            "sensing_diagnostic: diagnostic=%s period=%.2fs"
            % (self.diagnostic_topic, self.publish_period_s)
        )

    # Implements `_set_stamp` behavior.
    def _set_stamp(self, key: str) -> None:
        self._state[key].stamp = self.get_clock().now()

    # Implements `_on_lidar_filtered` behavior.
    def _on_lidar_filtered(self, msg: PointCloud2) -> None:
        self._msg_lidar_filtered = msg
        self._set_stamp("lidar_filtered")

    # Implements `_on_camera_image` behavior.
    def _on_camera_image(self, msg: Image) -> None:
        self._msg_camera_image = msg
        self._set_stamp("camera_image")

    # Implements `_on_camera_info` behavior.
    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._msg_camera_info = msg
        self._set_stamp("camera_info")

    # Implements `_on_imu` behavior.
    def _on_imu(self, msg: Imu) -> None:
        self._msg_imu = msg
        self._set_stamp("imu")

    # Implements `_on_gnss_navsat` behavior.
    def _on_gnss_navsat(self, msg: NavSatFix) -> None:
        self._msg_gnss_navsat = msg

    # Implements `_on_gnss_pose_cov` behavior.
    def _on_gnss_pose_cov(self, msg: PoseWithCovarianceStamped) -> None:
        self._msg_gnss_pose_cov = msg

    # Implements `_on_twist_cov` behavior.
    def _on_twist_cov(self, msg: TwistWithCovarianceStamped) -> None:
        self._msg_twist_cov = msg
        self._set_stamp("twist_cov")

    # Implements `_on_lidar_cost` behavior.
    def _on_lidar_cost(self, msg: OccupancyGrid) -> None:
        self._msg_lidar_cost = msg
        self._set_stamp("lidar_cost")

    # Implements `_on_radar_cost` behavior.
    def _on_radar_cost(self, msg: OccupancyGrid) -> None:
        self._msg_radar_cost = msg
        self._set_stamp("radar_cost")

    # Implements `_on_radar_front` behavior.
    def _on_radar_front(self, msg: Range) -> None:
        self._msg_radar_front = msg

    # Implements `_on_radar_right1` behavior.
    def _on_radar_right1(self, msg: Range) -> None:
        self._msg_radar_right1 = msg

    # Implements `_on_radar_right2` behavior.
    def _on_radar_right2(self, msg: Range) -> None:
        self._msg_radar_right2 = msg

    # Implements `_on_radar_left1` behavior.
    def _on_radar_left1(self, msg: Range) -> None:
        self._msg_radar_left1 = msg

    # Implements `_on_radar_left2` behavior.
    def _on_radar_left2(self, msg: Range) -> None:
        self._msg_radar_left2 = msg

    # Implements `_on_radar_rear` behavior.
    def _on_radar_rear(self, msg: Range) -> None:
        self._msg_radar_rear = msg

    # Implements `_health` behavior.
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

    # Implements `_on_timer` behavior.
    def _on_timer(self) -> None:
        now = self.get_clock().now()
        msg = AvgSensingMsgs()
        msg.stamp = now.to_msg()
        msg.health = self._health(now)
        msg.lidar = AvgSensingLidar(
            points_filtered=self._msg_lidar_filtered,
            near_cost_grid=self._msg_lidar_cost,
        )
        msg.camera = AvgSensingCamera(
            image=self._msg_camera_image,
            camera_info=self._msg_camera_info,
        )
        msg.imu = AvgSensingImu(
            imu_data=self._msg_imu,
            platform_twist_cov=self._msg_twist_cov,
        )
        msg.gnss = AvgSensingGnss(
            navsatfix=self._msg_gnss_navsat,
            pose_with_covariance=self._msg_gnss_pose_cov,
        )
        msg.radar = AvgSensingRadar(
            front=self._msg_radar_front,
            right1=self._msg_radar_right1,
            right2=self._msg_radar_right2,
            left1=self._msg_radar_left1,
            left2=self._msg_radar_left2,
            rear=self._msg_radar_rear,
            near_cost_grid=self._msg_radar_cost,
        )
        diag = DiagnosticArray()
        diag.header.stamp = msg.stamp
        st = DiagnosticStatus()
        st.name = "sensing/diagnostic"
        st.hardware_id = "sensing"
        st.level = _diag_level(msg.health.level)
        st.message = msg.health.message
        st.values.append(KeyValue(key="category", value="sensing"))
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


# Entry point for this executable.
def main() -> None:
    rclpy.init()
    node = SensingDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"sensing_diagnostic runtime exception: {e}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
