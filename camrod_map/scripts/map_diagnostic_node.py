#!/usr/bin/env python3
# HH_260310-00:00 Module-level diagnostics publisher for map/cost-grid outputs.

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

from avg_msgs.msg import AvgMapMsgs, ModuleHealth
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray


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


class MapDiagnosticNode(Node):
    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("map_diagnostic")

        self.diagnostic_topic = str(
            # HH_260311-00:00 Single consolidated ROS diagnostic stream.
            self.declare_parameter("diagnostic_topic", "diagnostic").value
        )
        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.2).value)
        self.stale_timeout_s = float(self.declare_parameter("stale_timeout_s", 2.0).value)
        # 2026-03-18: Inflation markers are RViz/debug-only and can be disabled by launch.
        self.require_inflation_markers = bool(
            self.declare_parameter("require_inflation_markers", False).value
        )

        self.topic_lanelet_grid = str(
            self.declare_parameter("topic_lanelet_grid", "/map/cost_grid/lanelet").value
        )
        self.topic_planning_base_grid = str(
            self.declare_parameter("topic_planning_base_grid", "/map/cost_grid/planning_base").value
        )
        self.topic_lanelet_markers = str(
            self.declare_parameter("topic_lanelet_markers", "/map/cost_grid/lanelet_markers").value
        )
        self.topic_lidar_markers = str(
            self.declare_parameter("topic_lidar_markers", "/map/cost_grid/lidar_markers").value
        )
        self.topic_radar_markers = str(
            self.declare_parameter("topic_radar_markers", "/map/cost_grid/radar_markers").value
        )
        self.topic_inflation_markers = str(
            self.declare_parameter("topic_inflation_markers", "/map/cost_grid/inflation_markers").value
        )

        self._diag_pub = self.create_publisher(DiagnosticArray, self.diagnostic_topic, 10)

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        marker_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._msg_lanelet_grid = OccupancyGrid()
        self._msg_planning_base_grid = OccupancyGrid()
        self._msg_lanelet_markers = MarkerArray()
        self._msg_lidar_markers = MarkerArray()
        self._msg_radar_markers = MarkerArray()
        self._msg_inflation_markers = MarkerArray()

        self._state: Dict[str, StampState] = {
            "lanelet_grid": StampState(),
            "planning_base_grid": StampState(),
        }
        if self.require_inflation_markers:
            self._state["inflation_markers"] = StampState()

        self.create_subscription(
            OccupancyGrid, self.topic_lanelet_grid, self._on_lanelet_grid, latched_qos
        )
        self.create_subscription(
            OccupancyGrid,
            self.topic_planning_base_grid,
            self._on_planning_base_grid,
            latched_qos,
        )
        self.create_subscription(
            MarkerArray, self.topic_lanelet_markers, self._on_lanelet_markers, marker_qos
        )
        self.create_subscription(MarkerArray, self.topic_lidar_markers, self._on_lidar_markers, marker_qos)
        self.create_subscription(MarkerArray, self.topic_radar_markers, self._on_radar_markers, marker_qos)
        self.create_subscription(
            MarkerArray, self.topic_inflation_markers, self._on_inflation_markers, marker_qos
        )

        self.create_timer(self.publish_period_s, self._on_timer)
        self.get_logger().info(
            "map_diagnostic: diagnostic=%s period=%.2fs"
            % (self.diagnostic_topic, self.publish_period_s)
        )

    # Implements `_set_stamp` behavior.
    def _set_stamp(self, key: str) -> None:
        self._state[key].stamp = self.get_clock().now()

    # Implements `_on_lanelet_grid` behavior.
    def _on_lanelet_grid(self, msg: OccupancyGrid) -> None:
        self._msg_lanelet_grid = msg
        self._set_stamp("lanelet_grid")

    # Implements `_on_planning_base_grid` behavior.
    def _on_planning_base_grid(self, msg: OccupancyGrid) -> None:
        self._msg_planning_base_grid = msg
        self._set_stamp("planning_base_grid")

    # Implements `_on_lanelet_markers` behavior.
    def _on_lanelet_markers(self, msg: MarkerArray) -> None:
        self._msg_lanelet_markers = msg

    # Implements `_on_lidar_markers` behavior.
    def _on_lidar_markers(self, msg: MarkerArray) -> None:
        self._msg_lidar_markers = msg

    # Implements `_on_radar_markers` behavior.
    def _on_radar_markers(self, msg: MarkerArray) -> None:
        self._msg_radar_markers = msg

    # Implements `_on_inflation_markers` behavior.
    def _on_inflation_markers(self, msg: MarkerArray) -> None:
        self._msg_inflation_markers = msg
        if "inflation_markers" in self._state:
            self._set_stamp("inflation_markers")

    # Implements `_health` behavior.
    def _health(self, now: Time) -> ModuleHealth:
        required = ["lanelet_grid", "planning_base_grid"]
        if self.require_inflation_markers:
            required.append("inflation_markers")
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
        out.module_name = "map"
        if missing:
            out.level = ModuleHealth.ERROR
            out.message = "missing/stale required map streams"
            out.missing_topics = missing
        else:
            out.level = ModuleHealth.OK
            out.message = "map streams healthy"
        return out

    # Implements `_on_timer` behavior.
    def _on_timer(self) -> None:
        now = self.get_clock().now()
        msg = AvgMapMsgs()
        msg.stamp = now.to_msg()
        msg.health = self._health(now)
        msg.lanelet_cost_grid = self._msg_lanelet_grid
        msg.planning_base_grid = self._msg_planning_base_grid
        msg.lanelet_markers = self._msg_lanelet_markers
        msg.lidar_markers = self._msg_lidar_markers
        msg.radar_markers = self._msg_radar_markers
        msg.inflation_markers = self._msg_inflation_markers
        diag = DiagnosticArray()
        diag.header.stamp = msg.stamp
        st = DiagnosticStatus()
        st.name = "map/diagnostic"
        st.hardware_id = "map"
        st.level = _diag_level(msg.health.level)
        st.message = msg.health.message
        st.values.append(KeyValue(key="category", value="map"))
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
    node = MapDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"map_diagnostic runtime exception: {e}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
