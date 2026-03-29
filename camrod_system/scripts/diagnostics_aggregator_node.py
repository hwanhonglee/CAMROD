#!/usr/bin/env python3
# HH_260327: Diagnostics aggregator ported from todo/robot_diagnostics to camrod_system.

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node


def _to_int_level(value: object) -> int:
    if isinstance(value, (bytes, bytearray)):
        if not value:
            return int(DiagnosticStatus.OK)
        return int(value[0])
    return int(value)


def _diag_level(value: int) -> bytes:
    return bytes([int(value) & 0xFF])


@dataclass
class Snapshot:
    status: DiagnosticStatus
    stamp_sec: float


class DiagnosticsAggregatorNode(Node):
    def __init__(self) -> None:
        super().__init__("diagnostics_aggregator")

        self.source_topic = str(
            self.declare_parameter("source_topic", "/diagnostics").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/diagnostics_agg").value
        )
        self.publish_period_s = float(
            self.declare_parameter("publish_period_s", 1.0).value
        )
        self.stale_timeout_s = float(
            self.declare_parameter("stale_timeout_s", 3.0).value
        )

        self._snapshots: Dict[str, Snapshot] = {}

        self.sub = self.create_subscription(
            DiagnosticArray,
            self.source_topic,
            self._on_diag,
            20,
        )
        self.pub = self.create_publisher(DiagnosticArray, self.output_topic, 10)
        self.timer = self.create_timer(self.publish_period_s, self._on_timer)

        self.get_logger().info(
            "diagnostics_aggregator ready: "
            f"source={self.source_topic} output={self.output_topic} "
            f"stale_timeout_s={self.stale_timeout_s:.1f}"
        )

    def _copy_status(self, src: DiagnosticStatus) -> DiagnosticStatus:
        dst = DiagnosticStatus()
        dst.level = src.level
        dst.name = src.name
        dst.message = src.message
        dst.hardware_id = src.hardware_id
        dst.values = list(src.values)
        return dst

    def _on_diag(self, msg: DiagnosticArray) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        for status in msg.status:
            key = status.name if status.name else status.hardware_id
            if not key:
                continue
            self._snapshots[key] = Snapshot(
                status=self._copy_status(status),
                stamp_sec=now_sec,
            )

    def _on_timer(self) -> None:
        if not self._snapshots:
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        out = DiagnosticArray()
        out.header.stamp = now.to_msg()

        for key in sorted(self._snapshots.keys()):
            snap = self._snapshots[key]
            age = now_sec - snap.stamp_sec
            status = self._copy_status(snap.status)

            if age > self.stale_timeout_s:
                status.level = _diag_level(DiagnosticStatus.STALE)
                status.message = f"stale ({age:.1f}s)"

            out.status.append(status)

        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = DiagnosticsAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
