#!/usr/bin/env python3
# HH_260309 Aggregate module health topics into system diagnostic summary.

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from avg_msgs.msg import ModuleHealth, SystemDiagnostic
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


@dataclass
class CachedHealth:
    msg: ModuleHealth
    stamp: Time


def _topic_to_module(topic: str) -> str:
    t = topic.strip("/")
    if not t:
        return "unknown"
    return t.split("/")[-1]


class ModuleStatusAggregator(Node):
    def __init__(self) -> None:
        super().__init__("module_status_aggregator")

        self.input_topics: List[str] = list(
            self.declare_parameter(
                "input_topics",
                [
                    "/map/healthchecker",
                    "/sensing/healthchecker",
                    "/localization/healthchecker",
                    "/planning/healthchecker",
                    "/platform/healthchecker",
                    "/perception/healthchecker",
                ],
            ).value
        )
        self.period_s = float(self.declare_parameter("publish_period_s", 1.0).value)
        self.stale_timeout_s = float(self.declare_parameter("stale_timeout_s", 3.0).value)
        self.system_diagnostic_topic = str(
            self.declare_parameter("diagnostic_topic", "/system/diagnostic").value
        )
        self.diag_topic = str(
            self.declare_parameter("diagnostics_topic", "/system/diagnostics/modules").value
        )

        self.pub_system_diagnostic = self.create_publisher(
            SystemDiagnostic, self.system_diagnostic_topic, 10
        )
        self.pub_diag = self.create_publisher(DiagnosticArray, self.diag_topic, 10)

        self.cache: Dict[str, CachedHealth] = {}
        self.subs = []
        for topic in self.input_topics:
            sub = self.create_subscription(
                ModuleHealth,
                topic,
                lambda msg, t=topic: self._on_health(msg, t),
                10,
            )
            self.subs.append(sub)

        self.timer = self.create_timer(self.period_s, self._on_timer)
        self.get_logger().info(
            "HH_260309 module status aggregator started: inputs=%s"
            % ",".join(self.input_topics)
        )

    def _on_health(self, msg: ModuleHealth, topic: str) -> None:
        self.cache[topic] = CachedHealth(msg=msg, stamp=self.get_clock().now())

    def _make_stale(self, topic: str, now: Time) -> ModuleHealth:
        out = ModuleHealth()
        out.stamp = now.to_msg()
        out.module_name = _topic_to_module(topic)
        out.level = ModuleHealth.ERROR
        out.message = "stale_or_missing"
        out.missing_nodes = []
        out.missing_topics = [topic]
        out.missing_lifecycle_nodes = []
        return out

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        modules: List[ModuleHealth] = []

        for topic in self.input_topics:
            cached = self.cache.get(topic)
            if cached is None:
                modules.append(self._make_stale(topic, now))
                continue

            age = (now - cached.stamp).nanoseconds / 1e9
            if age > self.stale_timeout_s:
                modules.append(self._make_stale(topic, now))
                continue

            modules.append(cached.msg)

        system_ok = all(m.level == ModuleHealth.OK for m in modules)
        system_diagnostic = SystemDiagnostic()
        system_diagnostic.stamp = now.to_msg()
        system_diagnostic.system_ok = bool(system_ok)
        system_diagnostic.message = "ok" if system_ok else "degraded"
        system_diagnostic.modules = modules
        self.pub_system_diagnostic.publish(system_diagnostic)

        diag = DiagnosticArray()
        diag.header.stamp = system_diagnostic.stamp
        for m in modules:
            st = DiagnosticStatus()
            st.name = f"{m.module_name}/aggregated_health"
            st.hardware_id = m.module_name
            st.level = bytes([int(m.level)])
            st.message = m.message
            st.values.append(KeyValue(key="missing_nodes", value=",".join(m.missing_nodes)))
            st.values.append(KeyValue(key="missing_topics", value=",".join(m.missing_topics)))
            st.values.append(
                KeyValue(
                    key="missing_lifecycle_nodes",
                    value=",".join(m.missing_lifecycle_nodes),
                )
            )
            diag.status.append(st)
        self.pub_diag.publish(diag)


def main() -> None:
    rclpy.init()
    node = ModuleStatusAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
