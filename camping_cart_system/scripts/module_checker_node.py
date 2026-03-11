#!/usr/bin/env python3
# HH_260309 Generic module checker: per-module graph/lifecycle health publisher.

from __future__ import annotations

import time
from typing import Dict, List, Set, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from avg_msgs.msg import ModuleHealth


def _normalize_name(name: str) -> str:
    if not name:
        return name
    return name if name.startswith("/") else f"/{name}"


class ModuleCheckerNode(Node):
    def __init__(self) -> None:
        super().__init__("module_checker")

        self.module_name = self.declare_parameter("module_name", "module").value
        self.check_period_s = float(self.declare_parameter("check_period_s", 1.0).value)
        self.warn_throttle_sec = float(self.declare_parameter("warn_throttle_sec", 2.0).value)
        self.publish_ok = bool(self.declare_parameter("publish_ok", True).value)

        sentinel = "__none__"
        self.declare_parameter("required_nodes", [sentinel], ParameterDescriptor())
        raw_required_nodes = list(
            self.get_parameter("required_nodes").get_parameter_value().string_array_value
        )
        self.required_nodes = [] if raw_required_nodes == [sentinel] else [
            _normalize_name(n) for n in raw_required_nodes
        ]

        self.declare_parameter("required_topics", [sentinel], ParameterDescriptor())
        raw_required_topics = list(
            self.get_parameter("required_topics").get_parameter_value().string_array_value
        )
        self.required_topics = [] if raw_required_topics == [sentinel] else [
            _normalize_name(t) for t in raw_required_topics
        ]

        self.declare_parameter("required_lifecycle_nodes", [sentinel], ParameterDescriptor())
        raw_required_lifecycle = list(
            self.get_parameter("required_lifecycle_nodes")
            .get_parameter_value()
            .string_array_value
        )
        self.required_lifecycle_nodes = [] if raw_required_lifecycle == [sentinel] else [
            _normalize_name(n) for n in raw_required_lifecycle
        ]

        diagnostics_topic = str(
            self.declare_parameter(
                "diagnostics_topic", f"/{self.module_name}/diagnostic"
            ).value
        )
        default_health_topic = f"/{self.module_name}/healthchecker"
        health_topic = str(self.declare_parameter("health_topic", default_health_topic).value)

        self.pub_health = self.create_publisher(ModuleHealth, health_topic, 10)
        self.timer = self.create_timer(self.check_period_s, self._on_timer)

        self._last_warn_t = 0.0
        self.get_logger().info(
            "HH_260309 module checker started: module=%s health=%s (diagnostics handled by module diagnostic node, cfg=%s)"
            % (self.module_name, health_topic, diagnostics_topic)
        )

    def _collect_graph(self) -> Tuple[Set[str], Set[str], Set[str]]:
        node_names: Set[str] = set()
        for name, ns in self.get_node_names_and_namespaces():
            if ns == "/":
                node_names.add(_normalize_name(name))
            else:
                node_names.add(_normalize_name(f"{ns}/{name}"))

        topic_names: Set[str] = {name for name, _ in self.get_topic_names_and_types()}
        service_names: Set[str] = {name for name, _ in self.get_service_names_and_types()}
        return node_names, topic_names, service_names

    def _on_timer(self) -> None:
        node_names, topic_names, service_names = self._collect_graph()

        missing_nodes = [name for name in self.required_nodes if name not in node_names]
        missing_topics = [name for name in self.required_topics if name not in topic_names]
        missing_lifecycle = []
        for name in self.required_lifecycle_nodes:
            get_state_srv = f"{name}/get_state"
            if get_state_srv not in service_names:
                missing_lifecycle.append(name)

        level = ModuleHealth.OK
        message = "ok"
        if missing_nodes:
            level = ModuleHealth.ERROR
            message = "missing_nodes"
        elif missing_topics or missing_lifecycle:
            level = ModuleHealth.WARN
            message = "degraded"

        if level != ModuleHealth.OK:
            now_t = time.time()
            if now_t - self._last_warn_t >= self.warn_throttle_sec:
                self.get_logger().warn(
                    "HH_260309 %s checker: missing_nodes=%s missing_topics=%s missing_lifecycle=%s"
                    % (
                        self.module_name,
                        ",".join(missing_nodes) if missing_nodes else "-",
                        ",".join(missing_topics) if missing_topics else "-",
                        ",".join(missing_lifecycle) if missing_lifecycle else "-",
                    )
                )
                self._last_warn_t = now_t
        elif not self.publish_ok:
            return

        health = ModuleHealth()
        health.stamp = self.get_clock().now().to_msg()
        health.module_name = self.module_name
        health.level = int(level)
        health.message = message
        health.missing_nodes = missing_nodes
        health.missing_topics = missing_topics
        health.missing_lifecycle_nodes = missing_lifecycle
        self.pub_health.publish(health)


def main() -> None:
    rclpy.init()
    node = ModuleCheckerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
