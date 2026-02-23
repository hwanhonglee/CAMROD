#!/usr/bin/env python3
# HH_260112 Basic system checker for bringup diagnostics.

import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_descriptor import ParameterDescriptor
from rclpy.parameter_type import ParameterType

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def _normalize_name(name: str) -> str:
    if not name:
        return name
    return name if name.startswith("/") else f"/{name}"


class SystemChecker(Node):
    def __init__(self):
        super().__init__("system_checker")
        self.check_period_s = self.declare_parameter("check_period_s", 1.0).value
        array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        self.declare_parameter("required_nodes", Parameter.Type.STRING_ARRAY, array_desc)
        self.declare_parameter("required_topics", Parameter.Type.STRING_ARRAY, array_desc)
        self.required_nodes = [
            _normalize_name(n)
            for n in self.get_parameter("required_nodes").get_parameter_value().string_array_value
        ]
        self.required_topics = [
            _normalize_name(t)
            for t in self.get_parameter("required_topics").get_parameter_value().string_array_value
        ]

        self.pub_diag = self.create_publisher(DiagnosticArray, "/system/diagnostics", 5)
        self._last_report = 0.0
        self._timer = self.create_timer(self.check_period_s, self._on_timer)
        self.get_logger().info("HH_260112 system checker started")

    def _on_timer(self):
        node_names = set()
        for name, ns in self.get_node_names_and_namespaces():
            if ns == "/":
                node_names.add(_normalize_name(name))
            else:
                node_names.add(_normalize_name(f"{ns}/{name}"))

        topic_names = {name for name, _ in self.get_topic_names_and_types()}

        missing_nodes = [n for n in self.required_nodes if n not in node_names]
        missing_topics = [t for t in self.required_topics if t not in topic_names]

        now = time.time()
        if missing_nodes or missing_topics:
            if now - self._last_report > 2.0:
                self.get_logger().warn(
                    "HH_260112 system check missing nodes=%s topics=%s",
                    ",".join(missing_nodes) if missing_nodes else "-",
                    ",".join(missing_topics) if missing_topics else "-",
                )
                self._last_report = now

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status.append(self._build_status("system_checker/nodes", missing_nodes))
        diag.status.append(self._build_status("system_checker/topics", missing_topics))
        self.pub_diag.publish(diag)

    @staticmethod
    def _build_status(name, missing):
        status = DiagnosticStatus()
        status.name = name
        if missing:
            status.level = DiagnosticStatus.WARN
            status.message = "missing"
            status.values.append(KeyValue(key="missing", value=",".join(missing)))
        else:
            status.level = DiagnosticStatus.OK
            status.message = "ok"
        return status


def main():
    rclpy.init()
    node = SystemChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
