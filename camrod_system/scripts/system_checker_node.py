#!/usr/bin/env python3
# Basic system checker for bringup diagnostics.

import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_descriptor import ParameterDescriptor
from rclpy.parameter_type import ParameterType

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# Implements `_diag_level` behavior.
def _diag_level(value):
    # Humble DiagnosticStatus constants may be bytes; normalize safely.
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 1:
            return bytes(value)
        if len(value) > 1:
            return bytes([value[0]])
        return b"\x00"
    return bytes([int(value) & 0xFF])


# Implements `_normalize_name` behavior.
def _normalize_name(name: str) -> str:
    if not name:
        return name
    return name if name.startswith("/") else f"/{name}"


class SystemChecker(Node):
    # Implements `__init__` behavior.
    def __init__(self):
        super().__init__("system_checker")
        self.check_period_s = self.declare_parameter("check_period_s", 1.0).value
        self.startup_grace_sec = float(self.declare_parameter("startup_grace_sec", 6.0).value)
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

        self.diagnostic_topic = self.declare_parameter(
            "diagnostic_topic", "/diagnostics"
        ).value
        self.pub_diag = self.create_publisher(DiagnosticArray, self.diagnostic_topic, 5)
        self._last_report = 0.0
        self._start_t = time.time()
        self._timer = self.create_timer(self.check_period_s, self._on_timer)
        self.get_logger().info(
            "system checker started: diagnostic=%s" % self.diagnostic_topic
        )

    # Implements `_on_timer` behavior.
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
        in_startup_grace = (time.time() - self._start_t) < self.startup_grace_sec

        now = time.time()
        if (missing_nodes or missing_topics) and not in_startup_grace:
            if now - self._last_report > 2.0:
                self.get_logger().warn(
                    "system check missing nodes=%s topics=%s"
                    % (
                        ",".join(missing_nodes) if missing_nodes else "-",
                        ",".join(missing_topics) if missing_topics else "-",
                    )
                )
                self._last_report = now

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status.append(self._build_status("system_checker/nodes", missing_nodes))
        diag.status.append(self._build_status("system_checker/topics", missing_topics))
        self.pub_diag.publish(diag)

    @staticmethod
    # Implements `_build_status` behavior.
    def _build_status(name, missing):
        status = DiagnosticStatus()
        status.name = name
        if missing:
            status.level = _diag_level(DiagnosticStatus.WARN)
            status.message = "missing"
            status.values.append(KeyValue(key="missing", value=",".join(missing)))
        else:
            status.level = _diag_level(DiagnosticStatus.OK)
            status.message = "ok"
        return status


# Entry point for this executable.
def main():
    rclpy.init()
    node = SystemChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"system_checker runtime exception: {e}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
