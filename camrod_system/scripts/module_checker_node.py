#!/usr/bin/env python3
# Generic module checker: per-module graph/lifecycle diagnostic publisher.

from __future__ import annotations

import time
from typing import List, Set, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# Implements `_normalize_name` behavior.
def _normalize_name(name: str) -> str:
    if not name:
        return name
    return name if name.startswith("/") else f"/{name}"


# Implements `_diag_level` behavior.
def _diag_level(value: object) -> bytes:
    # Humble DiagnosticStatus level field expects a one-byte value.
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 1:
            return bytes(value)
        if len(value) > 1:
            return bytes([value[0]])
        return b"\x00"
    return bytes([int(value) & 0xFF])


class ModuleCheckerNode(Node):
    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("module_checker")

        self.module_name = self.declare_parameter("module_name", "module").value
        self.check_period_s = float(self.declare_parameter("check_period_s", 1.0).value)
        self.warn_throttle_sec = float(self.declare_parameter("warn_throttle_sec", 2.0).value)
        self.startup_grace_sec = float(self.declare_parameter("startup_grace_sec", 6.0).value)
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

        self.diagnostic_topic = str(
            self.declare_parameter("diagnostic_topic", "/diagnostics").value
        )
        self.status_name = str(
            self.declare_parameter(
                "status_name", f"{self.module_name}/checker"
            ).value
        )

        self.pub_diag = self.create_publisher(DiagnosticArray, self.diagnostic_topic, 10)
        self.timer = self.create_timer(self.check_period_s, self._on_timer)

        self._last_warn_t = 0.0
        self._start_t = time.time()
        self.get_logger().info(
            "module checker started: module=%s diagnostic=%s status=%s"
            % (self.module_name, self.diagnostic_topic, self.status_name)
        )

    # Implements `_collect_graph` behavior.
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

    # Implements `_on_timer` behavior.
    def _on_timer(self) -> None:
        node_names, topic_names, service_names = self._collect_graph()
        in_startup_grace = (time.time() - self._start_t) < self.startup_grace_sec

        missing_nodes = [name for name in self.required_nodes if name not in node_names]
        missing_topics = [name for name in self.required_topics if name not in topic_names]
        missing_lifecycle = []
        for name in self.required_lifecycle_nodes:
            get_state_srv = f"{name}/get_state"
            if get_state_srv not in service_names:
                missing_lifecycle.append(name)

        level = DiagnosticStatus.OK
        message = "ok"
        if in_startup_grace:
            level = DiagnosticStatus.OK
            message = "starting"
        elif missing_nodes:
            level = DiagnosticStatus.ERROR
            message = "missing_nodes"
        elif missing_topics or missing_lifecycle:
            level = DiagnosticStatus.WARN
            message = "degraded"

        if level != DiagnosticStatus.OK and not in_startup_grace:
            now_t = time.time()
            if now_t - self._last_warn_t >= self.warn_throttle_sec:
                self.get_logger().warn(
                    "%s checker: missing_nodes=%s missing_topics=%s missing_lifecycle=%s"
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

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = self.status_name
        st.hardware_id = self.module_name
        st.level = _diag_level(level)
        st.message = message
        st.values.append(KeyValue(key="category", value=self.module_name))
        st.values.append(KeyValue(key="missing_nodes", value=",".join(missing_nodes)))
        st.values.append(KeyValue(key="missing_topics", value=",".join(missing_topics)))
        st.values.append(
            KeyValue(
                key="missing_lifecycle_nodes",
                value=",".join(missing_lifecycle),
            )
        )
        diag.status.append(st)
        self.pub_diag.publish(diag)


# Entry point for this executable.
def main() -> None:
    rclpy.init()
    node = ModuleCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"module_checker runtime exception: {e}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
