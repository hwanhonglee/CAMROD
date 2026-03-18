#!/usr/bin/env python3
# HH_260312-00:00 System-level diagnostic-only publisher.

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.node import Node

from avg_msgs.msg import AvgSystemMsgs, ModuleHealth, SystemDiagnostic
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# Implements `_diag_level` behavior.
def _diag_level(value: object) -> bytes:
    # HH_260312-00:00 Humble DiagnosticStatus level expects one-byte values.
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 1:
            return bytes(value)
        if len(value) > 1:
            return bytes([value[0]])
        return b"\x00"
    return bytes([int(value) & 0xFF])


# Implements `_to_int_level` behavior.
def _to_int_level(value: object) -> int:
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 0:
            return int(ModuleHealth.OK)
        return int(value[0])
    return int(value)


@dataclass
class ModuleSnapshot:
    level: int = int(ModuleHealth.OK)
    message: str = "no status yet"
    stamp_sec: float = 0.0


class SystemDiagnosticNode(Node):
    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("system_diagnostic")

        self.diagnostic_topic = str(
            self.declare_parameter("diagnostic_topic", "/diagnostics").value
        )
        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.5).value)
        self.stale_timeout_s = float(self.declare_parameter("stale_timeout_s", 2.0).value)
        self.source_diagnostic_topic = str(
            self.declare_parameter("source_diagnostic_topic", "/diagnostics").value
        )
        self.known_modules = list(
            self.declare_parameter(
                "known_modules",
                [
                    "map",
                    "sensing",
                    "localization",
                    "planning",
                    "platform",
                    "perception",
                    "sensor_kit",
                    "bringup",
                    "system",
                ],
            ).value
        )

        self._diag_pub = self.create_publisher(DiagnosticArray, self.diagnostic_topic, 10)

        self._snapshots: Dict[str, ModuleSnapshot] = {
            module: ModuleSnapshot(level=int(ModuleHealth.WARN), message="no status yet")
            for module in self.known_modules
        }

        self.create_subscription(
            DiagnosticArray,
            self.source_diagnostic_topic,
            self._on_diagnostic,
            10,
        )
        self.create_timer(self.publish_period_s, self._on_timer)

        self.get_logger().info(
            "system_diagnostic: diagnostic=%s period=%.2fs"
            % (self.diagnostic_topic, self.publish_period_s)
        )

    # Implements `_extract_category` behavior.
    def _extract_category(self, status: DiagnosticStatus) -> str:
        for kv in status.values:
            if kv.key == "category" and kv.value:
                return kv.value
        if status.name:
            return status.name.split("/")[0]
        if status.hardware_id:
            return status.hardware_id
        return ""

    # Implements `_on_diagnostic` behavior.
    def _on_diagnostic(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            category = self._extract_category(status)
            if not category:
                continue
            if category not in self._snapshots:
                self._snapshots[category] = ModuleSnapshot(
                    level=int(ModuleHealth.WARN),
                    message="discovered dynamically",
                )
            level = _to_int_level(status.level)
            snap = self._snapshots[category]
            snap.level = level
            snap.message = status.message
            snap.stamp_sec = self.get_clock().now().nanoseconds * 1e-9

    # Implements `_build_modules` behavior.
    def _build_modules(self, stamp_msg) -> List[ModuleHealth]:
        modules: List[ModuleHealth] = []
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        for module_name in sorted(self._snapshots.keys()):
            snap = self._snapshots[module_name]
            mh = ModuleHealth()
            mh.stamp = stamp_msg
            mh.module_name = module_name
            if snap.stamp_sec <= 0.0:
                mh.level = ModuleHealth.WARN
                mh.message = "no status yet"
                mh.missing_topics = [f"{module_name}/diagnostic"]
            elif (now_sec - snap.stamp_sec) > self.stale_timeout_s:
                mh.level = ModuleHealth.WARN
                mh.message = f"stale status ({now_sec - snap.stamp_sec:.2f}s)"
                mh.missing_topics = [f"{module_name}/diagnostic(stale)"]
            elif snap.level <= int(ModuleHealth.OK):
                mh.level = ModuleHealth.OK
                mh.message = snap.message
            elif snap.level == int(ModuleHealth.WARN):
                mh.level = ModuleHealth.WARN
                mh.message = snap.message
            else:
                mh.level = ModuleHealth.ERROR
                mh.message = snap.message
            modules.append(mh)
        return modules

    # Implements `_on_timer` behavior.
    def _on_timer(self) -> None:
        now_msg = self.get_clock().now().to_msg()
        modules = self._build_modules(now_msg)

        errors = [m for m in modules if int(m.level) >= int(ModuleHealth.ERROR)]
        warns = [m for m in modules if int(m.level) == int(ModuleHealth.WARN)]

        sys_diag = SystemDiagnostic()
        sys_diag.stamp = now_msg
        sys_diag.system_ok = len(errors) == 0
        if errors:
            sys_diag.message = f"error modules={len(errors)}"
        elif warns:
            sys_diag.message = f"warn modules={len(warns)}"
        else:
            sys_diag.message = "all modules healthy"
        sys_diag.modules = modules

        msg = AvgSystemMsgs()
        msg.stamp = now_msg
        msg.system_diagnostic = sys_diag
        msg.active_modules = [m.module_name for m in modules]
        msg.status_count = len(modules)

        msg.health.stamp = now_msg
        msg.health.module_name = "system"
        if errors:
            msg.health.level = ModuleHealth.ERROR
            msg.health.message = "one or more modules in ERROR"
            msg.health.missing_topics = [m.module_name for m in errors]
        elif warns:
            msg.health.level = ModuleHealth.WARN
            msg.health.message = "one or more modules in WARN"
            msg.health.missing_topics = [m.module_name for m in warns]
        else:
            msg.health.level = ModuleHealth.OK
            msg.health.message = "system healthy"

        diag = DiagnosticArray()
        diag.header.stamp = now_msg
        st = DiagnosticStatus()
        st.name = "system/diagnostic"
        st.hardware_id = "system"
        st.level = _diag_level(msg.health.level)
        st.message = msg.health.message
        st.values.append(KeyValue(key="category", value="system"))
        st.values.append(KeyValue(key="status_count", value=str(msg.status_count)))
        st.values.append(KeyValue(key="active_modules", value=",".join(msg.active_modules)))
        st.values.append(KeyValue(key="warn_modules", value=",".join(m.module_name for m in warns)))
        st.values.append(KeyValue(key="error_modules", value=",".join(m.module_name for m in errors)))
        diag.status.append(st)
        self._diag_pub.publish(diag)


# Entry point for this executable.
def main() -> None:
    rclpy.init()
    node = SystemDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"system_diagnostic runtime exception: {exc}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
