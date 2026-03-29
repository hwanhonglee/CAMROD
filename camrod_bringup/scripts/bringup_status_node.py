#!/usr/bin/env python3
# HH_260316-00:00 Bringup-level status publisher (status-only, no module message bus).

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from avg_msgs.msg import ModuleState
from status_msgs.msg import StatusArray, StatusStatus, KeyValue


# Implements `_diag_level` behavior.
def _diag_level(value: object) -> bytes:
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 1:
            return bytes(value)
        if len(value) > 1:
            return bytes([value[0]])
        return b"\x00"
    return bytes([int(value) & 0xFF])


# Implements `_to_bool` behavior.
def _to_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    return str(value).strip().lower() in ("1", "true", "yes", "on")


# Implements `_to_int_level` behavior.
def _to_int_level(value: object) -> int:
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 0:
            return int(ModuleState.OK)
        return int(value[0])
    return int(value)


@dataclass
class StampState:
    stamp: Time | None = None
    level: int = int(ModuleState.WARN)
    message: str = "no status yet"


class BringupStatusNode(Node):
    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("bringup_status")

        self.status_topic = str(
            self.declare_parameter("status_topic", "status").value
        )
        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.5).value)
        self.stale_timeout_s = float(self.declare_parameter("stale_timeout_s", 2.5).value)

        self.sim = _to_bool(self.declare_parameter("sim", True).value)
        self.rviz = _to_bool(self.declare_parameter("rviz", True).value)
        self.use_eskf = _to_bool(self.declare_parameter("use_eskf", True).value)
        self.enable_goal_replanner = _to_bool(
            self.declare_parameter("enable_goal_replanner", False).value
        )
        self.enable_state_machine = _to_bool(
            self.declare_parameter("enable_state_machine", False).value
        )
        self.enable_module_validators = _to_bool(
            self.declare_parameter("enable_module_validators", True).value
        )

        self.map_path = str(self.declare_parameter("map_path", "").value)
        self.origin_lat = float(self.declare_parameter("origin_lat", 0.0).value)
        self.origin_lon = float(self.declare_parameter("origin_lon", 0.0).value)
        self.origin_alt = float(self.declare_parameter("origin_alt", 0.0).value)

        self.source_status_topic = str(
            self.declare_parameter("source_status_topic", "/status_stream").value
        )

        self._diag_pub = self.create_publisher(StatusArray, self.status_topic, 10)

        self._state: Dict[str, StampState] = {
            "map": StampState(),
            "sensing": StampState(),
            "localization": StampState(),
            "planning": StampState(),
            "platform": StampState(),
            "perception": StampState(),
            "sensor_kit": StampState(),
            "system": StampState(),
        }

        self.create_subscription(
            StatusArray, self.source_status_topic, self._on_status, 20
        )
        self.create_timer(self.publish_period_s, self._on_timer)

        self.get_logger().info(
            "bringup_status: status=%s period=%.2fs"
            % (self.status_topic, self.publish_period_s)
        )

    # Implements `_extract_category` behavior.
    def _extract_category(self, status: StatusStatus) -> str:
        for kv in status.values:
            if kv.key == "category" and kv.value:
                return kv.value
        if status.name:
            return status.name.split("/")[0]
        if status.hardware_id:
            return status.hardware_id
        return ""

    # Implements `_on_status` behavior.
    def _on_status(self, msg: StatusArray) -> None:
        now = self.get_clock().now()
        for status in msg.status:
            category = self._extract_category(status)
            if category not in self._state:
                continue
            st = self._state[category]
            st.stamp = now
            st.level = _to_int_level(status.level)
            st.message = status.message

    # Implements `_module_ready` behavior.
    def _module_ready(self, now: Time, key: str) -> bool:
        st = self._state[key].stamp
        if st is None:
            return False
        age = (now - st).nanoseconds * 1e-9
        return age <= self.stale_timeout_s

    # Implements `_state` behavior.
    def _state(self, now: Time) -> ModuleState:
        out = ModuleState()
        out.stamp = now.to_msg()
        out.module_name = "bringup"

        missing = []
        for key in self._state:
            if not self._module_ready(now, key):
                st = self._state[key].stamp
                if st is None:
                    missing.append(key)
                else:
                    age = (now - st).nanoseconds * 1e-9
                    missing.append(f"{key}(stale:{age:.2f}s)")

        if missing:
            out.level = ModuleState.ERROR
            out.message = "missing/stale module status_stream"
            out.missing_topics = missing
        else:
            out.level = ModuleState.OK
            out.message = "all module status_stream statey"
        return out

    # Implements `_on_timer` behavior.
    def _on_timer(self) -> None:
        now = self.get_clock().now()
        state = self._state(now)

        diag = StatusArray()
        diag.header.stamp = now.to_msg()

        st = StatusStatus()
        st.name = "bringup/status"
        st.hardware_id = "bringup"
        st.level = _diag_level(state.level)
        st.message = state.message
        st.values.append(KeyValue(key="category", value="bringup"))
        st.values.append(KeyValue(key="missing_topics", value=",".join(state.missing_topics)))
        st.values.append(KeyValue(key="sim", value=str(self.sim).lower()))
        st.values.append(KeyValue(key="rviz", value=str(self.rviz).lower()))
        st.values.append(KeyValue(key="use_eskf", value=str(self.use_eskf).lower()))
        st.values.append(
            KeyValue(key="enable_goal_replanner", value=str(self.enable_goal_replanner).lower())
        )
        st.values.append(
            KeyValue(key="enable_state_machine", value=str(self.enable_state_machine).lower())
        )
        st.values.append(
            KeyValue(
                key="enable_module_validators", value=str(self.enable_module_validators).lower()
            )
        )
        st.values.append(KeyValue(key="map_path", value=self.map_path))
        st.values.append(KeyValue(key="origin_lat", value=f"{self.origin_lat:.8f}"))
        st.values.append(KeyValue(key="origin_lon", value=f"{self.origin_lon:.8f}"))
        st.values.append(KeyValue(key="origin_alt", value=f"{self.origin_alt:.3f}"))

        for key in sorted(self._state.keys()):
            st.values.append(
                KeyValue(key=f"{key}_ready", value=str(self._module_ready(now, key)).lower())
            )

        diag.status.append(st)
        self._diag_pub.publish(diag)


# Entry point for this executable.
def main() -> None:
    rclpy.init()
    node = BringupStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"bringup_status runtime exception: {exc}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
