#!/usr/bin/env python3
# HH_260327: Generic plugin-style API bridge backed by system diagnostics.

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from camrod_api.api_common import to_diag_level_bytes, to_diag_level_int


@dataclass
class ModuleSnapshot:
    """Runtime snapshot of each module's latest diagnostic state."""

    level: int = to_diag_level_int(DiagnosticStatus.WARN)
    stamp_sec: float = 0.0
    message: str = "no status yet"


class PluginApiBridgeNode(Node):
    """Bridge planning engage control and module diagnostics into plugin-style API endpoints."""

    # Initializes publishers/subscribers/services for the plugin API bridge.
    def __init__(self) -> None:
        super().__init__("plugin_api_bridge")

        self.api_set_engage_service = str(
            self.declare_parameter("api_set_engage_service", "/api/plugin/set/engage").value
        )
        self.api_set_operation_mode_service = str(
            self.declare_parameter(
                "api_set_operation_mode_service", "/api/plugin/set/operation_mode"
            ).value
        )
        self.api_set_auto_service = str(
            self.declare_parameter(
                "api_set_auto_service", "/api/plugin/set/operation_mode/auto"
            ).value
        )
        self.api_set_stop_service = str(
            self.declare_parameter(
                "api_set_stop_service", "/api/plugin/set/operation_mode/stop"
            ).value
        )
        self.api_get_engage_topic = str(
            self.declare_parameter("api_get_engage_topic", "/api/plugin/get/engage").value
        )
        self.api_ready_topic = str(
            self.declare_parameter("api_ready_topic", "/api/plugin/get/ready").value
        )
        self.api_operation_mode_topic = str(
            self.declare_parameter(
                "api_operation_mode_topic", "/api/plugin/get/operation_mode"
            ).value
        )
        self.api_module_states_topic = str(
            self.declare_parameter(
                "api_module_states_topic", "/api/plugin/get/module_states"
            ).value
        )
        self.api_ready_message_topic = str(
            self.declare_parameter(
                "api_ready_message_topic", "/api/plugin/get/ready_message"
            ).value
        )

        self.engage_command_topic = str(
            self.declare_parameter("engage_command_topic", "/planning/engage").value
        )
        self.engage_state_topic = str(
            self.declare_parameter("engage_state_topic", "/platform/drive_enabled").value
        )
        self.diagnostics_topic = str(
            self.declare_parameter("diagnostics_topic", "/diagnostics").value
        )

        self.required_modules = list(
            self.declare_parameter(
                "required_modules",
                ["map", "sensing", "localization", "planning", "platform", "perception"],
            ).value
        )
        self.status_stale_timeout_s = float(
            self.declare_parameter("status_stale_timeout_s", 3.0).value
        )
        self.warn_is_ready = bool(self.declare_parameter("warn_is_ready", True).value)
        self.publish_period_s = float(self.declare_parameter("publish_period_s", 0.5).value)

        self._engaged = False
        self._snapshots: Dict[str, ModuleSnapshot] = {
            name: ModuleSnapshot() for name in self.required_modules
        }

        self.pub_engage_api = self.create_publisher(Bool, self.api_get_engage_topic, 10)
        self.pub_ready_api = self.create_publisher(Bool, self.api_ready_topic, 10)
        self.pub_operation_mode_api = self.create_publisher(String, self.api_operation_mode_topic, 10)
        self.pub_module_states_api = self.create_publisher(
            DiagnosticArray, self.api_module_states_topic, 10
        )
        self.pub_ready_message = self.create_publisher(String, self.api_ready_message_topic, 10)
        self.pub_engage_cmd = self.create_publisher(Bool, self.engage_command_topic, 10)

        self.create_subscription(Bool, self.engage_state_topic, self._on_engage_state, 10)
        self.create_subscription(DiagnosticArray, self.diagnostics_topic, self._on_diagnostics, 10)

        self.create_service(SetBool, self.api_set_engage_service, self._on_set_engage)
        self.create_service(SetBool, self.api_set_operation_mode_service, self._on_set_operation_mode)
        self.create_service(Trigger, self.api_set_auto_service, self._on_change_to_auto)
        self.create_service(Trigger, self.api_set_stop_service, self._on_change_to_stop)
        self.create_timer(self.publish_period_s, self._on_timer)

        self.get_logger().info(
            "plugin_api_bridge ready: "
            f"set_engage={self.api_set_engage_service} "
            f"set_operation_mode={self.api_set_operation_mode_service} "
            f"get_ready={self.api_ready_topic} "
            f"engage_command_topic={self.engage_command_topic} "
            f"engage_state_topic={self.engage_state_topic} "
            f"diagnostics_topic={self.diagnostics_topic}"
        )

    # Tracks current platform engage state from platform module.
    def _on_engage_state(self, msg: Bool) -> None:
        self._engaged = bool(msg.data)

    # Resolves module category key from diagnostic payload fields.
    def _extract_category(self, status: DiagnosticStatus) -> str:
        for kv in status.values:
            if kv.key == "category" and kv.value:
                return str(kv.value)
        if status.name:
            return status.name.split("/")[0]
        if status.hardware_id:
            return str(status.hardware_id)
        return ""

    # Updates per-module snapshots from incoming /diagnostics stream.
    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        for status in msg.status:
            category = self._extract_category(status)
            if category not in self._snapshots:
                continue
            snap = self._snapshots[category]
            snap.level = to_diag_level_int(status.level)
            snap.stamp_sec = now_sec
            snap.message = status.message

    # Computes global readiness and a human-readable reason string.
    def _compute_ready(self) -> tuple[bool, str]:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        missing: List[str] = []
        stale: List[str] = []
        bad: List[str] = []

        error_level = to_diag_level_int(DiagnosticStatus.ERROR)
        warn_level = to_diag_level_int(DiagnosticStatus.WARN)

        for module in self.required_modules:
            snap = self._snapshots.get(module, ModuleSnapshot())
            if snap.stamp_sec <= 0.0:
                missing.append(module)
                continue
            if (now_sec - snap.stamp_sec) > self.status_stale_timeout_s:
                stale.append(module)
                continue
            if self.warn_is_ready:
                if snap.level >= error_level:
                    bad.append(module)
            elif snap.level >= warn_level:
                bad.append(module)

        if missing:
            return False, f"missing diagnostics: {','.join(missing)}"
        if stale:
            return False, f"stale diagnostics: {','.join(stale)}"
        if bad:
            return False, f"not ready modules: {','.join(bad)}"
        return True, "ready"

    # Publishes engage command to planning gate and returns trace message.
    def _forward_engage(self, desired: bool, source: str) -> str:
        cmd = Bool()
        cmd.data = desired
        self.pub_engage_cmd.publish(cmd)
        return (
            f"{source}: engage command forwarded to "
            f"{self.engage_command_topic}: {str(desired).lower()}"
        )

    # Handles explicit engage on/off API calls.
    def _on_set_engage(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        message = self._forward_engage(bool(request.data), "set_engage")
        response.success = True
        response.message = message
        self.get_logger().info(message)
        return response

    # Handles generic operation-mode requests mapped to engage boolean.
    def _on_set_operation_mode(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        message = self._forward_engage(bool(request.data), "set_operation_mode")
        response.success = True
        response.message = message
        self.get_logger().info(message)
        return response

    # Handles AUTO shortcut trigger request.
    def _on_change_to_auto(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = True
        response.message = self._forward_engage(True, "change_to_auto")
        self.get_logger().info(response.message)
        return response

    # Handles STOP shortcut trigger request.
    def _on_change_to_stop(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = True
        response.message = self._forward_engage(False, "change_to_stop")
        self.get_logger().info(response.message)
        return response

    # Builds module-state diagnostic array for API consumers.
    def _build_module_states(self) -> DiagnosticArray:
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        msg = DiagnosticArray()
        msg.header.stamp = now.to_msg()

        for module in self.required_modules:
            snap = self._snapshots.get(module, ModuleSnapshot())
            status = DiagnosticStatus()
            status.name = f"{module}/api_state"
            status.hardware_id = module
            status.values.append(KeyValue(key="category", value=module))

            if snap.stamp_sec <= 0.0:
                status.level = to_diag_level_bytes(DiagnosticStatus.WARN)
                status.message = "missing"
                status.values.append(KeyValue(key="state", value="missing"))
            elif (now_sec - snap.stamp_sec) > self.status_stale_timeout_s:
                status.level = to_diag_level_bytes(DiagnosticStatus.WARN)
                status.message = "stale"
                status.values.append(KeyValue(key="state", value="stale"))
            else:
                status.level = to_diag_level_bytes(snap.level)
                status.message = snap.message
                status.values.append(KeyValue(key="state", value="ok"))

            status.values.append(
                KeyValue(key="last_update_age_s", value=f"{max(now_sec - snap.stamp_sec, 0.0):.3f}")
            )
            msg.status.append(status)

        return msg

    # Derives API operation mode from engage and readiness states.
    def _compute_operation_mode(self, ready: bool) -> str:
        if self._engaged and ready:
            return "AUTO"
        if self._engaged and not ready:
            return "WAITING_FOR_READY"
        return "STOP"

    # Periodically publishes API status topics.
    def _on_timer(self) -> None:
        ready, ready_message = self._compute_ready()

        engage_msg = Bool()
        engage_msg.data = self._engaged
        self.pub_engage_api.publish(engage_msg)

        ready_msg = Bool()
        ready_msg.data = ready
        self.pub_ready_api.publish(ready_msg)

        mode_msg = String()
        mode_msg.data = self._compute_operation_mode(ready)
        self.pub_operation_mode_api.publish(mode_msg)

        text = String()
        text.data = ready_message
        self.pub_ready_message.publish(text)

        self.pub_module_states_api.publish(self._build_module_states())


# Spins the plugin API bridge node.
def main() -> None:
    rclpy.init()
    node = PluginApiBridgeNode()
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
