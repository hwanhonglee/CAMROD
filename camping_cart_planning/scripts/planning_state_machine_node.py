#!/usr/bin/env python3
# HH_260309-00:00 Planning state machine with keypoint mapping based on /system/diagnostic.

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
import yaml
from avg_msgs.msg import ModuleHealth, SystemDiagnostic
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


def _diag_level(value: object) -> bytes:
    # HH_260311-00:00 Humble DiagnosticStatus constants may be bytes; normalize safely.
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 1:
            return bytes(value)
        if len(value) > 1:
            return bytes([value[0]])
        return b"\x00"
    return bytes([int(value) & 0xFF])


@dataclass
class Keypoint:
    name: str
    frame_id: str
    x: float
    y: float
    z: float


class PlanningStateMachineNode(Node):
    def __init__(self) -> None:
        super().__init__("planning_state_machine")

        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.health_diagnostic_topic = str(
            self.declare_parameter("health_diagnostic_topic", "/system/diagnostic").value
        )
        self.pose_topic = str(self.declare_parameter("pose_topic", "/planning/lanelet_pose").value)
        self.goal_topic = str(self.declare_parameter("goal_topic", "/planning/goal_pose").value)
        self.state_topic = str(
            self.declare_parameter("state_topic", "/planning/state_machine/state").value
        )
        self.estop_topic = str(
            self.declare_parameter("estop_topic", "/planning/state_machine/estop").value
        )
        self.diag_topic = str(
            self.declare_parameter("diagnostics_topic", "/planning/diagnostic").value
        )
        self.keypoints_yaml = str(self.declare_parameter("keypoints_yaml", "").value)
        self.startup_goal_key = str(self.declare_parameter("startup_goal_key", "drop_zone").value)
        self.warn_goal_key = str(self.declare_parameter("warn_goal_key", "garage").value)
        self.auto_startup_goal = bool(self.declare_parameter("auto_startup_goal", True).value)
        self.auto_warn_recovery_goal = bool(
            self.declare_parameter("auto_warn_recovery_goal", True).value
        )
        self.auto_estop_on_error = bool(self.declare_parameter("auto_estop_on_error", True).value)
        self.min_goal_publish_interval_s = float(
            self.declare_parameter("min_goal_publish_interval_s", 1.0).value
        )
        self.goal_reached_distance_m = float(
            self.declare_parameter("goal_reached_distance_m", 0.8).value
        )
        self.loop_rate_hz = float(self.declare_parameter("loop_rate_hz", 5.0).value)

        self.keypoints: Dict[str, Keypoint] = {}
        self._load_keypoints()

        self.last_health: Optional[SystemDiagnostic] = None
        self.last_pose: Optional[PoseStamped] = None
        self.last_manual_goal: Optional[PoseStamped] = None
        self.active_goal: Optional[PoseStamped] = None
        self.active_goal_source: str = "none"
        self.state: str = "INIT"
        self.startup_goal_sent = False
        # HH_260309-00:00 Warn recovery goal should be one-shot per WARN epoch.
        self.warn_goal_sent = False
        self.prev_health_level: Optional[int] = None

        self._last_goal_publish_time = self.get_clock().now()
        self._last_self_goal: Optional[PoseStamped] = None

        self.pub_goal = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.pub_state = self.create_publisher(String, self.state_topic, 10)
        self.pub_estop = self.create_publisher(Bool, self.estop_topic, 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, self.diag_topic, 10)

        self.create_subscription(
            SystemDiagnostic, self.health_diagnostic_topic, self._on_health, 10
        )
        self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)

        period = 1.0 / max(0.5, self.loop_rate_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            "planning_state_machine: "
            f"health={self.health_diagnostic_topic} "
            f"pose={self.pose_topic} "
            f"goal={self.goal_topic} "
            f"keypoints={','.join(sorted(self.keypoints.keys()))}"
        )

    def _load_keypoints(self) -> None:
        self.keypoints.clear()
        if not self.keypoints_yaml:
            return
        if not os.path.exists(self.keypoints_yaml):
            self.get_logger().warn(f"keypoints_yaml not found: {self.keypoints_yaml}")
            return
        try:
            with open(self.keypoints_yaml, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f"failed to read keypoints_yaml {self.keypoints_yaml}: {e}"
            )
            return

        raw_kp = data.get("keypoints", {})
        if isinstance(raw_kp, dict):
            for name, v in raw_kp.items():
                if not isinstance(v, dict):
                    continue
                frame_id = str(v.get("frame_id", "map"))
                self.keypoints[name] = Keypoint(
                    name=name,
                    frame_id=frame_id,
                    x=float(v.get("x", 0.0)),
                    y=float(v.get("y", 0.0)),
                    z=float(v.get("z", 0.0)),
                )

        # Fallback: allow direct reuse of map drop_zones.yaml structure.
        if "drop_zone" not in self.keypoints:
            dz = data.get("drop_zones", [])
            if isinstance(dz, list) and dz:
                first = dz[0]
                if isinstance(first, dict):
                    self.keypoints["drop_zone"] = Keypoint(
                        name="drop_zone",
                        frame_id="map",
                        x=float(first.get("x", 0.0)),
                        y=float(first.get("y", 0.0)),
                        z=float(first.get("z", 0.0)),
                    )

        # If garage is missing, keep a deterministic fallback.
        if "garage" not in self.keypoints and "drop_zone" in self.keypoints:
            dz = self.keypoints["drop_zone"]
            self.keypoints["garage"] = Keypoint(
                name="garage",
                frame_id=dz.frame_id,
                x=dz.x,
                y=dz.y,
                z=dz.z,
            )

    def _on_health(self, msg: SystemDiagnostic) -> None:
        self.last_health = msg

    def _on_pose(self, msg: PoseStamped) -> None:
        self.last_pose = msg

    @staticmethod
    def _dist_xy(a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return math.hypot(dx, dy)

    def _on_goal(self, msg: PoseStamped) -> None:
        # Ignore immediate self-loopback on our own auto goal publish.
        if self._last_self_goal is not None:
            if self._dist_xy(self._last_self_goal, msg) < 0.02:
                return
        self.last_manual_goal = msg
        self.active_goal = msg
        self.active_goal_source = "manual"
        self.startup_goal_sent = True

    def _health_level(self) -> int:
        if self.last_health is None:
            # HH_260309-00:00 Do not force WARN policy before system summary is available.
            return ModuleHealth.OK
        worst = ModuleHealth.OK
        for m in self.last_health.modules:
            if m.level > worst:
                worst = m.level
        if not self.last_health.system_ok and worst < ModuleHealth.WARN:
            worst = ModuleHealth.WARN
        return worst

    def _publish_auto_goal(self, key_name: str, source: str) -> bool:
        kp = self.keypoints.get(key_name)
        if kp is None:
            self.get_logger().warn(f"keypoint '{key_name}' not found")
            return False
        now = self.get_clock().now()
        dt = (now - self._last_goal_publish_time).nanoseconds / 1e9
        if dt < self.min_goal_publish_interval_s:
            return False

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = kp.frame_id
        msg.pose.position.x = kp.x
        msg.pose.position.y = kp.y
        msg.pose.position.z = kp.z
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)

        self._last_self_goal = msg
        self._last_goal_publish_time = now
        self.active_goal = msg
        self.active_goal_source = source
        return True

    def _goal_reached(self) -> bool:
        if self.last_pose is None or self.active_goal is None:
            return False
        if self.last_pose.header.frame_id and self.active_goal.header.frame_id:
            if self.last_pose.header.frame_id != self.active_goal.header.frame_id:
                return False
        return self._dist_xy(self.last_pose, self.active_goal) <= self.goal_reached_distance_m

    def _publish_state_outputs(self, estop: bool) -> None:
        s = String()
        s.data = self.state
        self.pub_state.publish(s)

        b = Bool()
        b.data = bool(estop)
        self.pub_estop.publish(b)

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = "planning/state_machine"
        st.hardware_id = "planning"
        st.level = _diag_level(DiagnosticStatus.OK)
        if estop:
            st.level = _diag_level(DiagnosticStatus.ERROR)
            st.message = "estop_asserted"
        elif self.state.startswith("WARN"):
            st.level = _diag_level(DiagnosticStatus.WARN)
            st.message = "warn_recovery"
        else:
            st.message = self.state.lower()
        st.values.append(KeyValue(key="state", value=self.state))
        st.values.append(KeyValue(key="active_goal_source", value=self.active_goal_source))
        st.values.append(
            KeyValue(
                key="active_goal_xy",
                value=(
                    "none"
                    if self.active_goal is None
                    else f"{self.active_goal.pose.position.x:.2f},{self.active_goal.pose.position.y:.2f}"
                ),
            )
        )
        diag.status.append(st)
        self.pub_diag.publish(diag)

    def _tick(self) -> None:
        if not self.enabled:
            return

        level = self._health_level()
        estop = False
        if self.prev_health_level != ModuleHealth.WARN and level == ModuleHealth.WARN:
            self.warn_goal_sent = False
        if level != ModuleHealth.WARN:
            self.warn_goal_sent = False

        if level >= ModuleHealth.ERROR:
            self.state = "ERROR_STOP"
            estop = self.auto_estop_on_error
        elif level == ModuleHealth.WARN:
            self.state = "WARN_RECOVERY"
            if self.auto_warn_recovery_goal and not self.warn_goal_sent:
                if self._publish_auto_goal(self.warn_goal_key, "warn_recovery"):
                    self.warn_goal_sent = True
        else:
            # OK
            if self.auto_startup_goal and not self.startup_goal_sent:
                if self._publish_auto_goal(self.startup_goal_key, "startup"):
                    self.startup_goal_sent = True
            if self._goal_reached():
                self.state = "GOAL_REACHED"
            elif self.active_goal is not None:
                self.state = "RUNNING"
            else:
                self.state = "READY"

        self.prev_health_level = level
        self._publish_state_outputs(estop)


def main() -> None:
    rclpy.init()
    node = PlanningStateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
