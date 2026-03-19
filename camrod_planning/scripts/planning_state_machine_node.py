#!/usr/bin/env python3
# HH_260312-00:00 Planning state machine with keypoint mapping based on /diagnostic.

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
import yaml
from avg_msgs.srv import RequestGoalByKey
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


# Implements `_diag_level` behavior.
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
    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("planning_state_machine")

        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.health_diagnostic_topic = str(
            self.declare_parameter("health_diagnostic_topic", "/diagnostics").value
        )
        self.health_stale_timeout_s = float(
            self.declare_parameter("health_stale_timeout_s", 3.0).value
        )
        self.pose_topic = str(self.declare_parameter("pose_topic", "/planning/lanelet_pose").value)
        self.goal_topic = str(self.declare_parameter("goal_topic", "/planning/goal_pose").value)
        self.state_topic = str(
            self.declare_parameter("state_topic", "/planning/state_machine/state").value
        )
        self.estop_topic = str(
            self.declare_parameter("estop_topic", "/planning/state_machine/estop").value
        )
        self.return_topic = str(
            self.declare_parameter(
                "return_to_drop_zone_topic", "/planning/state_machine/return_to_drop_zone"
            ).value
        )
        self.return_goal_key = str(
            self.declare_parameter("return_goal_key", "drop_zone").value
        )
        self.goal_key_topic = str(
            self.declare_parameter(
                "goal_key_topic", "/planning/state_machine/goal_key"
            ).value
        )
        self.request_goal_service = str(
            self.declare_parameter(
                "request_goal_service", "/planning/state_machine/request_goal"
            ).value
        )
        self.diag_topic = str(
            # HH_260311-00:00 Single consolidated ROS diagnostic stream.
            self.declare_parameter("diagnostics_topic", "/diagnostics").value
        )
        self.keypoints_yaml = str(self.declare_parameter("keypoints_yaml", "").value)
        self.camping_sites_yaml = str(
            self.declare_parameter("camping_sites_yaml", "").value
        )
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

        self.last_health_stamp: Optional[rclpy.time.Time] = None
        self.module_levels: Dict[str, int] = {}
        self.last_pose: Optional[PoseStamped] = None
        self.last_manual_goal: Optional[PoseStamped] = None
        self.active_goal: Optional[PoseStamped] = None
        self.active_goal_source: str = "none"
        self.state: str = "INIT"
        self.startup_goal_sent = False
        # HH_260309-00:00 Warn recovery goal should be one-shot per WARN epoch.
        self.warn_goal_sent = False
        # HH_260313-00:00 Manual return request latch (drop-zone recovery button).
        self.return_requested = False
        self._last_return_cmd = False
        self.prev_health_level: Optional[int] = None
        self._ok_level = self._status_level_int(DiagnosticStatus.OK)
        self._warn_level = self._status_level_int(DiagnosticStatus.WARN)
        self._error_level = self._status_level_int(DiagnosticStatus.ERROR)

        self._last_goal_publish_time = self.get_clock().now()
        self._last_self_goal: Optional[PoseStamped] = None

        self.pub_goal = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.pub_state = self.create_publisher(String, self.state_topic, 10)
        self.pub_estop = self.create_publisher(Bool, self.estop_topic, 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, self.diag_topic, 10)

        self.create_subscription(
            DiagnosticArray, self.health_diagnostic_topic, self._on_health, 10
        )
        self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.create_subscription(Bool, self.return_topic, self._on_return_to_drop_zone, 10)
        self.create_subscription(String, self.goal_key_topic, self._on_goal_key_request, 10)
        self.create_service(
            RequestGoalByKey,
            self.request_goal_service,
            self._on_goal_key_service,
        )

        period = 1.0 / max(0.5, self.loop_rate_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            "planning_state_machine: "
            f"health={self.health_diagnostic_topic} "
            f"pose={self.pose_topic} "
            f"goal={self.goal_topic} "
            f"return={self.return_topic} "
            f"goal_key_topic={self.goal_key_topic} "
            f"request_goal_service={self.request_goal_service} "
            f"keypoints={','.join(sorted(self.keypoints.keys()))}"
        )

    # Implements `_load_keypoints` behavior.
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

        # Allow direct reuse of camping-site map export when it is colocated
        # with keypoints YAML.
        self._merge_camping_sites(data)

        # Optionally merge camping sites from a dedicated YAML file exported by
        # map area exporter.
        if self.camping_sites_yaml:
            if not os.path.exists(self.camping_sites_yaml):
                self.get_logger().warn(
                    f"camping_sites_yaml not found: {self.camping_sites_yaml}"
                )
            else:
                try:
                    with open(self.camping_sites_yaml, "r", encoding="utf-8") as f:
                        camping_data = yaml.safe_load(f) or {}
                    self._merge_camping_sites(camping_data)
                except Exception as e:  # noqa: BLE001
                    self.get_logger().error(
                        f"failed to read camping_sites_yaml {self.camping_sites_yaml}: {e}"
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

    # Implements `_merge_camping_sites` behavior.
    def _merge_camping_sites(self, data: dict) -> None:
        # Each camping site entry becomes a keypoint:
        # - prefer semantic "type" name (e.g. camping_site_1)
        # - fallback to deterministic camping_site_<index>
        raw_sites = data.get("camping_sites", [])
        if not isinstance(raw_sites, list):
            return
        for index, site in enumerate(raw_sites, start=1):
            if not isinstance(site, dict):
                continue
            key_name = str(site.get("type", "")).strip()
            if not key_name:
                key_name = f"camping_site_{index}"
            if key_name in self.keypoints:
                continue
            self.keypoints[key_name] = Keypoint(
                name=key_name,
                frame_id=str(site.get("frame_id", "map")),
                x=float(site.get("x", 0.0)),
                y=float(site.get("y", 0.0)),
                z=float(site.get("z", 0.0)),
            )

    @staticmethod
    # Implements `_status_level_int` behavior.
    def _status_level_int(level_value: object) -> int:
        if isinstance(level_value, (bytes, bytearray)):
            if len(level_value) > 0:
                return int(level_value[0])
            return int(DiagnosticStatus.OK)
        return int(level_value)

    @staticmethod
    # Implements `_extract_module` behavior.
    def _extract_module(st: DiagnosticStatus) -> str:
        for kv in st.values:
            if kv.key == "category" and kv.value:
                return kv.value
        if st.name:
            parts = [p for p in st.name.strip("/").split("/") if p]
            if parts:
                return parts[0]
        if st.hardware_id:
            return st.hardware_id
        return ""

    # Implements `_on_health` behavior.
    def _on_health(self, msg: DiagnosticArray) -> None:
        self.last_health_stamp = self.get_clock().now()
        current_levels: Dict[str, int] = {}
        for st in msg.status:
            module = self._extract_module(st)
            if not module:
                continue
            level = self._status_level_int(st.level)
            prev = current_levels.get(module, self._ok_level)
            if level > prev:
                current_levels[module] = level
        if current_levels:
            self.module_levels = current_levels

    # Implements `_on_pose` behavior.
    def _on_pose(self, msg: PoseStamped) -> None:
        self.last_pose = msg

    @staticmethod
    # Implements `_dist_xy` behavior.
    def _dist_xy(a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return math.hypot(dx, dy)

    # Implements `_on_goal` behavior.
    def _on_goal(self, msg: PoseStamped) -> None:
        # Ignore immediate self-loopback on our own auto goal publish.
        if self._last_self_goal is not None:
            if self._dist_xy(self._last_self_goal, msg) < 0.02:
                return
        self.last_manual_goal = msg
        self.active_goal = msg
        self.active_goal_source = "manual"
        self.startup_goal_sent = True
        # HH_260313-00:00 A new manual goal cancels pending return-request latch.
        self.return_requested = False

    # Implements `_on_return_to_drop_zone` behavior.
    def _on_return_to_drop_zone(self, msg: Bool) -> None:
        # HH_260313-00:00 Rising-edge trigger only.
        if msg.data and not self._last_return_cmd:
            self.return_requested = True
        self._last_return_cmd = bool(msg.data)

    # Implements `_health_level` behavior.
    def _health_level(self) -> int:
        if self.last_health_stamp is None:
            # HH_260312-00:00 Do not force WARN before any diagnostic arrives.
            return self._ok_level

        age = (self.get_clock().now() - self.last_health_stamp).nanoseconds / 1e9
        if age > self.health_stale_timeout_s:
            return self._warn_level

        if not self.module_levels:
            return self._ok_level
        return max(self.module_levels.values())

    # Implements `_publish_auto_goal` behavior.
    def _publish_auto_goal(self, key_name: str, source: str, force: bool = False) -> bool:
        kp = self.keypoints.get(key_name)
        if kp is None:
            self.get_logger().warn(f"keypoint '{key_name}' not found")
            return False
        now = self.get_clock().now()
        dt = (now - self._last_goal_publish_time).nanoseconds / 1e9
        if not force and dt < self.min_goal_publish_interval_s:
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
        self.startup_goal_sent = True
        return True

    # Implements `_on_goal_key_request` behavior.
    def _on_goal_key_request(self, msg: String) -> None:
        key_name = msg.data.strip()
        if not key_name:
            self.get_logger().warn("received empty goal-key request on topic")
            return
        if self._publish_auto_goal(key_name, f"key_topic:{key_name}", force=True):
            self.return_requested = False
            self.warn_goal_sent = False
            self.get_logger().info(f"published goal from key request topic: {key_name}")

    # Implements `_on_goal_key_service` behavior.
    def _on_goal_key_service(
        self, request: RequestGoalByKey.Request, response: RequestGoalByKey.Response
    ) -> RequestGoalByKey.Response:
        key_name = request.key.strip()
        if not key_name:
            response.accepted = False
            response.message = "empty key"
            return response

        keypoint = self.keypoints.get(key_name)
        if keypoint is None:
            response.accepted = False
            response.message = f"unknown key: {key_name}"
            return response

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = keypoint.frame_id
        goal_pose.pose.position.x = keypoint.x
        goal_pose.pose.position.y = keypoint.y
        goal_pose.pose.position.z = keypoint.z
        goal_pose.pose.orientation.w = 1.0

        published = self._publish_auto_goal(key_name, f"key_service:{key_name}", force=True)
        response.accepted = bool(published)
        if published:
            self.return_requested = False
            self.warn_goal_sent = False
            response.message = f"goal published for key: {key_name}"
            response.goal_pose = goal_pose
            self.get_logger().info(f"published goal from key request service: {key_name}")
        else:
            response.message = f"goal publish throttled for key: {key_name}"
        return response

    # Implements `_goal_reached` behavior.
    def _goal_reached(self) -> bool:
        if self.last_pose is None or self.active_goal is None:
            return False
        if self.last_pose.header.frame_id and self.active_goal.header.frame_id:
            if self.last_pose.header.frame_id != self.active_goal.header.frame_id:
                return False
        return self._dist_xy(self.last_pose, self.active_goal) <= self.goal_reached_distance_m

    # Implements `_publish_state_outputs` behavior.
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
        st.values.append(KeyValue(key="return_requested", value=str(self.return_requested).lower()))
        st.values.append(KeyValue(key="return_goal_key", value=self.return_goal_key))
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

    # Implements `_tick` behavior.
    def _tick(self) -> None:
        if not self.enabled:
            return

        level = self._health_level()
        estop = False
        if self.prev_health_level != self._warn_level and level == self._warn_level:
            self.warn_goal_sent = False
        if level != self._warn_level:
            self.warn_goal_sent = False

        if level >= self._error_level:
            self.state = "ERROR_STOP"
            estop = self.auto_estop_on_error
        elif self.return_requested:
            self.state = "RETURNING"
            if self._publish_auto_goal(self.return_goal_key, "return_request"):
                self.return_requested = False
                self.startup_goal_sent = True
                self.warn_goal_sent = False
        elif level == self._warn_level:
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


# Entry point for this executable.
def main() -> None:
    rclpy.init()
    node = PlanningStateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"planning_state_machine runtime exception: {e}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
