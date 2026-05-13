#!/usr/bin/env python3
# HH_260312-00:00 Planning state machine with keypoint mapping based on /status.

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
import yaml
from avg_msgs.srv import RequestGoalByKey
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

# Prefer status_msgs when available, but fall back to diagnostic_msgs.
# This keeps the state machine runnable even when status_msgs is removed.
try:
    from status_msgs.msg import KeyValue, StatusArray, StatusStatus
    _USING_STATUS_MSGS = True
except Exception:  # noqa: BLE001
    from diagnostic_msgs.msg import (
        DiagnosticArray as StatusArray,
        DiagnosticStatus as StatusStatus,
        KeyValue,
    )
    _USING_STATUS_MSGS = False


# Implements `_diag_level` behavior.
def _diag_level(value: object) -> object:
    # HH_260425-00:00 In Humble Python bindings, uint8 message fields are exposed
    # as 1-byte bytes objects. Normalize every input into exactly one byte.
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
        self.state_status_topic = str(
            self.declare_parameter("state_status_topic", "/diagnostics_agg").value
        )
        self.state_stale_timeout_s = float(
            self.declare_parameter("state_stale_timeout_s", 3.0).value
        )
        self.pose_topic = str(self.declare_parameter("pose_topic", "/planning/lanelet_pose").value)
        self.goal_topic = str(self.declare_parameter("goal_topic", "/planning/goal_pose").value)
        # HH_260319-00:00 Optional ROS-goal mirror topic for Nav2 BT input.
        # State-machine auto goals are published to both:
        # - goal_topic       : internal snapped-goal stream
        # - goal_topic_ros   : Nav2-facing goal stream (if configured)
        self.goal_topic_ros = str(
            self.declare_parameter("goal_topic_ros", "/planning/goal_pose_snapped_ros").value
        )
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
        self.recall_topic = str(
            self.declare_parameter(
                "camping_site_recall_topic",
                "/planning/state_machine/camping_site_recall",
            ).value
        )
        self.mission_source_topic = str(
            self.declare_parameter(
                "mission_source_topic",
                "/planning/state_machine/mission_source",
            ).value
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
            # HH_260311-00:00 Single consolidated ROS status stream.
            self.declare_parameter("status_stream_topic", "/status_stream").value
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
        # HH_260425: Optional site scenario behavior.
        # When enabled and a site goal (e.g. camping_site_*) is reached, the state machine
        # waits goal_reached_dwell_s and then automatically publishes return_goal_key.
        self.enable_auto_return_on_site_goal = bool(
            self.declare_parameter("enable_auto_return_on_site_goal", False).value
        )
        self.site_goal_key_prefix = str(
            self.declare_parameter("site_goal_key_prefix", "camping_site_").value
        )
        self.goal_reached_dwell_s = float(
            self.declare_parameter("goal_reached_dwell_s", 10.0).value
        )
        # HH_260425: Manual /goal_pose updates are matched to nearest known keypoint
        # for auto-return eligibility decisions.
        self.goal_key_match_distance_m = float(
            self.declare_parameter("goal_key_match_distance_m", 1.5).value
        )
        self.min_goal_publish_interval_s = float(
            self.declare_parameter("min_goal_publish_interval_s", 1.0).value
        )
        self.goal_reached_distance_m = float(
            self.declare_parameter("goal_reached_distance_m", 0.8).value
        )
        self.loop_rate_hz = float(self.declare_parameter("loop_rate_hz", 5.0).value)

        self.keypoints: Dict[str, Keypoint] = {}
        self._load_keypoints()

        self.last_state_stamp: Optional[rclpy.time.Time] = None
        self.module_levels: Dict[str, int] = {}
        self.last_pose: Optional[PoseStamped] = None
        self.last_manual_goal: Optional[PoseStamped] = None
        self.active_goal: Optional[PoseStamped] = None
        self.active_goal_source: str = "none"
        self.active_goal_key: str = ""
        self.state: str = "INIT"
        self.startup_goal_sent = False
        # HH_260309-00:00 Warn recovery goal should be one-shot per WARN epoch.
        self.warn_goal_sent = False
        # HH_260313-00:00 Manual return request latch (drop-zone recovery button).
        self.return_requested = False
        # HH_260425: Separate latch for camping-site recall → RECALLED state.
        self.recall_requested = False
        self.recall_target_key: str = ""   # site key to go to on recall (not return_goal_key)
        self._last_return_cmd = False
        self.prev_state_level: Optional[int] = None
        self._ok_level = self._status_level_int(StatusStatus.OK)
        self._warn_level = self._status_level_int(StatusStatus.WARN)
        self._error_level = self._status_level_int(StatusStatus.ERROR)

        self._last_goal_publish_time = self.get_clock().now()
        self._last_self_goal: Optional[PoseStamped] = None
        self._goal_reached_since: Optional[rclpy.time.Time] = None
        self._goal_reached_latched = False

        self.pub_goal = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.pub_goal_ros = None
        if self.goal_topic_ros and self.goal_topic_ros != self.goal_topic:
            self.pub_goal_ros = self.create_publisher(PoseStamped, self.goal_topic_ros, 10)
        self.pub_state = self.create_publisher(String, self.state_topic, 10)
        self.pub_estop = self.create_publisher(Bool, self.estop_topic, 10)
        self.pub_diag = self.create_publisher(StatusArray, self.diag_topic, 10)
        self.pub_mission_source = self.create_publisher(String, self.mission_source_topic, 10)

        self.create_subscription(
            StatusArray, self.state_status_topic, self._on_state, 10
        )
        self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self._on_goal, 10)
        self.create_subscription(Bool, self.return_topic, self._on_return_to_drop_zone, 10)
        self.create_subscription(String, self.recall_topic, self._on_camping_site_recall, 10)
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
            f"state={self.state_status_topic} "
            f"pose={self.pose_topic} "
            f"goal={self.goal_topic} "
            f"goal_ros={self.goal_topic_ros} "
            f"return={self.return_topic} "
            f"recall={self.recall_topic} "
            f"mission_source={self.mission_source_topic} "
            f"goal_key_topic={self.goal_key_topic} "
            f"request_goal_service={self.request_goal_service} "
            f"keypoints={','.join(sorted(self.keypoints.keys()))} "
            f"auto_return_on_site_goal={'true' if self.enable_auto_return_on_site_goal else 'false'} "
            f"dwell={self.goal_reached_dwell_s:.1f}s"
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
        # If recall_x/y/z are present, a separate "<name>_road" keypoint is registered
        # for recall navigation (nearest lanelet road position, not area centroid).
        raw_sites = data.get("camping_sites", [])
        if not isinstance(raw_sites, list):
            return
        for index, site in enumerate(raw_sites, start=1):
            if not isinstance(site, dict):
                continue
            key_name = str(site.get("type", "")).strip()
            if not key_name:
                key_name = f"camping_site_{index}"
            frame_id = str(site.get("frame_id", "map"))
            if key_name not in self.keypoints:
                self.keypoints[key_name] = Keypoint(
                    name=key_name,
                    frame_id=frame_id,
                    x=float(site.get("x", 0.0)),
                    y=float(site.get("y", 0.0)),
                    z=float(site.get("z", 0.0)),
                )
            road_key = f"{key_name}_road"
            if road_key not in self.keypoints and "recall_x" in site:
                self.keypoints[road_key] = Keypoint(
                    name=road_key,
                    frame_id=frame_id,
                    x=float(site["recall_x"]),
                    y=float(site["recall_y"]),
                    z=float(site.get("recall_z", 0.0)),
                )

    @staticmethod
    # Implements `_status_level_int` behavior.
    def _status_level_int(level_value: object) -> int:
        if isinstance(level_value, (bytes, bytearray)):
            if len(level_value) > 0:
                return int(level_value[0])
            return int(StatusStatus.OK)
        return int(level_value)

    @staticmethod
    # Implements `_extract_module` behavior.
    def _extract_module(st: StatusStatus) -> str:
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

    # Implements `_on_state` behavior.
    def _on_state(self, msg: StatusArray) -> None:
        self.last_state_stamp = self.get_clock().now()
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
        prev_source = self.active_goal_source
        # Ignore immediate self-loopback on our own auto goal publish.
        if self._last_self_goal is not None:
            if self._dist_xy(self._last_self_goal, msg) < 0.02:
                return
        self.last_manual_goal = msg
        self.active_goal = msg
        self.active_goal_source = "manual"
        matched_key = self._match_goal_key(msg)
        if matched_key:
            self.active_goal_key = matched_key
        elif not prev_source.startswith("key_"):
            self.active_goal_key = ""
        self.startup_goal_sent = True
        self._goal_reached_since = None
        self._goal_reached_latched = False
        # HH_260313-00:00 A new manual goal cancels pending return/recall latches.
        self.return_requested = False
        self.recall_requested = False
        self.recall_target_key = ""
        # HH_260513: Always relay manual goal to Nav2 so the latest goal preempts any
        # in-flight navigation, regardless of prior state (GOAL_REACHED or not).
        # This ensures heading-mismatch recovery and RViz '2D Nav Goal' preemption both work.
        if self.pub_goal_ros is not None:
            self.pub_goal_ros.publish(msg)
            self.get_logger().info(
                "manual goal relayed to Nav2: "
                f"xy=({msg.pose.position.x:.2f},{msg.pose.position.y:.2f})"
            )

    # Implements `_on_return_to_drop_zone` behavior.
    def _on_return_to_drop_zone(self, msg: Bool) -> None:
        # HH_260319-00:00 Treat any `true` command as a valid return request.
        # This avoids missed triggers when upstream does not publish a falling edge.
        if msg.data:
            self.return_requested = True
            # HH_260319-00:00 Return request must work even when state level is ERROR.
            # Publish return goal immediately on rising edge; if throttled/failed,
            # keep `return_requested` latched so `_tick` can retry.
            if self._publish_auto_goal(self.return_goal_key, "return_request", force=True):
                self.return_requested = False
                self.startup_goal_sent = True
                self.warn_goal_sent = False
                self.get_logger().info(
                    f"published return goal from topic trigger: {self.return_goal_key}"
                )
        self._last_return_cmd = bool(msg.data)

    def _on_camping_site_recall(self, msg: String) -> None:
        site_name = msg.data.strip()

        site_kp = self.keypoints.get(site_name) if site_name else None
        if site_name and site_kp is None:
            self.get_logger().warn(
                f"camping_site_recall: unknown site '{site_name}', falling back to {self.return_goal_key}"
            )

        # Prefer the lanelet road-snap position for recall (cargo blocks area centroid entry).
        # Falls back to area centroid, then return_goal_key if nothing is known.
        road_key = f"{site_name}_road" if site_name else ""
        if road_key and road_key in self.keypoints:
            target_key = road_key
        elif site_kp is not None:
            target_key = site_name
        else:
            target_key = self.return_goal_key

        nearest_info = "lanelet_pose=unknown"
        if self.last_pose is not None:
            lx = self.last_pose.pose.position.x
            ly = self.last_pose.pose.position.y
            nearest_info = f"nearest_lanelet=({lx:.2f},{ly:.2f})"
            if site_kp is not None:
                d = math.hypot(lx - site_kp.x, ly - site_kp.y)
                nearest_info += f" dist_to_site={d:.2f}m"

        self.get_logger().info(
            f"camping_site_recall from '{site_name or '(unspecified)'}': "
            f"{nearest_info} -> go to '{target_key}' then auto-return to '{self.return_goal_key}'"
        )

        self.recall_target_key = target_key
        self.recall_requested = True
        if self._publish_auto_goal(target_key, f"recall:{site_name or 'unspecified'}", force=True):
            self.recall_requested = False
            self.startup_goal_sent = True
            self.warn_goal_sent = False

    # Implements `_state_level` behavior.
    def _state_level(self) -> int:
        if self.last_state_stamp is None:
            # HH_260312-00:00 Do not force WARN before any status arrives.
            return self._ok_level

        age = (self.get_clock().now() - self.last_state_stamp).nanoseconds / 1e9
        if age > self.state_stale_timeout_s:
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
        if self.pub_goal_ros is not None:
            self.pub_goal_ros.publish(msg)

        self._last_self_goal = msg
        self._last_goal_publish_time = now
        self.active_goal = msg
        self.active_goal_source = source
        self.active_goal_key = key_name
        self.startup_goal_sent = True
        self._goal_reached_since = None
        self._goal_reached_latched = False
        return True

    # Matches a pose to the nearest known keypoint name within threshold.
    def _match_goal_key(self, goal: PoseStamped) -> str:
        best_name = ""
        best_dist = float("inf")
        goal_frame = str(goal.header.frame_id).strip()
        for name, kp in self.keypoints.items():
            kp_frame = str(kp.frame_id).strip()
            if goal_frame and kp_frame and goal_frame != kp_frame:
                continue
            d = math.hypot(goal.pose.position.x - kp.x, goal.pose.position.y - kp.y)
            if d < best_dist:
                best_dist = d
                best_name = name
        if best_name and best_dist <= self.goal_key_match_distance_m:
            return best_name
        return ""

    # Returns true if the active goal is a site goal candidate.
    def _is_active_site_goal(self) -> bool:
        if not self.active_goal_key:
            return False
        return self.active_goal_key.startswith(self.site_goal_key_prefix)

    # Implements `_on_goal_key_request` behavior.
    def _on_goal_key_request(self, msg: String) -> None:
        key_name = msg.data.strip()
        if not key_name:
            self.get_logger().warn("received empty goal-key request on topic")
            return
        if self._publish_auto_goal(key_name, f"key_topic:{key_name}", force=True):
            self.return_requested = False
            self.recall_requested = False
            self.recall_target_key = ""
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
            self.recall_requested = False
            self.recall_target_key = ""
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

        ms = String()
        ms.data = self.active_goal_source
        self.pub_mission_source.publish(ms)

        b = Bool()
        b.data = bool(estop)
        self.pub_estop.publish(b)

        diag = StatusArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        st = StatusStatus()
        st.name = "planning/state_machine"
        st.hardware_id = "planning"
        st.level = _diag_level(StatusStatus.OK)
        if estop:
            st.level = _diag_level(StatusStatus.ERROR)
            st.message = "estop_asserted"
        elif self.state.startswith("WARN"):
            st.level = _diag_level(StatusStatus.WARN)
            st.message = "warn_recovery"
        else:
            st.message = self.state.lower()
        st.values.append(KeyValue(key="state", value=self.state))
        st.values.append(KeyValue(key="active_goal_source", value=self.active_goal_source))
        st.values.append(KeyValue(key="active_goal_key", value=self.active_goal_key))
        st.values.append(KeyValue(key="goal_reached_latched", value=str(self._goal_reached_latched).lower()))
        st.values.append(KeyValue(key="return_requested", value=str(self.return_requested).lower()))
        st.values.append(KeyValue(key="recall_requested", value=str(self.recall_requested).lower()))
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
        now = self.get_clock().now()

        level = self._state_level()
        estop = False
        if self.prev_state_level != self._warn_level and level == self._warn_level:
            self.warn_goal_sent = False
        if level != self._warn_level:
            self.warn_goal_sent = False

        if level >= self._error_level:
            self.state = "ERROR_STOP"
            estop = self.auto_estop_on_error
        elif self.recall_requested:
            self.state = "RECALLED"
            recall_target = self.recall_target_key or self.return_goal_key
            if self._publish_auto_goal(recall_target, self.active_goal_source):
                self.recall_requested = False
                self.startup_goal_sent = True
                self.warn_goal_sent = False
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
                if not self._goal_reached_latched:
                    self._goal_reached_latched = True
                if self._goal_reached_since is None:
                    self._goal_reached_since = now
            if self._goal_reached_latched:
                self.state = "GOAL_REACHED"
                if (
                    self.enable_auto_return_on_site_goal and
                    self._is_active_site_goal() and
                    self.active_goal_key != self.return_goal_key
                ):
                    if self._goal_reached_since is None:
                        self._goal_reached_since = now
                    reached_elapsed_s = (now - self._goal_reached_since).nanoseconds / 1e9
                    if reached_elapsed_s >= self.goal_reached_dwell_s:
                        site_key = self.active_goal_key
                        if self._publish_auto_goal(self.return_goal_key, "auto_return"):
                            self.warn_goal_sent = False
                            self.state = "RETURNING"
                            self.get_logger().info(
                                f"auto-return after dwell: key={site_key} "
                                f"dwell={self.goal_reached_dwell_s:.1f}s -> {self.return_goal_key}"
                            )
            elif self.active_goal is not None:
                self.state = "RUNNING"
                self._goal_reached_since = None
                self._goal_reached_latched = False
            else:
                self.state = "READY"
                self._goal_reached_since = None
                self._goal_reached_latched = False

        self.prev_state_level = level
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
