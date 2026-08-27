#!/usr/bin/env python3
"""Latch Nav2 selector topics and switch them with the active goal source."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


# HH_260730 / TODOLIST 7 - A fresh 50.29 m sim audit returned zero poses from
# every configured grid/kinematic planner but a valid 256-pose LaneletRoute in
# 130 ms. LaneletRoute also preserves the operator's requested final yaw, so
# manual arrival direction remains unrestricted without sacrificing long,
# narrow-map connectivity or active-route sensor masks.
DEFAULT_MANUAL_PLANNER_ID = "LaneletRoute"
# Preserve the develop-branch executable-alone fallbacks. Launch profiles may
# still select LaneletRoute/RPP (as the CARLA overlay does) explicitly.
DEFAULT_REGULATED_PLANNER_ID = "NavFn"
DEFAULT_REGULATED_CONTROLLER_ID = "MPPI"


def resolve_goal_source(requested: str) -> tuple[str, bool]:
    """Return the canonical source and whether the input was recognized."""
    normalized = str(requested).strip().lower()
    if normalized.startswith("manual"):
        return ("manual", True)
    if normalized.startswith("regulated"):
        return ("regulated", True)
    return ("regulated", False)


def selector_ids_for_source(
    source: str,
    *,
    regulated: tuple[str, str, str],
    manual: tuple[str, str, str],
) -> tuple[str, str, str]:
    """Select planner, controller, and goal-checker IDs as one policy."""
    return manual if source == "manual" else regulated


class Nav2SelectorLatchNode(Node):

    # HH_260721 - Separate the class declaration from its first method for lint readability.
    def __init__(self) -> None:
        super().__init__("nav2_selector_latch")

        # HH_260528: Keep combo-level planner/controller choice sticky via transient-local QoS.
        self._regulated_planner_id = str(
            self.declare_parameter(
                "planner_id", DEFAULT_REGULATED_PLANNER_ID
            ).value
        ).strip()
        self._regulated_controller_id = str(
            self.declare_parameter(
                "controller_id", DEFAULT_REGULATED_CONTROLLER_ID
            ).value
        ).strip()
        # HH_260730 / TODOLIST 7 - Both sources use the connected route by
        # default and both receive a projected vehicle-lane position. Manual RViz
        # goals retain requested final yaw; regulated UI goals use lane yaw.
        self._regulated_goal_checker_id = str(
            self.declare_parameter("regulated_goal_checker_id", "goal_checker").value
        ).strip()
        self._manual_planner_id = str(
            self.declare_parameter(
                "manual_planner_id", DEFAULT_MANUAL_PLANNER_ID
            ).value
        ).strip()
        self._manual_controller_id = str(
            self.declare_parameter("manual_controller_id", "RotationShim").value
        ).strip()
        self._manual_goal_checker_id = str(
            self.declare_parameter("manual_goal_checker_id", "manual_goal_checker").value
        ).strip()
        self._goal_source_topic = str(
            self.declare_parameter("goal_source_topic", "/planning/goal_source").value
        ).strip()
        self._planner_topic = str(
            # HH_260720 - Nav2 selector plugins require std_msgs on an explicit ROS boundary.
            self.declare_parameter("planner_topic", "/planning/planner_selector_ros").value
        ).strip()
        self._controller_topic = str(
            # HH_260720 - Nav2 selector plugins require std_msgs on an explicit ROS boundary.
            self.declare_parameter("controller_topic", "/planning/controller_selector_ros").value
        ).strip()
        self._goal_checker_topic = str(
            self.declare_parameter(
                "goal_checker_topic", "/planning/goal_checker_selector_ros"
            ).value
        ).strip()
        self._repeat_hz = float(self.declare_parameter("repeat_hz", 1.0).value)
        self._active_source = "regulated"
        self._active_planner_id = self._regulated_planner_id
        self._active_controller_id = self._regulated_controller_id
        self._active_goal_checker_id = self._regulated_goal_checker_id

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._planner_pub = self.create_publisher(String, self._planner_topic, qos)
        self._controller_pub = self.create_publisher(String, self._controller_topic, qos)
        self._goal_checker_pub = self.create_publisher(String, self._goal_checker_topic, qos)
        self._goal_source_sub = self.create_subscription(
            String, self._goal_source_topic, self._on_goal_source, qos
        )

        self._publish_once()
        self.create_timer(1.0 / max(0.2, self._repeat_hz), self._publish_once)

        self.get_logger().info(
            "nav2_selector_latch active: "
            f"source={self._active_source} planner={self._active_planner_id} "
            f"controller={self._active_controller_id} "
            f"goal_checker={self._active_goal_checker_id}"
        )

    def _on_goal_source(self, msg: String) -> None:
        requested = str(msg.data).strip().lower()
        source, recognized = resolve_goal_source(requested)
        if not recognized:
            # HH_260727 - Fail closed to the lanelet-regulated policy for malformed sources.
            self.get_logger().warn(
                f"unknown goal source '{requested}'; using regulated selectors"
            )

        changed = source != self._active_source
        self._active_source = source
        (
            self._active_planner_id,
            self._active_controller_id,
            self._active_goal_checker_id,
        ) = selector_ids_for_source(
            source,
            regulated=(
                self._regulated_planner_id,
                self._regulated_controller_id,
                self._regulated_goal_checker_id,
            ),
            manual=(
                self._manual_planner_id,
                self._manual_controller_id,
                self._manual_goal_checker_id,
            ),
        )

        self._publish_once()
        if changed:
            self.get_logger().info(
                "nav2 selectors switched: "
                f"source={self._active_source} planner={self._active_planner_id} "
                f"controller={self._active_controller_id} "
                f"goal_checker={self._active_goal_checker_id}"
            )

    def _publish_once(self) -> None:
        planner_msg = String()
        planner_msg.data = self._active_planner_id
        self._planner_pub.publish(planner_msg)

        controller_msg = String()
        controller_msg.data = self._active_controller_id
        self._controller_pub.publish(controller_msg)

        goal_checker_msg = String()
        goal_checker_msg.data = self._active_goal_checker_id
        self._goal_checker_pub.publish(goal_checker_msg)


def main() -> None:
    rclpy.init()
    node = Nav2SelectorLatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # HH_260617: Ctrl+C during launch shutdown should exit cleanly instead
        # of printing a traceback and reporting this helper as a crashed process.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
