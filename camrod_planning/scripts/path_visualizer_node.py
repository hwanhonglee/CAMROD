#!/usr/bin/env python3
"""Publish high-contrast RViz markers for global/local planning paths."""

import math
from typing import Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


Color = Tuple[float, float, float, float]


def _path_point(point: Point, z_offset: float) -> Point:
    out = Point()
    out.x = point.x
    out.y = point.y
    out.z = point.z + z_offset
    return out


def _make_color(color: Color):
    msg = ColorRGBA()
    msg.r, msg.g, msg.b, msg.a = color
    return msg


class PathVisualizerNode(Node):
    """Converts nav_msgs/Path into thick line and direction arrow markers."""

    def __init__(self) -> None:
        super().__init__("path_visualizer")

        self.global_path_topic = self.declare_parameter(
            "global_path_topic", "/planning/global_path"
        ).value
        self.local_path_topic = self.declare_parameter(
            "local_path_topic", "/planning/local_path"
        ).value
        self.marker_topic = self.declare_parameter(
            "marker_topic", "/planning/path_markers"
        ).value
        self.global_line_width_m = float(self.declare_parameter("global_line_width_m", 0.22).value)
        self.local_line_width_m = float(self.declare_parameter("local_line_width_m", 0.30).value)
        self.global_arrow_spacing_m = float(
            self.declare_parameter("global_arrow_spacing_m", 3.0).value
        )
        self.local_arrow_spacing_m = float(
            self.declare_parameter("local_arrow_spacing_m", 1.2).value
        )
        self.max_global_arrows = int(self.declare_parameter("max_global_arrows", 80).value)
        self.max_local_arrows = int(self.declare_parameter("max_local_arrows", 40).value)
        self.republish_period_s = float(self.declare_parameter("republish_period_s", 0.20).value)
        self.global_path_stale_timeout_s = float(
            self.declare_parameter("global_path_stale_timeout_s", 1.0).value
        )
        self.route_endpoint_mismatch_m = float(
            self.declare_parameter("route_endpoint_mismatch_m", 1.0).value
        )

        self.global_path: Path | None = None
        self.local_path: Path | None = None
        self.last_global_path_rx = None
        self.last_marker_array: MarkerArray | None = None
        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 1)
        self.create_subscription(Path, self.global_path_topic, self._on_global_path, 10)
        self.create_subscription(Path, self.local_path_topic, self._on_local_path, 10)
        if self.republish_period_s > 0.0:
            self.create_timer(self.republish_period_s, self._republish)

        self.get_logger().info(
            f"path visualizer started: global={self.global_path_topic} "
            f"local={self.local_path_topic} marker={self.marker_topic}"
        )

    def _on_global_path(self, msg: Path) -> None:
        self.global_path = msg
        self.last_global_path_rx = self.get_clock().now()
        self._publish()

    def _on_local_path(self, msg: Path) -> None:
        self.local_path = msg
        self._clear_stale_global_path_if_mismatched(msg)
        self._publish()

    def _clear_stale_global_path_if_mismatched(self, local_path: Path) -> None:
        # HH_260619 - Avoid showing an old global route together with the newest
        # local path. Nav2 global routes are event-driven, while local paths are
        # republished continuously; if the global source topic is stale or silent
        # during a goal replacement, keep RViz honest by dropping the old marker.
        if self.global_path is None or self.last_global_path_rx is None:
            return
        if len(self.global_path.poses) < 2 or len(local_path.poses) < 2:
            return
        if self.global_path_stale_timeout_s <= 0.0:
            return
        age_s = (self.get_clock().now() - self.last_global_path_rx).nanoseconds * 1.0e-9
        if age_s < self.global_path_stale_timeout_s:
            return
        threshold = max(0.0, self.route_endpoint_mismatch_m)
        if threshold <= 0.0:
            return
        global_start = self.global_path.poses[0].pose.position
        global_goal = self.global_path.poses[-1].pose.position
        local_start = local_path.poses[0].pose.position
        local_goal = local_path.poses[-1].pose.position
        start_delta = math.hypot(global_start.x - local_start.x, global_start.y - local_start.y)
        goal_delta = math.hypot(global_goal.x - local_goal.x, global_goal.y - local_goal.y)
        if start_delta > threshold or goal_delta > threshold:
            self.get_logger().warn(
                "path_visualizer: clearing stale global marker "
                f"(age={age_s:.2f}s start_delta={start_delta:.2f}m goal_delta={goal_delta:.2f}m)"
            )
            self.global_path = None

    def _publish(self) -> None:
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        marker_id = 0
        if self.global_path is not None:
            marker_id = self._append_path_markers(
                marker_array,
                self.global_path,
                marker_id,
                namespace="global_path",
                line_color=(0.00, 0.20, 1.00, 0.98),
                arrow_color=(0.00, 0.08, 0.85, 0.95),
                endpoint_color=(0.00, 0.00, 0.45, 1.00),
                line_width=self.global_line_width_m,
                arrow_spacing=self.global_arrow_spacing_m,
                max_arrows=self.max_global_arrows,
                z_offset=0.22,
            )
        if self.local_path is not None:
            self._append_path_markers(
                marker_array,
                self.local_path,
                marker_id,
                namespace="local_path",
                line_color=(1.00, 0.52, 0.00, 1.00),
                arrow_color=(1.00, 0.25, 0.00, 1.00),
                endpoint_color=(0.70, 0.10, 0.00, 1.00),
                line_width=self.local_line_width_m,
                arrow_spacing=self.local_arrow_spacing_m,
                max_arrows=self.max_local_arrows,
                z_offset=0.32,
            )
        self.pub.publish(marker_array)
        self.last_marker_array = marker_array

    def _republish(self) -> None:
        # HH_260618 - Keep cached path markers visible for RViz late joins and
        # display resets without forcing cost-grid marker nodes to republish.
        if self.last_marker_array is not None:
            now = self.get_clock().now().to_msg()
            for marker in self.last_marker_array.markers:
                marker.header.stamp = now
            self.pub.publish(self.last_marker_array)

    def _append_path_markers(
        self,
        marker_array: MarkerArray,
        path: Path,
        marker_id: int,
        *,
        namespace: str,
        line_color: Color,
        arrow_color: Color,
        endpoint_color: Color,
        line_width: float,
        arrow_spacing: float,
        max_arrows: int,
        z_offset: float,
    ) -> int:
        if len(path.poses) < 2:
            return marker_id

        # HH_260618 - Publish thick path markers above cost grids so global/local
        # trajectories remain readable in RViz even when cost layers overlap.
        line = Marker()
        line.header = path.header
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = namespace
        line.id = marker_id
        marker_id += 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = line_width
        line.pose.orientation.w = 1.0
        line.color = _make_color(line_color)
        line.points = [_path_point(pose.pose.position, z_offset) for pose in path.poses]
        marker_array.markers.append(line)

        marker_id = self._append_direction_arrows(
            marker_array,
            path,
            marker_id,
            namespace=namespace + "_direction",
            color=arrow_color,
            spacing=arrow_spacing,
            max_arrows=max_arrows,
            z_offset=z_offset + 0.04,
        )
        marker_id = self._append_endpoint_spheres(
            marker_array,
            path,
            marker_id,
            namespace=namespace + "_endpoints",
            color=endpoint_color,
            z_offset=z_offset + 0.06,
        )
        return marker_id

    def _append_direction_arrows(
        self,
        marker_array: MarkerArray,
        path: Path,
        marker_id: int,
        *,
        namespace: str,
        color: Color,
        spacing: float,
        max_arrows: int,
        z_offset: float,
    ) -> int:
        if spacing <= 0.0 or max_arrows <= 0:
            return marker_id
        distance_since_arrow = spacing
        arrows_added = 0
        poses = path.poses
        for previous, current in zip(poses[:-1], poses[1:]):
            prev_pos = previous.pose.position
            curr_pos = current.pose.position
            dx = curr_pos.x - prev_pos.x
            dy = curr_pos.y - prev_pos.y
            segment_length = math.hypot(dx, dy)
            if segment_length < 1.0e-6:
                continue
            distance_since_arrow += segment_length
            if distance_since_arrow < spacing:
                continue
            distance_since_arrow = 0.0
            arrow = Marker()
            arrow.header = path.header
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = namespace
            arrow.id = marker_id
            marker_id += 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            arrow.scale.x = 0.55
            arrow.scale.y = 0.13
            arrow.scale.z = 0.13
            arrow.color = _make_color(color)
            start = _path_point(prev_pos, z_offset)
            end = _path_point(curr_pos, z_offset)
            direction_scale = min(0.75, segment_length) / segment_length
            end.x = start.x + dx * direction_scale
            end.y = start.y + dy * direction_scale
            arrow.points = [start, end]
            marker_array.markers.append(arrow)
            arrows_added += 1
            if arrows_added >= max_arrows:
                break
        return marker_id

    def _append_endpoint_spheres(
        self,
        marker_array: MarkerArray,
        path: Path,
        marker_id: int,
        *,
        namespace: str,
        color: Color,
        z_offset: float,
    ) -> int:
        for endpoint_name, pose in (("start", path.poses[0]), ("goal", path.poses[-1])):
            marker = Marker()
            marker.header = path.header
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = namespace + "_" + endpoint_name
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = _path_point(pose.pose.position, z_offset)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.34
            marker.scale.y = 0.34
            marker.scale.z = 0.34
            marker.color = _make_color(color)
            marker_array.markers.append(marker)
        return marker_id


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PathVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # HH_260618: Launch shutdown may already close the default context;
        # guard shutdown so Ctrl-C cleanup is not reported as a node crash.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
