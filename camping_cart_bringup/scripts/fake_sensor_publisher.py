#!/usr/bin/env python3
# HH_260109 Generate fake GNSS/IMU/Wheel/VIO/Obstacle topics from lanelet centerline.

import math
import time
import xml.etree.ElementTree as ET
from bisect import bisect_left

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def deg2rad(deg):
    return deg * math.pi / 180.0


def llh_to_ecef(lat_rad, lon_rad, alt):
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt) * cos_lat * cos_lon
    y = (n + alt) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + alt) * sin_lat
    return (x, y, z)


def ecef_to_enu(ref_ecef, cur_ecef, lat_ref, lon_ref):
    sin_lat = math.sin(lat_ref)
    cos_lat = math.cos(lat_ref)
    sin_lon = math.sin(lon_ref)
    cos_lon = math.cos(lon_ref)
    dx = cur_ecef[0] - ref_ecef[0]
    dy = cur_ecef[1] - ref_ecef[1]
    dz = cur_ecef[2] - ref_ecef[2]
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return (east, north, up)


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__("fake_sensor_publisher")

        self.map_path = self.declare_parameter("map_path", "").value
        self.origin_lat = self.declare_parameter("origin_lat", 0.0).value
        self.origin_lon = self.declare_parameter("origin_lon", 0.0).value
        self.origin_alt = self.declare_parameter("origin_alt", 0.0).value
        self.lanelet_id = self.declare_parameter("lanelet_id", -1).value
        self.speed_mps = self.declare_parameter("speed_mps", 1.4).value
        self.publish_rate_hz = self.declare_parameter("publish_rate_hz", 20.0).value
        self.loop = self.declare_parameter("loop", True).value
        # 2026-02-05 14:37: Skip crosswalk lanelets to prevent centerline jumps.
        self.exclude_crosswalk = self.declare_parameter("exclude_crosswalk", True).value
        # 2026-02-05 14:37: Optionally snap the start position to the nearest path point.
        self.use_start_pose = self.declare_parameter("use_start_pose", False).value
        self.start_x = self.declare_parameter("start_x", 0.0).value
        self.start_y = self.declare_parameter("start_y", 0.0).value
        # 2026-02-02: Use all lanelet centerlines to traverse the full loop.
        self.use_all_centerlines = self.declare_parameter("use_all_centerlines", True).value
        self.centerline_connect_max_gap = float(
            self.declare_parameter("centerline_connect_max_gap", 5.0).value
        )
        # 2026-02-02: Optionally close a near-loop path for continuous laps.
        self.close_loop = self.declare_parameter("close_loop", True).value
        self.close_loop_max_gap = float(
            self.declare_parameter("close_loop_max_gap", 3.0).value
        )
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.base_frame_id = self.declare_parameter("base_frame_id", "robot_base_link").value
        self.obstacle_offset = self.declare_parameter("obstacle_offset", 5.0).value
        self.obstacle_height = self.declare_parameter("obstacle_height", 0.5).value
        # 2026-02-02 11:05: Resample centerline from bounds when explicit centerline is missing.
        self.centerline_step = self.declare_parameter("centerline_step", 1.0).value

        if not self.map_path:
            # 2026-01-27 17:45: Remove HH tags from runtime logs.
            self.get_logger().fatal("map_path is required.")
            raise RuntimeError("map_path is empty")

        self._path = self._load_centerline_path()
        if len(self._path) < 2:
            self.get_logger().fatal("failed to load lanelet centerline path.")
            raise RuntimeError("centerline path missing")
        # 2026-02-02: Close loop if endpoints are close enough to avoid a hard jump.
        if self.close_loop and len(self._path) > 2:
            dx = self._path[0][0] - self._path[-1][0]
            dy = self._path[0][1] - self._path[-1][1]
            gap = math.hypot(dx, dy)
            if gap <= self.close_loop_max_gap:
                self._path.append(self._path[0])
            else:
                self.get_logger().warn(
                    f"Centerline loop gap {gap:.2f}m exceeds close_loop_max_gap; "
                    "path will wrap with a jump. Consider a loop lanelet_id."
                )

        self._distances = self._build_cumulative_distances(self._path)
        self._total_length = self._distances[-1]
        self._start_distance = 0.0
        if self.use_start_pose:
            self._start_distance = self._nearest_distance_on_path(self.start_x, self.start_y)
        self._t0 = time.time()
        self._last_yaw = None
        self._last_yaw_time = None

        # HH_260109 Publish fake sensors with module-prefixed topics.
        self.pub_navsat = self.create_publisher(NavSatFix, "/sensing/gnss/navsatfix", 10)
        self.pub_imu = self.create_publisher(Imu, "/sensing/imu/data", 10)
        self.pub_wheel = self.create_publisher(Odometry, "/platform/wheel/odometry", 10)
        self.pub_vio = self.create_publisher(Odometry, "/localization/vio/odometry", 10)
        self.pub_obstacles = self.create_publisher(PointCloud2, "/perception/obstacles", 10)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().debug(
            f"Fake sensors ready. lanelet_id={self.lanelet_id} speed={self.speed_mps} m/s"
        )

    def _load_centerline_path(self):
        # HH_260109 Parse Lanelet2 OSM and extract centerline ways.
        tree = ET.parse(self.map_path)
        root = tree.getroot()

        nodes = {}
        for node in root.findall("node"):
            nid = int(node.attrib["id"])
            lat = float(node.attrib["lat"])
            lon = float(node.attrib["lon"])
            nodes[nid] = (lat, lon)

        ways = {}
        for way in root.findall("way"):
            wid = int(way.attrib["id"])
            nds = [int(nd.attrib["ref"]) for nd in way.findall("nd")]
            ways[wid] = nds

        lanelets = []
        for rel in root.findall("relation"):
            tags = {t.attrib["k"]: t.attrib["v"] for t in rel.findall("tag")}
            if tags.get("type") != "lanelet":
                continue
            if (self.exclude_crosswalk and self.lanelet_id < 0 and
                    tags.get("subtype") == "crosswalk"):
                continue
            rid = int(rel.attrib["id"])
            if self.lanelet_id >= 0 and rid != int(self.lanelet_id):
                continue
            lanelets.append((rid, rel))

        segments = []
        for _, rel in lanelets:
            centerline_way = None
            left_way = None
            right_way = None
            for member in rel.findall("member"):
                role = member.attrib.get("role")
                if role == "centerline":
                    centerline_way = int(member.attrib["ref"])
                elif role == "left":
                    left_way = int(member.attrib["ref"])
                elif role == "right":
                    right_way = int(member.attrib["ref"])

            if centerline_way is not None and centerline_way in ways:
                path = self._build_path_from_way(ways[centerline_way], nodes)
            elif left_way is not None and right_way is not None and left_way in ways and right_way in ways:
                path = self._build_path_from_bounds(ways[left_way], ways[right_way], nodes)
            else:
                continue

            if len(path) >= 2:
                segments.append(path)

        if not segments:
            self.get_logger().error(
                "No centerline or bound-based paths found for lanelets in map."
            )
            return []

        # Lanelet-specific request: return the single lanelet path.
        if self.lanelet_id >= 0 or not self.use_all_centerlines:
            return segments[0]

        # 2026-02-02: Stitch all lanelet centerlines into one continuous loop.
        return self._stitch_centerlines(segments)

    def _stitch_centerlines(self, segments):
        if not segments:
            return []

        def seg_length(seg):
            total = 0.0
            for i in range(1, len(seg)):
                dx = seg[i][0] - seg[i - 1][0]
                dy = seg[i][1] - seg[i - 1][1]
                total += math.hypot(dx, dy)
            return total

        def endpoint_neighbors(idx, point, segs, max_gap):
            count = 0
            for j, seg in enumerate(segs):
                if j == idx:
                    continue
                if (math.hypot(point[0] - seg[0][0], point[1] - seg[0][1]) <= max_gap or
                        math.hypot(point[0] - seg[-1][0], point[1] - seg[-1][1]) <= max_gap):
                    count += 1
            return count

        def neighbor_count(point, segs, max_gap):
            count = 0
            for seg in segs:
                if (math.hypot(point[0] - seg[0][0], point[1] - seg[0][1]) <= max_gap or
                        math.hypot(point[0] - seg[-1][0], point[1] - seg[-1][1]) <= max_gap):
                    count += 1
            return count

        segments = list(segments)
        max_gap = self.centerline_connect_max_gap
        start_idx = None
        best_score = float("inf")
        best_length = -1.0
        for i, seg in enumerate(segments):
            n_start = endpoint_neighbors(i, seg[0], segments, max_gap)
            n_end = endpoint_neighbors(i, seg[-1], segments, max_gap)
            score = n_start + n_end
            length = seg_length(seg)
            if score < best_score or (score == best_score and length > best_length):
                best_score = score
                best_length = length
                start_idx = i

        path = segments.pop(start_idx)
        # Prefer starting from the endpoint with fewer neighbors (open chain start).
        if neighbor_count(path[0], segments, max_gap) > neighbor_count(path[-1], segments, max_gap):
            path = list(reversed(path))
        current_end = path[-1]

        while segments:
            candidates = []
            fallback = None
            for i, seg in enumerate(segments):
                d_start = math.hypot(current_end[0] - seg[0][0], current_end[1] - seg[0][1])
                d_end = math.hypot(current_end[0] - seg[-1][0], current_end[1] - seg[-1][1])
                reverse = d_end < d_start
                dist = min(d_start, d_end)
                if dist <= max_gap:
                    candidates.append((dist, i, reverse))
                if fallback is None or dist < fallback[0]:
                    fallback = (dist, i, reverse)

            if candidates:
                dist, best_idx, best_reverse = min(candidates, key=lambda t: t[0])
            else:
                dist, best_idx, best_reverse = fallback
                self.get_logger().warn(
                    f"No neighbor within {max_gap:.2f}m; stitching nearest gap {dist:.2f}m."
                )

            seg = segments.pop(best_idx)
            if best_reverse:
                seg = list(reversed(seg))
            if dist > max_gap:
                self.get_logger().warn(
                    f"Centerline stitch gap {dist:.2f}m exceeds centerline_connect_max_gap."
                )
            path.extend(seg[1:])
            current_end = path[-1]

        return path

    def _find_left_right_bounds(self, root):
        selected_left = None
        selected_right = None
        for rel in root.findall("relation"):
            tags = {t.attrib["k"]: t.attrib["v"] for t in rel.findall("tag")}
            if tags.get("type") != "lanelet":
                continue
            rid = int(rel.attrib["id"])
            if self.lanelet_id >= 0 and rid != int(self.lanelet_id):
                continue
            for member in rel.findall("member"):
                role = member.attrib.get("role")
                if role == "left":
                    selected_left = int(member.attrib["ref"])
                elif role == "right":
                    selected_right = int(member.attrib["ref"])
            if selected_left is not None and selected_right is not None:
                break
        return selected_left, selected_right

    def _build_path_from_way(self, way_nodes, nodes):
        origin_lat_rad = deg2rad(self.origin_lat)
        origin_lon_rad = deg2rad(self.origin_lon)
        ref_ecef = llh_to_ecef(origin_lat_rad, origin_lon_rad, self.origin_alt)
        path = []
        for nid in way_nodes:
            if nid not in nodes:
                continue
            lat, lon = nodes[nid]
            cur_ecef = llh_to_ecef(deg2rad(lat), deg2rad(lon), self.origin_alt)
            enu = ecef_to_enu(ref_ecef, cur_ecef, origin_lat_rad, origin_lon_rad)
            path.append((enu[0], enu[1], 0.0, lat, lon))
        return path

    def _build_path_from_bounds(self, left_way_nodes, right_way_nodes, nodes):
        origin_lat_rad = deg2rad(self.origin_lat)
        origin_lon_rad = deg2rad(self.origin_lon)
        ref_ecef = llh_to_ecef(origin_lat_rad, origin_lon_rad, self.origin_alt)

        def way_to_xy(way_nodes):
            pts = []
            for nid in way_nodes:
                if nid not in nodes:
                    continue
                lat, lon = nodes[nid]
                cur_ecef = llh_to_ecef(deg2rad(lat), deg2rad(lon), self.origin_alt)
                enu = ecef_to_enu(ref_ecef, cur_ecef, origin_lat_rad, origin_lon_rad)
                pts.append((enu[0], enu[1], lat, lon))
            return pts

        left_pts = way_to_xy(left_way_nodes)
        right_pts = way_to_xy(right_way_nodes)
        if len(left_pts) < 2 or len(right_pts) < 2:
            self.get_logger().error("Left/right bounds are too short to synthesize centerline.")
            return []

        left_dist = self._build_cumulative_distances(left_pts)
        right_dist = self._build_cumulative_distances(right_pts)
        max_s = min(left_dist[-1], right_dist[-1])
        step = max(float(self.centerline_step), 0.2)
        s = 0.0
        path = []
        while s <= max_s:
            lx, ly, lat_l, lon_l = self._sample_polyline(left_pts, left_dist, s)
            rx, ry, lat_r, lon_r = self._sample_polyline(right_pts, right_dist, s)
            path.append(((lx + rx) * 0.5, (ly + ry) * 0.5, 0.0, (lat_l + lat_r) * 0.5, (lon_l + lon_r) * 0.5))
            s += step
        if not path:
            self.get_logger().error("Failed to synthesize centerline path from bounds.")
        return path

    @staticmethod
    def _build_cumulative_distances(points):
        distances = [0.0]
        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            distances.append(distances[-1] + math.hypot(dx, dy))
        return distances

    def _nearest_distance_on_path(self, x, y):
        # 2026-02-05 14:37: Snap start offset to nearest vertex on the path.
        best_idx = 0
        best_dist = float("inf")
        for i, p in enumerate(self._path):
            dx = p[0] - x
            dy = p[1] - y
            dist = dx * dx + dy * dy
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        return self._distances[best_idx]

    @staticmethod
    def _sample_polyline(points, distances, s):
        if s <= 0.0:
            return points[0]
        if s >= distances[-1]:
            return points[-1]
        idx = bisect_left(distances, s)
        idx = max(1, min(idx, len(points) - 1))
        d0 = distances[idx - 1]
        d1 = distances[idx]
        t = 0.0 if d1 <= d0 else (s - d0) / (d1 - d0)
        p0 = points[idx - 1]
        p1 = points[idx]
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        lat = p0[2] + t * (p1[2] - p0[2])
        lon = p0[3] + t * (p1[3] - p0[3])
        return (x, y, lat, lon)

    def _sample_path(self, dist):
        if self._total_length <= 1e-6:
            return self._path[0], 0.0

        dist += self._start_distance
        if self.loop:
            dist = dist % self._total_length
        else:
            dist = min(dist, self._total_length)

        idx = 1
        while idx < len(self._distances) and self._distances[idx] < dist:
            idx += 1
        idx = min(idx, len(self._distances) - 1)
        d0 = self._distances[idx - 1]
        d1 = self._distances[idx]
        t = 0.0 if d1 <= d0 else (dist - d0) / (d1 - d0)
        p0 = self._path[idx - 1]
        p1 = self._path[idx]
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        z = p0[2] + t * (p1[2] - p0[2])
        lat = p0[3] + t * (p1[3] - p0[3])
        lon = p0[4] + t * (p1[4] - p0[4])
        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        return (x, y, z, lat, lon), yaw

    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        elapsed = time.time() - self._t0
        dist = elapsed * self.speed_mps
        (x, y, z, lat, lon), yaw = self._sample_path(dist)

        # HH_260112 Create a pose message for downstream odometry/cloud outputs.
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation = yaw_to_quat(yaw)

        navsat = NavSatFix()
        navsat.header.stamp = now
        navsat.header.frame_id = self.frame_id
        navsat.status.status = NavSatStatus.STATUS_FIX
        navsat.status.service = NavSatStatus.SERVICE_GPS
        navsat.latitude = lat
        navsat.longitude = lon
        navsat.altitude = self.origin_alt
        self.pub_navsat.publish(navsat)

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.base_frame_id
        imu_msg.orientation = yaw_to_quat(yaw)
        # 2026-02-02 11:55: Provide yaw rate so ESKF can integrate heading.
        now_sec = time.time()
        yaw_rate = 0.0
        if self._last_yaw is not None and self._last_yaw_time is not None:
            dt = max(1e-3, now_sec - self._last_yaw_time)
            dyaw = normalize_angle(yaw - self._last_yaw)
            yaw_rate = dyaw / dt
        self._last_yaw = yaw
        self._last_yaw_time = now_sec
        imu_msg.angular_velocity.z = yaw_rate
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        self.pub_imu.publish(imu_msg)

        wheel_msg = Odometry()
        wheel_msg.header.stamp = now
        wheel_msg.header.frame_id = "odom"
        wheel_msg.child_frame_id = self.base_frame_id
        wheel_msg.twist.twist.linear.x = self.speed_mps
        wheel_msg.twist.twist.angular.z = 0.0
        self.pub_wheel.publish(wheel_msg)

        vio_msg = Odometry()
        vio_msg.header.stamp = now
        vio_msg.header.frame_id = self.frame_id
        vio_msg.child_frame_id = self.base_frame_id
        vio_msg.pose.pose = pose_msg.pose
        vio_msg.twist.twist.linear.x = self.speed_mps
        vio_msg.twist.twist.angular.z = 0.0
        self.pub_vio.publish(vio_msg)

        obstacle_points = [
            (x + self.obstacle_offset, y, self.obstacle_height),
            (x + self.obstacle_offset, y + 1.0, self.obstacle_height),
            (x + self.obstacle_offset, y - 1.0, self.obstacle_height),
        ]
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = point_cloud2.create_cloud(pose_msg.header, fields, obstacle_points)
        self.pub_obstacles.publish(cloud_msg)


def main():
    rclpy.init()
    node = FakeSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
