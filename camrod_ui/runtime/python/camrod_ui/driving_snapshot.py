"""HH_261001 - Bounded passive display data; never owns motion or mission state.

Ages advance monotonically; timestamped motion/objects also include source age.
Diagnostic labels describe reported health, not proof of a fresh camera frame
or a clear obstacle field.
"""

from __future__ import annotations

import math
import threading
import time


TRAVEL_STATES = frozenset({
    "MOVING_TO_SITE", "RECALL_TO_SITE_ROAD", "RETURNING_TO_DROP_ZONE",
    # HH_261001 - The return controller may retain RETURN_WITH_CARGO after
    # its local maneuver is done and a global Nav2 return path is available.
    # Exact return-leg identity still keeps the outbound path hidden.
    "RETURN_WITH_CARGO",
})
# HH_261001 - Bind a latched planner path to its logical leg, not an individual
# service heartbeat. The planner can publish once before ROAD_HANDOFF_READY
# becomes MOVING_TO_SITE (or RETURN_WITH_CARGO becomes return-road travel).
OUTBOUND_STATES = frozenset({
    "DEPARTING_CHARGER", "DEPARTING_DROP_ZONE", "ROAD_HANDOFF_READY",
    "MOVING_TO_SITE", "RECALL_TO_SITE_ROAD", "GUEST_RECALL_SERVICE",
    "SITE_ENTRY",
})
RETURN_STATES = frozenset({
    "RETURN_WITH_CARGO", "RETURNING_TO_DROP_ZONE", "DROP_ZONE_PARKING",
})
PROGRESS_FIELDS = (
    "remaining_distance_m", "remaining_time_s", "completion_pct",
)
# HH_261002 - Keep static road geometry separate from mission-bound planner paths
# and the on-demand diagnostic map. Bounds apply to both storage and HTTP output.
BASE_MAP_NAMESPACES = frozenset({
    "lanelet/centerline", "lanelet/left_bound", "lanelet/right_bound",
})
BASE_MAP_MAX_LINES = 512
BASE_MAP_MAX_POINTS = 3000
MAP_AREA_MAX_COUNT = 64
MAP_AREA_MAX_VERTICES = 128


def sample_indices(size, limit):
    """HH_261002 - Bound geometry work while retaining both line endpoints."""
    if size <= limit:
        return range(size)
    return (round(index * (size - 1) / (limit - 1)) for index in range(limit))


def finite(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def validated_map_polygon(points):
    """HH_261002 - Preserve an authored simple ring; never infer or resize it."""
    if not isinstance(points, (list, tuple)) or not 3 <= len(points) <= MAP_AREA_MAX_VERTICES + 1:
        return None
    polygon = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            return None
        x, y = (finite(value) for value in point)
        if x is None or y is None:
            return None
        polygon.append((x, y))
    if polygon[0] == polygon[-1]:
        polygon.pop()
    if (not 3 <= len(polygon) <= MAP_AREA_MAX_VERTICES
            or len(set(polygon)) != len(polygon)):
        return None
    edges = list(zip(polygon, polygon[1:] + polygon[:1]))
    area_twice = sum(a[0] * b[1] - b[0] * a[1] for a, b in edges)
    if not math.isfinite(area_twice) or abs(area_twice) <= 1e-8:
        return None

    def orientation(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def on_segment(a, b, c):
        return (min(a[0], b[0]) - 1e-9 <= c[0] <= max(a[0], b[0]) + 1e-9
                and min(a[1], b[1]) - 1e-9 <= c[1] <= max(a[1], b[1]) + 1e-9)

    for index, (a, b) in enumerate(edges):
        for other in range(index + 1, len(edges)):
            if other == index + 1 or (index == 0 and other == len(edges) - 1):
                continue
            c, d = edges[other]
            turns = (orientation(a, b, c), orientation(a, b, d),
                     orientation(c, d, a), orientation(c, d, b))
            if not all(math.isfinite(value) for value in turns):
                return None
            if ((turns[0] > 0) != (turns[1] > 0) and (turns[2] > 0) != (turns[3] > 0)
                    or abs(turns[0]) <= 1e-9 and on_segment(a, b, c)
                    or abs(turns[1]) <= 1e-9 and on_segment(a, b, d)
                    or abs(turns[2]) <= 1e-9 and on_segment(c, d, a)
                    or abs(turns[3]) <= 1e-9 and on_segment(c, d, b)):
                return None
    return [[x, y] for x, y in polygon]


def mission_key(mission):
    """HH_261001 - A return or new dispatch must not inherit an outbound route."""
    state = mission.get("service_state_name")
    leg = ("outbound" if state in OUTBOUND_STATES else
           "return" if state in RETURN_STATES else f"state:{state}")
    return (
        mission.get("active") is True, mission.get("generation"),
        mission.get("site"), mission.get("intent"), leg,
    )


def transform_detection_point(point, translation, rotation):
    """HH_261001 - Use exact 3D TF; never invent unknown height or invalid TF."""
    values = [finite(value) for value in (*point, *translation, *rotation)]
    if len(values) != 10 or any(value is None for value in values):
        return None
    x, y, z, tx, ty, tz, qx, qy, qz, qw = values
    norm = math.hypot(qx, qy, qz, qw)
    if norm < 1e-12:
        return None
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    ax, ay, az = 2 * (qy * z - qz * y), 2 * (qz * x - qx * z), 2 * (qx * y - qy * x)
    result = [x + qw * ax + qy * az - qz * ay + tx,
              y + qw * ay + qz * ax - qx * az + ty,
              z + qw * az + qx * ay - qy * ax + tz]
    return result if all(math.isfinite(value) for value in result) else None


def transform_detection_orientation(orientation, rotation=(0, 0, 0, 1)):
    """HH_261002 - Compose normalized TF and observed-box quaternions, never yaw guesses."""
    def normalized(values):
        values = [finite(value) for value in values]
        if len(values) != 4 or any(value is None for value in values):
            return None
        length = math.hypot(*values)
        return ([value / length for value in values]
                if math.isfinite(length) and length >= 1e-12 else None)

    source, target = normalized(orientation), normalized(rotation)
    if source is None or target is None:
        return None
    x, y, z, w = target
    a, b, c, d = source
    return normalized((w * a + x * d + y * c - z * b,
                       w * b - x * c + y * d + z * a,
                       w * c + x * b - y * a + z * d,
                       w * d - x * a - y * b - z * c))


class DrivingSnapshotCache:
    """HH_261001 - Keep a small lock-protected cache outside the operator lease."""

    def __init__(self, now_fn=time.monotonic):
        self._now = now_fn
        self._lock = threading.Lock()
        self._pose = None
        self._platform = None
        self._wheels = None
        self._objects = {}
        self._object_boxes = {}
        self._base_map = {}
        self._base_map_received = None
        self._base_map_source = "/map/markers"
        self._map_areas = []
        self._map_areas_received = None
        self._route = None
        self._route_goal_cutover_stamp = None
        self._progress = {}
        self._sensors = {}

    def base_map(self, records, *, deleted_ids=(), clear=False, source="/map/markers"):
        """HH_261002 - Retain bounded static roads across idle, cancel and new goals.

        Marker IDs are scoped by namespace; DELETEALL/new-map publication replaces
        the cache. Unlike sensor data, a latched map has no freshness timeout.
        Only already-transformed map-frame geometry may enter this display cache.
        """
        with self._lock:
            if clear or source != self._base_map_source:
                self._base_map.clear()
            self._base_map_source = source
            self._base_map_received = self._now()
            for identity in deleted_ids:
                self._base_map.pop(identity, None)
            for record in records[:BASE_MAP_MAX_LINES]:
                namespace = record.get("namespace")
                marker_id = record.get("id")
                # HH_261002 - Expose the original ROS marker identity so clients
                # need not guess boundary pairs from filtered array ordering.
                # Identity is not proof of lanelet pairing; geometry still needs
                # validation and any derived road surface remains illustrative.
                if type(marker_id) is not int:
                    continue
                identity = (namespace, marker_id)
                self._base_map.pop(identity, None)
                if namespace not in BASE_MAP_NAMESPACES or record.get("frame_id") != "map":
                    continue
                points = record.get("points", [])
                selected = [points[index] for index in sample_indices(len(points), 300)]
                if len(selected) < 2 or any(
                    len(point) != 2 or any(finite(value) is None for value in point)
                    for point in selected
                ):
                    continue
                if len(self._base_map) < BASE_MAP_MAX_LINES:
                    self._base_map[identity] = {
                        "namespace": namespace, "marker_id": marker_id,
                        "points": [[float(x), float(y)] for x, y in selected],
                    }
            # HH_261002 - Share the budget across roads instead of filling it with
            # the first long centerline and silently dropping all later bounds.
            limit = max(2, BASE_MAP_MAX_POINTS // max(1, len(self._base_map)))
            self._base_map = {
                identity: {**line, "points": [line["points"][index] for index in
                                              sample_indices(len(line["points"]), limit)]}
                for identity, line in self._base_map.items()
            }

    def map_areas(self, records):
        """HH_261002 - Replace passive configured areas, independent of missions.

        Marker DELETEALL affects only marker roads. These polygons come from the
        separately configured map catalogs, which are loaded at backend startup.
        Invalid input never becomes a guessed rectangle around a goal/keypoint.
        """
        accepted, duplicate_ids = {}, set()
        if isinstance(records, (list, tuple)) and len(records) <= MAP_AREA_MAX_COUNT:
            for record in records:
                if not isinstance(record, dict):
                    continue
                identity, kind, site = record.get("id"), record.get("kind"), record.get("site")
                if not isinstance(identity, str) or not 1 <= len(identity) <= 96:
                    continue
                if identity in accepted or identity in duplicate_ids:
                    accepted.pop(identity, None)
                    duplicate_ids.add(identity)
                    continue
                if record.get("frame_id") != "map":
                    continue
                if kind == "camping_site":
                    if (not isinstance(site, str)
                            or site not in {f"B{index}" for index in range(1, 14)}
                            or identity != f"camping_site_{site[1:]}"
                            or record.get("source") != "camping_sites_yaml"):
                        continue
                    label = site
                elif kind == "drop_zone":
                    if record.get("source") != "drop_zones_yaml":
                        continue
                    label, site = "드롭존", None
                else:
                    continue
                points = validated_map_polygon(record.get("points"))
                if points is None:
                    continue
                accepted[identity] = {
                    "id": identity, "label": label, "kind": kind, "site": site,
                    "points": points, "source": record["source"],
                }
        with self._lock:
            self._map_areas = list(accepted.values())
            self._map_areas_received = self._now()

    def pose(self, x, y, yaw, frame_id):
        with self._lock:
            self._pose = (self._now(), {
                "x": finite(x), "y": finite(y), "yaw": finite(yaw),
                "frame_id": str(frame_id or "") or None,
            })

    @staticmethod
    def _source_received(now, stamp, ros_now, previous, *, receipt_fallback=False):
        """HH_261001 - Anchor source age to monotonic time across cached stamps."""
        if receipt_fallback and stamp is None and ros_now is None:
            return now
        stamp, ros_now = finite(stamp), finite(ros_now)
        if stamp is None or stamp <= 0 or ros_now is None or stamp > ros_now + 0.05:
            return None
        if (previous and previous.get("stamp") == stamp
                and previous.get("received") is not None):
            return previous.get("received")
        return now - max(0.0, ros_now - stamp)

    def platform(self, vx, vy, battery_percentage, *, yaw_rate_rps=None,
                 frame_id=None, velocity_stamp_s=None, ros_now_s=None):
        vx, vy = finite(vx), finite(vy)
        battery = finite(battery_percentage)
        with self._lock:
            now = self._now()
            previous = self._platform
            received = self._source_received(
                now, velocity_stamp_s, ros_now_s,
                {"stamp": previous["stamp"], "received": previous["motion_received"]}
                if previous else None, receipt_fallback=True,
            )
            self._platform = {
                "received": now, "motion_received": received,
                "stamp": finite(velocity_stamp_s),
                "speed": finite(math.hypot(vx, vy)) if vx is not None and vy is not None else None,
                "vx": vx, "vy": vy, "yaw_rate": finite(yaw_rate_rps),
                "frame_id": str(frame_id or "") or None,
                "battery": battery if battery is not None and 0 <= battery <= 100 else None,
            }

    def wheels(self, motor_speed, motor_angle, motor_rpm, *, stamp_s, ros_now_s):
        """HH_261001 - Expose reported channels without inventing wheel corners."""
        def channels(values, count):
            try:
                if isinstance(values, (str, bytes)) or len(values) != count:
                    return None
                result = [finite(value) for value in values]
            except TypeError:
                return None
            return result if all(value is not None for value in result) else None

        speeds = channels(motor_speed, 4)
        angles = channels(motor_angle, 4)
        rpm = channels(motor_rpm, 8)
        with self._lock:
            self._wheels = {
                "received": self._source_received(self._now(), stamp_s, ros_now_s, self._wheels),
                "stamp": finite(stamp_s),
                "speeds": speeds, "angles": angles, "rpm": rpm,
            }

    def fusion_objects(self, records, *, ros_now_s, deleted_ids=(), clear=False):
        """HH_261001 - Bound semantic centroids without treating glyphs as boxes."""
        with self._lock:
            now = self._now()
            if clear:
                self._objects.clear()
            for identity in list(deleted_ids)[:128]:
                self._objects.pop(str(identity), None)
            for record in records[:32]:
                identity = str(record.get("id", ""))[:80]
                if not identity:
                    continue
                previous = self._objects.get(identity)
                received = self._source_received(now, record.get("stamp_s"), ros_now_s, previous)
                coordinates = [finite(record.get(axis)) for axis in ("x", "y", "z")]
                if (received is None or record.get("frame_id") != "map"
                        or record.get("transform_available") is not True
                        or any(value is None or abs(value) > 1e8 for value in coordinates)):
                    self._objects.pop(identity, None)
                    continue
                lifetime = finite(record.get("lifetime_s"))
                # HH_261002 - Keep the passive display centroid for at least
                # 0.5 s: a 0.2 s fusion marker at about 5 Hz flickered on
                # network/TF jitter.  Cap the hold at 1 s; source age, map TF,
                # DELETE and DELETEALL checks remain independent and immediate.
                lifetime = (min(1.0, max(0.5, lifetime))
                            if lifetime is not None and lifetime > 0 else 1.0)
                label = record.get("class_name")
                label = label.strip()[:80] if isinstance(label, str) and label.strip() else None
                stamp = finite(record.get("stamp_s"))
                expires = (previous["expires"] if previous and previous["stamp"] == stamp
                           else now + lifetime)
                self._objects[identity] = {
                    "received": received, "stamp": stamp, "expires": expires,
                    "source_frame": record.get("source_frame", "map"),
                    "object": {"id": identity, "class_name": label,
                               "x": coordinates[0], "y": coordinates[1], "z": coordinates[2],
                               "confidence": None, "dimensions": None, "source": "camera_lidar_fusion"},
                }
            # HH_261001 - Keep expired entries as bounded timestamp tombstones, so repeated
            # cached markers cannot revive a marker whose lifetime has elapsed.
            if len(self._objects) > 32:
                self._objects = dict(sorted(self._objects.items(),
                                            key=lambda item: item[1]["received"], reverse=True)[:32])

    def observed_object_boxes(self, records, *, ros_now_s, deleted_ids=(), clear=False):
        """HH_261002 - Cache only explicit LiDAR extents, never legacy fixed-size glyphs.

        These boxes describe observed foreground returns, not the complete
        physical body of an occluded object. Each axis must be 0.02..30 metres;
        reject impossible/degenerate data rather than padding or rescaling it.
        """
        with self._lock:
            now = self._now()
            if clear:
                self._object_boxes.clear()
            for identity in list(deleted_ids)[:128]:
                self._object_boxes.pop(str(identity), None)
            for record in records[:32]:
                identity = str(record.get("id", ""))[:80]
                if not identity:
                    continue
                previous = self._object_boxes.get(identity)
                stamp = finite(record.get("stamp_s"))
                received = self._source_received(now, stamp, ros_now_s, previous)
                center = [finite(record.get(axis)) for axis in ("x", "y", "z")]
                size = record.get("size")
                size = ([finite(size.get(axis)) for axis in ("x", "y", "z")]
                        if isinstance(size, dict) else [])
                orientation = record.get("orientation")
                orientation = (transform_detection_orientation(
                    [orientation.get(axis) for axis in ("x", "y", "z", "w")])
                    if isinstance(orientation, dict) else None)
                if (received is None or record.get("frame_id") != "map"
                        or record.get("transform_available") is not True
                        or record.get("source") != "observed_lidar_extent"
                        or any(value is None or abs(value) > 1e8 for value in center)
                        or len(size) != 3 or any(value is None or not 0.02 <= value <= 30 for value in size)
                        or orientation is None):
                    self._object_boxes.pop(identity, None)
                    continue
                lifetime = finite(record.get("lifetime_s"))
                lifetime = min(1.0, max(0.5, lifetime)) if lifetime is not None and lifetime > 0 else 1.0
                self._object_boxes[identity] = {
                    "received": received, "stamp": stamp,
                    "source_frame": record.get("source_frame", "map"),
                    "expires": (previous["expires"] if previous and previous["stamp"] == stamp
                                else now + lifetime),
                    "bbox": {"center": dict(zip(("x", "y", "z"), center)),
                             "size": dict(zip(("x", "y", "z"), size)),
                             "orientation": dict(zip(("x", "y", "z", "w"), orientation)),
                             "source": "observed_lidar_extent"},
                }
            if len(self._object_boxes) > 32:
                self._object_boxes = dict(sorted(self._object_boxes.items(),
                    key=lambda item: item[1]["received"], reverse=True)[:32])

    def route(self, points, frame_id, mission, *, source_stamp_s=None):
        # HH_261001 - Bound work and JSON size while preserving the path endpoint.
        size = len(points)
        indices = (range(size) if size <= 500 else
                   (round(i * (size - 1) / 499) for i in range(500)))
        bounded = []
        for index in indices:
            x, y = finite(points[index][0]), finite(points[index][1])
            if x is not None and y is not None:
                bounded.append([x, y])
        with self._lock:
            source_stamp = finite(source_stamp_s)
            # HH_261001 - A delayed transient-local path from before the latest
            # snapped goal cannot become the route for that newly claimed goal.
            if (self._route_goal_cutover_stamp is not None and
                    (source_stamp is None or
                     source_stamp <= self._route_goal_cutover_stamp)):
                return
            previous = self._route
            self._route = (
                self._now(), mission_key(mission), bounded,
                str(frame_id or "") or None, source_stamp,
            )
            # HH_261001 - Progress received before this path cannot describe it.
            if previous is None or previous[1] != mission_key(mission) or previous[2] != bounded:
                self._progress.clear()

    def invalidate_route_before(self, goal_stamp_s):
        """HH_261001 - Retire the old path when a newer snapped goal is accepted.

        ROS topic ordering is not cross-topic ordering: a new path may reach us
        before its goal callback. Keep that path only when its producer stamp is
        strictly later than the goal stamp. A duplicate goal echo is filtered by
        the backend and does not repeatedly erase the same route.
        """
        cutover = finite(goal_stamp_s)
        with self._lock:
            self._route_goal_cutover_stamp = cutover if cutover is not None and cutover > 0 else None
            if self._route is not None and (
                self._route_goal_cutover_stamp is None or
                self._route[4] is None or
                self._route[4] <= self._route_goal_cutover_stamp
            ):
                self._route = None
                self._progress.clear()

    def progress(self, name, value, mission):
        if name not in PROGRESS_FIELDS:
            return
        with self._lock:
            self._progress[name] = (self._now(), mission_key(mission), finite(value))

    def diagnostics(self, statuses):
        """HH_261001 - Use named sensor reports; never infer health from silence."""
        aliases = {
            "gnss": ("gnss", "gps", "ublox"),
            "lidar": ("lidar", "vanjee"),
            "camera": ("camera", "econ_front", "econ_rear"),
            "radar": ("radar", "sen0592"),
        }
        levels = {}
        for status in statuses:
            name = str(status.get("name", "")).lower()
            level = status.get("level")
            if not isinstance(level, int):
                continue
            for sensor, names in aliases.items():
                if any(token in name for token in names):
                    levels[sensor] = max(levels.get(sensor, 0), level)
        with self._lock:
            now = self._now()
            for sensor, level in levels.items():
                label = {0: "DIAG OK", 1: "DIAG WARN", 2: "DIAG ERROR"}.get(level, "DIAG STALE")
                self._sensors[sensor] = (now, label)

    def snapshot(self, mission, *, perception=None, perception_received=None):
        now = self._now()
        with self._lock:
            pose = self._pose
            platform = self._platform
            wheels = self._wheels
            objects = list(self._objects.values())
            object_boxes = dict(self._object_boxes)
            base_map = list(self._base_map.values())
            base_map_received = self._base_map_received
            base_map_source = self._base_map_source
            map_areas = list(self._map_areas)
            map_areas_received = self._map_areas_received
            route = self._route
            progress = dict(self._progress)
            sensors = dict(self._sensors)

        def age(received):
            return round(max(0.0, now - received), 3) if received is not None else None

        pose_age = age(pose[0]) if pose else None
        platform_age = age(platform["received"]) if platform else None
        motion_age = age(platform["motion_received"]) if platform else None
        pose_live = bool(
            pose_age is not None and pose_age <= 2.5
            and all(pose[1][axis] is not None for axis in ("x", "y", "yaw"))
        )
        platform_live = platform_age is not None and platform_age <= 2.5
        motion_live = bool(motion_age is not None and motion_age <= 2.5
                           and platform and platform["speed"] is not None)
        pose_value = dict(pose[1]) if pose_live else {
            "x": None, "y": None, "yaw": None,
            "frame_id": pose[1]["frame_id"] if pose else None,
        }
        pose_value["age_s"] = pose_age
        speed = platform["speed"] if motion_live else None
        key = mission_key(mission)
        travel = mission.get("service_state_name") in TRAVEL_STATES
        route_age = age(route[0]) if route and route[1] == key else None
        route_live = bool(
            mission.get("active") is True and travel and route and route[1] == key
            and len(route[2]) >= 2 and pose_live
            and route[3] == pose_value["frame_id"] == "map"
        )
        result_progress = dict.fromkeys(PROGRESS_FIELDS)
        reason = "no_progress"
        progress_fresh = False
        # HH_261001 - RETURN_WITH_CARGO is a local maneuver until an actual
        # current-return path is received; without one it is not global travel.
        if not travel or (mission.get("service_state_name") == "RETURN_WITH_CARGO" and not route_live):
            reason = "waiting_or_maneuver"
        elif mission.get("phase") != "DRIVING":
            reason = "not_driving"
        elif not pose_live or not motion_live:
            reason = "stale_motion"
        elif not route_live:
            reason = "route_unavailable_or_stale"
        elif all(
            name in progress and progress[name][1] == key
            and age(progress[name][0]) <= 2.5
            and progress[name][2] is not None and progress[name][2] >= 0
            for name in PROGRESS_FIELDS
        ) and progress["completion_pct"][2] <= 100:
            progress_fresh = True
            result_progress.update({name: progress[name][2] for name in PROGRESS_FIELDS})
            if speed is None or speed < 0.05:
                # HH_261002 - A slow turn or brief physical stall must not erase
                # fresh route distance/completion.  ETA from a producer speed
                # floor is misleading while stopped, so hide only the ETA.
                result_progress["remaining_time_s"] = None
                reason = "stopped"
            else:
                reason = "estimated_global_travel"
        elif speed is None or speed < 0.05:
            reason = "stopped"
        result_progress.update(valid=progress_fresh, reason=reason)

        # HH_261001 - Existing telemetry is XY-only; unknown Z stays null. Incompatible,
        # untransformed or stale cached clouds are unavailable, never CLEAR.
        cloud = perception or {}
        cloud_age = age(perception_received)
        cloud_live = bool(
            cloud_age is not None and cloud_age <= 2.5
            and cloud.get("frame_id") == "map"
            and cloud.get("transform_available") is True and pose_live
        )
        cloud_points = []
        if cloud_live:
            for point in cloud.get("points", [])[:240]:
                if len(point) >= 2 and finite(point[0]) is not None and finite(point[1]) is not None:
                    cloud_points.append([point[0], point[1], finite(point[2]) if len(point) > 2 else None])

        live_objects = []
        if pose_live and pose_value["frame_id"] == "map":
            for entry in objects:
                object_age = age(entry["received"])
                if object_age <= 1.0 and now <= entry["expires"]:
                    item = {**entry["object"], "age_s": object_age, "bbox": None,
                            "orientation": None, "geometry_source": None}
                    box = object_boxes.get(item["id"])
                    # HH_261002 - Independent DDS topics may arrive in either order.
                    # Only the same detection stamp, ID and source frame can join;
                    # a new centroid must never inherit an older object's extent.
                    if (box and box["stamp"] == entry["stamp"]
                            and box["source_frame"] == entry["source_frame"]
                            and age(box["received"]) <= 1.0 and now <= box["expires"]):
                        item["bbox"] = box["bbox"]
                        item["dimensions"] = box["bbox"]["size"]
                        item["orientation"] = box["bbox"]["orientation"]
                        item["geometry_source"] = box["bbox"]["source"]
                    live_objects.append(item)
        object_age = min((item["age_s"] for item in live_objects), default=None)

        sensor_result = {}
        for sensor in ("gnss", "lidar", "camera", "radar"):
            report = sensors.get(sensor)
            report_age = age(report[0]) if report else None
            sensor_result[sensor] = {
                "label": (report[1] if report_age <= 5.0 else "STALE") if report else "NO DATA",
                "age_s": report_age,
            }

        wheel_age = age(wheels["received"]) if wheels else None
        wheel_reason = "no_data"
        if wheels:
            if wheel_age is None:
                wheel_reason = "invalid_timestamp"
            elif wheel_age > 2.5:
                wheel_reason = "stale"
            elif any(wheels[name] is None for name in ("speeds", "angles", "rpm")):
                wheel_reason = "invalid_channels"
            else:
                wheel_reason = "platform_reported_mapping_unverified"
        wheel_live = wheel_reason == "platform_reported_mapping_unverified"

        return {
            "schema_version": 1,
            "connected": bool(pose_live and motion_live),
            "mission": dict(mission),
            "motion": {
                "speed_mps": speed,
                "vx_mps": platform["vx"] if motion_live else None,
                "vy_mps": platform["vy"] if motion_live else None,
                "yaw_rate_rps": platform["yaw_rate"] if motion_live else None,
                "frame_id": platform["frame_id"] if platform else None,
                "age_s": motion_age,
            },
            "wheel_telemetry": {
                "valid": wheel_live, "source": "platform/status",
                "mapping_status": "unverified", "reason": wheel_reason,
                "age_s": wheel_age, "source_stamp_s": wheels["stamp"] if wheels else None,
                "motor_speed_mps": wheels["speeds"] if wheel_live else [],
                "motor_angle_rad": wheels["angles"] if wheel_live else [],
                "motor_rpm": wheels["rpm"] if wheel_live else [],
            },
            "battery": {"percentage": platform["battery"] if platform_live else None},
            "pose": pose_value,
            "base_map": {
                "valid": bool(base_map or map_areas), "frame_id": "map",
                "polylines": base_map, "age_s": age(base_map_received),
                "source": base_map_source,
                "areas": map_areas, "areas_age_s": age(map_areas_received),
                "areas_source": "configured_map_catalogs",
            },
            "route": {
                # HH_261001 - This is a mission-bound latched path, not a 30 s
                # sensor stream. Keep its true receipt age visible for auditing.
                "valid": route_live,
                "points": route[2] if route_live else [],
                "frame_id": route[3] if route and route[1] == key else None,
                "age_s": route_age,
            },
            "progress": result_progress,
            "perception": {
                "points": cloud_points, "objects": live_objects,
                "frame_id": "map" if cloud_live or live_objects else None,
                "age_s": cloud_age if cloud_live else object_age,
                "points_age_s": cloud_age, "objects_age_s": object_age,
                "objects_source": "camera_lidar_fusion",
            },
            "sensors": sensor_result,
        }
