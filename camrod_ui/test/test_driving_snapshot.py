"""HH_261001 - Test passive display data; these are not road-test evidence."""

import ast
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
import yaml


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))

from camrod_ui.driving_snapshot import (  # noqa: E402
    BASE_MAP_MAX_LINES, BASE_MAP_MAX_POINTS, BASE_MAP_NAMESPACES,
    MAP_AREA_MAX_COUNT, MAP_AREA_MAX_VERTICES,
    DrivingSnapshotCache, sample_indices,
    transform_detection_orientation, transform_detection_point, validated_map_polygon,
)


class DrivingSnapshotTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.cache = DrivingSnapshotCache(now_fn=lambda: self.now)
        self.mission = {
            "active": True, "generation": 7, "intent": "delivery", "site": "B2",
            "service_state_name": "MOVING_TO_SITE", "phase": "DRIVING",
            "description": "Moving to site",
        }

    def live(self):
        self.cache.pose(5, 1, math.pi / 2, "map")
        self.cache.platform(0.3, 0.4, 75)
        self.cache.route([[5, 1], [10, 1]], "map", self.mission)
        self.progress()

    def progress(self):
        for field, value in (
            ("remaining_distance_m", 5), ("remaining_time_s", 10), ("completion_pct", 50),
        ):
            self.cache.progress(field, value, self.mission)

    def test_empty_is_unknown_not_clear_zero_or_connected(self):
        snap = self.cache.snapshot(self.mission)
        self.assertFalse(snap["connected"])
        self.assertIsNone(snap["motion"]["speed_mps"])
        self.assertIsNone(snap["battery"]["percentage"])
        self.assertIsNone(snap["pose"]["yaw"])
        self.assertEqual(snap["route"]["points"], [])
        self.assertEqual(snap["sensors"]["camera"], {"label": "NO DATA", "age_s": None})
        self.assertFalse(snap["progress"]["valid"])
        self.assertIsNone(snap["perception"]["frame_id"])
        json.dumps(snap, allow_nan=False)

    def test_static_base_map_survives_idle_new_goal_and_stale_pose(self):
        # HH_261002 - Static roads are not an active mission's planned route.
        self.live()
        self.cache.base_map([{"namespace": "lanelet/centerline", "id": 1,
                              "frame_id": "map", "points": [[0, 0], [10, 0]]}])
        self.cache.invalidate_route_before(101.0)
        self.now += 86400
        changed = {**self.mission, "generation": 8, "site": "B9"}
        for mission in (self.mission, changed, {"active": False}):
            snapshot = self.cache.snapshot(mission)
            self.assertFalse(snapshot["route"]["valid"])
            self.assertEqual(snapshot["base_map"], {
                "valid": True, "frame_id": "map", "age_s": 86400,
                "source": "/map/markers", "polylines": [{
                    "namespace": "lanelet/centerline", "marker_id": 1,
                    "points": [[0.0, 0.0], [10.0, 0.0]],
                }],
                "areas": [], "areas_age_s": None,
                "areas_source": "configured_map_catalogs",
            })

    def test_authored_areas_are_permanent_and_independent_of_marker_roads_and_missions(self):
        # HH_261002 - These explicit test coordinates stand in for authored map
        # corners, never inferred rectangles around dispatch goal coordinates.
        points = [[2, 3], [5, 3], [5, 7], [2, 7], [2, 3]]
        site = {"id": "camping_site_9", "kind": "camping_site", "site": "B9",
                "frame_id": "map", "source": "camping_sites_yaml", "points": points}
        zone = {**site, "id": "dz_area_7144", "kind": "drop_zone", "site": None,
                "source": "drop_zones_yaml"}
        self.cache.map_areas([site, zone])
        self.cache.base_map([], clear=True)
        self.now += 86400
        for mission in ({"active": False}, self.mission, {**self.mission, "site": "B13", "generation": 9}):
            result = self.cache.snapshot(mission)["base_map"]
            self.assertTrue(result["valid"])
            self.assertEqual(result["polylines"], [])
            self.assertEqual(result["areas_age_s"], 86400)
            self.assertEqual(result["areas"][0], {
                "id": "camping_site_9", "kind": "camping_site", "site": "B9", "label": "B9",
                "points": points[:-1], "source": "camping_sites_yaml",
            })
            self.assertEqual(result["areas"][1]["label"], "드롭존")
        self.cache.map_areas([])
        self.assertFalse(self.cache.snapshot({})["base_map"]["valid"])

    def test_area_polygons_reject_invalid_geometry_without_sampling_or_guessing(self):
        valid = [[0, 0], [4, 0], [4, 3], [0, 3]]
        self.assertEqual(validated_map_polygon(valid), valid)
        self.assertEqual(validated_map_polygon(list(reversed(valid))), list(reversed(valid)))
        self.assertIsNone(validated_map_polygon([[0, 0], [2, 2], [0, 2], [2, 0]]))
        self.assertIsNone(validated_map_polygon([[0, 0], [4, 0], [1, 3], [3, -1], [0, 2]]))
        self.assertIsNone(validated_map_polygon([[0, 0], [1, 1], [2, 2]]))
        self.assertIsNone(validated_map_polygon([[0, 0], [1, 0], [True, 2]]))
        self.assertIsNone(validated_map_polygon([[0, 0], [1, 0], [float("nan"), 2]]))
        self.assertIsNone(validated_map_polygon([[0, 0], [1, 0], [float("inf"), 2]]))
        self.assertIsNone(validated_map_polygon([[0, 0], [1, 0], [1, 1], [1, 0]]))
        self.assertIsNone(validated_map_polygon([[i, i % 2] for i in range(MAP_AREA_MAX_VERTICES + 2)]))

    def test_area_catalog_replacement_rejects_unknown_sources_frames_ids_and_overflow(self):
        site = {"id": "camping_site_1", "kind": "camping_site", "site": "B1",
                "frame_id": "map", "source": "camping_sites_yaml", "points": [[0, 0], [2, 0], [1, 1]]}
        self.cache.map_areas([site])
        self.assertTrue(self.cache.snapshot({})["base_map"]["valid"])
        for wrong in ({"frame_id": "odom"}, {"source": "inferred"}, {"site": "B99"},
                      {"kind": "unknown"}, {"id": ""}, {"points": []}):
            self.cache.map_areas([{**site, **wrong}])
            self.assertEqual(self.cache.snapshot({})["base_map"]["areas"], [])
        self.cache.map_areas([site, site])
        self.assertEqual(self.cache.snapshot({})["base_map"]["areas"], [])
        self.cache.map_areas([{**site, "id": f"site{index}"} for index in range(MAP_AREA_MAX_COUNT + 1)])
        self.assertEqual(self.cache.snapshot({})["base_map"]["areas"], [])

    def test_backend_area_loader_uses_only_exact_configured_map_corners(self):
        source = (Path(__file__).resolve().parents[1] / "runtime/python/camrod_ui/ui_backend_node.py").read_text()
        method = next(node for node in ast.walk(ast.parse(source)) if isinstance(node, ast.FunctionDef)
                      and node.name == "_load_driving_map_areas")
        namespace = {"List": list, "Dict": dict, "Any": object, "Path": Path, "yaml": yaml,
                     "MAP_AREA_MAX_COUNT": MAP_AREA_MAX_COUNT, "MAP_AREA_MAX_VERTICES": MAP_AREA_MAX_VERTICES}
        exec(ast.unparse(method), namespace)
        loader = namespace["_load_driving_map_areas"]
        warnings = []
        with tempfile.TemporaryDirectory() as directory:
            camping = Path(directory) / "camping.yaml"
            drops = Path(directory) / "drops.yaml"
            corners = [{"x": 3, "y": 4}, {"x": 7, "y": 4}, {"x": 7, "y": 8}, {"x": 3, "y": 8}]
            camping.write_text(yaml.safe_dump({"camping_sites": [
                {"type": "camping_site_1", "x": 999, "y": 999, "corners": corners},
                {"type": "camping_site_2", "x": 6, "y": 7},
                {"type": "camping_site_3", "frame_id": "unknown", "corners": corners},
            ]}))
            drops.write_text(yaml.safe_dump({"drop_zones": [
                {"id": "authored_station", "type": "drop_zone", "corners": corners},
            ]}))
            backend = SimpleNamespace(camping_sites_yaml=str(camping), drop_zones_yaml=str(drops),
                default_goal_frame_id="map", get_logger=lambda: SimpleNamespace(warning=warnings.append))
            self.cache.map_areas(loader(backend))
            areas = self.cache.snapshot({})["base_map"]["areas"]
            self.assertEqual([area["id"] for area in areas], ["camping_site_1", "authored_station"])
            self.assertEqual(areas[0]["points"], [[3, 4], [7, 4], [7, 8], [3, 8]])
            self.assertEqual(warnings, [])
            backend.camping_sites_yaml = str(Path(directory) / "missing.yaml")
            self.cache.map_areas(loader(backend))
            self.assertEqual([area["id"] for area in self.cache.snapshot({})["base_map"]["areas"]], ["authored_station"])
            self.assertEqual(len(warnings), 1)
        self.assertNotIn("publish", ast.unparse(method))
        self.assertNotIn("_keypoints_by_mission_key", ast.unparse(method))
        self.assertNotIn("_drop_zone_polygons", ast.unparse(method))

    def test_static_map_public_marker_ids_survive_updates_and_respect_namespace(self):
        # HH_261002 - IDs remain producer identities, not array offsets after
        # deleted/invalid lines or refreshed marker ordering.
        left = {"namespace": "lanelet/left_bound", "id": 17, "frame_id": "map",
                "points": [[0, 1], [10, 1]]}
        right = {**left, "namespace": "lanelet/right_bound", "id": 18,
                 "points": [[0, -1], [10, -1]]}
        self.cache.base_map([right, left])
        self.cache.base_map([{**left, "points": [[0, 2], [10, 2]]}])
        lines = self.cache.snapshot({})["base_map"]["polylines"]
        self.assertEqual({(line["namespace"], line["marker_id"]) for line in lines},
                         {("lanelet/left_bound", 17), ("lanelet/right_bound", 18)})
        self.assertTrue(all(type(line["marker_id"]) is int for line in lines))
        self.cache.base_map([{**right, "id": 17}])
        self.cache.base_map([], deleted_ids=[("lanelet/left_bound", 17)])
        lines = self.cache.snapshot({})["base_map"]["polylines"]
        self.assertEqual({(line["namespace"], line["marker_id"]) for line in lines},
                         {("lanelet/right_bound", 17), ("lanelet/right_bound", 18)})
        self.cache.base_map([{**left, "id": 30}], clear=True)
        self.assertEqual([line["marker_id"] for line in
                          self.cache.snapshot({})["base_map"]["polylines"]], [30])
        self.cache.base_map([{**left, "id": value} for value in (True, 1.5, "17", None)], clear=True)
        self.assertFalse(self.cache.snapshot({})["base_map"]["valid"])

    def test_static_base_map_update_delete_new_source_and_invalid_frame(self):
        first = {"namespace": "lanelet/left_bound", "id": 1, "frame_id": "map",
                 "points": [[0, 0], [10, 0]]}
        self.cache.base_map([first])
        self.cache.base_map([{**first, "points": [[0, 1], [10, 1]]}])
        self.assertEqual(len(self.cache.snapshot({})["base_map"]["polylines"]), 1)
        self.cache.base_map([{**first, "frame_id": "unknown"}])
        self.assertFalse(self.cache.snapshot({})["base_map"]["valid"])
        self.cache.base_map([first])
        self.cache.base_map([], deleted_ids=[("lanelet/left_bound", 1)])
        self.assertFalse(self.cache.snapshot({})["base_map"]["valid"])
        self.cache.base_map([first])
        self.cache.base_map([], source="/replacement/map")
        self.assertFalse(self.cache.snapshot({})["base_map"]["valid"])
        self.cache.base_map([first])
        self.cache.base_map([], clear=True)
        self.assertFalse(self.cache.snapshot({})["base_map"]["valid"])

    def test_static_base_map_geometry_is_bounded_finite_and_endpoint_preserving(self):
        records = [{"namespace": "lanelet/centerline", "id": identity,
                    "frame_id": "map", "points": [[x, identity] for x in range(700)]}
                   for identity in range(600)]
        self.cache.base_map(records)
        lines = self.cache.snapshot({})["base_map"]["polylines"]
        self.assertEqual(len(lines), BASE_MAP_MAX_LINES)
        self.assertLessEqual(sum(len(line["points"]) for line in lines), BASE_MAP_MAX_POINTS)
        for identity, line in enumerate(lines):
            self.assertEqual(line["points"][0], [0, identity])
            self.assertEqual(line["points"][-1], [699, identity])
        self.cache.base_map([
            {**records[0], "points": [[0, 0], [float("nan"), 1]]},
            {**records[1], "points": [[True, 0], [1, 0]]},
            {**records[2], "namespace": "semantic/area"},
        ], clear=True)
        self.assertFalse(self.cache.snapshot({})["base_map"]["valid"])
        json.dumps(self.cache.snapshot({}), allow_nan=False)

    def test_static_map_callback_obeys_marker_pose_clear_delete_and_frame_contract(self):
        # HH_261002 - Execute the actual callback without a ROS runtime or lease.
        source = (Path(__file__).resolve().parents[1] / "runtime/python/camrod_ui/ui_backend_node.py").read_text()
        method = next(node for node in ast.walk(ast.parse(source)) if isinstance(node, ast.FunctionDef)
                      and node.name == "_on_driving_base_map")
        namespace = {"MarkerArray": object,
                     "Marker": SimpleNamespace(ADD=0, DELETE=2, DELETEALL=3, LINE_STRIP=4),
                     "BASE_MAP_NAMESPACES": BASE_MAP_NAMESPACES,
                     "BASE_MAP_MAX_POINTS": BASE_MAP_MAX_POINTS,
                     "BASE_MAP_MAX_LINES": BASE_MAP_MAX_LINES,
                     "sample_indices": sample_indices,
                     "transform_detection_point": transform_detection_point}
        exec(ast.unparse(method), namespace)
        callback = namespace["_on_driving_base_map"]
        backend = SimpleNamespace(_driving=self.cache, telemetry_topics={"map_markers": "/map/markers"})

        def marker(identity=1, action=0, frame="map", ns="lanelet/centerline"):
            return SimpleNamespace(ns=ns, id=identity, action=action, type=4,
                header=SimpleNamespace(frame_id=frame),
                points=[SimpleNamespace(x=1, y=0, z=1), SimpleNamespace(x=2, y=0, z=1)],
                pose=SimpleNamespace(position=SimpleNamespace(x=10, y=20, z=3),
                                     orientation=SimpleNamespace(x=0, y=0, z=0, w=0)))

        def apply(*markers):
            callback(backend, SimpleNamespace(markers=list(markers)))
            return self.cache.snapshot({})["base_map"]

        self.assertEqual(apply(marker())["polylines"][0]["points"], [[11, 20], [12, 20]])
        rotated = marker()
        rotated.pose.orientation = SimpleNamespace(x=0, y=math.sqrt(0.5), z=0, w=math.sqrt(0.5))
        # Pitch rotates actual Z into map X; this is not a yaw-only approximation.
        for point in apply(rotated)["polylines"][0]["points"]:
            self.assertAlmostEqual(point[0], 11)
            self.assertAlmostEqual(point[1], 20)
        self.assertFalse(apply(marker(action=2))["valid"])
        apply(marker())
        self.assertFalse(apply(marker(frame="unknown"))["valid"])
        apply(marker())
        self.assertFalse(apply(marker(action=3, ns="lanelet/clear"))["valid"])
        apply(marker())
        replacement = apply(marker(action=3, ns="lanelet/clear"), marker(identity=2))
        self.assertEqual(len(replacement["polylines"]), 1)
        self.assertEqual(set(self.cache._base_map), {("lanelet/centerline", 2)})
        self.assertFalse(apply()["valid"])
        self.assertFalse(apply(marker(ns="lanelet/area"))["valid"])
        invalid = marker()
        invalid.pose.orientation.w = float("nan")
        self.assertFalse(apply(invalid)["valid"])
        apply(marker())
        self.assertFalse(apply(*[marker() for _ in range(4097)])["valid"])
        self.assertNotIn("_telemetry_map", ast.unparse(method))
        self.assertNotIn("_driving_mission", ast.unparse(method))
        self.assertNotIn("publish", ast.unparse(method))

    def test_static_map_subscriber_is_always_on_reliable_transient_local(self):
        source = (Path(__file__).resolve().parents[1] / "runtime/python/camrod_ui/ui_backend_node.py").read_text()
        tree = ast.parse(source)
        subscription = next(node for node in ast.walk(tree) if isinstance(node, ast.Call)
                            and isinstance(node.func, ast.Attribute)
                            and node.func.attr == "create_subscription"
                            and len(node.args) >= 4
                            and isinstance(node.args[2], ast.Attribute)
                            and node.args[2].attr == "_on_driving_base_map")
        self.assertEqual(ast.unparse(subscription.args[1]), "self.telemetry_topics['map_markers']")
        qos = ast.unparse(subscription.args[3])
        self.assertIn("depth=1", qos)
        self.assertIn("ReliabilityPolicy.RELIABLE", qos)
        self.assertIn("DurabilityPolicy.TRANSIENT_LOCAL", qos)
        initializer = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                           and node.name == "__init__" and subscription in list(ast.walk(node)))
        self.assertIn("self._driving_subscriptions.append", ast.unparse(initializer))

    def test_live_values_are_measured_and_eta_explicitly_estimated(self):
        self.live()
        snap = self.cache.snapshot(self.mission)
        self.assertTrue(snap["connected"])
        self.assertEqual(snap["motion"]["speed_mps"], 0.5)
        self.assertEqual(snap["pose"]["yaw"], math.pi / 2)
        self.assertEqual(snap["battery"]["percentage"], 75)
        self.assertTrue(snap["progress"]["valid"])
        self.assertEqual(snap["progress"]["reason"], "estimated_global_travel")
        self.assertEqual(snap["progress"]["remaining_distance_m"], 5)

    def test_one_shot_latched_path_survives_thirty_seconds_in_same_mission_leg(self):
        # HH_261001 - /planning/global_path is one-shot transient-local; its
        # receipt age is audit data, not a sensor-style freshness deadline.
        self.live()
        self.now += 65
        self.cache.pose(6, 1, 0, "map")
        self.cache.platform(0.3, 0, 75)
        snap = self.cache.snapshot(self.mission)
        self.assertTrue(snap["route"]["valid"])
        self.assertEqual(snap["route"]["points"], [[5.0, 1.0], [10.0, 1.0]])
        self.assertEqual(snap["route"]["age_s"], 65)
        self.assertFalse(snap["progress"]["valid"])

    def test_path_published_during_departure_remains_bound_to_outbound_leg(self):
        # HH_261001 - The path can arrive before the service heartbeat changes
        # from a drop-zone departure to MOVING_TO_SITE.
        departing = {**self.mission, "service_state_name": "DEPARTING_DROP_ZONE"}
        self.cache.pose(5, 1, 0, "map")
        self.cache.platform(0.3, 0, 75)
        self.cache.route([[5, 1], [10, 1]], "map", departing)
        self.assertFalse(self.cache.snapshot(departing)["route"]["valid"])
        self.assertTrue(self.cache.snapshot(self.mission)["route"]["valid"])
        self.assertFalse(self.cache.snapshot({**self.mission, "service_state_name": "RETURNING_TO_DROP_ZONE"})["route"]["valid"])

    def test_new_return_path_can_display_before_return_with_cargo_state_changes(self):
        # HH_261001 - Nav2 can have the return path while the service state is
        # still RETURN_WITH_CARGO after the local maneuver reports DONE.
        self.live()
        returning = {**self.mission, "service_state_name": "RETURN_WITH_CARGO"}
        self.assertFalse(self.cache.snapshot(returning)["route"]["valid"])
        self.cache.route([[10, 1], [5, 1]], "map", returning)
        self.assertTrue(self.cache.snapshot(returning)["route"]["valid"])
        self.assertTrue(self.cache.snapshot({**returning, "service_state_name": "RETURNING_TO_DROP_ZONE"})["route"]["valid"])

    def test_new_goal_retires_old_path_and_rejects_late_transient_sample(self):
        self.cache.pose(5, 1, 0, "map")
        self.cache.platform(0.3, 0, 75)
        self.cache.route([[5, 1], [10, 1]], "map", self.mission, source_stamp_s=90)
        self.assertTrue(self.cache.snapshot(self.mission)["route"]["valid"])
        self.cache.invalidate_route_before(91)
        self.assertFalse(self.cache.snapshot(self.mission)["route"]["valid"])
        self.cache.route([[5, 1], [10, 1]], "map", self.mission, source_stamp_s=90)
        self.assertEqual(self.cache.snapshot(self.mission)["route"]["points"], [])
        self.cache.route([[5, 1], [11, 2]], "map", self.mission, source_stamp_s=92)
        self.assertEqual(self.cache.snapshot(self.mission)["route"]["points"], [[5.0, 1.0], [11.0, 2.0]])

    def test_goal_callback_after_new_path_does_not_erase_that_path(self):
        # HH_261001 - DDS gives no ordering guarantee across path/goal topics.
        self.cache.pose(5, 1, 0, "map")
        self.cache.platform(0.3, 0, 75)
        self.cache.route([[5, 1], [11, 2]], "map", self.mission, source_stamp_s=92)
        self.cache.invalidate_route_before(91)
        self.assertTrue(self.cache.snapshot(self.mission)["route"]["valid"])
        self.assertEqual(self.cache.snapshot(self.mission)["route"]["points"], [[5.0, 1.0], [11.0, 2.0]])

    def test_inactive_mission_cannot_retain_route_from_active_identity(self):
        self.live()
        inactive = {**self.mission, "active": False}
        self.assertFalse(self.cache.snapshot(inactive)["route"]["valid"])
        self.assertEqual(self.cache.snapshot(inactive)["route"]["points"], [])

    def test_stopped_robot_keeps_fresh_distance_but_never_uses_floor_speed_eta(self):
        self.live()
        self.cache.platform(0, 0, 75)
        progress = self.cache.snapshot(self.mission)["progress"]
        # HH_261002 - Zero speed is not a route disconnect.  Keep measured
        # mission progress visible while declining a moving-speed ETA.
        self.assertTrue(progress["valid"])
        self.assertEqual(progress["reason"], "stopped")
        self.assertEqual(progress["remaining_distance_m"], 5)
        self.assertEqual(progress["completion_pct"], 50)
        self.assertIsNone(progress["remaining_time_s"])

    def test_stopped_robot_without_fresh_progress_does_not_invent_distance(self):
        self.cache.pose(5, 1, 0, "map")
        self.cache.platform(0, 0, 75)
        self.cache.route([[5, 1], [10, 1]], "map", self.mission)
        progress = self.cache.snapshot(self.mission)["progress"]
        self.assertFalse(progress["valid"])
        self.assertEqual(progress["reason"], "stopped")
        self.assertIsNone(progress["remaining_distance_m"])

    def test_signed_motion_and_yaw_rate_preserve_actual_units_and_frame(self):
        self.cache.pose(5, 1, math.pi / 2, "map")
        self.cache.platform(-0.3, 0.4, 75, yaw_rate_rps=-0.25,
                            frame_id="robot_center_link", velocity_stamp_s=90,
                            ros_now_s=90.2)
        self.now += 0.3
        snap = self.cache.snapshot(self.mission)
        self.assertAlmostEqual(snap["motion"]["speed_mps"], 0.5)
        self.assertEqual(snap["motion"]["vx_mps"], -0.3)
        self.assertEqual(snap["motion"]["vy_mps"], 0.4)
        self.assertEqual(snap["motion"]["yaw_rate_rps"], -0.25)
        self.assertEqual(snap["motion"]["frame_id"], "robot_center_link")
        self.assertAlmostEqual(snap["motion"]["age_s"], 0.5)
        self.assertEqual(snap["pose"]["yaw"], math.pi / 2)

    def test_duplicate_source_stamp_cannot_refresh_cached_motion(self):
        self.live()
        self.cache.platform(0.5, 0, 75, yaw_rate_rps=0.2,
                            velocity_stamp_s=90, ros_now_s=90)
        self.now += 3
        self.cache.pose(6, 1, 0, "map")
        # HH_261001 - Aggregate heartbeats or paused ROS time cannot refresh a source.
        self.cache.platform(0.5, 0, 76, yaw_rate_rps=0.2,
                            velocity_stamp_s=90, ros_now_s=90)
        snap = self.cache.snapshot(self.mission)
        self.assertFalse(snap["connected"])
        for field in ("speed_mps", "vx_mps", "vy_mps", "yaw_rate_rps"):
            self.assertIsNone(snap["motion"][field])
        self.assertEqual(snap["motion"]["age_s"], 3)
        self.assertEqual(snap["battery"]["percentage"], 76)

    def test_unknown_or_future_motion_source_stamp_does_not_invent_zero_motion(self):
        for stamp in (None, 0, -1, float("nan"), 95):
            with self.subTest(stamp=stamp):
                self.cache.platform(0, 0, 75, yaw_rate_rps=0,
                                    velocity_stamp_s=stamp, ros_now_s=90)
                motion = self.cache.snapshot(self.mission)["motion"]
                self.assertIsNone(motion["speed_mps"])
                self.assertIsNone(motion["yaw_rate_rps"])
                self.assertIsNone(motion["age_s"])

    def test_null_nonfinite_and_malformed_motion_remain_unknown(self):
        for bad in (None, float("nan"), float("inf"), "bad", True, 10 ** 1000):
            with self.subTest(value=str(bad)[:25]):
                self.cache.platform(bad, 0.4, 75, yaw_rate_rps=bad)
                snap = self.cache.snapshot(self.mission)
                self.assertIsNone(snap["motion"]["speed_mps"])
                self.assertIsNone(snap["motion"]["vx_mps"])
                self.assertIsNone(snap["motion"]["yaw_rate_rps"])
                json.dumps(snap, allow_nan=False)
        self.cache.platform(1.7e308, 1.7e308, 75)
        json.dumps(self.cache.snapshot(self.mission), allow_nan=False)

    def test_measured_raw_wheel_channels_require_stamp_and_do_not_claim_corners(self):
        self.cache.wheels([-0.2, 0.3, 0.4, -0.5], [-0.1, 0.2, 0.3, -0.4],
                          list(range(8)), stamp_s=90, ros_now_s=90.1)
        wheels = self.cache.snapshot(self.mission)["wheel_telemetry"]
        self.assertTrue(wheels["valid"])
        self.assertEqual(wheels["mapping_status"], "unverified")
        self.assertEqual(wheels["motor_speed_mps"], [-0.2, 0.3, 0.4, -0.5])
        self.assertEqual(wheels["motor_angle_rad"], [-0.1, 0.2, 0.3, -0.4])
        self.assertEqual(wheels["motor_rpm"], list(range(8)))
        self.assertAlmostEqual(wheels["age_s"], 0.1)
        self.assertNotIn("front_left", wheels)

    def test_repeated_wheel_stamp_stays_stale_despite_new_aggregate_receipt(self):
        self.cache.wheels([0.2] * 4, [0.1] * 4, [10] * 8, stamp_s=90, ros_now_s=90)
        self.now += 3
        self.cache.wheels([0.2] * 4, [0.1] * 4, [10] * 8, stamp_s=90, ros_now_s=90)
        wheels = self.cache.snapshot(self.mission)["wheel_telemetry"]
        self.assertFalse(wheels["valid"])
        self.assertEqual(wheels["reason"], "stale")
        self.assertEqual(wheels["motor_speed_mps"], [])
        self.assertEqual(wheels["motor_angle_rad"], [])
        self.assertEqual(wheels["motor_rpm"], [])

    def test_invalid_wheel_timestamp_suppresses_even_plausible_channels(self):
        for stamp in (None, 0, -1, float("inf"), 95):
            self.cache.wheels([0.2] * 4, [0.1] * 4, [10] * 8,
                              stamp_s=stamp, ros_now_s=90)
            wheels = self.cache.snapshot(self.mission)["wheel_telemetry"]
            self.assertFalse(wheels["valid"])
            self.assertEqual(wheels["reason"], "invalid_timestamp")
            self.assertEqual(wheels["motor_angle_rad"], [])
            json.dumps(wheels, allow_nan=False)

    def test_malformed_wheel_channels_are_not_padded_with_fake_zero_values(self):
        for speeds, angles, rpm in (
            (None, [0] * 4, [0] * 8), ([0] * 3, [0] * 4, [0] * 8),
            ([0] * 4, [float("nan")] * 4, [0] * 8),
            ([0] * 4, [0] * 4, [float("inf")] * 8),
            ("1234", [0] * 4, [0] * 8), ([0] * 4, [0] * 4, 5),
        ):
            self.cache.wheels(speeds, angles, rpm, stamp_s=90, ros_now_s=90)
            wheels = self.cache.snapshot(self.mission)["wheel_telemetry"]
            self.assertFalse(wheels["valid"])
            self.assertEqual(wheels["reason"], "invalid_channels")
            self.assertEqual(wheels["motor_speed_mps"], [])
            json.dumps(wheels, allow_nan=False)

    def test_existing_platform_callback_passes_real_signed_fields_without_new_subscriber(self):
        source = (Path(__file__).resolve().parents[1] / "runtime/python/camrod_ui/ui_backend_node.py").read_text()
        tree = ast.parse(source)
        method = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                      and node.name == "_on_platform_status_serialized")
        fragment = next(node for node in method.body if isinstance(node, ast.If)
                        and ast.unparse(node.test) == "driving is not None")
        header = SimpleNamespace(frame_id="robot_center_link", stamp=SimpleNamespace(sec=90, nanosec=0))
        message = SimpleNamespace(
            velocity=SimpleNamespace(header=header, twist=SimpleNamespace(
                linear=SimpleNamespace(x=-0.3, y=0.4), angular=SimpleNamespace(z=-0.2))),
            wheel=SimpleNamespace(header=header), battery_percentage=0.75, battery_state_available=True,
            motor_speed=[0.1, 0.2, 0.3, 0.4], motor_angle=[-0.1, 0.2, -0.3, 0.4], motor_rpm=list(range(8)),
        )
        namespace = {"self": SimpleNamespace(_now_s=lambda: 90.1), "msg": message,
                     "driving": self.cache,
                     "_ros_stamp_seconds": lambda stamp: stamp.sec + stamp.nanosec * 1e-9 if stamp else None}
        exec(ast.unparse(fragment), namespace)
        snap = self.cache.snapshot(self.mission)
        self.assertEqual(snap["motion"]["vx_mps"], -0.3)
        self.assertEqual(snap["motion"]["yaw_rate_rps"], -0.2)
        self.assertTrue(snap["wheel_telemetry"]["valid"])
        self.assertNotIn("create_subscription", ast.unparse(fragment))
        self.assertNotIn("publish", ast.unparse(fragment))

    def test_wait_and_maneuver_do_not_inherit_global_travel_progress(self):
        self.live()
        for state in ("SITE_ENTRY", "RETURN_WITH_CARGO", "GUEST_LOADING_WAIT", "DROP_ZONE_PARKING"):
            with self.subTest(state=state):
                snap = self.cache.snapshot({**self.mission, "service_state_name": state})
                self.assertFalse(snap["progress"]["valid"])
                self.assertEqual(snap["progress"]["reason"], "waiting_or_maneuver")
                self.assertEqual(snap["route"]["points"], [])

    def test_safety_hold_hides_eta_even_with_residual_speed(self):
        self.live()
        snap = self.cache.snapshot({**self.mission, "phase": "SAFETY_STOP"})
        self.assertEqual(snap["progress"]["reason"], "not_driving")
        self.assertIsNone(snap["progress"]["remaining_time_s"])

    def test_stale_motion_hides_geometry_values_and_eta(self):
        self.live()
        self.now += 3
        snap = self.cache.snapshot(self.mission)
        self.assertFalse(snap["connected"])
        self.assertIsNone(snap["pose"]["x"])
        self.assertIsNone(snap["motion"]["speed_mps"])
        self.assertIsNone(snap["battery"]["percentage"])
        self.assertEqual(snap["route"]["points"], [])
        self.assertEqual(snap["progress"]["reason"], "stale_motion")

    def test_progress_needs_all_fresh_fields(self):
        self.live()
        self.now += 3
        self.cache.pose(6, 1, 0, "map")
        self.cache.platform(0.5, 0, 75)
        self.cache.progress("remaining_distance_m", 4, self.mission)
        self.assertFalse(self.cache.snapshot(self.mission)["progress"]["valid"])

    def test_frame_mismatch_hides_route(self):
        self.live()
        self.cache.pose(5, 1, 0, "odom")
        snap = self.cache.snapshot(self.mission)
        self.assertEqual(snap["route"]["points"], [])
        self.assertFalse(snap["progress"]["valid"])

    def test_missing_heading_does_not_establish_a_live_pose(self):
        self.live()
        self.cache.pose(5, 1, None, "map")
        snap = self.cache.snapshot(self.mission)
        self.assertFalse(snap["connected"])
        self.assertEqual(snap["route"]["points"], [])
        self.assertFalse(snap["progress"]["valid"])

    def test_route_and_progress_do_not_cross_generation_or_return_leg(self):
        self.live()
        for delta in (
            {"generation": 8}, {"site": "B3"}, {"intent": "recall"},
            {"service_state_name": "RETURNING_TO_DROP_ZONE", "intent": "return"},
        ):
            snap = self.cache.snapshot({**self.mission, **delta})
            self.assertEqual(snap["route"]["points"], [])
            self.assertFalse(snap["progress"]["valid"])

    def test_return_leg_keeps_original_intent_but_invalidates_outbound_route(self):
        self.live()
        returning = {**self.mission, "service_state_name": "RETURNING_TO_DROP_ZONE"}
        snap = self.cache.snapshot(returning)
        self.assertEqual(snap["mission"]["intent"], "delivery")
        self.assertEqual(snap["mission"]["generation"], 7)
        self.assertEqual(snap["route"]["points"], [])
        self.assertFalse(snap["progress"]["valid"])

    def test_backend_uses_exact_dispatch_identity_outside_nonreentrant_state_lock(self):
        # HH_261001 - Exercise the method without importing ROS or constructing a
        # node; the authoritative dispatch reader checks lock ordering.
        source = (Path(__file__).resolve().parents[1] / "runtime/python/camrod_ui/ui_backend_node.py").read_text()
        method = next(node for node in ast.walk(ast.parse(source)) if isinstance(node, ast.FunctionDef) and node.name == "_driving_mission")
        backend = SimpleNamespace(
            _lock=threading.Lock(),
            _state=SimpleNamespace(service_state_name="MOVING_TO_SITE", mission_phase="DRIVING", service_state_description="travel"),
        )
        dispatch = {
            "mission_dispatch_active": True, "mission_dispatch_generation": 17,
            "mission_dispatch_site": "B8", "mission_dispatch_intent": "recall",
        }

        def authoritative(node):
            self.assertFalse(node._lock.locked(), "dispatch reader takes the same nonreentrant lock")
            return dict(dispatch)

        namespace = {
            "UiBackendNode": SimpleNamespace(_mission_dispatch_snapshot=authoritative),
            "Dict": dict, "Any": object,
        }
        exec(ast.unparse(method), namespace)
        for state in ("MOVING_TO_SITE", "RETURN_WITH_CARGO", "RETURNING_TO_DROP_ZONE", "DROP_ZONE_PARKING", "DROP_ZONE_WAIT"):
            backend._state.service_state_name = state
            snapshot = namespace["_driving_mission"](backend)
            self.assertEqual(snapshot["active"], dispatch["mission_dispatch_active"])
            self.assertEqual(snapshot["generation"], 17)
            self.assertEqual(snapshot["site"], "B8")
            self.assertEqual(snapshot["intent"], "recall")
            self.assertEqual(snapshot["service_state_name"], state)
        dispatch.update(mission_dispatch_active=False, mission_dispatch_generation=0, mission_dispatch_site="", mission_dispatch_intent="")
        backend._state.service_state_name = "MOVING_TO_SITE"
        snapshot = namespace["_driving_mission"](backend)
        self.assertFalse(snapshot["active"])
        self.assertEqual(snapshot["generation"], 0)
        self.assertEqual(snapshot["site"], "")
        self.assertEqual(snapshot["intent"], "")

    def test_backend_connects_source_stamp_and_fresh_goal_invalidation(self):
        # HH_261001 - Verify the ROS adapter supplies producer time and retires
        # the old route only for a genuinely newer snapped goal.
        source = (Path(__file__).resolve().parents[1] / "runtime/python/camrod_ui/ui_backend_node.py").read_text()
        tree = ast.parse(source)
        path = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                    and node.name == "_on_driving_path")
        goal = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
                    and node.name == "_on_planning_route_goal")
        self.assertIn("source_stamp_s=_ros_stamp_seconds(message.header.stamp)", ast.unparse(path))
        self.assertIn("if stamp_key > previous_stamp:", ast.unparse(goal))
        self.assertIn("driving.invalidate_route_before", ast.unparse(goal))

    def test_route_is_bounded_and_republish_does_not_erase_progress(self):
        self.live()
        self.cache.route([[5, 1], [10, 1]], "map", self.mission)
        self.assertTrue(self.cache.snapshot(self.mission)["progress"]["valid"])
        self.cache.route([[i, i] for i in range(9000)], "map", self.mission)
        route = self.cache.snapshot(self.mission)["route"]
        self.assertEqual(len(route["points"]), 500)
        self.assertEqual(route["points"][-1], [8999, 8999])
        self.assertFalse(self.cache.snapshot(self.mission)["progress"]["valid"])

    def test_invalid_numbers_remain_json_safe(self):
        self.live()
        self.cache.platform(float("nan"), 0, float("inf"))
        self.cache.progress("remaining_time_s", float("inf"), self.mission)
        snap = self.cache.snapshot(self.mission)
        self.assertIsNone(snap["motion"]["speed_mps"])
        self.assertIsNone(snap["battery"]["percentage"])
        self.assertFalse(snap["progress"]["valid"])
        json.dumps(snap, allow_nan=False)

    def test_sensor_report_is_labeled_diagnostic_and_ages_out(self):
        self.cache.diagnostics([
            {"name": "sensing/camera/front", "level": 0},
            {"name": "sensing/camera/rear", "level": 1},
            {"name": "localization/gnss", "level": 2},
            {"name": "unrelated system", "level": 0},
        ])
        sensors = self.cache.snapshot(self.mission)["sensors"]
        self.assertEqual(sensors["camera"]["label"], "DIAG WARN")
        self.assertEqual(sensors["gnss"]["label"], "DIAG ERROR")
        self.assertEqual(sensors["lidar"]["label"], "NO DATA")
        self.now += 6
        self.assertEqual(self.cache.snapshot(self.mission)["sensors"]["camera"]["label"], "STALE")

    def test_perception_requires_fresh_explicit_transform_and_preserves_unknown_z(self):
        self.live()
        cloud = {"frame_id": "map", "points": [[6, 1]], "transform_available": True}
        snap = self.cache.snapshot(self.mission, perception=cloud, perception_received=self.now)
        self.assertEqual(snap["perception"]["points"], [[6, 1, None]])
        self.assertEqual(snap["perception"]["objects"], [])
        for invalid, received in (
            ({**cloud, "frame_id": "lidar"}, self.now),
            ({**cloud, "transform_available": False}, self.now),
            (cloud, self.now - 3),
        ):
            snap = self.cache.snapshot(self.mission, perception=invalid, perception_received=received)
            self.assertEqual(snap["perception"]["points"], [])
            self.assertIsNone(snap["perception"]["frame_id"])

    def test_get_endpoint_is_passive_and_not_an_operator_lease(self):
        source = (Path(__file__).resolve().parents[1] / "runtime/python/camrod_ui/ui_backend_node.py").read_text()
        tree = ast.parse(source)
        endpoint = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "get_driving")
        rendered = ast.unparse(endpoint)
        self.assertIn("@app.get('/api/driving')", rendered)
        self.assertNotIn("publish", rendered)
        snapshot = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "_snapshot_driving")
        self.assertNotIn("_request_telemetry_session", ast.unparse(snapshot))
        self.assertNotIn("_start_telemetry", ast.unparse(snapshot))

    def object_record(self, identity="fusion_det:0", **changes):
        return {"id": identity, "class_name": "person", "x": 7, "y": 1, "z": 0.8,
                "frame_id": "map", "transform_available": True, "stamp_s": 90,
                "lifetime_s": 0.2, **changes}

    def test_actual_objects_are_available_without_operator_cloud_or_lease(self):
        self.live()
        self.cache.fusion_objects([self.object_record()], ros_now_s=90.1)
        perception = self.cache.snapshot(self.mission)["perception"]
        self.assertEqual(perception["points"], [])
        self.assertEqual(perception["frame_id"], "map")
        self.assertEqual(perception["objects"][0]["class_name"], "person")
        self.assertEqual(perception["objects"][0]["z"], 0.8)
        self.assertIsNone(perception["objects"][0]["dimensions"])
        self.assertIsNone(perception["objects"][0]["confidence"])
        self.assertAlmostEqual(perception["objects"][0]["age_s"], 0.1)

    def test_object_freshness_does_not_rejuvenate_stale_pointcloud(self):
        self.live()
        self.cache.fusion_objects([self.object_record()], ros_now_s=90.1)
        cloud = {"frame_id": "map", "transform_available": True, "points": [[6, 2, 0.5]]}
        perception = self.cache.snapshot(self.mission, perception=cloud,
                                         perception_received=self.now - 3)["perception"]
        self.assertEqual(len(perception["objects"]), 1)
        self.assertEqual(perception["points"], [])
        self.assertEqual(perception["points_age_s"], 3)
        self.assertAlmostEqual(perception["objects_age_s"], 0.1)

    def test_each_object_expires_independently_and_duplicate_stamp_cannot_revive_it(self):
        self.live()
        self.cache.fusion_objects([self.object_record()], ros_now_s=90)
        self.now += 0.6
        self.cache.fusion_objects([self.object_record(), self.object_record("fusion_det:2", stamp_s=90.6)],
                                  ros_now_s=90.6)
        objects = self.cache.snapshot(self.mission)["perception"]["objects"]
        self.assertEqual([item["id"] for item in objects], ["fusion_det:2"])
        self.now += 0.6
        self.assertEqual(self.cache.snapshot(self.mission)["perception"]["objects"], [])

    def test_marker_lifetime_starts_at_receipt_but_source_age_is_also_bounded(self):
        self.live()
        self.cache.fusion_objects([self.object_record()], ros_now_s=90.3)
        self.assertEqual(len(self.cache.snapshot(self.mission)["perception"]["objects"]), 1)
        # HH_261002 - A 0.2 s source marker remains on the display through
        # the 5 Hz inter-frame gap, but never beyond the 0.5 s hold.
        self.now += 0.49
        self.assertEqual(len(self.cache.snapshot(self.mission)["perception"]["objects"]), 1)
        self.now += 0.02
        self.assertEqual(self.cache.snapshot(self.mission)["perception"]["objects"], [])

    def test_display_marker_hold_is_capped_at_one_second(self):
        # HH_261002 - A long or zero ROS marker lifetime cannot turn a
        # transient sensor object into an unbounded navigation glyph.
        self.live()
        self.cache.fusion_objects(
            [self.object_record(lifetime_s=5.0)], ros_now_s=90
        )
        self.assertAlmostEqual(self.cache._objects["fusion_det:0"]["expires"], self.now + 1.0)
        self.cache.fusion_objects(
            [self.object_record("fusion_det:2", lifetime_s=0.0)], ros_now_s=90
        )
        self.assertAlmostEqual(self.cache._objects["fusion_det:2"]["expires"], self.now + 1.0)

    def test_delete_and_deleteall_remove_only_current_objects(self):
        self.live()
        records = [self.object_record(), self.object_record("fusion_det:2")]
        self.cache.fusion_objects(records, ros_now_s=90)
        self.cache.fusion_objects([], ros_now_s=90, deleted_ids=["fusion_det:0"])
        self.assertEqual([item["id"] for item in self.cache.snapshot(self.mission)["perception"]["objects"]],
                         ["fusion_det:2"])
        self.cache.fusion_objects([], ros_now_s=90, clear=True)
        self.assertEqual(self.cache.snapshot(self.mission)["perception"]["objects"], [])

    def test_objects_require_actual_stamp_finite_xyz_and_confirmed_map_transform(self):
        self.live()
        for change in ({"stamp_s": None}, {"stamp_s": 0}, {"stamp_s": 92},
                       {"x": float("nan")}, {"z": None}, {"frame_id": "lidar"},
                       {"transform_available": False}):
            self.cache.fusion_objects([self.object_record(**change)], ros_now_s=90)
            snap = self.cache.snapshot(self.mission)
            self.assertEqual(snap["perception"]["objects"], [])
            json.dumps(snap, allow_nan=False)

    def test_object_count_is_bounded_and_missing_class_is_not_invented(self):
        self.live()
        self.cache.fusion_objects([self.object_record(f"fusion_det:{i * 2}", class_name=None)
                                   for i in range(500)], ros_now_s=90)
        objects = self.cache.snapshot(self.mission)["perception"]["objects"]
        self.assertEqual(len(objects), 32)
        self.assertTrue(all(item["class_name"] is None for item in objects))

    def test_full_3d_transform_preserves_height_and_rejects_invalid_quaternion(self):
        transformed = transform_detection_point([1, 0, 2], [10, 20, 3], [0, 0, math.sqrt(0.5), math.sqrt(0.5)])
        self.assertAlmostEqual(transformed[0], 10)
        self.assertAlmostEqual(transformed[1], 21)
        self.assertAlmostEqual(transformed[2], 5)
        self.assertIsNone(transform_detection_point([1, 0, None], [0, 0, 0], [0, 0, 0, 1]))
        self.assertIsNone(transform_detection_point([1, 0, 2], [0, 0, 0], [0, 0, 0, 0]))

    def observed_box(self, **changes):
        # HH_261002 - Values are explicit measured-extent fixtures, not road evidence.
        return {**self.object_record(), "source": "observed_lidar_extent",
                "size": {"x": 0.3, "y": 0.5, "z": 1.6},
                "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}, **changes}

    def test_observed_extent_joins_only_exact_identity_stamp_and_source_frame(self):
        self.live()
        self.cache.observed_object_boxes([self.observed_box()], ros_now_s=90.1)
        # An extent alone must not create a classified obstacle.
        self.assertEqual(self.cache.snapshot(self.mission)["perception"]["objects"], [])
        self.cache.fusion_objects([self.object_record()], ros_now_s=90.1)
        item = self.cache.snapshot(self.mission)["perception"]["objects"][0]
        self.assertEqual(item["bbox"]["size"], {"x": 0.3, "y": 0.5, "z": 1.6})
        self.assertEqual(item["dimensions"], item["bbox"]["size"])
        self.assertEqual(item["bbox"]["source"], "observed_lidar_extent")
        self.assertEqual(item["orientation"], item["bbox"]["orientation"])
        self.assertEqual(item["geometry_source"], "observed_lidar_extent")
        for changes in ({"stamp_s": 90.05}, {"id": "fusion_det:2"}, {"source_frame": "lidar"}):
            self.cache.observed_object_boxes([self.observed_box(**changes)], ros_now_s=90.1, clear=True)
            item = self.cache.snapshot(self.mission)["perception"]["objects"][0]
            self.assertIsNone(item["bbox"])
            self.assertIsNone(item["dimensions"])
            self.assertIsNone(item["orientation"])
            self.assertIsNone(item["geometry_source"])

    def test_observed_extent_arrival_after_centroid_and_deletion_preserve_centroid(self):
        self.live()
        self.cache.fusion_objects([self.object_record()], ros_now_s=90.1)
        self.cache.observed_object_boxes([self.observed_box(x=7.2, z=1.0)], ros_now_s=90.1)
        item = self.cache.snapshot(self.mission)["perception"]["objects"][0]
        self.assertEqual(item["x"], 7)
        self.assertEqual(item["bbox"]["center"], {"x": 7.2, "y": 1.0, "z": 1.0})
        self.cache.observed_object_boxes([], ros_now_s=90.1, deleted_ids=["fusion_det:0"])
        item = self.cache.snapshot(self.mission)["perception"]["objects"][0]
        self.assertEqual(item["class_name"], "person")
        self.assertIsNone(item["bbox"])

    def test_invalid_or_placeholder_extent_does_not_replace_centroid_with_fake_size(self):
        self.live()
        self.cache.fusion_objects([self.object_record()], ros_now_s=90.1)
        invalid = (
            {"size": {"x": 0, "y": 1, "z": 1}},
            {"size": {"x": 0.019, "y": 1, "z": 1}},
            {"size": {"x": 31, "y": 1, "z": 1}},
            {"size": {"x": float("nan"), "y": 1, "z": 1}},
            {"size": {"x": True, "y": 1, "z": 1}},
            {"size": [1, 1, 1]}, {"orientation": {"x": 0, "y": 0, "z": 0, "w": 0}},
            {"source": "fixed"}, {"transform_available": False}, {"frame_id": "lidar"},
            {"x": float("inf")}, {"stamp_s": 0}, {"stamp_s": 92},
        )
        for changes in invalid:
            self.cache.observed_object_boxes([self.observed_box(**changes)], ros_now_s=90.1)
            snapshot = self.cache.snapshot(self.mission)
            self.assertIsNone(snapshot["perception"]["objects"][0]["bbox"], changes)
            json.dumps(snapshot, allow_nan=False)

    def test_observed_extent_has_independent_bounded_age_and_hold(self):
        self.live()
        self.cache.fusion_objects([self.object_record(lifetime_s=1)], ros_now_s=90)
        self.cache.observed_object_boxes([self.observed_box()], ros_now_s=90)
        self.now += 0.6
        self.cache.observed_object_boxes([self.observed_box()], ros_now_s=90.6)
        item = self.cache.snapshot(self.mission)["perception"]["objects"][0]
        self.assertIsNone(item["bbox"])
        self.assertIsNone(item["dimensions"])
        self.assertIsNone(item["orientation"])
        self.assertIsNone(item["geometry_source"])
        self.assertEqual(item["class_name"], "person")

    def test_observed_extent_quaternion_is_full_tf_composition(self):
        half = math.sqrt(0.5)
        q = transform_detection_orientation((half, 0, 0, half), (0, 0, half, half))
        for value in q:
            self.assertAlmostEqual(value, 0.5)
        self.assertEqual(transform_detection_orientation((0, 0, 0, 2)), [0, 0, 0, 1])
        for q in ((0, 0, 0, 0), (float("nan"), 0, 0, 1), (True, 0, 0, 1), (1, 2, 3)):
            self.assertIsNone(transform_detection_orientation(q))

    def test_measured_box_callback_accepts_only_dedicated_cube_namespace_and_exact_tf(self):
        source = (Path(__file__).resolve().parents[1] / "runtime/python/camrod_ui/ui_backend_node.py").read_text()
        method = next(node for node in ast.walk(ast.parse(source)) if isinstance(node, ast.FunctionDef)
                      and node.name == "_on_driving_object_boxes")
        namespace = {"MarkerArray": object, "Marker": SimpleNamespace(ADD=0, DELETE=2, DELETEALL=3, CUBE=1),
                     "transform_detection_point": transform_detection_point,
                     "transform_detection_orientation": transform_detection_orientation,
                     "_ros_stamp_seconds": lambda stamp: stamp.sec + stamp.nanosec * 1e-9 if stamp else None,
                     "Time": SimpleNamespace(from_msg=lambda stamp: stamp.sec + stamp.nanosec * 1e-9),
                     "Duration": lambda *, seconds: seconds}
        exec(ast.unparse(method), namespace)
        calls = []
        half = math.sqrt(0.5)
        def lookup(target, frame, stamp, *, timeout):
            calls.append((target, frame, stamp, timeout))
            return SimpleNamespace(transform=SimpleNamespace(
                translation=SimpleNamespace(x=10, y=20, z=3),
                rotation=SimpleNamespace(x=0, y=0, z=half, w=half)))
        backend = SimpleNamespace(_now_s=lambda: 90.1, _driving=self.cache,
                                  _tf_buffer=SimpleNamespace(lookup_transform=lookup))
        def marker(ns="fusion_observed_extent", kind=1, action=0):
            return SimpleNamespace(ns=ns, id=0, type=kind, action=action,
                header=SimpleNamespace(frame_id="lidar", stamp=SimpleNamespace(sec=90, nanosec=0)),
                pose=SimpleNamespace(position=SimpleNamespace(x=1, y=0, z=2),
                                     orientation=SimpleNamespace(x=0, y=0, z=0, w=1)),
                scale=SimpleNamespace(x=0.3, y=0.5, z=1.6),
                lifetime=SimpleNamespace(sec=0, nanosec=200000000))
        callback = namespace["_on_driving_object_boxes"]
        self.live()
        self.cache.fusion_objects([self.object_record(source_frame="lidar")], ros_now_s=90.1)
        callback(backend, SimpleNamespace(markers=[marker(ns="fusion_det"), marker(kind=2)]))
        self.assertEqual(self.cache._object_boxes, {})
        callback(backend, SimpleNamespace(markers=[marker()]))
        box = self.cache.snapshot(self.mission)["perception"]["objects"][0]["bbox"]
        self.assertEqual(box["center"], {"x": 10.0, "y": 21.0, "z": 5.0})
        self.assertAlmostEqual(box["orientation"]["z"], half)
        self.assertAlmostEqual(box["orientation"]["w"], half)
        self.assertEqual(box["size"], {"x": 0.3, "y": 0.5, "z": 1.6})
        self.assertEqual(calls, [("map", "lidar", 90, 0.0)])
        callback(backend, SimpleNamespace(markers=[marker(action=2)]))
        self.assertEqual(self.cache._object_boxes, {})
        callback(backend, SimpleNamespace(markers=[marker()]))
        callback(backend, SimpleNamespace(markers=[marker(action=3)]))
        self.assertEqual(self.cache._object_boxes, {})
        callback(backend, SimpleNamespace(markers=[marker()]))
        callback(backend, SimpleNamespace(markers=[]))
        self.assertEqual(self.cache._object_boxes, {})
        def unavailable(*args, **kwargs):
            raise RuntimeError("missing source-time transform")
        backend._tf_buffer.lookup_transform = unavailable
        callback(backend, SimpleNamespace(markers=[marker()]))
        self.assertEqual(self.cache._object_boxes, {})
        self.assertNotIn("publish", ast.unparse(method))
        self.assertNotIn("PointCloud2", ast.unparse(method))

    def test_actual_fusion_marker_callback_matches_namespaces_and_handles_deleteall(self):
        source = (Path(__file__).resolve().parents[1] / "runtime/python/camrod_ui/ui_backend_node.py").read_text()
        method = next(node for node in ast.walk(ast.parse(source)) if isinstance(node, ast.FunctionDef)
                      and node.name == "_on_driving_objects")
        constants = SimpleNamespace(ADD=0, DELETE=2, DELETEALL=3, SPHERE=2, TEXT_VIEW_FACING=9)
        namespace = {"Marker": constants, "MarkerArray": object, "transform_detection_point": transform_detection_point,
                     "_ros_stamp_seconds": lambda stamp: stamp.sec + stamp.nanosec * 1e-9 if stamp else None}
        exec(ast.unparse(method), namespace)
        self.live()
        backend = SimpleNamespace(_now_s=lambda: 90.1, _driving=self.cache)

        def marker(identity, kind=2, ns="fusion_det", action=0, text="person\n2.00 m"):
            return SimpleNamespace(ns=ns, id=identity, type=kind, action=action, text=text,
                                   header=SimpleNamespace(frame_id="map", stamp=SimpleNamespace(sec=90, nanosec=0)),
                                   pose=SimpleNamespace(position=SimpleNamespace(x=7, y=1, z=0.8)),
                                   lifetime=SimpleNamespace(sec=0, nanosec=200000000))

        callback = namespace["_on_driving_objects"]
        callback(backend, SimpleNamespace(markers=[marker(0), marker(1, kind=9, ns="wrong", text="car")]))
        self.assertIsNone(self.cache.snapshot(self.mission)["perception"]["objects"][0]["class_name"])
        callback(backend, SimpleNamespace(markers=[marker(0), marker(1, kind=9)]))
        self.assertEqual(self.cache.snapshot(self.mission)["perception"]["objects"][0]["class_name"], "person")
        callback(backend, SimpleNamespace(markers=[marker(0, action=2)]))
        self.assertEqual(self.cache.snapshot(self.mission)["perception"]["objects"], [])
        callback(backend, SimpleNamespace(markers=[marker(0), marker(1, kind=9)]))
        callback(backend, SimpleNamespace(markers=[marker(0, action=3)]))
        self.assertEqual(self.cache.snapshot(self.mission)["perception"]["objects"], [])
        callback(backend, SimpleNamespace(markers=[marker(0), marker(1, kind=9)]))
        callback(backend, SimpleNamespace(markers=[]))
        self.assertEqual(self.cache.snapshot(self.mission)["perception"]["objects"], [])

        # HH_261001 - Non-map inputs require full rigid TF at the source stamp,
        # never a latest-TF fallback or a synthetic zero height.
        requests = []
        def lookup(target, source_frame, stamp, *, timeout):
            requests.append((target, source_frame, stamp, timeout))
            return SimpleNamespace(transform=SimpleNamespace(
                translation=SimpleNamespace(x=10, y=20, z=3),
                rotation=SimpleNamespace(x=0, y=0, z=0, w=1)))
        namespace["Time"] = SimpleNamespace(from_msg=lambda stamp: stamp.sec + stamp.nanosec * 1e-9)
        namespace["Duration"] = lambda *, seconds: seconds
        backend._tf_buffer = SimpleNamespace(lookup_transform=lookup)
        sphere, label = marker(0), marker(1, kind=9)
        sphere.header.frame_id = label.header.frame_id = "lidar"
        callback(backend, SimpleNamespace(markers=[sphere, label]))
        transformed = self.cache.snapshot(self.mission)["perception"]["objects"][0]
        self.assertEqual([transformed[axis] for axis in ("x", "y", "z")], [17, 21, 3.8])
        self.assertEqual(requests, [("map", "lidar", 90, 0.0)])

        def unavailable(*args, **kwargs):
            raise RuntimeError("no transform at source time")
        backend._tf_buffer.lookup_transform = unavailable
        callback(backend, SimpleNamespace(markers=[sphere, label]))
        self.assertEqual(self.cache.snapshot(self.mission)["perception"]["objects"], [])
        for seconds in (0, 88, 92):
            invalid = marker(0)
            invalid.header.stamp.sec = seconds
            callback(backend, SimpleNamespace(markers=[marker(0), invalid]))
            self.assertEqual(self.cache.snapshot(self.mission)["perception"]["objects"], [])
        self.assertNotIn("publish", ast.unparse(method))
        self.assertNotIn("PointCloud2", ast.unparse(method))


if __name__ == "__main__":
    unittest.main()
