#!/usr/bin/env python3
"""HH_261102 - Observe real CARLA/ROS inputs; optional owned actors never move the ego or publish sensor/control data."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import uuid


CHANNELS = ("front1", "front2", "left1", "left2", "right1", "right2", "rear")
BODY = {"front": 0.70837, "left": 0.53505, "right": 0.53495, "distance": 0.30}
# HH_261102 - CARLA headers use /clock; the existing canonical relay and native grids use reception wall time.
CLOCK_DOMAINS = {"raw_lidar": "carla_sim", "filtered_lidar": "wall", "radar_grid": "wall",
    "guard_grid": "wall", "gate": "wall", **{"raw_radar_"+name: "carla_sim" for name in CHANNELS},
    **{"range_"+name: "wall" for name in CHANNELS}}


def source_timing(name, stamp_s, wall_s, sim_s, sim_receipt_age_s):
    domain = CLOCK_DOMAINS[name]  # Unknown sources must declare a contract, not guess from timestamp magnitude.
    source_now = wall_s if domain == "wall" else sim_s
    if domain == "carla_sim" and (sim_receipt_age_s is None or not 0 <= sim_receipt_age_s <= .5):
        source_now = None
    age = source_now-stamp_s if source_now is not None and stamp_s is not None and stamp_s > 0 else None
    return {"source_clock": domain, "header_stamp_s": stamp_s, "wall_clock_s": wall_s,
            "carla_clock_s": sim_s, "carla_clock_receipt_age_s": sim_receipt_age_s, "source_age_s": age}


def source_fresh(sample, maximum_age=.35):
    age = sample.get("clock_now", {}).get("source_age_s")
    return "value" in sample and age is not None and 0 <= age <= maximum_age and 0 <= sample["receipt_age_s"] <= maximum_age


def shadow_grid_hit(sample):
    # HH_261102 - Unknown raster background is not free space; classify only presence/absence of measured high-cost cells.
    if not source_fresh(sample) or "front_corridor_cells_ge85" not in sample["value"]:
        return None
    return sample["value"]["front_corridor_cells_ge85"] > 0


def projected_bounds(vertices, origin, forward, right):
    coordinates = [((p.x-origin.x)*forward.x + (p.y-origin.y)*forward.y,
                    (p.x-origin.x)*right.x + (p.y-origin.y)*right.y, p.z) for p in vertices]
    return {name: [min(p[i] for p in coordinates), max(p[i] for p in coordinates)]
            for i, name in enumerate(("forward", "right", "z"))}


def placement_forward(ego_front, clearance, obstacle_rear_offset):
    if not all(math.isfinite(v) for v in (ego_front, clearance, obstacle_rear_offset)) or clearance <= 0:
        raise ValueError("finite geometry and positive body clearance required")
    return ego_front + clearance - obstacle_rear_offset


def yaw(q):
    if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .001:
        raise ValueError("invalid grid/TF quaternion")
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)  # Preserve invalid/no-return values explicitly; never turn them into fake clear ranges.
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def screen_ground_hits(hits, body_bottom):
    """HH_261102 - Select measured Roads/Terrain just below the hull; never authorize a roof or skip a Car by guess."""
    records = []
    for hit in hits:
        point = hit.location
        label = str(hit.label)
        eligible = (label.rsplit(".", 1)[-1].lower() in ("road", "roads", "terrain")
                    and all(math.isfinite(v) for v in (point.x, point.y, point.z, body_bottom))
                    and body_bottom-.30 <= point.z <= body_bottom+.05)
        records.append({"label": label, "label_repr": repr(hit.label),
            "xyz": {axis: getattr(point, axis) for axis in ("x", "y", "z")}, "eligible_ground": eligible})
    return next((index for index, record in enumerate(records) if record["eligible_ground"]), None), records


def raw_grid_metrics(message):
    if message.info.width <= 0 or message.info.height <= 0 or len(message.data) != message.info.width * message.info.height:
        raise ValueError("malformed grid")
    resolution = message.info.resolution
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("invalid grid resolution")
    return {"occupied_cells_ge85": sum(value >= 85 for value in message.data),
            "unknown_cells": sum(value < 0 for value in message.data),
            "resolution_m": resolution, "cell_overlap_margin_m": resolution/2}


def front_grid_metrics(message, transform):
    """HH_261102 - Shadow grid overlap is not the final gate's motion decision or a sensor-isolation test."""
    raw = raw_grid_metrics(message)
    resolution = message.info.resolution
    origin, translation = message.info.origin, transform.translation
    if not all(math.isfinite(v) for v in (origin.position.x, origin.position.y, translation.x, translation.y)):
        raise ValueError("invalid grid/TF position")
    grid_yaw, body_yaw = yaw(origin.orientation), yaw(transform.rotation)
    occupied = corridor = 0
    for index, value in enumerate(message.data):
        if value < 85:
            continue
        occupied += 1
        x = (index % message.info.width + .5) * resolution
        y = (index // message.info.width + .5) * resolution
        gx = origin.position.x + math.cos(grid_yaw)*x - math.sin(grid_yaw)*y
        gy = origin.position.y + math.sin(grid_yaw)*x + math.cos(grid_yaw)*y
        bx = translation.x + math.cos(body_yaw)*gx - math.sin(body_yaw)*gy
        by = translation.y + math.sin(body_yaw)*gx + math.cos(body_yaw)*gy
        margin = resolution / 2
        if BODY["front"]-margin <= bx <= BODY["front"]+BODY["distance"]+margin and -BODY["right"]-margin <= by <= BODY["left"]+margin:
            corridor += 1
    return {**raw, "front_corridor_cells_ge85": corridor}


def cloud_metrics(message):
    import numpy as np
    fields = [f for f in message.fields if f.count == 1 and f.datatype in (7, 8)]
    if not fields or message.point_step <= 0 or message.row_step < message.width*message.point_step or len(message.data) < message.row_step*message.height:
        raise ValueError("malformed cloud")
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype({"names": [f.name for f in fields], "formats": [endian+("f4" if f.datatype == 7 else "f8") for f in fields],
                      "offsets": [f.offset for f in fields], "itemsize": message.point_step})
    points = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                        strides=(message.row_step, message.point_step))
    result = {"points": message.width*message.height, "frame": message.header.frame_id, "fields": {}}
    for field in fields:
        values = points[field.name].reshape(-1)
        values = values[np.isfinite(values)]
        result["fields"][field.name] = {"finite": int(values.size),
            "min": float(values.min()) if values.size else None, "max": float(values.max()) if values.size else None}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="otherwise print plan without connecting")
    parser.add_argument("--place-obstacles", action="store_true", help="explicitly permit owned pedestrian spawn/destroy")
    parser.add_argument("--ego-id", type=int)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--baseline-seconds", type=float, default=60)
    parser.add_argument("--case-seconds", type=float, default=5)
    args = parser.parse_args()
    if not args.run:
        print(json.dumps({"mode": "PLAN_ONLY_NO_CONNECTION", "baseline_seconds": args.baseline_seconds,
            "front_body_clearances_m": [.10, .20, .30, .40], "roadside_controls": ["left", "right"],
            "comparison": "read_only_shadow_grid_overlap_not_final_gate_isolation", "fake_sensor_publish": False}))
        return 0
    if not args.ego_id or args.ego_id <= 0 or not args.output or not math.isfinite(args.baseline_seconds) or args.baseline_seconds < 60 or not math.isfinite(args.case_seconds) or not 2 <= args.case_seconds <= 60:
        parser.error("run requires positive --ego-id, new --output, baseline >=60s, cases 2..60s")
    import carla
    import rclpy
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.parameter import Parameter
    from tf2_ros import Buffer, TransformListener
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import PointCloud2, Range
    from rosgraph_msgs.msg import Clock
    from avg_msgs.msg import AvgOccupancyGrid, ModuleState

    role = "camrod_sensor_probe_" + uuid.uuid4().hex[:12]
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(5)
    world = client.get_world()
    world.wait_for_tick(10)  # Observe the existing bridge tick; never advance the world.
    ego = world.get_actor(args.ego_id)
    if ego is None or not ego.type_id.startswith("vehicle.ranger.") or ego.attributes.get("role_name") != "ego_vehicle":
        raise RuntimeError("expected exact Ranger ego actor")
    initial = ego.get_transform()
    heading = math.radians(initial.rotation.yaw)
    forward = carla.Vector3D(math.cos(heading), math.sin(heading), 0)
    right = carla.Vector3D(-math.sin(heading), math.cos(heading), 0)
    physical = projected_bounds(ego.bounding_box.get_world_vertices(initial), initial.location, forward, right)
    rclpy.init(args=[])
    node = rclpy.create_node("readonly_real_sensor_probe", parameter_overrides=[Parameter("use_sim_time", value=False)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    latest, counts, publishers, owned, totals = {}, {}, {}, [], {}
    pending_grids, removed_ids = {}, set()
    sim_clock = {"seconds": None, "received": None}

    def receive_clock(message):
        sim_clock.update(seconds=message.clock.sec+message.clock.nanosec/1e9, received=time.monotonic())

    clock_subscription = node.create_subscription(Clock, "/clock", receive_clock, qos_profile_sensor_data)

    def timing(name, stamp):
        clock_age = time.monotonic()-sim_clock["received"] if sim_clock["received"] is not None else None
        return source_timing(name, stamp, time.time(), sim_clock["seconds"], clock_age)
    topics = [("raw_lidar", PointCloud2, "/carla/ego_vehicle/lidar_front"),
              ("filtered_lidar", PointCloud2, "/sensing/lidar/points_filtered"),
              ("radar_grid", AvgOccupancyGrid, "/sensing/cost_grid/radar"),
              ("guard_grid", AvgOccupancyGrid, "/sensing/cost_grid/lidar_front_guard"),
              ("gate", ModuleState, "/control/cmd_vel_safety_gate/status")]
    topics += [("raw_radar_"+name, PointCloud2, "/carla/ego_vehicle/radar_"+name) for name in CHANNELS]
    topics += [("range_"+name, Range, "/sensing/radar/"+name+"/range_ros") for name in CHANNELS]

    def receive(name, message):
        stamp = message.header.stamp if hasattr(message, "header") else None
        stamp_s = stamp.sec+stamp.nanosec/1e9 if stamp else None
        received_timing = timing(name, stamp_s)
        try:
            if isinstance(message, PointCloud2):
                value = cloud_metrics(message)
            elif isinstance(message, AvgOccupancyGrid):
                value = raw_grid_metrics(message)
                # HH_261102 - Retain actual cell counts immediately; a later spin can receive the exact-stamp TF.
                pending_grids[name] = {"message": message, "received": time.monotonic(), "clock_at_receive": received_timing}
            else:
                value = message_to_ordereddict(message)
            latest[name] = {"value": value, "received": time.monotonic(), "clock_at_receive": received_timing}
        except Exception as error:
            latest[name] = {"error": str(error), "received": time.monotonic(), "clock_at_receive": received_timing}
        counts[name] = counts.get(name, 0) + 1

    subscriptions = [node.create_subscription(kind, topic, lambda msg, name=name: receive(name, msg), qos_profile_sensor_data)
                     for name, kind, topic in topics]

    def sensor_snapshot():
        now = time.monotonic()
        return {name: {**entry, "receipt_age_s": now-entry["received"],
            "clock_now": timing(name, entry["clock_at_receive"]["header_stamp_s"])} for name, entry in latest.items()}

    def process_pending_grids():
        for name, pending in list(pending_grids.items()):
            message = pending["message"]
            age = timing(name, pending["clock_at_receive"]["header_stamp_s"])["source_age_s"]
            if age is None or age > .35 or time.monotonic()-pending["received"] > .35:
                latest[name]["tf_error"] = "exact-stamp TF unavailable within 0.35s"
                pending_grids.pop(name)
                continue
            try:
                tf = buffer.lookup_transform("robot_center_link", message.header.frame_id,
                    Time.from_msg(message.header.stamp), timeout=Duration(seconds=0))
                latest[name] = {"value": front_grid_metrics(message, tf.transform),
                    "received": pending["received"], "clock_at_receive": pending["clock_at_receive"]}
                pending_grids.pop(name)
            except Exception as error:
                latest[name]["tf_error"] = str(error)

    def stationary():
        current = ego.get_transform()
        if ego.get_velocity().length() > .03 or current.location.distance(initial.location) > .03 or abs((current.rotation.yaw-initial.rotation.yaw+180)%360-180) > .5:
            raise RuntimeError("ego moved; stop observation and remove only owned test actors")

    def destroy(actor, stream):
        current = world.get_actor(actor.id)
        if current is not None:
            if current.type_id != actor.type_id or current.attributes.get("role_name") != role:
                raise RuntimeError("owned actor identity mismatch; refusing removal")
            if not current.destroy():
                raise RuntimeError("owned actor removal failed")
            removed_ids.add(actor.id)
            write(stream, {"event": "owned_actor_removed", "actor_id": actor.id})

    def write(stream, record):
        stream.write(json.dumps(json_safe({"utc": datetime.now(timezone.utc).isoformat(), **record}), allow_nan=False)+"\n")
        stream.flush()

    def observe(stream, label, seconds):
        deadline, next_sample = time.monotonic()+seconds, 0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            process_pending_grids()
            if time.monotonic() < next_sample:
                continue
            stationary()
            now = time.monotonic()
            snapshot = sensor_snapshot()
            hits = {}
            for name in ("radar_grid", "guard_grid"):
                sample = snapshot.get(name, {})
                hits[name] = shadow_grid_hit(sample)
            both = any(hits.values()) if all(value is not None for value in hits.values()) else None
            comparison = {"radar_only": hits["radar_grid"], "guard_only": hits["guard_grid"], "both": both}
            tally = totals.setdefault(label, {mode: {"hit": 0, "no_hit": 0, "unavailable": 0} for mode in comparison})
            for mode, hit in comparison.items():
                tally[mode]["unavailable" if hit is None else "hit" if hit else "no_hit"] += 1
            write(stream, {"event": "sample", "case": label, "sensors": snapshot,
                "carla_frame": world.get_snapshot().frame, "shadow_grid_hits": comparison})
            next_sample = now+.2

    def spawn(stream, distance, lateral, relative_bottom=-1.0, case="characterization", requested_gap=None):
        stationary()
        point = initial.location + forward*distance + right*lateral
        hull = projected_bounds(ego.bounding_box.get_world_vertices(ego.get_transform()), initial.location, forward, right)
        ray_start = carla.Location(point.x, point.y, hull["z"][1]+1)
        ray_end = carla.Location(point.x, point.y, hull["z"][0]-.35)
        hits = world.cast_ray(ray_start, ray_end)
        selected, hit_records = screen_ground_hits(hits, hull["z"][0])
        check = {"event": "placement_ground_check", "case": case, "requested_front_body_gap_m": requested_gap,
            "candidate_xyz_before_ground": {axis: getattr(point, axis) for axis in ("x", "y", "z")},
            "candidate_xyz": None, "actor_z_requires_verified_ground": True, "ego_hull_m": hull,
            "ray_start_xyz": {axis: getattr(ray_start, axis) for axis in ("x", "y", "z")},
            "ray_end_xyz": {axis: getattr(ray_end, axis) for axis in ("x", "y", "z")},
            "hits": hit_records, "selected_hit": selected, "actor_id_exposed_by_raycast": False}
        if selected is None:
            write(stream, {**check, "rejection_phase": "ground_selection"})
            raise RuntimeError("no bounded Roads/Terrain hit below ego hull; see exact ray receipt")
        ground = hits[selected]
        point.z = ground.location.z-relative_bottom+.02
        write(stream, {**check, "candidate_xyz": {axis: getattr(point, axis) for axis in ("x", "y", "z")}})
        blueprint = world.get_blueprint_library().find("walker.pedestrian.0001")
        blueprint.set_attribute("role_name", role)
        actor = world.try_spawn_actor(blueprint, carla.Transform(point, carla.Rotation(yaw=initial.rotation.yaw+180)))
        if actor is None:
            write(stream, {"event": "placement_rejected", "case": case, "rejection_phase": "collision_safe_spawn",
                "requested_front_body_gap_m": requested_gap, "candidate_xyz": {axis: getattr(point, axis) for axis in ("x", "y", "z")}})
            raise RuntimeError("collision-safe actor placement rejected")
        owned.append(actor)
        write(stream, {"event": "owned_actor_spawned", "actor_id": actor.id, "role": role,
            "ground_label": str(ground.label), "ground_z": ground.location.z})
        observe(stream, "settling", 1)
        return actor

    # HH_261102 - New exclusive output prevents overwriting prior physical observations.
    try:
        with args.output.open("x", encoding="utf8") as stream:
            for name, _, topic in topics:
                publishers[name] = [{"name": item.node_name, "namespace": item.node_namespace}
                                    for item in node.get_publishers_info_by_topic(topic)]
            write(stream, {"event": "start", "ego_id": ego.id, "physical_bbox_projected_m": physical,
                "ego_origin_xyz": {axis: getattr(initial.location, axis) for axis in ("x", "y", "z")},
                "ego_yaw_deg": initial.rotation.yaw,
                "probe_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "native_guard_body_m": BODY, "topics": {name: topic for name, _, topic in topics},
                "clock_contract": CLOCK_DOMAINS, "observer_use_sim_time": False,
                "publishers_initial": publishers, "actor_spawning_authorized": args.place_obstacles,
                "comparison_kind": "SHADOW_GRID_OVERLAP_NOT_FINAL_GATE_ISOLATION", "synthetic_sensor_publish": False})
            observe(stream, "no_test_actor_ground_roadside_baseline", args.baseline_seconds)
            write(stream, {"event": "publishers_after_baseline", "publishers": {
                name: [{"name": item.node_name, "namespace": item.node_namespace} for item in node.get_publishers_info_by_topic(topic)]
                for name, _, topic in topics}})
            if args.place_obstacles:
                observe(stream, "preflight_tf_settle", 1)
                snapshot = sensor_snapshot()
                for name in ("radar_grid", "guard_grid", "raw_lidar", "filtered_lidar", "raw_radar_front1", "raw_radar_front2"):
                    sample = snapshot.get(name, {})
                    if not source_fresh(sample) or (name.endswith("_grid") and "front_corridor_cells_ge85" not in sample["value"]):
                        raise RuntimeError(name+" missing/stale; no test actors spawned")
                # HH_261102 - Learn the real pedestrian bounds using a separate owned actor; never teleport any actor.
                template = spawn(stream, physical["forward"][1]+3, 0)
                transform = template.get_transform()
                bounds = projected_bounds(template.bounding_box.get_world_vertices(transform), transform.location, forward, right)
                rear_offset = bounds["forward"][0]
                half_width = (bounds["right"][1]-bounds["right"][0])/2
                bottom_offset = bounds["z"][0]-transform.location.z
                write(stream, {"event": "obstacle_shape", "actor_id": template.id,
                    "projected_offset_bounds_m": bounds, "bottom_offset_m": bottom_offset})
                destroy(template, stream)
                cases = [(f"front_{int(gap*100)}cm", gap, 0) for gap in (.1, .2, .3, .4)]
                cases += [("roadside_left", .2, physical["right"][0]-half_width-.20),
                          ("roadside_right", .2, physical["right"][1]+half_width+.20)]
                for label, gap, lateral in cases:
                    observe(stream, "no_owned_actor_before_"+label, 3)
                    body_now = projected_bounds(ego.bounding_box.get_world_vertices(ego.get_transform()), initial.location, forward, right)
                    actor = spawn(stream, placement_forward(body_now["forward"][1], gap, rear_offset), lateral, bottom_offset, label, gap)
                    actual = projected_bounds(actor.bounding_box.get_world_vertices(actor.get_transform()), initial.location, forward, right)
                    body_now = projected_bounds(ego.bounding_box.get_world_vertices(ego.get_transform()), initial.location, forward, right)
                    measured = actual["forward"][0]-body_now["forward"][1]
                    write(stream, {"event": "measured_placement", "case": label, "actor_id": actor.id,
                        "requested_front_body_gap_m": gap, "measured_front_body_gap_m": measured,
                        "ego_bounds_m": body_now, "obstacle_bounds_m": actual})
                    if abs(measured-gap) > .02:
                        raise RuntimeError("physical gap differs by >2cm; do not interpret this case")
                    observe(stream, label, args.case_seconds)
                    destroy(actor, stream)
            observe(stream, "final_no_test_actor", 3)
            write(stream, {"event": "complete", "received_counts": counts,
                "shadow_counts_by_case": totals, "scenario_acceptance": None})
    finally:
        # HH_261102 - Attempt every owned cleanup even if one RPC fails, and preserve the original failure.
        original_error = sys.exc_info()[0] is not None
        cleanup_errors = []
        for actor in owned:
            if actor.id in removed_ids:
                continue
            try:
                current = world.get_actor(actor.id)
                if current is not None and current.type_id == actor.type_id and current.attributes.get("role_name") == role:
                    if not current.destroy():
                        cleanup_errors.append({"actor_id": actor.id, "error": "destroy returned false"})
            except Exception as error:
                cleanup_errors.append({"actor_id": actor.id, "error": str(error)})
        node.destroy_node()
        rclpy.shutdown()
        if cleanup_errors:
            print(json.dumps({"owned_cleanup_errors": cleanup_errors, "role": role}), file=sys.stderr)
            if not original_error:
                raise RuntimeError("owned actor cleanup incomplete; inspect the reported IDs/role")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
