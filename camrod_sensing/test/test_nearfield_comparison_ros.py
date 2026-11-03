"""HH_261102 - Four-way actual-node raw PointCloud2/radar comparison; not a vehicle/CARLA braking test."""

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

import pytest
import yaml


def test_four_way_nearfield_comparison(monkeypatch, tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from avg_msgs.msg import AvgOccupancyGrid, AvgRange, AvgTwist
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header
    from tf2_ros import StaticTransformBroadcaster

    source = Path(__file__).resolve().parents[2]
    build = Path(os.environ.get("CAMROD_COMPARISON_BUILD_BASE", source.parent / "build"))
    binaries = {name: build / package / name for package, name in (
        ("camrod_sensing", "radar_cost_grid_node"), ("camrod_sensing", "lidar_cost_grid_node"),
        ("camrod_control", "cmd_vel_safety_gate_node"))}
    if not all(path.is_file() for path in binaries.values()):
        pytest.skip("build the current sensing and control packages before the actual-node comparison")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    domain = 120 + os.getpid() % 80
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("nearfield_comparison", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    prefix = "/nearfield_compare"
    modes = ("original_exclusions", "revised_radar", "raw_guard", "combined")
    processes, logs, outputs, grids, gate_overrides = [], {}, {}, {}, {}
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))

    def spawn(name, binary, params):
        path = tmp_path / f"{name}.log"
        log = path.open("w", encoding="utf-8")
        args = [str(binaries[binary]), "--ros-args", "-r", f"__node:={name}"]
        for key, value in params.items():
            args += ["-p", f"{key}:={json.dumps(value)}"]
        process = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT)
        processes.append((process, log))
        logs[name] = path

    canonical = yaml.safe_load((source / "camrod_bringup/config/sensor_kit/robot_params.yaml").read_text())["/**"]["ros__parameters"]
    body = canonical["robot"]["body_extents"]
    lidar = canonical["lidar"]
    radar = [canonical["radar"][f"front{i}"] for i in (1, 2)]
    tf = StaticTransformBroadcaster(node)
    transforms = []
    for parent, child, xyz in [("map", "robot_center_link", (0.0, 0.0, 0.0)),
                               ("robot_center_link", "lidar_link", (lidar["x"], lidar["y"], lidar["z"]))] + [
        ("robot_center_link", f"radar_front{i}_link", (pose["x"], pose["y"], pose["z"]))
        for i, pose in enumerate(radar, 1)
    ]:
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = parent, child
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = xyz
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    tf.sendTransform(transforms)
    range_pubs = [node.create_publisher(AvgRange, f"{prefix}/front{i}/range", 10) for i in (1, 2)]
    cloud_pub = node.create_publisher(PointCloud2, f"{prefix}/raw_ground_points", 10)
    merged_pub = node.create_publisher(AvgOccupancyGrid, f"{prefix}/merged", latched)
    command_pubs = {mode: node.create_publisher(AvgTwist, f"{prefix}/{mode}/in", 10) for mode in modes}

    radar_params = yaml.safe_load((source / "camrod_sensing/config/radar/cost_grid.yaml").read_text())["/sensing/radar/radar_cost_grid"]["ros__parameters"]
    radar_params.update(input_topics=[f"{prefix}/front{i}/range" for i in (1, 2)],
                        stop_candidate_max_ranges_m=[0.3, 0.3], startup_return_learning_enable=False,
                        route_lanelet_filter_enable=False, base_frame_id="robot_center_link")
    radar_params["fixed_return_bands"] = [band for band in radar_params["fixed_return_bands"]
                                          if band.startswith(("FRONT1:", "FRONT2:"))]
    for name, exclusions in (("radar_original", True), ("radar_revised", False)):
        topic = f"{prefix}/{name}"
        node.create_subscription(AvgOccupancyGrid, topic, lambda msg, key=name: grids.__setitem__(key, msg), latched)
        spawn(name, "radar_cost_grid_node", dict(radar_params, fixed_return_filter_enable=exclusions,
              output_topic=topic, obstacle_evidence_topic=f"{topic}/evidence"))
    guard_params = yaml.safe_load((source / "camrod_sensing/config/lidar/front_guard.yaml").read_text())["/**"]["ros__parameters"]
    guard_params.update(input_topic=f"{prefix}/raw_ground_points", input_topics=[f"{prefix}/raw_ground_points"],
                        output_topic=f"{prefix}/raw_guard")
    spawn("lidar_guard", "lidar_cost_grid_node", guard_params)
    node.create_subscription(AvgOccupancyGrid, f"{prefix}/raw_guard", lambda msg: grids.__setitem__("raw_guard", msg), latched)

    # Fixture-only short dwell keeps this CI test bounded; production remains 2 s clear + 1 s hold.
    for mode in modes:
        radar_topic = f"{prefix}/radar_original" if mode == "original_exclusions" else f"{prefix}/radar_revised"
        if mode == "raw_guard":
            radar_topic = f"{prefix}/unused"
        params = dict(input_topic=f"{prefix}/{mode}/in", output_topic=f"{prefix}/{mode}/out",
            ros_output_topic=f"{prefix}/{mode}/out_ros", status_topic=f"{prefix}/{mode}/status",
            state_topic=f"{prefix}/{mode}/enabled", allow_on_start=True, require_platform_drive_enable=False,
            platform_safety_source_mode="disabled", enable_gnss_recovery_hold=False,
            enable_resume_after_hold_voice_gate=False, enable_route_heading_alignment=False,
            lanelet_safety_enable=False, pose_source_preference="tf_robot_base", robot_base_frame="robot_center_link",
            cost_grid_topic=f"{prefix}/merged", cost_source_debug_topics=[radar_topic],
            cost_source_debug_labels=["radar"], cost_stop_dynamic_source_labels="radar",
            cost_stop_clear_required_s=0.15, cost_stop_hold_s=0.05, cost_source_debug_max_age_s=0.35,
            cost_stop_latch_log_interval_s=0.05, cost_grid_stale_log_interval_s=0.05,
            radar_grid_freshness_stop_enable=mode != "raw_guard",
            lidar_front_guard_stop_enable=mode in ("raw_guard", "combined"),
            lidar_front_guard_grid_topic=f"{prefix}/raw_guard", lidar_front_guard_max_age_s=0.35)
        node.create_subscription(AvgTwist, f"{prefix}/{mode}/out",
            lambda msg, key=mode: outputs.__setitem__(key, (msg, time.monotonic())), 10)
        spawn(f"gate_{mode}", "cmd_vel_safety_gate_node", params)
        gate_overrides[mode] = params

    rows = []
    clear_grid = AvgOccupancyGrid()
    clear_grid.header.frame_id = "map"
    clear_grid.info.resolution, clear_grid.info.width, clear_grid.info.height = 0.05, 80, 80
    clear_grid.info.origin.position.x = clear_grid.info.origin.position.y = -2.0
    clear_grid.info.origin.orientation.w = 1.0
    clear_grid.data = [0] * 6400

    def pump(duration, points=(), obstacle=False, radar_dropout=False, lidar_dropout=False,
             old_cloud=False, vx=0.2, cloud_frame="lidar_link", cloud_points=None):
        start = time.monotonic()
        while time.monotonic() - start < duration:
            stamp = node.get_clock().now().to_msg()
            clear_grid.header.stamp = stamp
            merged_pub.publish(clear_grid)
            if not radar_dropout:
                for index, publisher in enumerate(range_pubs):
                    value = 0.501
                    if obstacle and points:
                        dx, dy = points[0][0] - radar[index]["x"], points[0][1] - radar[index]["y"]
                        distance = math.hypot(dx, dy)
                        if dx > 0.0 and abs(math.atan2(dy, dx)) <= math.radians(32.5) and distance <= 0.5:
                            value = round(distance * 1000.0) / 1000.0
                    message = AvgRange()
                    message.header.stamp, message.header.frame_id = stamp, f"radar_front{index + 1}_link"
                    message.min_range, message.max_range, message.range = 0.02, 0.5, value
                    publisher.publish(message)
            if not lidar_dropout:
                header = Header()
                header.stamp, header.frame_id = stamp, cloud_frame
                if old_cloud:
                    header.stamp.sec -= 2
                cloud_pub.publish(create_cloud_xyz32(header, [(x - lidar["x"], y - lidar["y"], z - lidar["z"])
                    for x, y, z in (points if cloud_points is None else cloud_points)]))
            for publisher in command_pubs.values():
                command = AvgTwist()
                command.linear.x = vx
                publisher.publish(command)
            executor.spin_once(timeout_sec=0.02)

    def record(scenario, expected, points=(), clearance=None, obstacle=False, **kwargs):
        previous_lines = {mode: len(logs[f"gate_{mode}"].read_text().splitlines()) for mode in modes}
        pump(0.85, points, obstacle, **kwargs)
        assert set(outputs) == set(modes), {key: path.read_text()[-1500:] for key, path in logs.items()}
        for index, mode in enumerate(modes):
            command, receive_time = outputs[mode]
            assert time.monotonic() - receive_time < 0.4
            blocked = abs(command.linear.x) < 1.0e-6
            log = logs[f"gate_{mode}"].read_text()
            reasons = re.findall(r"cmd_vel cost stop: ([^\n]+)", log)
            new_reasons = re.findall(r"cmd_vel cost stop: ([^\n]+)",
                "\n".join(log.splitlines()[previous_lines[mode]:]))
            initial_reason = next((text for text in new_reasons if text.startswith(("dynamic_", "merged_"))),
                                  new_reasons[0] if new_reasons else None)
            reason = reasons[-1] if blocked and reasons else ("output_zero" if blocked else "clear_output")
            radar_key = "radar_original" if mode == "original_exclusions" else "radar_revised"
            radar_hits = sum(cost >= 85 for cost in grids[radar_key].data) if radar_key in grids and mode != "raw_guard" else 0
            raw_hits = sum(cost >= 85 for cost in grids["raw_guard"].data) if "raw_guard" in grids and mode in ("raw_guard", "combined") else 0
            sensor_range = round(math.hypot(points[0][0] - radar[0]["x"], points[0][1] - radar[0]["y"]) * 1000.0) / 1000.0 if obstacle and points else None
            lidar_visible = None
            if points and math.isfinite(points[0][0]):
                lidar_x, lidar_y = points[0][0] - lidar["x"], points[0][1] - lidar["y"]
                lidar_visible = lidar_x > 0.0 and abs(math.atan2(lidar_y, lidar_x)) <= math.radians(64.0)
            rows.append(dict(mode=mode, scenario=scenario, body_clearance_m=clearance, sensor_range_m=sensor_range,
                commanded_vx=kwargs.get("vx", 0.2), output_vx=command.linear.x, blocked=blocked, reason=reason,
                initial_stop_reason=initial_reason,
                latency_ms=None, hit_cells=radar_hits + raw_hits, radar_hit_cells=radar_hits, raw_guard_hit_cells=raw_hits,
                raw_point_count=len(kwargs.get("cloud_points", points)), upstream_lidar_fov_visible=lidar_visible,
                expected_blocked=expected[index], **{"pass": blocked == expected[index]}))
            if scenario.startswith("body_") and blocked:
                assert radar_hits + raw_hits > 0

    try:
        pump(2.0)
        for distance, expected in ((0.1, (False, True, True, True)), (0.2, (True, True, True, True)),
                                   (0.3, (False, False, True, True)), (0.4, (False, False, False, False))):
            record(f"body_{round(distance * 100)}cm", expected,
                   [(body["front"] + distance, -0.08, radar[0]["z"])], distance, True)
        # Sensor-face distance is not bumper clearance; the 10 cm face case is behind the existing LiDAR crop.
        for distance, expected in ((0.1, (False, True, False, True)), (0.2, (False, True, True, True)),
                                   (0.3, (True, True, True, True)), (0.4, (False,) * 4)):
            point = (radar[0]["x"] + distance, radar[0]["y"], radar[0]["z"])
            cloud_points = [point] if point[0] > lidar["x"] else []
            record(f"sensor_face_{round(distance * 100)}cm", expected, [point],
                   point[0] - body["front"], True, cloud_points=cloud_points)
        record("unknown_class_radar_no_return", (False, False, True, True), [(body["front"] + 0.2, -0.08, 0.2)], 0.2)
        record("ground_only", (False,) * 4, [(body["front"] + 0.1, 0.0, body["bottom_z"])])
        record("self_body_only", (False,) * 4, [(body["front"] - 0.05, 0.0, 0.2)])
        record("outside_body_width", (False,) * 4, [(body["front"] + 0.1, body["left"] + 0.05, 0.2)])
        record("lidar_dropout", (False, False, True, True), lidar_dropout=True)
        record("removed_obstacle_resume", (False,) * 4)
        record("radar_dropout", (True, True, False, True), radar_dropout=True)
        record("all_sources_recovered", (False,) * 4)
        record("replayed_cloud", (False, False, True, True), old_cloud=True)
        record("fresh_empty_scan_resume", (False,) * 4)
        record("malformed_cloud", (False, False, True, True), [(float("nan"), 0.0, 0.2)])
        record("valid_cloud_resume", (False,) * 4)
        record("missing_cloud_transform", (False, False, True, True), [(body["front"] + 0.1, 0.0, 0.2)], cloud_frame="missing_lidar_frame")
        record("transform_recovered", (False,) * 4)
        record("reverse_away_front_only_obstacle", (False,) * 4,
               [(body["front"] + 0.3, -0.08, 0.2)], 0.3, True, vx=-0.2)
        source_files = ["camrod_control/src/cmd_vel_safety_gate_node.cpp", "camrod_control/src/motion_cost_stop.cpp",
            "camrod_control/include/camrod_control/motion_cost_stop.hpp", "camrod_sensing/src/radar_cost_grid_node.cpp",
            "camrod_sensing/src/lidar_cost_grid_node.cpp", "camrod_sensing/include/camrod_sensing/front_lidar_guard.hpp",
            "camrod_sensing/config/lidar/front_guard.yaml", "camrod_sensing/config/radar/cost_grid.yaml",
            "camrod_sensing/test/test_nearfield_comparison_ros.py",
            "camrod_sensing/external/ground_segmentation_ros2/src/ground_segmentation_ros2_node.cpp"]
        implementations = dict(binaries, lidar_component=build / "camrod_sensing/liblidar_cost_grid_component.so")
        metadata = dict(runtime="isolated_ros_actual_nodes", modes=modes,
            body_front_m=body["front"], radar_front_x_m=radar[0]["x"], lidar_x_m=lidar["x"],
            radar_face_to_body_front_m=body["front"] - radar[0]["x"],
            radar_scalar_quantization_m=0.001, upstream_lidar_preprocessor_half_fov_deg=64.0,
            fixture_overrides=dict(clear_required_s=0.15, stop_hold_s=0.05, source_age_s=0.35),
            gate_parameter_overrides=gate_overrides, lidar_guard_parameter_overrides=guard_params,
            source_revision=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=source, text=True).strip(),
            source_sha256={name: hashlib.sha256((source / name).read_bytes()).hexdigest() for name in source_files},
            binary_sha256={name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in implementations.items()},
            baseline_definition="Historical exclusion CONFIG on the current rebuilt radar/gate code; not the old entire binary.",
            reason_semantics="reason is latest actual latched warning; initial_stop_reason is the first observed new dynamic/merged event where available; neither is physical braking latency.",
            production_clear_required_s=2.0, production_stop_hold_s=1.0,
            note="Static synthetic ground-segmented raw PointCloud2 and ideal-cone scalar ranges into actual native nodes/gates; not CARLA, real radar detection, or physical braking.")
        output = Path(os.environ.get("CAMROD_NEARFIELD_COMPARISON_OUTPUT", tmp_path / "nearfield_comparison.json"))
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(dict(metadata=metadata, rows=rows), indent=2) + "\n")
        with output.with_suffix(".csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        assert all(row["pass"] for row in rows), [row for row in rows if not row["pass"]]
    finally:
        for process, _ in processes:
            process.terminate()
        for process, log in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
