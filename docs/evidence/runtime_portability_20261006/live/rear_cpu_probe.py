#!/usr/bin/env python3
"""Read-only ROS camera probe and bounded UI-telemetry subscription CPU A/B/A.

No vehicle commands, parameter writes, runtime starts or runtime stops.
CPU uses process user+system time; 100 percent equals one logical CPU.
Image subscriptions are removed before CPU intervals to avoid measuring probes.
"""

import datetime
import json
import os
from pathlib import Path
import time
import urllib.request

import psutil
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


OUTPUT = Path(__file__).with_name("rear_cpu_probe_results.json")
BASE_URL = os.environ.get("CAMROD_UI_URL", "http://127.0.0.1:8010")
GROUPS = {
    "detector": "apriltag_parking_detector_node",
    "relay": "carla_sensor_relay",
    "ui": "ui_backend_node",
}


def api(path, post=False):
    request = urllib.request.Request(BASE_URL + path, method="POST" if post else "GET")
    with urllib.request.urlopen(request, timeout=3) as response:
        return json.load(response)


def telemetry(active):
    return api("/api/telemetry/session?active=" + ("true&view=docking" if active else "false&view=all"), True)


def processes():
    result = {}
    for proc in psutil.process_iter(["pid", "cmdline"]):
        names = [Path(part).name for part in (proc.info["cmdline"] or [])[:2]]
        for group, executable in GROUPS.items():
            if executable in names:
                result[group] = proc
    return result


def ros_metadata():
    rclpy.init()
    node = rclpy.create_node("rear_cpu_readonly_probe_20261006")
    observations = {"image": [], "camera_info": []}

    def callback(kind, msg):
        observations[kind].append({
            "monotonic": time.monotonic(), "width": msg.width, "height": msg.height,
            "frame_id": msg.header.frame_id,
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "encoding": getattr(msg, "encoding", None),
        })

    image_topic = "/sensing/camera/econ_rear/image_rect"
    info_topic = "/sensing/camera/econ_rear/camera_info"
    subs = [
        node.create_subscription(Image, image_topic, lambda m: callback("image", m), qos_profile_sensor_data),
        node.create_subscription(CameraInfo, info_topic, lambda m: callback("camera_info", m), qos_profile_sensor_data),
    ]
    started = time.monotonic()
    while time.monotonic() - started < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    endpoints = {}
    for topic in (image_topic, info_topic):
        endpoints[topic] = [
            {"node": item.node_namespace.rstrip("/") + "/" + item.node_name, "type": item.topic_type}
            for item in node.get_publishers_info_by_topic(topic)
        ]
    for sub in subs:
        node.destroy_subscription(sub)
    params = {}
    requests = {
        "/perception/apriltag_parking_detector": [
            "image_topic", "camera_info_topic", "quad_decimate", "n_threads",
            "roi_full_search_interval", "publish_debug_image", "debug_jpeg_quality",
        ],
        "/carla_sensor_relay": [
            "rear_image_input", "rear_info_input", "rear_image_outputs",
            "raw_image_max_rate_hz", "compressed_image_max_rate_hz", "publish_compressed_images",
        ],
    }
    for name, namespace in node.get_node_names_and_namespaces():
        if name == "ui_backend":
            requests[namespace.rstrip("/") + "/" + name] = [
                "enable_operator_telemetry", "telemetry_session_timeout_s",
                "telemetry_camera_raw_fallback_enabled", "telemetry_docking_rear_camera_fallback_enabled",
            ]
    for fqn, names in requests.items():
        client = node.create_client(GetParameters, fqn + "/get_parameters")
        if not client.wait_for_service(timeout_sec=3):
            params[fqn] = {"error": "parameter service unavailable"}
            continue
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, future, timeout_sec=4)
        if future.done() and future.result():
            params[fqn] = dict(zip(names, [parameter_value_to_python(v) for v in future.result().values]))
        else:
            params[fqn] = {"error": "parameter request timed out"}
        node.destroy_client(client)
    graph = [namespace.rstrip("/") + "/" + name for name, namespace in node.get_node_names_and_namespaces()]
    node.destroy_node()
    rclpy.shutdown()
    streams = {}
    for kind, rows in observations.items():
        streams[kind] = {"received": len(rows), "first": rows[0] if rows else None,
                         "last": rows[-1] if rows else None}
        if len(rows) > 1:
            streams[kind]["wall_rate_hz"] = (len(rows) - 1) / (rows[-1]["monotonic"] - rows[0]["monotonic"])
    return {"streams": streams, "publishers": endpoints, "parameters": params, "nodes": sorted(graph)}


def measure(label, duration, active):
    print("Starting " + label, flush=True)
    selected = processes()
    start = {}
    memory = {key: [] for key in selected}
    for key, proc in selected.items():
        times = proc.cpu_times()
        start[key] = {"pid": proc.pid, "cpu_seconds": times.user + times.system,
                      "command": proc.cmdline(), "threads_start": proc.num_threads()}
    started = time.monotonic()
    next_refresh = started
    while time.monotonic() - started < duration:
        now = time.monotonic()
        if active and now >= next_refresh:
            telemetry(True)
            next_refresh = now + 5
        for key, proc in selected.items():
            try:
                memory[key].append(proc.memory_info().rss / 1048576)
            except psutil.Error:
                pass
        time.sleep(0.25)
    elapsed = time.monotonic() - started
    result = {"label": label, "elapsed_seconds": elapsed, "processes": {}}
    for key, proc in selected.items():
        row = start[key]
        try:
            times = proc.cpu_times()
            row["cpu_percent_one_core"] = (times.user + times.system - row.pop("cpu_seconds")) / elapsed * 100
            row["rss_mib_mean"] = sum(memory[key]) / len(memory[key])
            row["rss_mib_max"] = max(memory[key])
            row["threads_end"] = proc.num_threads()
        except psutil.Error as error:
            row["error"] = str(error)
        result["processes"][key] = row
    result["telemetry"] = api("/api/telemetry")
    print(json.dumps({"label": label, "processes": result["processes"]}), flush=True)
    return result


def main():
    report = {"utc_start": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "cpu_convention": "100 percent equals one logical CPU; user+system process time",
              "logical_cpu_count": psutil.cpu_count(), "samples": []}
    try:
        deadline = time.monotonic() + 90
        while True:
            try:
                if len(processes()) == 3:
                    api("/api/telemetry")
                    break
            except Exception:
                pass
            if time.monotonic() > deadline:
                raise RuntimeError("detector, relay, and UI not all ready within 90 seconds")
            time.sleep(2)
        report["metadata"] = ros_metadata()
        report["closed_request"] = telemetry(False)
        time.sleep(3)
        report["samples"].append(measure("closed_before", 15, False))
        report["open_request"] = telemetry(True)
        time.sleep(3)
        report["samples"].append(measure("docking_open", 20, True))
        telemetry(False)
        time.sleep(3)
        report["samples"].append(measure("closed_after", 15, False))
    except Exception as error:
        report["error"] = str(error)
    finally:
        try:
            report["final_closed_request"] = telemetry(False)
        except Exception as error:
            report["final_close_error"] = str(error)
        report["utc_end"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        OUTPUT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(str(OUTPUT), flush=True)


if __name__ == "__main__":
    main()
