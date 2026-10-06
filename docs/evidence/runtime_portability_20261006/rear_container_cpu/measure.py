#!/usr/bin/env python3
"""HH_261006 - Bounded live-container CPU A/B/A; no commands or parameter writes."""
import datetime
import hashlib
import json
import platform
import subprocess
import time
from pathlib import Path

import psutil
import rclpy
from ament_index_python.packages import get_package_prefix
from avg_msgs.msg import AvgBool
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

ROOT = Path(__file__).resolve().parent


def main():
    rclpy.init()
    node = rclpy.create_node("rear_container_perf_readonly")
    result = {"started": datetime.datetime.now().astimezone().isoformat(),
              "architecture": platform.machine(), "cpu_logical": psutil.cpu_count(),
              "cpu_units": "100 percent = one logical CPU", "phases": []}
    component = Path(get_package_prefix("camrod_perception")) / "lib/libapriltag_parking_detector_component.so"
    result["component"] = {"path": str(component.resolve()),
                           "sha256": hashlib.sha256(component.read_bytes()).hexdigest()}
    result["git_head"] = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
    observations = []
    def image_callback(msg):
        observations.append({"time": time.monotonic(), "width": msg.width,
                             "height": msg.height, "encoding": msg.encoding})
    subscription = node.create_subscription(Image, "/carla/ego_vehicle/rgb_rear/image", image_callback, qos_profile_sensor_data)
    started = time.monotonic()
    while time.monotonic() - started < 5:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(subscription)
    if len(observations) < 3:
        raise RuntimeError("Insufficient live camera images; CPU numbers would not be valid")
    result["input"] = {"first": observations[0], "last": observations[-1], "count": len(observations),
                       "wall_hz": (len(observations)-1)/(observations[-1]["time"]-observations[0]["time"])}
    processes, parameters = {}, {}
    counters = {i: {"detected": 0, "true": 0, "jpeg": 0} for i in (1, 2)}
    persistent_subscriptions = []
    def count_detection(i, msg):
        counters[i]["detected"] += 1
        counters[i]["true"] += int(msg.data)
    for i in (1, 2):
        wanted = f"__node:=rear_perf_decimate_{i}_container"
        matches = [p for p in psutil.process_iter(["cmdline"]) if wanted in (p.info["cmdline"] or [])]
        if len(matches) != 1:
            raise RuntimeError(f"Expected one {wanted}, got {len(matches)}")
        processes[i] = matches[0]
        topic = f"/rear_perf_decimate_{i}/apriltag_parking_detector/tag_detected"
        persistent_subscriptions.append(node.create_subscription(AvgBool, topic, lambda m, k=i: count_detection(k, m), 10))
        client = node.create_client(GetParameters, f"/rear_perf_decimate_{i}/apriltag_parking_detector/get_parameters")
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError("Detector parameter service unavailable")
        names = ["quad_decimate", "n_threads", "roi_full_search_interval", "roi_scale", "debug_jpeg_quality", "publish_debug_image", "image_topic", "camera_info_topic", "target_tag_id", "tag_size", "tag_family", "publish_tf"]
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        parameters[i] = dict(zip(names, [parameter_value_to_python(v) for v in future.result().values]))
        node.destroy_client(client)
    result["parameters"] = parameters
    jpeg_subscriptions = []
    try:
        for label, subscribe in (("closed_before", False), ("debug_open", True), ("closed_after", False)):
            for sub in jpeg_subscriptions:
                node.destroy_subscription(sub)
            jpeg_subscriptions.clear()
            if subscribe:
                def jpeg_callback(i, _msg):
                    counters[i]["jpeg"] += 1
                for i in (1, 2):
                    topic = f"/rear_perf_decimate_{i}/apriltag_parking_detector/debug_image/compressed"
                    jpeg_subscriptions.append(node.create_subscription(CompressedImage, topic, lambda m, k=i: jpeg_callback(k, m), 1))
            settle = time.monotonic()
            while time.monotonic() - settle < 3:
                rclpy.spin_once(node, timeout_sec=0.05)
            initial = {}
            for i, proc in processes.items():
                cpu = proc.cpu_times()
                initial[i] = (cpu.user+cpu.system, dict(counters[i]))
            memory = {i: [] for i in processes}
            psutil.cpu_percent(interval=None)
            start = time.monotonic()
            while time.monotonic() - start < 20:
                rclpy.spin_once(node, timeout_sec=0.05)
                for i, proc in processes.items():
                    memory[i].append(proc.memory_info().rss/1048576)
            elapsed = time.monotonic()-start
            phase = {"label": label, "elapsed_s": elapsed,
                     "host_cpu_percent_all_cores": psutil.cpu_percent(interval=None), "containers": {}}
            for i, proc in processes.items():
                cpu = proc.cpu_times()
                counts = {k: v-initial[i][1][k] for k,v in counters[i].items()}
                phase["containers"][i] = {"pid": proc.pid, "cmdline": proc.cmdline(),
                    "cpu_percent": (cpu.user+cpu.system-initial[i][0])/elapsed*100,
                    "rss_mib_mean": sum(memory[i])/len(memory[i]), "threads": proc.num_threads(),
                    "counts": counts, "output_hz": counts["detected"]/elapsed,
                    "jpeg_hz": counts["jpeg"]/elapsed,
                    "cpu_ms_per_processed_image": (cpu.user+cpu.system-initial[i][0])*1000/max(1, counts["detected"])}
            result["phases"].append(phase)
            ROOT.joinpath("results.json").write_text(json.dumps(result, indent=2)+"\n")
            print(json.dumps(phase), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
