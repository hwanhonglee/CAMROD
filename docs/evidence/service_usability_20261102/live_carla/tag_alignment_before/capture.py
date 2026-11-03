#!/usr/bin/env python3
"""HH_261102 - Capture current real rear pixels and native tag/parking diagnostics without controls."""

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


# HH_261102 - Reuse the identical passive capture for before/after without replacing prior evidence.
root = Path(sys.argv[1]) if len(sys.argv) == 2 else Path(__file__).parent
root.mkdir(parents=True, exist_ok=True)
images = {
    "/sensing/camera/econ_rear/image_raw": "rear_raw.png",
    "/sensing/camera/econ_rear/image_rect": "rear_rect.png",
    "/perception/apriltag_parking_detector/debug_image": "detector_debug.png",
}
topics = {**{name: "sensor_msgs/msg/Image" for name in images},
          "/sensing/camera/econ_rear/camera_info": "sensor_msgs/msg/CameraInfo",
          "/perception/apriltag_parking_detector/tag_pose": "avg_msgs/msg/AvgAprilTagPose",
          "/localization/pose": "avg_msgs/msg/AvgPoseStamped",
          "/parking/private/apriltag/status": "avg_msgs/msg/ModuleState",
          "/parking/private/reverse/status": "avg_msgs/msg/ModuleState",
          "/control/drop_zone_maneuver_controller/status": "avg_msgs/msg/ModuleState"}
bridge = CvBridge()
counts, latest, saved = {}, {}, {}
rclpy.init(args=[])
node = rclpy.create_node("carla_tag_alignment_passive_capture")


def receive(topic, message):
    counts[topic] = counts.get(topic, 0) + 1
    if topic in images:
        if topic in saved:
            return
        ok, encoded = cv2.imencode(".png", bridge.imgmsg_to_cv2(message, desired_encoding="bgr8"))
        if not ok:
            raise RuntimeError("actual image PNG encoding failed")
        data = bytes(encoded)
        assert data.startswith(b"\x89PNG\r\n\x1a\n")
        with (root / images[topic]).open("xb") as output:
            output.write(data)
        saved[topic] = {"path": images[topic], "received_utc": datetime.now(timezone.utc).isoformat(),
                        "received_monotonic_s": time.monotonic(), "header": message_to_ordereddict(message.header),
                        "width": message.width, "height": message.height, "source_encoding": message.encoding,
                        "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
    else:
        latest[topic] = {"received_utc": datetime.now(timezone.utc).isoformat(),
                         "received_monotonic_s": time.monotonic(), "data": message_to_ordereddict(message)}


try:
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [node.create_subscription(get_message(kind), topic,
                     lambda message, topic=topic: receive(topic, message), qos)
                     for topic, kind in topics.items()]
    started = time.monotonic()
    while time.monotonic() - started < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    graph = {topic: {"publisher_count": node.count_publishers(topic),
                    "publishers": [{"node": item.node_namespace.rstrip("/") + "/" + item.node_name,
                                    "type": item.topic_type} for item in node.get_publishers_info_by_topic(topic)]}
             for topic in topics}
finally:
    node.destroy_node()
    rclpy.shutdown()

# HH_261102 - Read effective live parameters rather than inferring the running target from YAML defaults.
parameters = {}
for name in ["/perception/apriltag_parking_detector", "/parking/apriltag_parking_controller",
             "/parking/reverse_parking_controller", "/control/drop_zone_maneuver_controller"]:
    try:
        result = subprocess.run(["ros2", "param", "dump", name], capture_output=True, text=True, timeout=6)
        parameters[name] = {"exit_code": result.returncode, "stdout": result.stdout, "stderr": result.stderr}
    except subprocess.TimeoutExpired:
        parameters[name] = {"error": "read-only parameter query timed out after6s"}
record = {"schema": "camrod.live_carla.tag_alignment_before.v1",
          "HH_261102": "Preserve actual pixels, topic counts and live station parameters before any alignment change.",
          "captured_utc": datetime.now(timezone.utc).isoformat(), "duration_s": 5,
          "scope": "Stationary failed-trial CARLA rear sensor streams; no motion, SOC, lease, parameter or map mutation.",
          "counts": counts, "images": saved, "latest": latest, "graph": graph, "live_parameters": parameters}
with (root / "capture.json").open("x") as output:
    json.dump(record, output, ensure_ascii=False, indent=2)
print(json.dumps({"images": saved, "counts": counts,
                  "tag_pose_publishers": graph["/perception/apriltag_parking_detector/tag_pose"]}, ensure_ascii=False))
