#!/usr/bin/env python3
"""One genuine ROS rear-camera PNG; no publishers, UI actions or parameter writes."""
import datetime
import hashlib
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    rclpy.init(args=[])
    node = rclpy.create_node("optional_dock_rear_camera_readonly")
    frame = []
    topic = "/sensing/camera/econ_rear/image_rect"
    def receive(msg):
        if not frame:
            frame.append((msg, datetime.datetime.now(datetime.timezone.utc)))
    sub = node.create_subscription(Image, topic, receive, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 5.0
        while not frame and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not frame:
            raise RuntimeError("No actual rear ROS image received within 5 seconds")
        msg, received = frame[0]
        stamp = received.strftime("%Y%m%dT%H%M%S%fZ")
        target = Path(__file__).resolve().parent
        png_path = target / ("actual_rear_" + stamp + ".png")
        json_path = target / ("actual_rear_" + stamp + ".json")
        if png_path.exists() or json_path.exists():
            raise RuntimeError("Refusing to overwrite a previous actual camera observation")
        pixels = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if not cv2.imwrite(str(png_path), pixels):
            raise RuntimeError("PNG encoding failed")
        payload = png_path.read_bytes()
        result = {"received_utc": received.isoformat(), "topic": topic,
                  "source": "actual ROS Image callback, no image synthesis or overlays",
                  "width": msg.width, "height": msg.height, "source_encoding": msg.encoding,
                  "frame_id": msg.header.frame_id,
                  "ros_stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                  "png": {"path": str(png_path), "bytes": len(payload),
                          "sha256": hashlib.sha256(payload).hexdigest()},
                  "ros_commands_published": False, "ui_or_motion_commands_sent": False}
        with json_path.open("x") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
        print(json.dumps({**result, "sidecar": str(json_path)}, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
