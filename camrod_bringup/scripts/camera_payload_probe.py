#!/usr/bin/env python3
"""Validate the live front-camera compressed payload without moving the robot."""

import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from avg_msgs.msg import AvgBool
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage


# HH_260730 / TODOLIST 2 - A healthy topic rate alone cannot prove that NvJPEG
# produced a non-empty, decodable frame of the configured dimensions.
class CameraPayloadProbe(Node):

    def __init__(self, topic: str, camera_info_topic: str, dummy_topic: str):
        super().__init__("camera_payload_probe")
        self.topic = topic
        self.camera_info_topic = camera_info_topic
        self.dummy_topic = dummy_topic
        self.frame_count = 0
        self.decode_failures = 0
        self.shape_mismatches = 0
        self.empty_payloads = 0
        self.dummy_true_count = 0
        self.camera_info_count = 0
        self.min_payload_bytes: int | None = None
        self.max_payload_bytes = 0
        self.expected_width = 0
        self.expected_height = 0
        self.decoded_width = 0
        self.decoded_height = 0
        self.formats: set[str] = set()

        self.create_subscription(
            CompressedImage,
            topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(AvgBool, dummy_topic, self._on_dummy, 10)

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info_count += 1
        self.expected_width = int(message.width)
        self.expected_height = int(message.height)

    def _on_dummy(self, message: AvgBool) -> None:
        if message.data:
            self.dummy_true_count += 1

    def _on_image(self, message: CompressedImage) -> None:
        self.frame_count += 1
        payload_size = len(message.data)
        self.formats.add(message.format)
        self.max_payload_bytes = max(self.max_payload_bytes, payload_size)
        if self.min_payload_bytes is None:
            self.min_payload_bytes = payload_size
        else:
            self.min_payload_bytes = min(self.min_payload_bytes, payload_size)

        if payload_size == 0:
            self.empty_payloads += 1
            self.decode_failures += 1
            return

        encoded = np.frombuffer(message.data, dtype=np.uint8)
        decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if decoded is None or decoded.size == 0:
            self.decode_failures += 1
            return

        self.decoded_height, self.decoded_width = decoded.shape[:2]
        if (
            self.expected_width > 0
            and self.expected_height > 0
            and (
                self.decoded_width != self.expected_width
                or self.decoded_height != self.expected_height
            )
        ):
            self.shape_mismatches += 1

    def publisher_identities(self) -> list[str]:
        return sorted(
            {
                f"{info.node_namespace.rstrip('/')}/{info.node_name}"
                for info in self.get_publishers_info_by_topic(self.topic)
            }
        )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Decode and validate live front-camera JPEG frames."
    )
    parser.add_argument(
        "--topic",
        default="/sensing/camera/econ_front/image_rect/compressed",
    )
    parser.add_argument(
        "--camera-info-topic",
        default="/sensing/camera/econ_front/camera_info",
    )
    parser.add_argument(
        "--dummy-topic",
        default="/sensing/camera/econ_front/dummy_active",
    )
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--min-rate-hz", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.duration <= 0.0:
        raise SystemExit("--duration must be positive")
    if args.min_rate_hz < 0.0:
        raise SystemExit("--min-rate-hz must be non-negative")

    rclpy.init(args=[])
    node = CameraPayloadProbe(
        args.topic,
        args.camera_info_topic,
        args.dummy_topic,
    )
    started = time.monotonic()
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        interrupted = True

    elapsed = max(time.monotonic() - started, 1.0e-6)
    publishers = node.publisher_identities()
    minimum_frames = math.floor(args.min_rate_hz * elapsed * 0.8)
    observed_rate_hz = node.frame_count / elapsed
    payload_min = node.min_payload_bytes if node.min_payload_bytes is not None else 0
    print(
        "CAMERA_PAYLOAD_SUMMARY "
        f"elapsed_s={elapsed:.3f} frames={node.frame_count} "
        f"rate_hz={observed_rate_hz:.3f} payload_min_bytes={payload_min} "
        f"payload_max_bytes={node.max_payload_bytes} "
        f"decoded={node.frame_count - node.decode_failures} "
        f"decode_failures={node.decode_failures} "
        f"empty_payloads={node.empty_payloads} "
        f"shape_mismatches={node.shape_mismatches} "
        f"camera_info={node.camera_info_count} "
        f"expected_shape={node.expected_width}x{node.expected_height} "
        f"last_decoded_shape={node.decoded_width}x{node.decoded_height} "
        f"dummy_true={node.dummy_true_count} "
        f"publishers={len(publishers)} "
        f"publisher_nodes={','.join(publishers) or 'none'} "
        f"formats={','.join(sorted(node.formats)) or 'none'}"
    )

    failures = []
    if interrupted:
        failures.append("probe interrupted")
    if node.frame_count < minimum_frames:
        failures.append(
            f"only {node.frame_count} frames; expected at least {minimum_frames}"
        )
    if len(publishers) != 1:
        failures.append(f"expected exactly one image publisher, found {len(publishers)}")
    if node.camera_info_count == 0:
        failures.append("no CameraInfo received")
    if node.expected_width <= 1 or node.expected_height <= 1:
        failures.append(
            f"invalid CameraInfo shape {node.expected_width}x{node.expected_height}"
        )
    if node.decode_failures > 0:
        failures.append(f"{node.decode_failures} JPEG decode failures")
    if node.shape_mismatches > 0:
        failures.append(f"{node.shape_mismatches} decoded shape mismatches")
    if node.dummy_true_count > 0:
        failures.append("front-camera dummy_active=true was observed")

    node.destroy_node()
    rclpy.shutdown()
    if failures:
        print("CAMERA_PAYLOAD_FAIL " + "; ".join(failures))
        return 1
    print("CAMERA_PAYLOAD_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
