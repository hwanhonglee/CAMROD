#!/usr/bin/env python3
"""Require fresh CARLA camera/LiDAR messages before CAMROD UI bringup."""

import argparse
import json
import math
import re
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from camrod_carla_adapter.sensor_relay_node import encode_image_jpeg


ROLE_PATTERN = re.compile(r"^[A-Za-z0-9_]+$")


class SensorStreamProbe(Node):
    """Require sustained, fresh, structurally valid visual streams."""

    def __init__(
        self,
        role_name: str,
        min_rate_hz: float,
        min_observation_seconds: float,
        max_sample_age_seconds: float,
    ):
        super().__init__("camrod_carla_sensor_stream_preflight")
        self.min_rate_hz = min_rate_hz
        self.min_observation_seconds = min_observation_seconds
        self.max_sample_age_seconds = max_sample_age_seconds
        root = f"/carla/{role_name}"
        self.required = {
            "front_image": f"{root}/rgb_view/image",
            "front_camera_info": f"{root}/rgb_view/camera_info",
            "rear_image": f"{root}/rgb_rear/image",
            "rear_camera_info": f"{root}/rgb_rear/camera_info",
            "front_lidar": f"{root}/lidar_front",
        }
        self.received = {}
        self.invalid = {}
        self.sample_times = {name: [] for name in self.required}
        self._subscriptions = [
            self.create_subscription(
                Image,
                self.required["front_image"],
                lambda message: self._on_image("front_image", message),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                CameraInfo,
                self.required["front_camera_info"],
                lambda message: self._on_camera_info(
                    "front_camera_info", message
                ),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image,
                self.required["rear_image"],
                lambda message: self._on_image("rear_image", message),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                CameraInfo,
                self.required["rear_camera_info"],
                lambda message: self._on_camera_info(
                    "rear_camera_info", message
                ),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                PointCloud2,
                self.required["front_lidar"],
                self._on_lidar,
                qos_profile_sensor_data,
            ),
        ]

    def _accept(self, name: str, details: dict):
        now = time.monotonic()
        samples = self.sample_times[name]
        samples.append(now)
        if len(samples) > 200:
            del samples[:-200]
        self.received[name] = {
            "topic": self.required[name],
            **details,
        }
        self.invalid.pop(name, None)

    def _reject(self, name: str, reason: str):
        self.invalid[name] = reason
        self.received.pop(name, None)
        self.sample_times[name].clear()

    def _on_image(self, name: str, message: Image):
        if message.width <= 0 or message.height <= 0 or message.step <= 0:
            self._reject(name, "empty image payload")
            return
        expected_bytes = int(message.height) * int(message.step)
        if len(message.data) < expected_bytes:
            self._reject(
                name,
                f"short image payload {len(message.data)}/{expected_bytes}",
            )
            return
        try:
            jpeg_payload = encode_image_jpeg(message, 80)
        except (TypeError, ValueError, cv2.error) as error:
            self._reject(name, f"JPEG-incompatible image: {error}")
            return
        self._accept(
            name,
            {
                "width": int(message.width),
                "height": int(message.height),
                "encoding": str(message.encoding),
                "bytes": len(message.data),
                "jpeg_bytes": len(jpeg_payload),
            },
        )

    def _on_camera_info(self, name: str, message: CameraInfo):
        if message.width <= 0 or message.height <= 0:
            self._reject(name, "invalid camera dimensions")
            return
        calibration = [*message.k, *message.p]
        if (
            not all(math.isfinite(float(value)) for value in calibration)
            or float(message.k[0]) <= 0.0
            or float(message.k[4]) <= 0.0
            or float(message.p[0]) <= 0.0
            or float(message.p[5]) <= 0.0
        ):
            self._reject(name, "invalid camera calibration matrix")
            return
        self._accept(
            name,
            {"width": int(message.width), "height": int(message.height)},
        )

    def _on_lidar(self, message: PointCloud2):
        expected_fields = {"x", "y", "z", "intensity"}
        fields = {field.name: field for field in message.fields}
        fields_ready = expected_fields <= set(fields) and all(
            fields[name].datatype == PointField.FLOAT32
            and fields[name].count == 1
            for name in expected_fields
        )
        minimum_row_step = int(message.width) * int(message.point_step)
        expected_bytes = int(message.height) * int(message.row_step)
        if (
            message.width <= 0
            or message.height <= 0
            or message.point_step <= 0
            or message.row_step < minimum_row_step
            or len(message.data) < expected_bytes
            or not fields_ready
        ):
            self._reject(
                "front_lidar", "invalid point cloud payload or field layout"
            )
            return
        self._accept(
            "front_lidar",
            {
                "width": int(message.width),
                "height": int(message.height),
                "point_step": int(message.point_step),
                "bytes": len(message.data),
            },
        )

    def stream_metrics(self, name: str, now: float | None = None) -> dict:
        samples = self.sample_times[name]
        if now is None:
            now = time.monotonic()
        span = samples[-1] - samples[0] if len(samples) >= 2 else 0.0
        rate = (len(samples) - 1) / span if span > 0.0 else 0.0
        age = now - samples[-1] if samples else math.inf
        rate_ready = (
            span >= self.min_observation_seconds
            and rate >= self.min_rate_hz
        )
        fresh_ready = 0.0 <= age <= self.max_sample_age_seconds
        return {
            "samples": len(samples),
            "observation_seconds": round(span, 6),
            "rate_hz": round(rate, 3),
            "sample_age_seconds": (
                round(age, 6) if math.isfinite(age) else None
            ),
            "rate_ready": rate_ready,
            "fresh_ready": fresh_ready,
            "stream_ready": rate_ready and fresh_ready,
        }

    def payload_pairs_ready(self) -> bool:
        for image_name, info_name in (
            ("front_image", "front_camera_info"),
            ("rear_image", "rear_camera_info"),
        ):
            image = self.received.get(image_name, {})
            info = self.received.get(info_name, {})
            if not image or not info:
                return False
            if (image.get("width"), image.get("height")) != (
                info.get("width"),
                info.get("height"),
            ):
                return False
        return True

    def complete(self) -> bool:
        return (
            set(self.received) == set(self.required)
            and not self.invalid
            and self.payload_pairs_ready()
            and all(
                self.stream_metrics(name)["stream_ready"]
                for name in self.required
            )
        )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--role-name", default="ego_vehicle")
    parser.add_argument("--timeout-seconds", type=float, default=8.0)
    parser.add_argument("--min-rate-hz", type=float, default=8.0)
    parser.add_argument(
        "--min-observation-seconds", type=float, default=1.0
    )
    parser.add_argument(
        "--max-sample-age-seconds", type=float, default=0.5
    )
    return parser.parse_args()


def main():
    arguments = parse_args()
    if not ROLE_PATTERN.fullmatch(arguments.role_name):
        raise SystemExit(
            "role name may contain only letters, digits and underscore"
        )
    if (
        not math.isfinite(arguments.timeout_seconds)
        or arguments.timeout_seconds <= 0.0
        or arguments.timeout_seconds > 60.0
    ):
        raise SystemExit("timeout must be finite and in (0, 60] seconds")
    if (
        not math.isfinite(arguments.min_rate_hz)
        or arguments.min_rate_hz <= 0.0
        or arguments.min_rate_hz > 1000.0
    ):
        raise SystemExit("minimum rate must be finite and in (0, 1000] Hz")
    if (
        not math.isfinite(arguments.min_observation_seconds)
        or arguments.min_observation_seconds <= 0.0
        or arguments.min_observation_seconds >= arguments.timeout_seconds
    ):
        raise SystemExit(
            "minimum observation must be finite, positive, and below timeout"
        )
    if (
        not math.isfinite(arguments.max_sample_age_seconds)
        or arguments.max_sample_age_seconds <= 0.0
        or arguments.max_sample_age_seconds >= arguments.timeout_seconds
    ):
        raise SystemExit(
            "maximum sample age must be finite, positive, and below timeout"
        )

    rclpy.init(args=[])
    node = SensorStreamProbe(
        arguments.role_name,
        arguments.min_rate_hz,
        arguments.min_observation_seconds,
        arguments.max_sample_age_seconds,
    )
    deadline = time.monotonic() + arguments.timeout_seconds
    interrupted = False
    try:
        while time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.1)
        accepted = node.complete()
        missing = {
            name: topic
            for name, topic in node.required.items()
            if name not in node.received
        }
        metrics = {
            name: node.stream_metrics(name) for name in node.required
        }
        stream_rejected = {
            name: metric
            for name, metric in metrics.items()
            if not metric["stream_ready"]
        }
        report = {
            "accepted": accepted,
            "role_name": arguments.role_name,
            "received": node.received,
            "missing": missing,
            "invalid": node.invalid,
            "metrics": metrics,
            "minimum_rate_hz": arguments.min_rate_hz,
            "minimum_observation_seconds": (
                arguments.min_observation_seconds
            ),
            "maximum_sample_age_seconds": (
                arguments.max_sample_age_seconds
            ),
        }
        print(json.dumps(report, sort_keys=True))
        if not accepted:
            reasons = []
            if missing:
                reasons.append(
                    "missing="
                    + ",".join(
                        f"{name}:{topic}" for name, topic in missing.items()
                    )
                )
            if node.invalid:
                reasons.append(
                    "invalid="
                    + ",".join(
                        f"{name}:{reason}"
                        for name, reason in node.invalid.items()
                    )
                )
            if stream_rejected:
                reasons.append(
                    "rate/freshness="
                    + ",".join(
                        f"{name}:{metric['rate_hz']}Hz/"
                        f"{metric['observation_seconds']}s/"
                        f"age={metric['sample_age_seconds']}s"
                        for name, metric in stream_rejected.items()
                    )
                )
            raise SystemExit(
                "CARLA visual sensor streams are not ready: "
                + "; ".join(reasons)
            )
    except KeyboardInterrupt:
        interrupted = True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if interrupted:
        raise SystemExit(130)


if __name__ == "__main__":
    main()
