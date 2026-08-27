"""Relay CARLA standard sensor messages onto CAMROD's canonical I/O topics."""

import copy
import math
import time

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2


_IMAGE_LAYOUTS = {
    "mono8": (1, None),
    "bgr8": (3, None),
    "rgb8": (3, cv2.COLOR_RGB2BGR),
    "bgra8": (4, cv2.COLOR_BGRA2BGR),
    "rgba8": (4, cv2.COLOR_RGBA2BGR),
}


def encode_image_jpeg(message: Image, quality: int) -> bytes:
    """Encode the common CARLA/ROS image layouts without a helper process."""
    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    encoding = str(message.encoding).lower()
    if width <= 0 or height <= 0 or step <= 0:
        raise ValueError("image dimensions and step must be positive")
    if encoding not in _IMAGE_LAYOUTS:
        raise ValueError(f"unsupported image encoding: {message.encoding!r}")
    channels, conversion = _IMAGE_LAYOUTS[encoding]
    packed_row_bytes = width * channels
    if step < packed_row_bytes:
        raise ValueError(
            f"image step {step} is smaller than packed row {packed_row_bytes}"
        )
    payload = np.frombuffer(bytes(message.data), dtype=np.uint8)
    required_bytes = height * step
    if payload.size < required_bytes:
        raise ValueError(
            f"image payload has {payload.size} bytes, expected {required_bytes}"
        )
    rows = payload[:required_bytes].reshape(height, step)
    pixels = rows[:, :packed_row_bytes]
    if channels == 1:
        pixels = pixels.reshape(height, width)
    else:
        pixels = pixels.reshape(height, width, channels)
    if conversion is not None:
        pixels = cv2.cvtColor(pixels, conversion)
    success, encoded = cv2.imencode(
        ".jpg",
        np.ascontiguousarray(pixels),
        [cv2.IMWRITE_JPEG_QUALITY, int(quality)],
    )
    if not success or encoded.size == 0:
        raise ValueError("OpenCV could not encode the image as JPEG")
    return encoded.tobytes()


class CarlaSensorRelayNode(Node):
    """Keep simulator transport conversion separate from CAMROD algorithms."""

    def __init__(self, **node_kwargs):
        super().__init__("carla_sensor_relay", **node_kwargs)

        self.front_image_input = self.declare_parameter(
            "front_image_input", "/carla/ego_vehicle/rgb_view/image"
        ).value
        self.front_info_input = self.declare_parameter(
            "front_info_input", "/carla/ego_vehicle/rgb_view/camera_info"
        ).value
        self.rear_image_input = self.declare_parameter(
            "rear_image_input", "/carla/ego_vehicle/rgb_rear/image"
        ).value
        self.rear_info_input = self.declare_parameter(
            "rear_info_input", "/carla/ego_vehicle/rgb_rear/camera_info"
        ).value
        self.lidar_input = self.declare_parameter(
            "lidar_input", "/carla/ego_vehicle/lidar_front"
        ).value
        self.front_image_outputs = list(self.declare_parameter(
            "front_image_outputs",
            [
                "/sensing/camera/econ_front/image_raw",
                "/sensing/camera/econ_front/image_rect",
            ],
        ).value)
        self.front_info_output = self.declare_parameter(
            "front_info_output", "/sensing/camera/econ_front/camera_info"
        ).value
        self.rear_image_outputs = list(self.declare_parameter(
            "rear_image_outputs",
            [
                "/sensing/camera/econ_rear/image_raw",
                "/sensing/camera/econ_rear/image_rect",
            ],
        ).value)
        self.rear_info_output = self.declare_parameter(
            "rear_info_output", "/sensing/camera/econ_rear/camera_info"
        ).value
        self.lidar_output = self.declare_parameter(
            "lidar_output", "/sensing/lidar/vanjee/points_raw"
        ).value
        self.front_compressed_output = self.declare_parameter(
            "front_compressed_output",
            "/sensing/camera/econ_front/image_rect/compressed",
        ).value
        self.rear_compressed_output = self.declare_parameter(
            "rear_compressed_output",
            "/sensing/camera/econ_rear/image_raw/compressed",
        ).value
        self.publish_compressed_images = bool(self.declare_parameter(
            "publish_compressed_images", True
        ).value)
        self.jpeg_quality = int(self.declare_parameter(
            "jpeg_quality", 80
        ).value)
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in [1, 100]")
        self.front_frame_id = self.declare_parameter(
            "front_frame_id", "camera_front_link"
        ).value
        self.rear_frame_id = self.declare_parameter(
            "rear_frame_id", "camera_rear_link"
        ).value
        self.preserve_lidar_frame = bool(self.declare_parameter(
            "preserve_lidar_frame", True
        ).value)
        self.lidar_frame_id = self.declare_parameter(
            "lidar_frame_id", "lidar_link"
        ).value
        self.stamp_with_reception_time = bool(self.declare_parameter(
            "stamp_with_reception_time", True
        ).value)
        self.stream_timeout_sec = float(self.declare_parameter(
            "stream_timeout_sec", 3.0
        ).value)
        if not math.isfinite(self.stream_timeout_sec) or self.stream_timeout_sec <= 0.0:
            raise ValueError("stream_timeout_sec must be finite and > 0")

        self._front_image_publishers = [
            # CAMROD has both reliable and best-effort image consumers.  A
            # reliable local publisher is compatible with both subscription
            # policies; JPEG output is generated in this node below.
            self.create_publisher(Image, topic, 10)
            for topic in self.front_image_outputs
        ]
        self._rear_image_publishers = [
            self.create_publisher(Image, topic, 10)
            for topic in self.rear_image_outputs
        ]
        self._front_info_publisher = self.create_publisher(
            CameraInfo, self.front_info_output, 10
        )
        self._rear_info_publisher = self.create_publisher(
            CameraInfo, self.rear_info_output, 10
        )
        self._front_compressed_publisher = None
        self._rear_compressed_publisher = None
        if self.publish_compressed_images:
            self._front_compressed_publisher = self.create_publisher(
                CompressedImage, self.front_compressed_output, 10
            )
            self._rear_compressed_publisher = self.create_publisher(
                CompressedImage, self.rear_compressed_output, 10
            )
        self._lidar_publisher = self.create_publisher(
            PointCloud2, self.lidar_output, qos_profile_sensor_data
        )

        self._subscriptions = [
            self.create_subscription(
                Image, self.front_image_input, self._on_front_image,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                CameraInfo, self.front_info_input, self._on_front_info,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Image, self.rear_image_input, self._on_rear_image,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                CameraInfo, self.rear_info_input, self._on_rear_info,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                PointCloud2, self.lidar_input, self._on_lidar,
                qos_profile_sensor_data,
            ),
        ]
        self._last_seen = {
            "front_image": None,
            "front_info": None,
            "rear_image": None,
            "rear_info": None,
            "lidar": None,
        }
        if self.publish_compressed_images:
            self._last_seen.update({
                "front_compressed": None,
                "rear_compressed": None,
            })
        self._compression_errors = {}
        self._status_publisher = self.create_publisher(
            DiagnosticArray, "/camrod_carla/sensor_relay/status", 10
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self._status_timer = self.create_timer(0.5, self._publish_status)

    def _stamp(self, header):
        output = copy.deepcopy(header)
        if self.stamp_with_reception_time:
            output.stamp = self.get_clock().now().to_msg()
        return output

    def _on_front_image(self, message):
        output = copy.deepcopy(message)
        output.header = self._stamp(message.header)
        output.header.frame_id = self.front_frame_id
        for publisher in self._front_image_publishers:
            publisher.publish(output)
        self._last_seen["front_image"] = time.monotonic()
        self._publish_compressed(
            output,
            self._front_compressed_publisher,
            "front_compressed",
        )

    def _on_front_info(self, message):
        output = copy.deepcopy(message)
        output.header = self._stamp(message.header)
        output.header.frame_id = self.front_frame_id
        self._front_info_publisher.publish(output)
        self._last_seen["front_info"] = time.monotonic()

    def _on_rear_image(self, message):
        output = copy.deepcopy(message)
        output.header = self._stamp(message.header)
        output.header.frame_id = self.rear_frame_id
        for publisher in self._rear_image_publishers:
            publisher.publish(output)
        self._last_seen["rear_image"] = time.monotonic()
        self._publish_compressed(
            output,
            self._rear_compressed_publisher,
            "rear_compressed",
        )

    def _on_rear_info(self, message):
        output = copy.deepcopy(message)
        output.header = self._stamp(message.header)
        output.header.frame_id = self.rear_frame_id
        self._rear_info_publisher.publish(output)
        self._last_seen["rear_info"] = time.monotonic()

    def _on_lidar(self, message):
        output = copy.deepcopy(message)
        output.header = self._stamp(message.header)
        if not self.preserve_lidar_frame:
            output.header.frame_id = self.lidar_frame_id
        self._lidar_publisher.publish(output)
        self._last_seen["lidar"] = time.monotonic()

    def _publish_compressed(self, image, publisher, stream_name):
        if publisher is None:
            return
        try:
            payload = encode_image_jpeg(image, self.jpeg_quality)
        except (TypeError, ValueError, cv2.error) as error:
            message = str(error)
            if self._compression_errors.get(stream_name) != message:
                self.get_logger().error(
                    f"{stream_name} JPEG encoding failed: {message}"
                )
            self._compression_errors[stream_name] = message
            return
        output = CompressedImage()
        output.header = copy.deepcopy(image.header)
        output.format = "jpeg"
        output.data = payload
        publisher.publish(output)
        self._last_seen[stream_name] = time.monotonic()
        self._compression_errors.pop(stream_name, None)

    def stream_ages(self, now_monotonic):
        return {
            name: (
                math.inf if seen is None else float(now_monotonic) - seen
            )
            for name, seen in self._last_seen.items()
        }

    def _publish_status(self):
        ages = self.stream_ages(time.monotonic())
        stale = [
            name for name, age in ages.items()
            if age < 0.0 or age > self.stream_timeout_sec
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "camrod_carla/sensor_relay"
        status.hardware_id = "vehicle.ranger.default"
        if stale:
            status.level = DiagnosticStatus.WARN
            status.message = "CARLA render sensors unavailable/stale: " + ",".join(stale)
        else:
            status.level = DiagnosticStatus.OK
            status.message = "CARLA render sensors active"
        status.values = [
            KeyValue(key=name + "_age_sec", value=str(age))
            for name, age in ages.items()
        ]
        status.values.extend(
            KeyValue(key=name + "_error", value=message)
            for name, message in sorted(self._compression_errors.items())
        )
        array.status = [status]
        self._status_publisher.publish(array)
        self._diagnostics_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaSensorRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
