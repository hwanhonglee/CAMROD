"""Relay CARLA standard sensor messages onto CAMROD's canonical I/O topics."""

import copy
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


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
            self.create_publisher(Image, topic, qos_profile_sensor_data)
            for topic in self.front_image_outputs
        ]
        self._rear_image_publishers = [
            self.create_publisher(Image, topic, qos_profile_sensor_data)
            for topic in self.rear_image_outputs
        ]
        self._front_info_publisher = self.create_publisher(
            CameraInfo, self.front_info_output, qos_profile_sensor_data
        )
        self._rear_info_publisher = self.create_publisher(
            CameraInfo, self.rear_info_output, qos_profile_sensor_data
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
        self._status_publisher = self.create_publisher(
            DiagnosticArray, "/camrod_carla/sensor_relay/status", 10
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
        array.status = [status]
        self._status_publisher.publish(array)


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
