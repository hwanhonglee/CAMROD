#!/usr/bin/env python3
"""Publish explicit low-rate placeholders for deliberately disabled sensors."""

# Every output topic is absolute so this node can run below any launch
# namespace without changing the public sensing contract. Publishers are
# created only for groups whose ``publish_*`` parameter is true; this is the
# primary protection against overlapping a physical sensor publisher.

import base64
import math
from typing import Dict, Iterable, Tuple

import rclpy
from avg_msgs.msg import AvgBool
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import (
    CameraInfo,
    CompressedImage,
    Image,
    Imu,
    NavSatFix,
    NavSatStatus,
    PointCloud2,
    PointField,
)


GNSS_FIX_TOPIC = "/sensing/gnss/ublox_gps_node/fix"
GNSS_HEADING_TOPIC = "/sensing/gnss/navheading"
IMU_RAW_TOPIC = "/sensing/imu/data_ros"
LIDAR_RAW_TOPIC = "/sensing/lidar/vanjee/points_raw"
LIDAR_FILTERED_TOPIC = "/sensing/lidar/points_filtered"

FRONT_CAMERA_RAW_TOPIC = "/sensing/camera/econ_front/image_raw"
FRONT_CAMERA_COMPRESSED_TOPIC = (
    "/sensing/camera/econ_front/image_rect/compressed"
)
FRONT_CAMERA_INFO_TOPIC = "/sensing/camera/econ_front/camera_info"
REAR_CAMERA_RAW_TOPIC = "/sensing/camera/econ_rear/image_raw"
REAR_CAMERA_COMPRESSED_TOPIC = (
    "/sensing/camera/econ_rear/image_raw/compressed"
)
REAR_CAMERA_INFO_TOPIC = "/sensing/camera/econ_rear/camera_info"

DUMMY_ACTIVE_TOPICS = {
    "gnss": "/sensing/gnss/dummy_active",
    "imu": "/sensing/imu/dummy_active",
    "lidar": "/sensing/lidar/dummy_active",
    "front_camera": "/sensing/camera/econ_front/dummy_active",
    "rear_camera": "/sensing/camera/econ_rear/dummy_active",
}

GNSS_FRAME_ID = "gps"
IMU_FRAME_ID = "imu_link"
LIDAR_FRAME_ID = "lidar_link"
FRONT_CAMERA_FRAME_ID = "camera_front"
REAR_CAMERA_FRAME_ID = "camera_rear"
HIGH_UNCERTAINTY = 1.0e6

# One black 1x1 JPEG generated once with OpenCV/libjpeg quality 75.  Keeping the
# already-compressed payload in the source avoids OpenCV/Pillow dependencies and
# repeated image encoding in hardware-free tests.
BLACK_JPEG_1X1 = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLD"
    "BkSEw8UHRofHh0aHBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/2w"
    "BDAQkJCQwLDBgNDRgyIRwhMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjI"
    "yMjIyMjIyMjIyMjIyMjIyMjIyMjL/wAARCAABAAEDASIAAhEBAxEB/8QAHwAA"
    "AQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAA"
    "F9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRo"
    "lJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqD"
    "hIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1"
    "NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAA"
    "AAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJ"
    "BUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5"
    "OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUl"
    "ZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5e"
    "bn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwD5/ooooA//2Q=="
)


def validate_publish_rate(publish_rate_hz: float) -> float:
    """Validate and return the common low-rate timer frequency."""
    rate = float(publish_rate_hz)
    if not math.isfinite(rate) or rate <= 0.0 or rate > 20.0:
        raise ValueError("publish_rate_hz must be finite and in (0, 20]")
    return rate


def enabled_groups(
    publish_gnss: bool,
    publish_imu: bool,
    publish_lidar: bool,
    publish_front_camera: bool,
    publish_rear_camera: bool,
) -> Tuple[str, ...]:
    """Return enabled group names in stable diagnostic order."""
    requested = (
        ("gnss", publish_gnss),
        ("imu", publish_imu),
        ("lidar", publish_lidar),
        ("front_camera", publish_front_camera),
        ("rear_camera", publish_rear_camera),
    )
    return tuple(name for name, enabled in requested if bool(enabled))


def _set_stamp(message, stamp) -> None:
    message.header.stamp = stamp


def make_camera_info(stamp, frame_id: str) -> CameraInfo:
    """Build calibration-shaped CameraInfo for a 1x1 placeholder image."""
    message = CameraInfo()
    _set_stamp(message, stamp)
    message.header.frame_id = frame_id
    message.height = 1
    message.width = 1
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    message.k = [
        1.0, 0.0, 0.5,
        0.0, 1.0, 0.5,
        0.0, 0.0, 1.0,
    ]
    message.r = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    message.p = [
        1.0, 0.0, 0.5, 0.0,
        0.0, 1.0, 0.5, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return message


def make_raw_camera_image(stamp, frame_id: str) -> Image:
    """Build a valid one-pixel BGR image without allocating a full HD frame."""
    message = Image()
    _set_stamp(message, stamp)
    message.header.frame_id = frame_id
    message.height = 1
    message.width = 1
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = 3
    message.data = bytes((0, 0, 0))
    return message


def make_compressed_camera_image(stamp, frame_id: str) -> CompressedImage:
    """Build a decodable one-pixel JPEG placeholder."""
    message = CompressedImage()
    _set_stamp(message, stamp)
    message.header.frame_id = frame_id
    message.format = "jpeg"
    message.data = BLACK_JPEG_1X1
    return message


def _high_covariance() -> list:
    covariance = [0.0] * 9
    covariance[0] = HIGH_UNCERTAINTY
    covariance[4] = HIGH_UNCERTAINTY
    covariance[8] = HIGH_UNCERTAINTY
    return covariance


def make_gnss_fix(stamp) -> NavSatFix:
    """Build a transport heartbeat that cannot be mistaken for a valid fix."""
    message = NavSatFix()
    _set_stamp(message, stamp)
    message.header.frame_id = GNSS_FRAME_ID
    message.status.status = NavSatStatus.STATUS_NO_FIX
    message.status.service = NavSatStatus.SERVICE_GPS
    message.latitude = math.nan
    message.longitude = math.nan
    message.altitude = math.nan
    message.position_covariance = _high_covariance()
    message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    return message


def make_gnss_heading(stamp) -> Imu:
    """Build an explicitly unusable GNSS-heading heartbeat."""
    message = Imu()
    _set_stamp(message, stamp)
    message.header.frame_id = GNSS_FRAME_ID
    message.orientation.w = 1.0
    message.orientation_covariance = _high_covariance()
    message.angular_velocity_covariance = _high_covariance()
    message.linear_acceleration_covariance = _high_covariance()
    return message


def make_imu_raw(stamp) -> Imu:
    """Build a finite stationary IMU sample with deliberately high uncertainty."""
    message = Imu()
    _set_stamp(message, stamp)
    message.header.frame_id = IMU_FRAME_ID
    message.orientation.w = 1.0
    message.orientation_covariance = _high_covariance()
    message.angular_velocity_covariance = _high_covariance()
    message.linear_acceleration_covariance = _high_covariance()
    return message


def make_empty_point_cloud(stamp) -> PointCloud2:
    """Build a valid zero-point XYZ cloud that cannot create obstacle cost."""
    message = PointCloud2()
    _set_stamp(message, stamp)
    message.header.frame_id = LIDAR_FRAME_ID
    message.height = 1
    message.width = 0
    message.fields = [
        PointField(
            name="x", offset=0, datatype=PointField.FLOAT32, count=1
        ),
        PointField(
            name="y", offset=4, datatype=PointField.FLOAT32, count=1
        ),
        PointField(
            name="z", offset=8, datatype=PointField.FLOAT32, count=1
        ),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 0
    message.data = bytes()
    message.is_dense = True
    return message


class SensingDummyPublisher(Node):
    """Conditionally publish disabled-sensor contracts from one low-rate node."""

    def __init__(self) -> None:
        super().__init__("sensing_dummy_publisher")

        self._groups = enabled_groups(
            self.declare_parameter("publish_gnss", False).value,
            self.declare_parameter("publish_imu", False).value,
            self.declare_parameter("publish_lidar", False).value,
            self.declare_parameter("publish_front_camera", False).value,
            self.declare_parameter("publish_rear_camera", False).value,
        )
        self._publish_rate_hz = validate_publish_rate(
            self.declare_parameter("publish_rate_hz", 2.0).value
        )

        # Do not use rclpy.Node's internal ``_publishers`` member name.
        self._data_publishers: Dict[str, object] = {}
        self._dummy_active_publishers: Dict[str, object] = {}
        self._setup_publishers()

        if not self._groups:
            self.get_logger().warning(
                "No sensing dummy group enabled; no topics will be published"
            )
            self._timer = None
            return

        self._timer = self.create_timer(
            1.0 / self._publish_rate_hz, self._publish_all
        )
        # Publish the transient state immediately for late-starting diagnostics,
        # then keep it fresh on the same low-rate timer as the data contracts.
        self._publish_all()
        self.get_logger().warning(
            "DUMMY SENSING ACTIVE: physical data is disabled for "
            + ", ".join(self._groups)
            + f"; publishing explicit placeholders at "
            f"{self._publish_rate_hz:.1f} Hz"
        )

    def _add_publisher(self, key: str, message_type, topic: str) -> None:
        self._data_publishers[key] = self.create_publisher(
            message_type, topic, qos_profile_sensor_data
        )

    def _setup_publishers(self) -> None:
        state_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        for group in self._groups:
            self._dummy_active_publishers[group] = self.create_publisher(
                AvgBool, DUMMY_ACTIVE_TOPICS[group], state_qos
            )

        if "gnss" in self._groups:
            self._add_publisher("gnss_fix", NavSatFix, GNSS_FIX_TOPIC)
            self._add_publisher("gnss_heading", Imu, GNSS_HEADING_TOPIC)
        if "imu" in self._groups:
            self._add_publisher("imu_raw", Imu, IMU_RAW_TOPIC)
        if "lidar" in self._groups:
            self._add_publisher(
                "lidar_raw", PointCloud2, LIDAR_RAW_TOPIC
            )
            self._add_publisher(
                "lidar_filtered", PointCloud2, LIDAR_FILTERED_TOPIC
            )
        if "front_camera" in self._groups:
            self._add_publisher(
                "front_camera_raw", Image, FRONT_CAMERA_RAW_TOPIC
            )
            self._add_publisher(
                "front_camera_compressed",
                CompressedImage,
                FRONT_CAMERA_COMPRESSED_TOPIC,
            )
            self._add_publisher(
                "front_camera_info", CameraInfo, FRONT_CAMERA_INFO_TOPIC
            )
        if "rear_camera" in self._groups:
            self._add_publisher(
                "rear_camera_raw", Image, REAR_CAMERA_RAW_TOPIC
            )
            self._add_publisher(
                "rear_camera_compressed",
                CompressedImage,
                REAR_CAMERA_COMPRESSED_TOPIC,
            )
            self._add_publisher(
                "rear_camera_info", CameraInfo, REAR_CAMERA_INFO_TOPIC
            )

    def _publish_all(self) -> None:
        stamp = self.get_clock().now().to_msg()

        for publisher in self._dummy_active_publishers.values():
            active = AvgBool()
            active.data = True
            publisher.publish(active)

        if "gnss" in self._groups:
            self._data_publishers["gnss_fix"].publish(
                make_gnss_fix(stamp)
            )
            self._data_publishers["gnss_heading"].publish(
                make_gnss_heading(stamp)
            )
        if "imu" in self._groups:
            self._data_publishers["imu_raw"].publish(make_imu_raw(stamp))
        if "lidar" in self._groups:
            self._data_publishers["lidar_raw"].publish(
                make_empty_point_cloud(stamp)
            )
            self._data_publishers["lidar_filtered"].publish(
                make_empty_point_cloud(stamp)
            )
        if "front_camera" in self._groups:
            self._data_publishers["front_camera_raw"].publish(
                make_raw_camera_image(stamp, FRONT_CAMERA_FRAME_ID)
            )
            self._data_publishers["front_camera_compressed"].publish(
                make_compressed_camera_image(
                    stamp, FRONT_CAMERA_FRAME_ID
                )
            )
            self._data_publishers["front_camera_info"].publish(
                make_camera_info(stamp, FRONT_CAMERA_FRAME_ID)
            )
        if "rear_camera" in self._groups:
            self._data_publishers["rear_camera_raw"].publish(
                make_raw_camera_image(stamp, REAR_CAMERA_FRAME_ID)
            )
            self._data_publishers["rear_camera_compressed"].publish(
                make_compressed_camera_image(stamp, REAR_CAMERA_FRAME_ID)
            )
            self._data_publishers["rear_camera_info"].publish(
                make_camera_info(stamp, REAR_CAMERA_FRAME_ID)
            )


def main(args: Iterable[str] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SensingDummyPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (TypeError, ValueError) as exc:
        if node is not None:
            node.get_logger().fatal(
                f"invalid sensing dummy configuration: {exc}"
            )
        else:
            rclpy.logging.get_logger("sensing_dummy_publisher").fatal(
                f"invalid sensing dummy configuration: {exc}"
            )
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
