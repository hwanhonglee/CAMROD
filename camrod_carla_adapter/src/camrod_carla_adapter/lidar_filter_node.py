"""ROS boundary for deterministic filtering of the real CARLA LiDAR cloud."""

import copy
import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

from camrod_carla_adapter.lidar_filter import (
    LidarFilterConfig,
    nonground_mask_with_diagnostics,
)


def pointcloud_xyz_and_records(message: PointCloud2):
    """Expose XYZ and complete point records without changing CARLA fields."""
    if message.width <= 0 or message.height <= 0 or message.point_step <= 0:
        raise ValueError("PointCloud2 geometry is empty")
    fields = {field.name: field for field in message.fields}
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.datatype != PointField.FLOAT32 or field.count != 1:
            raise ValueError(f"PointCloud2 field {name!r} must be one FLOAT32")
        if field.offset + 4 > message.point_step:
            raise ValueError(f"PointCloud2 field {name!r} exceeds point_step")
    minimum_size = (
        (message.height - 1) * message.row_step
        + message.width * message.point_step
    )
    if len(message.data) < minimum_size:
        raise ValueError(
            f"PointCloud2 payload has {len(message.data)} bytes, "
            f"expected at least {minimum_size}"
        )

    endian = ">f4" if message.is_bigendian else "<f4"
    coordinates = []
    for name in ("x", "y", "z"):
        field = fields[name]
        view = np.ndarray(
            shape=(message.height, message.width),
            dtype=np.dtype(endian),
            buffer=message.data,
            offset=field.offset,
            strides=(message.row_step, message.point_step),
        )
        coordinates.append(np.asarray(view, dtype=np.float64).reshape(-1))

    byte_view = np.ndarray(
        shape=(message.height, message.width, message.point_step),
        dtype=np.uint8,
        buffer=message.data,
        strides=(message.row_step, message.point_step, 1),
    )
    records = np.empty(
        (message.height * message.width, message.point_step), dtype=np.uint8
    )
    cursor = 0
    for row in range(message.height):
        next_cursor = cursor + message.width
        records[cursor:next_cursor] = byte_view[row]
        cursor = next_cursor
    return np.column_stack(coordinates), records


def selected_pointcloud(message: PointCloud2, records: np.ndarray) -> PointCloud2:
    """Build an unorganized PointCloud2 while retaining the source field schema."""
    output = PointCloud2()
    output.header = copy.deepcopy(message.header)
    output.height = 1
    output.width = int(records.shape[0])
    output.fields = copy.deepcopy(message.fields)
    output.is_bigendian = message.is_bigendian
    output.point_step = message.point_step
    output.row_step = output.width * output.point_step
    output.data = np.ascontiguousarray(records).tobytes()
    output.is_dense = True
    return output


class CarlaLidarFilterNode(Node):
    """Convert a real CARLA raw cloud into CAMROD's nonground contract."""

    def __init__(self):
        super().__init__("carla_lidar_filter")
        self.input_topic = self.declare_parameter(
            "input_topic", "/sensing/lidar/vanjee/points_raw"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/sensing/lidar/points_filtered"
        ).value
        self.expected_frame_id = self.declare_parameter(
            "expected_frame_id", "lidar_link"
        ).value
        self.config = LidarFilterConfig(
            roi_x_min_m=float(self.declare_parameter("roi_x_min_m", 0.0).value),
            roi_x_max_m=float(self.declare_parameter("roi_x_max_m", 5.0).value),
            roi_y_min_m=float(self.declare_parameter("roi_y_min_m", -3.0).value),
            roi_y_max_m=float(self.declare_parameter("roi_y_max_m", 3.0).value),
            roi_z_min_m=float(self.declare_parameter("roi_z_min_m", -1.2).value),
            roi_z_max_m=float(self.declare_parameter("roi_z_max_m", 1.5).value),
            expected_ground_z_m=float(
                self.declare_parameter("expected_ground_z_m", -0.59538).value
            ),
            ground_seed_z_tolerance_m=float(
                self.declare_parameter("ground_seed_z_tolerance_m", 0.8).value
            ),
            ground_grid_size_m=float(
                self.declare_parameter("ground_grid_size_m", 0.40).value
            ),
            ground_inlier_threshold_m=float(
                self.declare_parameter("ground_inlier_threshold_m", 0.045).value
            ),
            minimum_obstacle_height_m=float(
                self.declare_parameter("minimum_obstacle_height_m", 0.08).value
            ),
            maximum_ground_slope_deg=float(
                self.declare_parameter("maximum_ground_slope_deg", 25.0).value
            ),
            minimum_ground_seed_cells=int(
                self.declare_parameter("minimum_ground_seed_cells", 12).value
            ),
            minimum_ground_inliers=int(
                self.declare_parameter("minimum_ground_inliers", 24).value
            ),
            robust_fit_iterations=int(
                self.declare_parameter("robust_fit_iterations", 4).value
            ),
            self_return_mask_enabled=bool(
                self.declare_parameter(
                    "self_return_mask_enabled", False
                ).value
            ),
            self_return_x_min_m=float(
                self.declare_parameter("self_return_x_min_m", 0.40).value
            ),
            self_return_x_max_m=float(
                self.declare_parameter("self_return_x_max_m", 1.31).value
            ),
            self_return_abs_y_min_m=float(
                self.declare_parameter(
                    "self_return_abs_y_min_m", 0.59
                ).value
            ),
            self_return_abs_y_max_m=float(
                self.declare_parameter(
                    "self_return_abs_y_max_m", 0.95
                ).value
            ),
            self_return_z_min_m=float(
                self.declare_parameter("self_return_z_min_m", -0.57).value
            ),
            self_return_z_max_m=float(
                self.declare_parameter("self_return_z_max_m", 0.18).value
            ),
        )
        self._validate_config()
        qos_depth = int(self.declare_parameter("qos_depth", 2).value)
        if qos_depth <= 0:
            raise ValueError("qos_depth must be positive")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos
        )
        self._subscription = self.create_subscription(
            PointCloud2, self.input_topic, self._on_cloud, qos
        )
        self._status_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self._last_receive_monotonic = None
        self._last_input_count = 0
        self._last_output_count = 0
        self._last_self_return_removed_count = 0
        self._last_plane = (0.0, 0.0, self.config.expected_ground_z_m)
        self._last_fallback = False
        self._last_reason = "waiting for CARLA LiDAR"
        self._timer = self.create_timer(0.5, self._publish_status)
        self.get_logger().info(
            "CARLA LiDAR filter ready: %s -> %s, frame=%s, "
            "ROI x[%.2f,%.2f] y[%.2f,%.2f], obstacle_height>=%.3f m, "
            "self_return_mask=%s"
            % (
                self.input_topic,
                self.output_topic,
                self.expected_frame_id,
                self.config.roi_x_min_m,
                self.config.roi_x_max_m,
                self.config.roi_y_min_m,
                self.config.roi_y_max_m,
                self.config.minimum_obstacle_height_m,
                "enabled" if self.config.self_return_mask_enabled else "disabled",
            )
        )

    def _validate_config(self):
        cfg = self.config
        values = tuple(vars(cfg).values())
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("all CARLA LiDAR filter parameters must be finite")
        if not cfg.roi_x_min_m < cfg.roi_x_max_m:
            raise ValueError("roi_x_min_m must be below roi_x_max_m")
        if not cfg.roi_y_min_m < cfg.roi_y_max_m:
            raise ValueError("roi_y_min_m must be below roi_y_max_m")
        if not cfg.roi_z_min_m < cfg.roi_z_max_m:
            raise ValueError("roi_z_min_m must be below roi_z_max_m")
        if cfg.ground_grid_size_m <= 0.0:
            raise ValueError("ground_grid_size_m must be positive")
        if cfg.ground_inlier_threshold_m <= 0.0:
            raise ValueError("ground_inlier_threshold_m must be positive")
        if cfg.minimum_obstacle_height_m <= cfg.ground_inlier_threshold_m:
            raise ValueError(
                "minimum_obstacle_height_m must exceed the ground inlier band"
            )
        if cfg.minimum_ground_seed_cells < 3:
            raise ValueError("minimum_ground_seed_cells must be at least 3")
        if not cfg.self_return_x_min_m < cfg.self_return_x_max_m:
            raise ValueError(
                "self_return_x_min_m must be below self_return_x_max_m"
            )
        if not 0.0 <= cfg.self_return_abs_y_min_m < (
            cfg.self_return_abs_y_max_m
        ):
            raise ValueError(
                "self_return_abs_y bounds must be ordered and nonnegative"
            )
        if not cfg.self_return_z_min_m < cfg.self_return_z_max_m:
            raise ValueError(
                "self_return_z_min_m must be below self_return_z_max_m"
            )

    def _on_cloud(self, message):
        try:
            if message.header.frame_id != self.expected_frame_id:
                raise ValueError(
                    f"expected frame {self.expected_frame_id!r}, got "
                    f"{message.header.frame_id!r}"
                )
            xyz, records = pointcloud_xyz_and_records(message)
            keep, plane, self_return_removed_count = (
                nonground_mask_with_diagnostics(xyz, self.config)
            )
            output = selected_pointcloud(message, records[keep])
        except (TypeError, ValueError, IndexError) as error:
            self._last_reason = str(error)
            self._last_receive_monotonic = time.monotonic()
            self.get_logger().error(f"CARLA LiDAR filtering failed: {error}")
            return
        self._publisher.publish(output)
        self._last_receive_monotonic = time.monotonic()
        self._last_input_count = int(xyz.shape[0])
        self._last_output_count = int(output.width)
        self._last_self_return_removed_count = self_return_removed_count
        self._last_plane = plane.coefficients
        self._last_fallback = plane.used_fallback
        self._last_reason = plane.reason

    def _publish_status(self):
        now_monotonic = time.monotonic()
        age = (
            math.inf
            if self._last_receive_monotonic is None
            else now_monotonic - self._last_receive_monotonic
        )
        status = DiagnosticStatus()
        status.name = "carla_lidar_filter: /sensing/lidar/points_filtered"
        status.hardware_id = "carla.sensor.lidar.ray_cast"
        if not math.isfinite(age) or age > 2.0:
            status.level = DiagnosticStatus.ERROR
            status.message = self._last_reason or "CARLA LiDAR input is stale"
        elif self._last_fallback:
            status.level = DiagnosticStatus.WARN
            status.message = "measured ground fit unavailable; fixed sensor height used"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "real CARLA nonground cloud"
        a, b, c = self._last_plane
        status.values = [
            KeyValue(key="input_points", value=str(self._last_input_count)),
            KeyValue(key="nonground_points", value=str(self._last_output_count)),
            KeyValue(
                key="self_return_points_removed",
                value=str(self._last_self_return_removed_count),
            ),
            KeyValue(
                key="self_return_mask_enabled",
                value=str(self.config.self_return_mask_enabled).lower(),
            ),
            KeyValue(key="plane_a", value=f"{a:.6f}"),
            KeyValue(key="plane_b", value=f"{b:.6f}"),
            KeyValue(key="plane_c", value=f"{c:.6f}"),
            KeyValue(key="fallback", value=str(self._last_fallback).lower()),
            KeyValue(key="detail", value=self._last_reason),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._status_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CarlaLidarFilterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
