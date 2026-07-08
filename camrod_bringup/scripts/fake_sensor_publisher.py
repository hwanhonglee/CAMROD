#!/usr/bin/env python3
# Generate fake GNSS/IMU/Wheel/DR/Obstacle topics from lanelet centerline.

import math
import time
import xml.etree.ElementTree as ET
from bisect import bisect_left

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sensor_msgs.msg import Imu, PointCloud2, PointField, Range
from sensor_msgs_py import point_cloud2


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


# Implements `deg2rad` behavior.
def deg2rad(deg):
    return deg * math.pi / 180.0


# Implements `llh_to_ecef` behavior.
def llh_to_ecef(lat_rad, lon_rad, alt):
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt) * cos_lat * cos_lon
    y = (n + alt) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + alt) * sin_lat
    return (x, y, z)


# Implements `ecef_to_enu` behavior.
def ecef_to_enu(ref_ecef, cur_ecef, lat_ref, lon_ref):
    sin_lat = math.sin(lat_ref)
    cos_lat = math.cos(lat_ref)
    sin_lon = math.sin(lon_ref)
    cos_lon = math.cos(lon_ref)
    dx = cur_ecef[0] - ref_ecef[0]
    dy = cur_ecef[1] - ref_ecef[1]
    dz = cur_ecef[2] - ref_ecef[2]
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return (east, north, up)


# Implements `yaw_to_quat` behavior.
def yaw_to_quat(yaw):
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


# Implements `quat_to_yaw` behavior.
def quat_to_yaw(quat):
    # Standard yaw extraction from quaternion.
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)

# Implements `normalize_angle` behavior.
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class FakeSensorPublisher(Node):
    # Implements `__init__` behavior.
    def __init__(self):
        super().__init__("fake_sensor_publisher")

        self.map_path = self.declare_parameter("map_path", "").value
        self.origin_lat = self.declare_parameter("origin_lat", 0.0).value
        self.origin_lon = self.declare_parameter("origin_lon", 0.0).value
        self.origin_alt = self.declare_parameter("origin_alt", 0.0).value
        self.lanelet_id = int(self.declare_parameter("lanelet_id", -1).value)
        self.speed_mps = self.declare_parameter("speed_mps", 1.4).value
        # Debug option:
        # - true  : keep fake sensor pose fixed on the path start anchor
        # - false : move along centerline path with `speed_mps`
        self.freeze_motion = bool(self.declare_parameter("freeze_motion", False).value)
        # HH_260618: Keep simulation input rate aligned with controller needs
        # while reducing Python CPU load on development PCs.
        self.publish_rate_hz = self.declare_parameter("publish_rate_hz", 10.0).value
        self.loop = self.declare_parameter("loop", True).value
        # HH_260526: Replace use_cmd_vel_for_motion with explicit motion source mode.
        # motion_source options: cmd_vel | constant_speed
        self.motion_source = str(self.declare_parameter("motion_source", "cmd_vel").value).strip().lower()
        if self.motion_source not in {"cmd_vel", "constant_speed"}:
            self.get_logger().warn(
                f"Invalid motion_source='{self.motion_source}', fallback to 'cmd_vel'"
            )
            self.motion_source = "cmd_vel"
        self.motion_uses_cmd_vel = self.motion_source == "cmd_vel"
        self.cmd_vel_motion_topic = str(
            self.declare_parameter("cmd_vel_motion_topic", "/platform/cmd_vel").value
        )
        self.cmd_vel_timeout_s = float(
            self.declare_parameter("cmd_vel_timeout_s", 0.5).value
        )
        # HH_260618: Nav2 controllers can interleave zero and non-zero Twist
        # samples during short-horizon replanning. The sim integrator runs at a
        # lower rate, so sampling only the latest zero can freeze the robot even
        # while valid drive commands are present. Hold the last non-zero command
        # briefly to approximate drivetrain command persistence.
        self.cmd_vel_nonzero_hold_s = float(
            self.declare_parameter("cmd_vel_nonzero_hold_s", 0.20).value
        )
        self.cmd_vel_deadband = float(
            self.declare_parameter("cmd_vel_deadband", 1.0e-4).value
        )
        self.max_cmd_speed_mps = float(
            self.declare_parameter("max_cmd_speed_mps", 2.5).value
        )
        # Accept RViz "2D Pose Estimate" reset input.
        self.initialpose_topic = str(
            self.declare_parameter("initialpose_topic", "/localization/initialpose").value
        )
        # HH_260528: Also listen to RViz default topic unless disabled.
        self.enable_initialpose_fallback_topic = bool(
            self.declare_parameter("enable_initialpose_fallback_topic", True).value
        )
        self.initialpose_topic_fallback = str(
            self.declare_parameter("initialpose_topic_fallback", "/initialpose").value
        )
        # HH_260526: Replace use_free_nav_sim with explicit navigation model mode.
        # simulation_nav_mode options: centerline | free_nav
        self.simulation_nav_mode = str(
            self.declare_parameter("simulation_nav_mode", "centerline").value
        ).strip().lower()
        if self.simulation_nav_mode not in {"centerline", "free_nav"}:
            self.get_logger().warn(
                f"Invalid simulation_nav_mode='{self.simulation_nav_mode}', fallback to 'centerline'"
            )
            self.simulation_nav_mode = "centerline"
        self.free_nav_mode_enabled = self.simulation_nav_mode == "free_nav"
        # 2026-02-05 14:37: Skip crosswalk lanelets to prevent centerline jumps.
        self.exclude_crosswalk = self.declare_parameter("exclude_crosswalk", True).value
        # HH_260522: Use a single canonical start anchor switch.
        # - start_from_pose=true : anchor path traversal from nearest point to (start_x, start_y)
        # - false                : start from beginning of stitched centerline path
        self.start_from_pose = bool(self.declare_parameter("start_from_pose", False).value)
        self.start_x = self.declare_parameter("start_x", 0.0).value
        self.start_y = self.declare_parameter("start_y", 0.0).value
        # HH_260526: Replace use_all_centerlines with explicit centerline scope.
        # centerline_scope options: all | selected_lanelet
        self.centerline_scope = str(
            self.declare_parameter("centerline_scope", "all").value
        ).strip().lower()
        if self.centerline_scope not in {"all", "selected_lanelet"}:
            self.get_logger().warn(
                f"Invalid centerline_scope='{self.centerline_scope}', fallback to 'all'"
            )
            self.centerline_scope = "all"
        self.centerline_scope_all = self.centerline_scope == "all"
        self.centerline_connect_max_gap = float(
            self.declare_parameter("centerline_connect_max_gap", 5.0).value
        )
        # HH_260618: Keep full-map sim startup logs readable; detailed stitching
        # gaps are available only when explicitly debugging map connectivity.
        self.log_centerline_stitch_details = bool(
            self.declare_parameter("log_centerline_stitch_details", False).value
        )
        # 2026-02-02: Optionally close a near-loop path for continuous laps.
        self.close_loop = self.declare_parameter("close_loop", True).value
        self.close_loop_max_gap = float(
            self.declare_parameter("close_loop_max_gap", 3.0).value
        )
        # 2026-03-03: Keep the robot stationary for a short warm-up so the
        # initial GNSS/IMU fusion settles before wheel motion begins.
        self.startup_hold_s = float(
            self.declare_parameter("startup_hold_s", 4.0).value
        )
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.base_frame_id = self.declare_parameter("base_frame_id", "robot_base_link").value
        # Default keeps synthetic obstacle outside stop corridor.
        # Lower at runtime when validating cost-stop behavior.
        self.obstacle_offset = self.declare_parameter("obstacle_offset", 12.0).value
        self.obstacle_height = self.declare_parameter("obstacle_height", 0.5).value
        # Synthetic obstacle placement direction relative to vehicle heading.
        # Supported values: front | left | right | rear
        self.obstacle_direction = self._normalize_obstacle_direction(
            str(self.declare_parameter("obstacle_direction", "front").value)
        )
        # Additional lateral shift in vehicle-left axis (meters).
        self.obstacle_lateral_offset = float(
            self.declare_parameter("obstacle_lateral_offset", 0.0).value
        )
        # HH_260630: Keep the synthetic obstacle compact around the requested
        # direction center. A wide perpendicular spread can make side/rear
        # tests appear in the wrong corridor.
        self.fake_obstacle_cluster_radius_m = float(
            self.declare_parameter("fake_obstacle_cluster_radius_m", 0.12).value
        )
        # Publish the same synthetic obstacle cloud to both
        # perception obstacle stream and filtered-lidar stream so cost-grid
        # source can be switched without breaking sim.
        self.obstacle_cloud_topic = str(
            self.declare_parameter("obstacle_cloud_topic", "/perception/obstacles").value
        )
        self.lidar_filtered_topic = str(
            self.declare_parameter("lidar_filtered_topic", "/sensing/lidar/points_filtered").value
        )
        # HH_260630: Split fake LiDAR cloud and fake radar range publishing so
        # sim safety tests can validate each obstacle source independently.
        self.publish_fake_lidar_obstacle_cloud = bool(
            self.declare_parameter("publish_fake_lidar_obstacle_cloud", True).value
        )
        self.publish_fake_radar_ranges = bool(
            self.declare_parameter("publish_fake_radar_ranges", True).value
        )
        self.fake_radar_min_range_m = float(
            self.declare_parameter("fake_radar_min_range_m", 0.02).value
        )
        self.fake_radar_field_of_view_rad = float(
            self.declare_parameter("fake_radar_field_of_view_rad", 0.26).value
        )
        self.fake_radar_topics = list(
            self.declare_parameter(
                "fake_radar_topics",
                [
                    "/sensing/radar/front1/range",
                    "/sensing/radar/front2/range",
                    "/sensing/radar/left1/range",
                    "/sensing/radar/left2/range",
                    "/sensing/radar/right1/range",
                    "/sensing/radar/right2/range",
                    "/sensing/radar/rear/range",
                ],
            ).value
        )
        self.fake_radar_frame_ids = list(
            self.declare_parameter(
                "fake_radar_frame_ids",
                [
                    "radar_front1_link",
                    "radar_front2_link",
                    "radar_left1_link",
                    "radar_left2_link",
                    "radar_right1_link",
                    "radar_right2_link",
                    "radar_rear_link",
                ],
            ).value
        )
        self.fake_radar_max_ranges_m = [
            float(v)
            for v in self.declare_parameter(
                "fake_radar_max_ranges_m",
                [1.50, 1.50, 0.80, 0.80, 0.80, 0.80, 0.50],
            ).value
        ]
        # HH_260617: In sim, hardware IMU drivers are disabled by bringup. Publish
        # the velocity-converter output directly so diagnostics/planning can focus
        # on planning/control behavior instead of waiting for a real converter node.
        self.publish_velocity_converter_output = bool(
            self.declare_parameter("publish_velocity_converter_output", True).value
        )
        self.velocity_converter_output_topic = str(
            self.declare_parameter(
                "velocity_converter_output_topic",
                "/sensing/platform_velocity_converter/twist_with_covariance",
            ).value
        )
        # HH_260617: Publish a free local cost grid in sim as a deterministic
        # sensor-health dummy. The real lidar_cost_grid_node may also publish when
        # TF is ready; this fallback prevents diagnostics from failing on TF startup.
        self.publish_dummy_lidar_cost_grid = bool(
            self.declare_parameter("publish_dummy_lidar_cost_grid", True).value
        )
        self.dummy_lidar_cost_grid_topic = str(
            self.declare_parameter("dummy_lidar_cost_grid_topic", "/sensing/lidar/near_cost_grid").value
        )
        self.dummy_lidar_cost_grid_width = int(
            self.declare_parameter("dummy_lidar_cost_grid_width", 120).value
        )
        self.dummy_lidar_cost_grid_height = int(
            self.declare_parameter("dummy_lidar_cost_grid_height", 120).value
        )
        self.dummy_lidar_cost_grid_resolution = float(
            self.declare_parameter("dummy_lidar_cost_grid_resolution", 0.10).value
        )
        # HH_260618: Publish the sim dummy cost grid at a lower rate than pose/IMU.
        # The grid is a static health/free-space fallback; publishing it every
        # fake sensor tick only adds serialization and merge load.
        self.dummy_lidar_cost_grid_publish_rate_hz = float(
            self.declare_parameter("dummy_lidar_cost_grid_publish_rate_hz", 3.0).value
        )
        # In sim mode, also publish nav_msgs/Odometry to the wheel bridge
        # input topic so localization wheel pipeline can run without real /rmp401/odom.
        self.wheel_bridge_input_topic = str(
            self.declare_parameter("wheel_bridge_input_topic", "/rmp401/odom").value
        )
        # VIO stack is disabled in this workspace. Publish localization
        # fallback odometry as DR reference instead of a VIO-only stream.
        self.dr_odometry_topic = str(
            self.declare_parameter("dr_odometry_topic", "/localization/fallback/odometry").value
        )
        # Optional helper: align localization pose yaw to nearest active path tangent.
        # Position is kept from incoming /localization/pose, only orientation(yaw) is replaced.
        self.localization_pose_yaw_align_enable = bool(
            self.declare_parameter("localization_pose_yaw_align_enable", True).value
        )
        self.localization_pose_input_topic = str(
            self.declare_parameter("localization_pose_input_topic", "/localization/pose").value
        )
        self.localization_pose_output_topic = str(
            self.declare_parameter(
                "localization_pose_output_topic", "/localization/pose_yaw_aligned"
            ).value
        )
        self.localization_pose_yaw_path_topic = str(
            self.declare_parameter(
                "localization_pose_yaw_path_topic", "/planning/global_path"
            ).value
        )
        self.localization_pose_yaw_min_segment_length = float(
            self.declare_parameter("localization_pose_yaw_min_segment_length", 0.10).value
        )
        # Optional stabilization for /localization/pose_yaw_aligned.
        # Keeps yaw continuous when nearest path segments change.
        self.localization_pose_yaw_smoothing_alpha = float(
            self.declare_parameter("localization_pose_yaw_smoothing_alpha", 1.0).value
        )
        self.localization_pose_yaw_max_rate_deg_s = float(
            self.declare_parameter("localization_pose_yaw_max_rate_deg_s", 180.0).value
        )
        # Optional debug mode: lock aligned pose position to one fixed point.
        # This is useful to isolate yaw-only behavior while keeping localization pipeline alive.
        self.localization_pose_lock_position = bool(
            self.declare_parameter("localization_pose_lock_position", False).value
        )
        self.localization_pose_lock_on_first_msg = bool(
            self.declare_parameter("localization_pose_lock_on_first_msg", True).value
        )
        self.localization_pose_lock_x = float(
            self.declare_parameter("localization_pose_lock_x", 0.0).value
        )
        self.localization_pose_lock_y = float(
            self.declare_parameter("localization_pose_lock_y", 0.0).value
        )
        self.localization_pose_lock_z = float(
            self.declare_parameter("localization_pose_lock_z", 0.0).value
        )
        self._yaw_path_points = []
        self._yaw_path_ready = False
        self._last_aligned_yaw = None
        self._last_aligned_yaw_time = None
        self._lock_position_initialized = False
        # 2026-02-02 11:05: Resample centerline from bounds when explicit centerline is missing.
        self.centerline_step = self.declare_parameter("centerline_step", 1.0).value
        # GNSS failure simulation for DR fallback testing.
        # When gnss_failure_after_s > 0, NavSatFix publishing stops at that elapsed time,
        # triggering DR_ONLY mode in localization_monitor. Resumes at gnss_recovery_after_s.
        # Both default to -1 (disabled) so existing runs are unaffected.
        self.gnss_failure_after_s = float(
            self.declare_parameter("gnss_failure_after_s", -1.0).value
        )

        self.gnss_recovery_after_s = float(
            self.declare_parameter("gnss_recovery_after_s", -1.0).value
        )
        self._gnss_active = True
        # Keep selected simulation controls adjustable at runtime via ros2 param set.
        self.add_on_set_parameters_callback(self._on_set_parameters)

        if not self.map_path:
            # 2026-01-27 17:45: Remove HH tags from runtime logs.
            self.get_logger().fatal("map_path is required.")
            raise RuntimeError("map_path is empty")

        self._path = self._load_centerline_path()
        if len(self._path) < 2:
            self.get_logger().fatal("failed to load lanelet centerline path.")
            raise RuntimeError("centerline path missing")
        # 2026-02-02: Close loop if endpoints are close enough to avoid a hard jump.
        if self.close_loop and len(self._path) > 2:
            dx = self._path[0][0] - self._path[-1][0]
            dy = self._path[0][1] - self._path[-1][1]
            gap = math.hypot(dx, dy)
            if gap <= self.close_loop_max_gap:
                self._path.append(self._path[0])
            else:
                self.get_logger().warn(
                    f"Centerline loop gap {gap:.2f}m exceeds close_loop_max_gap; "
                    "path will wrap with a jump. Consider a loop lanelet_id."
                )

        self._distances = self._build_cumulative_distances(self._path)
        self._total_length = self._distances[-1]
        self._start_distance = 0.0
        if self.start_from_pose:
            self._start_distance = self._nearest_distance_on_path(self.start_x, self.start_y)
        self._t0 = time.time()
        self._last_yaw = None
        self._last_yaw_time = None
        self._was_holding = True
        self._motion_distance = 0.0
        self._last_timer_time = time.time()
        self._cmd_linear_x = 0.0
        self._cmd_linear_y = 0.0
        self._cmd_angular_z = 0.0
        self._last_cmd_time = None
        self._last_nonzero_cmd_linear_x = 0.0
        self._last_nonzero_cmd_linear_y = 0.0
        self._last_nonzero_cmd_angular_z = 0.0
        self._last_nonzero_cmd_time = None
        self._last_dummy_lidar_cost_grid_pub_sec = 0.0
        # HH_260428: Free nav state — initialized from the path start point so the
        # robot begins at the same location regardless of mode.
        self._free_nav_x = 0.0
        self._free_nav_y = 0.0
        self._free_nav_yaw = 0.0
        if self.free_nav_mode_enabled:
            (x0, y0, _, _, _), yaw0 = self._sample_path(0.0)
            self._free_nav_x = x0
            self._free_nav_y = y0
            self._free_nav_yaw = yaw0

        # Publish fake sensors with module-prefixed topics.
        self.pub_navsat = self.create_publisher(NavSatFix, "/sensing/gnss/ublox_gps_node/fix", 10)
        self.pub_imu = self.create_publisher(Imu, "/sensing/imu/data", 10)
        # Keep fake wheel output aligned with unified wheel_odometry topic.
        self.pub_wheel = self.create_publisher(Odometry, "/platform/status/wheel_odometry", 10)
        self.pub_wheel_bridge_in = self.create_publisher(Odometry, self.wheel_bridge_input_topic, 10)
        # Use transient-local QoS for fallback odometry so late-joining
        # consumers (e.g. pose selector) can subscribe without durability mismatch.
        odom_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_dr_odom = self.create_publisher(Odometry, self.dr_odometry_topic, odom_qos)
        self.pub_obstacles = self.create_publisher(PointCloud2, self.obstacle_cloud_topic, 10)
        self.pub_lidar_filtered = self.create_publisher(
            PointCloud2, self.lidar_filtered_topic, 10
        )
        self.pub_fake_radar_ranges = []
        if self.publish_fake_radar_ranges:
            for topic in self.fake_radar_topics:
                self.pub_fake_radar_ranges.append(self.create_publisher(Range, topic, 10))
        self.pub_velocity_converter_output = None
        if self.publish_velocity_converter_output:
            self.pub_velocity_converter_output = self.create_publisher(
                TwistWithCovarianceStamped, self.velocity_converter_output_topic, 10
            )
        self.pub_dummy_lidar_cost_grid = None
        if self.publish_dummy_lidar_cost_grid:
            self.pub_dummy_lidar_cost_grid = self.create_publisher(
                OccupancyGrid, self.dummy_lidar_cost_grid_topic, 10
            )
        self.sub_cmd_vel = None
        if self.motion_uses_cmd_vel:
            self.sub_cmd_vel = self.create_subscription(
                Twist, self.cmd_vel_motion_topic, self._on_cmd_vel, 10
            )
        self.sub_initialpose = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            self._on_initialpose,
            10,
        )
        self.sub_initialpose_fallback = None
        if (
            self.enable_initialpose_fallback_topic
            and self.initialpose_topic_fallback
            and self.initialpose_topic_fallback != self.initialpose_topic
        ):
            self.sub_initialpose_fallback = self.create_subscription(
                PoseWithCovarianceStamped,
                self.initialpose_topic_fallback,
                self._on_initialpose,
                10,
            )

        # Optional pose-yaw align bridge for fake/sim mode.
        self.pub_localization_pose_yaw_aligned = None
        self.sub_localization_pose_input = None
        self.sub_localization_pose_path = None
        if self.localization_pose_yaw_align_enable:
            if self.localization_pose_input_topic == self.localization_pose_output_topic:
                self.get_logger().error(
                    "localization pose yaw align disabled: input_topic equals output_topic "
                    "(would create feedback loop)."
                )
            else:
                self.pub_localization_pose_yaw_aligned = self.create_publisher(
                    PoseStamped, self.localization_pose_output_topic, 10
                )
                self.sub_localization_pose_input = self.create_subscription(
                    PoseStamped,
                    self.localization_pose_input_topic,
                    self._on_localization_pose_input,
                    10,
                )
                self.sub_localization_pose_path = self.create_subscription(
                    Path,
                    self.localization_pose_yaw_path_topic,
                    self._on_localization_pose_yaw_path,
                    10,
                )
                self.get_logger().info(
                    "localization yaw align enabled: "
                    f"input={self.localization_pose_input_topic} "
                    f"output={self.localization_pose_output_topic} "
                    f"path={self.localization_pose_yaw_path_topic}"
                )
                if self.localization_pose_lock_position:
                    self.get_logger().warn(
                        "localization pose lock-position is enabled "
                        "(debug mode; aligned pose position is fixed)."
                    )

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().debug(
            f"Fake sensors ready. lanelet_id={self.lanelet_id} speed={self.speed_mps} m/s"
        )

    # Handles the `_on_cmd_vel` callback.
    def _on_cmd_vel(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        vmax = max(0.0, float(self.max_cmd_speed_mps))
        if vmax > 0.0:
            vx = max(-vmax, min(vmax, vx))
            vy = max(-vmax, min(vmax, vy))
        self._cmd_linear_x = vx
        # HH_260618: Preserve lateral crab commands in sim. Site maneuver uses
        # Twist.linear.y after Nav2 reaches the lanelet-snap pose; ignoring this
        # axis made the robot stay on the road and time out before site entry.
        self._cmd_linear_y = vy
        # HH_260428: Track angular.z for free nav mode (unicycle steering).
        self._cmd_angular_z = float(msg.angular.z)
        self._last_cmd_time = time.time()
        if (
            abs(self._cmd_linear_x) > self.cmd_vel_deadband
            or abs(self._cmd_linear_y) > self.cmd_vel_deadband
            or abs(self._cmd_angular_z) > self.cmd_vel_deadband
        ):
            self._last_nonzero_cmd_linear_x = self._cmd_linear_x
            self._last_nonzero_cmd_linear_y = self._cmd_linear_y
            self._last_nonzero_cmd_angular_z = self._cmd_angular_z
            self._last_nonzero_cmd_time = self._last_cmd_time

    # Handles external initial-pose reset requests (e.g., RViz `P` tool).
    def _on_initialpose(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = normalize_angle(quat_to_yaw(pose.orientation))

        now_sec = time.time()
        if self.free_nav_mode_enabled:
            self._free_nav_x = x
            self._free_nav_y = y
            self._free_nav_yaw = yaw
            mode_label = "free_nav"
        else:
            # Centerline mode cannot leave lanelet path. Snap to nearest point.
            self._start_distance = self._nearest_distance_on_path(x, y)
            self._motion_distance = 0.0
            mode_label = "centerline_snap"

        # Reset derived motion states to avoid a one-cycle yaw-rate spike.
        self._last_yaw = yaw
        self._last_yaw_time = now_sec
        self._last_timer_time = now_sec
        self._cmd_linear_x = 0.0
        self._cmd_linear_y = 0.0
        self._cmd_angular_z = 0.0
        self._last_cmd_time = None
        self._last_nonzero_cmd_linear_x = 0.0
        self._last_nonzero_cmd_linear_y = 0.0
        self._last_nonzero_cmd_angular_z = 0.0
        self._last_nonzero_cmd_time = None

        self.get_logger().info(
            f"initialpose applied ({mode_label}): x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad"
        )

    # Implements `_load_centerline_path` behavior.
    def _load_centerline_path(self):
        # Parse Lanelet2 OSM and extract centerline ways.
        tree = ET.parse(self.map_path)
        root = tree.getroot()

        nodes = {}
        for node in root.findall("node"):
            nid = int(node.attrib["id"])
            lat = float(node.attrib["lat"])
            lon = float(node.attrib["lon"])
            nodes[nid] = (lat, lon)

        ways = {}
        for way in root.findall("way"):
            wid = int(way.attrib["id"])
            nds = [int(nd.attrib["ref"]) for nd in way.findall("nd")]
            ways[wid] = nds

        lanelets = []
        for rel in root.findall("relation"):
            tags = {t.attrib["k"]: t.attrib["v"] for t in rel.findall("tag")}
            if tags.get("type") != "lanelet":
                continue
            if (self.exclude_crosswalk and self.lanelet_id < 0 and
                    tags.get("subtype") == "crosswalk"):
                continue
            rid = int(rel.attrib["id"])
            if self.lanelet_id >= 0 and rid != int(self.lanelet_id):
                continue
            lanelets.append((rid, rel))

        segments = []
        for _, rel in lanelets:
            centerline_way = None
            left_way = None
            right_way = None
            for member in rel.findall("member"):
                role = member.attrib.get("role")
                if role == "centerline":
                    centerline_way = int(member.attrib["ref"])
                elif role == "left":
                    left_way = int(member.attrib["ref"])
                elif role == "right":
                    right_way = int(member.attrib["ref"])

            if centerline_way is not None and centerline_way in ways:
                path = self._build_path_from_way(ways[centerline_way], nodes)
            elif left_way is not None and right_way is not None and left_way in ways and right_way in ways:
                path = self._build_path_from_bounds(ways[left_way], ways[right_way], nodes)
            else:
                continue

            if len(path) >= 2:
                segments.append(path)

        if not segments:
            self.get_logger().error(
                "No centerline or bound-based paths found for lanelets in map."
            )
            return []

        # Lanelet-specific request: return the single lanelet path.
        if self.lanelet_id >= 0 or not self.centerline_scope_all:
            return segments[0]

        # 2026-02-02: Stitch all lanelet centerlines into one continuous loop.
        return self._stitch_centerlines(segments)

    # Implements `_stitch_centerlines` behavior.
    def _stitch_centerlines(self, segments):
        if not segments:
            return []

        # Implements `seg_length` behavior.
        def seg_length(seg):
            total = 0.0
            for i in range(1, len(seg)):
                dx = seg[i][0] - seg[i - 1][0]
                dy = seg[i][1] - seg[i - 1][1]
                total += math.hypot(dx, dy)
            return total

        # Implements `endpoint_neighbors` behavior.
        def endpoint_neighbors(idx, point, segs, max_gap):
            count = 0
            for j, seg in enumerate(segs):
                if j == idx:
                    continue
                if (math.hypot(point[0] - seg[0][0], point[1] - seg[0][1]) <= max_gap or
                        math.hypot(point[0] - seg[-1][0], point[1] - seg[-1][1]) <= max_gap):
                    count += 1
            return count

        # Implements `neighbor_count` behavior.
        def neighbor_count(point, segs, max_gap):
            count = 0
            for seg in segs:
                if (math.hypot(point[0] - seg[0][0], point[1] - seg[0][1]) <= max_gap or
                        math.hypot(point[0] - seg[-1][0], point[1] - seg[-1][1]) <= max_gap):
                    count += 1
            return count

        segments = list(segments)
        max_gap = self.centerline_connect_max_gap
        start_idx = None
        best_score = float("inf")
        best_length = -1.0
        for i, seg in enumerate(segments):
            n_start = endpoint_neighbors(i, seg[0], segments, max_gap)
            n_end = endpoint_neighbors(i, seg[-1], segments, max_gap)
            score = n_start + n_end
            length = seg_length(seg)
            if score < best_score or (score == best_score and length > best_length):
                best_score = score
                best_length = length
                start_idx = i

        path = segments.pop(start_idx)
        # Prefer starting from the endpoint with fewer neighbors (open chain start).
        if neighbor_count(path[0], segments, max_gap) > neighbor_count(path[-1], segments, max_gap):
            path = list(reversed(path))
        current_end = path[-1]
        large_gap_count = 0
        max_stitch_gap = 0.0

        while segments:
            candidates = []
            fallback = None
            for i, seg in enumerate(segments):
                d_start = math.hypot(current_end[0] - seg[0][0], current_end[1] - seg[0][1])
                d_end = math.hypot(current_end[0] - seg[-1][0], current_end[1] - seg[-1][1])
                reverse = d_end < d_start
                dist = min(d_start, d_end)
                if dist <= max_gap:
                    candidates.append((dist, i, reverse))
                if fallback is None or dist < fallback[0]:
                    fallback = (dist, i, reverse)

            if candidates:
                dist, best_idx, best_reverse = min(candidates, key=lambda t: t[0])
            else:
                dist, best_idx, best_reverse = fallback

            seg = segments.pop(best_idx)
            if best_reverse:
                seg = list(reversed(seg))
            if dist > max_gap:
                large_gap_count += 1
                max_stitch_gap = max(max_stitch_gap, dist)
                if self.log_centerline_stitch_details:
                    self.get_logger().warn(
                        f"No neighbor within {max_gap:.2f}m; stitching nearest gap {dist:.2f}m."
                    )
            path.extend(seg[1:])
            current_end = path[-1]

        if large_gap_count > 0:
            self.get_logger().warn(
                f"Centerline stitching used {large_gap_count} gap bridge(s) above "
                f"{max_gap:.2f}m; max_gap={max_stitch_gap:.2f}m. "
                "Use centerline_scope:=selected_lanelet or "
                "log_centerline_stitch_details:=true for detailed map debugging."
            )

        return path

    # Implements `_find_left_right_bounds` behavior.
    def _find_left_right_bounds(self, root):
        selected_left = None
        selected_right = None
        for rel in root.findall("relation"):
            tags = {t.attrib["k"]: t.attrib["v"] for t in rel.findall("tag")}
            if tags.get("type") != "lanelet":
                continue
            rid = int(rel.attrib["id"])
            if self.lanelet_id >= 0 and rid != int(self.lanelet_id):
                continue
            for member in rel.findall("member"):
                role = member.attrib.get("role")
                if role == "left":
                    selected_left = int(member.attrib["ref"])
                elif role == "right":
                    selected_right = int(member.attrib["ref"])
            if selected_left is not None and selected_right is not None:
                break
        return selected_left, selected_right

    # HH_260428: Convert ENU map coordinates (x=east, y=north) to lat/lon.
    # Uses flat-earth approximation; accurate to ~cm within a few km of origin.
    def _xy_to_latlon(self, x, y):
        lat = self.origin_lat + (y / WGS84_A) * (180.0 / math.pi)
        cos_lat = math.cos(deg2rad(self.origin_lat))
        lon = self.origin_lon + (x / (WGS84_A * cos_lat)) * (180.0 / math.pi)
        return lat, lon

    # Implements `_build_path_from_way` behavior.
    def _build_path_from_way(self, way_nodes, nodes):
        origin_lat_rad = deg2rad(self.origin_lat)
        origin_lon_rad = deg2rad(self.origin_lon)
        ref_ecef = llh_to_ecef(origin_lat_rad, origin_lon_rad, self.origin_alt)
        path = []
        for nid in way_nodes:
            if nid not in nodes:
                continue
            lat, lon = nodes[nid]
            cur_ecef = llh_to_ecef(deg2rad(lat), deg2rad(lon), self.origin_alt)
            enu = ecef_to_enu(ref_ecef, cur_ecef, origin_lat_rad, origin_lon_rad)
            path.append((enu[0], enu[1], 0.0, lat, lon))
        return path

    # Implements `_build_path_from_bounds` behavior.
    def _build_path_from_bounds(self, left_way_nodes, right_way_nodes, nodes):
        origin_lat_rad = deg2rad(self.origin_lat)
        origin_lon_rad = deg2rad(self.origin_lon)
        ref_ecef = llh_to_ecef(origin_lat_rad, origin_lon_rad, self.origin_alt)

        # Implements `way_to_xy` behavior.
        def way_to_xy(way_nodes):
            pts = []
            for nid in way_nodes:
                if nid not in nodes:
                    continue
                lat, lon = nodes[nid]
                cur_ecef = llh_to_ecef(deg2rad(lat), deg2rad(lon), self.origin_alt)
                enu = ecef_to_enu(ref_ecef, cur_ecef, origin_lat_rad, origin_lon_rad)
                pts.append((enu[0], enu[1], lat, lon))
            return pts

        left_pts = way_to_xy(left_way_nodes)
        right_pts = way_to_xy(right_way_nodes)
        if len(left_pts) < 2 or len(right_pts) < 2:
            self.get_logger().error("Left/right bounds are too short to synthesize centerline.")
            return []

        left_dist = self._build_cumulative_distances(left_pts)
        right_dist = self._build_cumulative_distances(right_pts)
        max_s = min(left_dist[-1], right_dist[-1])
        step = max(float(self.centerline_step), 0.2)
        s = 0.0
        path = []
        while s <= max_s:
            lx, ly, lat_l, lon_l = self._sample_polyline(left_pts, left_dist, s)
            rx, ry, lat_r, lon_r = self._sample_polyline(right_pts, right_dist, s)
            path.append(((lx + rx) * 0.5, (ly + ry) * 0.5, 0.0, (lat_l + lat_r) * 0.5, (lon_l + lon_r) * 0.5))
            s += step
        if not path:
            self.get_logger().error("Failed to synthesize centerline path from bounds.")
        return path

    @staticmethod
    # Implements `_build_cumulative_distances` behavior.
    def _build_cumulative_distances(points):
        distances = [0.0]
        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            distances.append(distances[-1] + math.hypot(dx, dy))
        return distances

    # Implements `_nearest_distance_on_path` behavior.
    def _nearest_distance_on_path(self, x, y):
        # HH_260527: Use nearest projection on each segment (not nearest vertex only)
        # so RViz initialpose snaps to the geometrically closest path point.
        if len(self._path) < 2:
            return 0.0

        best_dist2 = float("inf")
        best_s = 0.0
        for i in range(len(self._path) - 1):
            p0 = self._path[i]
            p1 = self._path[i + 1]
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            seg_len2 = dx * dx + dy * dy
            if seg_len2 <= 1e-12:
                continue

            t = ((x - p0[0]) * dx + (y - p0[1]) * dy) / seg_len2
            t = max(0.0, min(1.0, t))
            px = p0[0] + t * dx
            py = p0[1] + t * dy
            dist2 = (x - px) * (x - px) + (y - py) * (y - py)
            if dist2 < best_dist2:
                best_dist2 = dist2
                s0 = self._distances[i]
                s1 = self._distances[i + 1]
                best_s = s0 + t * (s1 - s0)
        return best_s

    @staticmethod
    # Implements `_sample_polyline` behavior.
    def _sample_polyline(points, distances, s):
        if s <= 0.0:
            return points[0]
        if s >= distances[-1]:
            return points[-1]
        idx = bisect_left(distances, s)
        idx = max(1, min(idx, len(points) - 1))
        d0 = distances[idx - 1]
        d1 = distances[idx]
        t = 0.0 if d1 <= d0 else (s - d0) / (d1 - d0)
        p0 = points[idx - 1]
        p1 = points[idx]
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        lat = p0[2] + t * (p1[2] - p0[2])
        lon = p0[3] + t * (p1[3] - p0[3])
        return (x, y, lat, lon)

    # Implements `_sample_path` behavior.
    def _sample_path(self, dist):
        if self._total_length <= 1e-6:
            return self._path[0], 0.0

        dist += self._start_distance
        if self.loop:
            dist = dist % self._total_length
        else:
            dist = min(dist, self._total_length)

        idx = 1
        while idx < len(self._distances) and self._distances[idx] < dist:
            idx += 1
        idx = min(idx, len(self._distances) - 1)
        d0 = self._distances[idx - 1]
        d1 = self._distances[idx]
        t = 0.0 if d1 <= d0 else (dist - d0) / (d1 - d0)
        p0 = self._path[idx - 1]
        p1 = self._path[idx]
        x = p0[0] + t * (p1[0] - p0[0])
        y = p0[1] + t * (p1[1] - p0[1])
        z = p0[2] + t * (p1[2] - p0[2])
        lat = p0[3] + t * (p1[3] - p0[3])
        lon = p0[4] + t * (p1[4] - p0[4])
        yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        return (x, y, z, lat, lon), yaw

    # Handles the `_on_localization_pose_yaw_path` callback.
    def _on_localization_pose_yaw_path(self, msg: Path):
        if not msg or len(msg.poses) < 2:
            self._yaw_path_points = []
            self._yaw_path_ready = False
            return
        points = []
        for ps in msg.poses:
            points.append((ps.pose.position.x, ps.pose.position.y))
        self._yaw_path_points = points
        self._yaw_path_ready = len(points) >= 2

    # Computes `NearestPathYaw` values.
    def _nearest_path_yaw(self, x: float, y: float):
        if not self._yaw_path_ready or len(self._yaw_path_points) < 2:
            return None
        best_dist2 = float("inf")
        best_yaw = None
        min_seg_len = max(1e-6, float(self.localization_pose_yaw_min_segment_length))
        for i in range(len(self._yaw_path_points) - 1):
            x0, y0 = self._yaw_path_points[i]
            x1, y1 = self._yaw_path_points[i + 1]
            dx = x1 - x0
            dy = y1 - y0
            seg_len2 = dx * dx + dy * dy
            if seg_len2 < min_seg_len * min_seg_len:
                continue
            # project current pose to segment
            t = ((x - x0) * dx + (y - y0) * dy) / seg_len2
            t = max(0.0, min(1.0, t))
            px = x0 + t * dx
            py = y0 + t * dy
            dist2 = (x - px) * (x - px) + (y - py) * (y - py)
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_yaw = math.atan2(dy, dx)
        return best_yaw

    # Handles the `_on_localization_pose_input` callback.
    def _on_localization_pose_input(self, msg: PoseStamped):
        if self.pub_localization_pose_yaw_aligned is None:
            return
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        if self.localization_pose_lock_position:
            if self.localization_pose_lock_on_first_msg and not self._lock_position_initialized:
                self.localization_pose_lock_x = x
                self.localization_pose_lock_y = y
                self.localization_pose_lock_z = z
                self._lock_position_initialized = True
            x = self.localization_pose_lock_x
            y = self.localization_pose_lock_y
            z = self.localization_pose_lock_z
            out.pose.position.x = x
            out.pose.position.y = y
            out.pose.position.z = z

        yaw = self._nearest_path_yaw(x, y)
        if yaw is not None:
            now_sec = time.time()
            if self._last_aligned_yaw is not None and self._last_aligned_yaw_time is not None:
                # 1) unwrap to nearest equivalent angle around previous yaw
                dyaw = normalize_angle(yaw - self._last_aligned_yaw)
                # 2) optional yaw-rate limit
                max_rate = max(0.0, float(self.localization_pose_yaw_max_rate_deg_s)) * math.pi / 180.0
                dt = max(1e-3, now_sec - self._last_aligned_yaw_time)
                if max_rate > 0.0:
                    max_step = max_rate * dt
                    if dyaw > max_step:
                        dyaw = max_step
                    elif dyaw < -max_step:
                        dyaw = -max_step
                yaw = self._last_aligned_yaw + dyaw
                # 3) optional smoothing (1.0 = no smoothing)
                alpha = min(1.0, max(0.0, float(self.localization_pose_yaw_smoothing_alpha)))
                yaw = self._last_aligned_yaw + alpha * normalize_angle(yaw - self._last_aligned_yaw)
            self._last_aligned_yaw = yaw
            self._last_aligned_yaw_time = now_sec
            out.pose.orientation = yaw_to_quat(yaw)
        self.pub_localization_pose_yaw_aligned.publish(out)

    # Implements `_on_timer` behavior.
    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        now_sec = time.time()
        elapsed = now_sec - self._t0
        dt = max(1e-3, now_sec - self._last_timer_time)
        self._last_timer_time = now_sec
        holding = elapsed < self.startup_hold_s
        lateral_speed = 0.0
        if self.freeze_motion or holding:
            motion_speed = 0.0
        elif self.motion_uses_cmd_vel:
            if (self._last_cmd_time is None or
                    (now_sec - self._last_cmd_time) > max(0.01, self.cmd_vel_timeout_s)):
                motion_speed = 0.0
                lateral_speed = 0.0
            else:
                motion_speed = self._cmd_linear_x
                lateral_speed = self._cmd_linear_y
                if (
                    abs(motion_speed) <= self.cmd_vel_deadband and
                    abs(lateral_speed) <= self.cmd_vel_deadband and
                    self._last_nonzero_cmd_time is not None and
                    (now_sec - self._last_nonzero_cmd_time) <= max(0.0, self.cmd_vel_nonzero_hold_s)
                ):
                    motion_speed = self._last_nonzero_cmd_linear_x
                    lateral_speed = self._last_nonzero_cmd_linear_y
        else:
            motion_speed = self.speed_mps

        if self.free_nav_mode_enabled:
            # HH_260428: Free nav mode — body-frame cmd_vel integration.
            # Integrates linear.x, linear.y, and angular.z so Nav2 and parking
            # controllers can steer the simulated robot off the lanelet centerline.
            # HH_260618: Integrate angular-only commands as well. Nav2 can issue
            # rotate-in-place alignment before forward motion; ignoring zero-vx
            # angular.z leaves the simulated yaw frozen and the controller stuck.
            # HH_260618: Integrate lateral linear.y for campsite crab entry/exit.
            if not holding and not self.freeze_motion:
                omega = self._cmd_angular_z if self.motion_uses_cmd_vel else 0.0
                if (
                    self.motion_uses_cmd_vel and
                    abs(omega) <= self.cmd_vel_deadband and
                    self._last_nonzero_cmd_time is not None and
                    (now_sec - self._last_nonzero_cmd_time) <= max(0.0, self.cmd_vel_nonzero_hold_s)
                ):
                    omega = self._last_nonzero_cmd_angular_z
                self._free_nav_yaw = normalize_angle(self._free_nav_yaw + omega * dt)
                yaw_cos = math.cos(self._free_nav_yaw)
                yaw_sin = math.sin(self._free_nav_yaw)
                self._free_nav_x += (motion_speed * yaw_cos - lateral_speed * yaw_sin) * dt
                self._free_nav_y += (motion_speed * yaw_sin + lateral_speed * yaw_cos) * dt
            x = self._free_nav_x
            y = self._free_nav_y
            z = 0.0
            yaw = self._free_nav_yaw
            lat, lon = self._xy_to_latlon(x, y)
        else:
            self._motion_distance += motion_speed * dt
            if not self.loop:
                self._motion_distance = max(0.0, self._motion_distance)
            dist = self._motion_distance
            (x, y, z, lat, lon), yaw = self._sample_path(dist)

        # Create a pose message for downstream odometry/cloud outputs.
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation = yaw_to_quat(yaw)

        navsat = NavSatFix()
        navsat.header.stamp = now
        navsat.header.frame_id = self.frame_id
        navsat.status.status = NavSatStatus.STATUS_FIX
        navsat.status.service = NavSatStatus.SERVICE_GPS
        navsat.latitude = lat
        navsat.longitude = lon
        navsat.altitude = self.origin_alt
        # Set realistic RTK-quality covariance so localization_monitor's
        # gnss_cov_trace_fail (0.3 m^2) accepts the fake GNSS (trace = 0.08 < 0.3).
        # COVARIANCE_TYPE_DIAGONAL_KNOWN = 2; without this the adapter falls back to
        # pose_covariance_diagonal=[1,1,...] giving trace=2.0 which fails the monitor.
        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        navsat.position_covariance[0] = 0.04   # σ_x = 0.2 m
        navsat.position_covariance[4] = 0.04   # σ_y = 0.2 m
        navsat.position_covariance[8] = 0.10   # σ_z = 0.32 m
        # Gate NavSatFix on simulated GNSS failure window.
        gnss_should_publish = True
        if self.gnss_failure_after_s > 0.0 and elapsed >= self.gnss_failure_after_s:
            if self.gnss_recovery_after_s <= 0.0 or elapsed < self.gnss_recovery_after_s:
                gnss_should_publish = False
        if gnss_should_publish != self._gnss_active:
            self._gnss_active = gnss_should_publish
            if gnss_should_publish:
                self.get_logger().warn(f"[GNSS SIM] RECOVERED at t={elapsed:.1f}s")
            else:
                self.get_logger().warn(
                    f"[GNSS SIM] FAILURE at t={elapsed:.1f}s — NavSatFix publishing stopped"
                )
        if self._gnss_active:
            self.pub_navsat.publish(navsat)

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.base_frame_id
        imu_msg.orientation = yaw_to_quat(yaw)
        # 2026-02-02 11:55: Provide yaw rate so ESKF can integrate heading.
        yaw_rate = 0.0
        if (not holding and not self._was_holding and
                self._last_yaw is not None and self._last_yaw_time is not None):
            dt = max(1e-3, now_sec - self._last_yaw_time)
            dyaw = normalize_angle(yaw - self._last_yaw)
            yaw_rate = dyaw / dt
        self._last_yaw = yaw
        self._last_yaw_time = now_sec
        self._was_holding = holding
        imu_msg.angular_velocity.z = yaw_rate
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        self.pub_imu.publish(imu_msg)

        wheel_msg = Odometry()
        wheel_msg.header.stamp = now
        wheel_msg.header.frame_id = "odom"
        wheel_msg.child_frame_id = self.base_frame_id
        wheel_speed = motion_speed
        wheel_msg.twist.twist.linear.x = wheel_speed
        wheel_msg.twist.twist.linear.y = lateral_speed
        # HH_260618: Publish the simulated yaw-rate on wheel/DR odometry so
        # localization and Nav2 see the same in-place rotation that free_nav
        # integrates from /platform/cmd_vel.
        wheel_msg.twist.twist.angular.z = yaw_rate
        self.pub_wheel.publish(wheel_msg)
        # Mirror wheel odometry onto wheel-bridge input topic for
        # sim-time localization/TF/controller reproducibility.
        self.pub_wheel_bridge_in.publish(wheel_msg)

        if self.pub_velocity_converter_output is not None:
            twist_cov = TwistWithCovarianceStamped()
            twist_cov.header.stamp = now
            twist_cov.header.frame_id = self.base_frame_id
            twist_cov.twist.twist.linear.x = wheel_speed
            twist_cov.twist.twist.linear.y = lateral_speed
            twist_cov.twist.twist.angular.z = yaw_rate
            twist_cov.twist.covariance[0] = 0.05
            twist_cov.twist.covariance[7] = 0.05
            twist_cov.twist.covariance[14] = 0.10
            twist_cov.twist.covariance[21] = 0.20
            twist_cov.twist.covariance[28] = 0.20
            twist_cov.twist.covariance[35] = 0.20
            self.pub_velocity_converter_output.publish(twist_cov)

        dr_msg = Odometry()
        dr_msg.header.stamp = now
        dr_msg.header.frame_id = self.frame_id
        dr_msg.child_frame_id = self.base_frame_id
        dr_msg.pose.pose = pose_msg.pose
        dr_msg.twist.twist.linear.x = wheel_speed
        dr_msg.twist.twist.linear.y = lateral_speed
        dr_msg.twist.twist.angular.z = yaw_rate
        self.pub_dr_odom.publish(dr_msg)

        # Place fake obstacle cloud in vehicle-forward coordinates
        # so local cost-stop can validate front/side/rear stop behavior reliably.
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        left_x = -forward_y
        left_y = forward_x
        obstacle_dir_x = forward_x
        obstacle_dir_y = forward_y
        if self.obstacle_direction == "left":
            obstacle_dir_x = left_x
            obstacle_dir_y = left_y
        elif self.obstacle_direction == "right":
            obstacle_dir_x = -left_x
            obstacle_dir_y = -left_y
        elif self.obstacle_direction == "rear":
            obstacle_dir_x = -forward_x
            obstacle_dir_y = -forward_y

        center_x = (
            x
            + self.obstacle_offset * obstacle_dir_x
            + self.obstacle_lateral_offset * left_x
        )
        center_y = (
            y
            + self.obstacle_offset * obstacle_dir_y
            + self.obstacle_lateral_offset * left_y
        )
        # HH_260630: Publish a compact 3x3 cluster centered at the requested
        # obstacle point so directional stop tests exercise the intended corridor.
        obstacle_perp_x = -obstacle_dir_y
        obstacle_perp_y = obstacle_dir_x
        cluster_radius = max(0.0, float(self.fake_obstacle_cluster_radius_m))
        obstacle_points = []
        for along in (-cluster_radius, 0.0, cluster_radius):
            for side in (-cluster_radius, 0.0, cluster_radius):
                obstacle_points.append(
                    (
                        center_x + along * obstacle_dir_x + side * obstacle_perp_x,
                        center_y + along * obstacle_dir_y + side * obstacle_perp_y,
                        self.obstacle_height,
                    )
                )
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = point_cloud2.create_cloud(pose_msg.header, fields, obstacle_points)
        if self.publish_fake_lidar_obstacle_cloud:
            self.pub_obstacles.publish(cloud_msg)
            self.pub_lidar_filtered.publish(cloud_msg)
        if self.publish_fake_radar_ranges:
            self._publish_fake_radar_ranges(now)

        dummy_grid_period_s = 1.0 / max(0.1, self.dummy_lidar_cost_grid_publish_rate_hz)
        if (
            self.pub_dummy_lidar_cost_grid is not None and
            (now_sec - self._last_dummy_lidar_cost_grid_pub_sec) >= dummy_grid_period_s
        ):
            self._last_dummy_lidar_cost_grid_pub_sec = now_sec
            grid = OccupancyGrid()
            grid.header.stamp = now
            grid.header.frame_id = self.frame_id
            grid.info.resolution = float(self.dummy_lidar_cost_grid_resolution)
            grid.info.width = max(1, int(self.dummy_lidar_cost_grid_width))
            grid.info.height = max(1, int(self.dummy_lidar_cost_grid_height))
            grid.info.origin.position.x = x - 0.5 * grid.info.width * grid.info.resolution
            grid.info.origin.position.y = y - 0.5 * grid.info.height * grid.info.resolution
            grid.info.origin.orientation.w = 1.0
            grid.data = [0] * int(grid.info.width * grid.info.height)
            self.pub_dummy_lidar_cost_grid.publish(grid)

    # HH_260707: Mirror the real SEN0592 heartbeat behavior. Publish every
    # radar topic on every fake-sensor tick; non-hit sensors report
    # max_range+epsilon so diagnostics stay alive while cost-grid consumers
    # discard the sample as outside valid range.
    def _publish_fake_radar_ranges(self, stamp):
        if not self.pub_fake_radar_ranges:
            return
        direction_to_indices = {
            "front": (0, 1),
            "left": (2, 3),
            "right": (4, 5),
            "rear": (6,),
        }
        hit_indices = set(direction_to_indices.get(self.obstacle_direction, ()))
        distance = float(self.obstacle_offset)
        for idx, pub in enumerate(self.pub_fake_radar_ranges):
            max_range = (
                self.fake_radar_max_ranges_m[idx]
                if idx < len(self.fake_radar_max_ranges_m)
                else 0.0
            )
            min_range = max(0.0, float(self.fake_radar_min_range_m))
            hit = idx in hit_indices and min_range <= distance <= max_range
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = (
                self.fake_radar_frame_ids[idx]
                if idx < len(self.fake_radar_frame_ids)
                else ""
            )
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = float(self.fake_radar_field_of_view_rad)
            msg.min_range = min_range
            msg.max_range = float(max_range)
            msg.range = distance if hit else float(max_range + 0.001)
            pub.publish(msg)

    # Normalizes obstacle_direction parameter and falls back safely.
    def _normalize_obstacle_direction(self, value):
        direction = str(value).strip().lower()
        if direction in ("front", "left", "right", "rear"):
            return direction
        self.get_logger().warn(
            f"Invalid obstacle_direction '{value}'. Falling back to 'front'."
        )
        return "front"

    # Applies selected runtime parameter updates without node restart.
    def _on_set_parameters(self, params):
        for p in params:
            if p.name == "obstacle_offset":
                self.obstacle_offset = float(p.value)
            elif p.name == "obstacle_height":
                self.obstacle_height = float(p.value)
            elif p.name == "obstacle_direction":
                self.obstacle_direction = self._normalize_obstacle_direction(p.value)
            elif p.name == "obstacle_lateral_offset":
                self.obstacle_lateral_offset = float(p.value)
            elif p.name == "fake_obstacle_cluster_radius_m":
                self.fake_obstacle_cluster_radius_m = float(p.value)
            elif p.name == "publish_fake_lidar_obstacle_cloud":
                self.publish_fake_lidar_obstacle_cloud = bool(p.value)
            elif p.name == "publish_fake_radar_ranges":
                self.publish_fake_radar_ranges = bool(p.value)
            elif p.name == "speed_mps":
                self.speed_mps = float(p.value)
            elif p.name == "publish_rate_hz":
                self.publish_rate_hz = float(p.value)
            elif p.name == "dummy_lidar_cost_grid_publish_rate_hz":
                self.dummy_lidar_cost_grid_publish_rate_hz = float(p.value)
            elif p.name == "freeze_motion":
                self.freeze_motion = bool(p.value)
            elif p.name == "gnss_failure_after_s":
                self.gnss_failure_after_s = float(p.value)
            elif p.name == "gnss_recovery_after_s":
                self.gnss_recovery_after_s = float(p.value)
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = FakeSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"fake_sensor_publisher runtime exception: {e}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
