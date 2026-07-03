#!/usr/bin/env python3
"""
Core logic unit tests for planning_cmd_vel_gate_node.
Runs without a ROS 2 runtime by stubbing rclpy, tf2_ros, and message types.

Test coverage:
  1. front cost-stop corridor blocks cells above threshold
  2. below-threshold costs pass
  3. side corridors stop left/right motion
  4. rear corridor stops reverse motion
  5. unavoidable cluster detection stops large lethal groups
  6. GNSS recovery hold blocks cmd_vel on DR_ONLY -> NORMAL recovery
  7. recovery hold expiry restores _effective_enabled() = True
  8. e-stop blocks cmd_vel
  9. engage = False blocks cmd_vel
  10. cost_stop_hold expiry restores pass-through
  15. clear avoidance local-path corridor can pass a body-front obstacle

=== Behavior Model ===

[ Cost-Stop ]
The gate builds OccupancyGrid-based front/side/rear corridors around the robot
and scans the cost value of cells in each corridor.

  cost_stop_threshold (default 85):
    - any OccupancyGrid cell at or above this value stops immediately
    - front: speed-dependent lookahead (0.4 to 3.0 m)
      braking_dist = v^2/(2*g*mu) + reaction_time*v + margin
    - side: 1.2 m lookahead per side, 0.6 m corridor width
    - rear: 0.8 m lookahead, 0.9 m corridor width

  unavoidable_lethal_threshold (default 90):
    - even below the direct stop threshold, a connected lethal-cell group that
      covers at least 25% of the corridor stops the robot as unavoidable

  cost_stop_hold_s (default 1.0):
    - keeps zero cmd_vel for 1 second after the stop condition clears

[ GNSS Recovery Hold ]
When ESKF localization recovers from DR_ONLY(2) to NORMAL(0):
  1. the DR_ONLY source mode must persist long enough to count as recovery
  2. the gate forces zero cmd_vel for gnss_recovery_hold_s
  3. cmd_vel passes again after hold expiry; repeated holds are skipped during cooldown

  /localization/mode values:
    NORMAL=0, DEGRADED=1, DR_ONLY=2, INVALID=3
"""

from __future__ import annotations

import math
import sys
import time
import types
from pathlib import Path as FsPath

# --- ROS 2 stubs (rclpy, tf2_ros, nav_msgs, etc.) ---------------------------

class FakeTime:
    def __init__(self, ns: int = 0):
        self._ns = ns

    @property
    def nanoseconds(self) -> int:
        return self._ns


class FakeClock:
    def __init__(self, ns: int = 0):
        self._ns = ns

    def now(self) -> FakeTime:
        return FakeTime(self._ns)

    def set_ns(self, ns: int) -> None:
        self._ns = ns

    def advance_s(self, s: float) -> None:
        self._ns += int(s * 1e9)


class FakeLogger:
    def info(self, m: str) -> None:
        print(f"  [INFO ] {m}")

    def warn(self, m: str) -> None:
        print(f"  [WARN ] {m}")

    def error(self, m: str) -> None:
        print(f"  [ERROR] {m}")


class FakePublisher:
    def __init__(self) -> None:
        self.last = None
        self.history: list = []

    def publish(self, msg) -> None:
        self.last = msg
        self.history.append(msg)


class FakeBuffer:
    def lookup_transform(self, *a, **kw):
        raise Exception("TF not available in unit test")


class FakeTransformListener:
    def __init__(self, *a, **kw):
        pass


# --- Message stubs -----------------------------------------------------------

class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x; self.y = y; self.z = z

class Twist:
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()

class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x; self.y = y; self.z = z

class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x; self.y = y; self.z = z; self.w = w

class Pose:
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()

class PoseWithCovariance:
    def __init__(self):
        self.pose = Pose()
        self.covariance = [0.0] * 36

class TwistWithCovariance:
    def __init__(self):
        self.twist = Twist()
        self.covariance = [0.0] * 36

class Header:
    def __init__(self, frame_id="map"):
        self.frame_id = frame_id
        self.stamp = FakeTime()

class MapMetaData:
    def __init__(self):
        self.resolution = 0.1
        self.width = 0
        self.height = 0
        self.origin = Pose()

class OccupancyGrid:
    def __init__(self):
        self.header = Header()
        self.info = MapMetaData()
        self.data: list[int] = []

class PoseStamped:
    def __init__(self):
        self.header = Header()
        self.pose = Pose()

class Path:
    def __init__(self):
        self.header = Header()
        self.poses: list[PoseStamped] = []

class Odometry:
    def __init__(self):
        self.header = Header()
        self.pose = PoseWithCovariance()
        self.twist = TwistWithCovariance()

class BoolMsg:
    def __init__(self, data: bool = False):
        self.data = data

class SetParametersResult:
    def __init__(self, successful: bool = True):
        self.successful = successful

class AvgLocalizationMode:
    NORMAL  = 0
    DEGRADED = 1
    DR_ONLY = 2
    INVALID = 3

    def __init__(self, value: int = 0):
        self.value = value
        self.label = {0: "NORMAL", 1: "DEGRADED", 2: "DR_ONLY", 3: "INVALID"}.get(value, str(value))

class ModuleState:
    OK = 0
    WARN = 1
    ERROR = 2

    def __init__(self):
        self.message = ""


# --- Patch sys.modules -------------------------------------------------------

def _mod(**attrs):
    m = types.ModuleType("_stub")
    for k, v in attrs.items():
        setattr(m, k, v)
    return m

sys.modules.update({
    "rclpy":                        _mod(init=lambda *a,**kw: None, ok=lambda: True, shutdown=lambda: None),
    "rclpy.node":                   _mod(Node=object),
    "rclpy.time":                   _mod(Time=FakeTime),
    "rclpy.qos":                    _mod(QoSProfile=lambda **kw: None,
                                         QoSReliabilityPolicy=_mod(RELIABLE=1),
                                         QoSDurabilityPolicy=_mod(TRANSIENT_LOCAL=1)),
    "rcl_interfaces.msg":           _mod(SetParametersResult=SetParametersResult),
    "geometry_msgs.msg":            _mod(PoseStamped=PoseStamped, Twist=Twist),
    "nav_msgs.msg":                 _mod(OccupancyGrid=OccupancyGrid, Odometry=Odometry, Path=Path),
    "std_msgs.msg":                 _mod(Bool=BoolMsg),
    "tf2_ros":                      _mod(Buffer=FakeBuffer, TransformException=Exception,
                                         TransformListener=FakeTransformListener),
    "avg_msgs.msg":                 _mod(AvgLocalizationMode=AvgLocalizationMode, ModuleState=ModuleState),
})
for top in ["rcl_interfaces", "geometry_msgs", "nav_msgs", "std_msgs", "avg_msgs"]:
    if top not in sys.modules:
        sys.modules[top] = types.ModuleType(top)

# --- Import target -----------------------------------------------------------

sys.path.insert(0, str(FsPath(__file__).resolve().parents[1] / "scripts"))
import planning_cmd_vel_gate_node as gmod


# --- Factory -----------------------------------------------------------------

def make_gate(
    cost_threshold: int = 85,
    hold_s: float = 1.0,
    recovery_hold_s: float = 2.0,
    enable_side_rear: bool = True,
    enable_unavoidable: bool = True,
    enable_cost_latch: bool = False,
    enable_stale_stop: bool = False,
) -> gmod.PlanningCmdVelGateNode:
    """Create the node instance directly without a ROS 2 runtime."""
    n = gmod.PlanningCmdVelGateNode.__new__(gmod.PlanningCmdVelGateNode)

    # --- ROS 2 interface stubs ---
    n._clock = FakeClock()
    n._logger = FakeLogger()
    n.pub_cmd = FakePublisher()
    n.pub_state = FakePublisher()
    n._tf_buffer = FakeBuffer()
    n._tf_listener = FakeTransformListener()

    def get_clock():
        return n._clock
    def get_logger():
        return n._logger

    n.get_clock = get_clock
    n.get_logger = get_logger

    # --- Parameters ---
    n.input_topic = "/planning/cmd_vel_raw"
    n.output_topic = "/planning/cmd_vel"
    n.engage_topic = "/planning/engage"
    n.mission_engage_topic = "/planning/mission_engage"
    n.state_topic = "/planning/engaged"
    n.estop_topic_enabled = True
    n.dr_timeout_topic_enabled = True
    n.estop_topic = "/platform/status/estop"
    n.allow_on_start = False
    n.publish_zero_when_blocked = True
    n.speed_scale = 1.0

    n.enable_gnss_recovery_hold = True
    n.localization_mode_topic = "/localization/mode"
    n.gnss_recovery_hold_s = recovery_hold_s
    n.gnss_recovery_min_source_s = 0.5
    n.gnss_recovery_hold_cooldown_s = 5.0
    n.gnss_recovery_source_mode_min = int(AvgLocalizationMode.DR_ONLY)
    n.gnss_recovery_target_mode = int(AvgLocalizationMode.NORMAL)

    n.enable_cost_stop = True
    n.cost_grid_topic = "/planning/cost_grid/inflation"
    # HH_260623 - Unit tests exercise merged cost-stop only; lanelet safety
    # needs a full map grid fixture and is disabled in this lightweight stub.
    n.lanelet_safety_enable = False
    n.lanelet_safety_allow_rotation_in_place = True
    n.lanelet_safety_min_translation_mps = 0.02
    # HH_260624 - Keep route re-entry defaults in the lightweight unit stub.
    n.lanelet_safety_lookahead_m = 1.0
    n.lanelet_safety_front_path_allow_route_reentry = True
    n.lanelet_safety_current_allow_route_reentry = True
    n.lanelet_safety_current_route_reentry_max_distance_m = 4.0
    n.lanelet_safety_current_route_reentry_require_front_cmd = True
    n.pose_topic = "/localization/pose"
    n.odometry_topic = "/localization/fallback/odometry"
    n.pose_source_preference = "odometry"
    n.enable_pose_raw_fallback = True  # Use raw fallback without TF.
    n.robot_base_frame = "robot_base_link"
    n.cost_stop_threshold = cost_threshold
    n.cost_stop_lookahead_m = 2.0
    n.cost_stop_width_m = 1.0
    n.cost_stop_hold_s = hold_s
    n.cost_stop_latch_enable = enable_cost_latch
    n.cost_stop_clear_required_s = 2.0
    n.cost_stop_latch_log_interval_s = 1.0
    n.cost_grid_stale_stop_enable = enable_stale_stop
    n.cost_grid_stale_timeout_s = 1.0
    n.cost_grid_stale_log_interval_s = 1.0
    n.cost_source_debug_enable = False
    n.cost_source_debug_max_age_s = 1.0
    n._cost_source_grids = {}
    n._cost_source_recv_sec = {}
    n.cost_stop_require_dynamic_source = False
    n.cost_stop_dynamic_source_labels = {"lidar", "radar"}
    n.front_dynamic_stop_use_local_path = True
    n.front_dynamic_path_width_m = n.cost_stop_width_m
    n.front_dynamic_path_max_start_distance_m = 1.5
    n.lateral_cmd_bypass_static_cost_stop = True
    n.lateral_cmd_bypass_min_mps = 0.02
    n.reverse_cmd_bypass_static_cost_stop = True
    n.reverse_cmd_bypass_min_mps = 0.02

    n.enable_speed_dependent_lookahead = False  # Use fixed lookahead.
    n.front_lookahead_min_m = 0.4
    n.front_lookahead_max_m = 3.0
    n.front_lookahead_friction = 0.4
    n.front_reaction_time_s = 0.15
    n.front_lookahead_margin_m = 0.3

    n.enable_side_rear_cost_stop = enable_side_rear
    n.side_cost_threshold = cost_threshold
    n.side_lookahead_m = 1.2
    n.side_corridor_width_m = 0.6
    n.rear_cost_threshold = cost_threshold
    n.rear_lookahead_m = 0.8
    n.rear_corridor_width_m = 0.9

    n.enable_unavoidable_stop = enable_unavoidable
    n.unavoidable_lethal_threshold = 90
    n.unavoidable_cluster_min_cells = 25
    n.unavoidable_cluster_min_ratio = 0.25

    # --- Internal state ---
    # HH_260623 - Manual and UI-mission engage are independent OR latches.
    n._manual_enabled = False
    n._mission_enabled = False
    n._enabled = False
    n._estop = False
    n._estop_sources = {
        n.estop_topic: False,
        "/planning/state_machine/estop": False,
    }
    n._dr_timeout = False
    n._cost_blocked_until = 0.0
    n._cost_stop_latched = False
    n._cost_stop_clear_since_sec = None
    n._cost_stop_latch_reason = ""
    n._last_cost_stop_latch_log_sec = 0.0
    n._last_cost_grid_stale_log_sec = 0.0
    n._gnss_recovery_blocked_until = 0.0
    n._gnss_recovery_source_enter_sec = None
    n._gnss_recovery_last_hold_sec = -1.0e9
    n._last_localization_mode_value = None
    n._last_unavoidable_cluster_cells = 0
    n._last_unavoidable_cluster_ratio = 0.0
    n._last_tf_warn_sec = 0.0
    n._last_empty_corridor_warn_sec = 0.0
    n._last_yaw_align_log_sec = 0.0
    n._last_route_heading_log_sec = 0.0
    n._last_lanelet_front_path_reentry_bypass_log_sec = 0.0
    n._last_lanelet_current_reentry_bypass_log_sec = 0.0
    n._last_drop_zone_static_bypass_log_sec = 0.0
    n._last_site_static_phase_bypass_log_sec = 0.0
    n._last_lateral_static_bypass_log_sec = 0.0
    n._last_static_cost_ignored_log_sec = 0.0
    n._last_front_path_dynamic_clear_log_sec = 0.0
    n._last_block_reason_log_sec = 0.0
    n._parking_drop_zone_phase = ""
    n._parking_site_phase = ""
    n._current_speed = 0.0
    n._last_grid = None
    n._last_grid_recv_sec = n._clock.now().nanoseconds * 1e-9
    n._last_lanelet_safety_grid = None
    n._last_route_heading_path = None
    n._route_heading_align_active = False
    n._last_pose = None
    n._last_odom = None

    return n


# --- Grid builders -----------------------------------------------------------

def make_grid(
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    obstacle_x: float = 1.0,
    obstacle_y: float = 0.0,
    obstacle_cost: int = 90,
    grid_size_m: float = 6.0,
    resolution: float = 0.1,
    frame_id: str = "map",
) -> OccupancyGrid:
    """
    Create an OccupancyGrid centered on the robot.
    The obstacle cell is set at obstacle_x/y; all other cells are free.

    Corridor sampling converts world points through grid origin and resolution,
    so robot pose and grid origin must match for correct projection.
    """
    cells = int(grid_size_m / resolution)
    half = grid_size_m / 2.0
    grid = OccupancyGrid()
    grid.header.frame_id = frame_id
    grid.info.resolution = resolution
    grid.info.width = cells
    grid.info.height = cells
    # Origin is the lower-left corner, centered around the robot position.
    grid.info.origin.position.x = robot_x - half
    grid.info.origin.position.y = robot_y - half
    grid.data = [0] * (cells * cells)
    # Set obstacle cell.
    ox = int((obstacle_x - grid.info.origin.position.x) / resolution)
    oy = int((obstacle_y - grid.info.origin.position.y) / resolution)
    if 0 <= ox < cells and 0 <= oy < cells:
        grid.data[oy * cells + ox] = obstacle_cost
    return grid


def make_odom(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> Odometry:
    """Create an odometry message for the robot pose."""
    odom = Odometry()
    odom.header.frame_id = "map"
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation.w = math.cos(yaw * 0.5)
    odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
    return odom


def make_path(points: list[tuple[float, float]], frame_id: str = "map") -> Path:
    """Create a Path message for route-distance tests."""
    path = Path()
    path.header.frame_id = frame_id
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        path.poses.append(pose)
    return path


# --- Test execution ----------------------------------------------------------

results: list[bool] = []


def check(label: str, cond: bool) -> None:
    tag = "PASS" if cond else "FAIL"
    print(f"  [{tag}] {label}")
    results.append(cond)


# ===============================================================================
print("\n=== TEST 1: front cost-stop blocks obstacle above threshold ===")
print("  Place a cost=90 obstacle 1.5 m ahead; it is inside the 2.0 m lookahead")
n = make_gate()
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)   # Robot faces east at (0, 0).
n._last_grid = make_grid(
    robot_x=0.0, robot_y=0.0,
    obstacle_x=1.5, obstacle_y=0.0,  # 1.5 m front obstacle.
    obstacle_cost=90,
)
stop = n._should_stop_for_cost()
check("threshold=90 front obstacle -> stop", stop)
check("cost_blocked_until is set", n._cost_blocked_until > 0.0)


# ===============================================================================
print("\n=== TEST 2: below-threshold obstacle passes ===")
print("  cost=60 is below threshold=85, so it should not stop")
n = make_gate(cost_threshold=85)
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=60)
stop = n._should_stop_for_cost()
check("cost=60 < threshold=85 -> pass", not stop)
check("cost_blocked_until remains 0", n._cost_blocked_until == 0.0)


# ===============================================================================
print("\n=== TEST 3: side-left cost-stop ===")
print("  A cost=90 obstacle 0.8 m left is inside the 1.2 m side corridor")
n = make_gate(enable_side_rear=True)
n._enabled = True
# Robot faces east; left is north (+y).
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
n._last_grid = make_grid(
    robot_x=0.0, robot_y=0.0,
    obstacle_x=0.0, obstacle_y=0.8,  # 0.8 m left obstacle.
    obstacle_cost=90,
)
stop = n._should_stop_for_cost()
check("left 0.8 m obstacle -> side stop", stop)


# ===============================================================================
print("\n=== TEST 4: rear cost-stop ===")
print("  A cost=90 obstacle 0.6 m behind is inside the 0.8 m rear corridor")
n = make_gate(enable_side_rear=True)
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
n._last_grid = make_grid(
    robot_x=0.0, robot_y=0.0,
    obstacle_x=-0.6, obstacle_y=0.0,  # 0.6 m rear obstacle.
    obstacle_cost=90,
)
stop = n._should_stop_for_cost()
check("rear 0.6 m obstacle -> rear stop", stop)


# ===============================================================================
print("\n=== TEST 5: unavoidable cluster detection ===")
print("  Works when stop_threshold > unavoidable_lethal_threshold:")
print("  stop_threshold=95, unavoidable_lethal=90, cost=91")
print("  Single cells are below 95, so direct stop does not trigger")
print("  91 >= 90 enters lethal_cells; cluster coverage >= 25% triggers stop")

def make_cluster_grid(
    robot_x: float = 0.0,
    cluster_x_start: float = 0.5,
    cluster_width: float = 1.5,   # Cover the full 1.0 m corridor width.
    cluster_depth: float = 1.0,   # Depth along the lookahead direction.
    cost: int = 91,
    resolution: float = 0.1,
) -> OccupancyGrid:
    """
    Create a cluster grid that broadly covers the front corridor.
    The cost must be below direct stop_threshold but above
    unavoidable_lethal_threshold so the unavoidable-cluster path is tested.
    """
    grid_size_m = 6.0
    cells = int(grid_size_m / resolution)
    half = grid_size_m / 2.0
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = resolution
    grid.info.width = cells
    grid.info.height = cells
    grid.info.origin.position.x = robot_x - half
    grid.info.origin.position.y = -half
    grid.data = [0] * (cells * cells)
    x_lo = cluster_x_start
    x_hi = cluster_x_start + cluster_depth
    y_lo = -cluster_width / 2.0
    y_hi = cluster_width / 2.0
    for cx in range(cells):
        for cy in range(cells):
            wx = grid.info.origin.position.x + cx * resolution
            wy = grid.info.origin.position.y + cy * resolution
            if x_lo <= wx <= x_hi and y_lo <= wy <= y_hi:
                grid.data[cy * cells + cx] = cost
    return grid

# Raise stop_threshold to 95 so cost=91 bypasses direct stop.
# With unavoidable_lethal_threshold=90, cost=91 cells are lethal.
n = make_gate(cost_threshold=95, enable_unavoidable=True)
n.unavoidable_lethal_threshold = 90
n.unavoidable_cluster_min_cells = 25
n.unavoidable_cluster_min_ratio = 0.25
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
n._last_grid = make_cluster_grid(cost=91)  # Below 95 direct stop, above 90 lethal.
stop = n._should_stop_for_cost()
check("large lethal cluster -> unavoidable stop", stop)
check("cluster cell count >= 25", n._last_unavoidable_cluster_cells >= 25)
check("cluster ratio >= 0.25", n._last_unavoidable_cluster_ratio >= 0.25)


# ===============================================================================
print("\n=== TEST 6: GNSS recovery hold triggers ===")
print("  DR_ONLY -> NORMAL transition blocks cmd_vel for gnss_recovery_hold_s")
n = make_gate(recovery_hold_s=2.0)
n._enabled = True
t0_ns = int(100 * 1e9)   # t=100 s.
n._clock.set_ns(t0_ns)

# Receive DR_ONLY as the source mode.
mode_dr = AvgLocalizationMode(AvgLocalizationMode.DR_ONLY)
n._on_localization_mode(mode_dr)
check("DR_ONLY alone does not start hold", n._gnss_recovery_blocked_until == 0.0)

# If NORMAL returns too quickly, treat it as flap and skip hold.
mode_normal = AvgLocalizationMode(AvgLocalizationMode.NORMAL)
n._on_localization_mode(mode_normal)
check("short DR_ONLY flap -> hold skipped", n._gnss_recovery_blocked_until == 0.0)

# NORMAL after minimum DR_ONLY duration starts hold.
n._on_localization_mode(mode_dr)
n._clock.advance_s(0.6)
n._on_localization_mode(mode_normal)
hold_until = n._gnss_recovery_blocked_until
check("NORMAL recovery -> hold is set", hold_until > 0.0)
check(f"hold = now+2.0s ({hold_until:.1f}s)", abs(hold_until - (n._clock.now().nanoseconds * 1e-9 + 2.0)) < 0.1)
check("during hold _effective_enabled() = False", not n._effective_enabled())
check("during hold pub_state last = False", n.pub_state.last is not None and not n.pub_state.last.data)


# ===============================================================================
print("\n=== TEST 7: GNSS recovery hold expiry restores cmd_vel ===")
print("  After hold_s=2.0 s, _effective_enabled() should be True")
n._clock.advance_s(2.1)   # Hold expired.
check("after hold expiry _effective_enabled() = True", n._effective_enabled())


# ===============================================================================
print("\n=== TEST 8: e-stop blocks cmd_vel ===")
print("  platform/status or state_machine estop blocks regardless of engage state")
n = make_gate()
n._enabled = True
n._on_estop(BoolMsg(data=True))
check("estop=True -> _effective_enabled() = False", not n._effective_enabled())
n._on_estop(BoolMsg(data=False))
check("estop=False -> _effective_enabled() = True", n._effective_enabled())
n._on_estop(BoolMsg(data=True), "/planning/state_machine/estop")
check("soft estop=True -> _effective_enabled() = False", not n._effective_enabled())
n._on_estop(BoolMsg(data=True), "/platform/status/estop")
n._on_estop(BoolMsg(data=False), "/planning/state_machine/estop")
check("platform estop held -> _effective_enabled() = False", not n._effective_enabled())
n._on_estop(BoolMsg(data=False), "/platform/status/estop")
check("all estops clear -> _effective_enabled() = True", n._effective_enabled())


# ===============================================================================
print("\n=== TEST 9: engage=False blocks cmd_vel ===")
print("  /planning/engage=False prevents cmd_vel pass-through")
n = make_gate()
n._on_engage(BoolMsg(data=False))
check("engage=False -> _effective_enabled() = False", not n._effective_enabled())
n._on_engage(BoolMsg(data=True))
check("engage=True -> _effective_enabled() = True", n._effective_enabled())
n._on_mission_engage(BoolMsg(data=True))
n._on_engage(BoolMsg(data=False))
check("manual engage off while mission engage true -> _effective_enabled() = True", n._effective_enabled())
n._on_mission_engage(BoolMsg(data=False))
check("both manual and mission engage false -> _effective_enabled() = False", not n._effective_enabled())


# ===============================================================================
print("\n=== TEST 10: cost_stop_hold expiry restores pass-through ===")
print("  Hold is enforced by _effective_enabled(), not _should_stop_for_cost().")
print("  Obstacle detection sets _cost_blocked_until")
print("  After obstacle clears, _should_stop_for_cost()=False but _on_cmd is held")
print("  After hold expiry, _effective_enabled()=True and cmd_vel passes")
n = make_gate(hold_s=1.0)
n._enabled = True
t0_ns = int(200 * 1e9)
n._clock.set_ns(t0_ns)
# Grid with obstacle.
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=90)
stop = n._should_stop_for_cost()
check("obstacle detected -> stop (_should_stop_for_cost=True)", stop)
check("_cost_blocked_until is set", n._cost_blocked_until > 0.0)
# Obstacle removed: _should_stop_for_cost is false, hold keeps _effective_enabled false.
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=0)
stop_grid = n._should_stop_for_cost()
hold_blocks = not n._effective_enabled()   # Hold is still active.
check("after obstacle clear _should_stop_for_cost = False", not stop_grid)
check("during hold after obstacle clear -> _effective_enabled = False", hold_blocks)
# _on_cmd level: hold publishes zero even after cost-stop clears.
test_twist = Twist(); test_twist.linear.x = 0.5
n._on_cmd(test_twist)
check("during hold _on_cmd -> pub_cmd = 0", abs(n.pub_cmd.last.linear.x) < 0.01)
# Hold expiry.
n._clock.advance_s(1.1)
n._on_cmd(test_twist)
check("after hold expiry _on_cmd -> cmd_vel passes", abs(n.pub_cmd.last.linear.x - 0.5) < 0.01)


# ===============================================================================
print("\n=== TEST 11: speed-dependent front lookahead ===")
print("  v=1.0 m/s checks braking+reaction+margin calculation")
n = make_gate()
n.enable_speed_dependent_lookahead = True
n._current_speed = 0.0
la_stop = n._compute_front_lookahead()
n._current_speed = 1.0
la_1ms = n._compute_front_lookahead()
n._current_speed = 3.0
la_3ms = n._compute_front_lookahead()
check("v=0 -> min lookahead(0.4m)", abs(la_stop - 0.4) < 0.05)
check("v=1m/s -> lookahead > 0.4m", la_1ms > 0.4)
check("v=3m/s -> lookahead <= max(3.0m)", la_3ms <= 3.0)
check("speed increase -> lookahead increases", la_3ms >= la_1ms >= la_stop)


# ===============================================================================
print("\n=== TEST 12: lanelet FRONT_PATH route re-entry bypass ===")
print("  Site/drop-zone re-entry bypasses only static cost near the route")
n = make_gate()
n._enabled = True
n._last_route_heading_path = make_path([(0.0, 0.0), (2.0, 0.0)])
cmd = Twist(); cmd.linear.x = 0.2
near_path_detail = (0.5, 0.0, 100, "path_cost")
far_path_detail = (5.5, 0.0, 100, "path_cost")
allow = n._can_bypass_lanelet_front_path_for_route_reentry(
    cmd,
    "map",
    (0.0, 0.0, 0.0),
    near_path_detail,
    "pose_raw",
    300.0,
)
deny_far = n._can_bypass_lanelet_front_path_for_route_reentry(
    cmd,
    "map",
    (0.0, 0.0, 0.0),
    far_path_detail,
    "pose_raw",
    301.0,
)
cmd_reverse = Twist(); cmd_reverse.linear.x = -0.2
deny_reverse = n._can_bypass_lanelet_front_path_for_route_reentry(
    cmd_reverse,
    "map",
    (0.0, 0.0, 0.0),
    near_path_detail,
    "pose_raw",
    302.0,
)
check("near-route FRONT_PATH static cost -> bypass allowed", allow)
check("FRONT_PATH static cost outside lookahead -> bypass denied", not deny_far)
check("reverse/non-front command -> FRONT_PATH bypass denied", not deny_reverse)


# ===============================================================================
print("\n=== TEST 13: crab cmd_vel checks only the travel-direction corridor ===")
print("  During crab-left, body-front obstacles pass but left-travel obstacles stop")
n = make_gate(enable_side_rear=True)
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
crab_left = Twist(); crab_left.linear.y = 0.2
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=90)
front_stop_during_crab = n._should_stop_for_cost(crab_left)
check("crab-left body-front obstacle -> pass", not front_stop_during_crab)
n._last_grid = make_grid(obstacle_x=0.0, obstacle_y=0.8, obstacle_cost=90)
n._cost_source_grids = {"radar": n._last_grid}
n._cost_source_recv_sec = {"radar": 0.0}
left_stop_during_crab = n._should_stop_for_cost(crab_left)
check("crab-left left obstacle -> stop", left_stop_during_crab)

n = make_gate(enable_side_rear=True)
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
crab_right = Twist(); crab_right.linear.y = -0.2
n._last_grid = make_grid(obstacle_x=0.0, obstacle_y=-0.8, obstacle_cost=90)
n._cost_source_grids = {"radar": n._last_grid}
n._cost_source_recv_sec = {"radar": 0.0}
right_stop_during_crab = n._should_stop_for_cost(crab_right)
check("crab-right dynamic radar obstacle blocks", right_stop_during_crab)


# ===============================================================================
print("\n=== TEST 14: reverse cmd_vel uses rear corridor as travel-front ===")
print("  During reverse, body-front obstacles pass but rear obstacles stop")
n = make_gate(enable_side_rear=True)
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
reverse = Twist(); reverse.linear.x = -0.2
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=90)
front_stop_during_reverse = n._should_stop_for_cost(reverse)
check("reverse body-front obstacle -> pass", not front_stop_during_reverse)
n._last_grid = make_grid(obstacle_x=-0.6, obstacle_y=0.0, obstacle_cost=90)
n._cost_source_grids = {"radar": n._last_grid}
n._cost_source_recv_sec = {"radar": 0.0}
rear_stop_during_reverse = n._should_stop_for_cost(reverse)
check("reverse rear obstacle -> stop", rear_stop_during_reverse)


# ===============================================================================
print("\n=== TEST 15: clear avoidance local path can pass body-front obstacle ===")
print("  If local-path corridor avoids a front obstacle, cmd_vel gate can pass")
n = make_gate(enable_side_rear=True)
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
forward = Twist(); forward.linear.x = 0.2
n._last_route_heading_path = make_path([(0.0, 0.0), (0.5, 0.8), (2.0, 0.8)])
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=90)
n._cost_source_grids = {"lidar": n._last_grid}
n._cost_source_recv_sec = {"lidar": 0.0}
avoid_path_clear = n._should_stop_for_cost(forward)
check("avoidance local-path corridor clear -> body-front obstacle passes", not avoid_path_clear)

n = make_gate(enable_side_rear=True)
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
n._last_route_heading_path = make_path([(0.0, 0.0), (2.0, 0.0)])
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=90)
n._cost_source_grids = {"lidar": n._last_grid}
n._cost_source_recv_sec = {"lidar": 0.0}
path_blocked = n._should_stop_for_cost(forward)
check("obstacle on local-path corridor -> stop", path_blocked)


# ===============================================================================
print("\n=== TEST 16: dynamic cost-stop latch requires continuous clear ===")
print("  Once a dynamic stop is triggered, a brief clear frame must not release motion")
n = make_gate(enable_side_rear=True, enable_cost_latch=True)
n._enabled = True
n.cost_stop_clear_required_s = 2.0
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=90)
latched_stop = n._should_stop_for_cost(forward)
check("dynamic obstacle triggers latch", latched_stop and n._cost_stop_latched)
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=0)
still_latched = n._should_stop_for_cost(forward)
check("first clear frame remains blocked", still_latched)
n._clock.advance_s(2.1)
released = n._should_stop_for_cost(forward)
check("continuous clear window releases latch", not released and not n._cost_stop_latched)


# ===============================================================================
print("\n=== TEST 17: stale merged cost grid fail-safe ===")
print("  Missing or stale merged inflation grid blocks cmd_vel before diagnostics")
n = make_gate(enable_stale_stop=True)
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
missing_grid_stop = n._should_stop_for_cost(forward)
check("missing merged cost grid -> stop", missing_grid_stop)
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=0)
n._last_grid_recv_sec = n._clock.now().nanoseconds * 1e-9
fresh_grid_clear = n._should_stop_for_cost(forward)
check("fresh clear merged cost grid -> pass", not fresh_grid_clear)
n._clock.advance_s(1.2)
stale_grid_stop = n._should_stop_for_cost(forward)
check("stale merged cost grid -> stop", stale_grid_stop)


# ===============================================================================
total = len(results)
passed = sum(results)
print(f"\n{'='*62}")
print(f"  Result: {passed}/{total} passed  {'PASS' if passed == total else 'FAIL'}")
print(f"{'='*62}\n")
import sys as _sys
_sys.exit(0 if passed == total else 1)
