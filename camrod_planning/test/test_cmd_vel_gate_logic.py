#!/usr/bin/env python3
"""
planning_cmd_vel_gate_node 핵심 로직 유닛테스트.
ROS2 없이 실행 가능 (rclpy, tf2_ros 등 stub 처리).

테스트 항목:
  1. cost-stop 전방 코리더 — threshold 초과 셀 존재 시 정지
  2. cost-stop 임계값 미달 — 통과 허용
  3. cost-stop 측면(좌/우) 코리더 정지
  4. cost-stop 후방 코리더 정지
  5. unavoidable cluster 감지 — lethal 셀 집단이 코리더의 25% 이상이면 정지
  6. GNSS recovery hold — DR_ONLY → NORMAL 전환 시 gnss_recovery_hold_s 동안 cmd_vel 차단
  7. recovery hold 만료 — hold_s 경과 후 _effective_enabled() = True
  8. e-stop 상태에서 cmd_vel 차단
  9. engage = False 상태에서 cmd_vel 차단
 10. cost_stop_hold 만료 후 통과 재개

=== 동작 원리 ===

[ Cost-Stop ]
로봇 전방/측면/후방에 OccupancyGrid 기반 코리더를 설정하고
각 코리더 내 셀의 cost 값을 스캔합니다.

  cost_stop_threshold (기본 85):
    - OccupancyGrid 셀 값이 이 이상이면 즉시 정지
    - 전방: 속도 기반 가변 lookahead (0.4 ~ 3.0 m)
      braking_dist = v²/(2·g·μ) + reaction_time·v + margin
    - 측면: 좌우 각 1.2 m lookahead, 0.6 m 폭
    - 후방: 0.8 m lookahead, 0.9 m 폭

  unavoidable_lethal_threshold (기본 90):
    - threshold보다 높지 않더라도 코리더의 25% 이상을 덮는
      연결된 lethal 셀 집단이 있으면 정지 (피할 수 없는 장애물)

  cost_stop_hold_s (기본 1.0):
    - 정지 조건이 사라진 후에도 1초간 zero cmd_vel 유지

[ GNSS Recovery Hold ]
ESKF 로컬라이제이션이 DR_ONLY(2) → NORMAL(0)로 복구될 때:
  1. Nav2와 costmap이 복구된 위치 기반으로 재계산하는 데 시간이 필요
  2. gate 노드가 gnss_recovery_hold_s(기본 2.0s) 동안 cmd_vel을 0으로 강제
  3. hold 만료 후 cmd_vel 정상 통과 재개

  /localization/mode 토픽 값:
    NORMAL=0, DEGRADED=1, DR_ONLY=2, INVALID=3
"""

from __future__ import annotations

import math
import sys
import time
import types

# ─── ROS2 stub (rclpy, tf2_ros, nav_msgs 등) ─────────────────────────────────

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


# ─── msg stubs ────────────────────────────────────────────────────────────────

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


# ─── patch sys.modules ────────────────────────────────────────────────────────

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
    "avg_msgs.msg":                 _mod(AvgLocalizationMode=AvgLocalizationMode),
})
for top in ["rcl_interfaces", "geometry_msgs", "nav_msgs", "std_msgs", "avg_msgs"]:
    if top not in sys.modules:
        sys.modules[top] = types.ModuleType(top)

# ─── import target ────────────────────────────────────────────────────────────

sys.path.insert(0, "/home/hong/camrod_ws/src/camrod_planning/scripts")
import planning_cmd_vel_gate_node as gmod


# ─── factory ──────────────────────────────────────────────────────────────────

def make_gate(
    cost_threshold: int = 85,
    hold_s: float = 1.0,
    recovery_hold_s: float = 2.0,
    enable_side_rear: bool = True,
    enable_unavoidable: bool = True,
) -> gmod.PlanningCmdVelGateNode:
    """노드 인스턴스를 ROS2 없이 직접 생성합니다."""
    n = gmod.PlanningCmdVelGateNode.__new__(gmod.PlanningCmdVelGateNode)

    # ── ROS2 인터페이스 stub ──
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

    # ── 파라미터 ──
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
    n.gnss_recovery_source_mode_min = int(AvgLocalizationMode.DR_ONLY)
    n.gnss_recovery_target_mode = int(AvgLocalizationMode.NORMAL)

    n.enable_cost_stop = True
    n.cost_grid_topic = "/planning/cost_grid/inflation"
    # HHL_260623 - Unit tests exercise merged cost-stop only; lanelet safety
    # needs a full map grid fixture and is disabled in this lightweight stub.
    n.lanelet_safety_enable = False
    n.lanelet_safety_allow_rotation_in_place = True
    n.lanelet_safety_min_translation_mps = 0.02
    # HHL_260624 - Keep route re-entry defaults in the lightweight unit stub.
    n.lanelet_safety_lookahead_m = 1.0
    n.lanelet_safety_front_path_allow_route_reentry = True
    n.lanelet_safety_current_allow_route_reentry = True
    n.lanelet_safety_current_route_reentry_max_distance_m = 4.0
    n.lanelet_safety_current_route_reentry_require_front_cmd = True
    n.pose_topic = "/localization/pose"
    n.odometry_topic = "/localization/fallback/odometry"
    n.pose_source_preference = "odometry"
    n.enable_pose_raw_fallback = True  # TF 없이 raw fallback 사용
    n.robot_base_frame = "robot_base_link"
    n.cost_stop_threshold = cost_threshold
    n.cost_stop_lookahead_m = 2.0
    n.cost_stop_width_m = 1.0
    n.cost_stop_hold_s = hold_s
    n.cost_source_debug_enable = False
    n.cost_source_debug_max_age_s = 1.0
    n.cost_stop_require_dynamic_source = False
    n.cost_stop_dynamic_source_labels = {"lidar", "radar"}
    n.lateral_cmd_bypass_static_cost_stop = True
    n.lateral_cmd_bypass_min_mps = 0.02
    n.reverse_cmd_bypass_static_cost_stop = True
    n.reverse_cmd_bypass_min_mps = 0.02

    n.enable_speed_dependent_lookahead = False  # 고정 lookahead 사용
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

    # ── 내부 상태 ──
    # HHL_260623 - Manual and UI-mission engage are independent OR latches.
    n._manual_enabled = False
    n._mission_enabled = False
    n._enabled = False
    n._estop = False
    n._dr_timeout = False
    n._cost_blocked_until = 0.0
    n._gnss_recovery_blocked_until = 0.0
    n._last_localization_mode_value = None
    n._last_unavoidable_cluster_cells = 0
    n._last_unavoidable_cluster_ratio = 0.0
    n._last_tf_warn_sec = 0.0
    n._last_empty_corridor_warn_sec = 0.0
    n._last_lanelet_front_path_reentry_bypass_log_sec = 0.0
    n._last_block_reason_log_sec = 0.0
    n._current_speed = 0.0
    n._last_grid = None
    n._last_route_heading_path = None
    n._last_pose = None
    n._last_odom = None

    return n


# ─── 그리드 빌더 ──────────────────────────────────────────────────────────────

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
    로봇 위치를 중심으로 하는 OccupancyGrid를 생성합니다.
    obstacle_x/y 위치에 obstacle_cost를 넣고 나머지는 0입니다.

    코리더 샘플링은 grid 원점(origin)과 resolution 기준으로 셀 인덱스를 계산하므로
    로봇 pose와 grid origin이 일치해야 장애물 위치가 정확히 투영됩니다.
    """
    cells = int(grid_size_m / resolution)
    half = grid_size_m / 2.0
    grid = OccupancyGrid()
    grid.header.frame_id = frame_id
    grid.info.resolution = resolution
    grid.info.width = cells
    grid.info.height = cells
    # origin: 그리드의 좌하단 (로봇 위치에서 half씩 빼서 중심 맞춤)
    grid.info.origin.position.x = robot_x - half
    grid.info.origin.position.y = robot_y - half
    grid.data = [0] * (cells * cells)
    # 장애물 셀 설정
    ox = int((obstacle_x - grid.info.origin.position.x) / resolution)
    oy = int((obstacle_y - grid.info.origin.position.y) / resolution)
    if 0 <= ox < cells and 0 <= oy < cells:
        grid.data[oy * cells + ox] = obstacle_cost
    return grid


def make_odom(x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> Odometry:
    """로봇 자기 위치 odometry 메시지를 생성합니다."""
    odom = Odometry()
    odom.header.frame_id = "map"
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.orientation.w = math.cos(yaw * 0.5)
    odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
    return odom


def make_path(points: list[tuple[float, float]], frame_id: str = "map") -> Path:
    """경로 거리 계산 테스트용 Path 메시지를 생성합니다."""
    path = Path()
    path.header.frame_id = frame_id
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        path.poses.append(pose)
    return path


# ─── 테스트 실행 ──────────────────────────────────────────────────────────────

results: list[bool] = []


def check(label: str, cond: bool) -> None:
    tag = "✅" if cond else "❌"
    print(f"  {tag}  {label}")
    results.append(cond)


# ═══════════════════════════════════════════════════════════════════════════════
print("\n=== TEST 1: 전방 cost-stop — threshold 초과 장애물 → 정지 ===")
print("  로봇 전방 1.5m에 cost=90 장애물 배치 → lookahead=2.0m 내 감지됨")
n = make_gate()
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)   # 로봇: (0,0) 방향 동쪽
n._last_grid = make_grid(
    robot_x=0.0, robot_y=0.0,
    obstacle_x=1.5, obstacle_y=0.0,  # 전방 1.5m
    obstacle_cost=90,
)
stop = n._should_stop_for_cost()
check("threshold=90 전방 장애물 → 정지", stop)
check("cost_blocked_until 세팅됨", n._cost_blocked_until > 0.0)


# ═══════════════════════════════════════════════════════════════════════════════
print("\n=== TEST 2: cost 미달 — threshold 미만 장애물 → 통과 허용 ===")
print("  cost=60 (threshold=85 미만) → 정지 안 함")
n = make_gate(cost_threshold=85)
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=60)
stop = n._should_stop_for_cost()
check("cost=60 < threshold=85 → 통과", not stop)
check("cost_blocked_until = 0 (설정 안 됨)", n._cost_blocked_until == 0.0)


# ═══════════════════════════════════════════════════════════════════════════════
print("\n=== TEST 3: 측면(왼쪽) cost-stop ===")
print("  로봇 왼쪽 0.8m에 cost=90 → 좌측 코리더(lookahead=1.2m) 감지")
n = make_gate(enable_side_rear=True)
n._enabled = True
# 로봇이 동쪽(yaw=0) 향함, 왼쪽 = 북쪽(y+)
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
n._last_grid = make_grid(
    robot_x=0.0, robot_y=0.0,
    obstacle_x=0.0, obstacle_y=0.8,  # 왼쪽 0.8m
    obstacle_cost=90,
)
stop = n._should_stop_for_cost()
check("왼쪽 0.8m 장애물 → 측면 정지", stop)


# ═══════════════════════════════════════════════════════════════════════════════
print("\n=== TEST 4: 후방 cost-stop ===")
print("  로봇 뒤쪽 0.6m에 cost=90 → 후방 코리더(lookahead=0.8m) 감지")
n = make_gate(enable_side_rear=True)
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
n._last_grid = make_grid(
    robot_x=0.0, robot_y=0.0,
    obstacle_x=-0.6, obstacle_y=0.0,  # 후방 0.6m
    obstacle_cost=90,
)
stop = n._should_stop_for_cost()
check("뒤쪽 0.6m 장애물 → 후방 정지", stop)


# ═══════════════════════════════════════════════════════════════════════════════
print("\n=== TEST 5: unavoidable cluster 감지 ===")
print("  [설계] stop_threshold > unavoidable_lethal_threshold 조합일 때 작동:")
print("  stop_threshold=95, unavoidable_lethal=90, cost=91")
print("  → 단일 셀이 95 미만이므로 direct stop 안 됨")
print("  → 91≥90이므로 lethal_cells에 수집 → 집단이 25% 이상 → unavoidable 정지")

def make_cluster_grid(
    robot_x: float = 0.0,
    cluster_x_start: float = 0.5,
    cluster_width: float = 1.5,   # 코리더 폭(1.0m) 전체를 덮도록
    cluster_depth: float = 1.0,   # lookahead 방향 깊이
    cost: int = 91,
    resolution: float = 0.1,
) -> OccupancyGrid:
    """
    전방 코리더를 넓게 덮는 클러스터 그리드를 생성합니다.
    cost는 direct stop_threshold 미만, unavoidable_lethal_threshold 이상이어야
    unavoidable cluster 경로를 통해 정지 판정이 이뤄집니다.
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

# stop_threshold을 95로 올려서 91짜리 셀이 direct stop을 우회하도록 설정.
# unavoidable_lethal_threshold=90이므로 91 셀은 lethal로 수집됨.
n = make_gate(cost_threshold=95, enable_unavoidable=True)
n.unavoidable_lethal_threshold = 90
n.unavoidable_cluster_min_cells = 25
n.unavoidable_cluster_min_ratio = 0.25
n._enabled = True
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
n._last_grid = make_cluster_grid(cost=91)  # 95 미만(direct skip), 90 이상(lethal 수집)
stop = n._should_stop_for_cost()
check("대형 lethal 클러스터 → unavoidable 정지", stop)
check("클러스터 셀 수 >= 25", n._last_unavoidable_cluster_cells >= 25)
check("클러스터 비율 >= 0.25", n._last_unavoidable_cluster_ratio >= 0.25)


# ═══════════════════════════════════════════════════════════════════════════════
print("\n=== TEST 6: GNSS recovery hold 발동 ===")
print("  DR_ONLY → NORMAL 모드 전환 → gnss_recovery_hold_s 동안 cmd_vel 차단")
n = make_gate(recovery_hold_s=2.0)
n._enabled = True
t0_ns = int(100 * 1e9)   # 100초 시점
n._clock.set_ns(t0_ns)

# DR_ONLY 수신 (이전 상태로 등록)
mode_dr = AvgLocalizationMode(AvgLocalizationMode.DR_ONLY)
n._on_localization_mode(mode_dr)
check("DR_ONLY 수신 후 아직 hold 없음", n._gnss_recovery_blocked_until == 0.0)

# NORMAL 수신 → hold 발동
mode_normal = AvgLocalizationMode(AvgLocalizationMode.NORMAL)
n._on_localization_mode(mode_normal)
hold_until = n._gnss_recovery_blocked_until
check("NORMAL 수신 → hold 설정됨", hold_until > 0.0)
check(f"hold = now+2.0s ({hold_until:.1f}s)", abs(hold_until - (t0_ns * 1e-9 + 2.0)) < 0.1)
check("hold 중 _effective_enabled() = False", not n._effective_enabled())
check("hold 중 pub_state last = False", n.pub_state.last is not None and not n.pub_state.last.data)


# ═══════════════════════════════════════════════════════════════════════════════
print("\n=== TEST 7: GNSS recovery hold 만료 → cmd_vel 재개 ===")
print("  hold_s(2.0s) 경과 후 _effective_enabled() = True")
n._clock.advance_s(2.1)   # hold 만료
check("hold 만료 후 _effective_enabled() = True", n._effective_enabled())


# ═══════════════════════════════════════════════════════════════════════════════
print("\n=== TEST 8: e-stop → cmd_vel 차단 ===")
print("  /platform/status/estop = True → engage 상태와 무관하게 차단")
n = make_gate()
n._enabled = True
n._on_estop(BoolMsg(data=True))
check("estop=True → _effective_enabled() = False", not n._effective_enabled())
n._on_estop(BoolMsg(data=False))
check("estop=False → _effective_enabled() = True", n._effective_enabled())


# ═══════════════════════════════════════════════════════════════════════════════
print("\n=== TEST 9: engage = False → cmd_vel 차단 ===")
print("  /planning/engage = False → cmd_vel 통과 안 됨")
n = make_gate()
n._on_engage(BoolMsg(data=False))
check("engage=False → _effective_enabled() = False", not n._effective_enabled())
n._on_engage(BoolMsg(data=True))
check("engage=True → _effective_enabled() = True", n._effective_enabled())
n._on_mission_engage(BoolMsg(data=True))
n._on_engage(BoolMsg(data=False))
check("manual engage off while mission engage true → _effective_enabled() = True", n._effective_enabled())
n._on_mission_engage(BoolMsg(data=False))
check("both manual and mission engage false → _effective_enabled() = False", not n._effective_enabled())


# ═══════════════════════════════════════════════════════════════════════════════
print("\n=== TEST 10: cost_stop_hold 만료 후 통과 재개 ===")
print("  [설계] hold는 _should_stop_for_cost()가 아닌 _effective_enabled()에서 검사.")
print("  장애물 감지(직접 정지) → _cost_blocked_until 세팅")
print("  장애물 제거 후: _should_stop_for_cost()=False, 하지만 _on_cmd에서 _effective_enabled()=False(hold)")
print("  hold 만료 후: _effective_enabled()=True → cmd_vel 통과")
n = make_gate(hold_s=1.0)
n._enabled = True
t0_ns = int(200 * 1e9)
n._clock.set_ns(t0_ns)
# 장애물 있는 그리드
n._last_odom = make_odom(x=0.0, y=0.0, yaw=0.0)
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=90)
stop = n._should_stop_for_cost()
check("장애물 감지 → 정지 (_should_stop_for_cost=True)", stop)
check("_cost_blocked_until 세팅됨", n._cost_blocked_until > 0.0)
# 장애물 제거: _should_stop_for_cost는 False, 하지만 hold 때문에 _effective_enabled=False
n._last_grid = make_grid(obstacle_x=1.0, obstacle_y=0.0, obstacle_cost=0)
stop_grid = n._should_stop_for_cost()
hold_blocks = not n._effective_enabled()   # hold가 여전히 걸려 있음
check("장애물 제거 후 _should_stop_for_cost = False (그리드 클린)", not stop_grid)
check("장애물 제거 후 hold 중 → _effective_enabled = False", hold_blocks)
# _on_cmd 레벨: cost-stop 없어도 hold 때문에 zero 전달
test_twist = Twist(); test_twist.linear.x = 0.5
n._on_cmd(test_twist)
check("hold 중 _on_cmd → pub_cmd = 0 (hold 적용)", abs(n.pub_cmd.last.linear.x) < 0.01)
# hold 만료
n._clock.advance_s(1.1)
n._on_cmd(test_twist)
check("hold 만료 후 _on_cmd → cmd_vel 통과", abs(n.pub_cmd.last.linear.x - 0.5) < 0.01)


# ═══════════════════════════════════════════════════════════════════════════════
print("\n=== TEST 11: 속도 기반 전방 lookahead 계산 ===")
print("  v=1.0m/s → braking+reaction+margin 계산 확인")
n = make_gate()
n.enable_speed_dependent_lookahead = True
n._current_speed = 0.0
la_stop = n._compute_front_lookahead()
n._current_speed = 1.0
la_1ms = n._compute_front_lookahead()
n._current_speed = 3.0
la_3ms = n._compute_front_lookahead()
check("v=0 → min lookahead(0.4m)", abs(la_stop - 0.4) < 0.05)
check("v=1m/s → lookahead > 0.4m", la_1ms > 0.4)
check("v=3m/s → lookahead <= max(3.0m)", la_3ms <= 3.0)
check("속도 증가 → lookahead 증가", la_3ms >= la_1ms >= la_stop)


# ═══════════════════════════════════════════════════════════════════════════════
print("\n=== TEST 12: lanelet FRONT_PATH route re-entry bypass ===")
print("  사이트/드랍존에서 lanelet으로 재진입할 때 경로 근처 정적 cost만 우회")
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
check("경로 근처 FRONT_PATH static cost → 우회 허용", allow)
check("lookahead 밖 FRONT_PATH static cost → 우회 금지", not deny_far)
check("후진/비전방 cmd → FRONT_PATH 우회 금지", not deny_reverse)


# ═══════════════════════════════════════════════════════════════════════════════
total = len(results)
passed = sum(results)
print(f"\n{'═'*62}")
print(f"  결과: {passed}/{total} 통과  {'✅ ALL PASS' if passed == total else '❌ SOME FAILED'}")
print(f"{'═'*62}\n")
import sys as _sys
_sys.exit(0 if passed == total else 1)
