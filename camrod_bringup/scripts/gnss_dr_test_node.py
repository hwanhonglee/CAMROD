#!/usr/bin/env python3
# HH_260422: GNSS DR fallback integration test node.
# Publishes camping_site goal + engage, then validates that cmd_vel continues
# through a simulated GNSS failure (DR_ONLY mode) and recovers correctly.
#
# Pipeline under test:
#   fake_sensor(GNSS off) → ESKF DR(IMU+Wheel) → pose_selector(primary)
#   → nav2 → cmd_vel_raw → planning_gate(engage=true) → cmd_vel
#   → platform_gate → /platform/cmd_vel → fake_sensor(motion)

import math
import time

import rclpy
from avg_msgs.msg import AvgLocalizationMode
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool


# Camping site 1 target pose (from camrod_bringup/config/planning/camping_sites.yaml).
GOAL_X = -13.1858
GOAL_Y = -93.0608
GOAL_YAW_DEG = -179.992

# Drop zone start reference (from camrod_bringup/config/map/drop_zones.yaml).
START_X = 16.8676
START_Y = -117.408


def _yaw_to_quat(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


class GnssDrTestNode(Node):
    # Phase constants.
    PHASE_WAIT_READY = "WAIT_READY"
    PHASE_SEND_GOAL = "SEND_GOAL"
    PHASE_WAIT_PATH = "WAIT_PATH"
    PHASE_ENGAGE = "ENGAGE"
    PHASE_DRIVING_GNSS = "DRIVING_GNSS"
    PHASE_DRIVING_DR = "DRIVING_DR"
    PHASE_DRIVING_RECOVERED = "DRIVING_RECOVERED"
    PHASE_DONE = "DONE"

    def __init__(self) -> None:
        super().__init__("gnss_dr_test")

        self.goal_x = float(self.declare_parameter("goal_x", GOAL_X).value)
        self.goal_y = float(self.declare_parameter("goal_y", GOAL_Y).value)
        self.goal_yaw_deg = float(self.declare_parameter("goal_yaw_deg", GOAL_YAW_DEG).value)
        # How long to wait from node start before publishing goal (allow bringup to settle).
        self.goal_delay_s = float(self.declare_parameter("goal_delay_s", 13.0).value)
        goal_delay_sec_legacy = float(self.declare_parameter("goal_delay_sec", 13.0).value)
        if self.goal_delay_s == 13.0 and goal_delay_sec_legacy != 13.0:
            self.get_logger().warn(
                "Parameter 'goal_delay_sec' is deprecated. Use 'goal_delay_s'."
            )
            self.goal_delay_s = goal_delay_sec_legacy
        # How long after goal to wait before publishing engage.
        self.engage_delay_s = float(self.declare_parameter("engage_delay_s", 3.0).value)
        engage_delay_sec_legacy = float(self.declare_parameter("engage_delay_sec", 3.0).value)
        if self.engage_delay_s == 3.0 and engage_delay_sec_legacy != 3.0:
            self.get_logger().warn(
                "Parameter 'engage_delay_sec' is deprecated. Use 'engage_delay_s'."
            )
            self.engage_delay_s = engage_delay_sec_legacy
        # Total test duration.
        self.test_duration_s = float(self.declare_parameter("test_duration_s", 85.0).value)
        test_duration_sec_legacy = float(self.declare_parameter("test_duration_sec", 85.0).value)
        if self.test_duration_s == 85.0 and test_duration_sec_legacy != 85.0:
            self.get_logger().warn(
                "Parameter 'test_duration_sec' is deprecated. Use 'test_duration_s'."
            )
            self.test_duration_s = test_duration_sec_legacy
        # cmd_vel rate threshold to consider "active" (Hz).
        self.cmd_vel_active_hz = float(self.declare_parameter("cmd_vel_active_hz", 5.0).value)

        self._phase = self.PHASE_WAIT_READY
        self._t_start = time.monotonic()
        self._t_goal_sent: float | None = None
        self._t_engage_sent: float | None = None

        # Localization state.
        self._localization_ready = False
        self._current_mode_value = -1
        self._current_mode_label = "UNKNOWN"
        self._mode_history: list[tuple[float, str]] = []
        # HH_260422: Guard DR detection — only count DR_ONLY as "GNSS failure" after
        # the system has first reached NORMAL mode (avoids false-positive at cold start
        # when ESKF is still initializing and has not yet accepted its first GNSS update).
        self._saw_normal_mode = False

        # Position tracking.
        self._last_x: float | None = None
        self._last_y: float | None = None
        self._positions_during_dr: list[tuple[float, float, float]] = []

        # cmd_vel tracking.
        self._cmd_raw_count = 0
        self._cmd_raw_last_t: float | None = None
        self._cmd_vel_count = 0
        self._cmd_vel_last_t: float | None = None
        self._cmd_vel_during_dr: list[tuple[float, float]] = []   # (t, linear.x)
        self._cmd_vel_zero_during_dr = 0
        self._cmd_vel_nonzero_during_dr = 0

        # Phase timing.
        self._t_gnss_failed: float | None = None
        self._t_gnss_recovered: float | None = None

        # Publishers.
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.pub_engage = self.create_publisher(Bool, "/planning/engage", latched_qos)

        # Subscribers.
        self.create_subscription(
            AvgLocalizationMode, "/localization/mode", self._on_mode, 10
        )
        self.create_subscription(
            PoseStamped, "/localization/pose", self._on_pose, 10
        )
        self.create_subscription(
            Twist, "/planning/cmd_vel_raw", self._on_cmd_raw, 10
        )
        self.create_subscription(
            Twist, "/planning/cmd_vel", self._on_cmd_vel, 10
        )

        self.timer = self.create_timer(0.2, self._on_tick)
        self.get_logger().info(
            f"[DR TEST] started. goal=({self.goal_x:.2f},{self.goal_y:.2f}) "
            f"goal_delay={self.goal_delay_s}s engage_delay={self.engage_delay_s}s "
            f"duration={self.test_duration_s}s"
        )

    def _elapsed(self) -> float:
        return time.monotonic() - self._t_start

    def _on_mode(self, msg: AvgLocalizationMode) -> None:
        if msg.value == self._current_mode_value:
            return
        prev = self._current_mode_label
        self._current_mode_value = msg.value
        self._current_mode_label = msg.label if msg.label else str(msg.value)
        t = self._elapsed()
        self._mode_history.append((t, self._current_mode_label))
        self.get_logger().warn(
            f"[DR TEST] MODE CHANGE  t={t:.1f}s  {prev} → {self._current_mode_label}"
        )
        if msg.value == AvgLocalizationMode.NORMAL:
            self._saw_normal_mode = True
        if (msg.value == AvgLocalizationMode.DR_ONLY
                and self._saw_normal_mode
                and self._t_gnss_failed is None):
            self._t_gnss_failed = t
            self._phase = self.PHASE_DRIVING_DR
            self.get_logger().warn(f"[DR TEST] *** GNSS FAILURE DETECTED at t={t:.1f}s → DR mode ***")
        elif msg.value == AvgLocalizationMode.NORMAL and self._t_gnss_failed is not None and self._t_gnss_recovered is None:
            self._t_gnss_recovered = t
            self._phase = self.PHASE_DRIVING_RECOVERED
            dr_duration = t - self._t_gnss_failed
            self.get_logger().warn(f"[DR TEST] *** GNSS RECOVERED at t={t:.1f}s (DR lasted {dr_duration:.1f}s) ***")

    def _on_pose(self, msg: PoseStamped) -> None:
        self._localization_ready = True
        x = msg.pose.position.x
        y = msg.pose.position.y
        self._last_x = x
        self._last_y = y
        if self._phase == self.PHASE_DRIVING_DR:
            t = self._elapsed()
            self._positions_during_dr.append((t, x, y))

    def _on_cmd_raw(self, msg: Twist) -> None:
        self._cmd_raw_count += 1
        self._cmd_raw_last_t = self._elapsed()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel_count += 1
        t = self._elapsed()
        self._cmd_vel_last_t = t
        vx = msg.linear.x
        if self._phase == self.PHASE_DRIVING_DR:
            self._cmd_vel_during_dr.append((t, vx))
            if abs(vx) > 0.01:
                self._cmd_vel_nonzero_during_dr += 1
            else:
                self._cmd_vel_zero_during_dr += 1

    def _send_goal(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation = _yaw_to_quat(math.radians(self.goal_yaw_deg))
        self.pub_goal.publish(msg)
        self._t_goal_sent = self._elapsed()
        self.get_logger().info(
            f"[DR TEST] GOAL SENT  t={self._t_goal_sent:.1f}s → "
            f"camping_site({self.goal_x:.2f},{self.goal_y:.2f})"
        )
        self._phase = self.PHASE_WAIT_PATH

    def _send_engage(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self.pub_engage.publish(msg)
        self._t_engage_sent = self._elapsed()
        self.get_logger().info(
            f"[DR TEST] ENGAGE={'true' if enabled else 'false'}  t={self._t_engage_sent:.1f}s"
        )

    def _on_tick(self) -> None:
        t = self._elapsed()
        if self._phase == self.PHASE_DONE:
            return

        # Total test timeout.
        if t >= self.test_duration_s:
            self._finish_test()
            return

        if self._phase == self.PHASE_WAIT_READY:
            if self._localization_ready and t >= self.goal_delay_s:
                self._send_goal()

        elif self._phase == self.PHASE_WAIT_PATH:
            if self._cmd_raw_count > 0:
                self.get_logger().info(
                    f"[DR TEST] cmd_vel_raw APPEARED (nav2 planning active)  t={t:.1f}s"
                )
                self._phase = self.PHASE_ENGAGE
            elif self._t_goal_sent and (t - self._t_goal_sent) >= self.engage_delay_s:
                # Engage after delay even if cmd_vel_raw hasn't appeared yet.
                self._send_engage(True)
                self._phase = self.PHASE_DRIVING_GNSS

        elif self._phase == self.PHASE_ENGAGE:
            self._send_engage(True)
            self._phase = self.PHASE_DRIVING_GNSS

        elif self._phase in (self.PHASE_DRIVING_GNSS,):
            # Periodic heartbeat during normal driving.
            if int(t * 5) % 25 == 0:
                pos = f"({self._last_x:.1f},{self._last_y:.1f})" if self._last_x else "unknown"
                cmd_raw_rate = (self._cmd_raw_count / t) if t > 0 else 0
                cmd_rate = (self._cmd_vel_count / t) if t > 0 else 0
                self.get_logger().info(
                    f"[DR TEST] DRIVING(GNSS)  t={t:.0f}s  pos={pos}  "
                    f"cmd_raw={self._cmd_raw_count}({cmd_raw_rate:.1f}Hz)  "
                    f"cmd_vel={self._cmd_vel_count}({cmd_rate:.1f}Hz)  "
                    f"mode={self._current_mode_label}"
                )

    def _finish_test(self) -> None:
        self._phase = self.PHASE_DONE
        self._send_engage(False)
        t = self._elapsed()

        sep = "=" * 65
        self.get_logger().info(sep)
        self.get_logger().info("[DR TEST] RESULT SUMMARY")
        self.get_logger().info(sep)

        # --- Localization mode history ---
        self.get_logger().info(f"[DR TEST] Mode transitions ({len(self._mode_history)}):")
        for mt, ml in self._mode_history:
            self.get_logger().info(f"          t={mt:6.1f}s  {ml}")

        # --- cmd_vel pipeline ---
        cmd_raw_ok = self._cmd_raw_count > 0
        cmd_vel_ok = self._cmd_vel_count > 0
        self.get_logger().info(
            f"[DR TEST] cmd_vel_raw:  {'PUBLISHED' if cmd_raw_ok else 'NEVER SEEN'}  "
            f"({self._cmd_raw_count} msgs)"
        )
        self.get_logger().info(
            f"[DR TEST] cmd_vel:      {'PUBLISHED' if cmd_vel_ok else 'NEVER SEEN'}  "
            f"({self._cmd_vel_count} msgs)"
        )

        # --- DR window ---
        if self._t_gnss_failed is not None:
            dr_end = self._t_gnss_recovered if self._t_gnss_recovered else t
            dr_duration = dr_end - self._t_gnss_failed
            total_dr_samples = len(self._cmd_vel_during_dr)
            dr_continuity = (
                f"{self._cmd_vel_nonzero_during_dr}/{total_dr_samples} nonzero"
                if total_dr_samples > 0
                else "no cmd_vel samples"
            )
            self.get_logger().info(f"[DR TEST] DR window:    {self._t_gnss_failed:.1f}s → {dr_end:.1f}s  ({dr_duration:.1f}s)")
            self.get_logger().info(f"[DR TEST] cmd_vel DR:   {dr_continuity}")

            if self._positions_during_dr:
                x0, y0 = self._positions_during_dr[0][1], self._positions_during_dr[0][2]
                x1, y1 = self._positions_during_dr[-1][1], self._positions_during_dr[-1][2]
                dist = math.hypot(x1 - x0, y1 - y0)
                self.get_logger().info(
                    f"[DR TEST] DR position:  moved {dist:.2f}m during DR  "
                    f"({x0:.1f},{y0:.1f}) → ({x1:.1f},{y1:.1f})"
                )
        else:
            self.get_logger().warn("[DR TEST] DR window:    GNSS failure NOT detected in this run")

        if self._t_gnss_recovered is not None:
            self.get_logger().info(f"[DR TEST] GNSS recovery detected at t={self._t_gnss_recovered:.1f}s")
        elif self._t_gnss_failed is not None:
            self.get_logger().warn("[DR TEST] GNSS recovery NOT detected within test window")

        # --- PASS / FAIL ---
        self.get_logger().info(sep)
        checks = [
            ("cmd_vel_raw published (nav2 planning)", cmd_raw_ok),
            ("cmd_vel published (gate engaged)", cmd_vel_ok),
            ("GNSS failure detected (DR_ONLY mode)", self._t_gnss_failed is not None),
            (
                "cmd_vel continued during DR",
                self._cmd_vel_nonzero_during_dr > 0,
            ),
            ("GNSS recovery detected", self._t_gnss_recovered is not None),
        ]
        all_passed = True
        for label, result in checks:
            icon = "PASS" if result else "FAIL"
            self.get_logger().info(f"[DR TEST]  [{icon}]  {label}")
            if not result:
                all_passed = False
        self.get_logger().info(sep)
        self.get_logger().info(
            f"[DR TEST]  OVERALL: {'ALL PASS' if all_passed else 'SOME CHECKS FAILED'}"
        )
        self.get_logger().info(sep)
        self.timer.cancel()
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = GnssDrTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
