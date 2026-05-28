#!/usr/bin/env python3
"""Mock DockRobot Action Server — 통합 테스트용.

실제 opennav_docking 없이 /docking/dock_robot Action Server를 흉내낸다.
ManualDockServerNode와 UiBackendNode의 인터페이스를 검증하기 위한 모의 서버.

사용법:
    python3 mock_dock_robot_server.py --ros-args -p scenario:=success

시나리오:
    success      기본. Feedback 4단계 발행 후 success=true 반환
    fail         NAV_TO_STAGING 후 error_code=903(FAILED_TO_STAGE) 반환
    slow_accept  goal 수락을 12초 지연 (ManualDockServerNode 10초 타임아웃 검증)
    reject       goal 즉시 REJECT
"""

import time

import rclpy
from builtin_interfaces.msg import Duration
from opennav_docking_msgs.action import DockRobot
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node


# Feedback state 상수 (DockRobot.action과 동일)
NAV_TO_STAGING    = DockRobot.Feedback.NAV_TO_STAGING_POSE
INITIAL_PERC      = DockRobot.Feedback.INITIAL_PERCEPTION
CONTROLLING       = DockRobot.Feedback.CONTROLLING
WAIT_FOR_CHARGE   = DockRobot.Feedback.WAIT_FOR_CHARGE


def make_duration(sec: float) -> Duration:
    """float 초를 builtin_interfaces/Duration으로 변환."""
    msg = Duration()
    msg.sec = int(sec)
    msg.nanosec = int((sec - msg.sec) * 1e9)
    return msg


class MockDockRobotServer(Node):

    def __init__(self):
        super().__init__('mock_dock_robot_server')

        self.scenario = str(self.declare_parameter('scenario', 'success').value)
        self.get_logger().info(
            f"MockDockRobotServer ready — scenario='{self.scenario}' "
            f"| action=/docking/dock_robot"
        )

        self._server = ActionServer(
            self,
            DockRobot,
            '/docking/dock_robot',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
        )

    def _goal_callback(self, goal_request):
        if self.scenario == 'reject':
            self.get_logger().warn('[mock] REJECT goal (scenario=reject)')
            return GoalResponse.REJECT
        if self.scenario == 'slow_accept':
            # goal_accept_future는 이 콜백 응답 시점에 resolve된다.
            # 여기서 12초 지연 → ManualDockServerNode의 10초 타임아웃 트리거
            self.get_logger().warn('[mock] slow_accept: 12s delay before ACCEPT')
            time.sleep(12.0)
        self.get_logger().info(
            f"[mock] goal accepted (dock_id={goal_request.dock_id}, scenario={self.scenario})"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('[mock] cancel accepted')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        self.get_logger().info(f'[mock] execute start (scenario={self.scenario})')

        # ── success / fail 공통 시작 ─────────────────────────────────────────
        start = self.get_clock().now()

        def elapsed() -> float:
            return (self.get_clock().now() - start).nanoseconds * 1e-9

        def publish_feedback(state: int):
            fb = DockRobot.Feedback()
            fb.state = state
            fb.docking_time = make_duration(elapsed())
            fb.num_retries = 0
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f'[mock] Feedback state={state} elapsed={elapsed():.1f}s')

        # ── NAV_TO_STAGING (공통) ────────────────────────────────────────────
        for _ in range(4):  # 2초 동안 0.5초 간격
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return DockRobot.Result()
            publish_feedback(NAV_TO_STAGING)
            time.sleep(0.5)

        # ── fail 시나리오: staging 단계에서 실패 ─────────────────────────────
        if self.scenario == 'fail':
            self.get_logger().warn('[mock] FAIL result (scenario=fail, error_code=903)')
            result = DockRobot.Result()
            result.success = False
            result.error_code = DockRobot.Result.FAILED_TO_STAGE
            goal_handle.abort()
            return result

        # ── success 시나리오: 나머지 phase 진행 ─────────────────────────────
        phases = [
            (INITIAL_PERC,    3.0, 'INITIAL_PERCEPTION'),
            (CONTROLLING,     3.0, 'CONTROLLING'),
            (WAIT_FOR_CHARGE, 2.0, 'WAIT_FOR_CHARGE'),
        ]

        for state, duration, label in phases:
            steps = int(duration / 0.5)
            for _ in range(steps):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('[mock] cancel detected — aborting')
                    goal_handle.canceled()
                    return DockRobot.Result()
                publish_feedback(state)
                time.sleep(0.5)

        # ── 성공 결과 반환 ────────────────────────────────────────────────────
        self.get_logger().info(f'[mock] SUCCESS (elapsed={elapsed():.1f}s)')
        result = DockRobot.Result()
        result.success = True
        result.error_code = DockRobot.Result.NONE
        result.num_retries = 0
        goal_handle.succeed()
        return result

    def _sleep_interruptible(self, goal_handle, duration: float, step: float = 0.2):
        """cancel 체크하면서 대기."""
        elapsed = 0.0
        while elapsed < duration:
            if goal_handle.is_cancel_requested:
                break
            time.sleep(step)
            elapsed += step


def main():
    rclpy.init()
    node = MockDockRobotServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
