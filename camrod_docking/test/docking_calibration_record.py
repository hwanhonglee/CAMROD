#!/usr/bin/env python3
"""
도킹 캘리브레이션 측정 스크립트

로봇이 태그 정면에 위치한 상태에서 실행하면:
  1. external_detection_translation_x 권장값 (base_link → 도킹 접촉점 거리)
  2. external_detection_rotation_pitch 검증 (부호 방향 확인)
을 자동으로 계산하여 출력합니다.

사용법:
  ros2 run camrod_docking docking_calibration_record
  또는
  python3 src/camrod_docking/test/docking_calibration_record.py

사전 조건:
  - docking.launch.py 실행 중 (AprilTag 파이프라인 동작 중)
  - 로봇이 도킹 태그 정면 ~0.5m 이내에 위치
"""

import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException


MEASURE_INTERVAL_SEC = 1.5
MEASURE_COUNT = 5


class DockingCalibrationRecorder(Node):
    def __init__(self):
        super().__init__('docking_calibration_record')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.dock_pose_sub = self.create_subscription(
            PoseStamped,
            '/docking/detected_dock_pose',
            self._on_dock_pose,
            10)

        self.latest_dock_pose = None
        self.measure_count = 0

        self.timer = self.create_timer(MEASURE_INTERVAL_SEC, self._measure)
        self.get_logger().info(
            f'캘리브레이션 시작: {MEASURE_COUNT}회 측정 ({MEASURE_INTERVAL_SEC}초 간격)')

    def _on_dock_pose(self, msg: PoseStamped):
        self.latest_dock_pose = msg

    def _measure(self):
        if self.measure_count >= MEASURE_COUNT:
            return

        if self.latest_dock_pose is None:
            self.get_logger().warn('/docking/detected_dock_pose 수신 대기 중...')
            return

        try:
            t = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'TF odom→base_link 실패: {e}')
            return

        robot_x = t.transform.translation.x
        robot_y = t.transform.translation.y
        robot_yaw = _quat_to_yaw(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w)

        tag_pose = self.latest_dock_pose.pose
        tag_x = tag_pose.position.x
        tag_y = tag_pose.position.y
        tag_yaw = _quat_to_yaw(
            tag_pose.orientation.x,
            tag_pose.orientation.y,
            tag_pose.orientation.z,
            tag_pose.orientation.w)

        # base_link → 태그 직선 거리
        dist = math.hypot(tag_x - robot_x, tag_y - robot_y)

        # 접근축(robot_heading + π) 방향으로의 투영 거리
        # 후진 도킹: 로봇 후면이 도크를 향하므로 접근 방향 = robot_heading + π
        approach_yaw = robot_yaw + math.pi
        dx = tag_x - robot_x
        dy = tag_y - robot_y
        proj = dx * math.cos(approach_yaw) + dy * math.sin(approach_yaw)

        # rotation_pitch 검증: dock_pose yaw ≈ robot_heading + π 여부
        expected_yaw = (robot_yaw + math.pi + math.pi) % (2 * math.pi) - math.pi
        yaw_error_deg = math.degrees(tag_yaw - expected_yaw)

        self.measure_count += 1
        print('=' * 56)
        print(f'  측정 {self.measure_count}/{MEASURE_COUNT}')
        print(f'  [로봇]  x={robot_x:.3f}m  y={robot_y:.3f}m  yaw={math.degrees(robot_yaw):.1f}°')
        print(f'  [태그]  x={tag_x:.3f}m  y={tag_y:.3f}m  yaw={math.degrees(tag_yaw):.1f}°')
        print(f'  base_link → 태그 직선:  {dist:.4f}m')
        print(f'  접근축 투영:             {proj:.4f}m')
        print(f'  ★ external_detection_translation_x 권장값: {-proj:.3f}m')
        print()
        print(f'  rotation_pitch 검증:')
        print(f'    기대 yaw: {math.degrees(expected_yaw):.1f}°  '
              f'실제: {math.degrees(tag_yaw):.1f}°  '
              f'오차: {yaw_error_deg:.1f}°')
        if abs(yaw_error_deg) < 20:
            print('    → pitch 부호 정상 (현재 설정 유지)')
        else:
            print('    → pitch 부호 반전 필요 → external_detection_rotation_pitch 부호 변경')
        print('=' * 56)

        if self.measure_count >= MEASURE_COUNT:
            self.get_logger().info('측정 완료. 종료합니다.')
            self.timer.cancel()


def _quat_to_yaw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    rclpy.init(args=sys.argv)
    node = DockingCalibrationRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
