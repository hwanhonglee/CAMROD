#!/usr/bin/env python3
"""
도킹 캘리브레이션 측정 스크립트 (프레임 정합 버전)

로봇을 도킹 스테이션에 밀착(접촉)시킨 상태에서 실행하면:
  1. external_detection_translation_x / _y 권장값
       (밀착 시 base_link == dock_pose 여야 한다는 조건을 역산)
  2. external_detection_rotation_pitch (회전 보정) 검증
       (회전 config로 만든 dock 헤딩이 robot 헤딩과 정합하는지)
을 SimpleChargingDock 플러그인과 동일한 변환 규약으로 계산해 출력합니다.

핵심 (simple_charging_dock.cpp getRefinedPose 규약):
  - detected_dock_pose 는 camera_rear(광학) 프레임 → fixed_frame(odom)로 TF 변환해야 함
  - dock 방향   : q_dock = q_detected ⊗ q_config, 그 yaw 만 사용
  - dock 위치   : dock = tag_odom + R(dock_yaw)·[tx, ty]
  - 밀착 시 dock == base_link 이므로 [tx,ty] = R(dock_yaw)^-1 ·(base_link - tag_odom)

사용법:
  python3 src/camrod_docking/test/docking_calibration_record.py

사전 조건:
  - docking.launch.py 실행 중 (AprilTag 파이프라인 + docking_server 동작)
  - 로봇이 도킹 스테이션/태그에 밀착(접촉)되어 정렬된 상태
  - /docking/detected_dock_pose 가 발행되고 camera_rear→odom TF 가 존재
"""

import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # noqa: F401  (PoseStamped 에 do_transform 등록용)
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType


MEASURE_INTERVAL_SEC = 1.5
MEASURE_COUNT = 5

DOCKING_SERVER = '/docking/docking_server'
DOCK_PLUGIN = 'apriltag_dock'  # docking_server.yaml 의 dock_plugins 이름

# 라이브 파라미터 조회 실패 시 폴백 (docking_server.yaml 현재값과 일치시킬 것)
FALLBACK = {
    'rotation_pitch': 1.5708,
    'rotation_roll': -1.5708,
    'rotation_yaw': 0.0,
    'translation_x': 0.0,
    'translation_y': 0.0,
}


class DockingCalibrationRecorder(Node):
    def __init__(self):
        super().__init__('docking_calibration_record')

        self.fixed_frame = 'odom'
        self.base_frame = 'base_link'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.dock_pose_sub = self.create_subscription(
            PoseStamped,
            '/docking/detected_dock_pose',
            self._on_dock_pose,
            10)

        self.latest_dock_pose = None
        self.measure_count = 0
        self.done = False
        self.tx_samples = []
        self.ty_samples = []
        self.heading_err_samples = []
        self.dock_dist_samples = []  # isDocked() 가 보게 될 base_link↔dock_pose 거리

        # 실행 중인 docking_server 에서 현재 회전/병진 보정값을 읽어온다
        self.cfg = self._fetch_plugin_params()
        self.q_cfg = _euler_to_quat_tf2(
            self.cfg['rotation_pitch'],   # tf2 setEuler 의 yaw 인자  (플러그인 호출 규약)
            self.cfg['rotation_roll'],    # tf2 setEuler 의 pitch 인자
            self.cfg['rotation_yaw'])     # tf2 setEuler 의 roll 인자

        self.get_logger().info(
            '현재 회전 config: pitch=%.4f roll=%.4f yaw=%.4f / 병진: x=%.3f y=%.3f' % (
                self.cfg['rotation_pitch'], self.cfg['rotation_roll'],
                self.cfg['rotation_yaw'], self.cfg['translation_x'],
                self.cfg['translation_y']))

        self.timer = self.create_timer(MEASURE_INTERVAL_SEC, self._measure)
        self.get_logger().info(
            f'캘리브레이션 시작: {MEASURE_COUNT}회 측정 ({MEASURE_INTERVAL_SEC}초 간격) '
            f'— 로봇이 도킹 스테이션에 밀착되어 있어야 합니다')

    # ── 라이브 파라미터 조회 ────────────────────────────────────────────────
    def _fetch_plugin_params(self):
        names = [
            f'{DOCK_PLUGIN}.external_detection_rotation_pitch',
            f'{DOCK_PLUGIN}.external_detection_rotation_roll',
            f'{DOCK_PLUGIN}.external_detection_rotation_yaw',
            f'{DOCK_PLUGIN}.external_detection_translation_x',
            f'{DOCK_PLUGIN}.external_detection_translation_y',
        ]
        keys = ['rotation_pitch', 'rotation_roll', 'rotation_yaw',
                'translation_x', 'translation_y']

        cli = self.create_client(GetParameters, f'{DOCKING_SERVER}/get_parameters')
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                f'{DOCKING_SERVER} 파라미터 서비스 없음 → 폴백값 사용')
            return dict(FALLBACK)

        req = GetParameters.Request(names=names)
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        resp = future.result()
        if resp is None or len(resp.values) != len(names):
            self.get_logger().warn('파라미터 조회 실패 → 폴백값 사용')
            return dict(FALLBACK)

        cfg = {}
        for k, v in zip(keys, resp.values):
            if v.type == ParameterType.PARAMETER_DOUBLE:
                cfg[k] = v.double_value
            elif v.type == ParameterType.PARAMETER_INTEGER:
                cfg[k] = float(v.integer_value)
            else:
                self.get_logger().warn(f'{k} 조회 불가 → 폴백값 사용')
                cfg[k] = FALLBACK[k]
        return cfg

    def _on_dock_pose(self, msg: PoseStamped):
        self.latest_dock_pose = msg

    # ── 측정 ────────────────────────────────────────────────────────────────
    def _measure(self):
        if self.measure_count >= MEASURE_COUNT:
            return

        if self.latest_dock_pose is None:
            self.get_logger().warn('/docking/detected_dock_pose 수신 대기 중...')
            return

        # 1) 로봇(base_link) pose in odom
        try:
            t = self.tf_buffer.lookup_transform(
                self.fixed_frame, self.base_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f'TF {self.fixed_frame}->{self.base_frame} 실패: {e}')
            return

        robot_x = t.transform.translation.x
        robot_y = t.transform.translation.y
        robot_yaw = _quat_to_yaw(
            t.transform.rotation.x, t.transform.rotation.y,
            t.transform.rotation.z, t.transform.rotation.w)

        # 2) 태그(detected_dock_pose) 를 camera_rear → odom 으로 TF 변환 (플러그인과 동일)
        try:
            tag_odom = self.tf_buffer.transform(
                self.latest_dock_pose, self.fixed_frame,
                timeout=rclpy.duration.Duration(seconds=0.2))
        except TransformException as e:
            self.get_logger().warn(
                f'태그 pose {self.latest_dock_pose.header.frame_id}->{self.fixed_frame} '
                f'변환 실패: {e}')
            return

        tag_x = tag_odom.pose.position.x
        tag_y = tag_odom.pose.position.y
        q = tag_odom.pose.orientation

        # 3) dock 헤딩 = yaw( q_detected ⊗ q_config ) — 플러그인 규약 그대로
        qx, qy, qz, qw = _quat_mul(
            (q.x, q.y, q.z, q.w),
            self.q_cfg)
        dock_yaw = _quat_to_yaw(qx, qy, qz, qw)

        # 4) translation 권장값: 밀착 시 dock == base_link 가정 → 역산
        #    base_link = tag_odom + R(dock_yaw)·[tx,ty]
        ex = robot_x - tag_x
        ey = robot_y - tag_y
        c, s = math.cos(dock_yaw), math.sin(dock_yaw)
        tx = c * ex + s * ey
        ty = -s * ex + c * ey

        # 5) 회전(pitch) 검증: dock_yaw 가 robot_yaw 와 정합하는지
        #    밀착·정렬 시 base_link == dock_pose → dock_yaw ≈ robot_yaw (mod 180°)
        err_same = _wrap_deg(math.degrees(dock_yaw - robot_yaw))
        err_flip = _wrap_deg(math.degrees(dock_yaw - robot_yaw - math.pi))
        # 두 후보 중 0 에 더 가까운 쪽이 정렬 오차
        heading_err = err_same if abs(err_same) <= abs(err_flip) else err_flip

        # 6) docking_threshold 근거: 현재 '적용된' translation 으로 만든 dock_pose 와
        #    base_link 사이 2D 거리 — 플러그인 isDocked() 가 보게 될 값과 동일.
        #    (eq. simple_charging_dock.cpp:226-229 그대로 재현)
        cur_tx, cur_ty = self.cfg['translation_x'], self.cfg['translation_y']
        dock_x = tag_x + c * cur_tx - s * cur_ty
        dock_y = tag_y + s * cur_tx + c * cur_ty
        dock_dist = math.hypot(robot_x - dock_x, robot_y - dock_y)

        self.tx_samples.append(tx)
        self.ty_samples.append(ty)
        self.heading_err_samples.append(heading_err)
        self.dock_dist_samples.append(dock_dist)

        dist = math.hypot(ex, ey)
        self.measure_count += 1
        print('=' * 60)
        print(f'  측정 {self.measure_count}/{MEASURE_COUNT}')
        print(f'  [로봇 odom]  x={robot_x:.3f}  y={robot_y:.3f}  yaw={math.degrees(robot_yaw):.1f}°')
        print(f'  [태그 odom]  x={tag_x:.3f}  y={tag_y:.3f}  dock_yaw={math.degrees(dock_yaw):.1f}°')
        print(f'  base_link→태그 직선거리: {dist:.4f} m')
        print(f'  ★ translation_x 권장: {tx:+.3f} m   translation_y 권장: {ty:+.3f} m')
        print(f'  회전(pitch) 검증: dock_yaw vs robot_yaw 오차 = {heading_err:+.1f}°')
        print(f'      (same={err_same:+.1f}° / flip(180°)={err_flip:+.1f}°)')
        print(f'  isDocked() 잔여거리 d={dock_dist:.4f} m '
              f'(현재적용 tx={cur_tx:.3f} ty={cur_ty:.3f} 기준)')
        print('=' * 60)

        if self.measure_count >= MEASURE_COUNT:
            self._summary()
            self.get_logger().info('측정 완료. 종료합니다.')
            self.timer.cancel()
            self.done = True

    def _summary(self):
        tx_m, tx_s = _mean_std(self.tx_samples)
        ty_m, ty_s = _mean_std(self.ty_samples)
        he_m, he_s = _mean_std(self.heading_err_samples)
        dd_m, dd_s = _mean_std(self.dock_dist_samples)
        dd_max = max(self.dock_dist_samples) if self.dock_dist_samples else 0.0

        # docking_threshold 권장: 접촉 시 잔여거리 최댓값 + 노이즈 마진(2cm),
        # 단 컨트롤러 정지 정밀도(~3cm) 미만으로는 내리지 않음.
        margin = 0.02
        floor = 0.03
        thr_reco = max(dd_max + margin, floor)
        cur_tx, cur_ty = self.cfg['translation_x'], self.cfg['translation_y']
        calibrated = abs(cur_tx - tx_m) < 0.03 and abs(cur_ty - ty_m) < 0.03

        print()
        print('#' * 60)
        print(f'  요약 ({len(self.tx_samples)}회 평균)')
        print(f'  external_detection_translation_x: {tx_m:+.3f} m  (σ={tx_s:.3f})')
        print(f'  external_detection_translation_y: {ty_m:+.3f} m  (σ={ty_s:.3f})')
        print(f'  회전 정렬 오차: {he_m:+.1f}°  (σ={he_s:.1f})')
        if abs(he_m) < 10.0:
            print('    → 회전(pitch/roll) config 정상. 현재값 유지.')
        else:
            print('    → 회전 정렬 오차 큼. external_detection_rotation_pitch 부호/값 재검토 필요.')
        print('    ※ 회전 오차는 0° 또는 ±180°(도킹 방향 규약)에 가까워야 정상.')
        print('       ~24° 처럼 어중간하면 광학→nav 회전 보정 오류 신호.')
        print('-' * 60)
        print(f'  isDocked() 잔여거리 d: 평균={dd_m:.4f} 최대={dd_max:.4f} m (σ={dd_s:.4f})')
        print(f'  ★ docking_threshold 권장: {thr_reco:.3f} m')
        if calibrated:
            print('    → translation 보정이 이미 적용된 상태로 측정됨 → 신뢰 가능.')
        else:
            print(f'    ⚠ 현재 적용 translation(x={cur_tx:.3f}, y={cur_ty:.3f})이 권장값과 다름.')
            print(f'      → 권장 translation_x={tx_m:.3f}/y={ty_m:.3f} 을 먼저 적용·재런치한 뒤')
            print('        다시 측정해야 docking_threshold 권장값이 유효함.')
        print('    ※ threshold 는 접촉 잔여거리보다 크고, 접촉 직전 거리보다는 작아야 함.')
        print('       너무 작으면 영원히 미완료, 너무 크면 조기 완료 판정.')
        print('#' * 60)


def _quat_to_yaw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_mul(q1, q2):
    """Hamilton 곱 q1 ⊗ q2, (x,y,z,w) 순서. doTransform(pose(q2), tf(q1)) 과 동일."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _euler_to_quat_tf2(yaw, pitch, roll):
    """tf2::Quaternion::setEuler(yaw, pitch, roll) 와 동일한 (x,y,z,w) 생성.

    주의: 플러그인은 setEuler(pitch_cfg, roll_cfg, yaw_cfg) 로 호출하므로
    이 함수도 (yaw=pitch_cfg, pitch=roll_cfg, roll=yaw_cfg) 순으로 넘겨야 한다.
    """
    hy, hp, hr = yaw * 0.5, pitch * 0.5, roll * 0.5
    cy, sy = math.cos(hy), math.sin(hy)
    cp, sp = math.cos(hp), math.sin(hp)
    cr, sr = math.cos(hr), math.sin(hr)
    return (
        cr * sp * cy + sr * cp * sy,  # x
        cr * cp * sy - sr * sp * cy,  # y
        sr * cp * cy - cr * sp * sy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def _wrap_deg(deg):
    return (deg + 180.0) % 360.0 - 180.0


def _mean_std(xs):
    n = len(xs)
    if n == 0:
        return 0.0, 0.0
    m = sum(xs) / n
    var = sum((x - m) ** 2 for x in xs) / n
    return m, math.sqrt(var)


def main():
    rclpy.init(args=sys.argv)
    node = DockingCalibrationRecorder()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
