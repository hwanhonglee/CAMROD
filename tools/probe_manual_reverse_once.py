#!/usr/bin/env python3
"""HH_261006 - Trace one bounded visible-UI reverse pulse in local CARLA only."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from avg_msgs.msg import ModuleState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/virtual_carla'))
from manual_4ws_evidence import ManualDriveBrowser


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    # HH_261007 - A bounded longer pulse permits real wheel reorientation before
    # judging chassis motion. This does not raise or replace the UI speed limit.
    parser.add_argument('--duration', type=float, default=0.5)
    # HH_261007 - Exercise the same visible UI path for a bounded zero turn.
    # Directional map holds are expected in this case, not silently bypassed.
    parser.add_argument('--motion', choices=('reverse', 'turn_right'), default='reverse')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0.2 <= args.duration <= 2.0:
        parser.error('--duration must be between 0.2 and 2.0 seconds')
    if args.output.exists():
        parser.error('Refusing to overwrite existing evidence')
    rclpy.init()
    node = rclpy.create_node('bounded_manual_reverse_probe')
    rows = []
    latest = {}
    start = time.monotonic()

    def receive(name):
        def callback(msg):
            if isinstance(msg, Twist):
                data = dict(x=msg.linear.x, y=msg.linear.y, yaw=msg.angular.z)
            elif isinstance(msg, Odometry):
                p, v = msg.pose.pose.position, msg.twist.twist.linear
                q = msg.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                data = dict(x=p.x, y=p.y, speed=math.hypot(v.x, v.y),
                            heading_rad=yaw, yaw_rate=msg.twist.twist.angular.z)
            else:
                data = dict(state=msg.operating_state, message=msg.message)
            latest[name] = data
            rows.append(dict(t=time.monotonic() - start, source=name, **data))
        return callback

    subscriptions = [node.create_subscription(kind, topic, receive(name), 10)
                     for kind, topic, name in (
                         (Twist, '/control/manual_cmd_vel_ros', 'manual'),
                         (Twist, '/control/cmd_vel_ros', 'output'),
                         (Odometry, '/carla/ego_vehicle/odometry', 'odometry'),
                         (ModuleState, '/control/cmd_vel_safety_gate/status', 'gate'))]

    def observe(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    browser = None
    key = 'KeyS' if args.motion == 'reverse' else 'KeyD'
    result = dict(test='bounded_real_UI_' + args.motion + '_pulse', duration_s=args.duration,
                  obstacle_safety_bypassed=False, maximum_speed_changed=False)
    try:
        browser = ManualDriveBrowser('http://127.0.0.1:9224', 'http://127.0.0.1:8010')
        result['before_ui'] = browser.snapshot()
        if not result['before_ui'].get('armed'):
            raise RuntimeError('Manual control is not already armed; refusing to arm it')
        observe(1.5)
        gate_state = latest.get('gate', {}).get('state', '')
        if gate_state != 'ENABLED' and not (
                args.motion == 'turn_right' and gate_state == 'ROUTE_SAFETY_HOLD'):
            raise RuntimeError('Gate not ENABLED; refusing motion probe')
        if latest.get('odometry', {}).get('speed', math.inf) > 0.02:
            raise RuntimeError('Robot already moving; operator owns motion')
        if any(abs(latest.get('manual', {}).get(axis, 0)) > 0.001
               for axis in ('x', 'y', 'yaw')):
            raise RuntimeError('Operator command active; refusing overlapping input')
        # HH_261006 - Send normal browser key events, never a direct ROS command.
        # Always release the key and request zero, including on failed tracing.
        browser._call('Page.bringToFront', {})
        # HH_261007 - Range inputs intentionally own keyboard focus. Click a
        # non-control panel heading before a keyboard test, as an operator does.
        browser.scroll_pointer_target_into_view('.manual-drive-title', 'manual panel heading')
        browser._click('.manual-drive-title', 'manual panel heading')
        try:
            browser.dispatch_key(key, True)
            observe(args.duration)
        finally:
            browser.dispatch_key(key, False)
            browser.zero()
        observe(3.0)
        result['after_ui'] = browser.snapshot()
        result['transport'] = browser.probe()
        result['status'] = 'pulse_completed'
    except Exception as error:
        result.update(status='failed', error=str(error))
        raise
    finally:
        result['samples'] = rows
        result['peak_speed_mps'] = max(
            (r['speed'] for r in rows if r['source'] == 'odometry'), default=0)
        result['manual_reverse_observed'] = any(
            r['source'] == 'manual' and r['x'] < -0.01 for r in rows)
        result['gate_reverse_observed'] = any(
            r['source'] == 'output' and r['x'] < -0.01 for r in rows)
        # HH_261007 - Received commands alone are not motion evidence. Preserve
        # actual CARLA odometry displacement and the final stopped condition.
        positions = [r for r in rows if r['source'] == 'odometry']
        result['net_displacement_m'] = (math.hypot(
            positions[-1]['x'] - positions[0]['x'],
            positions[-1]['y'] - positions[0]['y']) if len(positions) > 1 else 0.0)
        result['last_speed_mps'] = positions[-1]['speed'] if positions else None
        result['net_yaw_deg'] = (math.degrees(math.atan2(
            math.sin(positions[-1]['heading_rad'] - positions[0]['heading_rad']),
            math.cos(positions[-1]['heading_rad'] - positions[0]['heading_rad'])))
            if len(positions) > 1 else 0.0)
        result['motion_verified'] = bool(
            result['manual_reverse_observed'] and result['gate_reverse_observed']
            and result['net_displacement_m'] > 0.02 and positions
            and positions[-1]['speed'] < 0.03)
        if args.motion == 'turn_right':
            result['manual_turn_observed'] = any(
                r['source'] == 'manual' and r['yaw'] < -0.01 for r in rows)
            result['gate_turn_observed'] = any(
                r['source'] == 'output' and r['yaw'] < -0.01 for r in rows)
            result['motion_verified'] = bool(
                result['manual_turn_observed'] and result['gate_turn_observed']
                and abs(result['net_yaw_deg']) > 1.0 and positions
                and positions[-1]['speed'] < 0.03
                and abs(positions[-1]['yaw_rate']) < 0.03)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print(json.dumps({k: v for k, v in result.items()
                          if k not in ('samples', 'transport', 'before_ui', 'after_ui')},
                         ensure_ascii=False))
        if browser:
            browser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
