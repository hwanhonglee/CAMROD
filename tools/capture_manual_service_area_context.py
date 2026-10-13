#!/usr/bin/env python3
"""HH_261007 - Read-only snapshot of a stopped simulator's manual map context.

This tool publishes no messages and never arms or moves the robot. The snapshot
supports offline geometry replay; it is not evidence of physical motion.
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from avg_msgs.msg import AvgOccupancyGrid, AvgPoseStamped, ModuleState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--areas-file', required=True, type=Path)
    parser.add_argument('--when-manual-blocked', action='store_true')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refusing to overwrite evidence')
    rclpy.init()
    node = rclpy.create_node('manual_service_context_read_only')
    values, received = {}, {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)

    def receive(name):
        def callback(msg):
            received[name] = time.monotonic()
            if name == 'pose':
                values[name] = dict(x=msg.pose.position.x, y=msg.pose.position.y,
                                    yaw=yaw(msg.pose.orientation), frame=msg.header.frame_id)
            elif name == 'gate':
                values[name] = dict(state=msg.operating_state, message=msg.message)
            else:
                origin = msg.info.origin
                values[name] = dict(frame=msg.header.frame_id, resolution=msg.info.resolution,
                                    width=msg.info.width, height=msg.info.height,
                                    origin=dict(x=origin.position.x, y=origin.position.y,
                                                yaw=yaw(origin.orientation)),
                                    data=list(msg.data))
        return callback

    subscriptions = [node.create_subscription(kind, topic, receive(name), qos)
                     for kind, topic, name in (
                         (AvgPoseStamped, '/localization/pose', 'pose'),
                         (AvgOccupancyGrid, '/map/cost_grid/lanelet_safety', 'grid'),
                         (ModuleState, '/control/cmd_vel_safety_gate/status', 'gate'))]
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            blocked = 'manual_service_area_sweep_outside' in values.get('gate', {}).get('message', '')
            if (len(values) == 3 and time.monotonic() - received['pose'] < 0.5
                    and (not args.when_manual_blocked or blocked)):
                break
        if len(values) != 3:
            raise RuntimeError(f'Missing evidence: {set(("pose", "grid", "gate")) - values.keys()}')
        if args.when_manual_blocked and not blocked:
            raise RuntimeError('No manual service rejection observed during capture')
        values.update(areas_file=str(args.areas_file.resolve()),
                      captured_unix_sec=time.time(), read_only=True,
                      ages_sec={key: time.monotonic() - value for key, value in received.items()})
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(values, ensure_ascii=False), encoding='utf-8')
        print(json.dumps({key: value for key, value in values.items() if key != 'grid'}, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
