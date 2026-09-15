#!/usr/bin/env python3
"""HH_260915 - Real ROS recorder with synthetic inputs, NEVER a driving test.

Creates only status/event publishers on an isolated localhost ROS domain. The
output directory must be new; neither production records nor control topics are
opened. Hold the recorder alive for optional real React screenshot capture.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--hold-seconds', type=float, default=0)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = '188'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.pop('CYCLONEDDS_URI', None)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'runtime/python'))
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from avg_msgs.msg import AvgPlatformStatus, ModuleState
    from std_msgs.msg import String
    from camrod_ui.mission_recorder_node import create_ros_node
    from camrod_ui.mission_recording_bridge import MissionRecordingEmitter

    rclpy.init(args=['--ros-args', '-p', f'storage_root:={output / "records"}',
                    '-p', 'environment:=test', '-p', 'robot_id:=isolated-ros-smoke'])
    recorder = create_ros_node()
    sender = Node('mission_recorder_test_inputs', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    executor.add_node(sender)
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    events = sender.create_publisher(String, '/ui/mission_recording/events', qos)
    statuses = sender.create_publisher(AvgPlatformStatus, '/platform/status', 20)
    gates = sender.create_publisher(ModuleState, '/control/cmd_vel_safety_gate/status', qos)
    mode_speed = [1, 0.0]
    def publish_status():
        msg = AvgPlatformStatus()
        msg.velocity.header.stamp = sender.get_clock().now().to_msg()
        msg.velocity.header.frame_id = 'robot_center_link'
        msg.control_mode = mode_speed[0]
        msg.velocity.twist.linear.x = mode_speed[1]
        msg.battery_state_available = True
        msg.battery_percentage = 80.0
        msg.motor_rpm = [float(mode_speed[1] * 100)] * 4
        statuses.publish(msg)
    sender.create_timer(0.05, publish_status)
    emitter = MissionRecordingEmitter(lambda data: events.publish(String(data=data)),
                                     session='isolated-ros-smoke')
    def pump(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=min(0.05, max(0.0, until-time.monotonic())))
    def snapshot():
        return recorder._worker.cached_snapshot()
    def wait_for(predicate, timeout=4):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            pump(0.1)
            value = snapshot()
            if predicate(value):
                return value
        raise AssertionError('Recorder condition not reached: ' + json.dumps(snapshot(), ensure_ascii=False))
    def gate(reason, level=0):
        msg = ModuleState()
        msg.module_name = 'cmd_vel_safety_gate'
        msg.level = level
        msg.operating_state = 'BLOCKED' if level else 'READY'
        msg.message = reason
        gates.publish(msg)

    report = {'provenance': 'Actual ROS node and typed DDS messages; synthetic test inputs, NOT robot/CARLA driving.',
              'ros_domain_id': 188, 'control_commands_published': False, 'checks': []}
    try:
        pump(1.5)
        wait_for(lambda s: s.get('recorder', {}).get('platform_stale') is False)
        generation = 0
        for site in ('B7', 'B8', 'B9'):
            for intent in ('delivery', 'recall'):
                generation += 1
                gate('reasons=none')
                emitter.start(site, intent, generation, f'attempt-{generation}', 'isolated_test')
                emitter.phase(1, 'MOVING_TO_SITE')
                mode_speed[:] = [1, 1.0]
                pump(1.1)
                if intent == 'recall':
                    emitter.request_return('recall_prepare', final_return=False)
                    emitter.phase(9, 'RETURN_WITH_CARGO', leg_kind='recall')
                gate('reasons=obstacle_stop; TEST_INPUT_ONLY', 1)
                mode_speed[:] = [1, 0.0]
                pump(1.3)
                stopped = wait_for(lambda s: s.get('current_mission', {}).get('stop_count', 0) >= 1)
                distance_before = stopped['current_mission']['total_m']
                pump(1.2)
                distance_after = snapshot()['current_mission']['total_m']
                assert abs(distance_after - distance_before) < 1e-9, 'stopped distance increased'
                emitter.stop('operator_manual_recovery_test')
                gate('reasons=can_control_mode_required; TEST_INPUT_ONLY', 1)
                mode_speed[:] = [0, 0.4]
                pump(1.1)
                emitter.request_return('approved_return_test')
                emitter.phase(3, 'RETURNING_TO_DROP_ZONE')
                gate('reasons=none')
                mode_speed[:] = [1, -0.6]
                pump(1.1)
                emitter.phase(10, 'DROP_ZONE_PARKING')
                mode_speed[:] = [1, 0.0]
                pump(0.8)
                emitter.phase(0, 'DROP_ZONE_WAIT')
                finished = wait_for(lambda s: s.get('lifetime', {}).get('completed_count') == generation)
                mission = finished['missions'][0]
                assert mission['site'] == site and mission['intent'] == intent
                assert mission['autonomous_m'] > 0 and mission['manual_m'] > 0
                assert mission['unknown_m'] > 0 and mission['manual_interventions'] == 1
                assert mission['stop_count'] >= 1 and mission['stop_duration_s'] > 0.5
                assert abs(mission['total_m'] - sum(mission[k] for k in ('autonomous_m', 'manual_m', 'unknown_m'))) < 1e-8
                assert any(e['event'] == 'stopped' and 'obstacle_stop' in e['reason'] for e in mission['events'])
                report['checks'].append({k: mission[k] for k in (
                    'id', 'name', 'site', 'intent', 'result', 'total_m', 'autonomous_m',
                    'manual_m', 'unknown_m', 'stop_count', 'stop_duration_s', 'manual_interventions')})
                print(f'PASS {site} {intent}: total={mission["total_m"]:.4f}m, stationary increment=0m', flush=True)
        mode_speed[:] = [0, 0.4]
        pump(0.7)
        mode_speed[:] = [0, 0.0]
        pump(1.1)
        final = snapshot()
        assert final['outside_missions']['manual_m'] > 0
        assert final['recorder']['environment'] == 'test'
        assert final['recorder']['queue_dropped_total'] == 0
        assert final['recorder']['raw_can_status'] == 'disabled'
        assert not final['recorder']['error'], final['recorder']['error']
        publisher_topics = sorted(topic for topic, _ in sender.get_publisher_names_and_types_by_node(
            recorder.get_name(), recorder.get_namespace()))
        assert not any('cmd_vel' in t or 'drive_enable' in t for t in publisher_topics)
        report.update(result='PASS', mission_count=6, completed_count=6,
                      recorder_publisher_topics=publisher_topics,
                      lifetime=final['lifetime'], outside_missions=final['outside_missions'],
                      actual_raw_can_tested=False, snapshot_path=str(output / 'records/snapshot.json'))
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print('RESULT ' + str(output / 'result.json'), flush=True)
        pump(max(0, args.hold_seconds))
    finally:
        executor.remove_node(recorder)
        executor.remove_node(sender)
        recorder.destroy_node()
        sender.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
