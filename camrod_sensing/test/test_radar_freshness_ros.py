"""HH_261102 - Verify raw-radar health through the actual rasterizer on an isolated ROS domain."""

import os
from pathlib import Path
import subprocess
import time

import pytest


def test_raw_dropout_never_publishes_fresh_clear_grid(monkeypatch, tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from avg_msgs.msg import AvgBool, AvgOccupancyGrid, AvgRange, AvgString
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_ros import StaticTransformBroadcaster

    binary = Path(os.environ.get("CAMROD_RADAR_COST_GRID_BINARY", ""))
    if not binary.is_file():
        pytest.skip("set CAMROD_RADAR_COST_GRID_BINARY to the built radar node")
    env = dict(os.environ, ROS_DOMAIN_ID=str(120 + os.getpid() % 80), ROS_LOCALHOST_ONLY="1",
               ROS_LOG_DIR=str(tmp_path / "ros_logs"))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", env["ROS_LOG_DIR"])
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=int(env["ROS_DOMAIN_ID"]))
    node = rclpy.create_node("radar_freshness_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    grids, evidence = [], []
    node.create_subscription(AvgOccupancyGrid, "/radar_probe/grid", grids.append, latched)
    node.create_subscription(AvgString, "/radar_probe/evidence", lambda msg: evidence.append(msg.data), 10)
    front = [node.create_publisher(AvgRange, f"/radar_probe/front{i}/range", 10) for i in (1, 2)]
    rear_dummy = node.create_publisher(AvgBool, "/radar_probe/rear/dummy_active", latched)
    tf = StaticTransformBroadcaster(node)
    transforms = []
    for child in ("radar_probe_base", "radar_probe_front"):
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        transforms.append(transform)
    tf.sendTransform(transforms)
    args = [str(binary), "--ros-args", "-r", "__ns:=/radar_probe"]
    params = {
        "input_topics": "[/radar_probe/front1/range,/radar_probe/front2/range,/radar_probe/rear/range]",
        "output_topic": "/radar_probe/grid", "obstacle_evidence_topic": "/radar_probe/evidence",
        "base_frame_id": "radar_probe_base", "output_frame_id": "map",
        "dummy_active_topic": "/radar_probe/dummy_active",
        "startup_return_learning_enable": "false", "fixed_return_filter_enable": "false",
        "route_lanelet_filter_enable": "false", "stop_candidate_max_range_m": "0.30",
    }
    for name, value in params.items():
        args += ["-p", f"{name}:={value}"]
    process = subprocess.Popen(args, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    def pump(duration, ranges=(0.501, 0.501), frame="radar_probe_front", old_stamp=False,
             fixed_stamp=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rear_dummy.publish(AvgBool(data=True))
            for index, value in enumerate(ranges):
                if value is None:
                    continue
                message = AvgRange()
                stamp = fixed_stamp or node.get_clock().now().to_msg()
                if old_stamp:
                    stamp.sec -= 2
                message.header.stamp = stamp
                message.header.frame_id = frame
                message.min_range, message.max_range, message.range = 0.02, 0.5, value
                front[index].publish(message)
            executor.spin_once(timeout_sec=0.02)

    def assert_stops_publication(**kwargs):
        pump(0.65, **kwargs)
        count = len(grids)
        pump(0.25, **kwargs)
        assert len(grids) == count
        assert any(text.startswith("sensor_unavailable:") for text in evidence)

    try:
        # No raw data at boot is not a healthy empty scan, despite the disabled rear marker.
        pump(0.8, ranges=(None, None))
        assert process.poll() is None
        assert not grids
        pump(0.6)
        assert grids and max(grids[-1].data) < 85, evidence[-5:]
        # The rear is explicitly disabled and never publishes a Range in this test.
        pump(0.3, ranges=(0.3, 0.501))
        assert max(grids[-1].data) == 100
        assert_stops_publication(ranges=(None, 0.501))
        count = len(grids)
        pump(0.4)
        assert len(grids) > count and max(grids[-1].data) < 85
        assert_stops_publication(old_stamp=True)
        pump(0.4)
        assert_stops_publication(ranges=(float("nan"), 0.501))
        pump(0.4)
        assert_stops_publication(ranges=(0.2, 0.501), frame="radar_probe_missing_tf")
        pump(0.4)
        assert max(grids[-1].data) < 85
        # The grid stamp must age with the raw sample instead of advancing with the timer.
        frozen_stamp = node.get_clock().now().to_msg()
        pump(0.15, fixed_stamp=frozen_stamp)
        assert grids[-1].header.stamp == frozen_stamp
        pump(0.15, ranges=(None, None))
        assert grids[-1].header.stamp == frozen_stamp
    finally:
        process.terminate()
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=5)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
        print(output)
        assert process.returncode in (0, -15), output
