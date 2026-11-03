"""HH_261102 - The real ground-segmentation callback must not freshen replayed raw source timestamps."""

import json
import os
from pathlib import Path
import subprocess
import time
import pytest
import yaml


def test_ground_segmenter_preserves_acquisition_stamp(monkeypatch, tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from std_msgs.msg import Header
    from tf2_ros import StaticTransformBroadcaster

    root = Path(__file__).resolve().parents[3]
    binary = root / "build/ground_segmentation_ros2/ground_segmentation_ros2_node"
    if not binary.is_file():
        pytest.skip("build ground_segmentation_ros2 in this workspace to test the actual callback")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    domain = 120 + os.getpid() % 80
    context = rclpy.Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("ground_stamp_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    outputs = []
    node.create_subscription(PointCloud2, "/ground_stamp_probe/out", outputs.append, 10)
    publisher = node.create_publisher(PointCloud2, "/ground_stamp_probe/in", 10)
    tf = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = "map", "lidar_link"
    transform.transform.rotation.w = 1.0
    tf.sendTransform(transform)
    params = yaml.safe_load((root / "src/camrod_sensing/config/lidar/ground_seg_params.yaml").read_text())["/sensing/lidar/ground_segmentation"]["ros__parameters"]
    args = [str(binary), "--ros-args", "-r", "__node:=ground_stamp_probe_worker",
        "-r", "/ground_segmentation/input_pointcloud:=/ground_stamp_probe/in",
        "-r", "/ground_segmentation/obstacle_points:=/ground_stamp_probe/out"]
    for key, value in params.items():
        args += ["-p", f"{key}:={json.dumps(value)}"]
    process = subprocess.Popen(args, env=dict(os.environ, ROS_DOMAIN_ID=str(domain)),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        header = Header()
        header.frame_id = "lidar_link"
        header.stamp = node.get_clock().now().to_msg()
        header.stamp.sec -= 2
        points = [(0.3 + ix * 0.07, -0.35 + iy * 0.07, -0.75) for ix in range(12) for iy in range(11)]
        cloud = create_cloud_xyz32(header, points)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not outputs:
            publisher.publish(cloud)
            executor.spin_once(timeout_sec=0.05)
        assert outputs
        assert outputs[-1].header.stamp == header.stamp
        assert node.get_clock().now().nanoseconds * 1e-9 - outputs[-1].header.stamp.sec > 2.0
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
        assert process.returncode in (0, -15), output
