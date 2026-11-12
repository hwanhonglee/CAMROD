#!/usr/bin/env python3
"""
RViz helper for ndt_rviz.launch.py: draw the NDT map, aligned scan and NDT pose at the 2D ground plane.

The camrod map frame keeps ellipsoidal-ish heights (origin altitude 0 -> the ground is at z ~290 m and varies by
~18 m over the site), while the lanelet2 markers and the 2D EKF pose sit at z = 0. This node publishes the TF
map -> map_viz with z = -(current NDT z) and relabels the clouds/pose into map_viz, so around the robot everything is
at z ~ 0 next to the lanelet2 map. Messages are not copied point by point (only the header changes).
  in  /localization/ndt/viz/map (latched), /localization/ndt/points_aligned, /localization/ndt/pose_with_covariance
  out /localization/ndt/viz/map_flat (latched), /localization/ndt/viz/scan_flat, /localization/ndt/viz/pose_flat,
      TF map -> map_viz
Runs only with the NDT RViz.
"""
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster

FRAME = 'map_viz'


class Flatten(Node):
    def __init__(self):
        super().__init__('ndt_viz_flatten')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.tf = TransformBroadcaster(self)
        self.pub_map = self.create_publisher(PointCloud2, '/localization/ndt/viz/map_flat', latched)
        self.pub_scan = self.create_publisher(PointCloud2, '/localization/ndt/viz/scan_flat', 5)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/localization/ndt/viz/pose_flat', 5)
        self.map = None        # held until the first NDT pose: RViz drops a latched cloud whose TF is not there yet
        self.have_tf = False
        self.create_subscription(PointCloud2, '/localization/ndt/viz/map', self.on_map, latched)
        self.create_subscription(PointCloud2, '/localization/ndt/points_aligned', self.on_scan, 5)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/ndt/pose_with_covariance', self.on_pose, 5)

    def on_map(self, m):
        m.header.frame_id = FRAME
        m.header.stamp.sec = m.header.stamp.nanosec = 0     # static map: use the latest map -> map_viz
        self.map = m
        if self.have_tf:
            self.pub_map.publish(m)

    def on_pose(self, p):
        t = TransformStamped()
        t.header.stamp = p.header.stamp
        t.header.frame_id = p.header.frame_id or 'map'
        t.child_frame_id = FRAME
        t.transform.translation.z = -p.pose.pose.position.z
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)
        if not self.have_tf:
            self.have_tf = True
            if self.map is not None:
                self.pub_map.publish(self.map)
        p.header.frame_id = FRAME
        self.pub_pose.publish(p)

    def on_scan(self, m):
        m.header.frame_id = FRAME
        self.pub_scan.publish(m)


def main():
    rclpy.init()
    rclpy.spin(Flatten())


if __name__ == '__main__':
    main()
