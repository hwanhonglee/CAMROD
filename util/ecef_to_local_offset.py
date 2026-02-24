#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class EcefToLocalOffset(Node):
    def __init__(self):
        super().__init__('ecef_to_local_offset')

        self.declare_parameter('input_topic', '/ekf/odometry_earth')
        self.declare_parameter('output_topic', '/ekf/odometry_local_ecef_offset')
        self.declare_parameter('output_frame_id', 'map')

        # 기준 ECEF origin (직접 지정 or 첫 샘플 자동)
        self.declare_parameter('use_first_sample_as_origin', False)
        self.declare_parameter('origin_x', -3152935.2139837136)
        self.declare_parameter('origin_y', 4022164.543522383)
        self.declare_parameter('origin_z', 3803695.2796160863)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value

        self.use_first = bool(self.get_parameter('use_first_sample_as_origin').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.origin_z = float(self.get_parameter('origin_z').value)
        self.origin_set = not self.use_first

        self.sub = self.create_subscription(Odometry, self.input_topic, self.cb, 10)
        self.pub = self.create_publisher(Odometry, self.output_topic, 10)

        self.get_logger().info(f'Input : {self.input_topic}')
        self.get_logger().info(f'Output: {self.output_topic}')
        if self.origin_set:
            self.get_logger().info(f'Origin(ECEF)=({self.origin_x}, {self.origin_y}, {self.origin_z})')
        else:
            self.get_logger().info('Using first sample as ECEF origin')

    def cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        if not self.origin_set:
            self.origin_x, self.origin_y, self.origin_z = x, y, z
            self.origin_set = True
            self.get_logger().info(
                f'Origin locked(ECEF)=({self.origin_x:.3f}, {self.origin_y:.3f}, {self.origin_z:.3f})'
            )

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        out.child_frame_id = msg.child_frame_id

        out.pose.pose.position.x = x - self.origin_x
        out.pose.pose.position.y = y - self.origin_y
        out.pose.pose.position.z = z - self.origin_z

        out.pose.pose.orientation = msg.pose.pose.orientation
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist

        self.pub.publish(out)


def main():
    rclpy.init()
    node = EcefToLocalOffset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
