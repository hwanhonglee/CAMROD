#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from pyproj import Transformer


class EcefToUtmLocal(Node):
    def __init__(self):
        super().__init__('ecef_to_utm_local')

        self.declare_parameter('input_topic', '/ekf/odometry_earth')
        self.declare_parameter('output_topic', '/ekf/local_pose_utm')
        self.declare_parameter('frame_id', 'map')

        # 네 맵 원점 (UTM/평면좌표)
        self.declare_parameter('origin_x', 419092.66)
        self.declare_parameter('origin_y', 4077909.06)
        self.declare_parameter('origin_z', 0.0)

        # 맵 좌표계 EPSG (기본 UTM 52N)
        self.declare_parameter('map_epsg', 32652)

        # z 처리
        self.declare_parameter('use_altitude', False)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.origin_z = float(self.get_parameter('origin_z').value)
        self.map_epsg = int(self.get_parameter('map_epsg').value)
        self.use_altitude = bool(self.get_parameter('use_altitude').value)

        # ECEF <-> LLH(WGS84)
        self.ecef_to_llh = Transformer.from_crs("EPSG:4978", "EPSG:4326", always_xy=True)
        # LLH -> Map(UTM 등)
        self.llh_to_map = Transformer.from_crs("EPSG:4326", f"EPSG:{self.map_epsg}", always_xy=True)

        self.sub = self.create_subscription(Odometry, self.input_topic, self.cb, 10)
        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)

        self.log_count = 0
        self.get_logger().info(f'Input : {self.input_topic}')
        self.get_logger().info(f'Output: {self.output_topic}')
        self.get_logger().info(f'Map EPSG: {self.map_epsg}')
        self.get_logger().info(f'Origin : ({self.origin_x}, {self.origin_y})')

    def cb(self, msg: Odometry):
        x_ecef = msg.pose.pose.position.x
        y_ecef = msg.pose.pose.position.y
        z_ecef = msg.pose.pose.position.z

        # ECEF -> LLH
        lon, lat, alt = self.ecef_to_llh.transform(x_ecef, y_ecef, z_ecef)

        # LLH -> UTM(or map projection)
        map_x, map_y = self.llh_to_map.transform(lon, lat)

        local_x = map_x - self.origin_x
        local_y = map_y - self.origin_y
        local_z = (alt - self.origin_z) if self.use_altitude else 0.0

        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.pose.position.x = float(local_x)
        out.pose.position.y = float(local_y)
        out.pose.position.z = float(local_z)
        out.pose.orientation = msg.pose.pose.orientation  # EKF orientation 재사용
        self.pub.publish(out)

        self.log_count += 1
        if self.log_count % 10 == 0:
            self.get_logger().info(
                f"ECEF->LLH ({lat:.7f}, {lon:.7f}, {alt:.2f}) -> MAP ({map_x:.2f}, {map_y:.2f}) -> LOCAL ({local_x:.2f}, {local_y:.2f})"
            )


def main():
    rclpy.init()
    node = EcefToUtmLocal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
