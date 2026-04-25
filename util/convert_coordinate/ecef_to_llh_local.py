#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from pyproj import Transformer


class EcefToLlhLocal(Node):
    def __init__(self):
        super().__init__('ecef_to_llh_local')

        self.declare_parameter('input_topic', '/ekf/odometry_earth')
        self.declare_parameter('output_topic', '/ekf/local_pose_llh')
        self.declare_parameter('frame_id', 'map')

        # 기준 LLH (맵 기준점 위경도)
        self.declare_parameter('use_first_sample_as_origin', False)
        self.declare_parameter('origin_lat', 36.843619)
        self.declare_parameter('origin_lon', 128.092549)
        self.declare_parameter('origin_alt', 0.0)

        # z 처리
        self.declare_parameter('use_altitude', False)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.use_first = bool(self.get_parameter('use_first_sample_as_origin').value)
        self.origin_lat = float(self.get_parameter('origin_lat').value)
        self.origin_lon = float(self.get_parameter('origin_lon').value)
        self.origin_alt = float(self.get_parameter('origin_alt').value)
        self.use_altitude = bool(self.get_parameter('use_altitude').value)
        self.origin_set = not self.use_first

        self.ecef_to_llh = Transformer.from_crs("EPSG:4978", "EPSG:4326", always_xy=True)

        self.sub = self.create_subscription(Odometry, self.input_topic, self.cb, 10)
        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)

        self.get_logger().info(f'Input : {self.input_topic}')
        self.get_logger().info(f'Output: {self.output_topic}')
        if self.origin_set:
            self.get_logger().info(
                f'Origin LLH=({self.origin_lat}, {self.origin_lon}, {self.origin_alt})'
            )
        else:
            self.get_logger().info('Using first sample as LLH origin')

    def llh_to_local_approx(self, lat, lon, alt):
        """
        기준점(origin_lat/lon) 기준 local meter 근사
        x: East, y: North
        """
        lat0 = math.radians(self.origin_lat)
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        lon0 = math.radians(self.origin_lon)

        dlat = lat_rad - lat0
        dlon = lon_rad - lon0

        # 작은 영역 근사 (WGS84 평균 반경)
        R = 6378137.0
        x_east = dlon * math.cos(lat0) * R
        y_north = dlat * R
        z_up = alt - self.origin_alt

        return x_east, y_north, z_up

    def cb(self, msg: Odometry):
        x_ecef = msg.pose.pose.position.x
        y_ecef = msg.pose.pose.position.y
        z_ecef = msg.pose.pose.position.z

        lon, lat, alt = self.ecef_to_llh.transform(x_ecef, y_ecef, z_ecef)

        if not self.origin_set:
            self.origin_lat, self.origin_lon, self.origin_alt = lat, lon, alt
            self.origin_set = True
            self.get_logger().info(
                f'Origin locked LLH=({self.origin_lat:.8f}, {self.origin_lon:.8f}, {self.origin_alt:.3f})'
            )

        local_x, local_y, local_z = self.llh_to_local_approx(lat, lon, alt)
        if not self.use_altitude:
            local_z = 0.0

        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.pose.position.x = float(local_x)
        out.pose.position.y = float(local_y)
        out.pose.position.z = float(local_z)
        out.pose.orientation = msg.pose.pose.orientation
        self.pub.publish(out)


def main():
    rclpy.init()
    node = EcefToLlhLocal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
