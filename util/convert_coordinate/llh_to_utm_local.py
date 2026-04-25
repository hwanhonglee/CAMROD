#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from pyproj import Transformer
import math


class LlhToUtmLocal(Node):
    def __init__(self):
        super().__init__('llh_to_utm_local')

        self.declare_parameter('input_topic', '/gnss_1/llh_position')
        self.declare_parameter('output_topic', '/gnss_1/local_pose_utm')
        self.declare_parameter('frame_id', 'map')

        # 맵 원점(UTM 기준)
        self.declare_parameter('origin_x', 419092.66)
        self.declare_parameter('origin_y', 4077909.06)

        # WGS84 UTM Zone 52N
        self.declare_parameter('map_epsg', 32652)

        # z 처리
        self.declare_parameter('use_altitude', False)
        self.declare_parameter('origin_z', 0.0)
        # HH_260307-00:00 Optional XY yaw alignment (deg) between map projection axes and lanelet map axes.
        self.declare_parameter('yaw_offset_deg', 0.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.map_epsg = int(self.get_parameter('map_epsg').value)

        self.use_altitude = bool(self.get_parameter('use_altitude').value)
        self.origin_z = float(self.get_parameter('origin_z').value)
        self.yaw_offset_deg = float(self.get_parameter('yaw_offset_deg').value)
        self.yaw_offset_rad = math.radians(self.yaw_offset_deg)

        self.llh_to_map = Transformer.from_crs("EPSG:4326", f"EPSG:{self.map_epsg}", always_xy=True)

        self.sub = self.create_subscription(NavSatFix, self.input_topic, self.cb, 10)
        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)

        self.count = 0
        self.get_logger().info(f"Input : {self.input_topic}")
        self.get_logger().info(f"Output: {self.output_topic}")
        self.get_logger().info(f"Map EPSG: {self.map_epsg}")
        self.get_logger().info(f"Origin : ({self.origin_x}, {self.origin_y})")
        self.get_logger().info(f"Yaw offset (deg): {self.yaw_offset_deg}")

    def cb(self, msg: NavSatFix):
        if msg.status.status < 0:
            return

        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        map_x, map_y = self.llh_to_map.transform(lon, lat)

        local_x = map_x - self.origin_x
        local_y = map_y - self.origin_y
        if abs(self.yaw_offset_rad) > 1e-12:
            c = math.cos(self.yaw_offset_rad)
            s = math.sin(self.yaw_offset_rad)
            local_x, local_y = (c * local_x - s * local_y, s * local_x + c * local_y)
        local_z = (alt - self.origin_z) if self.use_altitude else 0.0

        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.pose.position.x = float(local_x)
        out.pose.position.y = float(local_y)
        out.pose.position.z = float(local_z)
        out.pose.orientation.w = 1.0  # GNSS만으로는 heading 없음

        self.pub.publish(out)

        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info(
                f"LLH=({lat:.7f},{lon:.7f}) -> UTM=({map_x:.2f},{map_y:.2f}) -> LOCAL=({local_x:.2f},{local_y:.2f})"
            )


def main():
    rclpy.init()
    node = LlhToUtmLocal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
