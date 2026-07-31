#!/usr/bin/env python3
"""GNSS path recorder + built-in map viewer (single file, no RViz).

What it does, all in one process:
  1. Subscribes NavSatFix and appends every fix to an accumulated CSV
     (gnss_path_accum.csv) plus a per-session CSV, so data keeps piling up
     across restarts and driving days. All fixes are recorded (RTK
     fixed/float/none); filter later with the status/covariance columns.
  2. Opens a matplotlib window showing the active lanelet2 map
     (/home/nvidia/camrod_ws/src/lanelet2_maps.osm) with the accumulated
     path (green), the current position (red dot) and the direction of
     travel (red arrow), updating live. Previous sessions from the
     accumulated CSV are preloaded so the whole history stays visible.

Coordinates: lat/lon are converted with the same LLH -> ECEF -> ENU
(LocalCartesian) projection the runtime uses, around the map_info.yaml
origin. The OSM node local_x/local_y tags are in this exact frame (verified),
so the path overlays the map without any alignment step.

CSV columns:
  stamp, session, status, lat, lon, alt, cov_xx, cov_yy, cov_zz, x, y, z

Run (GNSS driver must be publishing):
  python3 gnss_csv_logger.py
  # options (ROS parameters):
  python3 gnss_csv_logger.py --ros-args -p gui:=false -p min_move_m:=0.1 \
      -p input_topic:=/sensing/gnss/ublox_gps_node/fix

Export to the waypoint tool format (header: x,y,z,yaw):
  python3 gnss_csv_logger.py export               # all points
  python3 gnss_csv_logger.py export --fixed-only  # RTK fixed only (recommended)
"""

import math
import os
import sys
import threading
import xml.etree.ElementTree as ET
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

# WGS84 ellipsoid
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)

_MAP_OSM = '/home/nvidia/camrod_ws/src/lanelet2_maps.osm'
_CSV_HEADER = "stamp,session,status,lat,lon,alt,cov_xx,cov_yy,cov_zz,x,y,z\n"
_STATUS_NAMES = {-1: 'NO FIX', 0: 'GPS', 1: 'SBAS', 2: 'GBAS'}


def _llh_to_ecef(lat_deg: float, lon_deg: float, alt_m: float):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * math.cos(lon)
    y = (n + alt_m) * cos_lat * math.sin(lon)
    z = (n * (1.0 - _E2) + alt_m) * sin_lat
    return x, y, z


class EnuConverter:
    """LLH -> ENU around a fixed WGS84 origin (matches LocalCartesian)."""

    def __init__(self, origin_lat: float, origin_lon: float, origin_alt: float):
        self._ox, self._oy, self._oz = _llh_to_ecef(origin_lat, origin_lon, origin_alt)
        lat = math.radians(origin_lat)
        lon = math.radians(origin_lon)
        self._sin_lat, self._cos_lat = math.sin(lat), math.cos(lat)
        self._sin_lon, self._cos_lon = math.sin(lon), math.cos(lon)

    def convert(self, lat_deg: float, lon_deg: float, alt_m: float):
        x, y, z = _llh_to_ecef(lat_deg, lon_deg, alt_m)
        dx, dy, dz = x - self._ox, y - self._oy, z - self._oz
        east = -self._sin_lon * dx + self._cos_lon * dy
        north = (-self._sin_lat * self._cos_lon * dx
                 - self._sin_lat * self._sin_lon * dy
                 + self._cos_lat * dz)
        up = (self._cos_lat * self._cos_lon * dx
              + self._cos_lat * self._sin_lon * dy
              + self._sin_lat * dz)
        return east, north, up


def parse_osm_ways(osm_path: str, converter: EnuConverter):
    """Return every OSM way as ([x...], [y...]) in the map-local frame.

    Uses the node local_x/local_y tags (already map-local); falls back to
    lat/lon -> ENU conversion when the tags are missing.
    """
    root = ET.parse(osm_path).getroot()
    nodes = {}
    for nd in root.iter('node'):
        tags = {t.get('k'): t.get('v') for t in nd.findall('tag')}
        if 'local_x' in tags and 'local_y' in tags:
            xy = (float(tags['local_x']), float(tags['local_y']))
        else:
            x, y, _ = converter.convert(
                float(nd.get('lat')), float(nd.get('lon')), 0.0)
            xy = (x, y)
        nodes[nd.get('id')] = xy
    ways = []
    for way in root.iter('way'):
        pts = [nodes[ref.get('ref')] for ref in way.findall('nd')
               if ref.get('ref') in nodes]
        if len(pts) >= 2:
            ways.append(([p[0] for p in pts], [p[1] for p in pts]))
    return ways


class GnssCsvLogger(Node):
    def __init__(self):
        super().__init__('gnss_csv_logger')

        # Standalone ublox driver publishes /gnss/...; under full bringup the
        # same stream is namespaced as /sensing/gnss/ublox_gps_node/fix.
        self.declare_parameter('input_topic', '/gnss/ublox_gps_node/fix')
        self.declare_parameter('output_dir', '/home/nvidia/camrod_ws/src/util/gnss_data')
        self.declare_parameter('accum_filename', 'gnss_path_accum.csv')
        self.declare_parameter('write_session_file', True)
        self.declare_parameter('map_osm', _MAP_OSM)
        self.declare_parameter('gui', True)
        # Map origin: keep in sync with camrod_map/config/map_info.yaml.
        self.declare_parameter('origin_lat', 36.8435737)
        self.declare_parameter('origin_lon', 128.0925646)
        self.declare_parameter('origin_alt', 0.0)
        # 0.0 records every fix; >0 skips points closer than this to the last
        # recorded one (useful to avoid stationary pile-up).
        self.declare_parameter('min_move_m', 0.0)
        # False writes z=0.0 (flat path for map drawing); raw alt is always kept.
        self.declare_parameter('use_altitude', False)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.map_osm = self.get_parameter('map_osm').value
        self.gui = bool(self.get_parameter('gui').value)
        self.min_move_m = float(self.get_parameter('min_move_m').value)
        self.use_altitude = bool(self.get_parameter('use_altitude').value)

        self.converter = EnuConverter(
            float(self.get_parameter('origin_lat').value),
            float(self.get_parameter('origin_lon').value),
            float(self.get_parameter('origin_alt').value),
        )

        self.session = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.accum_path = os.path.join(
            self.output_dir, self.get_parameter('accum_filename').value)
        self.session_path = (
            os.path.join(self.output_dir, f'gnss_path_{self.session}.csv')
            if bool(self.get_parameter('write_session_file').value) else '')

        # Files open lazily on the first fix so idle runs leave no clutter.
        self._accum_file = None
        self._session_file = None
        self._last_xy = None
        self._written = 0
        self._skipped_move = 0
        self._skipped_invalid = 0

        # Shared with the GUI thread (CPython list append is atomic).
        self.path_x = []
        self.path_y = []
        self.current = None          # (x, y, heading_rad)
        self.last_status = None      # (status_code, sigma_m)
        self._heading = 0.0

        self._preload_accum()

        self.sub = self.create_subscription(
            NavSatFix, self.input_topic, self.on_fix, qos_profile_sensor_data)
        self.get_logger().info(
            f'logging {self.input_topic} -> {self.accum_path} '
            f'(session {self.session}, min_move_m={self.min_move_m}, '
            f'preloaded={len(self.path_x)} pts)')

    def _preload_accum(self):
        if not os.path.isfile(self.accum_path):
            return
        try:
            with open(self.accum_path) as f:
                for line in f:
                    cols = line.strip().split(',')
                    if len(cols) < 12 or cols[0] == 'stamp':
                        continue
                    try:
                        self.path_x.append(float(cols[9]))
                        self.path_y.append(float(cols[10]))
                    except ValueError:
                        continue
        except OSError as e:
            self.get_logger().warn(f'accum preload failed: {e}')

    def _open_append(self, path: str):
        os.makedirs(self.output_dir, exist_ok=True)
        new_file = not os.path.isfile(path) or os.path.getsize(path) == 0
        f = open(path, 'a', buffering=1)
        if new_file:
            f.write(_CSV_HEADER)
        return f

    def on_fix(self, msg: NavSatFix):
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            self._skipped_invalid += 1
            return

        alt = msg.altitude if math.isfinite(msg.altitude) else 0.0
        x, y, up = self.converter.convert(msg.latitude, msg.longitude, alt)
        z = up if self.use_altitude else 0.0

        if self._last_xy is not None:
            dx, dy = x - self._last_xy[0], y - self._last_xy[1]
            if math.hypot(dx, dy) > 0.05:
                self._heading = math.atan2(dy, dx)
        self.current = (x, y, self._heading)
        self.last_status = (msg.status.status,
                            math.sqrt(max(msg.position_covariance[0], 0.0)))

        if self.min_move_m > 0.0 and self._last_xy is not None:
            if math.hypot(x - self._last_xy[0], y - self._last_xy[1]) < self.min_move_m:
                self._skipped_move += 1
                return
        self._last_xy = (x, y)

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cov = msg.position_covariance
        row = (f'{stamp:.9f},{self.session},{msg.status.status},'
               f'{msg.latitude:.9f},{msg.longitude:.9f},{alt:.4f},'
               f'{cov[0]:.6f},{cov[4]:.6f},{cov[8]:.6f},'
               f'{x:.4f},{y:.4f},{z:.4f}\n')

        if self._accum_file is None:
            self._accum_file = self._open_append(self.accum_path)
            if self.session_path:
                self._session_file = self._open_append(self.session_path)
        self._accum_file.write(row)
        if self._session_file is not None:
            self._session_file.write(row)

        self.path_x.append(x)
        self.path_y.append(y)

        self._written += 1
        if self._written % 500 == 0:
            self.get_logger().info(
                f'{self._written} fixes logged (last: x={x:.2f}, y={y:.2f})')

    def close(self):
        for f in (self._accum_file, self._session_file):
            if f is not None:
                f.close()
        self.get_logger().info(
            f'done: {self._written} written, {self._skipped_move} skipped by '
            f'min_move, {self._skipped_invalid} invalid fixes dropped')


def run_gui(node: GnssCsvLogger):
    """Matplotlib window: lanelet map + accumulated path + live position."""
    import matplotlib
    import matplotlib.pyplot as plt

    ways = []
    if os.path.isfile(node.map_osm):
        ways = parse_osm_ways(node.map_osm, node.converter)
        node.get_logger().info(f'map loaded: {node.map_osm} ({len(ways)} ways)')
    else:
        node.get_logger().warn(f'map not found, drawing path only: {node.map_osm}')

    fig, ax = plt.subplots(figsize=(11, 9))
    fig.canvas.manager.set_window_title('GNSS path on lanelet2 map')
    for wx, wy in ways:
        ax.plot(wx, wy, color='0.55', linewidth=0.8, zorder=1)
    path_line, = ax.plot(node.path_x, node.path_y, color='#00c853',
                         linewidth=1.6, zorder=2, label='GNSS path (accum)')
    cur_dot, = ax.plot([], [], 'o', color='#ff1744', markersize=9,
                       zorder=4, label='current')
    arrow = ax.annotate('', xy=(0, 0), xytext=(0, 0), zorder=5,
                        arrowprops=dict(arrowstyle='-|>', color='#ff1744',
                                        lw=2, mutation_scale=22))
    arrow.set_visible(False)
    ax.set_aspect('equal')
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.set_xlabel('x east [m]')
    ax.set_ylabel('y north [m]')
    ax.legend(loc='upper right', fontsize=9)
    if ways:
        all_x = [v for wx, _ in ways for v in wx]
        all_y = [v for _, wy in ways for v in wy]
        pad = 10.0
        ax.set_xlim(min(all_x) - pad, max(all_x) + pad)
        ax.set_ylim(min(all_y) - pad, max(all_y) + pad)

    def refresh(_):
        path_line.set_data(node.path_x, node.path_y)
        if node.current is not None:
            x, y, hdg = node.current
            cur_dot.set_data([x], [y])
            arrow.set_visible(True)
            arrow.set_position((x, y))                       # tail
            arrow.xy = (x + 3.0 * math.cos(hdg), y + 3.0 * math.sin(hdg))
        status = ''
        if node.last_status is not None:
            code, sigma = node.last_status
            status = (f' | fix: {_STATUS_NAMES.get(code, code)}'
                      f'  sigma={sigma:.2f} m')
        ax.set_title(f'logged {node._written} pts (session {node.session})'
                     f'{status}')
        return path_line, cur_dot, arrow

    from matplotlib.animation import FuncAnimation
    anim = FuncAnimation(fig, refresh, interval=500,
                         cache_frame_data=False)
    plt.show()  # blocks until the window is closed
    del anim


def export_xyzyaw(accum_path: str, out_path: str, fixed_only: bool,
                  cov_threshold: float = 0.005):
    """Convert the 12-column accumulated CSV to the waypoint format x,y,z,yaw.

    yaw [rad] is the direction of travel derived from consecutive points;
    stationary points reuse the previous heading.
    """
    pts = []
    with open(accum_path) as f:
        for line in f:
            cols = line.strip().split(',')
            if len(cols) < 12 or cols[0] == 'stamp':
                continue
            try:
                cov_xx = float(cols[6])
                x, y, z = float(cols[9]), float(cols[10]), float(cols[11])
            except ValueError:
                continue
            if fixed_only and cov_xx > cov_threshold:
                continue
            pts.append((x, y, z))

    yaw = 0.0
    yaws = []
    for i, (x, y, _) in enumerate(pts):
        if i + 1 < len(pts):
            dx, dy = pts[i + 1][0] - x, pts[i + 1][1] - y
            if math.hypot(dx, dy) > 0.05:
                yaw = math.atan2(dy, dx)
        yaws.append(yaw)
    # Backfill the leading stationary stretch with the first real heading.
    first_real = next((v for v in yaws if v != 0.0), 0.0)
    for i, v in enumerate(yaws):
        if v != 0.0:
            break
        yaws[i] = first_real

    with open(out_path, 'w') as f:
        f.write('x,y,z,yaw\n')
        for (x, y, z), yw in zip(pts, yaws):
            f.write(f'{x:.4f},{y:.4f},{z:.4f},{yw:.6f}\n')
    return len(pts)


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'export':
        fixed_only = '--fixed-only' in sys.argv[2:]
        base = os.path.dirname(os.path.abspath(__file__))
        accum = os.path.join(base, 'gnss_path_accum.csv')
        out = os.path.join(
            base, 'gnss_waypoints_fixed.csv' if fixed_only else 'gnss_waypoints.csv')
        n = export_xyzyaw(accum, out, fixed_only)
        print(f'exported {n} points -> {out}')
        return

    rclpy.init()
    node = GnssCsvLogger()

    use_gui = node.gui and (os.environ.get('DISPLAY') or
                            os.environ.get('WAYLAND_DISPLAY'))
    if node.gui and not use_gui:
        node.get_logger().warn('no display found; running without GUI')

    try:
        if use_gui:
            spin_thread = threading.Thread(
                target=rclpy.spin, args=(node,), daemon=True)
            spin_thread.start()
            try:
                run_gui(node)          # returns when the window is closed
            except Exception as e:
                node.get_logger().warn(f'GUI failed ({e}); logging only')
                spin_thread.join()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
