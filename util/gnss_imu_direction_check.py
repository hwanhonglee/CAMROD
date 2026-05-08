#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from ublox_msgs.msg import NavPVT
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped


def wrap_to_pi(angle_rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """
    Convert quaternion to yaw.
    Assumes ROS standard quaternion convention.
    yaw is in radians.
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class GnssImuDirectionCheck(Node):
    def __init__(self):
        super().__init__("gnss_imu_direction_check")

        # Parameters
        self.declare_parameter("navpvt_topic", "/sensing/gnss/ublox_gps_node/navpvt")
        self.declare_parameter("imu_topic", "/sensing/imu/data")
        self.declare_parameter("localization_pose_topic", "/localization/pose")
        self.declare_parameter("csv_path", "")

        # COG gating thresholds
        self.declare_parameter("min_speed_mps", 0.8)
        self.declare_parameter("max_s_acc_mps", 0.5)
        self.declare_parameter("max_head_acc_deg", 20.0)
        self.declare_parameter("max_h_acc_m", 1.0)
        self.declare_parameter("require_rtk_float_or_fix", False)

        self.navpvt_topic = self.get_parameter("navpvt_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.localization_pose_topic = self.get_parameter("localization_pose_topic").value

        self.min_speed_mps = float(self.get_parameter("min_speed_mps").value)
        self.max_s_acc_mps = float(self.get_parameter("max_s_acc_mps").value)
        self.max_head_acc_deg = float(self.get_parameter("max_head_acc_deg").value)
        self.max_h_acc_m = float(self.get_parameter("max_h_acc_m").value)
        self.require_rtk_float_or_fix = bool(
            self.get_parameter("require_rtk_float_or_fix").value
        )

        csv_path = self.get_parameter("csv_path").value
        if csv_path == "":
            now = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_path = os.path.expanduser(f"~/gnss_imu_direction_check_{now}.csv")
        else:
            csv_path = os.path.expanduser(csv_path)

        self.csv_path = csv_path
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "ros_time_sec",
            "gnss_i_tow",
            "fix_type",
            "flags",
            "carr_soln",
            "num_sv",
            "h_acc_m",
            "s_acc_mps",
            "head_acc_deg",
            "g_speed_mps",
            "vel_n_mps",
            "vel_e_mps",
            "ublox_heading_deg",
            "gnss_yaw_enu_deg",
            "imu_yaw_deg",
            "localization_yaw_deg",
            "gnss_minus_imu_deg",
            "gnss_minus_localization_deg",
            "imu_minus_localization_deg",
            "gyro_z_radps",
            "use_cog",
            "reject_reason",
        ])
        self.csv_file.flush()

        self.latest_imu_msg = None
        self.latest_imu_yaw = None
        self.latest_gyro_z = None

        self.latest_localization_pose_msg = None
        self.latest_localization_yaw = None

        self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            50,
        )

        self.create_subscription(
            PoseStamped,
            self.localization_pose_topic,
            self.localization_pose_callback,
            50,
        )

        self.create_subscription(
            NavPVT,
            self.navpvt_topic,
            self.navpvt_callback,
            50,
        )

        self.get_logger().info(f"Subscribed NavPVT:           {self.navpvt_topic}")
        self.get_logger().info(f"Subscribed IMU:              {self.imu_topic}")
        self.get_logger().info(f"Subscribed LocalizationPose: {self.localization_pose_topic}")
        self.get_logger().info(f"CSV logging to:              {self.csv_path}")

    def imu_callback(self, msg: Imu):
        q = msg.orientation

        # If orientation_covariance[0] == -1, orientation is usually invalid.
        if len(msg.orientation_covariance) > 0 and msg.orientation_covariance[0] == -1.0:
            self.latest_imu_msg = msg
            self.latest_imu_yaw = None
            self.latest_gyro_z = msg.angular_velocity.z
            return

        self.latest_imu_msg = msg
        self.latest_imu_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.latest_gyro_z = msg.angular_velocity.z

    def localization_pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        self.latest_localization_pose_msg = msg
        self.latest_localization_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def navpvt_callback(self, msg: NavPVT):
        if self.latest_imu_yaw is None:
            self.get_logger().warn("No valid IMU yaw yet.")
            return

        # u-blox NavPVT unit conversion
        vel_n_mps = msg.vel_n * 1e-3
        vel_e_mps = msg.vel_e * 1e-3
        g_speed_mps = msg.g_speed * 1e-3

        h_acc_m = msg.h_acc * 1e-3
        s_acc_mps = msg.s_acc * 1e-3
        head_acc_deg = msg.head_acc * 1e-5

        # In ublox_msgs/NavPVT, "heading" is heading of motion.
        # Unit: 1e-5 deg, 0 = North, 90 = East.
        ublox_heading_deg = msg.heading * 1e-5

        # RTK carrier solution from flags bits 6~7
        # 0: none, 1: float, 2: fixed
        carr_soln = (msg.flags >> 6) & 0b11

        # GNSS direction from velocity vector
        # Navigation frame heading: 0=N, 90=E
        cog_nav_rad = math.atan2(vel_e_mps, vel_n_mps)

        # Convert navigation heading to ROS ENU yaw
        # ROS ENU yaw: 0=East, +90=North
        gnss_yaw_enu = wrap_to_pi(math.pi / 2.0 - cog_nav_rad)

        imu_yaw = self.latest_imu_yaw

        if self.latest_localization_yaw is not None:
            localization_yaw = self.latest_localization_yaw
            gnss_minus_localization = wrap_to_pi(gnss_yaw_enu - localization_yaw)
            imu_minus_localization = wrap_to_pi(imu_yaw - localization_yaw)
            localization_yaw_deg_str = f"{rad2deg(localization_yaw):.3f}"
            gnss_minus_localization_deg_str = f"{rad2deg(gnss_minus_localization):.3f}"
            imu_minus_localization_deg_str = f"{rad2deg(imu_minus_localization):.3f}"
        else:
            localization_yaw = None
            localization_yaw_deg_str = "nan"
            gnss_minus_localization_deg_str = "nan"
            imu_minus_localization_deg_str = "nan"

        gnss_minus_imu = wrap_to_pi(gnss_yaw_enu - imu_yaw)

        use_cog = True
        reasons = []

        if msg.fix_type != 3:
            use_cog = False
            reasons.append("fix_type_not_3")

        if self.require_rtk_float_or_fix and carr_soln < 1:
            use_cog = False
            reasons.append("not_rtk_float_or_fix")

        if g_speed_mps < self.min_speed_mps:
            use_cog = False
            reasons.append("speed_low")

        if s_acc_mps > self.max_s_acc_mps:
            use_cog = False
            reasons.append("s_acc_high")

        if head_acc_deg > self.max_head_acc_deg:
            use_cog = False
            reasons.append("head_acc_high")

        if h_acc_m > self.max_h_acc_m:
            use_cog = False
            reasons.append("h_acc_high")

        reject_reason = "|".join(reasons) if reasons else "OK"

        ros_time_sec = self.get_clock().now().nanoseconds * 1e-9

        self.csv_writer.writerow([
            f"{ros_time_sec:.6f}",
            msg.i_tow,
            msg.fix_type,
            msg.flags,
            carr_soln,
            msg.num_sv,
            f"{h_acc_m:.3f}",
            f"{s_acc_mps:.3f}",
            f"{head_acc_deg:.3f}",
            f"{g_speed_mps:.3f}",
            f"{vel_n_mps:.3f}",
            f"{vel_e_mps:.3f}",
            f"{ublox_heading_deg:.3f}",
            f"{rad2deg(gnss_yaw_enu):.3f}",
            f"{rad2deg(imu_yaw):.3f}",
            localization_yaw_deg_str,
            f"{rad2deg(gnss_minus_imu):.3f}",
            gnss_minus_localization_deg_str,
            imu_minus_localization_deg_str,
            f"{self.latest_gyro_z:.6f}",
            int(use_cog),
            reject_reason,
        ])
        self.csv_file.flush()

        # Console output
        status = "USE" if use_cog else "SKIP"

        loc_yaw_print = (
            f"{rad2deg(localization_yaw):7.2f}deg"
            if localization_yaw is not None
            else "   nan deg"
        )

        gnss_loc_print = (
            f"{gnss_minus_localization_deg_str:>7}deg"
            if localization_yaw is not None
            else "    nan deg"
        )

        imu_loc_print = (
            f"{imu_minus_localization_deg_str:>7}deg"
            if localization_yaw is not None
            else "    nan deg"
        )

        self.get_logger().info(
            f"[{status}] "
            f"RTK={carr_soln} "
            f"fix={msg.fix_type} "
            f"sv={msg.num_sv} "
            f"spd={g_speed_mps:.2f}m/s "
            f"hAcc={h_acc_m:.2f}m "
            f"sAcc={s_acc_mps:.2f}m/s "
            f"headAcc={head_acc_deg:.1f}deg | "
            f"GNSS={rad2deg(gnss_yaw_enu):7.2f}deg "
            f"IMU={rad2deg(imu_yaw):7.2f}deg "
            f"LOC={loc_yaw_print} | "
            f"G-I={rad2deg(gnss_minus_imu):7.2f}deg "
            f"G-L={gnss_loc_print} "
            f"I-L={imu_loc_print} | "
            f"{reject_reason}"
        )

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = GnssImuDirectionCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Saved CSV: {node.csv_path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
