from __future__ import annotations

import math
from typing import Iterable

from avg_msgs.msg import AvgPoseStamped, AvgTwist, ModuleState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped as RosPoseStamped
from geometry_msgs.msg import Twist as RosTwist
from rclpy.node import Node

# HH_260720 - Keep shared control math, diagnostics, and boundary conversions in one module.


def pose_from_ros(message: RosPoseStamped) -> AvgPoseStamped:
    """Convert an operator ROS pose into the generated CAMROD contract."""
    # HH_260720 - Copy every field explicitly so this conversion cannot act as an alias.
    output = AvgPoseStamped()
    output.header.stamp = message.header.stamp
    output.header.frame_id = message.header.frame_id
    output.pose.position.x = message.pose.position.x
    output.pose.position.y = message.pose.position.y
    output.pose.position.z = message.pose.position.z
    output.pose.orientation.x = message.pose.orientation.x
    output.pose.orientation.y = message.pose.orientation.y
    output.pose.orientation.z = message.pose.orientation.z
    output.pose.orientation.w = message.pose.orientation.w
    return output


def twist_from_ros(message: RosTwist) -> AvgTwist:
    """Convert a Nav2 ROS Twist into the generated control contract."""
    # HH_260720 - Copy boundary values explicitly; no type alias is involved.
    output = AvgTwist()
    output.linear.x = message.linear.x
    output.linear.y = message.linear.y
    output.linear.z = message.linear.z
    output.angular.x = message.angular.x
    output.angular.y = message.angular.y
    output.angular.z = message.angular.z
    return output


def twist_to_ros(message: AvgTwist) -> RosTwist:
    """Convert the generated final command for the Ranger ROS boundary."""
    # HH_260720 - Keep Ranger's standard Twist requirement at one named boundary.
    output = RosTwist()
    output.linear.x = message.linear.x
    output.linear.y = message.linear.y
    output.linear.z = message.linear.z
    output.angular.x = message.angular.x
    output.angular.y = message.angular.y
    output.angular.z = message.angular.z
    return output


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def diagnostic_level(value: object) -> bytes:
    # HH_260617: ROS Humble Python may expose uint8 constants as bytes.
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 1:
            return bytes(value)
        if len(value) > 1:
            return bytes([value[0]])
        return b"\x00"
    return bytes([int(value) & 0xFF])


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_pose(pose: AvgPoseStamped) -> float:
    q = pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def relative_xy(reference: AvgPoseStamped, target: AvgPoseStamped) -> tuple[float, float]:
    yaw = yaw_from_pose(reference)
    dx = target.pose.position.x - reference.pose.position.x
    dy = target.pose.position.y - reference.pose.position.y
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def make_module_state(node: Node, module_name: str, level: int, message: str) -> ModuleState:
    msg = ModuleState()
    msg.stamp = node.get_clock().now().to_msg()
    msg.module_name = module_name
    msg.level = int(level)
    msg.message = message
    return msg


def make_diagnostics(
    node: Node,
    name: str,
    category: str,
    level: int,
    message: str,
    values: Iterable[tuple[str, str]] = (),
    hardware_id: str = "camrod_control",
) -> DiagnosticArray:
    array = DiagnosticArray()
    array.header.stamp = node.get_clock().now().to_msg()

    status = DiagnosticStatus()
    status.name = name
    status.hardware_id = hardware_id
    status.level = diagnostic_level(level)
    status.message = message
    status.values.append(KeyValue(key="category", value=category))
    status.values.append(KeyValue(key="module", value=category))
    for key, value in values:
        status.values.append(KeyValue(key=str(key), value=str(value)))

    array.status.append(status)
    return array
