from __future__ import annotations

import math
from typing import Iterable

from avg_msgs.msg import ModuleState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


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


def yaw_from_pose(pose: PoseStamped) -> float:
    q = pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def relative_xy(reference: PoseStamped, target: PoseStamped) -> tuple[float, float]:
    yaw = yaw_from_pose(reference)
    dx = target.pose.position.x - reference.pose.position.x
    dy = target.pose.position.y - reference.pose.position.y
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def distance_xy(a: PoseStamped, b: PoseStamped) -> float:
    dx = a.pose.position.x - b.pose.position.x
    dy = a.pose.position.y - b.pose.position.y
    return math.hypot(dx, dy)


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
) -> DiagnosticArray:
    array = DiagnosticArray()
    array.header.stamp = node.get_clock().now().to_msg()

    status = DiagnosticStatus()
    status.name = name
    status.hardware_id = "camrod_parking"
    status.level = diagnostic_level(level)
    status.message = message
    status.values.append(KeyValue(key="category", value=category))
    status.values.append(KeyValue(key="module", value=category))
    for key, value in values:
        status.values.append(KeyValue(key=str(key), value=str(value)))

    array.status.append(status)
    return array
