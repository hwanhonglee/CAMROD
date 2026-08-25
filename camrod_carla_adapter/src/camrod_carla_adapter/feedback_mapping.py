"""Pure pose/frame mapping helpers for CARLA feedback."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlanarTransform:
    """Rigid transform from the CARLA ROS map into the CAMROD map."""

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(float(value)) for value in (
                self.x_m, self.y_m, self.yaw_rad)):
            raise ValueError("planar transform must be finite")


def normalize_quaternion(x, y, z, w):
    """Return a finite unit quaternion."""

    values = tuple(float(value) for value in (x, y, z, w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in values)


def multiply_quaternions(left, right):
    """Return ``left * right`` in ROS x/y/z/w order."""

    lx, ly, lz, lw = normalize_quaternion(*left)
    rx, ry, rz, rw = normalize_quaternion(*right)
    return normalize_quaternion(
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_from_yaw(yaw_rad):
    yaw_rad = float(yaw_rad)
    if not math.isfinite(yaw_rad):
        raise ValueError("yaw must be finite")
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def yaw_from_quaternion(quaternion):
    x, y, z, w = normalize_quaternion(*quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def transform_pose(position, quaternion, transform=None):
    """Apply one SE(2) map alignment while preserving height/roll/pitch."""

    tf = transform if transform is not None else PlanarTransform()
    px, py, pz = tuple(float(value) for value in position)
    if not all(math.isfinite(value) for value in (px, py, pz)):
        raise ValueError("position must be finite")
    source_q = normalize_quaternion(*quaternion)
    cosine = math.cos(tf.yaw_rad)
    sine = math.sin(tf.yaw_rad)
    output_position = (
        tf.x_m + cosine * px - sine * py,
        tf.y_m + sine * px + cosine * py,
        pz,
    )
    output_quaternion = multiply_quaternions(
        quaternion_from_yaw(tf.yaw_rad), source_q)
    return output_position, output_quaternion


def validate_motion_values(linear, angular):
    """Reject malformed odometry feedback before it reaches CAMROD."""

    values = tuple(float(value) for value in tuple(linear) + tuple(angular))
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError("motion vectors must contain six finite values")


def diagonal_covariance(diagonal):
    """Build a row-major covariance matrix from a finite positive diagonal."""

    diagonal = tuple(float(value) for value in diagonal)
    if not diagonal:
        raise ValueError("covariance diagonal must not be empty")
    if not all(math.isfinite(value) and value > 0.0 for value in diagonal):
        raise ValueError("covariance diagonal values must be finite and > 0")
    size = len(diagonal)
    covariance = [0.0] * (size * size)
    for index, value in enumerate(diagonal):
        covariance[index * size + index] = value
    return covariance
