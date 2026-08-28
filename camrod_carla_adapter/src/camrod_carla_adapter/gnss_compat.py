"""
Build UI compatibility messages exclusively from live CARLA sensors.

CARLA publishes standard ``NavSatFix`` messages rather than the u-blox
receiver-specific messages used by CAMROD's hardware telemetry page.  The
helpers in this module perform a source-preserving representation conversion:
they never invent a trajectory, satellite count, RTK carrier state, or fix.
"""

from datetime import datetime, timezone
import math

from sensor_msgs.msg import NavSatFix, NavSatStatus
from ublox_msgs.msg import NavCOV, NavPVT, NavRELPOSNED9


EARTH_SEMIMAJOR_AXIS_M = 6378137.0
MILLISECONDS_PER_GPS_WEEK = 604800000


def normalize_degrees(value: float) -> float:
    """Normalize degrees to the half-open interval [-180, 180)."""
    return (float(value) + 180.0) % 360.0 - 180.0


def quaternion_yaw_rad(orientation) -> float:
    """Return finite ROS yaw from a quaternion-like message."""
    values = (
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion contains non-finite values')
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError('quaternion norm is zero')
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def geodetic_delta_ned_m(reference: NavSatFix, rover: NavSatFix):
    """Approximate the short-baseline NED vector from reference to rover."""
    values = (
        reference.latitude,
        reference.longitude,
        reference.altitude,
        rover.latitude,
        rover.longitude,
        rover.altitude,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('GNSS pair contains non-finite coordinates')
    mean_latitude = math.radians(
        0.5 * (float(reference.latitude) + float(rover.latitude))
    )
    north = EARTH_SEMIMAJOR_AXIS_M * math.radians(
        float(rover.latitude) - float(reference.latitude)
    )
    east = (
        EARTH_SEMIMAJOR_AXIS_M
        * math.cos(mean_latitude)
        * math.radians(float(rover.longitude) - float(reference.longitude))
    )
    down = float(reference.altitude) - float(rover.altitude)
    return north, east, down


def _stamp_seconds(stamp) -> float:
    value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return value if math.isfinite(value) and value >= 0.0 else 0.0


def _gps_tow_ms(timestamp_seconds: float) -> int:
    return int(round(timestamp_seconds * 1000.0)) % MILLISECONDS_PER_GPS_WEEK


def _bounded_int(value: float, minimum: int, maximum: int) -> int:
    if not math.isfinite(float(value)):
        return 0
    return min(maximum, max(minimum, int(round(float(value)))))


def _variance(covariance, index: int, fallback: float = 0.0) -> float:
    try:
        value = float(covariance[index])
    except (IndexError, TypeError, ValueError):
        return max(0.0, float(fallback))
    return value if math.isfinite(value) and value >= 0.0 else max(
        0.0, float(fallback)
    )


def build_navpvt(
    fix: NavSatFix,
    *,
    yaw_rad: float | None,
    velocity_ned_mps=(0.0, 0.0, 0.0),
    speed_accuracy_mps: float = 0.1,
    heading_accuracy_rad: float = math.radians(3.0),
) -> NavPVT:
    """
    Represent actual CARLA GNSS/odometry/IMU values as NAV-PVT.

    CARLA does not model satellites, DOP, differential corrections, or RTK
    carrier phase. Those fields remain explicitly unknown/no-solution.
    """
    timestamp = _stamp_seconds(fix.header.stamp)
    message = NavPVT()
    message.i_tow = _gps_tow_ms(timestamp)
    if timestamp > 0.0:
        utc = datetime.fromtimestamp(timestamp, tz=timezone.utc)
        message.year = utc.year
        message.month = utc.month
        message.day = utc.day
        message.hour = utc.hour
        message.min = utc.minute
        message.sec = utc.second
        message.valid = NavPVT.VALID_DATE | NavPVT.VALID_TIME

    has_fix = int(fix.status.status) != int(NavSatStatus.STATUS_NO_FIX)
    message.fix_type = NavPVT.FIX_TYPE_3D if has_fix else NavPVT.FIX_TYPE_NO_FIX
    message.flags = NavPVT.FLAGS_GNSS_FIX_OK if has_fix else 0
    message.num_sv = 0
    message.lon = _bounded_int(float(fix.longitude) * 1.0e7, -(2**31), 2**31 - 1)
    message.lat = _bounded_int(float(fix.latitude) * 1.0e7, -(2**31), 2**31 - 1)
    message.height = _bounded_int(float(fix.altitude) * 1000.0, -(2**31), 2**31 - 1)
    message.h_msl = message.height

    east_variance = _variance(fix.position_covariance, 0)
    north_variance = _variance(fix.position_covariance, 4)
    up_variance = _variance(fix.position_covariance, 8)
    message.h_acc = _bounded_int(
        math.sqrt(max(east_variance, north_variance)) * 1000.0,
        0,
        2**32 - 1,
    )
    message.v_acc = _bounded_int(
        math.sqrt(up_variance) * 1000.0, 0, 2**32 - 1
    )

    north, east, down = (float(value) for value in velocity_ned_mps)
    if not all(math.isfinite(value) for value in (north, east, down)):
        north = east = down = 0.0
    message.vel_n = _bounded_int(north * 1000.0, -(2**31), 2**31 - 1)
    message.vel_e = _bounded_int(east * 1000.0, -(2**31), 2**31 - 1)
    message.vel_d = _bounded_int(down * 1000.0, -(2**31), 2**31 - 1)
    message.g_speed = _bounded_int(
        math.hypot(north, east) * 1000.0, -(2**31), 2**31 - 1
    )
    message.s_acc = _bounded_int(
        max(0.0, float(speed_accuracy_mps)) * 1000.0, 0, 2**32 - 1
    )

    if yaw_rad is not None and math.isfinite(float(yaw_rad)):
        heading_deg = normalize_degrees(math.degrees(float(yaw_rad)))
        encoded_heading = _bounded_int(
            heading_deg * 1.0e5, -(2**31), 2**31 - 1
        )
        message.heading = encoded_heading
        message.head_veh = encoded_heading
        message.flags |= NavPVT.FLAGS_HEAD_VEH_VALID
        message.head_acc = _bounded_int(
            math.degrees(max(0.0, float(heading_accuracy_rad))) * 1.0e5,
            0,
            2**32 - 1,
        )
    # p_dop=0 and carrier bits=NO_SOLUTION mean unavailable, not a fake fix.
    return message


def build_navcov(
    fix: NavSatFix,
    *,
    velocity_covariance=(0.0, 0.0, 0.0),
) -> NavCOV:
    """Represent actual CARLA GNSS/odometry covariance as NAV-COV."""
    message = NavCOV()
    message.i_tow = _gps_tow_ms(_stamp_seconds(fix.header.stamp))
    message.version = 0
    message.pos_cov_valid = 1
    message.vel_cov_valid = 1
    # NavSatFix covariance is ENU while NAV-COV is NED.
    message.pos_cov_nn = _variance(fix.position_covariance, 4)
    message.pos_cov_ee = _variance(fix.position_covariance, 0)
    message.pos_cov_dd = _variance(fix.position_covariance, 8)
    message.pos_cov_ne = _variance(fix.position_covariance, 3)
    message.pos_cov_nd = 0.0
    message.pos_cov_ed = 0.0
    values = tuple(float(value) for value in velocity_covariance)
    if len(values) != 3 or not all(
        math.isfinite(value) and value >= 0.0 for value in values
    ):
        values = (0.0, 0.0, 0.0)
        message.vel_cov_valid = 0
    message.vel_cov_nn, message.vel_cov_ee, message.vel_cov_dd = values
    message.vel_cov_ne = 0.0
    message.vel_cov_nd = 0.0
    message.vel_cov_ed = 0.0
    return message


def _split_0_1_mm(value_m: float):
    total_units = _bounded_int(value_m * 10000.0, -(2**31) * 100, (2**31 - 1) * 100)
    coarse_cm = math.trunc(total_units / 100)
    high_precision = total_units - coarse_cm * 100
    return int(coarse_cm), int(high_precision)


def build_relposned(
    right_reference: NavSatFix,
    left_rover: NavSatFix,
    *,
    position_std_m: float,
    moving: bool,
) -> NavRELPOSNED9:
    """
    Build the UI moving-baseline sample from two actual CARLA GNSS actors.

    The physical pair is mounted right-reference to left-rover.  CAMROD's UI
    compares ``rel_pos_heading`` directly with ROS IMU yaw, so the known
    +90-degree left-baseline mounting is removed here and the field contains
    vehicle yaw in the same ROS convention used by that UI.
    """
    north, east, down = geodetic_delta_ned_m(right_reference, left_rover)
    length = math.sqrt(north * north + east * east + down * down)
    if not math.isfinite(length) or length <= 1.0e-4:
        raise ValueError('GNSS baseline is zero or invalid')

    message = NavRELPOSNED9()
    message.version = 0
    message.i_tow = _gps_tow_ms(_stamp_seconds(left_rover.header.stamp))
    message.rel_pos_n, message.rel_pos_hpn = _split_0_1_mm(north)
    message.rel_pos_e, message.rel_pos_hpe = _split_0_1_mm(east)
    message.rel_pos_d, message.rel_pos_hpd = _split_0_1_mm(down)
    message.rel_pos_length, message.rel_pos_hp_length = _split_0_1_mm(length)

    left_baseline_yaw_rad = math.atan2(north, east)
    vehicle_yaw_deg = normalize_degrees(
        math.degrees(left_baseline_yaw_rad - math.pi / 2.0)
    )
    message.rel_pos_heading = _bounded_int(
        vehicle_yaw_deg * 1.0e5, -(2**31), 2**31 - 1
    )
    standard_deviation = max(0.0, float(position_std_m))
    accuracy_units = _bounded_int(
        standard_deviation * 10000.0, 0, 2**32 - 1
    )
    message.acc_n = accuracy_units
    message.acc_e = accuracy_units
    message.acc_d = accuracy_units
    message.acc_length = accuracy_units
    heading_accuracy_rad = min(
        math.pi,
        math.sqrt(2.0) * standard_deviation / max(length, 1.0e-6),
    )
    message.acc_heading = _bounded_int(
        math.degrees(heading_accuracy_rad) * 1.0e5, 0, 2**32 - 1
    )
    message.flags = (
        NavRELPOSNED9.FLAGS_GNSS_FIX_OK
        | NavRELPOSNED9.FLAGS_REL_POS_VALID
        | NavRELPOSNED9.FLAGS_REL_POS_HEAD_VALID
    )
    if moving:
        message.flags |= NavRELPOSNED9.FLAGS_IS_MOVING
    # Differential/carrier-solution flags intentionally remain unset because
    # CARLA does not simulate an RTK receiver.
    return message
