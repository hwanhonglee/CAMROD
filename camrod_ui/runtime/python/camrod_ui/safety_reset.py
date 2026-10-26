"""HH_261024 - Read-only admission helpers for a stopped administrator reset.

These checks never authorize driving or clear an obstacle/EStop. The actual
control gate independently checks fresh motion and every hard safety source.
"""

import base64
import binascii
import hmac
import math


def admin_authorized(header, username, password):
    """Reject malformed credentials without recording passwords in responses."""
    if not isinstance(header, str) or not username or not password:
        return False
    scheme, _, encoded = header.partition(" ")
    if scheme.lower() != "basic" or not encoded or len(encoded) > 4096:
        return False
    try:
        decoded = base64.b64decode(encoded, validate=True).decode("utf-8")
        actual_user, separator, actual_password = decoded.partition(":")
    except (binascii.Error, UnicodeError, ValueError):
        return False
    if not separator:
        return False
    user_ok = hmac.compare_digest(actual_user.encode(), str(username).encode())
    password_ok = hmac.compare_digest(actual_password.encode(), str(password).encode())
    return user_ok and password_ok


def stationary_reset_blockers(observation, now_ros_s, now_monotonic_s):
    # HH_261024 - A sent zero or UI ARM state is not measured standstill.
    # Match the gate's stricter 0.5 s / 0.03 m/s / 0.03 rad/s reset boundary.
    if not isinstance(observation, dict):
        return ["motion_feedback_missing_or_stale"]
    try:
        stamp = float(observation["source_stamp_s"])
        received = float(observation["received_monotonic_s"])
        speed = float(observation["speed_mps"])
        yaw_rate = float(observation["yaw_rate_radps"])
        values = (stamp, received, speed, yaw_rate, now_ros_s, now_monotonic_s)
        if not all(math.isfinite(value) for value in values):
            return ["invalid_motion_feedback"]
        if (stamp <= 0.0 or not 0.0 <= now_monotonic_s - received <= 0.5
                or not 0.0 <= now_ros_s - stamp <= 0.5):
            return ["motion_feedback_missing_or_stale"]
        blockers = []
        if abs(speed) > 0.03 or abs(yaw_rate) > 0.03:
            blockers.append("robot_not_stationary")
        if observation["estop"] is not False:
            blockers.append("estop_active")
        if observation["error_code"] != 0 or observation["vehicle_state"] != 0:
            blockers.append("platform_fault_active")
        return blockers
    except (KeyError, TypeError, ValueError, OverflowError):
        return ["invalid_motion_feedback"]


def cancellations_complete(futures, expected_count, nav_status):
    # HH_261024 - ACK may mean still CANCELING; require observed idle/terminal
    # navigation as well, and never treat a missing cancel service as success.
    try:
        if (type(expected_count) is not int or expected_count <= 0
                or not isinstance(futures, (list, tuple))
                or len(futures) != expected_count
                or type(nav_status) is not int or nav_status not in {0, 4, 5, 6}):
            return False
        for future in futures:
            if not future.done():
                return False
            result = future.result()
            # A successful cancellation ACK can still name CANCELING goals.
            # Only a fresh empty cancel-all reply proves this server is idle.
            goals = result.goals_canceling
            if (type(result.return_code) is not int or result.return_code not in {0, 3}
                    or not isinstance(goals, (list, tuple)) or len(goals) != 0):
                return False
    except Exception:
        return False
    return True
