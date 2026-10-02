"""HH_261002 - Pure, fail-closed policy for an explicit post-manual road replan."""
from __future__ import annotations

import math


ROAD_STAGES = {
    "MOVING_TO_SITE": "outbound",
    "RECALL_TO_SITE_ROAD": "outbound",
    "RETURNING_TO_DROP_ZONE": "return",
}


def resume_reason(context, *, enabled, manual, active_generation, now_s,
                  pose_received_s, pose_stamp_s, platform_received_s,
                  platform_stamp_s, speed_mps, yaw_rate_rps, pose_valid,
                  platform_ready, ready, startup_pending, cancellation_ready,
                  settled, outside_station):
    """Never infer authorization from releasing a key, a timer, or reconnecting."""
    if not enabled:
        return "disabled"
    if not context:
        return "no_suspended_mission"
    if context.get("stage") not in {"outbound", "return"}:
        return "unsupported_stage"
    if active_generation:
        return "another_mission_active"
    if manual.get("armed") or manual.get("holding"):
        return "manual_drive_active"
    if startup_pending or not cancellation_ready or not settled:
        return "cancellation_pending"
    stamps = (now_s, pose_received_s, pose_stamp_s,
              platform_received_s, platform_stamp_s)
    if not all(isinstance(x, (int, float)) and math.isfinite(x) for x in stamps):
        return "stale_feedback"
    if any(stamp <= 0 or not 0 <= now_s - stamp <= 2.0 for stamp in stamps[1:]):
        return "stale_feedback"
    if not pose_valid:
        return "invalid_pose"
    if not outside_station:
        return "station_maneuver_required"
    if not all(math.isfinite(x) for x in (speed_mps, yaw_rate_rps)):
        return "invalid_motion_feedback"
    if abs(speed_mps) > 0.03 or abs(yaw_rate_rps) > 0.03:
        return "robot_not_stationary"
    if not platform_ready:
        return "platform_not_ready"
    if not ready:
        return "system_not_ready"
    return "ready"


def resume_payload(context, reason):
    context = context or {}
    return {
        "pending": bool(context), "token": context.get("token", ""),
        "site": context.get("site", ""), "intent": context.get("intent", ""),
        "stage": context.get("stage", ""), "can_resume": reason == "ready",
        "reason": reason,
        "message": (
            "Ready for explicit autonomous resume from the current position"
            if reason == "ready" else f"Autonomous resume blocked: {reason}"
        ),
    }
