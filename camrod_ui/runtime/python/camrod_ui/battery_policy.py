"""SOC decisions independent of ROS, motion authorization, and UI ownership.

35% admits a new mission; 25% remains in the finish-current-mission band.
Only strictly below 25% requests an urgent return. No SOC value authorizes
ignoring chassis faults, E-stop, localization, or obstacle checks.
"""
# HH_260907 - Keep SOC decisions as policy signals, never as proof of safe motion or charging.

import math


# HH_261002 - Restore the BMS full-status signal used only for the operator's
# charging-complete presentation; mission admission and return thresholds stay unchanged.
POWER_SUPPLY_STATUS_FULL = 4


def battery_policy_snapshot(
    percentage, *, mission_minimum=35.0, urgent_threshold=25.0,
    urgent_latched=False, parking_method="auto", selected_method="",
):
    """Expose SOC policy, not measured charger contact or parking completion.

    ``charging_required`` is intent; ``parking_selected_method`` is the active
    controller choice. Neither field alone establishes successful charging.
    """
    # HH_260907 - Treat missing or invalid SOC as needing charge, not as an urgent return.
    try:
        soc = float(percentage)
        known = math.isfinite(soc) and 0.0 <= soc <= 100.0
    except (TypeError, ValueError):
        soc, known = -1.0, False
    return {
        "battery_return_urgent": bool(urgent_latched),
        "charging_required": not known or soc < mission_minimum,
        "parking_policy_mode": str(parking_method),
        "parking_selected_method": str(selected_method),
        "minimum_battery_percentage": float(mission_minimum),
        "urgent_return_battery_percentage": float(urgent_threshold),
    }


def urgent_return_required(percentage, *, urgent_threshold=25.0):
    # HH_260907 - Reserve urgent return for valid SOC strictly below the 25% band.
    try:
        soc = float(percentage)
    except (TypeError, ValueError):
        return False
    return math.isfinite(soc) and 0.0 <= soc < urgent_threshold


def battery_charge_complete(
    percentage,
    *,
    charging=False,
    power_supply_status=0,
    previously_complete=False,
):
    """Report a confirmed full battery without mistaking docking for charging.

    HH_261002 - Preserve a completed charge during the same charging session.
    A merely parked robot, or a 100% SOC reading without charging/full BMS
    status, must not present successful charging to the operator.
    """
    if int(power_supply_status) == POWER_SUPPLY_STATUS_FULL:
        return True
    if not charging:
        return False
    if previously_complete:
        return True
    try:
        soc = float(percentage)
    except (TypeError, ValueError):
        return False
    return math.isfinite(soc) and soc >= 100.0
