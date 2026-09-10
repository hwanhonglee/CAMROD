"""SOC decisions independent of ROS, motion authorization, and UI ownership.

35% admits a new mission; 25% remains in the finish-current-mission band.
Only strictly below 25% requests an urgent return. No SOC value authorizes
ignoring chassis faults, E-stop, localization, or obstacle checks.
"""

import math


POWER_SUPPLY_STATUS_FULL = 4


def battery_policy_snapshot(
    percentage, *, mission_minimum=35.0, urgent_threshold=25.0,
    urgent_latched=False, parking_method="auto", selected_method="",
):
    """Expose SOC policy, not measured charger contact or parking completion.

    ``charging_required`` is intent; ``parking_selected_method`` is the active
    controller choice. Neither field alone establishes successful charging.
    """
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
    """Return a stable UI completion signal for the current charge session."""
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
