"""Read the robot UI's existing snapshots and the requested camera frame."""

import json
from http.client import HTTPConnection, HTTPException
from urllib.parse import urlencode


MONITORING_VIEWS = frozenset({"overview", "mission", "vehicle", "diagnostics"})


def _get_local(port, path, *, limit=4 * 1024 * 1024, timeout=1.5):
    connection = HTTPConnection("127.0.0.1", port, timeout=timeout)
    try:
        connection.request("GET", path)
        with connection.getresponse() as response:
            if response.status != 200:
                return None
            body = response.read(limit + 1)
            return body if len(body) <= limit else None
    except (OSError, HTTPException, ValueError):
        return None
    finally:
        connection.close()


def _read_json(port, path):
    body = _get_local(port, path)
    if body is None:
        return None
    try:
        value = json.loads(body)
    except (ValueError, RecursionError):
        return None
    return value if isinstance(value, dict) else None


class ManagerMonitoring:
    def __init__(self, robot_port, video_port, front_topic, rear_topic):
        self.robot_port = robot_port
        self.video_port = video_port
        self.camera_topics = {"front": front_topic, "rear": rear_topic}

    def snapshot(self, view, platform, occupancy, sites, disabled=()):
        state = _read_json(self.robot_port, "/ui/state")
        if state is None:
            return None
        diagnostics = state.get("diagnostics", [])
        result = {
            "health": state.get("system_health"),
            "alert_count": sum(item.get("level", 0) >= 1 for item in diagnostics),
        }
        if view == "diagnostics":
            return {**result, "diagnostics": diagnostics}

        driving = _read_json(self.robot_port, "/api/driving") or {}
        mission = driving.get("mission") or {}
        result["mission"] = {
            "state": state.get("service_state_name"),
            "phase": state.get("mission_phase"),
            "description": state.get("service_state_description"),
            "site": mission.get("site") or state.get("mission_dispatch_site") or "",
            "intent": mission.get("intent") or state.get("mission_dispatch_intent"),
            "source": state.get("mission_source"),
            "active": mission.get("active", state.get("mission_dispatch_active")),
        }
        # HJ_261120 - The 차량 상태 tab embeds the Robot UI's read-only driving
        # view (http://<same host>:<robot_ui_port>/?view=driving-monitor).
        result["robot_ui_port"] = self.robot_port
        result["vehicle"] = {
            **platform,
            "battery_pct": (driving.get("battery") or {}).get("percentage"),
            "motion": driving.get("motion") or {},
        }
        if view == "overview":
            result.update(map=driving.get("base_map"), pose=driving.get("pose"),
                          route=driving.get("route"), sensors=driving.get("sensors"))
        if view == "mission":
            areas = (driving.get("base_map") or {}).get("areas") or []
            site_keys = {area.get("site"): area.get("id") for area in areas if area.get("site")}
            # HJ_261120 - disabled: sites the manager closed for delivery (site_access store).
            result.update(sites=sites, site_keys=site_keys, occupancy=occupancy,
                          disabled_sites=list(disabled), progress=driving.get("progress"))
        return result

    def camera_frame(self, camera):
        # web_video_server expects ROS topic separators as literal slashes.
        query = urlencode({"topic": self.camera_topics[camera], "type": "jpeg",
                           "width": 640, "height": 360, "quality": 50}, safe="/")
        frame = _get_local(self.video_port, f"/snapshot?{query}", limit=2 * 1024 * 1024)
        return frame if frame and frame.startswith(b"\xff\xd8") else None
