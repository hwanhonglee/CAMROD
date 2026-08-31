#!/usr/bin/env python3
# flake8: noqa
"""Run (or plan) a CAMROD camping-site round-trip matrix in CARLA.

The default action is deliberately read-only: it loads the active CAMROD map
configuration and writes a plan.  ``--run`` is required before this program
can call the production UI REST endpoints.  The runner never publishes a
motion command, a goal pose, an initial pose, or a CARLA teleport.  Movement
is therefore the same UI -> CAMROD -> controller path an operator uses.

ROS imports are intentionally deferred until the live path is selected.  The
configuration, state-machine, and report helpers can consequently be tested
on a plain Python installation.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import hashlib
import sys
import tempfile
import time
from typing import Any, Callable, Iterable, Mapping, Sequence
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urlparse
from urllib.request import Request, urlopen


DEFAULT_SITES = tuple(f"B{i}" for i in range(1, 14))
WAITING_FOR_RETURN_REQUEST = 11
WAITING_FOR_CHARGING = 12
CHARGING = 13
DROP_ZONE_PARKING = 10
FAILURE_STATES = {16}
SITE_TYPE_RE = re.compile(r"^camping_site_(\d+)$", re.IGNORECASE)
STATIONARY_DROP_ZONE_STATES = {0, WAITING_FOR_CHARGING, CHARGING}
SITE_PHASES = {
    "turnaround": (
        "CRAB_IN", "ROTATE_180", "UNLOAD_WAIT", "WAIT_RETURN",
        "ALIGN_RETRACE_YAW", "CRAB_OUT", "DONE",
    ),
    "roadside_stop": (
        "CRAB_IN", "UNLOAD_WAIT", "WAIT_RETURN", "CRAB_OUT", "DONE",
    ),
}


class MatrixError(RuntimeError):
    """A fail-closed matrix validation or runtime error."""


@dataclass(frozen=True)
class Site:
    """One site from the active ``camping_sites.yaml``."""

    key: str
    source_id: str
    x_m: float
    y_m: float
    z_m: float
    yaw_deg: float
    service_mode: str


@dataclass(frozen=True)
class DropZone:
    """The active drop-zone reference used for final return error."""

    source_id: str
    x_m: float
    y_m: float
    z_m: float
    yaw_deg: float


@dataclass
class SiteResult:
    """Durable evidence for one outbound/return attempt."""

    site: str
    source_id: str
    service_mode: str
    target_pose: dict[str, float]
    status: str = "NOT_ATTEMPTED"
    started_at_utc: str = ""
    finished_at_utc: str = ""
    elapsed_s: float = 0.0
    milestones: list[dict[str, Any]] = field(default_factory=list)
    failure_reason: str = ""
    start_localization_pose: dict[str, Any] = field(default_factory=dict)
    arrival_localization_pose: dict[str, Any] = field(default_factory=dict)
    site_phase_sequence: list[str] = field(default_factory=list)
    service_state_sequence: list[str] = field(default_factory=list)
    drop_zone_phase_sequence: list[str] = field(default_factory=list)
    parking_phase_sequence: list[str] = field(default_factory=list)
    motion_metrics: dict[str, Any] = field(default_factory=dict)
    final_service_state: dict[str, Any] = field(default_factory=dict)
    final_parking_status: dict[str, Any] = field(default_factory=dict)
    final_drop_zone_status: dict[str, Any] = field(default_factory=dict)
    final_gate_status: dict[str, Any] = field(default_factory=dict)
    final_physical_four_wheel_status: dict[str, Any] = field(default_factory=dict)
    final_localization_pose: dict[str, Any] = field(default_factory=dict)
    final_carla_odometry: dict[str, Any] = field(default_factory=dict)
    drop_zone_error_m: float | None = None
    parking_confirmed: bool = False
    charging_confirmed: bool = False


def utc_now() -> str:
    """Return an ISO-8601 UTC timestamp."""
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise MatrixError(f"{name} must be numeric, got {value!r}") from None
    if not math.isfinite(number):
        raise MatrixError(f"{name} must be finite, got {value!r}")
    return number


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MatrixError(f"{name} must be a mapping")
    return value


def load_yaml(path: Path, key: str) -> list[Mapping[str, Any]]:
    """Load one required list from an active YAML file."""
    if not path.is_file():
        raise MatrixError(f"active configuration is missing: {path}")
    try:
        import yaml  # deferred so plan helpers remain importable
        root = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as error:
        raise MatrixError(f"cannot read {path}: {error}") from None
    except ImportError:
        raise MatrixError("PyYAML is required to load active CAMROD YAML") from None
    document = _mapping(root, str(path))
    values = document.get(key)
    if not isinstance(values, list) or not values:
        raise MatrixError(f"{path} must contain a non-empty {key}: list")
    return [_mapping(item, f"{path}:{key}[{index}]") for index, item in enumerate(values)]


def load_sites(path: Path) -> dict[str, Site]:
    """Parse B1..B13 from the active CAMROD camping-site configuration."""
    result: dict[str, Site] = {}
    for item in load_yaml(path, "camping_sites"):
        source_id = str(item.get("id", "")).strip()
        match = SITE_TYPE_RE.fullmatch(str(item.get("type", "")).strip())
        if not source_id or match is None:
            raise MatrixError(f"invalid campsite id/type in {path}: {item!r}")
        key = f"B{int(match.group(1))}"
        if key in result:
            raise MatrixError(f"duplicate active campsite {key} in {path}")
        result[key] = Site(
            key=key,
            source_id=source_id,
            x_m=_finite(item.get("x"), f"{key}.x"),
            y_m=_finite(item.get("y"), f"{key}.y"),
            z_m=_finite(item.get("z", 0.0), f"{key}.z"),
            yaw_deg=_finite(item.get("yaw_deg", 0.0), f"{key}.yaw_deg"),
            service_mode=str(item.get("service_mode", "")).strip(),
        )
        if result[key].service_mode not in SITE_PHASES:
            raise MatrixError(
                f"{key}.service_mode must be one of {sorted(SITE_PHASES)}, "
                f"got {result[key].service_mode!r}"
            )
    return result


def load_drop_zone(path: Path) -> DropZone:
    """Load exactly one active drop-zone reference."""
    values = load_yaml(path, "drop_zones")
    if len(values) != 1:
        raise MatrixError(f"expected exactly one active drop zone in {path}, found {len(values)}")
    item = values[0]
    source_id = str(item.get("id", "")).strip()
    if not source_id:
        raise MatrixError(f"drop-zone id is empty in {path}")
    return DropZone(
        source_id=source_id,
        x_m=_finite(item.get("x"), "drop_zone.x"),
        y_m=_finite(item.get("y"), "drop_zone.y"),
        z_m=_finite(item.get("z", 0.0), "drop_zone.z"),
        yaw_deg=_finite(item.get("yaw_deg", 0.0), "drop_zone.yaw_deg"),
    )


def parse_site_selection(value: str | None) -> tuple[str, ...]:
    """Parse and validate ``--sites`` without touching ROS or HTTP."""
    if not value:
        return DEFAULT_SITES
    requested = tuple(part.strip().upper() for part in value.split(",") if part.strip())
    if not requested:
        raise MatrixError("--sites must contain at least one site")
    invalid = [site for site in requested if not re.fullmatch(r"B(?:[1-9]|1[0-3])", site)]
    if invalid:
        raise MatrixError(f"invalid site selection {invalid!r}; expected B1..B13")
    if len(set(requested)) != len(requested):
        raise MatrixError("--sites must not contain duplicates")
    return requested


def pose_error_m(pose: Mapping[str, Any], drop_zone: DropZone) -> float | None:
    """Compute planar localization-to-drop-zone error from a normalized pose."""
    try:
        x = _finite(pose["x_m"], "pose.x_m")
        y = _finite(pose["y_m"], "pose.y_m")
    except (KeyError, MatrixError):
        return None
    return math.hypot(x - drop_zone.x_m, y - drop_zone.y_m)


def contains_ordered_subsequence(values: Sequence[str], expected: Sequence[str]) -> bool:
    """Return true when every expected phase appears in order."""
    position = 0
    for value in values:
        if position < len(expected) and value == expected[position]:
            position += 1
    return position == len(expected)


def arrival_ready_for_return(
    snapshot: Mapping[str, Any], expected_phases: Sequence[str]
) -> bool:
    """Require both lifecycle and controller evidence before UI return.

    ``/service/state`` and the camping-site controller status are independent
    ROS topics.  The lifecycle can therefore publish
    ``WAITING_FOR_RETURN_REQUEST`` one callback before the controller's
    ``WAIT_RETURN`` status is observed.  Treating the lifecycle sample alone
    as arrival would race the evidence observer and could issue Return before
    the controller is observably ready.  Malformed or incomplete snapshots
    fail closed.
    """
    service_state = snapshot.get("service_state")
    sequences = snapshot.get("sequences")
    if not isinstance(service_state, Mapping) or not isinstance(sequences, Mapping):
        return False
    if service_state.get("state") != WAITING_FOR_RETURN_REQUEST:
        return False
    try:
        wait_return_index = expected_phases.index("WAIT_RETURN") + 1
    except ValueError:
        return False
    site_phases = sequences.get("site_phases")
    if not isinstance(site_phases, Sequence) or isinstance(site_phases, (str, bytes)):
        return False
    return contains_ordered_subsequence(
        site_phases, expected_phases[:wait_return_index]
    )


def load_sensor_audit(path: Path, *, maximum_age_s: float = 120.0) -> dict[str, Any]:
    """Validate and bind one fresh fail-closed CARLA sensor-source audit."""
    if not path.is_file():
        raise MatrixError(f"sensor-source audit is missing: {path}")
    age_s = time.time() - path.stat().st_mtime
    if not math.isfinite(age_s) or age_s < -5.0 or age_s > maximum_age_s:
        raise MatrixError(
            f"sensor-source audit is not fresh: age={age_s:.3f}s, "
            f"maximum={maximum_age_s:.3f}s"
        )
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise MatrixError(
            f"sensor-source audit is unreadable: {path}: {error}"
        ) from None
    if not isinstance(document, dict):
        raise MatrixError(f"sensor-source audit root must be an object: {path}")
    summary = document.get("summary")
    if not isinstance(summary, dict):
        raise MatrixError(f"sensor-source audit summary is missing: {path}")
    exact = {
        "streams_checked": 32,
        "stream_failures": 0,
        "actors_checked": 13,
        "actor_failures": 0,
    }
    mismatches = [
        f"{key}={summary.get(key)!r} expected {expected!r}"
        for key, expected in exact.items()
        if summary.get(key) != expected
    ]
    if document.get("status") != "PASS" or document.get("passed") is not True:
        mismatches.append(
            f"status/passed={document.get('status')!r}/"
            f"{document.get('passed')!r}"
        )
    if mismatches:
        raise MatrixError("sensor-source audit rejected: " + ", ".join(mismatches))
    return {
        "path": str(path.resolve()),
        "sha256": sha256_file(path),
        "status": "PASS",
        "summary": exact,
        "age_s_at_matrix_start": round(max(0.0, age_s), 3),
    }


def build_plan(sites: Mapping[str, Site], selected: Iterable[str], drop_zone: DropZone,
               *, output: Path, config_paths: Mapping[str, Path]) -> dict[str, Any]:
    """Build a report that contains no runtime claim and no motion side effect."""
    selected_tuple = tuple(selected)
    selected_set = set(selected_tuple)
    results = []
    for key in selected_tuple:
        site = sites[key]
        results.append(asdict(SiteResult(
            site=site.key,
            source_id=site.source_id,
            service_mode=site.service_mode,
            target_pose={"x_m": site.x_m, "y_m": site.y_m, "z_m": site.z_m, "yaw_deg": site.yaw_deg},
        )))
    return {
        "schema": "camrod.virtual_carla.camping_site_matrix.v1",
        "status": "PLAN_ONLY",
        "created_at_utc": utc_now(),
        "scope": {
            "selected_sites": list(selected_tuple),
            "unattempted_sites": [key for key in sites if key not in selected_set],
            "motion_commands_sent": False,
            "pose_teleport_used": False,
            "fake_sensor_data_used": False,
            "ui_endpoints": {"dispatch": "POST /ui/destination?site=Bx&run=true", "return": "POST /ui/manual_return", "stop": "POST /ui/stop"},
        },
        "active_configuration": {
            "camping_sites_yaml": str(config_paths["camping_sites"]),
            "drop_zones_yaml": str(config_paths["drop_zones"]),
            "sha256": {name: sha256_file(path) for name, path in config_paths.items()},
            "drop_zone": asdict(drop_zone),
        },
        "sites": results,
        "runtime_observation_contract": observation_contract(),
        "report_path": str(output),
    }


def sha256_file(path: Path) -> str:
    """Return the provenance digest for an active configuration file."""
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as error:
        raise MatrixError(f"cannot hash active configuration {path}: {error}") from None
    return digest.hexdigest()


def observation_contract() -> dict[str, Any]:
    """Document every live source the runner observes."""
    return {
        "required_topics": [
            "/service/state", "/localization/pose", "/control/cmd_vel",
            "/carla/ego_vehicle/odometry",
            "/carla/ego_vehicle/physical_four_wheel_status",
            "/control/camping_site_maneuver_controller/status",
        ],
        "status_topics": [
            "/parking/reverse_parking_controller/status",
            "/parking/apriltag_parking_controller/status",
            "/control/drop_zone_maneuver_controller/status",
            "/control/cmd_vel_safety_gate/status",
        ],
        "sensor_policy": "a fresh 32-stream/13-actor CARLA source audit is required; this runner neither publishes nor synthesizes sensor data",
        "readiness_policy": "physical actor id and strict PHYSX_FOUR_WHEEL_STEERING readiness must remain unchanged for the whole mission",
    }


def update_motion_command_scope(
    report: dict[str, Any], snapshot: Mapping[str, Any]
) -> None:
    """Latch live nonzero-command evidence into the report-level scope."""
    motion = snapshot.get("motion_metrics") or {}
    observed = bool(motion.get("motion_command_observed"))
    scope = report.setdefault("scope", {})
    scope["motion_commands_sent"] = bool(
        scope.get("motion_commands_sent", False) or observed
    )


def write_json_atomic(
    path: Path,
    document: Mapping[str, Any],
    *,
    create_only: bool = False,
) -> None:
    """Atomically replace a JSON report, preserving partial progress."""
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if create_only:
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError:
            raise MatrixError(f"refusing to overwrite existing report: {path}") from None
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(
                    document,
                    stream,
                    indent=2,
                    ensure_ascii=False,
                    sort_keys=False,
                )
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
        except BaseException:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
            raise
        return
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, ensure_ascii=False, sort_keys=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


class UIClient:
    """Small production UI client with strict success handling."""

    def __init__(self, base_url: str, timeout_s: float = 10.0):
        self.base_url = base_url.rstrip("/")
        self.timeout_s = timeout_s
        parsed = urlparse(self.base_url)
        if (
            parsed.scheme != "http"
            or parsed.hostname not in {"127.0.0.1", "localhost"}
            or parsed.username is not None
            or parsed.password is not None
            or parsed.query
            or parsed.fragment
        ):
            raise MatrixError(
                "--ui-url must be a local http://127.0.0.1 or "
                "http://localhost endpoint without credentials/query/fragment"
            )

    def post(self, path: str, query: Mapping[str, str] | None = None) -> dict[str, Any]:
        url = f"{self.base_url}{path}"
        if query:
            url += "?" + urlencode(query)
        request = Request(url, method="POST", headers={"Accept": "application/json"})
        try:
            with urlopen(request, timeout=self.timeout_s) as response:  # nosec B310 - operator-selected local UI URL
                body = response.read().decode("utf-8")
                status = int(response.status)
        except (HTTPError, URLError, TimeoutError, OSError) as error:
            raise MatrixError(f"UI request failed {url}: {error}") from None
        if status < 200 or status >= 300:
            raise MatrixError(f"UI request returned HTTP {status}: {url}")
        try:
            decoded = json.loads(body) if body else {}
        except json.JSONDecodeError:
            raise MatrixError(f"UI response was not JSON: {url}") from None
        if not isinstance(decoded, dict):
            raise MatrixError(f"UI response was not an object: {url}")
        if decoded.get("success") is False:
            raise MatrixError(f"UI rejected {url}: {decoded}")
        return decoded

    def dispatch(self, site: str) -> dict[str, Any]:
        return self.post("/ui/destination", {"site": site, "run": "true"})

    def request_return(self) -> dict[str, Any]:
        return self.post("/ui/manual_return")

    def stop(self) -> dict[str, Any]:
        return self.post("/ui/stop")


def _attr(message: Any, name: str, default: Any = None) -> Any:
    if isinstance(message, Mapping):
        return message.get(name, default)
    return getattr(message, name, default)


def _to_jsonable(message: Any, names: Sequence[str]) -> dict[str, Any]:
    """Copy only stable scalar fields from a ROS message for evidence."""
    result: dict[str, Any] = {}
    for name in names:
        value = _attr(message, name)
        if value is None:
            continue
        if isinstance(value, (str, int, float, bool)):
            result[name] = value
    return result


def validate_physical_status(status: Any, expected_actor_id: int | None = None) -> int:
    """Reject every status except the bound, ready physical CARLA backend."""
    required = {
        "ready": True,
        "physical_gate_accepted": True,
        "physx_substep_control_verified": True,
        "independent_wheel_drive_available": True,
        "motion_backend": "PHYSX_FOUR_WHEEL_STEERING",
    }
    mismatches = []
    for name, expected in required.items():
        actual = _attr(status, name)
        if actual != expected:
            mismatches.append(f"{name}={actual!r} expected {expected!r}")
    try:
        actor_id = int(_attr(status, "actor_id", 0))
    except (TypeError, ValueError):
        actor_id = 0
    if actor_id <= 0:
        mismatches.append(f"actor_id={actor_id!r} expected >0")
    if expected_actor_id is not None and actor_id != expected_actor_id:
        mismatches.append(f"actor_id changed {expected_actor_id}->{actor_id}")
    for name in (
        "physical_manifest_sha256",
        "production_authorization_sha256",
        "ros_integration_sha256",
        "imported_libcarla_sha256",
    ):
        value = str(_attr(status, name, "")).strip().lower()
        if re.fullmatch(r"[0-9a-f]{64}", value) is None:
            mismatches.append(f"{name} is not a SHA-256")
    try:
        torque_cap = float(_attr(status, "wheel_torque_safety_cap_nm", math.nan))
    except (TypeError, ValueError):
        torque_cap = math.nan
    if not math.isfinite(torque_cap) or abs(torque_cap - 20.0) > 1.0e-6:
        mismatches.append(
            f"wheel_torque_safety_cap_nm={torque_cap!r} expected 20.0"
        )
    if mismatches:
        raise MatrixError("physical CARLA readiness lost: " + ", ".join(mismatches))
    return actor_id


class RosObservation:
    """ROS 2 observer; imports happen only when ``--run`` is selected."""

    def __init__(self, role_name: str, *, expected_actor_id: int | None = None):
        try:
            import rclpy
            from rclpy.qos import QoSProfile, QoSReliabilityPolicy
            from avg_msgs.msg import (
                AvgPoseStamped,
                AvgServiceState,
                AvgTwist,
                ModuleState,
            )
            from nav_msgs.msg import Odometry
            from carla_extended_ackermann_msgs.msg import PhysicalFourWheelStatus
        except (ImportError, ModuleNotFoundError) as error:
            raise MatrixError(f"live ROS interfaces unavailable: {error}") from None
        self.rclpy = rclpy
        rclpy.init(args=[])
        self.node = rclpy.create_node("camrod_carla_camping_site_matrix")
        self.latest: dict[str, Any] = {
            "service_state": None,
            "pose": None,
            "cmd_vel": None,
            "carla_odometry": None,
            "physical": None,
            "site": None,
            "parking": {},
            "drop_zone": None,
            "gate": None,
        }
        self.expected_actor_id = expected_actor_id
        self._physical_received_monotonic: float | None = None
        self._physical_identity: dict[str, Any] | None = None
        self._fatal_error = ""
        self._service_state_ids: list[int] = []
        self._service_state_names: list[str] = []
        self._site_phases: list[str] = []
        self._parking_phases: list[str] = []
        self._drop_zone_phases: list[str] = []
        self._gate_states: list[str] = []
        self._cmd_vel_samples = 0
        self._cmd_vel_max_abs = 0.0
        self._motion_command_observed = False
        self._carla_odom_samples = 0
        self._carla_odom_distance_m = 0.0
        self._last_carla_odom_xy: tuple[float, float] | None = None
        self._physical_samples = 0
        qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(AvgServiceState, "/service/state", lambda msg: self._service(msg), qos)
        self.node.create_subscription(AvgPoseStamped, "/localization/pose", lambda msg: self._pose(msg), qos)
        self.node.create_subscription(AvgTwist, "/control/cmd_vel", lambda msg: self._cmd(msg), qos)
        self.node.create_subscription(Odometry, "/carla/ego_vehicle/odometry", lambda msg: self._odom(msg), qos)
        self.node.create_subscription(PhysicalFourWheelStatus, f"/carla/{role_name}/physical_four_wheel_status", lambda msg: self._physical(msg), qos)
        self.node.create_subscription(ModuleState, "/control/camping_site_maneuver_controller/status", lambda msg: self._site(msg), qos)
        self.node.create_subscription(ModuleState, "/parking/reverse_parking_controller/status", lambda msg: self._parking("reverse", msg), qos)
        self.node.create_subscription(ModuleState, "/parking/apriltag_parking_controller/status", lambda msg: self._parking("apriltag", msg), qos)
        self.node.create_subscription(ModuleState, "/control/drop_zone_maneuver_controller/status", lambda msg: self._drop_zone(msg), qos)
        self.node.create_subscription(ModuleState, "/control/cmd_vel_safety_gate/status", lambda msg: self._gate(msg), qos)

    def _service(self, msg: Any) -> None:
        state = int(_attr(msg, "state", -1))
        state_name = str(_attr(msg, "state_name", "")).strip()
        self.latest["service_state"] = {
            "state": state,
            "state_name": state_name,
            "description": str(_attr(msg, "description", "")),
        }
        self._append_changed(self._service_state_ids, state)
        self._append_changed(self._service_state_names, state_name or str(state))
        if state in FAILURE_STATES and self._site_phases:
            self._fatal_error = (
                f"service lifecycle entered fail-closed state {state_name or state}"
            )

    def _pose(self, msg: Any) -> None:
        pose = _attr(msg, "pose", msg)
        position = _attr(pose, "position", _attr(_attr(pose, "pose", None), "position", None))
        if position is not None:
            values = tuple(
                float(_attr(position, name, math.nan)) for name in ("x", "y", "z")
            )
            if not all(math.isfinite(value) for value in values):
                self._fatal_error = "localization pose contains non-finite values"
                return
            self.latest["pose"] = {
                "x_m": values[0], "y_m": values[1], "z_m": values[2]
            }

    def _cmd(self, msg: Any) -> None:
        linear = _attr(msg, "linear", None); angular = _attr(msg, "angular", None)
        values = (
            float(_attr(linear, "x", math.nan)),
            float(_attr(linear, "y", math.nan)),
            float(_attr(angular, "z", math.nan)),
        )
        if not all(math.isfinite(value) for value in values):
            self._fatal_error = "final cmd_vel contains non-finite values"
            return
        self.latest["cmd_vel"] = {
            "linear_x_mps": values[0],
            "linear_y_mps": values[1],
            "angular_z_radps": values[2],
        }
        magnitude = sum(abs(value) for value in values)
        self._cmd_vel_samples += 1
        self._cmd_vel_max_abs = max(self._cmd_vel_max_abs, magnitude)
        self._motion_command_observed = (
            self._motion_command_observed or magnitude > 0.03
        )

    def _odom(self, msg: Any) -> None:
        pose = _attr(_attr(msg, "pose", None), "pose", None); twist = _attr(_attr(msg, "twist", None), "twist", None)
        position = _attr(pose, "position", None); linear = _attr(twist, "linear", None)
        if position is not None:
            x = float(_attr(position, "x", math.nan))
            y = float(_attr(position, "y", math.nan))
            z = float(_attr(position, "z", math.nan))
            vx = float(_attr(linear, "x", math.nan))
            vy = float(_attr(linear, "y", math.nan))
            if not all(math.isfinite(value) for value in (x, y, z, vx, vy)):
                self._fatal_error = "CARLA odometry contains non-finite values"
                return
            self.latest["carla_odometry"] = {
                "x_m": x, "y_m": y, "z_m": z,
                "speed_mps": math.hypot(vx, vy),
            }
            if self._last_carla_odom_xy is not None:
                step = math.hypot(
                    x - self._last_carla_odom_xy[0],
                    y - self._last_carla_odom_xy[1],
                )
                if step > 2.0:
                    self._fatal_error = (
                        f"CARLA odometry discontinuity {step:.3f} m; "
                        "teleport/reset is not accepted"
                    )
                    return
                self._carla_odom_distance_m += step
            self._last_carla_odom_xy = (x, y)
            self._carla_odom_samples += 1

    def _physical(self, msg: Any) -> None:
        validate_physical_status(msg, self.expected_actor_id)
        self.latest["physical"] = _to_jsonable(
            msg,
            (
                "actor_id", "ready", "api_version", "motion_backend",
                "physical_gate_accepted", "physical_manifest_sha256",
                "physx_substep_control_verified",
                "physx_substep_threshold_mps", "physx_substep_low_count",
                "physx_substep_high_count", "production_authorization_sha256",
                "ros_integration_sha256", "last_applied_sequence",
                "independent_wheel_drive_available",
                "wheel_torque_safety_cap_nm", "carla_python_origin",
                "imported_libcarla_path", "imported_libcarla_sha256", "reason",
            ),
        )
        self._physical_received_monotonic = time.monotonic()
        self._physical_samples += 1
        if self.expected_actor_id is None:
            self.expected_actor_id = int(self.latest["physical"]["actor_id"])

        identity_keys = (
            "actor_id", "api_version", "motion_backend",
            "physical_manifest_sha256", "production_authorization_sha256",
            "ros_integration_sha256", "wheel_torque_safety_cap_nm",
            "carla_python_origin", "imported_libcarla_path",
            "imported_libcarla_sha256",
        )
        identity = {key: self.latest["physical"].get(key) for key in identity_keys}
        if self._physical_identity is None:
            self._physical_identity = identity
        elif identity != self._physical_identity:
            self._fatal_error = "physical CARLA actor/gate/runtime identity changed"

    def _site(self, msg: Any) -> None:
        self.latest["site"] = _to_jsonable(
            msg, ("level", "operating_state", "message")
        )
        phase = str(_attr(msg, "operating_state", "")).strip()
        self._append_changed(self._site_phases, phase)
        if phase == "ERROR" or int(_attr(msg, "level", 0)) >= 2:
            self._fatal_error = (
                f"camping-site controller ERROR: {_attr(msg, 'message', '')}"
            )

    def _parking(self, name: str, msg: Any) -> None:
        self.latest["parking"][name] = _to_jsonable(
            msg, ("level", "operating_state", "message")
        )
        phase = str(_attr(msg, "operating_state", "")).strip()
        self._append_changed(self._parking_phases, f"{name}:{phase}")
        if phase == "ERROR" or int(_attr(msg, "level", 0)) >= 2:
            self._fatal_error = (
                f"{name} parking controller ERROR: {_attr(msg, 'message', '')}"
            )

    def _drop_zone(self, msg: Any) -> None:
        self.latest["drop_zone"] = _to_jsonable(
            msg, ("level", "operating_state", "message")
        )
        phase = str(_attr(msg, "operating_state", "")).strip()
        self._append_changed(self._drop_zone_phases, phase)
        if phase == "ERROR" or int(_attr(msg, "level", 0)) >= 2:
            self._fatal_error = (
                f"drop-zone controller ERROR: {_attr(msg, 'message', '')}"
            )

    def _gate(self, msg: Any) -> None:
        self.latest["gate"] = _to_jsonable(
            msg, ("level", "operating_state", "message")
        )
        self._append_changed(
            self._gate_states, str(_attr(msg, "operating_state", "")).strip()
        )
        if int(_attr(msg, "level", 0)) >= 2:
            self._fatal_error = (
                f"control safety gate ERROR: {_attr(msg, 'message', '')}"
            )

    @staticmethod
    def _append_changed(values: list[Any], value: Any) -> None:
        if value in (None, ""):
            return
        if not values or values[-1] != value:
            values.append(value)

    def begin_site(self) -> None:
        """Reset per-site accumulators without changing any ROS/CARLA state."""
        self._service_state_ids = []
        self._service_state_names = []
        self._site_phases = []
        self._parking_phases = []
        self._drop_zone_phases = []
        self._gate_states = []
        self._cmd_vel_samples = 0
        self._cmd_vel_max_abs = 0.0
        self._motion_command_observed = False
        self._carla_odom_samples = 0
        self._carla_odom_distance_m = 0.0
        self._last_carla_odom_xy = None
        self._physical_samples = 0
        self.latest["site"] = None
        self.latest["parking"] = {}
        self.latest["drop_zone"] = None

    def spin_once(self, timeout_s: float = 0.2) -> None:
        try:
            self.rclpy.spin_once(self.node, timeout_sec=timeout_s)
        except Exception as error:
            raise MatrixError(f"ROS observation failed closed: {error}") from None
        if self._fatal_error:
            raise MatrixError(self._fatal_error)
        physical = self.latest.get("physical")
        if physical is not None:
            validate_physical_status(physical, self.expected_actor_id)
            if (self._physical_received_monotonic is None or
                    time.monotonic() - self._physical_received_monotonic > 1.5):
                raise MatrixError("physical CARLA readiness heartbeat is stale")

    def snapshot(self) -> dict[str, Any]:
        document = dict(self.latest)
        document["sequences"] = {
            "service_state_ids": list(self._service_state_ids),
            "service_state_names": list(self._service_state_names),
            "site_phases": list(self._site_phases),
            "parking_phases": list(self._parking_phases),
            "drop_zone_phases": list(self._drop_zone_phases),
            "gate_states": list(self._gate_states),
        }
        document["motion_metrics"] = {
            "cmd_vel_samples": self._cmd_vel_samples,
            "cmd_vel_max_component_sum": round(self._cmd_vel_max_abs, 6),
            "motion_command_observed": self._motion_command_observed,
            "carla_odometry_samples": self._carla_odom_samples,
            "carla_odometry_distance_m": round(
                self._carla_odom_distance_m, 6
            ),
            "physical_status_samples": self._physical_samples,
        }
        document["physical_identity"] = dict(self._physical_identity or {})
        return json.loads(json.dumps(document, allow_nan=False))

    def close(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def _default_paths() -> tuple[Path, Path, Path]:
    src = Path(
        os.environ.get("CAMROD_SRC_ROOT") or Path(__file__).resolve().parents[2]
    )
    work = Path(os.environ.get("RANGER_WORK_ROOT") or "/tmp/camrod-work")
    output = work / "evidence" / "camping_site_matrix" / f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.json"
    return (
        src / "camrod_planning/config/camping_sites.yaml",
        src / "camrod_bringup/config/map/drop_zones.yaml",
        output,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--run", action="store_true", help="enable live UI/ROS execution; without this flag only a plan is written")
    parser.add_argument("--sites", help="comma-separated subset such as B1,B12 (default: all B1..B13)")
    parser.add_argument("--camping-sites-yaml", type=Path)
    parser.add_argument("--drop-zones-yaml", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--sensor-audit-report",
        type=Path,
        help="fresh PASS JSON from carla_sensor_source_audit --json (required with --run)",
    )
    parser.add_argument("--ui-url", default=os.environ.get("CAMROD_UI_URL", "http://127.0.0.1:8010"))
    parser.add_argument("--role-name", default=os.environ.get("CARLA_ROLE_NAME", "ego_vehicle"))
    parser.add_argument("--phase-timeout-s", type=float, default=900.0)
    parser.add_argument(
        "--start-drop-zone-tolerance-m",
        type=float,
        default=5.0,
        help=(
            "initial lane-connected spawn distance from the Drop Zone center; "
            "kept separate from the stricter final parking tolerance"
        ),
    )
    parser.add_argument("--drop-zone-tolerance-m", type=float, default=3.0)
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if not math.isfinite(args.phase_timeout_s) or args.phase_timeout_s <= 0.0 or args.phase_timeout_s > 3600.0:
        raise MatrixError("--phase-timeout-s must be in (0, 3600]")
    if not math.isfinite(args.drop_zone_tolerance_m) or args.drop_zone_tolerance_m <= 0.0:
        raise MatrixError("--drop-zone-tolerance-m must be positive")
    if (
        not math.isfinite(args.start_drop_zone_tolerance_m)
        or args.start_drop_zone_tolerance_m <= 0.0
    ):
        raise MatrixError("--start-drop-zone-tolerance-m must be positive")
    if not args.role_name.strip():
        raise MatrixError("--role-name must not be empty")
    if args.run and args.sensor_audit_report is None:
        raise MatrixError("--run requires --sensor-audit-report")


def run_matrix(args: argparse.Namespace, sites: Mapping[str, Site], drop_zone: DropZone, selected: Sequence[str], report: dict[str, Any]) -> int:
    """Execute UI-driven site missions and preserve every milestone."""
    client = UIClient(args.ui_url)
    observer = RosObservation(args.role_name)
    report["status"] = "RUNNING"
    report["runtime"] = {
        "started_at_utc": utc_now(),
        "ui_url": args.ui_url,
        "carla_town": os.environ.get("CARLA_TOWN", ""),
        "carla_ue_map": os.environ.get("CARLA_UE_MAP", ""),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
        "phase_timeout_s": args.phase_timeout_s,
        "start_drop_zone_tolerance_m": args.start_drop_zone_tolerance_m,
        "drop_zone_tolerance_m": args.drop_zone_tolerance_m,
        "motion_path": "UI REST -> CAMROD planning -> CAMROD control -> CARLA physical 4WS bridge",
    }
    write_json_atomic(args.output, report)

    def progress(message: str) -> None:
        print(f"[camping-matrix] {message}", flush=True)

    started_monotonic: dict[str, float] = {}

    def checkpoint(result: dict[str, Any], event: str) -> None:
        elapsed = round(time.monotonic() - started_monotonic.get(result["site"], time.monotonic()), 3)
        result.setdefault("milestones", []).append({"at_utc": utc_now(), "elapsed_s": elapsed, "event": event, "observation": observer.snapshot()})
        write_json_atomic(args.output, report)
        progress(f"{result['site']}: {event}")

    def wait_until(predicate: Callable[[dict[str, Any]], bool], timeout_s: float, label: str, result: dict[str, Any]) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            observer.spin_once(min(0.2, max(0.01, deadline - time.monotonic())))
            snapshot = observer.snapshot()
            if predicate(snapshot):
                checkpoint(result, label)
                return
        raise MatrixError(f"{result['site']}: timeout waiting for {label}")

    def copy_final_observation(result: dict[str, Any], snapshot: Mapping[str, Any]) -> None:
        result["final_service_state"] = snapshot.get("service_state") or {}
        result["final_parking_status"] = snapshot.get("parking") or {}
        result["final_drop_zone_status"] = snapshot.get("drop_zone") or {}
        result["final_gate_status"] = snapshot.get("gate") or {}
        result["final_physical_four_wheel_status"] = snapshot.get("physical") or {}
        result["final_localization_pose"] = snapshot.get("pose") or {}
        result["final_carla_odometry"] = snapshot.get("carla_odometry") or {}
        sequences = snapshot.get("sequences") or {}
        result["site_phase_sequence"] = list(sequences.get("site_phases") or [])
        result["service_state_sequence"] = list(
            sequences.get("service_state_names") or []
        )
        result["drop_zone_phase_sequence"] = list(
            sequences.get("drop_zone_phases") or []
        )
        result["parking_phase_sequence"] = list(
            sequences.get("parking_phases") or []
        )
        result["motion_metrics"] = dict(snapshot.get("motion_metrics") or {})
        update_motion_command_scope(report, snapshot)

    try:
        for index, key in enumerate(selected):
            result = next(item for item in report["sites"] if item["site"] == key)
            site = sites[key]
            result["status"] = "RUNNING"
            result["started_at_utc"] = utc_now()
            started = time.monotonic()
            started_monotonic[key] = started
            observer.begin_site()
            write_json_atomic(args.output, report)
            progress(
                f"{key} ({index + 1}/{len(selected)}): waiting for live preflight"
            )
            wait_until(
                lambda snap: all(
                    snap.get(name) is not None
                    for name in (
                        "physical", "pose", "carla_odometry",
                        "site", "drop_zone", "gate",
                    )
                ),
                20.0,
                "live dispatch preflight",
                result,
            )
            preflight = observer.snapshot()
            actor_id = observer.expected_actor_id
            if actor_id is None:
                raise MatrixError(f"{key}: no bound CARLA actor readiness")
            service_state = preflight.get("service_state") or {}
            if service_state:
                start_state = int(service_state.get("state", -1))
                if start_state not in STATIONARY_DROP_ZONE_STATES:
                    raise MatrixError(
                        f"{key}: dispatch must start at a stationary Drop Zone "
                        f"state; got {start_state}"
                    )
                result["start_service_state_source"] = "observed_ros_topic"
            elif index == 0:
                # /service/state is volatile and event-driven.  A runner that
                # attaches after bringup can legitimately miss the initial
                # DROP_ZONE_WAIT sample.  Do not invent a state: bind the cold
                # start to independent live evidence instead (STANDBY gate,
                # stationary CARLA odometry, accepted physical actor, and the
                # checked-in lane-connected Drop Zone-front spawn).  Once the
                # UI dispatches, every service transition is observed normally.
                gate_state = str(
                    (preflight.get("gate") or {}).get("operating_state", "")
                ).strip()
                speed_mps = float(
                    (preflight.get("carla_odometry") or {}).get(
                        "speed_mps", math.inf
                    )
                )
                if gate_state not in {"STANDBY", "CHARGING"}:
                    raise MatrixError(
                        f"{key}: unpublished initial service state requires a "
                        f"stationary control gate; got {gate_state!r}"
                    )
                if not math.isfinite(speed_mps) or speed_mps > 0.05:
                    raise MatrixError(
                        f"{key}: unpublished initial service state requires a "
                        f"stationary actor; speed={speed_mps!r} m/s"
                    )
                result["start_service_state_source"] = (
                    "cold_start_volatile_topic_unpublished"
                )
            else:
                raise MatrixError(
                    f"{key}: service state disappeared after an earlier site"
                )
            start_error = pose_error_m(preflight.get("pose") or {}, drop_zone)
            if (
                start_error is None
                or start_error > args.start_drop_zone_tolerance_m
            ):
                raise MatrixError(
                    f"{key}: start pose is not at the accepted Drop Zone-front "
                    f"spawn: error={start_error!r} m"
                )
            result["start_localization_pose"] = preflight.get("pose") or {}
            result["start_drop_zone_error_m"] = start_error
            result["actor_id"] = actor_id
            progress(
                f"{key} ({index + 1}/{len(selected)}): dispatching via production UI"
            )
            response = client.dispatch(key)
            result["dispatch_response"] = response
            checkpoint(result, "dispatch accepted")
            expected_phases = SITE_PHASES[site.service_mode]
            wait_until(
                lambda snap: arrival_ready_for_return(snap, expected_phases),
                args.phase_timeout_s,
                "WAITING_FOR_RETURN_REQUEST with ordered WAIT_RETURN phase",
                result,
            )
            arrival = observer.snapshot()
            result["arrival_localization_pose"] = arrival.get("pose") or {}
            arrival_phases = list(
                (arrival.get("sequences") or {}).get("site_phases") or []
            )
            wait_return_index = expected_phases.index("WAIT_RETURN") + 1
            if not contains_ordered_subsequence(
                arrival_phases, expected_phases[:wait_return_index]
            ):
                raise MatrixError(
                    f"{key}: arrival phase sequence is incomplete for "
                    f"{site.service_mode}: {arrival_phases}"
                )
            result["return_response"] = client.request_return()
            checkpoint(result, "return request accepted")
            wait_until(
                lambda snap: WAITING_FOR_CHARGING
                in ((snap.get("sequences") or {}).get("service_state_ids") or []),
                args.phase_timeout_s,
                "WAITING_FOR_CHARGING at Drop Zone",
                result,
            )
            wait_until(
                lambda snap: (
                    (snap.get("service_state") or {}).get("state") == CHARGING
                    and any(
                        phase.endswith(":PARKED")
                        for phase in (
                            (snap.get("sequences") or {}).get(
                                "parking_phases"
                            ) or []
                        )
                    )
                ),
                args.phase_timeout_s,
                "PARKED and CHARGING",
                result,
            )
            final_snapshot = observer.snapshot()
            copy_final_observation(result, final_snapshot)
            missing_observations = [
                name for name in (
                    "pose", "carla_odometry", "cmd_vel", "site", "parking",
                    "drop_zone", "gate", "physical",
                )
                if not final_snapshot.get(name)
            ]
            if missing_observations:
                raise MatrixError(
                    f"{key}: required live observations missing at charging: "
                    f"{', '.join(missing_observations)}"
                )
            error = pose_error_m(result["final_localization_pose"], drop_zone)
            result["drop_zone_error_m"] = error
            sequences = final_snapshot.get("sequences") or {}
            site_phases = list(sequences.get("site_phases") or [])
            parking_phases = list(sequences.get("parking_phases") or [])
            drop_phases = list(sequences.get("drop_zone_phases") or [])
            service_ids = list(sequences.get("service_state_ids") or [])
            motion = final_snapshot.get("motion_metrics") or {}
            if not contains_ordered_subsequence(site_phases, expected_phases):
                raise MatrixError(
                    f"{key}: complete site phase sequence is invalid for "
                    f"{site.service_mode}: {site_phases}"
                )
            if "ALIGN_PARKING_YAW" not in drop_phases:
                raise MatrixError(
                    f"{key}: Drop Zone ALIGN_PARKING_YAW was not observed: "
                    f"{drop_phases}"
                )
            if not all(
                state in service_ids
                for state in (DROP_ZONE_PARKING, WAITING_FOR_CHARGING, CHARGING)
            ):
                raise MatrixError(
                    f"{key}: charging service sequence is incomplete: {service_ids}"
                )
            parked = any(phase.endswith(":PARKED") for phase in parking_phases)
            result["parking_confirmed"] = parked
            result["charging_confirmed"] = bool(
                (final_snapshot.get("service_state") or {}).get("state")
                == CHARGING
            )
            if not bool(motion.get("motion_command_observed")):
                raise MatrixError(f"{key}: no nonzero final cmd_vel was observed")
            if int(motion.get("cmd_vel_samples", 0)) < 2:
                raise MatrixError(f"{key}: insufficient final cmd_vel samples")
            if int(motion.get("carla_odometry_samples", 0)) < 2:
                raise MatrixError(f"{key}: insufficient CARLA odometry samples")
            if float(motion.get("carla_odometry_distance_m", 0.0)) < 1.0:
                raise MatrixError(f"{key}: CARLA actor movement was below 1.0 m")
            final_speed = float(
                (final_snapshot.get("carla_odometry") or {}).get(
                    "speed_mps", math.nan
                )
            )
            if not math.isfinite(final_speed) or final_speed > 0.05:
                raise MatrixError(
                    f"{key}: final CARLA speed {final_speed!r} m/s is not stopped"
                )
            if error is None or error > args.drop_zone_tolerance_m:
                raise MatrixError(f"{key}: drop-zone final error {error!r} m exceeds {args.drop_zone_tolerance_m:g} m")
            result["status"] = "PASS"
            result["finished_at_utc"] = utc_now()
            result["elapsed_s"] = round(time.monotonic() - started, 3)
            checkpoint(result, f"PASS; return error={error:.3f} m")
        report["status"] = "PASS"
        report["runtime"]["finished_at_utc"] = utc_now()
        write_json_atomic(args.output, report)
        progress(f"matrix PASS: {len(selected)} site(s)")
        return 0
    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        report["runtime"]["finished_at_utc"] = utc_now()
        for item in report["sites"]:
            if item["status"] == "RUNNING":
                item["status"] = "INTERRUPTED"
                item["finished_at_utc"] = utc_now()
                item["elapsed_s"] = round(
                    time.monotonic()
                    - started_monotonic.get(item["site"], time.monotonic()),
                    3,
                )
                copy_final_observation(item, observer.snapshot())
        write_json_atomic(args.output, report)
        try:
            client.stop()
        except MatrixError:
            pass
        raise
    except Exception as error:
        report["status"] = "FAIL"
        report["failure_reason"] = str(error)
        report["runtime"]["finished_at_utc"] = utc_now()
        for item in report["sites"]:
            if item["status"] == "RUNNING":
                item["status"] = "FAIL"
                item["failure_reason"] = str(error)
                item["finished_at_utc"] = utc_now()
                item["elapsed_s"] = round(
                    time.monotonic()
                    - started_monotonic.get(item["site"], time.monotonic()),
                    3,
                )
                copy_final_observation(item, observer.snapshot())
            elif item["status"] == "NOT_ATTEMPTED":
                item["failure_reason"] = "not attempted after earlier site failure"
        write_json_atomic(args.output, report)
        progress(f"FAIL CLOSED: {error}")
        try:
            client.stop()
        except MatrixError as stop_error:
            report["stop_error"] = str(stop_error)
            write_json_atomic(args.output, report)
        return 1
    finally:
        observer.close()


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        _validate_args(args)
        default_sites, default_drop_zones, default_output = _default_paths()
        camping_path = (args.camping_sites_yaml or default_sites).expanduser().resolve()
        drop_path = (args.drop_zones_yaml or default_drop_zones).expanduser().resolve()
        output_explicit = args.output is not None
        args.output = (args.output or default_output).expanduser().resolve()
        all_sites = load_sites(camping_path)
        selected = parse_site_selection(args.sites)
        missing = [key for key in selected if key not in all_sites]
        if missing:
            raise MatrixError(f"selected sites are missing from active YAML: {missing}")
        drop_zone = load_drop_zone(drop_path)
        report = build_plan(all_sites, selected, drop_zone, output=args.output, config_paths={"camping_sites": camping_path, "drop_zones": drop_path})
        if args.run:
            assert args.sensor_audit_report is not None
            sensor_audit_path = args.sensor_audit_report.expanduser().resolve()
            report["sensor_source_audit"] = load_sensor_audit(sensor_audit_path)
        if args.run or output_explicit:
            write_json_atomic(args.output, report, create_only=True)
        print(f"[camping-matrix] {'RUN' if args.run else 'PLAN_ONLY'}: {', '.join(selected)}")
        print(f"[camping-matrix] active drop zone={drop_zone.source_id} ({drop_zone.x_m:.4f}, {drop_zone.y_m:.4f}); report={args.output}")
        if not args.run:
            if output_explicit:
                print(f"[camping-matrix] explicit plan report written: {args.output}")
            else:
                print("[camping-matrix] no file or directory was created")
            print("[camping-matrix] no ROS/UI/motion side effects; add --run only after server -> bridge -> pacer -> spawn -> camrod is healthy")
            return 0
        return run_matrix(args, all_sites, drop_zone, selected, report)
    except KeyboardInterrupt:
        return 130
    except MatrixError as error:
        print(f"[camping-matrix] ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
