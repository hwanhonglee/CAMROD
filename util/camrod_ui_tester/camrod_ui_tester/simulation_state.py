"""Thread-safe-agnostic state model and deterministic motion engine."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Dict, Optional


SERVICE_STATES = {
    "DROP_ZONE_WAIT": 0,
    "MOVING_TO_SITE": 1,
    "SITE_ARRIVED": 2,
    "RETURNING_TO_DROP_ZONE": 3,
    "GUEST_RECALL_SERVICE": 4,
    "SITE_ENTRY": 5,
    "UNLOAD_WAIT": 6,
    "RECALL_TO_SITE_ROAD": 7,
    "GUEST_LOADING_WAIT": 8,
    "RETURN_WITH_CARGO": 9,
    "DROP_ZONE_PARKING": 10,
    "WAITING_FOR_RETURN_REQUEST": 11,
    "WAITING_FOR_CHARGING": 12,
    "CHARGING": 13,
    "DEPARTING_CHARGER": 14,
    "DEPARTING_DROP_ZONE": 15,
    "OPERATOR_STOPPED": 16,
}

PLANNING_STATES = {
    "INIT": 0,
    "READY": 1,
    "WAIT_DZ": 2,
    "RUNNING": 3,
    "GOAL_REACHED": 4,
    "RECALLED": 5,
    "RETURNING": 6,
    "WARN_RECOVERY": 7,
    "ERROR_STOP": 8,
}

WORKFLOW_TYPES = {"delivery", "recall"}
WORKFLOW_PHASES = {
    "move_to_site",
    "site_arrived",
    "return_to_drop_zone",
    "drop_zone_return_complete",
    "docking_start",
    "docking_complete",
}

PARKING_CHARGE_THRESHOLD_FRACTION = 0.35


def parking_method_for_battery(
    percentage: float,
    *,
    available: bool = True,
) -> str:
    """Mirror the production auto-parking SOC policy for simulator status."""
    try:
        fraction = float(percentage)
    except (TypeError, ValueError):
        return "apriltag"
    if not available or not math.isfinite(fraction):
        return "apriltag"
    return (
        "reverse"
        if fraction >= PARKING_CHARGE_THRESHOLD_FRACTION
        else "apriltag"
    )


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0


@dataclass
class Motion:
    kind: str
    start: Pose2D
    target: Pose2D
    elapsed_s: float
    duration_s: float
    site: str = ""


@dataclass
class SimulationState:
    mode: str = "closed_loop"
    scenario: str = "boot"
    paused: bool = False
    speed_scale: float = 1.0
    sim_time_s: float = 0.0
    service_state: str = "DROP_ZONE_WAIT"
    planning_state: str = "INIT"
    localization_mode: int = 0
    gate_state: str = "STANDBY"
    battery_percentage: float = 0.80
    battery_available: bool = True
    charging: bool = False
    estop: bool = False
    platform_error_code: int = 0
    system_error: bool = False
    safety_hold: bool = False
    planning_engaged: bool = False
    mission_engaged: bool = False
    drive_enabled: bool = False
    headlight: bool = False
    active_site: str = ""
    active_mission_key: str = ""
    goal_source: str = ""
    workflow_type: str = ""
    workflow_phase: str = "ready"
    workflow_manual: bool = False
    recall_turn_complete: bool = False
    pose: Pose2D = field(default_factory=Pose2D)
    velocity_mps: float = 0.0
    route_progress: float = 0.0
    motion: Optional[Motion] = None

    def snapshot(self) -> dict:
        payload = asdict(self)
        payload["battery_percent"] = round(self.battery_percentage * 100.0, 1)
        payload["service_state_value"] = SERVICE_STATES[self.service_state]
        payload["planning_state_value"] = PLANNING_STATES[self.planning_state]
        return payload


class ScenarioEngine:
    """Small kinematic state machine driven by UI commands or operator controls."""

    def __init__(
        self,
        *,
        station_pose: Pose2D,
        sites: Dict[str, Pose2D],
        mode: str = "closed_loop",
        speed_scale: float = 1.0,
        route_duration_s: float = 12.0,
        maneuver_duration_s: float = 2.0,
    ) -> None:
        self.station_pose = station_pose
        self.sites = dict(sites)
        self.route_duration_s = max(1.0, float(route_duration_s))
        self.maneuver_duration_s = max(0.2, float(maneuver_duration_s))
        self.state = SimulationState(
            mode=self._normalize_mode(mode),
            speed_scale=self._bounded_speed(speed_scale),
            pose=Pose2D(station_pose.x, station_pose.y, station_pose.yaw_deg),
        )
        self._parking_elapsed_s = 0.0

    @staticmethod
    def _normalize_mode(value: str) -> str:
        normalized = str(value).strip().lower()
        return normalized if normalized in {"manual", "closed_loop", "scripted"} else "closed_loop"

    @staticmethod
    def _bounded_speed(value: float) -> float:
        return max(0.1, min(10.0, float(value)))

    def reset(self, *, ready: bool = True) -> None:
        mode = self.state.mode
        scale = self.state.speed_scale
        self._parking_elapsed_s = 0.0
        self.state = SimulationState(
            mode=mode,
            speed_scale=scale,
            scenario="ready" if ready else "boot",
            planning_state="READY" if ready else "INIT",
            pose=Pose2D(
                self.station_pose.x,
                self.station_pose.y,
                self.station_pose.yaw_deg,
            ),
        )

    def set_mode(self, mode: str) -> None:
        self.state.mode = self._normalize_mode(mode)

    def set_speed_scale(self, value: float) -> None:
        self.state.speed_scale = self._bounded_speed(value)

    def apply_scenario(self, name: str) -> None:
        scenario = str(name).strip().lower()
        if scenario in {"reset", "ready"}:
            self.reset(ready=True)
            return
        if scenario == "boot":
            self.reset(ready=False)
            return
        if scenario == "low_battery":
            self.state.battery_percentage = 0.34
        elif scenario == "urgent_battery":
            self.state.battery_percentage = 0.24
        elif scenario == "charging":
            self.state.charging = True
            self.state.service_state = "CHARGING"
            self.state.planning_state = "WAIT_DZ"
            self._stop_motion()
        elif scenario == "safety_hold":
            self.state.safety_hold = True
            self.state.gate_state = "SAFETY_HOLD"
        elif scenario == "estop":
            self.state.estop = True
            self.state.gate_state = "SAFETY_HOLD"
            self._stop_motion()
        elif scenario == "platform_fault":
            self.state.platform_error_code = 1
            self.state.gate_state = "SAFETY_HOLD"
            self._stop_motion()
        elif scenario == "localization_lost":
            self.state.localization_mode = 3
        elif scenario == "clear_faults":
            self.state.estop = False
            self.state.platform_error_code = 0
            self.state.system_error = False
            self.state.safety_hold = False
            self.state.localization_mode = 0
            self.state.gate_state = (
                "ENABLED" if self.state.planning_engaged else "STANDBY"
            )
        elif scenario.startswith("arrived_"):
            self.arrive(scenario.removeprefix("arrived_").upper())
        elif scenario == "loading_wait":
            self.state.service_state = "UNLOAD_WAIT"
            self._stop_motion()
        elif scenario == "returned_drop_zone":
            self.state.pose = Pose2D(
                self.station_pose.x,
                self.station_pose.y,
                self.station_pose.yaw_deg,
            )
            self.state.service_state = "DROP_ZONE_PARKING"
            self.state.planning_state = "GOAL_REACHED"
            self._stop_motion()
        elif scenario == "parking_complete":
            self.state.service_state = "WAITING_FOR_CHARGING"
            self.state.planning_state = "WAIT_DZ"
            self._stop_motion()
        else:
            raise ValueError(f"unknown scenario: {name}")
        self.state.scenario = scenario

    def select_destination(self, site: str, mission_key: str, source: str) -> None:
        normalized = str(site).strip().upper()
        if normalized not in self.sites:
            return
        self.state.active_site = normalized
        self.state.active_mission_key = str(mission_key).strip() or self._mission_key(normalized)
        self.state.goal_source = str(source).strip() or "ui_simulator"

    def receive_goal(self, pose: Pose2D, *, source: str = "ui") -> None:
        self.state.goal_source = source
        site = self.state.active_site
        if not site:
            site = self._nearest_site(pose)
            self.state.active_site = site
        if self.state.mode == "closed_loop" and self.state.drive_enabled:
            self._start_motion("to_site", pose, self.route_duration_s, site)

    def set_engaged(self, *, planning: Optional[bool] = None, mission: Optional[bool] = None) -> None:
        if planning is not None:
            self.state.planning_engaged = bool(planning)
        if mission is not None:
            self.state.mission_engaged = bool(mission)
        engaged = self.state.planning_engaged or self.state.mission_engaged
        if self.state.safety_hold or self.state.estop or self.state.platform_error_code:
            self.state.gate_state = "SAFETY_HOLD"
        else:
            self.state.gate_state = "ENABLED" if engaged else "STANDBY"
        if not engaged:
            self.state.velocity_mps = 0.0

    def set_drive_enabled(self, enabled: bool) -> None:
        self.state.drive_enabled = bool(enabled)
        if not enabled:
            self.state.velocity_mps = 0.0

    def begin_departure(self) -> None:
        if self.state.mode != "closed_loop":
            return
        self.state.charging = False
        self.state.service_state = "DEPARTING_DROP_ZONE"
        self.state.planning_state = "WAIT_DZ"
        target = Pose2D(
            self.station_pose.x + 2.0 * math.cos(math.radians(self.station_pose.yaw_deg)),
            self.station_pose.y + 2.0 * math.sin(math.radians(self.station_pose.yaw_deg)),
            self.station_pose.yaw_deg,
        )
        self._start_motion("drop_zone_exit", target, self.maneuver_duration_s)

    def begin_return(self, site: str = "") -> None:
        if self.state.mode != "closed_loop":
            return
        if site:
            self.state.active_site = str(site).strip().upper()
        # Site arrival makes the production UI release mission and drive
        # authority. A subsequent RETURN command is a new motion request, so
        # the simulator must re-arm both or the return path remains at 0%.
        self.state.drive_enabled = True
        self.set_engaged(mission=True)
        self.state.service_state = (
            "RETURN_WITH_CARGO"
            if self.state.workflow_type == "recall"
            else "RETURNING_TO_DROP_ZONE"
        )
        self.state.planning_state = "RETURNING"
        self.state.workflow_phase = "return_to_drop_zone"
        self._start_motion("return", self.station_pose, self.route_duration_s, self.state.active_site)

    def begin_recall(self, site: str) -> bool:
        normalized = str(site).strip().upper()
        target = self.sites.get(normalized)
        if self.state.mode != "closed_loop" or target is None:
            return False
        self.state.active_site = normalized
        self.state.workflow_type = "recall"
        self.state.workflow_phase = "move_to_site"
        self.state.workflow_manual = False
        self.state.recall_turn_complete = False
        self.state.service_state = "GUEST_RECALL_SERVICE"
        self.state.planning_state = "RUNNING"
        self.state.drive_enabled = True
        self._start_motion("recall", target, self.route_duration_s, normalized)
        return True

    def begin_recall_site_turn(self) -> bool:
        """Run the in-site clearance turn the first RETURN command requests."""
        if (
            self.state.mode != "closed_loop"
            or self.state.workflow_type != "recall"
            or self.state.recall_turn_complete
            or self.state.service_state != "GUEST_LOADING_WAIT"
            or self.state.motion is not None
        ):
            return False
        self.state.service_state = "SITE_ENTRY"
        self.state.workflow_phase = "site_entry"
        self.state.planning_state = "RUNNING"
        self.state.drive_enabled = True
        pose = self.state.pose
        self._start_motion(
            "recall_site_turn",
            Pose2D(pose.x, pose.y, pose.yaw_deg + 180.0),
            self.maneuver_duration_s,
            self.state.active_site,
        )
        return True

    def control_workflow(self, workflow: str, site: str, phase: str) -> None:
        """Force one observable phase of a delivery or guest-recall lifecycle."""
        normalized_workflow = str(workflow).strip().lower()
        normalized_phase = str(phase).strip().lower()
        normalized_site = str(site).strip().upper()
        if normalized_workflow not in WORKFLOW_TYPES:
            raise ValueError(f"invalid workflow: {workflow}")
        if normalized_phase not in WORKFLOW_PHASES:
            raise ValueError(f"invalid workflow phase: {phase}")
        target = self.sites.get(normalized_site)
        if target is None:
            raise ValueError(f"invalid site: {site}")

        state = self.state
        state.workflow_type = normalized_workflow
        state.workflow_phase = normalized_phase
        state.workflow_manual = True
        state.active_site = normalized_site
        state.active_mission_key = self._mission_key(normalized_site)
        state.goal_source = f"simulator_{normalized_workflow}_workflow"
        state.scenario = f"{normalized_workflow}:{normalized_phase}"
        state.charging = False

        if normalized_phase == "move_to_site":
            state.drive_enabled = True
            state.mission_engaged = True
            state.gate_state = "ENABLED"
            state.planning_state = "RUNNING"
            state.service_state = (
                "RECALL_TO_SITE_ROAD"
                if normalized_workflow == "recall"
                else "MOVING_TO_SITE"
            )
            motion_kind = "recall" if normalized_workflow == "recall" else "to_site"
            self._start_motion(motion_kind, target, self.route_duration_s, normalized_site)
            # _start_motion owns the delivery state; recall uses its distinct ROS state.
            if normalized_workflow == "recall":
                state.service_state = "RECALL_TO_SITE_ROAD"
            return

        if normalized_phase == "site_arrived":
            state.pose = Pose2D(target.x, target.y, target.yaw_deg)
            state.service_state = (
                "GUEST_LOADING_WAIT"
                if normalized_workflow == "recall"
                else "SITE_ARRIVED"
            )
            # Scripted recall arrival presents the post-turn loading wait, so
            # the RECALL_RETURN_WAIT campsite phase must accompany it.
            state.recall_turn_complete = normalized_workflow == "recall"
            state.planning_state = "GOAL_REACHED"
            state.route_progress = 1.0
            state.drive_enabled = False
            self._stop_motion()
            return

        if normalized_phase == "return_to_drop_zone":
            state.pose = Pose2D(target.x, target.y, target.yaw_deg)
            state.drive_enabled = True
            state.mission_engaged = True
            state.gate_state = "ENABLED"
            self.begin_return(normalized_site)
            return

        if normalized_phase == "drop_zone_return_complete":
            state.pose = Pose2D(
                self.station_pose.x,
                self.station_pose.y,
                self.station_pose.yaw_deg,
            )
            state.service_state = (
                "RETURN_WITH_CARGO"
                if normalized_workflow == "recall"
                else "RETURNING_TO_DROP_ZONE"
            )
            state.planning_state = "GOAL_REACHED"
            state.route_progress = 1.0
            state.drive_enabled = False
            self._stop_motion()
            return

        if normalized_phase == "docking_start":
            state.pose = Pose2D(
                self.station_pose.x,
                self.station_pose.y,
                self.station_pose.yaw_deg,
            )
            state.service_state = "DROP_ZONE_PARKING"
            state.planning_state = "WAIT_DZ"
            state.drive_enabled = True
            state.gate_state = "ENABLED"
            state.route_progress = 0.0
            self._stop_motion()
            return

        # docking_complete
        state.pose = Pose2D(
            self.station_pose.x,
            self.station_pose.y,
            self.station_pose.yaw_deg,
        )
        state.service_state = "WAITING_FOR_CHARGING"
        state.planning_state = "WAIT_DZ"
        state.drive_enabled = False
        state.planning_engaged = False
        state.mission_engaged = False
        state.gate_state = "STANDBY"
        state.route_progress = 1.0
        self._stop_motion()

    def arrive(self, site: str) -> None:
        normalized = str(site).strip().upper()
        target = self.sites.get(normalized)
        if target is not None:
            self.state.pose = Pose2D(target.x, target.y, target.yaw_deg)
            self.state.active_site = normalized
        self.state.service_state = "SITE_ARRIVED"
        self.state.planning_state = "GOAL_REACHED"
        self.state.route_progress = 1.0
        self._stop_motion()

    def stop(self) -> None:
        self.state.service_state = "OPERATOR_STOPPED"
        self.state.planning_state = "ERROR_STOP"
        self.state.planning_engaged = False
        self.state.mission_engaged = False
        self.state.drive_enabled = False
        self.state.gate_state = "STANDBY"
        self._stop_motion()

    def tick(self, wall_dt_s: float) -> Optional[str]:
        if self.state.paused:
            return None
        dt = max(0.0, min(float(wall_dt_s), 0.5)) * self.state.speed_scale
        self.state.sim_time_s += dt
        motion = self.state.motion
        if motion is None:
            # HH_260908 - The production docking controllers finish parking on
            # their own; without this the closed loop stays in DROP_ZONE_PARKING
            # forever and no follow-up mission can ever be admitted.
            if (
                self.state.mode == "closed_loop"
                and self.state.service_state == "DROP_ZONE_PARKING"
                and not self.state.safety_hold
                and not self.state.estop
            ):
                self._parking_elapsed_s += dt
                if self._parking_elapsed_s >= self.maneuver_duration_s:
                    self._parking_elapsed_s = 0.0
                    self.state.pose = Pose2D(
                        self.station_pose.x,
                        self.station_pose.y,
                        self.station_pose.yaw_deg,
                    )
                    self.state.service_state = "WAITING_FOR_CHARGING"
                    self.state.planning_state = "WAIT_DZ"
                    if self.state.workflow_type:
                        self.state.workflow_phase = "docking_complete"
                    self.state.drive_enabled = False
                    self.state.planning_engaged = False
                    self.state.mission_engaged = False
                    self.state.gate_state = "STANDBY"
                    self.state.scenario = "parking_complete"
                    return "parking_complete"
            else:
                self._parking_elapsed_s = 0.0
            return None
        if self.state.safety_hold or self.state.estop or not self.state.drive_enabled:
            self.state.velocity_mps = 0.0
            return None
        motion.elapsed_s += dt
        progress = min(1.0, motion.elapsed_s / motion.duration_s)
        eased = progress * progress * (3.0 - 2.0 * progress)
        self.state.pose = Pose2D(
            x=motion.start.x + (motion.target.x - motion.start.x) * eased,
            y=motion.start.y + (motion.target.y - motion.start.y) * eased,
            yaw_deg=self._lerp_yaw(motion.start.yaw_deg, motion.target.yaw_deg, eased),
        )
        distance = math.hypot(
            motion.target.x - motion.start.x,
            motion.target.y - motion.start.y,
        )
        self.state.velocity_mps = distance / motion.duration_s if progress < 1.0 else 0.0
        self.state.route_progress = progress
        if self.state.battery_available and not self.state.charging:
            self.state.battery_percentage = max(0.0, self.state.battery_percentage - dt * 0.00005)
        if progress < 1.0:
            return None

        completed = motion.kind
        self.state.motion = None
        self.state.velocity_mps = 0.0
        if completed == "drop_zone_exit":
            self.state.scenario = "drop_zone_exit_complete"
            return "drop_zone_exit_complete"
        if completed == "to_site":
            self.state.workflow_phase = "site_arrived"
            self.state.service_state = "SITE_ARRIVED"
            self.state.planning_state = "GOAL_REACHED"
            self.state.scenario = "arrived"
            return "site_arrived"
        if completed == "recall":
            # HH_260908 - The recall completion contract (backend Return
            # admission and both UIs' buttons) accepts loading-complete only
            # in GUEST_LOADING_WAIT; WAITING_FOR_RETURN_REQUEST here left
            # every closed-loop recall stranded at the roadside.  The
            # in-site clearance turn is NOT part of arrival: the first
            # loading-complete authorizes it through the campsite
            # controller (begin_recall_site_turn), exactly like the robot.
            self.state.workflow_phase = "site_arrived"
            self.state.service_state = "GUEST_LOADING_WAIT"
            self.state.planning_state = "GOAL_REACHED"
            self.state.scenario = "recall_arrived"
            return "recall_arrived"
        if completed == "recall_site_turn":
            # The turn parks the robot inside the site facing the exit and
            # arms the final loading-complete confirmation
            # (RECALL_RETURN_WAIT campsite phase).
            self.state.workflow_phase = "RECALL_RETURN_WAIT"
            self.state.service_state = "GUEST_LOADING_WAIT"
            self.state.planning_state = "GOAL_REACHED"
            self.state.recall_turn_complete = True
            self.state.scenario = "recall_loading_wait"
            return "recall_loading_wait"
        if completed == "return":
            self.state.workflow_phase = "drop_zone_return_complete"
            if self.state.workflow_manual:
                self.state.service_state = (
                    "RETURN_WITH_CARGO"
                    if self.state.workflow_type == "recall"
                    else "RETURNING_TO_DROP_ZONE"
                )
            else:
                self.state.service_state = "DROP_ZONE_PARKING"
            self.state.planning_state = "GOAL_REACHED"
            self.state.scenario = "returned_drop_zone"
            return "returned_drop_zone"
        return completed

    def _start_motion(self, kind: str, target: Pose2D, duration_s: float, site: str = "") -> None:
        start = self.state.pose
        self.state.motion = Motion(
            kind=kind,
            start=Pose2D(start.x, start.y, start.yaw_deg),
            target=Pose2D(target.x, target.y, target.yaw_deg),
            elapsed_s=0.0,
            duration_s=max(0.1, duration_s),
            site=site,
        )
        self.state.route_progress = 0.0
        if kind == "to_site":
            self.state.service_state = "MOVING_TO_SITE"
            self.state.planning_state = "RUNNING"

    def _stop_motion(self) -> None:
        self.state.motion = None
        self.state.velocity_mps = 0.0

    def _nearest_site(self, pose: Pose2D) -> str:
        if not self.sites:
            return ""
        return min(
            self.sites,
            key=lambda name: math.hypot(
                self.sites[name].x - pose.x,
                self.sites[name].y - pose.y,
            ),
        )

    @staticmethod
    def _mission_key(site: str) -> str:
        suffix = str(site).strip().upper().removeprefix("B")
        return f"camping_site_{suffix}"

    @staticmethod
    def _lerp_yaw(start: float, target: float, progress: float) -> float:
        delta = (target - start + 180.0) % 360.0 - 180.0
        return start + delta * progress
