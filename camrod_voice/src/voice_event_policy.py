"""Pure state policy for CAMROD startup and mission voice sequencing."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping, Optional, Sequence, Tuple


@dataclass(frozen=True)
class VoiceEvent:
    """One request for the voice announcer."""

    key: str
    priority: int = 1
    interrupt: bool = False


ModuleSnapshot = Tuple[int, str]


class VoiceEventPolicy:
    """Combine asynchronous inputs into ordered, de-duplicated events."""

    ERROR_LEVEL = 2
    NORMAL_LOCALIZATION_MODE = 0

    _IDLE_PLANNING_STATES = frozenset({"READY", "WAIT_DZ"})
    _UNSAFE_PLANNING_STATES = frozenset(
        {"", "INIT", "WARN_RECOVERY", "ERROR_STOP"}
    )
    _IDLE_GATE_STATES = frozenset({"STANDBY", "CHARGING"})
    _ACTIVE_GATE_STATES = frozenset({"ENABLED", "DEPARTING_CHARGER"})
    _ROUTE_TO_SITE_SCENARIOS = frozenset(
        {"DELIVERY_TO_SITE", "RECALL_TO_SITE", "RECALL_TO_SITE_ROAD"}
    )
    # HH_260812 - Every drop-zone-bound phase shares one voice context so the
    # bed and the periodic reminder survive the return handoffs.
    _ROUTE_TO_DROP_ZONE_SCENARIOS = frozenset(
        {"RETURN_TO_DROP_ZONE", "RETURN_WITH_CARGO", "DROP_ZONE_PARKING"}
    )
    _TRAVEL_STATES = frozenset({"RUNNING", "RETURNING"})
    # Departure cue per travel context, played once when motion begins.
    _DEPARTURE_KEYS = {
        "site": "navigation.to_campsite",
        "drop_zone": "navigation.to_dropzone",
    }
    # Reminders repeated over the music bed while the trip is under way.
    _TRAVEL_ANNOUNCE_KEYS = {
        "site": ("system.announce1", "system.announce2"),
        "drop_zone": ("navigation.return_to_dropzone",),
    }
    _SITE_ARRIVAL_SCENARIOS = frozenset(
        {
            "DELIVERY_TO_SITE",
            "RECALL_TO_SITE",
            "UNLOAD_WAIT",
            "GUEST_LOADING_WAIT",
        }
    )
    _NOT_READY_OPERATING_STATES = frozenset(
        {
            "STARTING",
            "INITIALIZING",
            "INACTIVE",
            "UNCONFIGURED",
            "UNINITIALIZED",
            "FAULT",
            "UNKNOWN",
            "NOT_READY",
        }
    )

    def __init__(
        self,
        required_modules: Sequence[str],
        *,
        return_mission_key: str = "drop_zone",
        max_ready_localization_mode: int = NORMAL_LOCALIZATION_MODE,
    ) -> None:
        self.required_modules = tuple(
            dict.fromkeys(
                str(name).strip()
                for name in required_modules
                if str(name).strip()
            )
        )
        self.return_mission_key = (
            str(return_mission_key).strip() or "drop_zone"
        )
        self.max_ready_localization_mode = int(max_ready_localization_mode)

        self.system_received = False
        self.system_modules: dict[str, ModuleSnapshot] = {}
        self.planning_received = False
        self.planning_state = ""
        self.planning_scenario = ""
        self.active_mission_key = ""
        self.active_goal_source = ""
        self.return_requested = False
        self.action_server_ready = False
        self.localization_received = False
        self.localization_mode = 255
        self.tf_ready = False
        self.gate_received = False
        self.gate_level = 2
        self.gate_state = ""
        self.gate_message = ""
        self.platform_received = False
        self.platform_estop = True
        self.platform_error_code = -1
        self.engage_received = False
        self.engaged = False

        self.startup_announced = False
        self.ready_announced = False
        self._engage_epoch = 0
        self._route_epoch = 0
        self._last_route_identity: Optional[tuple[str, str, str, str]] = None
        self._announced_motion_signatures: set[tuple[object, ...]] = set()
        self._motion_started_route_epochs: set[int] = set()
        self._announced_arrival_signatures: set[tuple[object, ...]] = set()
        self._obstacle_announced = False

    def announce_startup(self) -> list[VoiceEvent]:
        if self.startup_announced:
            return []
        self.startup_announced = True
        events = [VoiceEvent("system.startup", priority=1)]
        events.extend(self._events_after_update())
        return events

    def update_system(
        self, modules: Mapping[str, ModuleSnapshot]
    ) -> list[VoiceEvent]:
        self.system_received = True
        self.system_modules = {
            str(name): (int(snapshot[0]), str(snapshot[1]))
            for name, snapshot in modules.items()
        }
        return self._events_after_update()

    def update_planning(
        self,
        *,
        state: str,
        scenario: str,
        active_mission_key: str,
        active_goal_source: str,
        return_requested: bool,
    ) -> list[VoiceEvent]:
        state = str(state).strip().upper()
        scenario = str(scenario).strip().upper()
        mission_key = str(active_mission_key).strip()
        goal_source = str(active_goal_source).strip()
        route_identity = (state, scenario, mission_key, goal_source)
        if (
            state in {"RUNNING", "RETURNING"}
            and route_identity != self._last_route_identity
        ):
            self._route_epoch += 1
            self._last_route_identity = route_identity
        elif state not in {"RUNNING", "RETURNING"}:
            self._last_route_identity = None

        self.planning_received = True
        self.planning_state = state
        self.planning_scenario = scenario
        self.active_mission_key = mission_key
        self.active_goal_source = goal_source
        self.return_requested = bool(return_requested)
        return self._events_after_update()

    def update_action_server(self, ready: bool) -> list[VoiceEvent]:
        self.action_server_ready = bool(ready)
        return self._events_after_update()

    def update_localization(self, mode: int) -> list[VoiceEvent]:
        self.localization_received = True
        self.localization_mode = int(mode)
        return self._events_after_update()

    def update_tf(self, ready: bool) -> list[VoiceEvent]:
        self.tf_ready = bool(ready)
        return self._events_after_update()

    def update_gate(
        self, *, level: int, operating_state: str, message: str
    ) -> list[VoiceEvent]:
        obstacle_was_active = self._obstacle_hold_active()
        self.gate_received = True
        self.gate_level = int(level)
        self.gate_state = str(operating_state).strip().upper()
        self.gate_message = str(message).strip().lower()
        obstacle_is_active = self._obstacle_hold_active()

        events: list[VoiceEvent] = []
        if (
            obstacle_is_active
            and not obstacle_was_active
            and self.engaged
            and self.ready_announced
        ):
            self._obstacle_announced = True
            events.append(VoiceEvent("safety.obstacle", priority=2))
        elif not obstacle_is_active and self._obstacle_announced:
            # HH_260812 - The hold that was announced has cleared, so the robot
            # is moving again; thank whoever stepped out of the way.
            self._obstacle_announced = False
            if self.engaged and self.ready_announced:
                events.append(VoiceEvent("safety.thankyou", priority=1))
        events.extend(self._events_after_update())
        return events

    def update_platform(
        self, *, estop: bool, error_code: int
    ) -> list[VoiceEvent]:
        self.platform_received = True
        self.platform_estop = bool(estop)
        self.platform_error_code = int(error_code)
        return self._events_after_update()

    def update_engaged(self, engaged: bool) -> list[VoiceEvent]:
        engaged = bool(engaged)
        if engaged and (not self.engage_received or not self.engaged):
            self._engage_epoch += 1
        self.engage_received = True
        self.engaged = engaged
        return self._events_after_update()

    def readiness_reasons(
        self, *, require_idle: bool = True
    ) -> tuple[str, ...]:
        reasons: list[str] = []
        if not self.system_received:
            reasons.append("system_status_missing")
        else:
            for module_name in self.required_modules:
                snapshot = self.system_modules.get(module_name)
                if snapshot is None:
                    reasons.append(f"module_missing:{module_name}")
                    continue
                level, operating_state = snapshot
                # HH_260730 - Intentional sensor dummies are explicit WARN,
                # not fake hardware-OK.  They remain audible/visible as
                # degraded health but do not suppress all mission speech.
                if level >= self.ERROR_LEVEL:
                    reasons.append(f"module_unhealthy:{module_name}:{level}")
                normalized_state = operating_state.strip().upper()
                if normalized_state in self._NOT_READY_OPERATING_STATES:
                    state_label = normalized_state.lower()
                    reasons.append(
                        f"module_not_ready:{module_name}:{state_label}"
                    )

        if not self.planning_received:
            reasons.append("planning_state_missing")
        elif self.planning_state in self._UNSAFE_PLANNING_STATES:
            planning_state = self.planning_state.lower() or "missing"
            reasons.append(f"planning_not_ready:{planning_state}")
        elif (
            require_idle
            and self.planning_state not in self._IDLE_PLANNING_STATES
        ):
            reasons.append(f"planning_not_idle:{self.planning_state.lower()}")
        if not self.action_server_ready:
            reasons.append("planning_action_server_missing")

        if not self.localization_received:
            reasons.append("localization_mode_missing")
        elif self.localization_mode > self.max_ready_localization_mode:
            reasons.append(f"localization_mode:{self.localization_mode}")

        if not self.tf_ready:
            reasons.append("tf_unavailable")

        if not self.gate_received:
            reasons.append("control_gate_missing")
        else:
            if self.gate_level >= self.ERROR_LEVEL:
                reasons.append(f"control_gate_unhealthy:{self.gate_level}")
            allowed_gate_states = self._IDLE_GATE_STATES
            if not require_idle and self.engaged:
                allowed_gate_states = self._ACTIVE_GATE_STATES
            if self.gate_state not in allowed_gate_states:
                gate_state = self.gate_state.lower() or "missing"
                reasons.append(f"control_gate_state:{gate_state}")

        if not self.platform_received:
            reasons.append("platform_status_missing")
        else:
            if self.platform_estop:
                reasons.append("platform_estop")
            if self.platform_error_code != 0:
                reasons.append(f"platform_error:{self.platform_error_code}")

        if not self.engage_received:
            reasons.append("engage_state_missing")
        elif require_idle and self.engaged:
            reasons.append("engaged")

        return tuple(reasons)

    @property
    def ready(self) -> bool:
        return not self.readiness_reasons(require_idle=True)

    @property
    def operational(self) -> bool:
        return not self.readiness_reasons(require_idle=False)

    def _events_after_update(self) -> list[VoiceEvent]:
        events: list[VoiceEvent] = []
        if self.startup_announced and self.ready and not self.ready_announced:
            self.ready_announced = True
            events.append(VoiceEvent("system.ready", priority=1))

        motion_event = self._motion_event()
        if motion_event is not None:
            events.append(motion_event)
        arrival_event = self._arrival_event()
        if arrival_event is not None:
            events.append(arrival_event)
        return events

    def _valid_goal(self) -> bool:
        source = self.active_goal_source.strip().lower()
        return bool(source) and source not in {"none", "startup", "unknown"}

    def travel_context(self) -> str:
        """Return ``"site"``, ``"drop_zone"`` or ``""`` for the current trip."""

        if not self._valid_goal():
            return ""
        if self.planning_state not in self._TRAVEL_STATES:
            return ""
        if self.planning_scenario in self._ROUTE_TO_DROP_ZONE_SCENARIOS:
            # A return that is still only requested has no released goal yet.
            if (
                self.active_mission_key != self.return_mission_key
                or self.return_requested
            ):
                return ""
            return "drop_zone"
        if (
            self.planning_state == "RUNNING"
            and self.planning_scenario in self._ROUTE_TO_SITE_SCENARIOS
        ):
            return "site"
        return ""

    @property
    def travel_active(self) -> bool:
        """True while a departure was announced and the trip is still running.

        This drives the music bed, so it deliberately survives a safety hold:
        the bed ducks under the obstacle cue instead of restarting.
        """

        return (
            self.ready_announced
            and self.engaged
            and bool(self.travel_context())
            and self._route_epoch in self._motion_started_route_epochs
        )

    def travel_announce_events(self) -> list[VoiceEvent]:
        """Reminders to repeat over the bed, empty when no trip is running."""

        if not self.travel_active:
            return []
        keys = self._TRAVEL_ANNOUNCE_KEYS.get(self.travel_context(), ())
        return [VoiceEvent(key, priority=0) for key in keys]

    @property
    def obstacle_hold_announced(self) -> bool:
        """True while the announced hold is still blocking the robot."""

        return (
            self._obstacle_announced
            and self._obstacle_hold_active()
            and self.engaged
            and self.ready_announced
        )

    def obstacle_repeat_events(self) -> list[VoiceEvent]:
        """Standing explanation while the robot waits out a blocked route."""

        if not self.obstacle_hold_announced:
            return []
        return [VoiceEvent("navigation.please_step_aside", priority=1)]

    def _motion_event(self) -> Optional[VoiceEvent]:
        if (
            not self.ready_announced
            or not self.operational
            or not self.engaged
        ):
            return None

        key = self._DEPARTURE_KEYS.get(self.travel_context(), "")
        if not key:
            return None

        signature = (
            self._engage_epoch,
            self._route_epoch,
            key,
            self.active_goal_source,
            self.active_mission_key,
        )
        if signature in self._announced_motion_signatures:
            return None
        self._announced_motion_signatures.add(signature)
        self._motion_started_route_epochs.add(self._route_epoch)
        return VoiceEvent(key, priority=1)

    def _arrival_event(self) -> Optional[VoiceEvent]:
        if not self.ready_announced or not self._valid_goal():
            return None
        if self.planning_state != "GOAL_REACHED":
            return None
        if self.planning_scenario not in self._SITE_ARRIVAL_SCENARIOS:
            return None
        if self._route_epoch not in self._motion_started_route_epochs:
            return None

        signature = (
            self._route_epoch,
            self.active_goal_source,
            self.active_mission_key,
        )
        if signature in self._announced_arrival_signatures:
            return None
        self._announced_arrival_signatures.add(signature)
        return VoiceEvent("navigation.arrived_campsite", priority=1)

    def _obstacle_hold_active(self) -> bool:
        return (
            self.gate_state == "ROUTE_SAFETY_HOLD"
            or "cost_stop_latched" in self.gate_message
            or "cost_hold=" in self.gate_message
        )


def module_snapshots(
    values: Iterable[tuple[str, int, str]]
) -> dict[str, ModuleSnapshot]:
    """Build the compact module mapping used by :class:`VoiceEventPolicy`."""

    return {
        str(name): (int(level), str(operating_state))
        for name, level, operating_state in values
    }
