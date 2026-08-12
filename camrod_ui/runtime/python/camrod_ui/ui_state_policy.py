"""Pure readiness and mission-phase policy for the operator UI."""

from __future__ import annotations

from typing import Mapping, Sequence, Tuple


ModuleSnapshot = Tuple[int, str]


class UiStatePolicy:
    """Combine runtime prerequisites into one source-neutral UI state."""

    ERROR_LEVEL = 2
    NORMAL_LOCALIZATION_MODE = 0

    INITIALIZING = "INITIALIZING"
    READY = "READY"
    GOAL_RECEIVED = "GOAL_RECEIVED"
    PATH_PREPARING = "PATH_PREPARING"
    DRIVING = "DRIVING"
    SAFETY_STOP = "SAFETY_STOP"
    ARRIVED = "ARRIVED"
    STOPPED = "STOPPED"

    _IDLE_PLANNING_STATES = frozenset({"READY", "WAIT_DZ"})
    _UNSAFE_PLANNING_STATES = frozenset(
        {"", "INIT", "WARN_RECOVERY", "ERROR_STOP"}
    )
    _IDLE_GATE_STATES = frozenset({"STANDBY", "CHARGING"})
    _ACTIVE_GATE_STATES = frozenset({"ENABLED", "DEPARTING_CHARGER"})
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
        max_ready_localization_mode: int = NORMAL_LOCALIZATION_MODE,
    ) -> None:
        self.required_modules = tuple(
            dict.fromkeys(
                str(name).strip()
                for name in required_modules
                if str(name).strip()
            )
        )
        self.max_ready_localization_mode = int(max_ready_localization_mode)

        self.system_received = False
        self.system_modules: dict[str, ModuleSnapshot] = {}
        self.planning_received = False
        self.planning_state = ""
        self.planning_scenario = ""
        self.active_mission_key = ""
        self.active_goal_source = ""
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
        self.goal_received = False
        self.nav_status = 0
        self.initialized_once = False

    def update_system(
        self, modules: Mapping[str, ModuleSnapshot]
    ) -> None:
        self.system_received = True
        self.system_modules = {
            str(name): (int(snapshot[0]), str(snapshot[1]))
            for name, snapshot in modules.items()
        }
        self._latch_initialization()

    def update_planning(
        self,
        *,
        state: str,
        scenario: str,
        active_mission_key: str,
        active_goal_source: str,
    ) -> None:
        self.planning_received = True
        self.planning_state = str(state).strip().upper()
        self.planning_scenario = str(scenario).strip().upper()
        self.active_mission_key = str(active_mission_key).strip()
        self.active_goal_source = str(active_goal_source).strip()
        if self._valid_goal():
            self.goal_received = True
        if self.planning_state in {"READY", "WAIT_DZ"} and not self._valid_goal():
            self.goal_received = False
            self.nav_status = 0
        self._latch_initialization()

    def update_action_server(self, ready: bool) -> None:
        self.action_server_ready = bool(ready)
        self._latch_initialization()

    def update_localization(self, mode: int) -> None:
        self.localization_received = True
        self.localization_mode = int(mode)
        self._latch_initialization()

    def update_tf(self, ready: bool) -> None:
        self.tf_ready = bool(ready)
        self._latch_initialization()

    def update_gate(
        self, *, level: int, operating_state: str, message: str
    ) -> None:
        self.gate_received = True
        self.gate_level = int(level)
        self.gate_state = str(operating_state).strip().upper()
        self.gate_message = str(message).strip().lower()
        self._latch_initialization()

    def update_platform(self, *, estop: bool, error_code: int) -> None:
        self.platform_received = True
        self.platform_estop = bool(estop)
        self.platform_error_code = int(error_code)
        self._latch_initialization()

    def update_engaged(self, engaged: bool) -> None:
        self.engage_received = True
        self.engaged = bool(engaged)
        self._latch_initialization()

    def update_goal_received(self, source: str = "") -> None:
        if str(source).strip():
            self.active_goal_source = str(source).strip()
        self.goal_received = True

    def update_goal_source(self, source: str) -> None:
        self.active_goal_source = str(source).strip()

    def update_nav_status(self, status: int) -> None:
        self.nav_status = int(status)

    def readiness_reasons(
        self, *, require_idle: bool = False
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
                # HH_260730 - WARN includes intentional DUMMY DATA and other
                # degraded-but-operational states.  Keep it visible in system
                # health without trapping the mission display in INITIALIZING.
                if level >= self.ERROR_LEVEL:
                    reasons.append(f"module_unhealthy:{module_name}:{level}")
                normalized_state = operating_state.strip().upper()
                if normalized_state in self._NOT_READY_OPERATING_STATES:
                    reasons.append(
                        f"module_not_ready:{module_name}:"
                        f"{normalized_state.lower()}"
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
        """Whether all prerequisites remain valid in the current mode."""

        return self.initialized_once and not self.readiness_reasons(
            require_idle=False
        )

    @property
    def mission_source(self) -> str:
        source = self.active_goal_source.strip().lower()
        if source.startswith("manual"):
            return "manual"
        if self._valid_goal():
            return "ui"
        return "none"

    @property
    def mission_phase(self) -> str:
        if self.planning_state == "ERROR_STOP":
            return self.STOPPED
        if self._safety_hold_active():
            return self.SAFETY_STOP
        if not self.ready:
            reasons = self.readiness_reasons(require_idle=False)
            if (
                self.initialized_once
                and reasons
                and all(
                    reason.startswith("control_gate_state:")
                    for reason in reasons
                )
            ):
                # HH_260812 - A command gate that is merely holding is not an
                # uninitialized robot.  Picking a destination is intent, not
                # motion, so an already-initialized stack stays selectable and
                # the gate release alone decides when the robot actually moves.
                if self._valid_goal():
                    return self.PATH_PREPARING
                return self.READY
            return self.INITIALIZING
        if self.planning_state == "GOAL_REACHED" and self._valid_goal():
            return self.ARRIVED
        if not self._valid_goal():
            return self.READY
        if (
            self.planning_state in {"RUNNING", "RETURNING"}
            and self.engaged
            and self.gate_state in self._ACTIVE_GATE_STATES
        ):
            return self.DRIVING
        if self.nav_status in {1, 2, 3}:
            return self.PATH_PREPARING
        if self.planning_state in {"RUNNING", "RETURNING"}:
            return self.PATH_PREPARING
        if self.goal_received:
            return self.GOAL_RECEIVED
        return self.READY

    def _latch_initialization(self) -> None:
        # HH_260810 - Readiness belongs to sensors, localization, planning,
        # control, and platform health.  Do not require an idle interval: an
        # engage or campsite request may arrive before the final prerequisite,
        # and no goal event should be needed to release INITIALIZING.
        if (
            not self.initialized_once
            and not self.readiness_reasons(require_idle=False)
        ):
            self.initialized_once = True

    def _valid_goal(self) -> bool:
        source = self.active_goal_source.strip().lower()
        return bool(source) and source not in {"none", "startup", "unknown"}

    def _safety_hold_active(self) -> bool:
        if not self._valid_goal():
            return False
        return (
            self.planning_state == "WARN_RECOVERY"
            or self.gate_state in {"SAFETY_HOLD", "ROUTE_SAFETY_HOLD"}
            or "cost_stop_latched" in self.gate_message
            or "cost_hold=" in self.gate_message
            or "route_safety_hold=" in self.gate_message
        )
