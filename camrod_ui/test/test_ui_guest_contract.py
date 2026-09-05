"""Guest UI regression tests for the shared Robot UI mission contract."""

from pathlib import Path
import threading
from types import SimpleNamespace
import unittest

from avg_msgs.msg import AvgServiceState, ModuleState, MotionOperation
from builtin_interfaces.msg import Time

from camrod_ui.ui_backend_node import UiBackendNode
from camrod_ui.ui_guest_node import (
    UiGuestNode,
    guest_gate_safety_hold,
    guest_mission_cancel_available,
    guest_mission_dispatch_ready,
    normalize_platform_battery_percent,
)


class GuestUiContractTest(unittest.TestCase):
    def test_platform_battery_fraction_matches_robot_ui_percent(self) -> None:
        self.assertEqual(normalize_platform_battery_percent(0.80), 80)
        self.assertEqual(normalize_platform_battery_percent(0.349), 35)
        self.assertEqual(normalize_platform_battery_percent(1.20), 100)

    def test_guest_dispatch_requires_stationary_state_and_soc_margin(self) -> None:
        self.assertTrue(
            guest_mission_dispatch_ready(AvgServiceState.DROP_ZONE_WAIT, 35, 35)
        )
        self.assertTrue(
            guest_mission_dispatch_ready(AvgServiceState.CHARGING, 80, 35)
        )
        self.assertTrue(
            guest_mission_dispatch_ready(
                AvgServiceState.WAITING_FOR_CHARGING, 50, 35
            )
        )
        self.assertFalse(
            guest_mission_dispatch_ready(AvgServiceState.CHARGING, 34, 35)
        )
        self.assertFalse(
            guest_mission_dispatch_ready(
                AvgServiceState.WAITING_FOR_CHARGING, 34, 35
            )
        )
        # The approach itself is not a parked-ready state.  Only the terminal
        # charger wait left behind after successful parking is dispatchable.
        self.assertFalse(
            guest_mission_dispatch_ready(
                AvgServiceState.DROP_ZONE_PARKING, 80, 35
            )
        )
        self.assertFalse(
            guest_mission_dispatch_ready(AvgServiceState.MOVING_TO_SITE, 80, 35)
        )

    def test_guest_cancel_is_available_only_while_motion_is_owned(self) -> None:
        cancellable_states = {
            AvgServiceState.MOVING_TO_SITE,
            AvgServiceState.SITE_ENTRY,
            AvgServiceState.RECALL_TO_SITE_ROAD,
            AvgServiceState.GUEST_RECALL_SERVICE,
            AvgServiceState.RETURNING_TO_DROP_ZONE,
            AvgServiceState.RETURN_WITH_CARGO,
            AvgServiceState.DROP_ZONE_PARKING,
            AvgServiceState.DEPARTING_CHARGER,
            AvgServiceState.DEPARTING_DROP_ZONE,
        }
        for state in cancellable_states:
            with self.subTest(state=state):
                self.assertTrue(guest_mission_cancel_available(state))

        for state in {
            AvgServiceState.DROP_ZONE_WAIT,
            AvgServiceState.UNLOAD_WAIT,
            AvgServiceState.GUEST_LOADING_WAIT,
            AvgServiceState.WAITING_FOR_CHARGING,
            AvgServiceState.CHARGING,
            AvgServiceState.OPERATOR_STOPPED,
        }:
            with self.subTest(state=state):
                self.assertFalse(guest_mission_cancel_available(state))

    def test_guest_cancel_publishes_typed_operation_request(self) -> None:
        published = []

        class Publisher:
            @staticmethod
            def publish(message: MotionOperation) -> None:
                published.append(message)

        stamp = Time(sec=123, nanosec=456)
        node = SimpleNamespace(
            pub_operation_request=Publisher(),
            ui_camping_site_operation_request_topic=(
                "/ui/camping_site_operation_request"
            ),
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=lambda: stamp)
            ),
            get_logger=lambda: SimpleNamespace(info=lambda _message: None),
        )

        UiGuestNode._publish_cancel(node)

        self.assertEqual(len(published), 1)
        self.assertEqual(published[0].operation, MotionOperation.CANCEL)
        self.assertEqual(published[0].source, "guest:cancel")
        self.assertEqual(published[0].header.stamp, stamp)

    def test_uncharged_parked_guest_dispatch_enters_backend_station_exit(self) -> None:
        events = []
        timers = []

        class Logger:
            @staticmethod
            def info(_message: str) -> None:
                pass

            @staticmethod
            def warn(_message: str) -> None:
                pass

        class Timer:
            def __init__(self, period_s, callback) -> None:
                self.period_s = period_s
                self.callback = callback
                self.cancelled = False

            def cancel(self) -> None:
                self.cancelled = True

        backend = SimpleNamespace(
            _drop_zone_exit_active=False,
            _charging_departure_delay_pending=False,
            _pending_site_after_drop_zone_exit=None,
            _drop_zone_exit_handoff_ready=False,
            _drop_zone_exit_failure_latched=False,
            _drop_zone_exit_waiting_for_fresh_status=False,
            _drop_zone_exit_cancel_suppressed=False,
            _charging_departure_from_charger=False,
            _charging_departure_transition_lock=threading.Lock(),
            _charging_departure_transition_timer=None,
            _latest_platform_is_charging=False,
            _latest_service_state=int(AvgServiceState.WAITING_FOR_CHARGING),
            _active_mission_site="",
            _active_mission_source="",
            _service_metrics=None,
            _lock=threading.Lock(),
            _state=SimpleNamespace(battery_percentage=50),
            _runtime_policy=SimpleNamespace(update_goal_received=lambda _mode: None),
            require_battery_for_mission_dispatch=True,
            minimum_mission_dispatch_battery_percent=35,
            charging_departure_delay_s=7.0,
            publish_engage_from_destination=False,
            publish_mission_engage_from_destination=False,
            _resolve_mission_key_for_site=lambda _site: "camping_site_2",
            _is_site_occupied=lambda _site: False,
            _site_arrival_match=lambda _site: (
                False,
                "camping_site_2",
                10.0,
                "outside_arrival_radius",
            ),
            _update_runtime_state=lambda callback: callback(),
            _schedule_broadcast=lambda payload: events.append(("broadcast", payload)),
            _publish_parking_operation=lambda operation, source: events.append(
                ("parking", operation, source)
            ),
            _publish_drop_zone_operation=lambda operation, source: events.append(
                ("drop_zone", operation, source)
            ),
            _publish_service_state=lambda state, source: events.append(
                ("state", state, source)
            ),
            _publish_engage=lambda enabled, source, **_kwargs: events.append(
                ("engage", enabled, source)
            ),
            _publish_platform_drive_enable=lambda enabled, source: events.append(
                ("drive_enable", enabled, source)
            ),
            create_timer=lambda period, callback: timers.append(
                Timer(period, callback)
            ) or timers[-1],
            destroy_timer=lambda timer: events.append(("destroy_timer", timer)),
            get_logger=lambda: Logger(),
        )
        backend._mission_dispatch_battery_block = lambda site: (
            UiBackendNode._mission_dispatch_battery_block(backend, site)
        )

        # A reverse-parking charge timeout reports module ERROR without
        # replacing the last service state.  That retained terminal wait is the
        # physical parked evidence used for departure admission.
        parking_error = ModuleState()
        parking_error.operating_state = "ERROR"
        UiBackendNode._on_parking_controller_status(
            backend, "reverse_parking", parking_error
        )
        self.assertEqual(
            backend._latest_service_state,
            int(AvgServiceState.WAITING_FOR_CHARGING),
        )
        self.assertTrue(
            guest_mission_dispatch_ready(
                backend._latest_service_state,
                backend._state.battery_percentage,
                backend.minimum_mission_dispatch_battery_percent,
            )
        )

        result = UiBackendNode._apply_destination_command(
            backend, site="B2", run=True, source="guest:test"
        )

        self.assertTrue(result["run"])
        self.assertFalse(result["goal_pose_published"])
        self.assertTrue(backend._charging_departure_delay_pending)
        self.assertFalse(backend._drop_zone_exit_active)
        self.assertEqual(
            backend._pending_site_after_drop_zone_exit,
            ("B2", "camping_site_2", "guest:test"),
        )
        self.assertFalse(any(
            event[0] == "drop_zone" and event[1] == MotionOperation.EXIT
            for event in events
        ))

        self.assertEqual([timer.period_s for timer in timers], [7.0])
        timers[0].callback()

        self.assertFalse(backend._charging_departure_delay_pending)
        self.assertTrue(backend._drop_zone_exit_active)
        self.assertIn(
            ("drop_zone", MotionOperation.EXIT, "guest:test:site_departure"),
            events,
        )

    def test_guest_frontend_exposes_parked_charge_wait_for_dispatch(self) -> None:
        html = (
            Path(__file__).resolve().parents[1]
            / "camrod_ui_guest"
            / "assets"
            / "guest_frontend"
            / "index.html"
        ).read_text(encoding="utf-8")

        self.assertIn("phase === 'waiting_for_charging'", html)
        self.assertIn("currentPhase === 'waiting_for_charging'", html)
        self.assertIn("The robot is parked and ready for a new mission.", html)

    def test_guest_frontend_exposes_cancel_for_travel_return_and_docking(self) -> None:
        package_root = Path(__file__).resolve().parents[1]
        html = (
            package_root
            / "camrod_ui_guest"
            / "assets"
            / "guest_frontend"
            / "index.html"
        ).read_text(encoding="utf-8")
        guest_node = (
            package_root
            / "runtime"
            / "python"
            / "camrod_ui"
            / "ui_guest_node.py"
        ).read_text(encoding="utf-8")

        self.assertIn('id="cancelCard"', html)
        self.assertIn("showMissionCancel('사이트 이동 취소')", html)
        self.assertIn("showMissionCancel('도킹 취소')", html)
        self.assertIn("phase === 'recall' ? '호출 취소' : '복귀 취소'", html)
        self.assertIn("JSON.stringify({ action: 'cancel' })", html)
        self.assertIn('elif action == "cancel":', guest_node)
        self.assertIn("node._publish_cancel()", guest_node)
        self.assertNotIn("취소할 수 없습니다.", html)

    def test_guest_uses_control_hold_not_generic_warning_as_safety_overlay(self) -> None:
        self.assertTrue(guest_gate_safety_hold("ROUTE_SAFETY_HOLD"))
        self.assertTrue(
            guest_gate_safety_hold(
                "ENABLED", "route_safety_hold=lanelet_footprint_cost"
            )
        )
        self.assertFalse(guest_gate_safety_hold("ENABLED", "system warning"))


if __name__ == "__main__":
    unittest.main()
