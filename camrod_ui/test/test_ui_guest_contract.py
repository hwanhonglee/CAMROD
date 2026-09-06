"""Guest UI regression tests for the shared Robot UI mission contract."""

import json
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

from avg_msgs.msg import (
    AvgServiceState,
    ModuleState,
    MotionOperation,
    UiDestinationCommand,
)
from builtin_interfaces.msg import Time
from std_msgs.msg import String

from camrod_ui.ui_backend_node import UiBackendNode
from camrod_ui.ui_guest_node import (
    UiGuestNode,
    destination_request_owner,
    guest_gate_safety_hold,
    guest_mission_cancel_available,
    guest_mission_dispatch_ready,
    guest_usage_complete_available,
    normalize_platform_battery_percent,
)


class GuestUiContractTest(unittest.TestCase):
    def test_guest_consumes_only_backend_filtered_service_lifecycle(self) -> None:
        source = (
            Path(__file__).resolve().parents[1]
            / "runtime"
            / "python"
            / "camrod_ui"
            / "ui_guest_node.py"
        ).read_text(encoding="utf-8")
        subscription_block = source[
            source.index("state_qos = QoSProfile(") :
            source.index("self.sub_battery = self.create_subscription(")
        ]
        self.assertIn("self.sub_service_state = None", subscription_block)
        self.assertNotIn("self._on_service_state", subscription_block)

        broadcasts = []
        node = SimpleNamespace(
            _lock=threading.Lock(),
            _service_state=int(AvgServiceState.DROP_ZONE_WAIT),
            _active_site="",
            _active_request_intent="",
            _active_request_owner="",
            _active_mission_generation=0,
            _active_request_retryable=False,
            _mission_terminal_clear_armed=False,
            _guest_return_request_pending=False,
            _pending_dispatch_nonce="",
            _pending_dispatch_deadline_s=0.0,
            site_names=[f"B{index}" for index in range(1, 14)],
            _schedule_broadcast=broadcasts.append,
            get_logger=lambda: SimpleNamespace(
                info=lambda _message: None,
                warn=lambda _message: None,
            ),
        )
        accepted = String()
        accepted.data = json.dumps({
            "accepted": True,
            "request_site": "B7",
            "request_source": "service_lifecycle",
            "active_site": "B7",
            "active_source": "guest:dispatch:r=current",
            "active_intent": "recall",
            "active_generation": 42,
            "service_state": int(AvgServiceState.GUEST_LOADING_WAIT),
            "service_state_name": "GUEST_LOADING_WAIT",
        })

        UiGuestNode._on_destination_dispatch_status(node, accepted)

        self.assertEqual(
            node._service_state, int(AvgServiceState.GUEST_LOADING_WAIT)
        )
        self.assertEqual(node._active_mission_generation, 42)
        self.assertEqual(broadcasts[-1]["service_state_name"], "GUEST_LOADING_WAIT")
        self.assertEqual(broadcasts[-1]["phase"], "arrived")

    def test_destination_request_owner_separates_guest_robot_and_operator(self) -> None:
        self.assertEqual(destination_request_owner("guest"), "guest")
        self.assertEqual(destination_request_owner("guest:kiosk"), "guest")
        self.assertEqual(destination_request_owner("robot_ui:recall"), "robot")
        self.assertEqual(destination_request_owner("http_ui_destination"), "operator")
        self.assertEqual(destination_request_owner(""), "")

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
        self.assertFalse(
            guest_mission_dispatch_ready(
                AvgServiceState.CHARGING, 80, 35, "recall"
            )
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
                self.assertTrue(
                    guest_mission_cancel_available(
                        state, "recall", "B1", "guest"
                    )
                )

        for state in {
            AvgServiceState.DROP_ZONE_WAIT,
            AvgServiceState.UNLOAD_WAIT,
            AvgServiceState.GUEST_LOADING_WAIT,
            AvgServiceState.WAITING_FOR_CHARGING,
            AvgServiceState.CHARGING,
            AvgServiceState.OPERATOR_STOPPED,
        }:
            with self.subTest(state=state):
                self.assertFalse(
                    guest_mission_cancel_available(
                        state, "recall", "B1", "guest"
                    )
                )

        self.assertFalse(
            guest_mission_cancel_available(
                AvgServiceState.MOVING_TO_SITE, "delivery", "B1", "guest"
            )
        )
        self.assertFalse(
            guest_mission_cancel_available(
                AvgServiceState.RECALL_TO_SITE_ROAD, "recall", "", "guest"
            )
        )
        self.assertFalse(
            guest_mission_cancel_available(
                AvgServiceState.RECALL_TO_SITE_ROAD,
                "recall",
                "B1",
                "robot",
            )
        )

    def test_guest_usage_complete_requires_owned_recall_arrival(self) -> None:
        self.assertTrue(
            guest_usage_complete_available(
                AvgServiceState.GUEST_LOADING_WAIT, "recall", "B13", "guest"
            )
        )
        self.assertFalse(
            guest_usage_complete_available(
                AvgServiceState.GUEST_LOADING_WAIT, "delivery", "B13", "guest"
            )
        )
        self.assertFalse(
            guest_usage_complete_available(
                AvgServiceState.UNLOAD_WAIT, "recall", "B13", "guest"
            )
        )
        self.assertFalse(
            guest_usage_complete_available(
                AvgServiceState.GUEST_LOADING_WAIT, "recall", "B13", "robot"
            )
        )

    def test_guest_recall_reservation_blocks_competing_site_during_departure_dwell(
        self,
    ) -> None:
        broadcasts = []
        node = SimpleNamespace(
            _lock=threading.Lock(),
            _service_state=int(AvgServiceState.CHARGING),
            _battery=80,
            _active_site="",
            _active_request_intent="",
            _mission_terminal_clear_armed=False,
            _guest_return_request_pending=False,
            minimum_mission_dispatch_battery_percent=35,
            _schedule_broadcast=broadcasts.append,
        )

        first = UiGuestNode._reserve_guest_recall_request(node, "B2")
        second = UiGuestNode._reserve_guest_recall_request(node, "B7")

        self.assertEqual(first, (True, "", int(AvgServiceState.CHARGING), 80))
        self.assertFalse(second[0])
        self.assertEqual(second[1], "mission_already_active")
        self.assertEqual(node._active_site, "B2")
        self.assertEqual(node._active_request_intent, "recall")
        self.assertEqual(node._dispatch_identity_revision, 1)
        self.assertRegex(node._pending_dispatch_nonce, r"^1-[0-9a-f]+$")
        self.assertGreater(node._pending_dispatch_deadline_s, 0.0)
        self.assertEqual(
            broadcasts,
            [{
                "site": "B2",
                "request_intent": "recall",
                "request_owner": "guest",
                "mission_retryable": False,
                "mission_generation": 0,
                "identity_revision": 1,
            }],
        )

    def test_guest_dispatch_ack_timeout_releases_only_generation_zero_reservation(
        self,
    ) -> None:
        broadcasts = []
        warnings = []
        node = SimpleNamespace(
            _lock=threading.Lock(),
            _active_site="B2",
            _active_request_intent="recall",
            _active_request_owner="guest",
            _active_mission_generation=0,
            _active_request_retryable=False,
            _mission_terminal_clear_armed=False,
            _guest_return_request_pending=False,
            _pending_dispatch_nonce="1-deadbeef",
            _pending_dispatch_deadline_s=10.0,
            _schedule_broadcast=broadcasts.append,
            get_logger=lambda: SimpleNamespace(
                warn=warnings.append,
                info=lambda _message: None,
            ),
        )

        with mock.patch(
            "camrod_ui.ui_guest_node.time.monotonic", return_value=10.1
        ):
            UiGuestNode._expire_pending_dispatch_ack(node)

        self.assertEqual(node._active_site, "")
        self.assertEqual(node._active_request_intent, "")
        self.assertEqual(node._active_request_owner, "")
        self.assertEqual(node._pending_dispatch_nonce, "")
        self.assertEqual(node._pending_dispatch_deadline_s, 0.0)
        self.assertIn("site=B2 nonce=1-deadbeef", warnings[-1])
        self.assertEqual(broadcasts[-1]["error"], "dispatch_ack_timeout")
        self.assertEqual(broadcasts[-1]["mission_generation"], 0)

        # Once a backend ACK assigned a generation, the local watchdog can no
        # longer clear that authoritative mission even if its old deadline is
        # observed later.
        node._active_site = "B2"
        node._active_request_intent = "recall"
        node._active_request_owner = "guest"
        node._active_mission_generation = 1_787_000_000_000_001
        node._pending_dispatch_nonce = "1-deadbeef"
        node._pending_dispatch_deadline_s = 10.0
        with mock.patch(
            "camrod_ui.ui_guest_node.time.monotonic", return_value=20.0
        ):
            UiGuestNode._expire_pending_dispatch_ack(node)
        self.assertEqual(node._active_site, "B2")
        self.assertEqual(
            node._active_mission_generation,
            1_787_000_000_000_001,
        )
        self.assertEqual(len(broadcasts), 1)

    def test_guest_dispatch_stale_nonce_ack_cannot_replace_current_reservation(
        self,
    ) -> None:
        broadcasts = []
        warnings = []
        node = SimpleNamespace(
            _lock=threading.Lock(),
            _active_site="B7",
            _active_request_intent="recall",
            _active_request_owner="guest",
            _active_mission_generation=0,
            _active_request_retryable=False,
            _mission_terminal_clear_armed=False,
            _guest_return_request_pending=False,
            _pending_dispatch_nonce="2-current",
            _pending_dispatch_deadline_s=25.0,
            site_names=[f"B{index}" for index in range(1, 14)],
            _schedule_broadcast=broadcasts.append,
            get_logger=lambda: SimpleNamespace(
                warn=warnings.append,
                info=lambda _message: None,
            ),
        )
        stale = String()
        stale.data = json.dumps({
            "accepted": True,
            "request_site": "B2",
            "request_source": "guest:dispatch:r=1-stale",
            "active_site": "B2",
            "active_source": "guest:dispatch:r=1-stale",
            "active_intent": "recall",
            "active_generation": 1_787_000_000_000_001,
        })

        UiGuestNode._on_destination_dispatch_status(node, stale)

        self.assertEqual(node._active_site, "B7")
        self.assertEqual(node._active_mission_generation, 0)
        self.assertEqual(node._pending_dispatch_nonce, "2-current")
        self.assertEqual(broadcasts, [])
        self.assertIn("request_nonce=1-stale pending=2-current", warnings[-1])

        # Older Guest operation statuses have no dispatch nonce. They must not
        # erase a newer generation-0 site reservation while its ACK is in flight.
        stale_without_nonce = String()
        stale_without_nonce.data = json.dumps({
            "accepted": True,
            "request_site": "B2",
            "request_source": "guest:usage_complete:site=B2:g=41",
            "active_site": "B2",
            "active_source": "guest:dispatch:r=1-stale",
            "active_intent": "recall",
            "active_generation": 41,
            "service_state": int(AvgServiceState.RETURN_WITH_CARGO),
            "service_state_name": "RETURN_WITH_CARGO",
        })
        UiGuestNode._on_destination_dispatch_status(node, stale_without_nonce)

        self.assertEqual(node._active_site, "B7")
        self.assertEqual(node._active_mission_generation, 0)
        self.assertEqual(node._pending_dispatch_nonce, "2-current")
        self.assertEqual(broadcasts, [])
        self.assertIn("request_nonce=none", warnings[-1])

        current = String()
        current.data = json.dumps({
            "accepted": True,
            "request_site": "B7",
            "request_source": "guest:dispatch:r=2-current",
            "active_site": "B7",
            "active_source": "guest:dispatch:r=2-current",
            "active_intent": "recall",
            "active_generation": 1_787_000_000_000_002,
        })
        UiGuestNode._on_destination_dispatch_status(node, current)

        self.assertEqual(node._active_site, "B7")
        self.assertEqual(
            node._active_mission_generation,
            1_787_000_000_000_002,
        )
        self.assertEqual(node._pending_dispatch_nonce, "")
        self.assertEqual(broadcasts[-1]["mission_generation"], 1_787_000_000_000_002)

    def test_guest_return_reservation_is_owned_and_single_shot(self) -> None:
        node = SimpleNamespace(
            _lock=threading.Lock(),
            _service_state=int(AvgServiceState.GUEST_LOADING_WAIT),
            _active_site="B11",
            _active_request_intent="recall",
            _active_request_owner="guest",
            _active_mission_generation=42,
            _guest_return_request_pending=False,
        )

        first = UiGuestNode._reserve_guest_return_request(node)
        second = UiGuestNode._reserve_guest_return_request(node)

        self.assertEqual(
            first, (True, "", int(AvgServiceState.GUEST_LOADING_WAIT))
        )
        self.assertEqual(
            second,
            (False, "request_in_progress", int(AvgServiceState.GUEST_LOADING_WAIT)),
        )

        node._guest_return_request_pending = False
        node._active_request_intent = "delivery"
        denied = UiGuestNode._reserve_guest_return_request(node)
        self.assertEqual(denied[0:2], (False, "not_guest_recall_arrival"))

    def test_guest_cancel_publishes_typed_operation_request(self) -> None:
        published = []

        class Publisher:
            @staticmethod
            def publish(message: MotionOperation) -> None:
                published.append(message)

        stamp = Time(sec=123, nanosec=456)
        node = SimpleNamespace(
            _lock=threading.Lock(),
            _active_site="B4",
            _active_mission_generation=17,
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
        self.assertEqual(published[0].source, "guest:cancel:site=B4:g=17")
        self.assertEqual(published[0].header.stamp, stamp)

    def test_guest_recall_and_return_keep_typed_topic_contracts(self) -> None:
        destinations = []
        operations = []

        class DestinationPublisher:
            @staticmethod
            def publish(message: UiDestinationCommand) -> None:
                destinations.append(message)

        class OperationPublisher:
            @staticmethod
            def publish(message: MotionOperation) -> None:
                operations.append(message)

        stamp = Time(sec=321, nanosec=654)
        node = SimpleNamespace(
            _lock=threading.Lock(),
            _active_site="B13",
            _active_mission_generation=23,
            _pending_dispatch_nonce="23-deadbeef",
            pub_destination=DestinationPublisher(),
            ui_destination_topic="/ui/selected_destination",
            pub_operation_request=OperationPublisher(),
            ui_camping_site_operation_request_topic=(
                "/ui/camping_site_operation_request"
            ),
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=lambda: stamp)
            ),
            get_logger=lambda: SimpleNamespace(info=lambda _message: None),
        )

        UiGuestNode._publish_navigate(node, "B13")
        UiGuestNode._publish_usage_complete(node)

        self.assertEqual(len(destinations), 1)
        self.assertEqual(destinations[0].site, "B13")
        self.assertTrue(destinations[0].run)
        self.assertEqual(
            destinations[0].source,
            "guest:dispatch:r=23-deadbeef",
        )
        self.assertTrue(UiBackendNode._is_guest_recall_source("guest"))
        self.assertEqual(len(operations), 1)
        self.assertEqual(operations[0].operation, MotionOperation.RETURN)
        self.assertEqual(
            operations[0].source,
            "guest:usage_complete:site=B13:g=23",
        )
        self.assertEqual(operations[0].header.stamp, stamp)

    def test_backend_dispatch_site_is_replayed_with_service_state(self) -> None:
        broadcasts = []
        node = SimpleNamespace(
            _lock=threading.Lock(),
            _service_state=int(AvgServiceState.DROP_ZONE_WAIT),
            _active_site="",
            _active_request_intent="",
            _active_request_owner="",
            _active_mission_generation=0,
            _mission_terminal_clear_armed=False,
            _is_charging=False,
            _schedule_broadcast=broadcasts.append,
            _phase_of=lambda state: UiGuestNode._phase_of(node, state),
            _state_name_of=lambda state: UiGuestNode._state_name_of(node, state),
            get_logger=lambda: SimpleNamespace(
                info=lambda _message: None,
                warn=lambda _message: None,
            ),
            site_names=[f"B{index}" for index in range(1, 14)],
        )

        accepted = String()
        accepted.data = json.dumps({
            "accepted": True,
            "request_site": "B8",
            "active_site": "B8",
            "active_source": "guest",
            "active_intent": "recall",
            "active_generation": 9,
        })
        UiGuestNode._on_destination_dispatch_status(node, accepted)

        loading = AvgServiceState()
        loading.state = AvgServiceState.GUEST_LOADING_WAIT
        loading.state_name = "GUEST_LOADING_WAIT"
        UiGuestNode._on_service_state(node, loading)

        self.assertEqual(node._active_site, "B8")
        self.assertEqual(node._active_request_intent, "recall")
        self.assertEqual(node._active_request_owner, "guest")
        self.assertEqual(
            broadcasts[0], {
                "site": "B8",
                "request_intent": "recall",
                "request_owner": "guest",
                "mission_generation": 9,
                "dispatch_accepted": True,
                "dispatch_error": "",
                "dispatch_request_site": "B8",
                "mission_retryable": False,
                "identity_revision": 1,
            }
        )
        self.assertEqual(broadcasts[1]["site"], "B8")
        self.assertEqual(broadcasts[1]["request_intent"], "recall")
        self.assertEqual(broadcasts[1]["phase"], "arrived")

        cleared = String()
        cleared.data = json.dumps({
            "accepted": True,
            "request_site": "B8",
            "active_site": "",
            "active_source": "",
            "active_intent": "",
        })
        UiGuestNode._on_destination_dispatch_status(node, cleared)
        self.assertEqual(node._active_site, "")
        self.assertEqual(node._active_request_intent, "")
        self.assertEqual(node._active_request_owner, "")
        self.assertEqual(broadcasts[-1]["site"], "")
        self.assertEqual(broadcasts[-1]["request_intent"], "")

    def test_destination_request_topic_is_not_treated_as_backend_admission(self) -> None:
        logs = []
        node = SimpleNamespace(
            _active_site="B2",
            _active_request_intent="recall",
            _active_request_owner="robot",
            get_logger=lambda: SimpleNamespace(info=logs.append),
        )
        request = UiDestinationCommand()
        request.site = "B7"
        request.run = True
        request.source = "guest"

        UiGuestNode._on_destination_command(node, request)

        self.assertEqual(node._active_site, "B2")
        self.assertEqual(node._active_request_intent, "recall")
        self.assertEqual(node._active_request_owner, "robot")
        self.assertEqual(len(logs), 1)
        self.assertIn("awaiting backend admission", logs[0])

    def test_rejected_guest_request_restores_backend_authoritative_robot_site(self) -> None:
        broadcasts = []
        node = SimpleNamespace(
            _lock=threading.Lock(),
            # Guest UI optimistically reserves its own click before the ROS
            # callback crosses into ui_backend_node.
            _active_site="B7",
            _active_request_intent="recall",
            _active_request_owner="guest",
            _mission_terminal_clear_armed=False,
            _guest_return_request_pending=False,
            site_names=[f"B{index}" for index in range(1, 14)],
            _schedule_broadcast=broadcasts.append,
            get_logger=lambda: SimpleNamespace(
                info=lambda _message: None,
                warn=lambda _message: None,
            ),
        )
        rejected = String()
        rejected.data = json.dumps({
            "accepted": False,
            "request_site": "B7",
            "request_source": "guest",
            "error": "mission_already_active",
            "active_site": "B2",
            "active_source": "robot_ui:recall",
            "active_intent": "recall",
        })

        UiGuestNode._on_destination_dispatch_status(node, rejected)

        self.assertEqual(node._active_site, "B2")
        self.assertEqual(node._active_request_intent, "recall")
        self.assertEqual(node._active_request_owner, "robot")
        self.assertFalse(
            guest_usage_complete_available(
                AvgServiceState.GUEST_LOADING_WAIT,
                node._active_request_intent,
                node._active_site,
                node._active_request_owner,
            )
        )
        self.assertEqual(broadcasts[-1]["dispatch_accepted"], False)
        self.assertEqual(broadcasts[-1]["dispatch_error"], "mission_already_active")
        self.assertEqual(broadcasts[-1]["site"], "B2")

    def test_unrelated_dispatch_status_preserves_pending_usage_complete(self) -> None:
        node = SimpleNamespace(
            _lock=threading.Lock(),
            _active_site="B7",
            _active_request_intent="recall",
            _active_request_owner="guest",
            _mission_terminal_clear_armed=False,
            _guest_return_request_pending=True,
            site_names=[f"B{index}" for index in range(1, 14)],
            _schedule_broadcast=lambda _payload: None,
            get_logger=lambda: SimpleNamespace(
                info=lambda _message: None,
                warn=lambda _message: None,
            ),
        )
        duplicate = String()
        duplicate.data = json.dumps({
            "accepted": False,
            "request_site": "B2",
            "request_source": "robot_ui:recall",
            "error": "mission_already_active",
            "active_site": "B7",
            "active_source": "guest",
            "active_intent": "recall",
        })

        UiGuestNode._on_destination_dispatch_status(node, duplicate)

        self.assertTrue(node._guest_return_request_pending)

    def test_charging_heartbeat_preserves_recall_intent_until_return_completes(self) -> None:
        broadcasts = []
        node = SimpleNamespace(
            _lock=threading.Lock(),
            _service_state=int(AvgServiceState.CHARGING),
            _active_site="B2",
            _active_request_intent="recall",
            _active_request_owner="guest",
            _mission_terminal_clear_armed=False,
            _guest_return_request_pending=False,
            site_names=[f"B{index}" for index in range(1, 14)],
            _schedule_broadcast=broadcasts.append,
            _phase_of=lambda state: UiGuestNode._phase_of(node, state),
            _state_name_of=lambda state: UiGuestNode._state_name_of(node, state),
            get_logger=lambda: SimpleNamespace(
                info=lambda _message: None,
                warn=lambda _message: None,
            ),
        )

        charging = AvgServiceState()
        charging.state = AvgServiceState.CHARGING
        charging.state_name = "CHARGING"
        UiGuestNode._on_service_state(node, charging)
        self.assertEqual(node._active_site, "B2")
        self.assertEqual(node._active_request_intent, "recall")
        self.assertFalse(node._mission_terminal_clear_armed)
        self.assertEqual(broadcasts[-1]["request_intent"], "recall")

        returning = AvgServiceState()
        returning.state = AvgServiceState.RETURN_WITH_CARGO
        returning.state_name = "RETURN_WITH_CARGO"
        UiGuestNode._on_service_state(node, returning)
        self.assertFalse(node._mission_terminal_clear_armed)
        self.assertEqual(node._active_request_intent, "recall")

        UiGuestNode._on_service_state(node, charging)
        self.assertEqual(node._active_site, "B2")
        self.assertEqual(node._active_request_intent, "recall")

        cleared = String()
        cleared.data = json.dumps({
            "accepted": True,
            "request_site": "B2",
            "active_site": "",
            "active_source": "",
            "active_intent": "",
        })
        UiGuestNode._on_destination_dispatch_status(node, cleared)
        self.assertEqual(node._active_site, "")
        self.assertEqual(node._active_request_intent, "")
        self.assertFalse(node._mission_terminal_clear_armed)
        self.assertEqual(broadcasts[-1]["request_intent"], "")

    def test_dispatch_status_distinguishes_delivery_recall_and_owner(self) -> None:
        broadcasts = []
        node = SimpleNamespace(
            _lock=threading.Lock(),
            _active_site="",
            _active_request_intent="",
            _active_request_owner="",
            _mission_terminal_clear_armed=False,
            _guest_return_request_pending=False,
            site_names=[f"B{index}" for index in range(1, 14)],
            _schedule_broadcast=broadcasts.append,
            get_logger=lambda: SimpleNamespace(
                info=lambda _message: None,
                warn=lambda _message: None,
            ),
        )

        delivery = String()
        delivery.data = json.dumps({
            "accepted": True,
            "request_site": "B4",
            "active_site": "B4",
            "active_source": "ws",
            "active_intent": "delivery",
        })
        UiGuestNode._on_destination_dispatch_status(node, delivery)
        self.assertEqual(node._active_request_intent, "delivery")
        self.assertEqual(node._active_request_owner, "operator")
        self.assertEqual(broadcasts[-1]["request_intent"], "delivery")

        recall = String()
        recall.data = json.dumps({
            "accepted": True,
            "request_site": "B4",
            "active_site": "B4",
            "active_source": "guest:kiosk",
            "active_intent": "recall",
        })
        UiGuestNode._on_destination_dispatch_status(node, recall)
        self.assertEqual(node._active_request_intent, "recall")
        self.assertEqual(node._active_request_owner, "guest")
        self.assertEqual(broadcasts[-1]["request_intent"], "recall")

        robot_recall = String()
        robot_recall.data = json.dumps({
            "accepted": True,
            "request_site": "B9",
            "active_site": "B9",
            "active_source": "robot_ui:recall",
            "active_intent": "recall",
        })
        UiGuestNode._on_destination_dispatch_status(node, robot_recall)
        self.assertEqual(node._active_request_intent, "recall")
        self.assertEqual(node._active_request_owner, "robot")
        self.assertEqual(broadcasts[-1]["request_owner"], "robot")

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
        backend._revoke_manual_drive = lambda _reason: None

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

        self.assertIn(
            "currentServiceStateName === 'WAITING_FOR_CHARGING'", html
        )
        self.assertIn("currentPhase === 'waiting_for_charging'", html)
        self.assertIn("로봇이 주차를 마쳐 새 호출을 받을 수 있습니다.", html)

    def test_guest_frontend_rejects_stale_lifecycle_with_stale_identity(self) -> None:
        html = (
            Path(__file__).resolve().parents[1]
            / "camrod_ui_guest"
            / "assets"
            / "guest_frontend"
            / "index.html"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "const acceptLifecycle = d.identity_revision === undefined || acceptIdentity;",
            html,
        )
        self.assertIn(
            "if (acceptLifecycle && d.service_state_name !== undefined)",
            html,
        )
        self.assertIn(
            "if (acceptLifecycle && d.phase !== undefined)",
            html,
        )

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
        self.assertIn("CANCELLABLE_SERVICE_STATES.has(currentServiceStateName)", html)
        self.assertIn("activeRequestIntent === 'recall'", html)
        self.assertIn("'이용객 호출 취소'", html)
        self.assertIn("showMissionCancel('주차·도킹 취소')", html)
        self.assertIn("'적재 복귀 취소'", html)
        self.assertNotIn("'배송 이동 취소'", html)
        self.assertNotIn("'배송 복귀 취소'", html)
        self.assertIn("JSON.stringify({ action: 'cancel' })", html)
        self.assertIn('elif action == "cancel":', guest_node)
        self.assertIn("node._publish_cancel", guest_node)
        self.assertIn("node._guest_cancel_is_owned", guest_node)
        self.assertNotIn("취소할 수 없습니다.", html)

    def test_guest_frontend_copy_distinguishes_recall_loading_and_return(self) -> None:
        html = (
            Path(__file__).resolve().parents[1]
            / "camrod_ui_guest"
            / "assets"
            / "guest_frontend"
            / "index.html"
        ).read_text(encoding="utf-8")

        for text in (
            "로봇 호출 위치 선택",
            "사이트 내부로 들어가지 않고 도로 측 대기점으로 이동합니다.",
            "적재 완료 · 로봇 복귀",
            "수령 완료 · 로봇 복귀",
            "도로 측 대기점으로 이동 중",
            "짐 싣기 대기",
            "짐 내리기 대기",
            "짐을 싣고 복귀 중",
            "대기·충전 장소로 복귀 중",
            "DELIVERY_TRAVEL_SERVICE_STATES.has(currentServiceStateName)",
            "RECALL_TRAVEL_SERVICE_STATES.has(currentServiceStateName)",
            "DELIVERY_ARRIVAL_SERVICE_STATES.has(currentServiceStateName)",
            "RECALL_ARRIVAL_SERVICE_STATES.has(currentServiceStateName)",
            "DELIVERY_RETURN_SERVICE_STATES.has(currentServiceStateName)",
            "RECALL_RETURN_SERVICE_STATES.has(currentServiceStateName)",
            "이 임무의 완료·복귀는 로봇 관리자 화면에서 진행합니다.",
            "completeCard.style.display = isGuestOwnedRecallArrival ? 'block' : 'none'",
        ):
            self.assertIn(text, html)
        for generic_phase_branch in (
            "else if (phase === 'moving')",
            "else if (phase === 'arrived')",
            "phase === 'returning' || phase === 'recall'",
        ):
            self.assertNotIn(generic_phase_branch, html)
        self.assertNotIn("The robot is returning to the drop zone.", html)

    def test_guest_frontend_requires_explicit_request_intent_for_recall_copy(self) -> None:
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

        self.assertIn("let activeRequestIntent = null;", html)
        self.assertIn("let activeRequestOwner = null;", html)
        self.assertIn("activeRequestIntent = 'recall';", html)
        self.assertIn("activeRequestOwner = 'guest';", html)
        self.assertIn("d.request_intent !== undefined", html)
        self.assertIn("d.request_owner !== undefined", html)
        self.assertIn(
            "activeRequestIntent = ['delivery', 'recall'].includes(replayedIntent)",
            html,
        )
        # A new page/reconnected socket receives this persisted source-derived
        # intent in its very first state payload; it is never reconstructed
        # from RETURN_WITH_CARGO in the browser.
        self.assertIn("request_intent = node._active_request_intent", guest_node)
        self.assertIn('"request_intent": request_intent', guest_node)
        self.assertIn('"request_owner": request_owner', guest_node)
        self.assertIn(
            "const isRecallArrival = activeRequestIntent === 'recall'",
            html,
        )
        self.assertIn(
            "const isRecallReturn = activeRequestIntent === 'recall'",
            html,
        )
        self.assertNotIn(
            "const isRecallReturn = RECALL_RETURN_SERVICE_STATES.has",
            html,
        )
        self.assertIn(
            "currentServiceStateName !== 'RETURN_WITH_CARGO'",
            html,
        )
        self.assertNotIn(
            "['DROP_ZONE_WAIT',\n          'WAITING_FOR_CHARGING'",
            html,
        )
        self.assertIn("self._mission_terminal_clear_armed", guest_node)
        self.assertIn("let activeRequestRetryable = false;", html)
        self.assertIn("const retryReady = activeRequestRetryable", html)
        self.assertIn(
            "return stationaryReady && (!activeRequestIntent || retryReady);",
            html,
        )
        self.assertIn("activeRequestOwner === 'guest'", html)
        self.assertIn("updateUI();\n      ws.send(JSON.stringify({ action: 'navigate'", html)

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
