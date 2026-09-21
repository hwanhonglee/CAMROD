"""Unit coverage for persistent CAMROD service-operation metrics."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import math
import json
import sqlite3
import sys
from tempfile import TemporaryDirectory
import unittest
import pytest


sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "runtime" / "python"),
)

from camrod_ui.service_metrics import ServiceMetricsTracker  # noqa: E402


DROP_ZONE_WAIT = 0
MOVING_TO_SITE = 1
RETURNING_TO_DROP_ZONE = 3
SITE_ENTRY = 5
UNLOAD_WAIT = 6
RETURN_WITH_CARGO = 9
DROP_ZONE_PARKING = 10
WAITING_FOR_RETURN_REQUEST = 11
WAITING_FOR_CHARGING = 12
CHARGING = 13
DEPARTING_DROP_ZONE = 15
OPERATOR_STOPPED = 16


class _Clock:
    """Deterministic wall and monotonic clock injected into the tracker."""

    def __init__(self, wall_s: float = 1_700_000_000.0) -> None:
        self.wall_s = float(wall_s)
        self.monotonic_s = 0.0

    def now(self) -> float:
        return self.wall_s

    def monotonic(self) -> float:
        return self.monotonic_s


def _utc_timestamp(year: int, month: int, day: int, hour: int, minute: int = 0) -> float:
    return datetime(
        year, month, day, hour, minute, tzinfo=timezone.utc
    ).timestamp()


class ServiceMetricsTrackerTest(unittest.TestCase):

    @staticmethod
    def _tracker(
        clock: _Clock | None = None,
        database_path: Path | str | None = None,
        **kwargs,
    ) -> ServiceMetricsTracker:
        clock = clock or _Clock()
        return ServiceMetricsTracker(
            database_path,
            now_fn=clock.now,
            monotonic_fn=clock.monotonic,
            **kwargs,
        )

    def test_full_service_sequence_and_duplicate_terminals_complete_once(self) -> None:
        clock = _Clock()
        tracker = self._tracker(clock)
        self.assertTrue(
            tracker.start_service(
                "B1",
                mission_key="camping_site_1",
                source="test",
            )
        )

        sequence = (
            (DEPARTING_DROP_ZONE, "DEPARTING_DROP_ZONE"),
            (MOVING_TO_SITE, "MOVING_TO_SITE"),
            (SITE_ENTRY, "SITE_ENTRY"),
            (UNLOAD_WAIT, "UNLOAD_WAIT"),
            (WAITING_FOR_RETURN_REQUEST, "WAITING_FOR_RETURN_REQUEST"),
            (RETURN_WITH_CARGO, "RETURN_WITH_CARGO"),
            (RETURNING_TO_DROP_ZONE, "RETURNING_TO_DROP_ZONE"),
            (DROP_ZONE_PARKING, "DROP_ZONE_PARKING"),
        )
        for state, name in sequence:
            clock.wall_s += 1.0
            self.assertFalse(
                tracker.observe_service_state(state, name, now_s=clock.wall_s)
            )

        clock.wall_s += 1.0
        self.assertTrue(
            tracker.observe_service_state(
                WAITING_FOR_CHARGING,
                "WAITING_FOR_CHARGING",
                now_s=clock.wall_s,
            )
        )
        # Parking and platform publishers may subsequently repeat terminal
        # states.  A finished run must not be counted again.
        self.assertFalse(
            tracker.observe_service_state(CHARGING, "CHARGING", now_s=clock.wall_s + 1)
        )
        self.assertFalse(
            tracker.observe_service_state(
                DROP_ZONE_WAIT, "DROP_ZONE_WAIT", now_s=clock.wall_s + 2
            )
        )

        snapshot = tracker.snapshot()
        self.assertIsNone(snapshot["current_service"])
        self.assertEqual(snapshot["lifetime"]["service_attempt_count"], 1)
        self.assertEqual(snapshot["lifetime"]["completed_service_count"], 1)
        self.assertEqual(snapshot["lifetime"]["interrupted_service_count"], 0)
        self.assertEqual(snapshot["last_completed_service"]["site"], "B1")
        self.assertEqual(
            snapshot["last_completed_service"]["state"], WAITING_FOR_CHARGING
        )

    def test_same_site_start_is_coalesced(self) -> None:
        clock = _Clock()
        tracker = self._tracker(clock)

        self.assertTrue(tracker.start_service("B3", source="guest"))
        first = tracker.snapshot()["current_service"]
        clock.wall_s += 10.0
        self.assertFalse(
            tracker.start_service(
                "B3",
                mission_key="camping_site_3",
                source="duplicate",
            )
        )
        second = tracker.snapshot()["current_service"]

        self.assertEqual(second["id"], first["id"])
        self.assertEqual(second["started_at"], first["started_at"])
        self.assertEqual(second["mission_key"], "camping_site_3")
        self.assertEqual(tracker.snapshot()["lifetime"]["service_attempt_count"], 1)

    def test_different_site_supersedes_active_service(self) -> None:
        clock = _Clock()
        tracker = self._tracker(clock)

        self.assertTrue(tracker.start_service("B2", source="operator"))
        first_id = tracker.snapshot()["current_service"]["id"]
        clock.wall_s += 5.0
        self.assertTrue(tracker.start_service("B8", source="operator"))

        snapshot = tracker.snapshot()
        self.assertEqual(snapshot["current_service"]["site"], "B8")
        self.assertNotEqual(snapshot["current_service"]["id"], first_id)
        self.assertEqual(snapshot["lifetime"]["service_attempt_count"], 2)
        self.assertEqual(snapshot["lifetime"]["completed_service_count"], 0)
        self.assertEqual(snapshot["lifetime"]["interrupted_service_count"], 1)
        self.assertEqual(snapshot["recent_services"][0]["site"], "B2")
        self.assertEqual(
            snapshot["recent_services"][0]["status"],
            ServiceMetricsTracker.SUPERSEDED_RESULT,
        )

    def test_operator_stopped_interrupts_active_service(self) -> None:
        clock = _Clock()
        tracker = self._tracker(clock)
        tracker.start_service("B4")

        clock.wall_s += 3.0
        self.assertTrue(
            tracker.observe_service_state(
                OPERATOR_STOPPED,
                "OPERATOR_STOPPED",
                now_s=clock.wall_s,
            )
        )
        self.assertFalse(
            tracker.observe_service_state(
                OPERATOR_STOPPED,
                "OPERATOR_STOPPED",
                now_s=clock.wall_s + 1.0,
            )
        )

        snapshot = tracker.snapshot()
        self.assertIsNone(snapshot["current_service"])
        self.assertIsNone(snapshot["last_completed_service"])
        self.assertEqual(snapshot["lifetime"]["completed_service_count"], 0)
        self.assertEqual(snapshot["lifetime"]["interrupted_service_count"], 1)
        self.assertEqual(
            snapshot["recent_services"][0]["status"],
            ServiceMetricsTracker.INTERRUPTED_RESULT,
        )
        self.assertEqual(snapshot["recent_services"][0]["state"], OPERATOR_STOPPED)

    def test_planar_velocity_uses_trapezoid_and_minimum_speed_filter(self) -> None:
        tracker = self._tracker(
            minimum_speed_mps=0.1,
            maximum_speed_mps=10.0,
            maximum_sample_gap_s=2.0,
        )
        tracker.start_service("B5")

        # hypot(3, 4) = 5 m/s.  Falling linearly to zero over one second
        # contributes 2.5 m, proving both x/y use and trapezoidal integration.
        self.assertEqual(tracker.observe_velocity(3.0, 4.0, 0.0), 0.0)
        self.assertAlmostEqual(tracker.observe_velocity(0.0, 0.0, 1.0), 2.5)

        # Sub-threshold samples are normalized to zero and add no stationary
        # noise, including across a valid sample interval.
        self.assertEqual(tracker.observe_velocity(0.06, 0.06, 2.0), 0.0)
        self.assertEqual(tracker.observe_velocity(0.09, 0.0, 3.0), 0.0)
        self.assertAlmostEqual(
            tracker.snapshot()["current_service"]["distance_m"], 2.5
        )

    def test_invalid_duplicate_backwards_gap_and_fast_samples_are_filtered(self) -> None:
        def rejected_distance(second_sample: tuple[object, object, object]) -> float:
            tracker = self._tracker(
                minimum_speed_mps=0.0,
                maximum_speed_mps=3.0,
                maximum_sample_gap_s=2.0,
            )
            tracker.start_service("B6")
            self.assertEqual(tracker.observe_velocity(1.0, 0.0, 10.0), 0.0)
            added = tracker.observe_velocity(*second_sample)
            self.assertEqual(tracker.snapshot()["current_service"]["distance_m"], 0.0)
            return added

        for sample in (
            (float("nan"), 0.0, 11.0),
            (0.0, float("inf"), 11.0),
            ("invalid", 0.0, 11.0),
            (1.0, 0.0, 10.0),  # duplicate stamp
            (1.0, 0.0, 9.0),   # backwards stamp
            (1.0, 0.0, 13.0),  # excessive gap
            (4.0, 0.0, 11.0),  # implausibly fast
        ):
            with self.subTest(sample=sample):
                self.assertEqual(rejected_distance(sample), 0.0)

        # Non-finite timestamp is covered separately so it cannot compare equal
        # to itself in a subTest label on all Python versions.
        self.assertEqual(rejected_distance((1.0, 0.0, math.inf)), 0.0)

    def test_sqlite_close_reopen_recovers_active_run_then_completes(self) -> None:
        clock = _Clock(_utc_timestamp(2026, 8, 19, 1))
        with TemporaryDirectory() as temporary_directory:
            database_path = Path(temporary_directory) / "service_metrics.sqlite3"
            tracker = self._tracker(clock, database_path)
            self.assertTrue(tracker.persistence_enabled)
            tracker.start_service(
                "B7", mission_key="camping_site_7", source="guest"
            )
            tracker.observe_velocity(1.0, 0.0, 0.0)
            tracker.observe_velocity(1.0, 0.0, 1.0)
            active_before = tracker.snapshot()["current_service"]
            tracker.close()

            recovered = self._tracker(clock, database_path)
            active_after = recovered.snapshot()["current_service"]
            self.assertIsNotNone(active_after)
            self.assertEqual(active_after["id"], active_before["id"])
            self.assertEqual(active_after["site"], "B7")
            self.assertAlmostEqual(active_after["distance_m"], 1.0)

            clock.wall_s += 20.0
            self.assertTrue(
                recovered.observe_service_state(
                    CHARGING, "CHARGING", now_s=clock.wall_s
                )
            )
            recovered.close()

            final = self._tracker(clock, database_path)
            snapshot = final.snapshot()
            self.assertIsNone(snapshot["current_service"])
            self.assertEqual(snapshot["lifetime"]["completed_service_count"], 1)
            self.assertAlmostEqual(snapshot["lifetime"]["distance_m"], 1.0)
            self.assertEqual(snapshot["last_completed_service"]["id"], active_before["id"])
            final.close()

    def test_asia_seoul_today_daily_history_and_lifetime_aggregation(self) -> None:
        # 2026-08-18 15:00 UTC is midnight at Asia/Seoul.  These two runs
        # therefore belong to consecutive local dates despite being close in UTC.
        clock = _Clock(_utc_timestamp(2026, 8, 19, 14, 0))  # 2026-08-19 23:00 KST
        tracker = self._tracker(clock, timezone_name="Asia/Seoul")

        tracker.start_service("B1")
        tracker.observe_velocity(2.0, 0.0, 0.0)
        tracker.observe_velocity(2.0, 0.0, 1.0)
        tracker.observe_service_state(CHARGING, "CHARGING", now_s=clock.wall_s + 60)

        clock.wall_s = _utc_timestamp(2026, 8, 19, 15, 30)  # 2026-08-20 00:30 KST
        tracker.start_service("B2")
        tracker.observe_velocity(1.0, 0.0, 10.0)
        tracker.observe_velocity(1.0, 0.0, 11.0)
        tracker.observe_service_state(CHARGING, "CHARGING", now_s=clock.wall_s + 60)

        clock.wall_s = _utc_timestamp(2026, 8, 19, 16, 0)
        snapshot = tracker.snapshot()
        self.assertEqual(snapshot["timezone"], "Asia/Seoul")
        self.assertEqual(snapshot["today"]["date"], "2026-08-20")
        self.assertEqual(snapshot["today"]["completed_service_count"], 1)
        self.assertAlmostEqual(snapshot["today"]["distance_m"], 1.0)
        self.assertEqual(snapshot["lifetime"]["completed_service_count"], 2)
        self.assertAlmostEqual(snapshot["lifetime"]["distance_m"], 3.0)
        self.assertEqual(snapshot["lifetime"]["operating_day_count"], 2)
        self.assertEqual(
            [entry["date"] for entry in snapshot["daily_history"]],
            ["2026-08-20", "2026-08-19"],
        )
        self.assertEqual(
            [entry["completed_service_count"] for entry in snapshot["daily_history"]],
            [1, 1],
        )
        self.assertEqual(
            [entry["distance_m"] for entry in snapshot["daily_history"]],
            [1.0, 2.0],
        )
        self.assertEqual(
            [entry["date"] for entry in tracker.snapshot(days=1)["daily_history"]],
            ["2026-08-20"],
        )

    def test_interrupted_distance_counts_in_totals_but_not_completed_count(self) -> None:
        clock = _Clock(_utc_timestamp(2026, 8, 19, 3))
        tracker = self._tracker(clock, timezone_name="Asia/Seoul")

        tracker.start_service("B9")
        tracker.observe_velocity(1.0, 0.0, 0.0)
        tracker.observe_velocity(1.0, 0.0, 1.0)
        tracker.observe_service_state(CHARGING, "CHARGING", now_s=clock.wall_s + 10)

        clock.wall_s += 20.0
        tracker.start_service("B10")
        tracker.observe_velocity(2.0, 0.0, 10.0)
        tracker.observe_velocity(2.0, 0.0, 11.0)
        tracker.observe_service_state(
            OPERATOR_STOPPED,
            "OPERATOR_STOPPED",
            now_s=clock.wall_s + 10,
        )

        snapshot = tracker.snapshot()
        self.assertAlmostEqual(snapshot["today"]["distance_m"], 3.0)
        self.assertEqual(snapshot["today"]["service_attempt_count"], 2)
        self.assertEqual(snapshot["today"]["completed_service_count"], 1)
        self.assertEqual(snapshot["today"]["interrupted_service_count"], 1)
        self.assertAlmostEqual(snapshot["lifetime"]["distance_m"], 3.0)
        self.assertEqual(snapshot["lifetime"]["completed_service_count"], 1)
        self.assertEqual(snapshot["lifetime"]["interrupted_service_count"], 1)

    def test_site_summaries_cover_b1_to_b13_with_average_latest_and_progress(self) -> None:
        clock = _Clock(_utc_timestamp(2026, 9, 4, 0))
        tracker = self._tracker(
            clock,
            maximum_speed_mps=5.0,
            maximum_sample_gap_s=20.0,
        )

        # Two completed B1 services establish 15 m / 100 s averages and prove
        # that the latest execution remains independently visible.
        tracker.start_service("b01")
        tracker.observe_velocity(1.0, 0.0, 0.0)
        tracker.observe_velocity(1.0, 0.0, 10.0)
        tracker.observe_service_state(CHARGING, "CHARGING", now_s=clock.wall_s + 100)
        clock.wall_s += 200
        tracker.start_service("B1")
        tracker.observe_velocity(2.0, 0.0, 20.0)
        tracker.observe_velocity(2.0, 0.0, 30.0)
        tracker.observe_service_state(CHARGING, "CHARGING", now_s=clock.wall_s + 100)

        # B2 has one 10 m / 100 s baseline and a 5 m / 50 s active run.
        clock.wall_s += 200
        tracker.start_service("B2")
        tracker.observe_velocity(1.0, 0.0, 40.0)
        tracker.observe_velocity(1.0, 0.0, 50.0)
        tracker.observe_service_state(CHARGING, "CHARGING", now_s=clock.wall_s + 100)
        clock.wall_s += 200
        tracker.start_service("B2")
        tracker.observe_velocity(0.5, 0.0, 60.0)
        tracker.observe_velocity(0.5, 0.0, 70.0)
        clock.wall_s += 50

        summaries = tracker.snapshot()["site_summaries"]
        canonical = [summary for summary in summaries if summary["site"].startswith("B")]
        self.assertEqual([summary["site"] for summary in canonical], [
            f"B{index}" for index in range(1, 14)
        ])
        b1 = summaries[0]
        self.assertEqual(b1["completed_service_count"], 2)
        self.assertAlmostEqual(b1["average_distance_m"], 15.0)
        self.assertEqual(b1["average_duration_s"], 100)
        self.assertAlmostEqual(b1["latest_service"]["distance_m"], 20.0)

        b2 = summaries[1]
        self.assertEqual(b2["service_attempt_count"], 2)
        self.assertAlmostEqual(b2["average_distance_m"], 10.0)
        self.assertEqual(b2["average_duration_s"], 100)
        self.assertAlmostEqual(b2["current_service"]["distance_m"], 5.0)
        self.assertEqual(b2["current_service"]["duration_s"], 50)
        self.assertEqual(b2["current_distance_progress_percentage"], 50.0)
        self.assertEqual(b2["current_duration_progress_percentage"], 50.0)
        self.assertIsNone(summaries[12]["average_distance_m"])
        self.assertIsNone(summaries[12]["latest_service"])


@pytest.mark.parametrize("site", [f"B{index}" for index in range(1, 14)])
@pytest.mark.parametrize("intent,state", [("delivery", 1), ("recall", 7)])
@pytest.mark.parametrize("status_first", [True, False])
def test_v2_handoff_keeps_all_three_meters_for_every_site(site, intent, state, status_first):
    """Tracker counterpart to backend's distinct-writer ordering regression."""
    tracker = ServiceMetricsTrackerTest._tracker()
    tracker.start_service(site, intent=intent, request_id=f"{site}-{intent}")
    tracker.observe_service_state(15, "DEPARTING_DROP_ZONE")
    tracker.observe_velocity(1.0, 0.0, 1.0)
    tracker.observe_velocity(1.0, 0.0, 2.0)
    if not status_first:
        tracker.observe_service_state(state, "OUTBOUND")
    assert not tracker.observe_service_state(0, "ROAD_HANDOFF_READY")
    if status_first:
        tracker.observe_service_state(state, "OUTBOUND")
    tracker.observe_velocity(1.0, 0.0, 3.0)
    tracker.observe_velocity(1.0, 0.0, 4.0)
    result = tracker.summary()
    assert tracker.has_active_service
    assert result["current_service"]["distance_m"] == 3.0
    assert result["current_service"]["distance_breakdown_m"][intent] == 3.0
    assert result["current_service"]["distance_breakdown_m"]["unknown"] == 0.0
    assert result["lifetime"]["completed_service_count"] == 0
    tracker.close()


class ServiceMetricsV2Test(unittest.TestCase):
    """Additive schema and accounting tests; all stores are new test fixtures."""

    _tracker = staticmethod(ServiceMetricsTrackerTest._tracker)

    def test_explicit_interruption_does_not_fake_success_or_robot_state(self):
        tracker = self._tracker()
        tracker.start_service("B1", intent="delivery", request_id="request-new")
        tracker.observe_service_state(15, "DEPARTING_DROP_ZONE")
        tracker.observe_velocity(1, 0, 0)
        tracker.observe_velocity(1, 0, 1)
        self.assertFalse(tracker.interrupt_service("stale failure", request_id="request-old"))
        self.assertTrue(tracker.has_active_service)
        self.assertTrue(tracker.interrupt_service("departure ERROR", request_id="request-new"))
        self.assertFalse(tracker.interrupt_service("duplicate"))
        for state in (0, 13):
            self.assertFalse(tracker.observe_service_state(state, "SAFE_IDLE"))
        snapshot = tracker.snapshot()
        self.assertEqual(snapshot["lifetime"]["completed_service_count"], 0)
        self.assertEqual(snapshot["lifetime"]["interrupted_service_count"], 1)
        record = snapshot["recent_services"][0]
        self.assertEqual(record["state"], 15)  # no fictitious OPERATOR_STOPPED
        self.assertEqual(record["interruption_reason"], "departure ERROR")
        self.assertEqual(record["distance_m"], 1)
        self.assertFalse(tracker.has_active_service)

    def test_request_id_deduplicates_active_and_closed_but_new_same_site_starts(self):
        tracker = self._tracker()
        self.assertTrue(tracker.start_service("B1", intent="delivery", request_id="first"))
        first_id = tracker.summary()["current_service"]["id"]
        self.assertFalse(tracker.start_service("B1", intent="recall", request_id="first"))
        self.assertEqual(tracker.summary()["current_service"]["intent"], "delivery")
        self.assertTrue(tracker.start_service("B1", intent="recall", request_id="second"))
        self.assertNotEqual(tracker.summary()["current_service"]["id"], first_id)
        self.assertEqual(tracker.summary()["lifetime"]["interrupted_service_count"], 1)
        tracker.observe_service_state(12, "WAITING_FOR_CHARGING")
        self.assertFalse(tracker.start_service("B1", request_id="second"))
        self.assertFalse(tracker.start_service("B2", request_id="first"))
        self.assertEqual(tracker.summary()["lifetime"]["service_attempt_count"], 2)

    def test_distances_crossing_a_leg_boundary_remain_unknown_not_double_counted(self):
        tracker = self._tracker()
        tracker.start_service("B4", intent="delivery", request_id="delivery-4")
        tracker.observe_service_state(1, "MOVING_TO_SITE")
        for stamp in (0, 1, 2):
            tracker.observe_velocity(1, 0, stamp)
        tracker.observe_service_state(3, "RETURNING_TO_DROP_ZONE")
        tracker.observe_velocity(-1, 0, 3)  # unknown crossing interval, still 1m
        tracker.observe_velocity(-1, 0, 4)  # confirmed return interval, 1m
        snapshot = tracker.snapshot()
        expected = {"delivery": 2.0, "recall": 0.0, "return": 1.0, "unknown": 1.0}
        for item in (snapshot["current_service"], snapshot["today"], snapshot["lifetime"],
                     snapshot["daily_history"][0], snapshot["site_summaries"][3]):
            self.assertEqual(item["distance_m"], 4)
            self.assertEqual(item["distance_breakdown_m"], expected)
        self.assertEqual(sum(segment["distance_m"] for segment in tracker._active["segments"]), 4)
        self.assertEqual(tracker._active["distance_m"], 4)
        tracker.interrupt_service("operator cancellation")
        self.assertEqual(tracker.snapshot()["site_summaries"][3]["distance_breakdown_m"], expected)

    def test_recall_phase_override_preserves_recall_until_actual_return(self):
        tracker = self._tracker()
        tracker.start_service("B5", intent="recall", request_id="recall-5")
        tracker.observe_service_state(7, "RECALL_TO_SITE_ROAD")
        tracker.observe_velocity(1, 0, 0)
        tracker.observe_velocity(1, 0, 1)
        tracker.observe_service_state(9, "RETURN_WITH_CARGO", phase="CRAB_IN", leg_kind="recall")
        tracker.observe_velocity(1, 0, 2)
        record = tracker.summary()["current_service"]
        self.assertEqual(record["phase"], "CRAB_IN")
        self.assertEqual(record["distance_breakdown_m"]["recall"], 2)
        self.assertEqual(record["distance_breakdown_m"]["return"], 0)
        tracker.observe_service_state(9, "RETURN_WITH_CARGO", phase="CRAB_OUT", leg_kind="return")
        tracker.observe_velocity(1, 0, 3)
        tracker.observe_velocity(1, 0, 4)
        result = tracker.summary()["current_service"]["distance_breakdown_m"]
        self.assertEqual(result, {"delivery": 0, "recall": 2, "return": 1, "unknown": 1})

    def test_recall_site_entry_does_not_turn_into_delivery(self):
        tracker = self._tracker()
        tracker.start_service("B1", intent="recall")
        tracker.observe_service_state(7, "RECALL_TO_SITE_ROAD")
        tracker.observe_velocity(1, 0, 0)
        tracker.observe_service_state(5, "SITE_ENTRY")
        tracker.observe_velocity(1, 0, 1)
        record = tracker.summary()["current_service"]
        self.assertEqual(record["distance_breakdown_m"]["recall"], 1)
        self.assertEqual(record["distance_breakdown_m"]["delivery"], 0)

    def test_timing_counts_only_valid_observed_intervals_including_zero_speed(self):
        clock = _Clock()
        tracker = self._tracker(clock)
        tracker.start_service("B6", intent="delivery")
        for stamp in (0, 1, 2):
            tracker.observe_velocity(0, 0, stamp)
        clock.wall_s += 100
        tracker.observe_service_state(11, "WAITING_FOR_RETURN_REQUEST")
        self.assertEqual(tracker.summary()["current_service"]["waiting_s"], 2)
        tracker.observe_velocity(1, 0, 102)  # no 100-second interpolation
        tracker.observe_velocity(1, 0, 103)
        record = tracker.summary()["current_service"]
        self.assertEqual(record["distance_m"], 1)
        self.assertEqual(record["moving_s"], 1)
        self.assertEqual(record["waiting_s"], 2)
        self.assertEqual(record["duration_s"], 100)
        self.assertFalse(record["timing_complete"])

    def test_invalid_sample_breaks_anchor_without_later_fabricated_distance(self):
        for bad in ((float("nan"), 0, 1), ("bad", 0, 1), (4, 0, 1), (1, 0, float("inf"))):
            with self.subTest(bad=bad):
                tracker = self._tracker()
                tracker.start_service("B1", intent="delivery")
                tracker.observe_velocity(1, 0, 0)
                self.assertEqual(tracker.observe_velocity(*bad), 0)
                self.assertEqual(tracker.observe_velocity(1, 0, 2), 0)
                self.assertEqual(tracker.observe_velocity(1, 0, 3), 1)
                self.assertEqual(tracker.summary()["current_service"]["distance_m"], 1)

    def test_v2_reopen_preserves_segments_idempotency_and_never_bridges_downtime(self):
        clock = _Clock()
        with TemporaryDirectory() as directory:
            path = Path(directory) / "new-v2.sqlite3"
            tracker = self._tracker(clock, path)
            tracker.start_service("B7", intent="recall", request_id="persisted-7")
            tracker.observe_service_state(7, "RECALL_TO_SITE_ROAD")
            tracker.observe_velocity(1, 0, 0)
            tracker.observe_velocity(1, 0, 1)
            before = tracker.summary()["current_service"]
            tracker.close()
            recovered = self._tracker(clock, path)
            after = recovered.summary()["current_service"]
            for key in ("id", "request_id", "intent", "phase", "segments", "distance_breakdown_m"):
                self.assertEqual(after[key], before[key])
            self.assertFalse(recovered.start_service("B7", request_id="persisted-7"))
            self.assertEqual(recovered.observe_velocity(1, 0, 2), 0)
            self.assertEqual(recovered.observe_velocity(1, 0, 3), 1)
            recovered.observe_service_state(0, "DROP_ZONE_WAIT")
            recovered.close()
            final = self._tracker(clock, path)
            self.assertFalse(final.start_service("B7", request_id="persisted-7"))
            self.assertEqual(final.summary()["lifetime"]["distance_m"], 2)
            self.assertEqual(final.summary()["lifetime"]["distance_breakdown_m"]["recall"], 2)
            final.close()

    def test_v1_additive_migration_preserves_243_original_rows_and_unknown_distance(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "NEW_synthetic_v1.sqlite3"
            connection = sqlite3.connect(path)
            connection.execute("""CREATE TABLE service_runs (
                id TEXT PRIMARY KEY, service_date TEXT NOT NULL, site TEXT NOT NULL,
                mission_key TEXT NOT NULL, source TEXT NOT NULL, started_at REAL NOT NULL,
                ended_at REAL, result TEXT NOT NULL, distance_m REAL NOT NULL,
                last_state INTEGER, last_state_name TEXT NOT NULL, updated_at REAL NOT NULL)""")
            rows = [(f"old-{index}", "2026-09-04", f"B{index % 13 + 1}", "old-key",
                     "guest:recall" if index % 2 else "robot_ui", 1700000000.0 + index,
                     1700000010.0 + index, "completed", index / 10.0,
                     0, "DROP_ZONE_WAIT", 1700000010.0 + index) for index in range(243)]
            connection.executemany("INSERT INTO service_runs VALUES (?,?,?,?,?,?,?,?,?,?,?,?)", rows)
            connection.commit()
            before = connection.execute("SELECT * FROM service_runs ORDER BY id").fetchall()
            connection.close()
            tracker = self._tracker(database_path=path)
            result = tracker.snapshot()
            expected = round(sum(row[8] for row in rows), 2)
            self.assertEqual(result["schema_version"], 2)
            self.assertEqual(result["lifetime"]["service_attempt_count"], 243)
            self.assertEqual(result["lifetime"]["completed_service_count"], 243)
            self.assertEqual(result["lifetime"]["distance_m"], expected)
            self.assertEqual(result["lifetime"]["distance_breakdown_m"],
                             {"delivery": 0, "recall": 0, "return": 0, "unknown": expected})
            self.assertEqual(result["historical_unclassified"], {
                "record_count": 243, "distance_m": expected,
                "distance_km": round(expected / 1000.0, 3),
                "included_in_lifetime_total": True,
            })  # Includes the zero-distance historical row, without adding distance.
            self.assertIsNone(result["lifetime"]["moving_s"])
            self.assertIsNone(result["lifetime"]["waiting_s"])
            for record in result["recent_services"]:
                self.assertEqual(record["intent"], "unknown")
                self.assertEqual(record["phase"], "LEGACY_UNCLASSIFIED")
            tracker.close()
            connection = sqlite3.connect(path)
            columns = "id,service_date,site,mission_key,source,started_at,ended_at,result,distance_m,last_state,last_state_name,updated_at"
            after = connection.execute(f"SELECT {columns} FROM service_runs ORDER BY id").fetchall()
            self.assertEqual(before, after)
            self.assertEqual(connection.execute("PRAGMA user_version").fetchone()[0], 2)
            connection.close()
            reopened = self._tracker(database_path=path)
            self.assertEqual(reopened.summary()["lifetime"]["distance_m"], expected)
            reopened.close()

    def test_display_rounding_preserves_total_and_unknown_legacy_time(self):
        for total in (2.675, 0.015, 1.005, 3.335):
            raw = {"delivery": total / 3, "recall": total / 3, "return": total / 3, "unknown": 0}
            rounded = ServiceMetricsTracker._rounded_breakdown(raw, total)
            self.assertAlmostEqual(sum(rounded.values()), round(total, 2), places=8)
            self.assertTrue(all(value >= 0 for value in rounded.values()))

    def test_new_unknown_transition_is_not_historical_unclassified(self):
        tracker = self._tracker()
        expected = {"record_count": 0, "distance_m": 0.0, "distance_km": 0.0,
                    "included_in_lifetime_total": True}
        self.assertEqual(tracker.snapshot()["historical_unclassified"], expected)
        tracker.start_service("B1", intent="delivery")
        tracker.observe_velocity(1, 0, 0)
        tracker.observe_velocity(1, 0, 1)
        tracker.observe_service_state(3, "RETURNING_TO_DROP_ZONE")
        tracker.observe_velocity(-1, 0, 2)
        tracker.observe_velocity(-1, 0, 3)
        snapshot = tracker.snapshot()
        self.assertEqual(snapshot["lifetime"]["distance_m"], 3)
        self.assertEqual(snapshot["lifetime"]["distance_breakdown_m"]["unknown"], 1)
        self.assertEqual(snapshot["historical_unclassified"], expected)
        self.assertEqual(tracker.summary()["historical_unclassified"], expected)

    def test_active_historical_record_counts_once_and_excludes_new_observed_legs(self):
        for old_distance in (0.0, 2.0):
            with self.subTest(old_distance=old_distance), TemporaryDirectory() as directory:
                path = Path(directory) / "NEW_historical_fixture.sqlite3"
                tracker = self._tracker(database_path=path)
                tracker.start_service("B3")
                tracker.close()
                connection = sqlite3.connect(path)
                row = connection.execute("SELECT started_at FROM service_runs").fetchone()
                parts = [{"kind": "unknown", "phase": "LEGACY_UNCLASSIFIED",
                          "distance_m": old_distance / 2, "moving_s": None, "waiting_s": None,
                          "started_at": row[0], "ended_at": row[0], "timing_complete": False}
                         for _ in range(2)]
                connection.execute("UPDATE service_runs SET distance_m=?,segments_json=?,phase=?",
                                   (old_distance, json.dumps(parts), "LEGACY_UNCLASSIFIED"))
                connection.commit()
                connection.close()
                resumed = self._tracker(database_path=path)
                resumed.observe_service_state(1, "MOVING_TO_SITE")
                for stamp in (0, 1, 2):
                    resumed.observe_velocity(1, 0, stamp)
                result = resumed.summary()
                self.assertEqual(result["lifetime"]["distance_m"], old_distance + 2)
                self.assertEqual(result["lifetime"]["distance_breakdown_m"]["delivery"], 2)
                expected = {"record_count": 1, "distance_m": old_distance,
                            "distance_km": round(old_distance / 1000.0, 3),
                            "included_in_lifetime_total": True}
                self.assertEqual(result["historical_unclassified"], expected)
                resumed.close()
                reopened = self._tracker(database_path=path)
                self.assertEqual(reopened.snapshot()["historical_unclassified"], expected)
                self.assertEqual(reopened.summary()["lifetime"]["distance_m"], old_distance + 2)
                reopened.close()

    def test_does_not_downgrade_newer_database_schema(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "future.sqlite3"
            connection = sqlite3.connect(path)
            connection.execute("PRAGMA user_version=3")
            connection.commit()
            connection.close()
            tracker = self._tracker(database_path=path)
            self.assertFalse(tracker.persistence_enabled)
            self.assertIn("newer", tracker.persistence_error)
            connection = sqlite3.connect(path)
            self.assertEqual(connection.execute("PRAGMA user_version").fetchone()[0], 3)
            self.assertIsNone(connection.execute("SELECT name FROM sqlite_master WHERE name='service_runs'").fetchone())
            connection.close()

    def test_malformed_segment_metadata_falls_back_without_losing_total(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "fixture.sqlite3"
            tracker = self._tracker(database_path=path)
            tracker.start_service("B2", intent="delivery")
            tracker.observe_velocity(1, 0, 0)
            tracker.observe_velocity(1, 0, 1)
            tracker.observe_service_state(0, "DROP_ZONE_WAIT")
            tracker.close()
            connection = sqlite3.connect(path)
            connection.execute("UPDATE service_runs SET segments_json=?", (
                json.dumps([{"kind": "delivery", "phase": "missing-timestamps", "distance_m": 1}]),))
            connection.commit()
            connection.close()
            recovered = self._tracker(database_path=path)
            self.assertEqual(recovered.summary()["lifetime"]["distance_m"], 1)
            self.assertEqual(recovered.summary()["lifetime"]["distance_breakdown_m"]["unknown"], 1)
            self.assertIsNone(recovered.summary()["lifetime"]["moving_s"])
            recovered.close()

    def test_standalone_return_and_real_completion_states_remain_supported(self):
        for state in (0, 12, 13):
            with self.subTest(state=state):
                tracker = self._tracker()
                tracker.start_service("DROP_ZONE", intent="return", request_id=f"standalone-{state}")
                tracker.observe_service_state(3, "RETURNING_TO_DROP_ZONE")
                tracker.observe_velocity(-1, 0, 0)
                tracker.observe_velocity(-1, 0, 1)
                self.assertTrue(tracker.observe_service_state(state, "STATION_TERMINAL"))
                self.assertFalse(tracker.observe_service_state(state, "STATION_TERMINAL"))
                result = tracker.summary()
                self.assertEqual(result["lifetime"]["completed_service_count"], 1)
                self.assertEqual(result["lifetime"]["distance_breakdown_m"]["return"], 1)

    def test_additive_store_failure_remains_observational(self):
        with TemporaryDirectory() as directory:
            tracker = self._tracker(database_path=directory)  # directory is not a SQLite file
            self.assertFalse(tracker.persistence_enabled)
            self.assertTrue(tracker.persistence_error)
            tracker.start_service("B1", intent="delivery")
            tracker.observe_velocity(1, 0, 0)
            tracker.observe_velocity(1, 0, 1)
            tracker.observe_service_state(13, "CHARGING")
            self.assertEqual(tracker.summary()["lifetime"]["distance_m"], 1)


if __name__ == "__main__":
    unittest.main()
