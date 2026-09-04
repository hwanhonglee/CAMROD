"""Unit coverage for persistent CAMROD service-operation metrics."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import math
import sys
from tempfile import TemporaryDirectory
import unittest


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


if __name__ == "__main__":
    unittest.main()
