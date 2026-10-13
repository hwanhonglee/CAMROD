"""HH_261006 - ROS-independent sensor-distance and exclusion regression tests."""

import math
import unittest

from record_simulated_mileage import (
    MileageAccumulator, classification, compact_metadata, platform_metadata,
)


class MileageTests(unittest.TestCase):
    def test_zero_baseline_and_distinct_measurements(self):
        counter = MileageAccumulator(target_m=1)
        self.assertEqual(counter.snapshot()["xy_distance_m"], 0)
        counter.observe(100, 50, 50, 0.5, 0)
        row = counter.observe(101, 51, 50, 0.5, 0)
        self.assertEqual(row["xy_added_m"], 1)
        self.assertEqual(row["velocity_added_m"], 0.5)
        self.assertTrue(counter.snapshot()["target_reached"])

    def test_invalid_intervals_never_bridge(self):
        cases = [(math.nan, 1, "nonfinite"), (0, 1, "nonpositive_dt"),
                 (-1, 1, "nonpositive_dt"), (3, 1, "sample_gap"),
                 (1, 3, "position_jump")]
        for stamp, x, reason in cases:
            with self.subTest(reason=reason, stamp=stamp):
                counter = MileageAccumulator()
                counter.observe(0, 0, 0, 1, 0)
                self.assertEqual(counter.observe(stamp, x, 0, 1, 0)["reason"], reason)
                self.assertEqual(counter.observe(10, 10, 0, 1, 0)["status"], "anchor")
                self.assertEqual(counter.xy_distance_m, 0)
                counter.observe(11, 11, 0, 1, 0)
                self.assertEqual(counter.xy_distance_m, 1)

    def test_stationary_jitter_and_microjitter_do_not_add_xy(self):
        counter = MileageAccumulator()
        counter.observe(0, 0, 0, 0.01, 0)
        counter.observe(1, 0.01, 0, 0.01, 0)
        self.assertEqual(counter.xy_distance_m, 0)
        self.assertEqual(counter.velocity_integral_m, 0)
        counter.observe(2, 0.01001, 0, 1, 0)
        self.assertEqual(counter.xy_distance_m, 0)

    def test_sim_clock_not_wall_clock(self):
        counter = MileageAccumulator()
        counter.observe(0, 0, 0, 1, 0, received_monotonic_s=0)
        counter.observe(1, 1, 0, 1, 0, received_monotonic_s=100)
        self.assertEqual(counter.velocity_integral_m, 1)

    def test_metadata_staleness_and_transition_unknown(self):
        # HH_261006 - Manual-to-unknown boundaries retain no invented mode split.
        metadata = {"received_monotonic_s": 10, "mission_id": "a", "intent": "delivery",
                    "phase": "OUTBOUND", "leg": "delivery",
                    "manual_resume_reason": "manual_drive_active"}
        self.assertEqual(classification(metadata, 13)["mode"], "unknown")
        counter = MileageAccumulator()
        counter.observe(0, 0, 0, 1, 0, metadata, 10)
        counter.observe(1, 1, 0, 1, 0, metadata, 11)
        counter.observe(2, 2, 0, 1, 0,
                        {**metadata, "manual_resume_reason": "ready"}, 12)
        self.assertEqual(counter.by_mode, {"manual": 1, "unknown": 1})
        self.assertEqual(counter.by_leg, {"delivery": 2})

    def test_gap_coverage_explicit(self):
        counter = MileageAccumulator()
        counter.observe(0, 0, 0, 1, 0)
        counter.observe(1, 1, 0, 1, 0)
        counter.observe(4, 2, 0, 1, 0)
        snapshot = counter.snapshot()
        self.assertEqual(snapshot["covered_sim_s"], 1)
        self.assertEqual(snapshot["excluded_known_sim_s"], 3)
        self.assertEqual(snapshot["known_time_coverage_fraction"], 0.25)

    def test_metadata_whitelist_and_no_inferred_return(self):
        result = compact_metadata({"geometry": [1] * 100},
                                  {"mission_dispatch_intent": "delivery", "control_mode": True}, 1)
        self.assertNotIn("geometry", result)
        self.assertEqual(classification(result, 1)["mode"], "unknown")
        self.assertIsNone(result["leg"])

    def test_platform_mode_and_ui_auto_are_not_authority(self):
        # HH_261006 - Even fresh CAN/RC observations cannot certify command ownership.
        for control_mode in (0, 1, True, "1", None):
            with self.subTest(control_mode=control_mode):
                metadata = compact_metadata({}, {"control_mode": control_mode,
                                                  "operation_mode": "AUTO"}, 10)
                metadata.update(platform_metadata(control_mode, 100, 10, 100))
                result = classification(metadata, 11)
                self.assertEqual(result["mode"], "unknown")
                self.assertEqual(result["platform_control_mode"],
                                 control_mode if type(control_mode) is int else None)
                self.assertEqual(result["platform_mode_source"], "/platform/status.control_mode")
                self.assertTrue(result["platform_mode_metadata_fresh"])
        self.assertEqual(classification({"received_monotonic_s": 10, "mode": "auto"}, 11)["mode"], "unknown")
        self.assertEqual(classification({"mode": "manual", "mode_received_monotonic_s": 10}, 11)["mode"], "unknown")

    def test_manual_authority_survives_platform_metadata_merge(self):
        # HH_261006 - Fresh manual evidence takes precedence over raw CAN metadata.
        metadata = compact_metadata({}, {"manual_resume": {
            "pending": True, "reason": "manual_drive_active"}}, 10)
        metadata.update(platform_metadata(1, 100, 10, 100))
        result = classification(metadata, 11)
        self.assertEqual(result["mode"], "manual")
        self.assertEqual(result["manual_resume_reason"], "manual_drive_active")
        self.assertIs(result["manual_resume_pending"], True)
        self.assertEqual(result["mode_basis"], "ui.state.manual_resume.reason=manual_drive_active")
        self.assertEqual(result["platform_control_mode"], 1)
        stale = classification(metadata, 13)
        self.assertEqual(stale["mode"], "unknown")
        self.assertEqual(stale["manual_resume_reason"], "manual_drive_active")
        self.assertFalse(stale["mode_metadata_fresh"])
        metadata.update(platform_metadata(1, 103, 13, 103))
        self.assertEqual(classification(metadata, 13)["mode"], "unknown")
        self.assertTrue(classification(metadata, 13)["platform_mode_metadata_fresh"])

    def test_missing_or_nonmanual_resume_never_implies_auto(self):
        # HH_261006 - Resume policy can return before inspecting the manual lease.
        for reason in (None, "no_suspended_mission", "unsupported_stage",
                       "another_mission_active", "ready", "disabled"):
            with self.subTest(reason=reason):
                metadata = compact_metadata({}, {"mission_dispatch_active": True,
                                                  "operation_mode": "AUTO",
                                                  "manual_resume": {"reason": reason}}, 10)
                self.assertEqual(classification(metadata, 11)["mode"], "unknown")

    def test_manual_metadata_is_bounded_scalar_whitelist(self):
        # HH_261006 - Archive evidence fields only, not UI tokens or geometry.
        metadata = compact_metadata({}, {"manual_resume": {
            "reason": "x" * 1000, "pending": "true", "token": "private",
            "geometry": [1] * 100}}, 10)
        result = classification(metadata, 11)
        self.assertEqual(result["manual_resume_reason"], "x" * 160)
        self.assertIsNone(result["manual_resume_pending"])
        self.assertNotIn("token", result)
        self.assertNotIn("geometry", result)
        self.assertTrue(all(value is None or type(value) in (str, int, float, bool)
                            for value in result.values()))
        for resume in ([], "manual_drive_active", {"reason": {"bad": "value"}}):
            result = classification(compact_metadata({}, {"manual_resume": resume}, 10), 11)
            self.assertEqual(result["mode"], "unknown")
            self.assertEqual(result["manual_resume_reason"], "unknown")

    def test_platform_freshness_never_sets_authority(self):
        # HH_261006 - Receipt freshness cannot repair an old or invalid source stamp.
        for stamp, wall_now in ((97, 100), (101, 100), (math.nan, 100)):
            metadata = platform_metadata(1, stamp, 10, wall_now)
            result = classification(metadata, 11)
            self.assertFalse(result["platform_mode_metadata_fresh"])
            self.assertEqual(result["mode"], "unknown")

    def test_mode_metadata_does_not_change_distance_or_target(self):
        # HH_261006 - Corrected labels leave all raw XY/velocity accounting unchanged.
        unknown, manual = MileageAccumulator(target_m=2), MileageAccumulator(target_m=2)
        for stamp in range(3):
            unknown.observe(stamp, stamp, 0, 0.5, 0, received_monotonic_s=stamp)
            metadata = compact_metadata({}, {"manual_resume": {
                "pending": True, "reason": "manual_drive_active"}}, stamp)
            manual.observe(stamp, stamp, 0, 0.5, 0, metadata, stamp)
        unknown_snapshot, manual_snapshot = unknown.snapshot(), manual.snapshot()
        self.assertEqual(unknown_snapshot.pop("distance_by_mode_m"), {"unknown": 2})
        self.assertEqual(manual_snapshot.pop("distance_by_mode_m"), {"manual": 2})
        self.assertEqual(unknown_snapshot, manual_snapshot)
        self.assertTrue(manual_snapshot["target_reached"])

    def test_phase_and_session_local_identity(self):
        state = {"mission_dispatch_active": True, "mission_dispatch_generation": 5,
                 "mission_dispatch_intent": "recall", "service_state": 3,
                 "mission_phase": "RETURNING"}
        result = compact_metadata({}, state, 1)
        self.assertEqual(result["mission_id"], "ui-generation:5")
        self.assertEqual(result["leg"], "return")
        state["service_state"] = 9
        self.assertIsNone(compact_metadata({}, state, 1)["leg"])


if __name__ == "__main__":
    unittest.main()
