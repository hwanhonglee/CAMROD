"""HH_261007 - Pure direction evidence tests; no ROS or vehicle is started."""
import math
import json
from pathlib import Path
import signal
import tempfile
from types import SimpleNamespace
import unittest
from record_sim_forward_direction import ContextCache, DirectionAccumulator, main
from unittest.mock import Mock, patch


class DirectionTests(unittest.TestCase):
    def add(self, accumulator, stamp, xyz, q=(0., 0., 0., 1.), frame="odom"):
        return accumulator.observe(stamp, xyz, q, frame, "base_link")

    def test_forward_reverse_and_lateral_are_measured_not_commanded(self):
        a = DirectionAccumulator()
        self.add(a, 0, (0, 0, 0))
        self.assertAlmostEqual(self.add(a, 100000000, (.1, 0, 0))["signed_forward_displacement_m"], .1)
        self.assertAlmostEqual(self.add(a, 200000000, (0, 0, 0))["signed_forward_displacement_m"], -.1)
        self.assertAlmostEqual(self.add(a, 300000000, (0, .1, 0))["signed_forward_displacement_m"], 0)
        self.assertAlmostEqual(a.forward_m, .1)
        self.assertAlmostEqual(a.reverse_m, .1)
        self.assertEqual(a.summary()["road_parking_classification"], "not_performed")

    def test_rotated_and_sign_equivalent_quaternion(self):
        a = DirectionAccumulator(); s = math.sqrt(.5)
        self.add(a, 0, (0, 0, 0), (0, 0, s, s))
        row = self.add(a, 100000000, (0, .1, 0), (0, 0, -s, -s))
        self.assertAlmostEqual(row["signed_forward_displacement_m"], .1)

    def test_sloped_forward_projection_uses_xyz(self):
        a = DirectionAccumulator(); s, c = math.sin(math.pi/8), math.cos(math.pi/8)
        q = (0, s, 0, c)
        self.add(a, 0, (0, 0, 0), q)
        row = self.add(a, 100000000, (.05, 0, -.05), q)
        self.assertAlmostEqual(row["signed_forward_displacement_m"], math.sqrt(.005))

    def test_bad_time_gap_jump_frame_and_pose_reject_without_bridging(self):
        cases = [(0, (.1, 0, 0), (0, 0, 0, 1), "odom", "nonpositive_dt"),
                 (-1, (.1, 0, 0), (0, 0, 0, 1), "odom", "invalid_raw_pose_or_stamp"),
                 (250000001, (.1, 0, 0), (0, 0, 0, 1), "odom", "sample_gap"),
                 (100000000, (1, 0, 0), (0, 0, 0, 1), "odom", "implausible_position_jump"),
                 (100000000, (.1, 0, 0), (0, 0, 0, 1), "map", "frame_changed"),
                 (100000000, (.1, 0, 0), (0, 0, 0, 0), "odom", "invalid_quaternion_norm"),
                 (100000000, (math.nan, 0, 0), (0, 0, 0, 1), "odom", "invalid_raw_pose_or_stamp")]
        for stamp, xyz, q, frame, reason in cases:
            with self.subTest(reason=reason):
                a = DirectionAccumulator(); self.add(a, 0, (0, 0, 0))
                self.assertEqual(self.add(a, stamp, xyz, q, frame)["reason"], reason)
                self.assertEqual(self.add(a, 400000000, (.1, 0, 0))["reason"], "anchor")
                self.assertEqual(a.accepted, 0)

    def test_exact_gap_boundary_and_opposite_heading(self):
        a = DirectionAccumulator(); self.add(a, 0, (0, 0, 0))
        self.assertTrue(self.add(a, 250000000, (.1, 0, 0))["valid"])
        self.assertEqual(self.add(a, 350000000, (.2, 0, 0), (0, 0, 1, 0))["reason"],
                         "ambiguous_opposite_headings")

    def test_context_staleness_does_not_reclassify_reverse_parking(self):
        context = ContextCache()
        context.update("parking", {"operating_state": "REVERSE_APPROACH"}, 10, 123)
        self.assertTrue(context.snapshot(10.5)["parking"]["receipt_fresh_within_1s"])
        self.assertFalse(context.snapshot(12)["parking"]["receipt_fresh_within_1s"])
        self.assertEqual(context.snapshot(12)["parking"]["value"]["operating_state"], "REVERSE_APPROACH")

    def test_duration_and_disk_bounds_rejected_before_ros_import(self):
        for extra in (["--duration-s", "901"], ["--duration-s", "nan"],
                      ["--duration-s", "0"], ["--max-bytes", "1"]):
            with patch("sys.stderr"), self.assertRaises(SystemExit):
                main(["--output", "/unused/evidence.jsonl", *extra])

    def test_existing_output_or_summary_is_never_overwritten(self):
        for existing_summary in (False, True):
            with tempfile.TemporaryDirectory() as directory:
                output = Path(directory) / "direction.jsonl"
                protected = output.with_suffix(".jsonl.summary.json") if existing_summary else output
                protected.write_text("original evidence")
                with patch("sys.stderr"), self.assertRaises(SystemExit):
                    main(["--output", str(output)])
                self.assertEqual(protected.read_text(), "original evidence")

    def test_owned_signals_are_clean_but_unexpected_context_shutdown_is_error(self):
        # HH_261007 - Fake ROS verifies lifecycle handling without any graph.
        class ExternalShutdownException(Exception):
            pass
        for event in (signal.SIGINT, signal.SIGTERM, "external_shutdown"):
            with self.subTest(event=event), tempfile.TemporaryDirectory() as directory:
                handlers, disabled = {}, object()
                ros, node = Mock(), Mock()
                ros.create_node.return_value = node
                ros.ok.return_value = True
                def spin(*_args, **_kwargs):
                    if event == "external_shutdown":
                        raise ExternalShutdownException()
                    handlers[event](event, None)
                ros.spin_once.side_effect = spin
                modules = {
                    "rclpy": ros,
                    "rclpy.qos": SimpleNamespace(QoSProfile=Mock(),
                        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1, RELIABLE=2),
                        DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1)),
                    "rclpy.signals": SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=disabled)),
                    "nav_msgs": SimpleNamespace(), "nav_msgs.msg": SimpleNamespace(Odometry=object),
                    "avg_msgs": SimpleNamespace(), "avg_msgs.msg": SimpleNamespace(
                        AvgTwist=object, AvgServiceState=object, ModuleState=object),
                    "std_msgs": SimpleNamespace(), "std_msgs.msg": SimpleNamespace(String=object),
                }
                output = Path(directory) / "evidence.jsonl"
                with patch.dict("sys.modules", modules), \
                        patch("record_sim_forward_direction.signal.signal",
                              side_effect=lambda sig, handler: handlers.__setitem__(sig, handler)), \
                        patch("builtins.print"):
                    result = main(["--output", str(output), "--duration-s", "1"])
                ros.init.assert_called_once_with(args=[], signal_handler_options=disabled)
                footer = json.loads(output.read_text().splitlines()[-1])
                if event == "external_shutdown":
                    self.assertEqual(result, 1)
                    self.assertEqual(footer["status"], "ERROR")
                    self.assertIn("ExternalShutdownException", footer["error"])
                else:
                    self.assertEqual(result, 0)
                    self.assertEqual(footer["status"], "STOPPED")
                    self.assertEqual(footer["stop_reason"], "signal")
                    self.assertIsNone(footer["error"])
                self.assertIsNone(footer["scenario_pass"])
                node.destroy_node.assert_called_once()
                ros.shutdown.assert_called_once()


if __name__ == "__main__":
    unittest.main()
