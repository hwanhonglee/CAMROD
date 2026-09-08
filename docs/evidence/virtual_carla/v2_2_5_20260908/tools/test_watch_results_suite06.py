"""Offline fake states only: never invoke the index, services, or runtime."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import watch_results_suite06 as observer
import update_results_index as index


class FakeLoop:
    def __init__(self, states, returncode=0):
        self.states = iter(states)
        self.clock = 0.0
        self.calls = []
        self.statuses = []
        self.returncode = returncode

    def read(self):
        return next(self.states)

    def update(self, remaining):
        self.calls.append((self.clock, remaining))
        return self.returncode, "offline fake index"

    def sleep(self, seconds):
        self.clock += seconds

    def run(self, limit=300):
        with contextlib.redirect_stdout(io.StringIO()):
            return observer.run_loop(self.read, self.update, self.statuses.append,
                                     lambda: self.clock, self.sleep, limit)


def state(status="RUNNING", fingerprint="same"):
    return {"status": status, "fingerprint": fingerprint, "active_stage": "B1_operator_recall"}


class ObserverTests(unittest.TestCase):
    def test_command_is_fixed_evidence_index_only(self):
        command = observer.index_command()
        self.assertEqual(command[2], str(observer.HERE / "update_results_index.py"))
        self.assertEqual(command[3:], ["--suite", "full_suite_v224_06", "--expected-head", observer.HEAD, "--runtime-head", observer.HEAD])

    def test_explicit_suite07_uses_same_pinned_head(self):
        command = observer.index_command("full_suite_v224_07")
        self.assertEqual(command[3:], ["--suite", "full_suite_v224_07", "--expected-head", observer.HEAD, "--runtime-head", observer.HEAD])

    def test_suite07_status_does_not_overwrite_suite06(self):
        with tempfile.TemporaryDirectory(prefix="camrod-index-observer-test-") as temporary:
            root = Path(temporary)
            before, _ = observer.status_paths("full_suite_v224_06", root)
            after, temporary_path = observer.status_paths("full_suite_v224_07", root)
            self.assertNotEqual(before, after)
            self.assertEqual(after.name, "results_live_suite07_status.json")
            self.assertTrue(temporary_path.name.startswith(".results_live_suite07_status."))

    def test_unknown_suite_and_directory_escape_rejected(self):
        for name in (None, "full_suite_v224_9", "full_suite_v224_009", "full_suite_v224_09/file",
                     "other_suite_09", "full_suite_v224_09\n", "../full_suite_v224_07", "/tmp/full_suite_v224_07", ""):
            with self.subTest(name=name), self.assertRaises(ValueError):
                observer.suite_directory(name)

    def test_explicit_two_digit_suite_does_not_require_source_code_edits(self):
        for name in ("full_suite_v224_09", "full_suite_v224_10", "full_suite_v224_99"):
            with self.subTest(name=name):
                self.assertEqual(observer.validate_suite_name(name), name)
                command = observer.index_command(name, "a" * 40, "b" * 40)
                self.assertEqual(command[4], name)
                self.assertEqual(command[-3:], ["a" * 40, "--runtime-head", "b" * 40])

    def test_missing_actual_suite_files_never_start_observation(self):
        with tempfile.TemporaryDirectory(prefix="camrod-index-observer-test-") as temporary:
            arguments = ["watcher", "--suite", "full_suite_v224_09", "--expected-head", "a" * 40, "--runtime-head", "b" * 40]
            with patch.object(observer.sys, "argv", arguments), patch.object(observer, "suite_directory", return_value=Path(temporary)), patch.object(observer, "run_loop") as loop, contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    observer.main()
            loop.assert_not_called()

    def test_explicit_source_arguments_remain_required(self):
        for tail in ([], ["--expected-head", "a" * 40], ["--runtime-head", "b" * 40]):
            with self.subTest(tail=tail), patch.object(observer.sys, "argv", ["watcher", "--suite", "full_suite_v224_09"] + tail), patch.object(observer, "run_loop") as loop, contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    observer.main()
            loop.assert_not_called()

    def test_suite08_explicit_source_is_not_replaced_with_old_source(self):
        new = "2237edc814fa9c5dd730a5f05bf7c91ab17ccae1"
        command = observer.index_command("full_suite_v224_08", new, new)
        self.assertEqual(command[3:], ["--suite", "full_suite_v224_08", "--expected-head", new, "--runtime-head", new])
        self.assertNotIn(observer.HEAD, command)

    def test_test_driver_and_runtime_shas_are_independent(self):
        command = observer.index_command("full_suite_v224_08", "a" * 40, "b" * 40)
        self.assertEqual(command[-3:], ["a" * 40, "--runtime-head", "b" * 40])

    def test_invalid_source_shas_cannot_be_passed_to_index(self):
        for value in (None, "9f7cd2ba", "G" * 40, "A" * 40, "a" * 39, "a" * 41, "a" * 40 + " "):
            with self.subTest(value=value), self.assertRaises(ValueError):
                observer.index_command("full_suite_v224_08", value, observer.HEAD)
            with self.subTest(runtime=value), self.assertRaises(ValueError):
                observer.index_command("full_suite_v224_08", observer.HEAD, value)

    def test_suite08_status_is_separate(self):
        target, _ = observer.status_paths("full_suite_v224_08")
        self.assertEqual(target.name, "results_live_suite08_status.json")

    def test_long_failure_excerpt_is_bounded_without_mutating_source(self):
        reason = "No Robot UI confirmation: " + "fixture payload " * 1000
        excerpt = index.failure_excerpt(reason)
        self.assertLess(len(excerpt), 1000)
        self.assertTrue(excerpt.startswith("No Robot UI confirmation:"))
        self.assertIn("native", excerpt)
        self.assertGreater(len(reason), 1000)

    def test_short_failure_excerpt_is_exact(self):
        reason = "precompensate_entry steady timeout after 15.0s"
        self.assertEqual(index.failure_excerpt(reason), reason)

    def test_suite_directory_symlink_outside_root_rejected(self):
        with tempfile.TemporaryDirectory(prefix="camrod-index-observer-test-") as temporary:
            root = Path(temporary) / "validation"
            outside = Path(temporary) / "outside"
            root.mkdir()
            outside.mkdir()
            (root / "full_suite_v224_07").symlink_to(outside, target_is_directory=True)
            with self.assertRaises(ValueError):
                observer.suite_directory("full_suite_v224_07", root)

    def test_explicit_suite07_reads_only_its_status(self):
        with tempfile.TemporaryDirectory(prefix="camrod-index-observer-test-") as temporary:
            root = Path(temporary)
            for suffix, status in (("06", "FAIL"), ("07", "RUNNING")):
                suite = root / f"full_suite_v224_{suffix}"
                suite.mkdir()
                (suite / "suite_status.json").write_text(json.dumps({"status": status}))
            self.assertEqual(observer.snapshot(root, "full_suite_v224_07")["status"], "RUNNING")
            self.assertEqual(observer.snapshot(root, "full_suite_v224_06")["status"], "FAIL")

    def test_continuation_retains_unexecuted_delivery_without_acceptance(self):
        skip = {"stage": "B1_robot_delivery", "status": "NOT_EXECUTED_IN_THIS_SUITE", "prior_evidence": None}
        plan = {"continuation_of": str(index.HERE / "full_suite_v224_06"), "skipped_prior_stages": [skip]}
        previous = {"stages": [], "skipped_prior_stages": [skip]}
        with patch.object(index, "read_json", side_effect=[(previous, ""), ({}, "")]), patch.object(index, "inspect_stage") as stage_inspector:
            rows, errors = index.inspect_prior_stages(plan, {"skipped_prior_stages": [skip]}, observer.HEAD)
        self.assertEqual(errors, [])
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["execution_scope"], "NOT_EXECUTED_IN_THIS_SUITE")
        self.assertFalse(rows[0]["accepted_current_source"])
        self.assertFalse(rows[0]["accepted_prior_source"])
        stage_inspector.assert_not_called()

    def test_unbound_continuation_skip_warns_but_never_becomes_pass(self):
        skip = {"stage": "B1_robot_delivery", "status": "NOT_EXECUTED_IN_THIS_SUITE", "prior_evidence": None}
        plan = {"continuation_of": str(index.HERE / "full_suite_v224_06"), "skipped_prior_stages": [skip]}
        with patch.object(index, "read_json", side_effect=[({"stages": []}, ""), ({}, "")]):
            rows, errors = index.inspect_prior_stages(plan, {}, observer.HEAD)
        self.assertEqual(len(errors), 1)
        self.assertFalse(rows[0]["accepted_current_source"])
        self.assertFalse(rows[0]["accepted_prior_source"])

    def test_unchanged_polls_update_at_60s_then_final(self):
        fake = FakeLoop([state(), state(), state(), state("PASS", "done")])
        self.assertEqual(fake.run(), 0)
        self.assertEqual([t for t, _ in fake.calls], [0, 60, 90])
        self.assertEqual(fake.statuses[-1]["observer_status"], "FINAL_UPDATED")

    def test_meaningful_manifest_change_updates_at_next_30s_poll(self):
        fake = FakeLoop([state(), state(fingerprint="manifest changed"), state("FAIL", "failed")])
        self.assertEqual(fake.run(), 0)
        self.assertEqual([t for t, _ in fake.calls], [0, 30, 60])

    def test_already_terminal_updates_exactly_once(self):
        fake = FakeLoop([state("FAIL")])
        self.assertEqual(fake.run(), 0)
        self.assertEqual(len(fake.calls), 1)

    def test_terminal_update_failure_exits_nonzero(self):
        fake = FakeLoop([state("FAIL")], returncode=1)
        self.assertEqual(fake.run(), 1)
        self.assertEqual(fake.statuses[-1]["observer_status"], "FINAL_UPDATE_FAILED")

    def test_hard_limit_does_not_keep_running(self):
        fake = FakeLoop([state(), state()])
        self.assertEqual(fake.run(limit=45), 2)
        self.assertEqual(fake.clock, 45)
        self.assertEqual(fake.statuses[-1]["observer_status"], "HARD_LIMIT")

    def test_read_missing_state_is_bounded(self):
        with tempfile.TemporaryDirectory(prefix="camrod-index-observer-test-") as temporary:
            self.assertEqual(observer.snapshot(Path(temporary))["status"], "UNAVAILABLE")

    def test_heartbeat_ignored_but_manifest_status_change_detected(self):
        with tempfile.TemporaryDirectory(prefix="camrod-index-observer-test-") as temporary:
            root = Path(temporary)
            suite = root / observer.SUITE
            stage = suite / "B1_operator_recall"
            stage.mkdir(parents=True)
            status = {"status": "RUNNING", "active_stage": "B1_operator_recall", "updated_utc": "first", "completed": []}
            status_path = suite / "suite_status.json"
            status_path.write_text(json.dumps(status))
            manifest = stage / "run_manifest.json"
            manifest.write_text('{"status":"RUNNING"}')
            first = observer.snapshot(root)
            status["updated_utc"] = "second"
            status_path.write_text(json.dumps(status))
            self.assertEqual(observer.snapshot(root)["fingerprint"], first["fingerprint"])
            manifest.write_text('{"status":"FAIL"}')
            self.assertNotEqual(observer.snapshot(root)["fingerprint"], first["fingerprint"])

    def test_unsafe_stage_and_external_symlink_are_not_read(self):
        with tempfile.TemporaryDirectory(prefix="camrod-index-observer-test-") as temporary:
            root = Path(temporary)
            suite = root / observer.SUITE
            suite.mkdir()
            (suite / "suite_status.json").write_text('{"status":"RUNNING","active_stage":"../../outside"}')
            self.assertEqual(observer.snapshot(root)["manifest_count"], 0)
            external = root / "external.json"
            external.write_text('{"secret":"not observable"}')
            link = suite / "linked.json"
            link.symlink_to(external)
            self.assertEqual(observer.read_object(link, suite), {"read_error": "ValueError"})


if __name__ == "__main__":
    unittest.main()
