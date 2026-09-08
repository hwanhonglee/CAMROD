#!/usr/bin/env python3
"""Read-only continuation regression against accepted suite02 B1 evidence."""
import importlib.util
import contextlib
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("durable_suite", HERE / "run_durable_suite.py")
suite = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(suite)
os.environ.setdefault("RANGER_EVIDENCE_ROOT", str(HERE.parent))
PRIOR = HERE / "full_suite_v224_02"


class ContinuationTests(unittest.TestCase):
    def test_resume_rotated_plan_references_only_actual_guest_pass(self):
        prior = HERE / "full_suite_v224_12"
        order = suite.continuation_stage_order(suite.stages(), prior)
        selected, skipped = suite.select_stages(order, "B1_optional_docking", prior)
        self.assertEqual(len(selected), 39)
        self.assertEqual(selected[0]["kind"], "dock")
        self.assertEqual([stage["name"] for stage in selected[-2:]],
                         ["B1_robot_delivery", "B1_operator_recall"])
        self.assertEqual(len(skipped), 1)
        self.assertEqual(skipped[0]["stage"], "B1_guest_recall_robot_handoff")
        self.assertEqual(skipped[0]["prior_evidence"]["prior_test_driver_checkout_head"],
                         "067568ecfe411a5cc31844fa84696220da879249")

    def test_prior_order_rejects_missing_duplicate_or_changed_cases(self):
        original = suite.stages()
        variants = [original[:-1], original[:-1]+[original[0]],
                    [{**original[0], "authority": "guest"}]+original[1:]]
        with tempfile.TemporaryDirectory(prefix="camrod-order-regression-") as directory:
            prior = Path(directory)
            (prior / "suite_status.json").write_text(json.dumps({"status": "FAIL"}))
            for order in variants:
                (prior / "suite_plan.json").write_text(json.dumps({"stages": order}))
                with self.subTest(order=order[0]), self.assertRaisesRegex(ValueError, "exact canonical"):
                    suite.continuation_stage_order(original, prior)

    def test_prior_order_rejects_live_suite(self):
        with tempfile.TemporaryDirectory(prefix="camrod-order-regression-") as directory:
            prior = Path(directory)
            (prior / "suite_status.json").write_text(json.dumps({"status": "RUNNING"}))
            (prior / "suite_plan.json").write_text(json.dumps({"stages": suite.stages()}))
            with self.assertRaisesRegex(ValueError, "still-running"):
                suite.continuation_stage_order(suite.stages(), prior)

    def test_prior_order_requires_explicit_absolute_prior(self):
        for prior in (None, Path("relative-suite")):
            with self.assertRaisesRegex(ValueError, "absolute non-symlink"):
                suite.continuation_stage_order(suite.stages(), prior)

    def idle_server(self, **changes):
        return dict(service_state=13, ready=True, engaged=False, mission_dispatch_active=False, **changes)

    def guest_snapshot(self, **changes):
        state = dict(url="http://127.0.0.1:8012/", title="국립공원 로봇 서비스", ready_state="complete",
                     time_origin=100, ws_ready=1, identity_revision=10, service_state=13,
                     active_intent=None, site_card_visible=True, visible_site_count=13)
        return {**state, **changes}

    def test_guest_reload_once_records_stale_before_and_fresh_picker_after(self):
        page = dict(id="production", url="http://127.0.0.1:8012/", webSocketDebuggerUrl="unused")
        before = self.guest_snapshot(identity_revision=79, site_card_visible=False, visible_site_count=0)
        after = self.guest_snapshot(time_origin=200)
        with tempfile.TemporaryDirectory(prefix="camrod-guest-preparation-test-") as directory:
            output = Path(directory) / "preparation.json"
            with patch.object(suite, "guest_preparation_server_state", return_value=self.idle_server()), \
                    patch.object(suite, "guest_page_snapshot", side_effect=[before, after]), \
                    patch.object(suite, "guest_page_call") as calls:
                result = suite.prepare_guest_page(page, reload_existing=True, log=io.StringIO(), evidence_path=output)
            calls.assert_called_once_with(page, "Page.reload", dict(ignoreCache=False))
            self.assertEqual(result["status"], "READY_AFTER_RELOAD")
            self.assertEqual(result["reload_count"], 1)
            self.assertEqual(result["before"]["identity_revision"], 79)
            self.assertEqual(result["after"]["identity_revision"], 10)
            self.assertEqual(json.loads(output.read_text()), result)
            self.assertIn("not_observed", result["manual_drive_armed"])

    def test_guest_preparation_rejects_active_stop_notready_or_unknown_before_reload(self):
        unsafe = [{"service_state": 14}, {"service_state": 8}, {"service_state": True},
                  {"mission_dispatch_active": True}, {"mission_dispatch_active": None},
                  {"engaged": True}, {"engaged": None}, {"ready": False}, {"ready": None}]
        page = dict(id="production", url="http://127.0.0.1:8012/")
        for change in unsafe:
            with self.subTest(change=change), \
                    patch.object(suite, "guest_preparation_server_state", return_value={**self.idle_server(), **change}), \
                    patch.object(suite, "guest_page_call") as calls, patch.object(suite, "guest_page_snapshot") as snapshots:
                with self.assertRaisesRegex(RuntimeError, "inactive, disengaged"):
                    suite.prepare_guest_page(page, reload_existing=True, log=io.StringIO())
                calls.assert_not_called()
                snapshots.assert_not_called()

    def test_guest_reload_does_not_accept_old_document_missing_picker_or_unbootstrapped_ws(self):
        page = dict(id="production", url="http://127.0.0.1:8012/")
        before = self.guest_snapshot()
        invalid = [{"time_origin": 100}, {"ws_ready": 0}, {"identity_revision": -1},
                   {"site_card_visible": False}, {"visible_site_count": 0}, {"service_state": 8},
                   {"active_intent": "recall"}, {"url": "http://other.example/"}]
        for change in invalid:
            with self.subTest(change=change), \
                    patch.object(suite, "guest_preparation_server_state", return_value=self.idle_server()), \
                    patch.object(suite, "guest_page_snapshot", side_effect=[before, self.guest_snapshot(time_origin=200, **{k:v for k,v in change.items() if k != "time_origin"}) if "time_origin" not in change else before]), \
                    patch.object(suite, "guest_page_call") as calls:
                with self.assertRaisesRegex(RuntimeError, "fresh idle identity"):
                    suite.prepare_guest_page(page, reload_existing=True, log=io.StringIO(), timeout_s=0)
                self.assertEqual(calls.call_count, 1, "no retry reload or UI action")

    def test_new_guest_page_is_observed_without_reload(self):
        page = dict(id="production", url="http://127.0.0.1:8012/")
        with patch.object(suite, "guest_preparation_server_state", return_value=self.idle_server()), \
                patch.object(suite, "guest_page_snapshot", return_value=self.guest_snapshot()), \
                patch.object(suite, "guest_page_call") as calls:
            result = suite.prepare_guest_page(page, reload_existing=False, log=io.StringIO())
        self.assertEqual(result["status"], "READY_NEW_PAGE")
        self.assertEqual(result["reload_count"], 0)
        calls.assert_not_called()

    def test_existing_guest_preparation_failure_never_starts_another_browser(self):
        with patch.object(suite, "target", return_value=dict(id="production")), \
                patch.object(suite, "prepare_guest_page", side_effect=OSError("server unavailable")), \
                patch.object(suite.subprocess, "run") as commands:
            with self.assertRaisesRegex(OSError, "server unavailable"):
                suite.ensure_guest("unused", io.StringIO())
            commands.assert_not_called()

    def test_guest_preparation_rechecks_server_and_fails_closed_after_reload(self):
        page = dict(id="production", url="http://127.0.0.1:8012/")
        with patch.object(suite, "guest_preparation_server_state", side_effect=[self.idle_server(),
                {**self.idle_server(), "mission_dispatch_active": True}]), \
                patch.object(suite, "guest_page_snapshot", side_effect=[self.guest_snapshot(), self.guest_snapshot(time_origin=200)]), \
                patch.object(suite, "guest_page_call") as calls:
            with self.assertRaisesRegex(RuntimeError, "fresh idle identity"):
                suite.prepare_guest_page(page, reload_existing=True, log=io.StringIO(), timeout_s=0)
            self.assertEqual(calls.call_count, 1)

    def test_cli_defer_plan_records_all_stages_without_creating_output(self):
        with tempfile.TemporaryDirectory(prefix="camrod-deferred-plan-test-") as directory:
            output = Path(directory) / "never-created"
            stdout = io.StringIO()
            with patch.object(suite.sys, "argv", ["run_durable_suite.py", "plan", "--output-root", str(output),
                    "--start-at", "B1_guest_recall_robot_handoff", "--defer-earlier-stages"]), \
                    patch.object(suite, "driver_identity", return_value={"test_only": True}), \
                    patch.object(suite.subprocess, "run") as commands, contextlib.redirect_stdout(stdout):
                suite.main()
            plan = json.loads(stdout.getvalue())
            self.assertTrue(plan["defer_earlier_stages"])
            self.assertIsNone(plan["continuation_of"])
            self.assertEqual(plan["execution_stage_count"], 40)
            self.assertEqual(plan["total_planned_stages"], 40)
            self.assertEqual(plan["stages"], suite.stages()[2:] + suite.stages()[:2])
            self.assertEqual(plan["skipped_prior_stages"], [])
            self.assertFalse(output.exists())
            self.assertEqual(commands.call_count, 3)
            self.assertTrue(all(call.args[0][1] == "plan" for call in commands.call_args_list))

    def test_deferred_guest_first_plan_executes_all_forty_unique_stages(self):
        original = suite.stages()
        selected, skipped = suite.select_stages(original, "B1_guest_recall_robot_handoff", None,
                                               defer_earlier_stages=True)
        self.assertEqual(len(selected), 40)
        self.assertEqual(sum(stage.get("kind") != "dock" for stage in selected), 39)
        self.assertEqual(selected, original[2:] + original[:2])
        self.assertEqual([stage["name"] for stage in selected[-2:]],
                         ["B1_robot_delivery", "B1_operator_recall"])
        self.assertEqual(len({stage["name"] for stage in selected}), 40)
        self.assertEqual({stage["name"] for stage in selected}, {stage["name"] for stage in original})
        self.assertEqual(skipped, [])
        self.assertEqual(original, suite.stages(), "caller plan is not mutated")

    def test_defer_rejects_continuation_without_importing_prior_pass(self):
        with patch.object(suite, "accepted_matrix_reference") as accept:
            with self.assertRaisesRegex(ValueError, "cannot import prior acceptance"):
                suite.select_stages(suite.stages(), "B1_guest_recall_robot_handoff", PRIOR,
                                    defer_earlier_stages=True)
            accept.assert_not_called()

    def test_defer_requires_later_enabled_start(self):
        with self.assertRaisesRegex(ValueError, "later --start-at"):
            suite.select_stages(suite.stages(), "B1_robot_delivery", None, defer_earlier_stages=True)
        with self.assertRaisesRegex(ValueError, "Unknown/disabled"):
            suite.select_stages(suite.stages(True), "B1_optional_docking", None, defer_earlier_stages=True)

    def test_defer_preserves_explicit_optional_dock_omission(self):
        original = suite.stages(True)
        selected, skipped = suite.select_stages(original, "B1_guest_recall_robot_handoff", None,
                                               defer_earlier_stages=True)
        self.assertEqual(len(selected), 39)
        self.assertEqual(selected, original[2:] + original[:2])
        self.assertEqual(skipped, [])

    def test_actual_suite06_carries_unexecuted_delivery_and_strict_recall(self):
        selected, skipped = suite.select_stages(suite.stages(), "B1_guest_recall_robot_handoff", HERE / "full_suite_v224_06")
        self.assertEqual(len(selected), 38)
        self.assertEqual(skipped[0], dict(stage="B1_robot_delivery", status="NOT_EXECUTED_IN_THIS_SUITE", prior_evidence=None))
        self.assertEqual(skipped[1]["prior_evidence"]["acceptance"], "PRIOR_ACCEPTED_EVIDENCE")
        self.assertEqual(skipped[1]["prior_evidence"]["prior_runtime_identity"]["runtime_source_head"],
                         "9f7cd2bacda17a4581bd44ed66aaba857e89f1d4")

    def test_skipped_absence_is_never_transitive_acceptance_or_failure(self):
        stage = "B1_robot_delivery"
        empty = dict(stage=stage, status="NOT_EXECUTED_IN_THIS_SUITE", prior_evidence=None)
        variants = [
            {"skipped_prior_stages": []},
            {"skipped_prior_stages": [empty, empty]},
            {"skipped_prior_stages": [{**empty, "prior_evidence": {"acceptance": "PRIOR_ACCEPTED_EVIDENCE"}}]},
            {"skipped_prior_stages": [{**empty, "status": "PASS"}]},
            {"skipped_prior_stages": [{k: v for k, v in empty.items() if k != "prior_evidence"}]},
            {"skipped_prior_stages": [empty], "completed": [{"stage": stage, "status": "FAIL"}]},
            {"skipped_prior_stages": [empty], "completed": [{"stage": stage, "status": "PASS"}]},
            {"skipped_prior_stages": [empty], "failed_stage": stage},
        ]
        for previous in variants:
            with self.subTest(previous=previous), patch.object(Path, "read_text", return_value=json.dumps(previous)), \
                    patch.object(suite, "accepted_matrix_reference") as accept:
                with self.assertRaisesRegex(ValueError, "did not itself execute and accept"):
                    suite.select_stages(suite.stages(), "B1_operator_recall", PRIOR)
                accept.assert_not_called()

    def test_actual_accepted_b1_continues_39_stages_without_relabeling(self):
        selected, skipped = suite.select_stages(suite.stages(), "B1_operator_recall", PRIOR)
        self.assertEqual(len(selected), 39)
        self.assertEqual(selected[0]["name"], "B1_operator_recall")
        self.assertEqual(selected[1]["name"], "B1_guest_recall_robot_handoff")
        self.assertEqual(selected[2]["kind"], "dock")
        self.assertEqual(skipped[0]["status"], "NOT_EXECUTED_IN_THIS_SUITE")
        prior = skipped[0]["prior_evidence"]
        self.assertEqual(prior["prior_runtime_identity"]["runtime_source_head"],
                         "8aad1200f860e389d6174964bd2667ecbe04064e")
        self.assertEqual(prior["accepted_metrics"]["total_odom_distance_m"], 173.119457)

    def test_start_only_does_not_claim_earlier_acceptance(self):
        _, skipped = suite.select_stages(suite.stages(), "B1_operator_recall", None)
        self.assertEqual(skipped[0]["status"], "NOT_EXECUTED_IN_THIS_SUITE")
        self.assertIsNone(skipped[0]["prior_evidence"])

    def test_failed_prior_recall_cannot_be_skipped_as_accepted(self):
        with self.assertRaisesRegex(ValueError, "did not itself execute and accept"):
            suite.select_stages(suite.stages(), "B1_guest_recall_robot_handoff", PRIOR)

    def test_unknown_or_disabled_start_rejected(self):
        for name, stages in (("unknown", suite.stages()), ("B1_optional_docking", suite.stages(True))):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "Unknown/disabled"):
                suite.select_stages(stages, name, PRIOR)

    def test_empty_continuation_rejected(self):
        with self.assertRaisesRegex(ValueError, "later --start-at"):
            suite.select_stages(suite.stages(), "B1_robot_delivery", PRIOR)

    def test_tampered_manifest_rejected_without_changing_any_file(self):
        original = suite.artifact

        def changed_manifest(path):
            value = original(path)
            if str(path).endswith("/site_manifest.json"):
                value["sha256"] = "0" * 64
            return value

        with patch.object(suite, "artifact", side_effect=changed_manifest):
            with self.assertRaisesRegex(ValueError, "changed since strict validation"):
                suite.select_stages(suite.stages(), "B1_operator_recall", PRIOR)


class WindowTests(unittest.TestCase):
    title = "국립공원 로봇 서비스"
    row = "0x04c00004 0 3328862 1920 60 1840 2010 htop 국립공원 로봇 서비스\n"

    def test_utf8_exact_guest_title_waits_only_for_missing_window(self):
        with patch.object(suite.subprocess, "check_output", side_effect=["", self.row]) as read, \
                patch.object(suite.time, "sleep") as sleep:
            self.assertEqual(suite.window(self.title, exact=True)["pid"], 3328862)
            sleep.assert_called_once_with(0.1)
            self.assertEqual(read.call_args.kwargs["env"]["LC_ALL"], "C.UTF-8")
            self.assertEqual(read.call_args.kwargs["encoding"], "utf-8")
            self.assertLessEqual(read.call_args.kwargs["timeout"], 5.0)

    def test_duplicates_fail_without_waiting(self):
        with patch.object(suite.subprocess, "check_output", return_value=self.row * 2), \
                patch.object(suite.time, "sleep") as sleep:
            with self.assertRaisesRegex(RuntimeError, "exactly one"):
                suite.window(self.title, exact=True)
            sleep.assert_not_called()

    def test_command_errors_are_not_retried(self):
        with patch.object(suite.subprocess, "check_output", side_effect=OSError("wmctrl failed")) as read, \
                patch.object(suite.time, "sleep") as sleep:
            with self.assertRaises(OSError): suite.window(self.title, exact=True)
            self.assertEqual(read.call_count, 1)
            sleep.assert_not_called()

    def test_missing_or_nonexact_title_times_out_within_five_seconds(self):
        for output in ("", self.row.replace(self.title, self.title + " duplicate suffix")):
            with self.subTest(output=output), \
                    patch.object(suite.subprocess, "check_output", return_value=output) as read, \
                    patch.object(suite.time, "monotonic", side_effect=[0, 0, 5]), \
                    patch.object(suite.time, "sleep") as sleep:
                with self.assertRaisesRegex(RuntimeError, "5 s startup timeout"):
                    suite.window(self.title, exact=True)
                self.assertEqual(read.call_count, 1)
                sleep.assert_not_called()


if __name__ == "__main__":
    unittest.main()
