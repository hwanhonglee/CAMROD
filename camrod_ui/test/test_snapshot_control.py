"""Unit coverage for the administrator snapshot bridge and frontend contract."""

import asyncio
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest

from rclpy.time import Time

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "runtime" / "python"),
)

from camrod_ui.ui_backend_node import UiBackendNode  # noqa: E402


class _DoneFuture:

    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _FakeClient:

    def __init__(self, result, ready=True):
        self.ready = ready
        self.result = result
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return _DoneFuture(self.result)


def _backend(output_directory: str):
    backend = UiBackendNode.__new__(UiBackendNode)
    backend.snapshot_output_directory = Path(output_directory).resolve()
    backend.snapshot_request_timeout_s = 1.0
    backend.snapshot_minimum_free_space_mb = 0
    backend.snapshot_minimum_free_space_ratio = 0.0
    backend.snapshot_size_safety_factor = 1.0
    backend._snapshot_lock = threading.Lock()
    backend._snapshot_write_pending = False
    backend._snapshot_last_result = {}
    backend.snapshot_client = _FakeClient(_trigger_response())
    backend.snapshot_configure_client = _FakeClient(
        SimpleNamespace(
            success=True,
            recording=True,
            writing=False,
            active_topics=[],
            dynamic_topics=[],
            rejected_topics=[],
            message="active",
        )
    )
    backend.snapshot_estimate_client = _FakeClient(
        SimpleNamespace(
            success=True,
            requested_bytes=1000,
            selected_bytes=1000,
            message_count=10,
            actual_start_time=SimpleNamespace(sec=70, nanosec=0),
            newest_time=SimpleNamespace(sec=100, nanosec=0),
            truncated=False,
            message="fits",
        )
    )
    return backend


def _trigger_response(success=True, message="Saved 12 messages"):
    return SimpleNamespace(
        success=success,
        message=message,
        requested_bytes=1000,
        selected_bytes=1000,
        message_count=10,
        actual_start_time=SimpleNamespace(sec=70, nanosec=0),
        truncated=False,
    )


class SnapshotBackendTest(unittest.TestCase):

    def test_topic_input_normalizes_deduplicates_and_rejects_invalid_names(self):
        topics, rejected = UiBackendNode._normalize_snapshot_topics([
            "sensing/radar/right1/range",
            "/sensing/radar/right1/range",
            "/bad topic",
        ])
        self.assertEqual(topics, ["/sensing/radar/right1/range"])
        self.assertEqual(rejected, ["/bad topic"])

    def test_snapshot_selection_accepts_complete_base_topic_profile(self):
        requested = [f"/topic_{index}" for index in range(74)]
        topics, rejected = UiBackendNode._normalize_snapshot_topics(
            requested, max_topics=1024
        )
        self.assertEqual(topics, requested)
        self.assertEqual(rejected, [])

    def test_snapshot_status_includes_all_topics_visible_in_ros_graph(self):
        with tempfile.TemporaryDirectory() as directory:
            backend = _backend(directory)
            backend.snapshot_client = _FakeClient(
                SimpleNamespace(success=True, message="unused")
            )
            # Match the actual rclpy Node.get_topic_names_and_types() shape.
            backend.get_topic_names_and_types = lambda: [
                ("/not_buffered", ["sensor_msgs/msg/Image"]),
                ("/buffered", ["std_msgs/msg/String"]),
            ]
            result = asyncio.run(backend.get_snapshot_status())

        self.assertEqual(
            result["available_topics"],
            [
                {
                    "name": "/buffered",
                    "type": "std_msgs/msg/String",
                    "selectable": True,
                },
                {
                    "name": "/not_buffered",
                    "type": "sensor_msgs/msg/Image",
                    "selectable": True,
                },
            ],
        )

    def test_snapshot_storage_budget_preserves_larger_of_floor_and_ratio(self):
        with tempfile.TemporaryDirectory() as directory:
            backend = _backend(directory)
            backend.snapshot_minimum_free_space_mb = 5_000
            backend.snapshot_minimum_free_space_ratio = 0.10
            backend.snapshot_size_safety_factor = 1.25
            backend._snapshot_disk_usage = lambda _path=None: SimpleNamespace(
                total=100_000_000_000,
                free=20_000_000_000,
            )
            budget = backend._snapshot_storage_budget()

        self.assertEqual(budget["reserve_bytes"], 10_000_000_000)
        self.assertEqual(budget["writable_bytes"], 10_000_000_000)
        self.assertEqual(budget["serialized_budget_bytes"], 8_000_000_000)

    def test_snapshot_estimate_passes_disk_budget_and_reports_latest_window(self):
        with tempfile.TemporaryDirectory() as directory:
            backend = _backend(directory)
            backend.snapshot_configure_client.result.active_topics = [
                SimpleNamespace(name="/buffered", type="std_msgs/msg/String")
            ]
            backend.snapshot_estimate_client.result = SimpleNamespace(
                success=True,
                requested_bytes=2_000,
                selected_bytes=1_000,
                message_count=4,
                actual_start_time=SimpleNamespace(sec=90, nanosec=0),
                newest_time=SimpleNamespace(sec=100, nanosec=0),
                truncated=True,
                message="adjusted",
            )
            backend.get_clock = lambda: SimpleNamespace(
                now=lambda: Time(seconds=100.0)
            )
            result = asyncio.run(backend.estimate_snapshot(
                selected_topics=["/buffered"],
                lookback_seconds=30,
                auto_fit=True,
            ))

        self.assertTrue(result["success"])
        self.assertTrue(result["truncated"])
        self.assertEqual(result["actual_lookback_seconds"], 10.0)
        request = backend.snapshot_estimate_client.requests[0]
        self.assertGreater(request.max_bytes, 0)
        self.assertEqual(request.start_time.sec, 70)

    def test_snapshot_uses_server_owned_directory_and_sanitized_label(self):
        response = _trigger_response()
        with tempfile.TemporaryDirectory() as directory:
            backend = _backend(directory)
            backend.snapshot_client = _FakeClient(response)
            result = asyncio.run(
                backend.trigger_snapshot(label="right radar/../check", selected_topics=[])
            )

            self.assertTrue(result["success"])
            self.assertTrue(result["path"].startswith(str(Path(directory).resolve())))
            self.assertIn("right_radar_check", Path(result["path"]).name)
            self.assertNotIn("..", Path(result["path"]).name)
            self.assertEqual(backend.snapshot_client.requests[0].topics, [])
            self.assertFalse(backend._snapshot_write_pending)

    def test_snapshot_uses_operator_selected_output_directory(self):
        response = _trigger_response()
        with tempfile.TemporaryDirectory() as directory:
            backend = _backend(directory)
            backend.snapshot_client = _FakeClient(response)
            selected_directory = Path(directory) / "field test" / "bags"
            result = asyncio.run(
                backend.trigger_snapshot(
                    label="radar",
                    selected_topics=[],
                    output_directory=str(selected_directory),
                )
            )

            self.assertTrue(result["success"])
            self.assertEqual(Path(result["path"]).parent, selected_directory)
            self.assertTrue(selected_directory.is_dir())
            self.assertEqual(
                Path(backend.snapshot_client.requests[0].filename).parent,
                selected_directory,
            )

    def test_snapshot_rejects_relative_output_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            backend = _backend(directory)
            backend.snapshot_client = _FakeClient(
                _trigger_response(message="unused")
            )
            result = asyncio.run(
                backend.trigger_snapshot(output_directory="relative/bags")
            )

            self.assertFalse(result["success"])
            self.assertIn("absolute path", result["message"])
            self.assertEqual(backend.snapshot_client.requests, [])
            self.assertFalse(backend._snapshot_write_pending)

    def test_snapshot_applies_requested_lookback_to_start_time(self):
        response = _trigger_response()
        with tempfile.TemporaryDirectory() as directory:
            backend = _backend(directory)
            backend.snapshot_client = _FakeClient(response)
            backend.get_clock = lambda: SimpleNamespace(
                now=lambda: Time(seconds=100.0)
            )
            result = asyncio.run(
                backend.trigger_snapshot(selected_topics=[], lookback_seconds=30)
            )

            self.assertTrue(result["success"])
            request = backend.snapshot_client.requests[0]
            self.assertEqual(request.start_time.sec, 70)
            self.assertEqual(request.start_time.nanosec, 0)

    def test_snapshot_writes_all_74_explicitly_selected_topics(self):
        response = _trigger_response(message="Saved messages")
        requested = [f"/topic_{index}" for index in range(74)]
        active_topics = [
            SimpleNamespace(name=name, type="std_msgs/msg/String")
            for name in requested
        ]
        with tempfile.TemporaryDirectory() as directory:
            backend = _backend(directory)
            backend.snapshot_configure_client.result.active_topics = active_topics
            backend.snapshot_client = _FakeClient(response)
            result = asyncio.run(
                backend.trigger_snapshot(selected_topics=requested)
            )

            self.assertTrue(result["success"])
            self.assertEqual(
                [topic.name for topic in backend.snapshot_client.requests[0].topics],
                requested,
            )

    def test_busy_snapshot_is_rejected_without_calling_service(self):
        with tempfile.TemporaryDirectory() as directory:
            backend = _backend(directory)
            backend._snapshot_write_pending = True
            backend.snapshot_client = _FakeClient(
                _trigger_response(message="unused")
            )
            result = asyncio.run(backend.trigger_snapshot())
            self.assertFalse(result["success"])
            self.assertTrue(result["busy"])
            self.assertEqual(backend.snapshot_client.requests, [])

    def test_frontend_mounts_separate_buffer_and_write_controls(self):
        frontend = (
            Path(__file__).resolve().parents[1]
            / "camrod_ui_robot/assets/frontend/src/SnapshotControl.js"
        ).read_text(encoding="utf-8")
        app = (
            Path(__file__).resolve().parents[1]
            / "camrod_ui_robot/assets/frontend/src/App.js"
        ).read_text(encoding="utf-8")
        self.assertIn("<SnapshotControl />", app)
        self.assertIn("{ id: 'snapshot', label: '스냅샷' }", app)
        self.assertIn("activeTab === 'snapshot'", app)
        self.assertIn("...TELEMETRY_TABS,", app)
        self.assertNotIn("      <SnapshotControl />\n\n      <div className=\"steering-tuning-card\">", app)
        self.assertIn("/api/admin/snapshot/topics", frontend)
        self.assertIn("/api/admin/snapshot/status", frontend)
        self.assertIn("/api/admin/snapshot/estimate", frontend)
        self.assertIn("/api/admin/snapshot'", frontend)
        self.assertIn("available_topics", frontend)
        self.assertIn('role="switch"', frontend)
        self.assertIn("버퍼링 토픽 전체 선택", frontend)
        self.assertIn("기존 버퍼링 토픽은 처음부터 선택", frontend)
        self.assertIn("ROSBAG Snapshot", frontend)
        self.assertIn("스냅샷 저장", frontend)
        self.assertIn("저장 폴더", frontend)
        self.assertIn("저장 범위", frontend)
        self.assertIn("lookback_seconds", frontend)
        self.assertIn("파일 이름 태그 (선택)", frontend)
        self.assertIn("output_directory", frontend)
        self.assertIn("topics: selectedTopics", frontend)
        self.assertIn("용량 자동 맞춤", frontend)
        self.assertIn("auto_fit: autoFit", frontend)


if __name__ == "__main__":
    unittest.main()
