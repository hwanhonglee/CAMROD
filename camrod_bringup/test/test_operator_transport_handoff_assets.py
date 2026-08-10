"""Keep operator transport and return-handoff result media reproducible."""

# HH_260810 - Bind the new UI/system representative media to structured facts,
# synchronized runtime defaults, and explicit non-ARM/non-field claim limits.

import json
from pathlib import Path
import subprocess
import sys

from PIL import Image
import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = SRC_ROOT / "docs" / "assets" / "module-guides"
UI_RELATIVE = Path("ui/test-results/operator-telemetry-websocket-amd64-20260810")
SYSTEM_RELATIVE = Path("system/test-results/return-handoff-nav-status-20260810")
RENDERER = (
    SRC_ROOT
    / "camrod_bringup"
    / "scripts"
    / "visualization"
    / "render_operator_transport_handoff_results.py"
)
UI_PNG = "operator-telemetry-websocket-amd64.png"
UI_GIF = "operator-telemetry-websocket-amd64.gif"
SYSTEM_PNG = "return-handoff-nav-status.png"
SYSTEM_GIF = "return-handoff-nav-status.gif"


def test_renderer_recreates_ui_and_system_result_media(tmp_path: Path) -> None:
    """The central renderer must reproduce all four committed assets."""
    subprocess.run(
        [
            sys.executable,
            str(RENDERER),
            "--repo-root",
            str(SRC_ROOT),
            "--output-root",
            str(tmp_path),
        ],
        cwd=SRC_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    for relative in (UI_RELATIVE / UI_PNG, SYSTEM_RELATIVE / SYSTEM_PNG):
        with Image.open(tmp_path / relative) as image:
            assert image.format == "PNG"
            assert image.size == (1600, 900)

    for relative in (UI_RELATIVE / UI_GIF, SYSTEM_RELATIVE / SYSTEM_GIF):
        with Image.open(tmp_path / relative) as animation:
            assert animation.format == "GIF"
            assert animation.size == (1000, 600)
            assert animation.n_frames == 24
            assert animation.info["duration"] == 160


def test_operator_measurement_retains_exact_scope_and_values() -> None:
    """The representative media must remain tied to the measured AMD64 run."""
    record = json.loads((ASSET_ROOT / UI_RELATIVE / "measurement.json").read_text())
    assert record["classification"] == "MEASURED_AMD64_STANDALONE"
    assert record["target_runtime"] == {
        "architecture": "arm64",
        "cpu_cores": 8,
        "ram_gib": 16,
        "accepted_by_this_measurement": False,
    }
    assert record["scope"]["sensor_publishers"] is False
    assert record["scope"]["stream_rate_parameter_hz"] == 10.0
    assert record["websocket"]["frames"] == 201
    assert record["websocket"]["effective_rate_hz"] == 9.938
    assert record["websocket"]["interval_ms"]["p95"] == 100.792
    assert record["process_resources"]["rss_kib"]["delta"] == 896
    assert record["lease_cleanup"]["normal_close_detected_ms"] == 83.3
    assert record["lease_cleanup"]["silent_client_expired_s"] == 12.078


def test_handoff_cases_match_the_deployed_grace() -> None:
    """The policy record and package/bringup YAML must use the same 3 s grace."""
    record = json.loads((ASSET_ROOT / SYSTEM_RELATIVE / "policy-cases.json").read_text())
    assert record["classification"] == "POLICY_REGRESSION"
    assert record["field_claim"] is False
    assert record["transition_grace_s"] == 3.0
    assert [case["expected"] for case in record["cases"]] == [
        "SUPPRESSED_HANDOFF",
        "VISIBLE_ABORT",
        "VISIBLE_ABORT",
    ]

    for config in (
        SRC_ROOT
        / "camrod_system/config/diagnostics/default/planning/planning_nav_status_checker.yaml",
        SRC_ROOT
        / "camrod_bringup/config/system/diagnostics/default/planning/planning_nav_status_checker.yaml",
    ):
        params = yaml.safe_load(config.read_text())["planning_nav_status_checker"][
            "ros__parameters"
        ]
        assert params["service_transition_abort_grace_s"] == record["transition_grace_s"]


def test_committed_media_and_owner_document_links_exist() -> None:
    """Package and full-stack guides must directly expose both result sets."""
    for relative in (
        UI_RELATIVE / UI_PNG,
        UI_RELATIVE / UI_GIF,
        SYSTEM_RELATIVE / SYSTEM_PNG,
        SYSTEM_RELATIVE / SYSTEM_GIF,
    ):
        assert (ASSET_ROOT / relative).is_file(), relative

    documents = {
        SRC_ROOT / "camrod_ui/README.md": (UI_PNG, UI_GIF),
        SRC_ROOT / "camrod_system/README.md": (SYSTEM_PNG, SYSTEM_GIF),
        SRC_ROOT / "camrod_bringup/README.md": (
            UI_PNG,
            UI_GIF,
            SYSTEM_PNG,
            SYSTEM_GIF,
        ),
    }
    for document, names in documents.items():
        text = document.read_text(encoding="utf-8")
        for name in names:
            assert name in text, f"{document.relative_to(SRC_ROOT)} missing {name}"

    ui_record = (ASSET_ROOT / UI_RELATIVE / "README.md").read_text(encoding="utf-8")
    system_record = (ASSET_ROOT / SYSTEM_RELATIVE / "README.md").read_text(
        encoding="utf-8"
    )
    assert "not ARM64 acceptance" in ui_record
    assert "not a physical drive or live ROS" in " ".join(system_record.split())
