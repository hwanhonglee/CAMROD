"""Validate committed v2.1.5 service evidence and regenerated visuals."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys

from PIL import Image


SRC_ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "test_result"
    / "v2-1-5-service-validation-20260807"
)
RENDERER = (
    SRC_ROOT
    / "camrod_bringup"
    / "scripts"
    / "render_v2_1_5_service_results.py"
)
MAP_SHA256 = "8cd05c66f846cae8718b5af148d123718f403a086f2e7d16165da89fb625e021"


def test_v2_1_5_service_reports_retain_release_acceptance() -> None:
    """Lock repeated service, safe obstacle hold, and B2 recovery results."""
    service = json.loads((EVIDENCE / "repeated-service-soak.json").read_text())
    soak = next(item for item in service["checks"] if item["name"] == "repeated_service_soak")
    assert service["overall_pass"] is True
    assert soak["success"] is True
    assert soak["metrics"]["cycles_completed"] == 3
    assert soak["metrics"]["bringup_restart_count"] == 0
    assert all(cycle["boundary_recovery_ok"] for cycle in soak["metrics"]["cycles"])
    assert all(not cycle["boundary_retry_latched"] for cycle in soak["metrics"]["cycles"])
    assert soak["metrics"]["cycles"][1]["obstacle_resume_seen"] is True

    obstacle = json.loads((EVIDENCE / "obstacle-safe-hold.json").read_text())
    check = next(item for item in obstacle["checks"] if item["name"] == "obstacle_replan")
    assert obstacle["overall_pass"] is True
    assert check["metrics"]["fallback_failed_hold_seen"] is True
    assert check["metrics"]["fallback_selector_seen"] is False
    assert check["metrics"]["resumed_after_obstacle_clear"] is True

    for trial_path in sorted(EVIDENCE.glob("b2-boundary-recovery-trial-*.json")):
        trial = json.loads(trial_path.read_text())
        assert trial["map"] == {
            "source_file": "lanelet2_maps.osm",
            "map_version": 17,
            "sha256": MAP_SHA256,
        }
        assert trial["mission_completed"] is True
        assert trial["automatic_recovery_motion"] == "REVERSE_YAW_RIGHT"
        assert trial["second_hold"] is None
        assert trial["rapid_recontact_latched"] is False

    assert hashlib.sha256((SRC_ROOT / "lanelet2_maps.osm").read_bytes()).hexdigest() == MAP_SHA256


def test_v2_1_5_service_visuals_are_reproducible(tmp_path: Path) -> None:
    """Regenerate a temporary copy and validate every output format."""
    for name in (
        "repeated-service-soak.json",
        "obstacle-safe-hold.json",
        "b2-boundary-recovery-trial-1.json",
        "b2-boundary-recovery-trial-2.json",
        "b2-boundary-recovery-trial-3.json",
    ):
        (tmp_path / name).write_bytes((EVIDENCE / name).read_bytes())

    subprocess.run(
        [sys.executable, str(RENDERER), "--evidence-dir", str(tmp_path)],
        cwd=SRC_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    for name in (
        "repeated-service-summary.png",
        "obstacle-safe-hold.png",
        "b2-boundary-recovery.png",
    ):
        with Image.open(tmp_path / name) as visual:
            assert visual.width == 1600
            assert visual.height >= 880
            assert visual.format == "PNG"

    with Image.open(tmp_path / "repeated-service-timeline.gif") as visual:
        assert visual.size == (1400, 520)
        assert visual.n_frames == 24
        assert visual.format == "GIF"
