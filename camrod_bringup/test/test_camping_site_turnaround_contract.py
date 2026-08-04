"""Lock the active campsite arrival-turn policy and its package mirror."""

from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_CONFIG = SRC_ROOT / "camrod_planning" / "config" / "camping_sites.yaml"
BRINGUP_CONFIG = (
    SRC_ROOT / "camrod_bringup" / "config" / "planning" / "camping_sites.yaml"
)


def _sites(path: Path) -> list[dict]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))["camping_sites"]


def test_active_campsite_config_is_byte_synchronized() -> None:
    """Standalone planning and full bringup must select the same maneuver policy."""
    assert PACKAGE_CONFIG.read_bytes() == BRINGUP_CONFIG.read_bytes()


def test_every_active_campsite_turns_before_return_wait() -> None:
    """No active site may defer its only 180-degree turn until after RETURN."""
    modes = {
        site["type"]: str(site.get("service_mode", "turnaround")).strip().lower()
        for site in _sites(PACKAGE_CONFIG)
    }

    assert modes
    assert set(modes.values()) == {"turnaround"}
    assert modes["camping_site_12"] == "turnaround"
    assert modes["camping_site_13"] == "turnaround"
