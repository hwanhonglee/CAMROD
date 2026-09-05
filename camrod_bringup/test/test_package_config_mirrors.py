"""Keep standalone-package configs synchronized with full bringup mirrors."""

from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]

# HH_260807 - Package configs are the standalone defaults and therefore the
# source of truth.  Bringup mirrors let the deployment remain self-contained.
CONFIG_MIRRORS = {
    "control": "camrod_control",
    "localization": "camrod_localization",
    "map": "camrod_map",
    "perception": "camrod_perception",
    "planning": "camrod_planning",
    "platform": "camrod_platform",
    "sensing": "camrod_sensing",
    "sensor_kit": "camrod_sensor_kit",
    "system": "camrod_system",
}

# HH_260818 - These files intentionally retain a package/deployment A/B split.
# Dedicated controller-profile tests constrain the exact differing keys, while
# this generic mirror test still verifies that both copies exist.
INTENTIONAL_DEPLOYMENT_OVERRIDES = {
    # Existing field gains and parked distance intentionally differ from the
    # package standalone profile. The dedicated AprilTag contract test still
    # requires the new default-off retry safety envelope to match exactly.
    ("control", Path("parking.yaml")),
    ("planning", Path("nav2_base.yaml")),
    ("planning", Path("nav2_vehicle.yaml")),
    ("platform", Path("ranger_driver.yaml")),
}


def _regular_files(root: Path) -> set[Path]:
    """Return relative files without following mirrored directory symlinks."""
    return {
        path.relative_to(root)
        for path in root.rglob("*")
        if path.is_file()
    }


def test_all_package_config_files_have_byte_identical_bringup_mirrors() -> None:
    for label, package_name in CONFIG_MIRRORS.items():
        package_root = SRC_ROOT / package_name / "config"
        bringup_root = SRC_ROOT / "camrod_bringup" / "config" / label

        assert package_root.is_dir(), f"missing package config: {package_root}"
        assert bringup_root.is_dir(), f"missing bringup config: {bringup_root}"

        package_files = _regular_files(package_root)
        bringup_files = _regular_files(bringup_root)

        # Check from both directions.  planning/nav2_combo_profiles is a
        # bringup symlink into camrod_planning, so package enumeration also
        # verifies every file reached through that directory link.
        for relative in sorted(package_files):
            package_file = package_root / relative
            bringup_file = bringup_root / relative
            assert bringup_file.is_file(), (
                f"missing bringup mirror: {label}/{relative}"
            )
            if (label, relative) not in INTENTIONAL_DEPLOYMENT_OVERRIDES:
                assert package_file.read_bytes() == bringup_file.read_bytes(), (
                    f"config drift: {label}/{relative}"
                )

        for relative in sorted(bringup_files):
            assert (package_root / relative).is_file(), (
                f"bringup-only mirrored config: {label}/{relative}"
            )


def test_current_parking_reference_follows_canonical_yaml() -> None:
    """Keep current docs coupled to current config, not a historical release."""
    # HH_260904 - Historical asset tests must not hard-code today's parking
    # values. Derive the live documentation contract from the source-of-truth
    # YAML so a deliberate field tune updates one clearly owned relationship.
    parking_path = SRC_ROOT / "camrod_control/config/parking.yaml"
    parking = yaml.safe_load(parking_path.read_text(encoding="utf-8"))
    april = parking["/parking/apriltag_parking_controller"]["ros__parameters"]
    reverse = parking["/parking/reverse_parking_controller"]["ros__parameters"]
    parameter_reference = (
        SRC_ROOT / "docs/RUNTIME_PARAMETER_REFERENCE.md"
    ).read_text(encoding="utf-8")

    assert "parallel_command_lateral_deadband_mps" in parameter_reference
    assert (
        f"`{float(reverse['slowdown_start_remaining_distance_m']):.2f} m` remaining"
        in parameter_reference
    )
    assert (
        "UI camera range "
        f"`{float(april['slowdown_start_tag_distance_m']):.2f} -> "
        f"{float(april['translation_stop_tag_distance_m']):.2f} m`"
        in parameter_reference
    )
    assert "Charging CAN feedback immediately publishes zero" in parameter_reference
