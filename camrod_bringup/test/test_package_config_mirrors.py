"""Keep standalone-package configs synchronized with full bringup mirrors."""

from pathlib import Path


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
