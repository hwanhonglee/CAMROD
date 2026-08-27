#!/usr/bin/env python3
"""Deeply validate the Ranger baseline and physical-4WS runtime gates.

This adapter deliberately imports the validators from the caller-selected
Ranger portable repository source tree.  A manifest's top-level ``status`` is
not an authorization: the Ranger validators re-read and hash every bound
artifact, source file, and nested evidence record before CAMROD may start.
"""

from __future__ import annotations

import argparse
import importlib
from pathlib import Path
import sys
from typing import Any, Sequence


EXACT_RANGER_BLUEPRINT = "vehicle.ranger.default"
PACKAGE_NAME = "carla_extended_ackermann_control"
BASELINE_MODULE_NAME = f"{PACKAGE_NAME}.backend_contract"
PHYSICAL_MODULE_NAME = f"{PACKAGE_NAME}.physical_gate_contract"


class GateValidationError(RuntimeError):
    """Expected, user-facing runtime authorization failure."""


def _regular_file(path: Path, label: str) -> Path:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise GateValidationError(f"{label} is not a regular file: {resolved}")
    return resolved


def _load_validator_modules(validator_source: Path):
    source = validator_source.expanduser().resolve()
    package_dir = source / PACKAGE_NAME
    _regular_file(package_dir / "__init__.py", "Ranger validator package")
    _regular_file(
        package_dir / "backend_contract.py", "Ranger baseline validator source"
    )
    _regular_file(
        package_dir / "physical_gate_contract.py",
        "Ranger physical validator source",
    )

    # Put the selected portable source ahead of ambient ROS/Python overlays.
    # The provenance assertion below makes shadowing fail closed as well.
    sys.path.insert(0, str(source))
    baseline_module = importlib.import_module(BASELINE_MODULE_NAME)
    physical_module = importlib.import_module(PHYSICAL_MODULE_NAME)
    for module, label in (
        (baseline_module, "baseline"),
        (physical_module, "physical"),
    ):
        module_path = Path(module.__file__ or "").resolve()
        if module_path.parent != package_dir:
            raise GateValidationError(
                f"Ranger {label} validator resolved outside the selected "
                f"portable source: {module_path} (expected under {package_dir})"
            )
    return baseline_module, physical_module


def _accepted_result(result: Any, label: str) -> str:
    if getattr(result, "accepted", False) is not True:
        reason = getattr(result, "reason_text", "validator returned no reason")
        raise GateValidationError(f"{label} rejected: {reason}")
    digest = getattr(result, "manifest_sha256", "")
    if (
        not isinstance(digest, str)
        or len(digest) != 64
        or any(character not in "0123456789abcdef" for character in digest)
    ):
        raise GateValidationError(
            f"{label} validator returned an invalid manifest SHA-256"
        )
    return digest


def validate_runtime_gates(
    validator_source: Path,
    baseline_manifest: Path,
    physical_manifest: Path,
) -> tuple[str, str]:
    """Return the two verified digests, or raise and authorize nothing."""

    baseline = _regular_file(baseline_manifest, "baseline gate")
    physical = _regular_file(physical_manifest, "physical 4WS gate")
    baseline_module, physical_module = _load_validator_modules(validator_source)

    baseline_result = baseline_module.validate_baseline_manifest(
        str(baseline), EXACT_RANGER_BLUEPRINT
    )
    baseline_sha256 = _accepted_result(baseline_result, "baseline gate")

    physical_result = physical_module.validate_physical_four_wheel_manifest(
        str(physical), EXACT_RANGER_BLUEPRINT, str(baseline)
    )
    physical_sha256 = _accepted_result(physical_result, "physical 4WS gate")
    return baseline_sha256, physical_sha256


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--validator-source",
        required=True,
        type=Path,
        help=".../carla_extended_ackermann_control/src in the Ranger repo",
    )
    parser.add_argument("--baseline-manifest", required=True, type=Path)
    parser.add_argument("--physical-manifest", required=True, type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        baseline_sha256, physical_sha256 = validate_runtime_gates(
            args.validator_source,
            args.baseline_manifest,
            args.physical_manifest,
        )
    except GateValidationError as error:
        print(f"[virtual_carla] ERROR: {error}", file=sys.stderr)
        return 1
    except Exception as error:  # All runtime authorization failures are closed.
        print(
            "[virtual_carla] ERROR: Ranger gate validator failed closed: "
            f"{type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return 1

    print(
        "[virtual_carla] baseline gate deep VERIFIED "
        f"sha256={baseline_sha256}"
    )
    print(
        "[virtual_carla] physical 4WS gate deep VERIFIED "
        f"sha256={physical_sha256}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
