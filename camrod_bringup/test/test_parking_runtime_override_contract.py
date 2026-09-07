"""Production-safe contract for final parking runtime overlays."""

from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]


def test_production_parking_runtime_overlay_is_empty_and_forwarded_last():
    defaults = yaml.safe_load(
        (
            SRC_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]
    disabled = yaml.safe_load(
        (
            SRC_ROOT
            / "camrod_control"
            / "config"
            / "parking_runtime_profiles"
            / "disabled.yaml"
        ).read_text(encoding="utf-8")
    )
    parking_launch = (
        SRC_ROOT / "camrod_control" / "launch" / "parking.launch.py"
    ).read_text(encoding="utf-8")
    bringup_launch = (
        SRC_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")

    assert defaults["parking"]["runtime_override_param_file"] == (
        "__module_default__"
    )
    assert all(
        entry["ros__parameters"] == {} for entry in disabled.values()
    )
    # Auto mode adds a dispatcher which intentionally consumes only the base
    # profile.  Both concrete parking controllers must still receive the final
    # sparse runtime overlay after that base profile.
    for executable in (
        "reverse_parking_controller_node",
        "apriltag_parking_controller_node",
    ):
        controller_block = parking_launch.split(
            f'executable="{executable}"', 1
        )[1].split("Node(", 1)[0]
        parameter_chain = controller_block.split("parameters=[", 1)[1].split(
            "remappings=", 1
        )[0]
        assert parameter_chain.index('LaunchConfiguration("parameter_file")') < (
            parameter_chain.index(
                'LaunchConfiguration("runtime_override_parameter_file")'
            )
        )
    assert "'parking_runtime_override_param_file'," in bringup_launch
    assert "'runtime_override_parameter_file': lc[" in bringup_launch
