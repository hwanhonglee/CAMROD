"""Verify graph manifests distinguish auto parking from each standalone owner."""

from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("profile", ["system_checker.yaml", "system_checker_sim.yaml"])
def test_exactly_one_parking_authority_matches_each_launch_topology(profile):
    source = ROOT / "camrod_system" / "config" / profile
    mirror = ROOT / "camrod_bringup" / "config" / "system" / profile
    assert source.read_bytes() == mirror.read_bytes()
    params = yaml.safe_load(source.read_text())["/system/system_checker"]["ros__parameters"]
    alternatives = params["final_parking"]["alternatives"]

    for expected in ("parking_dispatcher", "reverse_parking_controller", "apriltag_parking_controller"):
        # Materialize each launched topology from its actual node/topic contract.
        selected = params[expected]
        nodes = set(selected["required_nodes"])
        topics = {item.split("|", 1)[0] for item in selected["required_topics"]}
        matching = []
        for name in alternatives:
            spec = params[name]
            if set(spec["required_nodes"]) <= nodes and {
                item.split("|", 1)[0] for item in spec["required_topics"]
            } <= topics:
                matching.append(name)
        assert matching == [expected]
    auto = params["parking_dispatcher"]
    assert "/parking/status|avg_msgs/msg/ModuleState|1" in auto["required_topics"]
    assert "/parking/private/reverse/status|avg_msgs/msg/ModuleState|1" in auto["required_topics"]
    assert "/parking/private/apriltag/status|avg_msgs/msg/ModuleState|1" in auto["required_topics"]


@pytest.mark.parametrize("profile", ["default", "sim"])
def test_aggregate_registry_includes_only_public_dispatcher_and_legacy_identities(profile):
    relative = Path("diagnostics") / profile / "aggregator" / "diagnostics_config.yaml"
    source = ROOT / "camrod_system" / "config" / relative
    mirror = ROOT / "camrod_bringup" / "config" / "system" / relative
    assert source.read_bytes() == mirror.read_bytes()
    document = yaml.safe_load(source.read_text())
    parking = {item["name"] for item in document["topics"] if item["group"] == "parking/runtime"}
    assert "parking/parking_dispatcher" in parking
    assert "parking/reverse_parking_controller" in parking
    assert "parking/apriltag_parking_controller" in parking
    assert all("private" not in name for name in parking)
