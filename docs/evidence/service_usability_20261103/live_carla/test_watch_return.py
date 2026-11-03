"""HH_261103 - Reject stale identity/runtime/consumer authority before any live action."""

import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location("watch_return_test_target", Path(__file__).with_name("watch_return.py"))
watch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(watch)


def admitted():
    return {"mission_dispatch_generation": 123, "mission_dispatch_site": "B7", "mission_dispatch_active": True,
            "mission_dispatch_intent": "delivery", "mission_dispatch_owner": "operator", "ready": True,
            "battery_percentage": 80, "service_state_name": "WAITING_FOR_RETURN_REQUEST"}


def test_exact_admitted_identity_and_observed_arrival_are_required():
    state = admitted()
    watch.validate_identity(state, 123, "B7", arrival_soc=80)
    for key, value in [("mission_dispatch_generation", 124), ("mission_dispatch_generation", True),
            ("mission_dispatch_site", "B6"), ("mission_dispatch_active", False),
            ("mission_dispatch_owner", "guest"), ("mission_dispatch_intent", "recall"),
            ("ready", False), ("battery_percentage", 34), ("service_state_name", "MOVING_TO_SITE")]:
        with pytest.raises(RuntimeError):
            watch.validate_identity({**state, key: value}, 123, "B7", arrival_soc=80)


def test_runtime_is_exact_isolated_simulation():
    env = {"ROS_DOMAIN_ID": "188", "ROS_LOCALHOST_ONLY": "1", "CAMROD_RECORDING_ENVIRONMENT": "simulation"}
    watch.validate_runtime(env)
    for key in env:
        with pytest.raises(RuntimeError):
            watch.validate_runtime({**env, key: "wrong"})


def test_soc_graph_allows_only_heartbeat_and_passive_observer_without_a_publisher():
    graph = "Type: std_msgs/msg/Float32\nPublisher count: 0\nNode name: carla_platform_heartbeat\nNode name: carla_battery_evidence_observer\n"
    watch.validate_soc_graph(graph)
    for invalid in [graph + "Node name: real_robot\n", graph.replace("Publisher count: 0", "Publisher count: 1"),
                    graph.replace("carla_platform_heartbeat", "fake_platform")]:
        with pytest.raises(RuntimeError):
            watch.validate_soc_graph(invalid)
