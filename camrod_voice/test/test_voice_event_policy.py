from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from voice_event_policy import VoiceEventPolicy, module_snapshots  # noqa: E402


REQUIRED_MODULES = (
    "map",
    "sensing",
    "localization",
    "planning",
    "control",
    "platform",
    "system",
)


def event_keys(events):
    return [event.key for event in events]


def healthy_modules(*, starting_module=""):
    values = []
    for name in REQUIRED_MODULES:
        operating_state = "STARTING" if name == starting_module else "READY"
        values.append((name, 0, operating_state))
    return module_snapshots(values)


def degraded_modules():
    values = []
    for name in REQUIRED_MODULES:
        if name == "sensing":
            values.append((name, 1, "DUMMY"))
        elif name == "system":
            values.append((name, 1, "DEGRADED"))
        else:
            values.append((name, 0, "READY"))
    return module_snapshots(values)


def make_ready_policy(*, announce_startup=True):
    policy = VoiceEventPolicy(REQUIRED_MODULES)
    events = []
    if announce_startup:
        events.extend(policy.announce_startup())
    events.extend(policy.update_system(healthy_modules()))
    events.extend(
        policy.update_planning(
            state="WAIT_DZ",
            scenario="WAIT_DROP_ZONE",
            active_mission_key="",
            active_goal_source="none",
            return_requested=False,
        )
    )
    events.extend(policy.update_action_server(True))
    events.extend(policy.update_localization(0))
    events.extend(policy.update_tf(True))
    events.extend(
        policy.update_gate(
            level=0,
            operating_state="STANDBY",
            message="reasons=engage=false",
        )
    )
    events.extend(policy.update_platform(estop=False, error_code=0))
    events.extend(policy.update_engaged(False))
    return policy, events


def test_startup_wait_state_never_announces_return_or_ready_early():
    policy = VoiceEventPolicy(REQUIRED_MODULES)

    events = policy.update_planning(
        state="WAIT_DZ",
        scenario="WAIT_DROP_ZONE",
        active_mission_key="",
        active_goal_source="none",
        return_requested=False,
    )
    events.extend(policy.announce_startup())

    assert event_keys(events) == ["system.startup"]
    assert not policy.ready


def test_ready_requires_every_input_and_is_announced_once():
    policy, events = make_ready_policy()

    assert event_keys(events) == ["system.startup", "system.ready"]
    assert policy.ready
    assert policy.update_system(healthy_modules()) == []
    assert policy.update_localization(0) == []
    assert policy.update_tf(True) == []


def test_dummy_warn_is_degraded_ready_but_error_still_blocks():
    policy = VoiceEventPolicy(REQUIRED_MODULES)
    events = policy.announce_startup()
    events.extend(policy.update_system(degraded_modules()))
    events.extend(
        policy.update_planning(
            state="WAIT_DZ",
            scenario="WAIT_DROP_ZONE",
            active_mission_key="",
            active_goal_source="none",
            return_requested=False,
        )
    )
    events.extend(policy.update_action_server(True))
    events.extend(policy.update_localization(0))
    events.extend(policy.update_tf(True))
    events.extend(
        policy.update_gate(
            level=1,
            operating_state="STANDBY",
            message="degraded monitoring",
        )
    )
    events.extend(policy.update_platform(estop=False, error_code=0))
    events.extend(policy.update_engaged(False))

    assert event_keys(events) == ["system.startup", "system.ready"]
    assert policy.ready

    failed_modules = degraded_modules()
    failed_modules["sensing"] = (2, "FAULT")
    assert policy.update_system(failed_modules) == []
    assert not policy.ready
    assert "module_unhealthy:sensing:2" in policy.readiness_reasons()


def test_starting_module_and_missing_tf_block_false_ready():
    policy = VoiceEventPolicy(REQUIRED_MODULES)
    policy.announce_startup()
    policy.update_system(healthy_modules(starting_module="planning"))
    policy.update_planning(
        state="WAIT_DZ",
        scenario="WAIT_DROP_ZONE",
        active_mission_key="",
        active_goal_source="none",
        return_requested=False,
    )
    policy.update_action_server(True)
    policy.update_localization(0)
    policy.update_gate(level=0, operating_state="STANDBY", message="")
    policy.update_platform(estop=False, error_code=0)
    policy.update_engaged(False)

    assert "module_not_ready:planning:starting" in policy.readiness_reasons()
    assert "tf_unavailable" in policy.readiness_reasons()
    assert not policy.ready_announced


def test_action_server_is_an_explicit_readiness_condition():
    policy, events = make_ready_policy()
    assert event_keys(events).count("system.ready") == 1

    policy = VoiceEventPolicy(REQUIRED_MODULES)
    policy.announce_startup()
    policy.update_system(healthy_modules())
    policy.update_planning(
        state="WAIT_DZ",
        scenario="WAIT_DROP_ZONE",
        active_mission_key="",
        active_goal_source="none",
        return_requested=False,
    )
    policy.update_localization(0)
    policy.update_tf(True)
    policy.update_gate(level=0, operating_state="STANDBY", message="")
    policy.update_platform(estop=False, error_code=0)
    policy.update_engaged(False)

    assert "planning_action_server_missing" in policy.readiness_reasons()
    assert event_keys(policy.update_action_server(True)) == ["system.ready"]


def test_localization_reacquisition_does_not_repeat_ready():
    policy, events = make_ready_policy()
    assert event_keys(events).count("system.ready") == 1

    assert policy.update_localization(2) == []
    assert not policy.ready
    assert policy.update_localization(0) == []
    assert policy.ready


def test_manual_goal_waits_for_engage_and_reengage_announces_once_each_time():
    policy, _ = make_ready_policy()

    events = policy.update_planning(
        state="RUNNING",
        scenario="DELIVERY_TO_SITE",
        active_mission_key="",
        active_goal_source="manual",
        return_requested=False,
    )
    assert events == []

    events = policy.update_engaged(True)
    assert events == []
    events = policy.update_gate(
        level=0, operating_state="ENABLED", message="reasons=none"
    )
    assert event_keys(events) == ["navigation.to_campsite"]
    assert policy.update_engaged(True) == []
    assert (
        policy.update_planning(
            state="RUNNING",
            scenario="DELIVERY_TO_SITE",
            active_mission_key="",
            active_goal_source="manual",
            return_requested=False,
        )
        == []
    )

    assert policy.update_engaged(False) == []
    assert policy.update_gate(
        level=0, operating_state="STANDBY", message="reasons=engage=false"
    ) == []
    assert policy.update_engaged(True) == []
    assert event_keys(
        policy.update_gate(
            level=0, operating_state="ENABLED", message="reasons=none"
        )
    ) == ["navigation.to_campsite"]


def test_ui_goal_uses_same_running_vocabulary_as_manual_goal():
    policy, _ = make_ready_policy()
    policy.update_engaged(True)
    policy.update_gate(
        level=0, operating_state="ENABLED", message="reasons=none"
    )

    events = policy.update_planning(
        state="RUNNING",
        scenario="DELIVERY_TO_SITE",
        active_mission_key="camping_site_3",
        active_goal_source="regulated",
        return_requested=False,
    )

    assert event_keys(events) == ["navigation.to_campsite"]


def test_return_audio_requires_a_released_drop_zone_goal():
    policy, _ = make_ready_policy()
    policy.update_engaged(True)
    policy.update_gate(
        level=0, operating_state="ENABLED", message="reasons=none"
    )

    pending = policy.update_planning(
        state="RETURNING",
        scenario="RETURN_TO_DROP_ZONE",
        active_mission_key="camping_site_3",
        active_goal_source="return_request",
        return_requested=True,
    )
    assert pending == []
    assert not policy.travel_active

    released = policy.update_planning(
        state="RETURNING",
        scenario="RETURN_TO_DROP_ZONE",
        active_mission_key="drop_zone",
        active_goal_source="auto_snapper:drop_zone",
        return_requested=False,
    )
    assert event_keys(released) == ["navigation.to_dropzone"]
    assert policy.travel_context() == "drop_zone"
    assert policy.travel_active


def test_travel_reminders_follow_the_trip_direction():
    policy, _ = make_ready_policy()
    policy.update_engaged(True)
    policy.update_gate(
        level=0, operating_state="ENABLED", message="reasons=none"
    )
    assert policy.travel_announce_events() == []

    policy.update_planning(
        state="RUNNING",
        scenario="DELIVERY_TO_SITE",
        active_mission_key="camping_site_2",
        active_goal_source="regulated",
        return_requested=False,
    )
    assert event_keys(policy.travel_announce_events()) == [
        "system.announce1",
        "system.announce2",
    ]

    policy.update_planning(
        state="RETURNING",
        scenario="RETURN_WITH_CARGO",
        active_mission_key="drop_zone",
        active_goal_source="auto_snapper:drop_zone",
        return_requested=False,
    )
    assert event_keys(policy.travel_announce_events()) == [
        "navigation.return_to_dropzone",
    ]


def test_music_bed_waits_for_departure_and_ends_with_the_trip():
    policy, _ = make_ready_policy()
    policy.update_engaged(True)
    policy.update_gate(
        level=0, operating_state="ENABLED", message="reasons=none"
    )
    assert not policy.travel_active

    departure = policy.update_planning(
        state="RUNNING",
        scenario="DELIVERY_TO_SITE",
        active_mission_key="camping_site_2",
        active_goal_source="regulated",
        return_requested=False,
    )
    assert event_keys(departure) == ["navigation.to_campsite"]
    assert policy.travel_active

    # A safety hold keeps the bed running; only the trip ending stops it.
    policy.update_gate(
        level=1,
        operating_state="SAFETY_HOLD",
        message="reasons=cost_stop_latched",
    )
    assert policy.travel_active

    policy.update_planning(
        state="GOAL_REACHED",
        scenario="DELIVERY_TO_SITE",
        active_mission_key="camping_site_2",
        active_goal_source="regulated",
        return_requested=False,
    )
    assert not policy.travel_active
    assert policy.travel_announce_events() == []


def test_blocked_route_keeps_a_standing_explanation_available():
    policy, _ = make_ready_policy()
    policy.update_engaged(True)
    policy.update_gate(
        level=0, operating_state="ENABLED", message="reasons=none"
    )
    assert policy.obstacle_repeat_events() == []

    policy.update_gate(
        level=1,
        operating_state="ROUTE_SAFETY_HOLD",
        message="reasons=cost_hold=1",
    )
    assert policy.obstacle_hold_announced
    assert event_keys(policy.obstacle_repeat_events()) == [
        "navigation.please_step_aside",
    ]

    policy.update_gate(
        level=0, operating_state="ENABLED", message="reasons=none"
    )
    assert not policy.obstacle_hold_announced
    assert policy.obstacle_repeat_events() == []


def test_cleared_obstacle_thanks_before_the_robot_resumes():
    policy, _ = make_ready_policy()
    policy.update_engaged(True)
    policy.update_gate(
        level=0, operating_state="ENABLED", message="reasons=none"
    )

    held = policy.update_gate(
        level=1,
        operating_state="ROUTE_SAFETY_HOLD",
        message="reasons=cost_hold=1",
    )
    assert event_keys(held) == ["safety.obstacle"]

    cleared = policy.update_gate(
        level=0, operating_state="ENABLED", message="reasons=none"
    )
    assert event_keys(cleared) == ["safety.thankyou"]

    # Nothing was held, so a second clear stays silent.
    assert policy.update_gate(
        level=0, operating_state="ENABLED", message="reasons=none"
    ) == []


def test_generic_warn_does_not_claim_obstacle_but_gate_cost_hold_does():
    policy, _ = make_ready_policy()
    policy.update_engaged(True)
    policy.update_gate(
        level=0, operating_state="ENABLED", message="reasons=none"
    )

    assert (
        policy.update_planning(
            state="WARN_RECOVERY",
            scenario="DELIVERY_TO_SITE",
            active_mission_key="camping_site_1",
            active_goal_source="regulated",
            return_requested=False,
        )
        == []
    )
    first_hold = policy.update_gate(
        level=1,
        operating_state="SAFETY_HOLD",
        message="reasons=cost_stop_latched",
    )
    assert event_keys(first_hold) == ["safety.obstacle"]
    assert (
        policy.update_gate(
            level=1,
            operating_state="SAFETY_HOLD",
            message="reasons=cost_stop_latched",
        )
        == []
    )


def test_arrival_requires_prior_announced_motion_and_is_not_duplicated():
    policy, _ = make_ready_policy()
    policy.update_engaged(True)
    policy.update_gate(
        level=0, operating_state="ENABLED", message="reasons=none"
    )
    policy.update_planning(
        state="RUNNING",
        scenario="DELIVERY_TO_SITE",
        active_mission_key="camping_site_2",
        active_goal_source="regulated",
        return_requested=False,
    )

    first = policy.update_planning(
        state="GOAL_REACHED",
        scenario="DELIVERY_TO_SITE",
        active_mission_key="camping_site_2",
        active_goal_source="regulated",
        return_requested=False,
    )
    duplicate = policy.update_planning(
        state="GOAL_REACHED",
        scenario="UNLOAD_WAIT",
        active_mission_key="camping_site_2",
        active_goal_source="regulated",
        return_requested=False,
    )

    assert event_keys(first) == ["navigation.arrived_campsite"]
    assert duplicate == []


def test_stale_goal_reached_at_startup_is_silent():
    policy, _ = make_ready_policy()

    events = policy.update_planning(
        state="GOAL_REACHED",
        scenario="DELIVERY_TO_SITE",
        active_mission_key="camping_site_2",
        active_goal_source="regulated",
        return_requested=False,
    )

    assert events == []
