from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SNAPSHOT_CONFIG = PACKAGE_ROOT / "config" / "snapshot" / "camrod_topics.params.yaml"
LAUNCH_DEFAULTS = PACKAGE_ROOT / "config" / "bringup" / "launch_defaults.yaml"
BRINGUP_LAUNCH = PACKAGE_ROOT / "launch" / "_bringup_impl.py"


def _snapshot_parameters():
    return yaml.safe_load(SNAPSHOT_CONFIG.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]


# HH_260918 - Give every snapshot topic one typed absolute-name specification.
def test_snapshot_topics_have_unique_details_and_absolute_names():
    params = _snapshot_parameters()
    topics = params["topics"]
    details = params["topic_details"]

    assert len(topics) == len(set(topics))
    assert all(topic.startswith("/") for topic in topics)
    assert set(topics) == set(details)
    assert all(details[topic].get("type") for topic in topics)


# HH_260918 - Retain five minutes of evidence in bounded one-minute bagfiles.
def test_five_minute_buffer_is_written_in_one_minute_bagfiles():
    params = _snapshot_parameters()

    assert params["default_duration_limit"] == 300.0
    assert params["bagfile_split_duration_s"] == 60


# HH_260918 - Capture sensing through control evidence, including right radar raw data.
def test_snapshot_covers_each_debug_layer_and_right_radar_raw_data():
    topics = set(_snapshot_parameters()["topics"])
    required = {
        "/system/diagnostics_agg",
        "/sensing/radar/right1/range",
        "/perception/obstacles",
        "/planning/state_machine/state",
        "/ui/selected_destination",
        "/control/cmd_vel_safety_gate/status",
        "/platform/status",
        "/tf",
        "/tf_static",
    }

    assert required <= topics


# HH_260918 - Enable and pass the snapshot policy through central bringup.
def test_bringup_owns_and_forwards_snapshot_configuration():
    defaults = yaml.safe_load(LAUNCH_DEFAULTS.read_text(encoding="utf-8"))["bringup"]
    launch_source = BRINGUP_LAUNCH.read_text(encoding="utf-8")

    assert defaults["runtime"]["enable_snapshot"] is True
    assert defaults["snapshot"]["param_file"] == "snapshot/camrod_topics.params.yaml"
    assert "bringup_cfg(cfg_get(" in launch_source
    assert "'params_file': lc['snapshot_param_file']" in launch_source
    assert "'camrod_snapshot.launch.py'" in launch_source


# HH_260921 - Automatic evidence capture lives in camrod_snapshot, but the
# policy it applies is configuration. Pin the decisions that are easy to
# regress by editing the YAML.

SNAPSHOT_PACKAGE_CONFIG = (
    PACKAGE_ROOT.parent / "camrod_snapshot" / "param" / "camrod_topics.params.yaml"
)


def _auto_trigger(config_path=SNAPSHOT_CONFIG):
    params = yaml.safe_load(config_path.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]
    return params["auto_trigger"]


def test_auto_trigger_rules_are_declared_and_well_formed():
    auto_trigger = _auto_trigger()
    rules = auto_trigger["rules"]

    assert auto_trigger["enabled"] is True
    assert len(rules) == len(set(rules))
    # Rule names become part of the bag filename; the node rejects anything else.
    assert all(rule.replace("_", "").replace("-", "").isalnum() for rule in rules)
    assert set(rules) == set(auto_trigger["rule"])

    for name in rules:
        rule = auto_trigger["rule"][name]
        assert rule["topic"].startswith("/"), name
        assert rule["kind"] in {"module_state", "system_status"}, name
        # Every rule needs some debounce, but which kind depends on the shape
        # of the event: a sustained condition waits (hold_s), a self-clearing
        # one counts (min_occurrences). Neither would spend the buffer freely.
        assert rule["hold_s"] >= 0.0, name
        assert rule["hold_s"] > 0.0 or rule.get("min_occurrences", 1) > 1, name
        conditions = (
            rule.get("operating_states"),
            rule.get("min_level", -1) >= 0 or None,
            rule.get("on_system_not_ok") or None,
        )
        assert any(conditions), name


def test_startup_faults_cannot_write_a_bag():
    auto_trigger = _auto_trigger()

    # Boot ordering reaches ERROR before it ever reaches OK, so a cold-start
    # fault is excluded by the shape of the transition rather than by guessing
    # how long boot takes: a rule stays disarmed until its topic has reported
    # healthy. startup_grace_s is only the second, independent guard.
    assert auto_trigger["startup_grace_s"] > 0.0
    for name in auto_trigger["rules"]:
        rule = auto_trigger["rule"][name]
        assert rule["require_healthy_first"] is True, name
        assert rule["require_healthy_s"] > 0.0, name


def test_auto_capture_cooldown_covers_one_whole_buffer():
    params = _snapshot_parameters()

    # A write pauses recording and clears every buffer, so a second capture
    # inside one buffer length would record almost nothing. 0.0 tells the node
    # to inherit default_duration_limit.
    assert params["auto_trigger"]["cooldown_s"] == 0.0
    assert params["default_duration_limit"] == 300.0
    # Module ERROR is routine during boot ordering.
    assert params["auto_trigger"]["startup_grace_s"] > 0.0


# HH_260928 - Routine lanelet contact must not spend the automatic bag buffer.
def test_lanelet_contact_does_not_trigger_an_automatic_snapshot():
    auto_trigger = _auto_trigger()

    assert "route_boundary_repeat_contact" not in auto_trigger["rules"]
    assert "route_boundary_repeat_contact" not in auto_trigger["rule"]


def test_platform_faults_capture_immediately_and_everywhere():
    auto_trigger = _auto_trigger()
    rule = auto_trigger["rule"]["gate_fault_hold"]

    # FAULT_HOLD is estop, an abnormal vehicle_state or a platform error.
    # Those are hardware faults rather than ordinary holds, so one is already
    # worth a bag and it is never traded against a contact count.
    assert rule["operating_states"] == ["FAULT_HOLD"]
    assert rule.get("min_occurrences", 1) == 1
    # Deliberately unscoped: a platform fault inside a campsite or at the
    # charger matters as much as one on the road.
    assert "scope_topic" not in rule
    assert "scope_active_states" not in rule

    assert "gate_fault_hold" in auto_trigger["rules"]


def test_every_counting_rule_declares_its_scope():
    auto_trigger = _auto_trigger()
    for name in auto_trigger["rules"]:
        rule = auto_trigger["rule"][name]
        if rule.get("min_occurrences", 1) > 1:
            assert rule.get("scope_topic"), name
            assert rule.get("scope_reset_states"), name


def test_bringup_and_package_snapshot_configs_stay_in_sync():
    # camrod_snapshot ships a standalone fallback that its launch file prefers
    # bringup's copy over. They must not drift.
    assert SNAPSHOT_PACKAGE_CONFIG.is_file()
    assert yaml.safe_load(
        SNAPSHOT_PACKAGE_CONFIG.read_text(encoding="utf-8")
    ) == yaml.safe_load(SNAPSHOT_CONFIG.read_text(encoding="utf-8"))



# HH_260921 - Finished bags are moved to shared storage. Pin the parts of that
# policy that are easy to get wrong by editing the YAML.


def _offload(config_path=SNAPSHOT_CONFIG):
    return yaml.safe_load(config_path.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]["offload"]


def test_offload_targets_shared_storage_over_ssh():
    offload = _offload()

    assert offload["enabled"] is True
    assert offload["host"]
    assert offload["port"] == 8008
    assert offload["user"] == "admin"
    assert offload["remote_directory"] == "/volume1/home/admin/camrod/storage"
    assert offload["remote_directory"].startswith("/")


def test_offload_never_leaves_a_bag_without_a_copy():
    offload = _offload()

    # remove_local_after_transfer makes this a move, which is only safe
    # because the local bag is removed after a verified transfer and never
    # after a failed one. Retries and bounded timeouts keep a stalled link
    # from silently dropping evidence.
    assert offload["retries"] >= 1
    assert offload["connect_timeout_s"] >= 1
    assert offload["transfer_timeout_s"] >= 1
    assert offload["retry_delay_s"] >= 0


def test_offload_carries_no_secret():
    # The transfer runs with BatchMode=yes and authenticates with a key, so no
    # password may ever appear in configuration.
    offload = _offload()
    for key, value in offload.items():
        assert "password" not in key.lower(), key
        assert "passwd" not in key.lower(), key
        if isinstance(value, str):
            assert "@" not in value or key == "host", key
