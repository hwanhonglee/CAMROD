"""Regression tests for bringup-to-sensing radar default synchronization."""

import ast
from pathlib import Path

import yaml


_BRINGUP_ROOT = Path(__file__).resolve().parents[1]
_WORKSPACE_SRC = _BRINGUP_ROOT.parent
_BRINGUP_IMPL_PATH = _BRINGUP_ROOT / "launch" / "_bringup_impl.py"
_SENSING_LAUNCH_PATH = (
    _WORKSPACE_SRC / "camrod_sensing" / "launch" / "sensing.launch.py"
)
_RADAR_LAUNCH_PATH = (
    _WORKSPACE_SRC / "camrod_sensing" / "launch" / "radar.launch.py"
)
_SYSTEM_CHECKER_PATH = (
    _WORKSPACE_SRC / "camrod_system" / "config" / "system_checker.yaml"
)
_BRINGUP_SYSTEM_CHECKER_PATH = (
    _BRINGUP_ROOT / "config" / "system" / "system_checker.yaml"
)

_SENSOR_NAMES = [
    "FRONT1",
    "FRONT2",
    "LEFT1",
    "LEFT2",
    "RIGHT1",
    "RIGHT2",
    "REAR",
]

_EXPECTED_DRIVER_ARRAYS = {
    "sensor_names": _SENSOR_NAMES,
    "sensor_enabled": [True] * 7,
    "hardware_angle_levels": [1] * 7,
    "hardware_range_levels": [2, 2, 2, 2, 2, 2, 1],
    "software_max_ranges_m": [1.50, 1.50, 0.80, 0.80, 0.80, 0.80, 0.50],
    "frame_ids": [
        "radar_front1_link",
        "radar_front2_link",
        "radar_left1_link",
        "radar_left2_link",
        "radar_right1_link",
        "radar_right2_link",
        "radar_rear_link",
    ],
    "ports": [
        "/dev/ttyCH9344USB0",
        "/dev/ttyCH9344USB1",
        "/dev/ttyCH9344USB4",
        "/dev/ttyCH9344USB5",
        "/dev/ttyCH9344USB2",
        "/dev/ttyCH9344USB3",
        "/dev/ttyCH9344USB6",
    ],
    "topics": [
        "front1/range",
        "front2/range",
        "left1/range",
        "left2/range",
        "right1/range",
        "right2/range",
        "rear/range",
    ],
    "standard_ros_topics": [
        "front1/range_ros",
        "front2/range_ros",
        "left1/range_ros",
        "left2/range_ros",
        "right1/range_ros",
        "right2/range_ros",
        "rear/range_ros",
    ],
}

_EXPECTED_DRIVER_SCALARS = {
    "hardware_write_on_startup": True,
    "software_min_range_m": 0.02,
    "software_default_max_range_m": 4.50,
    "range_message_field_of_view_rad": 0.26,
}

_EXPECTED_FIXED_RETURN_INTERVALS = [
    ("FRONT1", 0.099, 0.123),
    ("FRONT1", 0.152, 0.220),
    ("FRONT2", 0.097, 0.117),
    ("LEFT1", 0.182, 0.226),
    ("LEFT1", 0.234, 0.258),
    ("LEFT2", 0.210, 0.280),
    ("RIGHT1", 0.055, 0.080),
    ("RIGHT1", 0.253, 0.277),
    ("RIGHT2", 0.248, 0.278),
    ("REAR", 0.090, 0.190),
]

_OLD_DRIVER_KEYS = {
    "configure_hardware_on_startup",
    "field_of_view_rad",
    "max_range_m",
    "min_range_m",
    "sensor_angle_config_values",
    "sensor_max_ranges_m",
    "sensor_range_config_levels",
    "sensor_range_config_mm",
}

_OLD_COST_GRID_KEYS = {
    "cost_range_min_m",
    "cost_range_max_m",
    "self_echo_filter_enable",
    "self_echo_band_specs",
    "self_echo_sensor_indices",
    "self_echo_centers_m",
    "self_echo_half_widths_m",
    "startup_self_echo_calibration_enable",
    "startup_self_echo_calibration_duration_s",
    "startup_self_echo_first_sample_timeout_s",
    "startup_self_echo_calibration_min_samples",
    "startup_self_echo_cluster_gap_m",
    "startup_self_echo_min_cluster_fraction",
    "startup_self_echo_min_half_width_m",
    "startup_self_echo_max_half_width_m",
    "startup_self_echo_margin_m",
    "startup_self_echo_candidate_max_range_m",
    "startup_self_echo_candidate_max_ranges_m",
    "startup_self_echo_authorization_topic",
}


def _radar_config(package, filename):
    return (
        _WORKSPACE_SRC
        / package
        / "config"
        / ("sensing/radar" if package == "camrod_bringup" else "radar")
        / filename
    )


def _load_driver_params():
    config = yaml.safe_load(
        _radar_config("camrod_sensing", "sen0592_radar.yaml").read_text(
            encoding="utf-8"
        )
    )
    return config["/**"]["ros__parameters"]


def _load_cost_grid_params():
    config = yaml.safe_load(
        _radar_config("camrod_sensing", "cost_grid.yaml").read_text(
            encoding="utf-8"
        )
    )
    return config["/sensing/radar/radar_cost_grid"]["ros__parameters"]


def _parse_band_spec(spec):
    fields = spec.split(":")
    assert len(fields) == 3
    sensor, minimum, maximum = fields
    return sensor, float(minimum), float(maximum)


def _bringup_sensing_argument(key_name):
    tree = ast.parse(_BRINGUP_IMPL_PATH.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(
            isinstance(target, ast.Name) and target.id == "sensing_args"
            for target in node.targets
        ):
            continue
        if not isinstance(node.value, ast.Dict):
            continue
        for key, value in zip(node.value.keys, node.value.values):
            if isinstance(key, ast.Constant) and key.value == key_name:
                return value
    raise AssertionError(f"missing sensing_args[{key_name!r}]")


# HH_260729 - The sensing package owns the two active radar profiles; full
# bringup must deploy byte-identical mirrors so comments and safety values
# cannot drift independently.
def test_bringup_radar_configs_match_sensing_package_byte_for_byte():
    for filename in ("sen0592_radar.yaml", "cost_grid.yaml"):
        sensing = _radar_config("camrod_sensing", filename)
        bringup = _radar_config("camrod_bringup", filename)
        assert bringup.read_bytes() == sensing.read_bytes()


# HH_260729 - Keep the five operator-facing sections visible and ordered in
# both active profiles. This protects the category layout as well as values.
def test_radar_configs_keep_readable_ordered_category_headings():
    expected_headings = {
        "sen0592_radar.yaml": [
            "1. SERIAL COMMUNICATION",
            "2. PHYSICAL SENSOR REGISTERS",
            "3. SOFTWARE ACCEPTANCE",
            "4. ROS OUTPUT",
            "5. CHANNEL ORDER AND WIRING",
        ],
        "cost_grid.yaml": [
            "1. ROS I/O AND FRAMES",
            "2. GRID GEOMETRY",
            "3. OBSTACLE COST PAINTING",
            "4. FIXED RANGE EXCLUSIONS",
            "5. STARTUP RETURN LEARNING",
            "6. ACTIVE-ROUTE FILTER",
            "7. ACTIVE-HIT EVIDENCE, TIMING, AND OPTIONAL AGGREGATE STATUS",
        ],
    }

    for filename, headings in expected_headings.items():
        text = _radar_config("camrod_sensing", filename).read_text(encoding="utf-8")
        offsets = [text.index(heading) for heading in headings]
        assert offsets == sorted(offsets)


# HH_260729 - Lock every per-sensor array to one logical sensor order,
# including physical register levels, software limits, and the intentionally
# crossed LEFT/RIGHT CH9344 field harness.
def test_driver_arrays_have_complete_ordered_sensor_mapping():
    params = _load_driver_params()
    for name, expected in _EXPECTED_DRIVER_ARRAYS.items():
        assert len(params[name]) == len(_SENSOR_NAMES)
        assert params[name] == expected


# HH_260729 - Physical register requests, software range acceptance, and ROS
# visualization metadata have separate, explicit names and units.
def test_driver_uses_canonical_layered_parameters_only():
    params = _load_driver_params()

    assert all(type(value) is bool for value in params["sensor_enabled"])
    assert all(
        type(value) is int and 1 <= value <= 4
        for value in params["hardware_angle_levels"]
    )
    assert all(
        type(value) is int and 1 <= value <= 5
        for value in params["hardware_range_levels"]
    )
    for name, expected in _EXPECTED_DRIVER_SCALARS.items():
        assert params[name] == expected
    assert _OLD_DRIVER_KEYS.isdisjoint(params)


# HH_260729 - Validate the preferred SENSOR:min_m:max_m profile directly so
# operators never need to correlate three parallel arrays by index.
def test_named_fixed_return_bands_are_known_ordered_positive_intervals():
    params = _load_cost_grid_params()
    specs = params["fixed_return_bands"]
    input_labels = {
        topic.rstrip("/").split("/")[-2].upper()
        for topic in params["input_topics"]
    }
    parsed = [_parse_band_spec(spec) for spec in specs]

    assert params["fixed_return_filter_enable"] is True
    assert parsed == _EXPECTED_FIXED_RETURN_INTERVALS
    assert all(sensor in input_labels for sensor, _, _ in parsed)
    assert all(0.0 < minimum < maximum for _, minimum, maximum in parsed)


# HH_260729 - Startup learning limits are safety mappings tied directly to the
# seven input topics. Preserve the field-tested values and canonical names.
def test_startup_return_learning_has_complete_sensor_mapping():
    params = _load_cost_grid_params()
    assert params["startup_return_learning_enable"] is False
    assert params["startup_return_learning_duration_s"] == 8.0
    assert params["startup_return_first_sample_timeout_s"] == 15.0
    assert params["startup_return_min_samples"] == 15
    assert params["startup_return_cluster_gap_m"] == 0.020
    assert params["startup_return_min_cluster_fraction"] == 0.50
    assert params["startup_return_min_half_width_m"] == 0.012
    assert params["startup_return_max_half_width_m"] == 0.030
    assert params["startup_return_margin_m"] == 0.005
    assert params["startup_return_default_max_range_m"] == 0.30
    assert params["startup_return_max_ranges_m"] == [
        0.25,
        0.25,
        0.30,
        0.30,
        0.30,
        0.30,
        0.20,
    ]
    assert len(params["startup_return_max_ranges_m"]) == len(params["input_topics"])
    assert (
        params["startup_return_authorization_topic"]
        == "/control/planning_engaged"
    )


# HH_260729 - Active deployment exposes canonical processing-stage names only;
# retired self-echo/index aliases must not reappear in either mirror.
def test_active_cost_grid_uses_canonical_cost_and_return_names_only():
    params = _load_cost_grid_params()
    assert params["cost_near_distance_m"] == 0.30
    assert params["cost_far_distance_m"] == 2.00
    assert (
        params["obstacle_evidence_topic"]
        == "/sensing/radar/obstacle_evidence"
    )
    assert params["obstacle_evidence_warn_interval_s"] == 1.0
    assert params["dummy_active_topic"] == "/sensing/radar/dummy_active"
    assert params["dummy_active_timeout_s"] == 1.0
    assert _OLD_COST_GRID_KEYS.isdisjoint(params)


# HH_260729 - One master policy enables explicit low-rate dummy contracts for
# every disabled hardware input. Top-level sim forces it off because
# fake_sensors.launch.py already owns the synthetic topics.
def test_sensor_dummy_default_is_forwarded_with_sim_forced_off():
    defaults = yaml.safe_load(
        (_BRINGUP_ROOT / "config" / "bringup" / "launch_defaults.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert (
        defaults["bringup"]["sensing"]["publish_sensor_dummies_when_disabled"]
        is True
    )

    expression = ast.unparse(
        _bringup_sensing_argument("publish_sensor_dummies_when_disabled")
    )
    assert expression == (
        "sim_switch(lc['sim'], 'false', "
        "lc['publish_sensor_dummies_when_disabled'])"
    )
    bringup_source = _BRINGUP_IMPL_PATH.read_text(encoding="utf-8")
    assert (
        "\"').lower() in ['1', 'true', 'yes', 'on'] else '\""
        in bringup_source
    )


# HH_260729 - Keep the switch visible through both aggregate launch layers,
# and keep the serial and dummy publishers mutually exclusive at the radar
# leaf launch.
def test_radar_dummy_launch_is_mutually_exclusive_and_fully_forwarded():
    sensing_source = _SENSING_LAUNCH_PATH.read_text(encoding="utf-8")
    radar_source = _RADAR_LAUNCH_PATH.read_text(encoding="utf-8")

    assert (
        '"publish_sensor_dummies_when_disabled"'
        in sensing_source
    )
    assert (
        "enable_radar_dummy_when_disabled=LaunchConfiguration("
        in sensing_source
    )
    assert (
        'executable="radar_dummy_publisher.py"'
        in radar_source
    )
    assert "parameters=[radar_sensor_param_file]" in radar_source
    assert (
        "enable_radar_dummy_when_disabled = LaunchConfiguration("
        in radar_source
    )
    assert (
        "enable_radar_dummy_when_disabled,\n"
        "                \"').lower() in ['1', 'true', 'yes', 'on']"
        in radar_source
    )
    assert (
        "\"').lower() not in ['1', 'true', 'yes', 'on']"
        in radar_source
    )


# HH_260729 - Full hardware-off radar tests use a different executable name.
# Graph readiness therefore validates the seven shared typed topic contracts,
# while radar_checker separately and visibly identifies physical versus dummy.
def test_system_graph_accepts_physical_or_dummy_radar_source_by_topics():
    assert (
        _BRINGUP_SYSTEM_CHECKER_PATH.read_bytes()
        == _SYSTEM_CHECKER_PATH.read_bytes()
    )
    config = yaml.safe_load(
        _SYSTEM_CHECKER_PATH.read_text(encoding="utf-8")
    )
    params = config["/system/system_checker"]["ros__parameters"]
    sensing = params["sensing"]

    assert "/sensing/radar/sen0592_radar_node" not in sensing["required_nodes"]
    radar_topics = {
        entry.split("|", maxsplit=1)[0]
        for entry in sensing["required_topics"]
        if entry.startswith("/sensing/radar/")
        and entry.endswith("|avg_msgs/msg/AvgRange|1")
    }
    assert radar_topics == {
        f"/sensing/radar/{name.lower()}/range"
        for name in _SENSOR_NAMES
    }
