"""Contracts for deterministic CARLA visual sensors and CAMROD UI topics."""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent
CONFIG_ROOT = PACKAGE_ROOT / "config"
CONTROL_SPAWN = CONFIG_ROOT / "ranger_spawn_camrod_control_only.json"
VISUAL_SPAWN = CONFIG_ROOT / "ranger_spawn_camrod_full_sensors.json"
SENSOR_PREFLIGHT = (
    REPO_ROOT / "scripts" / "virtual_carla" / "check_carla_sensor_streams.py"
)
CARLA_LIDAR_COST_GRID = CONFIG_ROOT / "carla_lidar_cost_grid.yaml"
CARLA_LIDAR_CHECKER = (
    REPO_ROOT
    / "camrod_bringup"
    / "config"
    / "system"
    / "diagnostics"
    / "carla"
    / "sensing"
    / "lidar_checker.yaml"
)
SYSTEM_CARLA_LIDAR_CHECKER = (
    REPO_ROOT
    / "camrod_system"
    / "config"
    / "diagnostics"
    / "carla"
    / "sensing"
    / "lidar_checker.yaml"
)
CAMROD_LIDAR_COST_GRID = (
    REPO_ROOT / "camrod_bringup" / "config" / "sensing" / "lidar" / "cost_grid.yaml"
)
DROP_ZONE_FRONT_SPAWN = {
    "x": -20.672548294067383,
    "y": 33.95176696777344,
    "z": 3.063404083251953,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 6.8785247802734375,
}


def _vehicle(path: Path) -> dict:
    document = json.loads(path.read_text(encoding="utf-8"))
    vehicles = [
        item
        for item in document["objects"]
        if item.get("type") == "vehicle.ranger.default"
    ]
    assert len(vehicles) == 1
    return vehicles[0]


def test_visual_profile_preserves_the_accepted_actor_and_control_sensors():
    control = _vehicle(CONTROL_SPAWN)
    visual = _vehicle(VISUAL_SPAWN)

    assert visual["id"] == control["id"] == "ego_vehicle"
    assert visual["spawn_point"] == control["spawn_point"]

    control_contract = {
        (sensor["id"], sensor["type"])
        for sensor in control["sensors"]
    }
    visual_contract = {
        (sensor["id"], sensor["type"])
        for sensor in visual["sensors"]
    }
    assert control_contract <= visual_contract


def test_both_profiles_spawn_on_road_in_front_of_woraksan_drop_zone():
    assert _vehicle(CONTROL_SPAWN)["spawn_point"] == DROP_ZONE_FRONT_SPAWN
    assert _vehicle(VISUAL_SPAWN)["spawn_point"] == DROP_ZONE_FRONT_SPAWN


def test_carla_lidar_cost_grid_returns_to_production_height_after_ground_filter():
    node_key = "/sensing/lidar/lidar_cost_grid"
    production = yaml.safe_load(
        CAMROD_LIDAR_COST_GRID.read_text(encoding="utf-8")
    )[node_key]["ros__parameters"]
    carla = yaml.safe_load(
        CARLA_LIDAR_COST_GRID.read_text(encoding="utf-8")
    )[node_key]["ros__parameters"]

    assert carla == production
    assert carla["cloud_min_z_m"] == pytest.approx(-0.55)


def test_full_carla_launch_passes_its_lidar_cost_profile_to_bringup():
    source = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    bringup = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    assert "carla_lidar_cost_grid.yaml" in source
    assert '"lidar_cost_grid_param_file": LaunchConfiguration(' in source
    assert '"carla_lidar_cost_grid_param_file"' in source
    assert "'lidar_cost_grid_param_file'," in bringup
    assert "sensing_args['lidar_cost_grid_param_file'] = lc[" in bringup
    assert "'lidar_cost_grid_param_file'\n    ]" in bringup


@pytest.mark.parametrize(
    ("sensor_id", "sensor_type", "mount"),
    (
        (
            "rgb_view",
            "sensor.camera.rgb",
            pytest.approx((0.76337, 0.0, 0.49568)),
        ),
        (
            "rgb_rear",
            "sensor.camera.rgb",
            pytest.approx((-0.61933, 0.0, 0.30013)),
        ),
        (
            "lidar_front",
            "sensor.lidar.ray_cast",
            pytest.approx((0.76336, 0.0, 0.59538)),
        ),
        (
            "gnss",
            "sensor.other.gnss",
            pytest.approx((0.0, 0.45, 0.0)),
        ),
        (
            "gnss_right",
            "sensor.other.gnss",
            pytest.approx((0.0, -0.45, 0.0)),
        ),
        (
            "imu",
            "sensor.other.imu",
            pytest.approx((0.688, 0.0, 0.756)),
        ),
    ),
)
def test_visual_profile_has_one_canonical_ten_hz_sensor(
    sensor_id, sensor_type, mount
):
    visual = _vehicle(VISUAL_SPAWN)
    matches = [
        sensor for sensor in visual["sensors"] if sensor["id"] == sensor_id
    ]
    assert len(matches) == 1
    sensor = matches[0]
    assert sensor["type"] == sensor_type
    assert sensor["sensor_tick"] == pytest.approx(0.1)
    point = sensor["spawn_point"]
    assert (point["x"], point["y"], point["z"]) == mount


def test_carla_lidar_is_a_deterministic_forward_solid_state_approximation():
    lidar = next(
        sensor
        for sensor in _vehicle(VISUAL_SPAWN)["sensors"]
        if sensor["id"] == "lidar_front"
    )

    assert lidar["horizontal_fov"] == pytest.approx(120.0)
    assert lidar["upper_fov"] == pytest.approx(10.0)
    assert lidar["lower_fov"] == pytest.approx(-25.0)
    assert lidar["channels"] == 16
    assert lidar["points_per_second"] == 60000
    assert lidar["sensor_tick"] == pytest.approx(0.1)
    # CARLA advances rotation_frequency * horizontal_fov * fixed_delta.
    # At 20 Hz this covers the complete fixed -60..+60 degree sector per tick,
    # rather than alternating left/right half scans.
    assert lidar["rotation_frequency"] * 0.05 == pytest.approx(1.0)
    assert lidar["dropoff_general_rate"] == pytest.approx(0.0)
    assert lidar["dropoff_intensity_limit"] == pytest.approx(1.0)
    assert lidar["dropoff_zero_intensity"] == pytest.approx(0.0)
    assert lidar["noise_stddev"] == pytest.approx(0.0)


def test_rendered_default_uses_visual_profile_and_nullrhi_stays_control_only():
    source = (
        REPO_ROOT / "scripts" / "virtual_carla" / "env.sh"
    ).read_text(encoding="utf-8")
    assert '"${CARLA_RENDER_MODE}" == "nullrhi"' in source
    assert "ranger_spawn_camrod_control_only.json" in source
    assert "ranger_spawn_camrod_full_sensors.json" in source


def test_relay_owns_canonical_frames_and_compressed_topics_in_carla_overlay():
    launch_source = (
        PACKAGE_ROOT / "launch" / "sensor_relay.launch.py"
    ).read_text(encoding="utf-8")
    for expected in (
        'default_value="camera_front"',
        'default_value="camera_rear"',
        'default_value="lidar_link"',
        'default_value="/sensing/lidar/vanjee/points_raw"',
        'default_value="false"',
        'DeclareLaunchArgument("jpeg_quality", default_value="80")',
        '"compressed_image_max_rate_hz",',
        '"raw_image_max_rate_hz",',
        'default_value="10.0"',
        'LaunchConfiguration("launch_image_compression")',
        'value_type=bool',
        'LaunchConfiguration("jpeg_quality"), value_type=int',
        'LaunchConfiguration("compressed_image_max_rate_hz")',
        'LaunchConfiguration("raw_image_max_rate_hz")',
        'value_type=float',
    ):
        assert expected in launch_source

    manifest = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>python3-numpy</exec_depend>" in manifest
    assert "<exec_depend>python3-opencv</exec_depend>" in manifest
    assert "<exec_depend>image_transport</exec_depend>" not in manifest

    relay_source = (
        PACKAGE_ROOT
        / "src"
        / "camrod_carla_adapter"
        / "sensor_relay_node.py"
    ).read_text(encoding="utf-8")
    assert "camera_input_qos = QoSProfile(" in relay_source
    assert "camera_output_qos = QoSProfile(" in relay_source
    assert "depth=2" in relay_source
    assert "depth=1" in relay_source
    assert "reliability=ReliabilityPolicy.RELIABLE" in relay_source
    assert "Image, topic, camera_output_qos" in relay_source
    assert (
        "CompressedImage, self.front_compressed_output, camera_output_qos"
        in relay_source
    )
    assert (
        "CompressedImage, self.rear_compressed_output, camera_output_qos"
        in relay_source
    )
    assert "Image, self.front_image_input, self._on_front_image," in relay_source
    assert "payload = np.frombuffer(message.data" in relay_source
    assert "def _publish_raw_image(" in relay_source
    assert "publisher.get_subscription_count() > 0" in relay_source
    assert "Image, self.rear_image_input, self._on_rear_image," in relay_source
    assert relay_source.count("camera_input_qos,") == 2
    assert "def encode_image_jpeg" in relay_source
    assert "publisher.get_subscription_count() <= 0" in relay_source
    assert "def active_stream_ages" in relay_source
    assert 'output.format = "jpeg"' in relay_source
    assert 'DiagnosticArray, "/diagnostics", 10' in relay_source
    assert "self._diagnostics_publisher.publish(array)" in relay_source
    assert "self._lidar_publisher = self.create_publisher(" in relay_source
    assert "self._lidar_publisher.publish(output)" in relay_source
    assert "lidar_filtered_output" not in relay_source
    assert "obstacle_cloud_output" not in relay_source


def test_carla_lidar_processing_owns_filtered_and_cluster_boundaries():
    processing = (
        PACKAGE_ROOT / "launch" / "carla_lidar_processing.launch.py"
    ).read_text(encoding="utf-8")
    filter_config = yaml.safe_load(
        (CONFIG_ROOT / "carla_lidar_filter.yaml").read_text(encoding="utf-8")
    )["/sensing/lidar/carla_lidar_filter"]["ros__parameters"]
    perception = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_perception"
            / "config"
            / "perception_params.yaml"
        ).read_text(encoding="utf-8")
    )["/perception/obstacle_lidar"]["ros__parameters"]

    assert 'executable="carla_lidar_filter"' in processing
    assert 'executable="obstacle_lidar_node"' in processing
    assert '"publish_cluster_cloud": True' in processing
    assert filter_config["input_topic"] == "/sensing/lidar/vanjee/points_raw"
    assert filter_config["output_topic"] == "/sensing/lidar/points_filtered"
    assert filter_config["expected_ground_z_m"] == pytest.approx(-0.59538)
    assert filter_config["minimum_obstacle_height_m"] == pytest.approx(0.08)
    assert filter_config["self_return_mask_enabled"] is True
    assert filter_config["self_return_x_min_m"] == pytest.approx(0.49)
    assert filter_config["self_return_x_max_m"] == pytest.approx(1.31)
    assert filter_config["self_return_abs_y_min_m"] == pytest.approx(0.59)
    assert filter_config["self_return_abs_y_max_m"] == pytest.approx(0.95)
    assert filter_config["self_return_z_min_m"] == pytest.approx(-0.57)
    assert filter_config["self_return_z_max_m"] == pytest.approx(0.18)
    filter_node = (
        PACKAGE_ROOT / "src" / "camrod_carla_adapter" / "lidar_filter_node.py"
    ).read_text(encoding="utf-8")
    assert 'key="self_return_points_removed"' in filter_node
    assert perception["publish_cluster_cloud"] is False
    assert perception["obstacle_cloud_topic"] == "/perception/obstacles"


def test_carla_diagnostics_accept_an_empty_filtered_clear_scene():
    """Keep sensor liveness strict while allowing zero detected obstacles."""
    key = "/system/lidar_checker"
    bringup = yaml.safe_load(
        CARLA_LIDAR_CHECKER.read_text(encoding="utf-8")
    )[key]["ros__parameters"]
    system = yaml.safe_load(
        SYSTEM_CARLA_LIDAR_CHECKER.read_text(encoding="utf-8")
    )[key]["ros__parameters"]

    assert bringup == system
    assert bringup["main"]["min_point_count"] == 1
    assert bringup["filtered"]["min_point_count"] == 0
    assert bringup["filtered"]["expected_hz"] > 0.0
    assert bringup["filtered"]["stale_timeout_s"] > 0.0


def test_virtual_carla_cyclonedds_config_reserves_both_socket_directions():
    root = ET.parse(REPO_ROOT / "cyclonedds.xml").getroot()
    minimums = {
        element.tag.rsplit("}", 1)[-1]: element.attrib.get("min")
        for element in root.iter()
        if element.tag.rsplit("}", 1)[-1]
        in {"SocketReceiveBufferSize", "SocketSendBufferSize"}
    }
    assert minimums == {
        "SocketReceiveBufferSize": "20MiB",
        "SocketSendBufferSize": "20MiB",
    }


def test_sensor_stream_preflight_requires_payloads_from_all_five_inputs():
    source = SENSOR_PREFLIGHT.read_text(encoding="utf-8")
    compile(source, str(SENSOR_PREFLIGHT), "exec")
    assert SENSOR_PREFLIGHT.stat().st_mode & 0o111
    for suffix in (
        "/rgb_view/image",
        "/rgb_view/camera_info",
        "/rgb_rear/image",
        "/rgb_rear/camera_info",
        "/lidar_front",
    ):
        assert suffix in source
    assert "qos_profile_sensor_data" in source
    assert "message.step <= 0" in source
    assert 'default=2.0' in source
    assert 'default=3.0' in source
    assert 'default=1.0' in source
    assert 'metric["stream_ready"]' in source
    assert "encode_image_jpeg(message, 80)" in source
