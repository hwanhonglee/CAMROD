"""Contracts for deterministic CARLA visual sensors and CAMROD UI topics."""

import json
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent
CONFIG_ROOT = PACKAGE_ROOT / "config"
CONTROL_SPAWN = CONFIG_ROOT / "ranger_spawn_camrod_control_only.json"
VISUAL_SPAWN = CONFIG_ROOT / "ranger_spawn_camrod_full_sensors.json"
SENSOR_PREFLIGHT = (
    REPO_ROOT / "scripts" / "virtual_carla" / "check_carla_sensor_streams.py"
)


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
        'default_value="/sensing/lidar/points_filtered"',
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
    assert "self._lidar_publishers = [" in relay_source
    assert "for publisher in self._lidar_publishers:" in relay_source
    assert '"obstacle_cloud_output", "/perception/obstacles"' in relay_source


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
