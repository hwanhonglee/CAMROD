"""Contracts for deterministic CARLA visual sensors and CAMROD UI topics."""

import json
from pathlib import Path

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
        'default_value="false"',
        'DeclareLaunchArgument("jpeg_quality", default_value="80")',
        'LaunchConfiguration("launch_image_compression")',
        'value_type=bool',
        'LaunchConfiguration("jpeg_quality"), value_type=int',
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
    assert "self.create_publisher(Image, topic, 10)" in relay_source
    assert "CompressedImage, self.front_compressed_output, 10" in relay_source
    assert "CompressedImage, self.rear_compressed_output, 10" in relay_source
    assert "def encode_image_jpeg" in relay_source
    assert 'output.format = "jpeg"' in relay_source
    assert 'DiagnosticArray, "/diagnostics", 10' in relay_source
    assert "self._diagnostics_publisher.publish(array)" in relay_source


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
    assert 'default=8.0' in source
    assert 'default=1.0' in source
    assert 'default=0.5' in source
    assert 'metric["stream_ready"]' in source
    assert "encode_image_jpeg(message, 80)" in source
