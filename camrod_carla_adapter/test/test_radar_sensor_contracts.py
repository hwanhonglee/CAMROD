"""Static ownership and geometry contracts for seven CARLA radars."""

import json
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent
SPAWN = PACKAGE_ROOT / 'config' / 'ranger_spawn_camrod_full_sensors.json'
PARAMS = PACKAGE_ROOT / 'config' / 'radar_relay.yaml'

CHANNELS = ('front1', 'front2', 'left1', 'left2', 'right1', 'right2', 'rear')
MAX_RANGES = (1.5, 1.5, 0.8, 0.8, 0.8, 0.8, 0.5)
MOUNTS = {
    'front1': (0.62787, -0.11005, 0.33378, 0.0),
    'front2': (0.62787, 0.11005, 0.33378, 0.0),
    'left1': (0.38, 0.53, 0.29013, 90.0),
    'left2': (-0.38, 0.53, 0.29013, 90.0),
    'right1': (0.38, -0.53, 0.29013, -90.0),
    'right2': (-0.38, -0.53, 0.29013, -90.0),
    'rear': (-0.61733, 0.0, 0.33978, 180.0),
}


def _vehicle():
    document = json.loads(SPAWN.read_text(encoding='utf-8'))
    matches = [item for item in document['objects'] if item.get('id') == 'ego_vehicle']
    assert len(matches) == 1
    return matches[0]


def test_visual_spawn_has_exactly_seven_physical_mount_radars():
    radars = {
        sensor['id'].removeprefix('radar_'): sensor
        for sensor in _vehicle()['sensors']
        if sensor['type'] == 'sensor.other.radar'
    }
    assert tuple(radars) == CHANNELS
    for name, maximum in zip(CHANNELS, MAX_RANGES):
        sensor = radars[name]
        point = sensor['spawn_point']
        assert (
            point['x'], point['y'], point['z'], point['yaw']
        ) == pytest.approx(MOUNTS[name])
        assert sensor['sensor_tick'] == pytest.approx(0.1)
        assert sensor['range'] == pytest.approx(maximum)
        assert sensor['horizontal_fov'] == pytest.approx(15.0)
        assert sensor['points_per_second'] == 500


def test_spawn_mounts_are_ros_coordinates_matching_canonical_robot_params():
    """Do not pre-flip ROS +Y-left poses for CARLA's left-handed frame."""
    params = yaml.safe_load(
        (
            REPO_ROOT
            / 'camrod_sensor_kit'
            / 'config'
            / 'robot_params.yaml'
        ).read_text(encoding='utf-8')
    )['/**']['ros__parameters']
    sensors = {sensor['id']: sensor for sensor in _vehicle()['sensors']}

    for name in CHANNELS:
        spawn = sensors[f'radar_{name}']['spawn_point']
        canonical = params['radar'][name]
        assert (
            spawn['x'],
            spawn['y'],
            spawn['z'],
            spawn['roll'],
            spawn['pitch'],
            spawn['yaw'],
        ) == pytest.approx(
            (
                canonical['x'],
                canonical['y'],
                canonical['z'],
                canonical['roll'],
                canonical['pitch'],
                canonical['yaw'],
            )
        )

    gnss_spawn = sensors['gnss']['spawn_point']
    gnss = params['gnss']
    assert (
        gnss_spawn['x'],
        gnss_spawn['y'],
        gnss_spawn['z'],
        gnss_spawn['roll'],
        gnss_spawn['pitch'],
        gnss_spawn['yaw'],
    ) == pytest.approx(
        (
            gnss['x'],
            gnss['y'],
            gnss['z'],
            gnss['roll'],
            gnss['pitch'],
            gnss['yaw'],
        )
    )

    # These sign assertions catch the exact left/right inversion that occurs
    # when JSON poses are incorrectly authored as CARLA coordinates. The spawn
    # service accepts ROS coordinates and performs this conversion itself.
    assert sensors['radar_front1']['spawn_point']['y'] < 0.0
    assert sensors['radar_front2']['spawn_point']['y'] > 0.0
    for name in ('left1', 'left2'):
        assert sensors[f'radar_{name}']['spawn_point']['y'] > 0.0
        assert sensors[f'radar_{name}']['spawn_point']['yaw'] == 90.0
    for name in ('right1', 'right2'):
        assert sensors[f'radar_{name}']['spawn_point']['y'] < 0.0
        assert sensors[f'radar_{name}']['spawn_point']['yaw'] == -90.0
    assert gnss_spawn['y'] > 0.0

    gnss_right_spawn = sensors['gnss_right']['spawn_point']
    assert (
        gnss_right_spawn['x'],
        gnss_right_spawn['y'],
        gnss_right_spawn['z'],
        gnss_right_spawn['roll'],
        gnss_right_spawn['pitch'],
        gnss_right_spawn['yaw'],
    ) == pytest.approx((gnss['x'], -gnss['y'], gnss['z'], 0.0, 0.0, 0.0))
    assert gnss_right_spawn['y'] < 0.0


def test_relay_config_owns_all_canonical_and_standard_topics_once():
    params = yaml.safe_load(PARAMS.read_text(encoding='utf-8'))[
        '/carla_radar_relay'
    ]['ros__parameters']
    assert tuple(params['channel_names']) == CHANNELS
    assert tuple(params['max_ranges_m']) == MAX_RANGES
    assert params['output_topics'] == [
        f'/sensing/radar/{name}/range' for name in CHANNELS
    ]
    assert params['standard_output_topics'] == [
        f'/sensing/radar/{name}/range_ros' for name in CHANNELS
    ]
    assert params['frame_ids'] == [f'radar_{name}_link' for name in CHANNELS]


def test_carla_full_launch_disables_fake_radar_and_starts_real_relay():
    full = (
        PACKAGE_ROOT / 'launch' / 'camrod_carla_full.launch.py'
    ).read_text(encoding='utf-8')
    fake_launch = (
        REPO_ROOT / 'camrod_bringup' / 'launch' / 'fake_sensors.launch.py'
    ).read_text(encoding='utf-8')
    bringup = (
        REPO_ROOT / 'camrod_bringup' / 'launch' / '_bringup_impl.py'
    ).read_text(encoding='utf-8')

    assert 'DeclareLaunchArgument("launch_radar_relay", default_value="true")' in full
    assert '"sim_publish_fake_radar_ranges": "false"' in full
    assert 'radar_relay.launch.py' in full
    assert "'publish_fake_radar_ranges'" in fake_launch
    assert 'value_type=bool' in fake_launch
    assert "'sim_publish_fake_radar_ranges'" in bringup


def test_package_declares_bridge_message_dependencies_and_entry_point():
    manifest = (PACKAGE_ROOT / 'package.xml').read_text(encoding='utf-8')
    setup = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    relay = (
        PACKAGE_ROOT / 'src' / 'camrod_carla_adapter' / 'radar_relay_node.py'
    ).read_text(encoding='utf-8')
    assert '<exec_depend>avg_msgs</exec_depend>' in manifest
    assert '<exec_depend>sensor_msgs_py</exec_depend>' in manifest
    assert 'carla_radar_relay = ' in setup
    assert 'PointCloud2' in relay
    assert "names.get('range')" in relay
