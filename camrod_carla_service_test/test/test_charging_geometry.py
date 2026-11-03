"""HH_261103 - Separate real measured test contact geometry from the unchanged native cargo anchor."""
from dataclasses import replace
import importlib
from pathlib import Path
import sys

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'tools'))
import charging_geometry as stage


def emulator():
    return importlib.import_module('camrod_carla_adapter.charging_contact_emulator_node')


def test_config_separates_only_test_contact_anchor():
    g = stage.geometry()
    contact = yaml.safe_load((PACKAGE / 'config/charging_station.yaml').read_text())['drop_zones'][0]
    paths = [Path(get_package_share_directory('camrod_map')) / 'config/drop_zones.yaml',
             PACKAGE.parent / 'camrod_map/config/drop_zones.yaml']
    for path in paths:
        normal = yaml.safe_load(path.read_text())['drop_zones'][0]
        assert (normal['x'], normal['y']) == pytest.approx(g['normal_station_ros_xy_m'])
    assert (contact['x'], contact['y']) == pytest.approx(g['charging_body_ros_xy_m'])
    assert g['charging_behind_station_m'] == .60
    assert g['minimum_nominal_charger_gap_at_cargo_stop_m'] > g['unchanged_contact_radius_m']
    assert g['rotation_degrees'] == 0 and g['preserve_absolute_tag_z']
    assert stage.SOURCE_NAME != stage.TARGET_NAME


def test_existing_contact_policy_asserts_at_charger_but_not_at_normal_station():
    m = emulator()
    anchor = m.load_charging_station(str(PACKAGE / 'config/charging_station.yaml'))
    sample = m.ContactSample(parking_state='WAITING_FOR_CHARGING', x_m=anchor.x_m,
            y_m=anchor.y_m, speed_mps=0.0, pose_age_s=.01, odometry_age_s=.01,
            parking_status_age_s=.01, parking_method='apriltag')
    assert m.ContactConfig().position_tolerance_m == .35
    assert m.ContactConfig().dwell_s == 1.0
    assert m.contact_candidate(sample, anchor)
    assert not m.contact_candidate(replace(sample, x_m=-8.47366, y_m=40.391), anchor)
    for change in [dict(parking_method='reverse'), dict(parking_state='ERROR'),
                   dict(parking_state='IDLE'), dict(speed_mps=.051),
                   dict(pose_age_s=.501), dict(odometry_age_s=.501),
                   dict(parking_status_age_s=2.001)]:
        assert not m.contact_candidate(replace(sample, **change), anchor)


def test_scene_translation_matches_body_axis_without_vertical_or_yaw_mutation():
    import math
    g = stage.geometry()
    dx, dy, dz = g['translation_ue_cm']
    assert math.hypot(dx, dy) == pytest.approx(60.0)
    assert dz == 0
    x, y = g['charging_body_ros_xy_m']
    sx, sy = g['normal_station_ros_xy_m']
    yaw = math.radians(g['body_yaw_ros_deg'])
    assert (sx - x) * math.cos(yaw) + (sy - y) * math.sin(yaw) == pytest.approx(.6)
    assert -(sx - x) * math.sin(yaw) + (sy - y) * math.cos(yaw) == pytest.approx(0, abs=1e-12)
