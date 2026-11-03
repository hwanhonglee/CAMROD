"""HH_261102 - Geometry and plan checks do not connect to CARLA/ROS or publish test sensor data."""
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS

import pytest

SOURCE = Path(__file__).with_name("sensor_probe.py")
SPEC = importlib.util.spec_from_file_location("sensor_probe", SOURCE)
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


def vector(x=0, y=0, z=0):
    return NS(x=x, y=y, z=z)


def rotation(angle=0):
    return NS(x=0, y=0, z=math.sin(angle/2), w=math.cos(angle/2))


def test_default_is_plan_only_without_simulator_imports():
    result = subprocess.run([sys.executable, str(SOURCE)], capture_output=True, text=True, check=True)
    document = json.loads(result.stdout)
    assert document["mode"] == "PLAN_ONLY_NO_CONNECTION"
    assert document["front_body_clearances_m"] == [.1, .2, .3, .4]
    assert document["baseline_seconds"] == 60
    assert document["fake_sensor_publish"] is False


@pytest.mark.parametrize("gap", [.1, .2, .3, .4])
def test_gap_is_between_body_faces_not_actor_origins(gap):
    center = probe.placement_forward(.76, gap, -.19)
    assert center-.19-.76 == pytest.approx(gap)


def test_projected_bounds_follow_ego_heading():
    vertices = [vector(10, 20, 1), vector(11, 22, 2)]
    bounds = probe.projected_bounds(vertices, vector(10, 20), vector(0, 1), vector(-1, 0))
    assert bounds == {"forward": [0, 2], "right": [-1, 0], "z": [1, 2]}


def test_actual_grid_near_body_and_roadside_are_distinguished():
    msg = NS(data=[95], info=NS(width=1, height=1, resolution=.05,
        origin=NS(position=vector(.8, -.025), orientation=rotation())))
    transform = NS(translation=vector(), rotation=rotation())
    assert probe.front_grid_metrics(msg, transform)["front_corridor_cells_ge85"] == 1
    msg.info.origin.position.y = 1
    assert probe.front_grid_metrics(msg, transform)["front_corridor_cells_ge85"] == 0


def test_invalid_grid_and_nonfinite_geometry_are_not_clear():
    with pytest.raises(ValueError):
        probe.placement_forward(.7, math.nan, -.2)
    with pytest.raises(ValueError):
        probe.front_grid_metrics(NS(data=[], info=NS(width=1, height=1)), None)
    assert probe.json_safe({"range": math.inf}) == {"range": "inf"}
    with pytest.raises(ValueError):
        probe.yaw(NS(x=0, y=0, z=0, w=0))


def test_probe_never_publishes_or_teleports_and_removal_is_owned():
    source = SOURCE.read_text()
    for forbidden in ("create_publisher(", ".set_transform(", ".set_location(", "world.tick(", ".apply_control("):
        assert forbidden not in source
    assert 'current.attributes.get("role_name") == role' in source
    assert 'comparison_kind": "SHADOW_GRID_OVERLAP_NOT_FINAL_GATE_ISOLATION"' in source


def test_clock_contract_uses_sim_only_for_raw_carla_sources():
    # HH_261102 - Large wall timestamps must not be aged against the much smaller CARLA simulation clock.
    wall = probe.source_timing("guard_grid", 1800000000, 1800000000.1, 120, .01)
    raw = probe.source_timing("raw_lidar", 119.9, 1800000000.1, 120, .01)
    assert wall["source_age_s"] == pytest.approx(.1)
    assert raw["source_age_s"] == pytest.approx(.1)
    assert wall["carla_clock_s"] == raw["carla_clock_s"] == 120
    assert wall["wall_clock_s"] == raw["wall_clock_s"] == 1800000000.1
    assert probe.source_timing("raw_radar_front1", 119.9, 1800000000, None, None)["source_age_s"] is None
    assert probe.source_timing("raw_lidar", 119.9, 1800000000, 120, .6)["source_age_s"] is None
    with pytest.raises(KeyError):
        probe.source_timing("undeclared", 120, 1800000000, 120, 0)


def test_paused_sim_clock_does_not_advance_source_age_by_wall_receipt_time():
    clock = probe.source_timing("raw_lidar", 119.9, 1800000060, 120, .01)
    assert clock["source_age_s"] == pytest.approx(.1)
    sample = {"value": {}, "clock_now": clock, "receipt_age_s": 60}
    assert probe.source_fresh(sample) is False
    sample["receipt_age_s"] = .1
    assert probe.source_fresh(sample) is True


def test_clock_diagnostic_defaults_to_plan_without_ros_or_carla():
    result = subprocess.run([sys.executable, str(SOURCE.with_name("sensor_probe_clock_check.py"))],
        capture_output=True, text=True, check=True)
    assert json.loads(result.stdout)["mode"] == "PLAN_ONLY_NO_CONNECTION"


def test_multihit_ground_requires_real_road_below_hull_not_a_car_or_roof():
    # HH_261102 - A Car hit has no actor ID in this API; preserve it and select only qualifying measured ground.
    hits = [NS(label="Car", location=vector(1, 2, .5)),
            NS(label="Roads", location=vector(1, 2, 3)),
            NS(label="Roads", location=vector(1, 2, -.01))]
    index, records = probe.screen_ground_hits(hits, 0)
    assert index == 2
    assert records[0]["label"] == "Car"
    assert records[0]["xyz"]["z"] == .5
    assert records[1]["eligible_ground"] is False
    assert probe.screen_ground_hits([NS(label="Buildings", location=vector(z=-.01))], 0)[0] is None
    assert probe.screen_ground_hits([NS(label="Terrain", location=vector(z=-1))], 0)[0] is None


def test_grid_raw_counts_survive_missing_tf_and_unknown_background_is_not_global_clear():
    message = NS(data=[-1, 95], info=NS(width=2, height=1, resolution=.05))
    raw = probe.raw_grid_metrics(message)
    assert raw["occupied_cells_ge85"] == 1
    assert raw["unknown_cells"] == 1
    sample = {"value": raw, "receipt_age_s": .1, "clock_now": {"source_age_s": .1}, "tf_error": "pending"}
    assert probe.shadow_grid_hit(sample) is None
    sample["value"]["front_corridor_cells_ge85"] = 1
    assert probe.shadow_grid_hit(sample) is True
    sample["value"]["front_corridor_cells_ge85"] = 0
    assert probe.shadow_grid_hit(sample) is False  # NO_HIT, not proof that unknown cells are free.
    sample["receipt_age_s"] = 1
    assert probe.shadow_grid_hit(sample) is None


def test_results_preserve_hit_no_hit_unavailable_and_measured_geometry(tmp_path):
    # HH_261102 - Derived reports must not turn missing TF into a successful clear or invent geometry.
    sys.modules.setdefault("sensor_probe", probe)
    spec = importlib.util.spec_from_file_location("sensor_probe_results", SOURCE.with_name("sensor_probe_results.py"))
    results = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(results)
    def grid(cells):
        return {"value": {"front_corridor_cells_ge85": cells, "occupied_cells_ge85": cells, "unknown_cells": 10},
                "clock_now": {"source_age_s": .1}, "receipt_age_s": .1}
    rows = [{"event": "start", "physical_bbox_projected_m": {"forward": [-.7, .76347]},
             "native_guard_body_m": {"front": .70837}},
            {"event": "sample", "case": "no_test_actor_ground_roadside_baseline", "utc": "2026-11-02T00:00:00+00:00",
             "carla_frame": 1, "sensors": {"radar_grid": grid(0), "guard_grid": grid(0)}},
            {"event": "measured_placement", "case": "front_10cm", "actor_id": 10,
             "utc": "2026-11-02T00:01:00+00:00", "measured_front_body_gap_m": .10001},
            {"event": "sample", "case": "front_10cm", "sensors": {"radar_grid": grid(0), "guard_grid": grid(2)}},
            {"event": "sample", "case": "front_10cm", "sensors": {"radar_grid": grid(0)}},
            {"event": "complete"}]
    source = tmp_path/"unit_fixture.jsonl"
    source.write_text("\n".join(json.dumps(row) for row in rows))
    report = results.summarize(source)
    case = report["case_results"][0]
    assert case["measured_body_gap_m"] == .10001
    assert case["modes"]["guard_only"] == {"HIT": 1, "NO_HIT": 0, "UNAVAILABLE": 1}
    assert case["modes"]["radar_only"] == {"HIT": 0, "NO_HIT": 2, "UNAVAILABLE": 0}
    assert report["physical_minus_native_front_m"] == pytest.approx(.0551)
