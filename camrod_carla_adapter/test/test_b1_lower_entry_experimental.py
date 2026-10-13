"""HH_261007 - Offline opt-in B1 service target; never launch or drive CARLA."""

import hashlib
import math
from pathlib import Path
import sys

from lanelet2.io import Origin, load
from lanelet2.projection import LocalCartesianProjector
import pytest
from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
import yaml


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "camrod_planning/config/camping_sites.yaml"
CANDIDATE = ROOT / "camrod_carla_adapter/config/camping_sites_b1_lower_entry_experimental.yaml"
TARGET = (25.153470698312496, -5.542307115362121)
ANCHOR = (22.37485367477521, -6.942739330241457)
YAW_DEG = -63.251743982441376
sys.path.insert(0, str(ROOT / "camrod_bringup/scripts/visualization"))
from render_tapered_rounded_boundary import make_boundaries


@pytest.fixture(scope="module")
def catalogs():
    return tuple(yaml.safe_load(path.read_text()) for path in (SOURCE, CANDIDATE))


def test_original_catalog_and_map_remain_hash_bound():
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == (
        "a12ff6a2c6f730e6b0ef47a0850138c0567f03e4a90c4bd0be5bc049cfa6c07f")
    assert hashlib.sha256((ROOT / "lanelet2_maps.osm").read_bytes()).hexdigest() == (
        "cf5490cbf2067b4ce7dc703f80e75095366cacd364097cbad431131e7b435c8d")


def test_only_two_b1_service_fields_are_added(catalogs):
    original, candidate = catalogs
    assert set(original) == set(candidate) == {"camping_sites"}
    assert len(original["camping_sites"]) == len(candidate["camping_sites"]) == 13
    restored = yaml.safe_load(CANDIDATE.read_text())
    b1 = next(s for s in restored["camping_sites"] if s["type"] == "camping_site_1")
    assert (b1.pop("service_x"), b1.pop("service_y")) == TARGET
    assert restored == original


@pytest.mark.parametrize("site_number", range(2, 14))
def test_other_sites_are_identical(catalogs, site_number):
    original, candidate = catalogs
    select = lambda d: next(s for s in d["camping_sites"]
                            if s["type"] == f"camping_site_{site_number}")
    assert select(original) == select(candidate)


def test_target_is_finite_inside_exact_site_and_preserves_inward_depth(catalogs):
    b1 = catalogs[1]["camping_sites"][0]
    assert all(math.isfinite(value) for value in TARGET)
    corners = [(p["x"], p["y"]) for p in b1["corners"]]
    assert Polygon(corners).contains(Point(TARGET))
    dx, dy = corners[2][0]-corners[1][0], corners[2][1]-corners[1][1]
    length = math.hypot(dx, dy)
    shift = TARGET[0]-b1["x"], TARGET[1]-b1["y"]
    assert (shift[0]*dy-shift[1]*dx)/length == pytest.approx(0., abs=1e-10)
    assert (shift[0]*dx+shift[1]*dy)/length == pytest.approx(-0.740408688355739, abs=1e-9)
    assert b1["service_mode"] == "turnaround"
    assert catalogs[1]["camping_sites"][10]["service_mode"] == "roadside_stop"


def test_derived_anchor_and_full_margin_sweep(catalogs):
    # HH_261007 - Derive the anchor from original road geometry. No anchor
    # override is introduced; the existing live goal snapper remains authority.
    lane_map = load(str(ROOT / "lanelet2_maps.osm"),
                    LocalCartesianProjector(Origin(36.8435737, 128.0925646, 0)))
    line = LineString([(p.x, p.y) for p in lane_map.laneletLayer[2193].centerline])
    station = line.project(Point(TARGET))
    anchor = line.interpolate(station)
    assert (anchor.x, anchor.y) == pytest.approx(ANCHOR, abs=1e-9)
    before, after = (line.interpolate(station+d) for d in (-.001, .001))
    yaw = math.atan2(after.y-before.y, after.x-before.x)
    assert math.degrees(yaw) == pytest.approx(YAW_DEG, abs=1e-8)
    extents = dict(front=.70837, rear=.68323, left=.53505, right=.53495)
    contour = make_boundaries(extents, {key:.1 for key in extents}, .12, .12, .05, 4)[3]
    assert max(math.hypot(*p) for p in contour)+.075 == pytest.approx(1.021560482367391)

    def placed(center, angle):
        c, s = math.cos(angle), math.sin(angle)
        return Polygon([(center[0]+c*x-s*y, center[1]+s*x+c*y) for x,y in contour])

    centers = [(ANCHOR[0]+i/80*(TARGET[0]-ANCHOR[0]),
                ANCHOR[1]+i/80*(TARGET[1]-ANCHOR[1])) for i in range(81)]
    sweep = unary_union([placed(center, angle) for center in centers
                         for angle in (yaw, yaw+math.pi)] +
                        [placed(TARGET, yaw+math.radians(i)) for i in range(361)]).buffer(.075)
    b1 = catalogs[1]["camping_sites"][0]
    site = Polygon([(p["x"],p["y"]) for p in b1["corners"]])
    access = yaml.safe_load((ROOT / "camrod_carla_adapter/config/manual_service_access.yaml").read_text())
    entry = next(item for item in access["service_access"] if item["site"] == "camping_site_1")
    entrance = Polygon([(p["x"],p["y"]) for p in entry["corners"]])
    roads = [Polygon([(p.x,p.y) for p in lane_map.laneletLayer[i].polygon2d()])
             for i in (2193, 2219)]
    assert sweep.difference(unary_union(roads+[site, entrance])).area < 1e-10


def test_candidate_is_not_a_default_or_runtime_anchor_override():
    data = yaml.safe_load(CANDIDATE.read_text())
    assert set(data["camping_sites"][0])-set(yaml.safe_load(SOURCE.read_text())["camping_sites"][0]) == {
        "service_x", "service_y"}
    for path in (ROOT / "camrod_bringup/config/bringup/launch_defaults.yaml",
                 ROOT / "camrod_carla_adapter/launch/camrod_carla_full.launch.py",
                 ROOT / "camrod_carla_adapter/launch/camrod_carla_develop_site_geometry.launch.py"):
        assert CANDIDATE.name not in path.read_text()
