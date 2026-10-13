"""HH_261012 - Freeze CARLA return policy without changing service geometry."""

from pathlib import Path

import pytest
import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config"
BASELINE = CONFIG / "camping_sites_b1_lower_entry_experimental.yaml"
FORWARD = CONFIG / "camping_sites_carla_forward_return.yaml"
PROFILE_FIELDS = {
    "forward_return_centering_tolerance_m": 0.015,
    "forward_return_recenter_trigger_m": 0.035,
}
STEERING_SETTLE_FIELD = "forward_return_steering_settle_s"
RECENTERING_LIMIT_FIELD = "forward_return_recentering_limit"


class UniqueKeyLoader(yaml.SafeLoader):
    """HH_261012 - Fail on duplicate keys at every YAML mapping depth."""


def _unique_mapping(loader, node, deep=False):
    loader.flatten_mapping(node)
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping", node.start_mark,
                f"duplicate key: {key!r}", key_node.start_mark,
            )
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping,
)


def _read_catalog(path):
    return yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueKeyLoader)


def _without_return_policy(catalog):
    # HH_261012 - Compare every other field, including B1's existing service
    # override, exact corners, yaw, IDs, order, and any future catalog metadata.
    return {
        **catalog,
        "camping_sites": [
            {key: value for key, value in site.items()
             if key not in {"return_route_policy", *PROFILE_FIELDS,
                            STEERING_SETTLE_FIELD, RECENTERING_LIMIT_FIELD}}
            for site in catalog["camping_sites"]
        ],
    }


def test_forward_catalog_preserves_complete_existing_service_geometry():
    baseline = _read_catalog(BASELINE)
    forward = _read_catalog(FORWARD)
    assert _without_return_policy(forward) == _without_return_policy(baseline)


def test_forward_catalog_contains_each_original_site_exactly_once():
    sites = _read_catalog(FORWARD)["camping_sites"]
    assert len(sites) == 13
    assert {site["type"] for site in sites} == {
        f"camping_site_{number}" for number in range(1, 14)
    }
    assert len({site["id"] for site in sites}) == 13


@pytest.mark.parametrize("number", range(1, 14))
def test_only_b9_b10_have_the_bounded_centering_profile(number):
    # HH_261012 - No geometry, service mode, native row or other forward site
    # may inherit this measured candidate; limits remain in the controller.
    site = next(site for site in _read_catalog(FORWARD)["camping_sites"]
                if site["type"] == f"camping_site_{number}")
    actual = {key: site[key] for key in PROFILE_FIELDS if key in site}
    assert actual == (PROFILE_FIELDS if number in (9, 10) else {})
    if actual:
        assert 0 < actual["forward_return_centering_tolerance_m"] < (
            actual["forward_return_recenter_trigger_m"]) < 0.05


@pytest.mark.parametrize("number", range(1, 14))
def test_only_b10_opts_into_measured_departure_steering_dwell(number):
    # HH_261012 - A slope-specific recovery must never replace the native
    # 1.2 s dwell or leak to B9 and the other previously completed sites.
    site = next(site for site in _read_catalog(FORWARD)["camping_sites"]
                if site["type"] == f"camping_site_{number}")
    if number == 10:
        assert site[STEERING_SETTLE_FIELD] == 0.4
        assert 0.289 < site[STEERING_SETTLE_FIELD] < 1.2
        assert site["return_route_policy"] == "forward_loop"
    else:
        assert STEERING_SETTLE_FIELD not in site


@pytest.mark.parametrize("number", range(1, 14))
def test_only_b10_opts_into_bounded_additional_recovery_count(number):
    # HH_261012 - Preserve the ten-recovery default everywhere except this
    # explicit measured B10 profile; no time or position limit is enlarged.
    site = next(site for site in _read_catalog(FORWARD)["camping_sites"]
                if site["type"] == f"camping_site_{number}")
    if number == 10:
        assert type(site[RECENTERING_LIMIT_FIELD]) is int
        assert site[RECENTERING_LIMIT_FIELD] == 20
        assert site["return_route_policy"] == "forward_loop"
    else:
        assert RECENTERING_LIMIT_FIELD not in site


@pytest.mark.parametrize("number", range(1, 14))
def test_only_turnaround_b1_through_b10_opt_into_forward_loop(number):
    sites = _read_catalog(FORWARD)["camping_sites"]
    site = next(site for site in sites
                if site["type"] == f"camping_site_{number}")
    if number <= 10:
        assert site["service_mode"] == "turnaround"
        assert site["return_route_policy"] == "forward_loop"
    else:
        assert site["service_mode"] == "roadside_stop"
        assert site.get("return_route_policy", "native") == "native"


@pytest.mark.parametrize("document", [
    "camping_sites: []\ncamping_sites: []\n",
    "site:\n  return_route_policy: native\n  return_route_policy: forward_loop\n",
    "site:\n  corners:\n    - x: 1.0\n      x: 2.0\n",
])
def test_duplicate_key_guard_rejects_root_site_and_corner_duplicates(document):
    with pytest.raises(yaml.constructor.ConstructorError, match="duplicate key"):
        yaml.load(document, Loader=UniqueKeyLoader)
