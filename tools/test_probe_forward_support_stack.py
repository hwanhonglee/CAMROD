"""HH_261007 - Offline tests; no CARLA connection or commands."""
import pytest
from probe_forward_support_stack import evaluate_support_heights


def test_lower_ground_cannot_hide_obstructing_upper_support():
    actual = evaluate_support_heights([2.579221, 2.274393], 2.27439)
    assert actual['expected_support_present']
    assert not actual['clear_suspension_band']
    assert actual['nearby_upper_collision_z_m'] == [2.579221]


def test_missing_floor_is_not_cleared_by_removing_everything():
    assert not evaluate_support_heights([], 2.27439)['clear_suspension_band']
    assert not evaluate_support_heights([2.579221], 2.27439)['clear_suspension_band']


def test_single_floor_and_same_height_seam_remain_valid_for_narrow_check():
    assert evaluate_support_heights([2.27439], 2.27439)['clear_suspension_band']
    assert evaluate_support_heights([2.27439, 2.275], 2.27439)['clear_suspension_band']


@pytest.mark.parametrize('heights,z', [([float('nan')], 2), ([2], float('inf'))])
def test_invalid_measurements_fail(heights, z):
    with pytest.raises(ValueError):
        evaluate_support_heights(heights, z)
