"""HH_261103 - Measure forward motion in the initial body frame, not map Y by assumption."""
import math
import pytest
from run_cargo_exit import displacement


@pytest.mark.parametrize('yaw', [0, math.pi / 2, math.pi, -math.pi / 2])
def test_forward_projection_and_wrapped_heading(yaw):
    start = {'x': -8.47366 - .6 * math.cos(yaw), 'y': 40.391 - .6 * math.sin(yaw), 'yaw': yaw}
    end = {'x': -8.47366, 'y': 40.391, 'yaw': yaw + 2 * math.pi}
    delta = displacement(start, end)
    assert delta['forward_m'] == pytest.approx(.6)
    assert delta['lateral_m'] == pytest.approx(0)
    assert delta['yaw_change_rad'] == pytest.approx(0)
    assert delta['station_error_m'] == 0
