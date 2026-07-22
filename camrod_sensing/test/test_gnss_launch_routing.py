"""Regression tests for CAMROD GNSS correction routing."""

import importlib.util
from pathlib import Path


# HH_260722 - Load the launch file by path because its dotted filename is not a
# normal Python module name.
_GNSS_LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "gnss.launch.py"
_SPEC = importlib.util.spec_from_file_location("camrod_gnss_launch", _GNSS_LAUNCH_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_GNSS_LAUNCH = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_GNSS_LAUNCH)


def test_single_antenna_keeps_existing_ntrip_forwarding():
    """Single antenna keeps publisher and receiver on the requested topic."""
    ntrip_topic, ublox_topic = _GNSS_LAUNCH._resolve_rtcm_topics(
        "rtcm", dual_antenna=False, forward_ntrip_to_rover=False
    )
    assert ntrip_topic == "rtcm"
    assert ublox_topic == "rtcm"


def test_dual_heading_only_keeps_ntrip_separate_and_usb_rtcm_off():
    """The compatibility toggle preserves the old heading-only bench mode."""
    ntrip_topic, ublox_topic = _GNSS_LAUNCH._resolve_rtcm_topics(
        "rtcm", dual_antenna=True, forward_ntrip_to_rover=False
    )
    params = _GNSS_LAUNCH._dual_antenna_runtime_params(False)
    assert ntrip_topic == "ntrip_client/rtcm"
    assert ublox_topic == "rtcm"
    assert params["dual_antenna.usb_rtcm_in"] is False


def test_dual_field_mode_connects_ntrip_and_enables_usb_rtcm():
    """Field mode feeds CORS RTCM to USB while retaining the 4072 ROS filter."""
    ntrip_topic, ublox_topic = _GNSS_LAUNCH._resolve_rtcm_topics(
        "rtcm", dual_antenna=True, forward_ntrip_to_rover=True
    )
    params = _GNSS_LAUNCH._dual_antenna_runtime_params(True)
    assert ntrip_topic == "ntrip_client/rtcm"
    assert ublox_topic == ntrip_topic
    assert params["dual_antenna.usb_rtcm_in"] is True
    assert params["dual_antenna.configure_usb"] is True
    assert params["dual_antenna.configure_navigation"] is False
    assert params["dual_antenna.warm_start_on_startup"] is False
    assert params["dual_antenna.block_rtcm_ids"] == [4072]
