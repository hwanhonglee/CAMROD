"""Contracts for CARLA-only u-blox-shaped UI telemetry conversion."""

import math

from camrod_carla_adapter.gnss_compat import (
    build_navcov,
    build_navpvt,
    build_relposned,
    EARTH_SEMIMAJOR_AXIS_M,
)
import pytest
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ublox_msgs.msg import NavPVT, NavRELPOSNED9


def _fix(latitude=37.0, longitude=127.0, altitude=100.0):
    message = NavSatFix()
    message.header.stamp.sec = 1700000000
    message.header.frame_id = 'gnss_link'
    message.status.status = NavSatStatus.STATUS_FIX
    message.latitude = latitude
    message.longitude = longitude
    message.altitude = altitude
    message.position_covariance = [
        0.01, 0.0, 0.0,
        0.0, 0.04, 0.0,
        0.0, 0.0, 0.09,
    ]
    message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return message


def test_navpvt_and_navcov_preserve_actual_fix_without_claiming_rtk():
    fix = _fix()
    navpvt = build_navpvt(
        fix,
        yaw_rad=math.radians(30.0),
        velocity_ned_mps=(1.0, 2.0, 0.0),
        heading_accuracy_rad=math.radians(2.0),
    )
    navcov = build_navcov(fix, velocity_covariance=(0.1, 0.2, 0.3))

    assert navpvt.fix_type == NavPVT.FIX_TYPE_3D
    assert navpvt.flags & NavPVT.FLAGS_GNSS_FIX_OK
    assert navpvt.flags & NavPVT.FLAGS_HEAD_VEH_VALID
    assert navpvt.flags & NavPVT.FLAGS_CARRIER_PHASE_MASK == 0
    assert navpvt.num_sv == 0
    assert navpvt.lat == 370000000
    assert navpvt.lon == 1270000000
    assert navpvt.g_speed == round(math.hypot(1.0, 2.0) * 1000.0)
    assert navpvt.head_veh == 3000000
    assert navcov.pos_cov_valid == 1
    assert navcov.pos_cov_nn == 0.04
    assert navcov.pos_cov_ee == 0.01
    assert navcov.pos_cov_dd == 0.09
    assert navcov.vel_cov_nn == pytest.approx(0.1)
    assert navcov.vel_cov_ee == pytest.approx(0.2)
    assert navcov.vel_cov_dd == pytest.approx(0.3)


def test_dual_actual_gnss_baseline_yields_ros_vehicle_heading_and_no_rtk_claim():
    right = _fix()
    # For a vehicle pointing along ROS +X/east, right->left is +Y/north.
    left = _fix(
        latitude=right.latitude + math.degrees(0.9 / EARTH_SEMIMAJOR_AXIS_M)
    )
    relpos = build_relposned(
        right,
        left,
        position_std_m=0.01,
        moving=True,
    )

    baseline_m = (
        relpos.rel_pos_length + relpos.rel_pos_hp_length * 0.01
    ) * 0.01
    assert baseline_m == pytest.approx(0.9, abs=0.001)
    assert relpos.rel_pos_heading == pytest.approx(0.0, abs=2)
    assert relpos.flags & NavRELPOSNED9.FLAGS_GNSS_FIX_OK
    assert relpos.flags & NavRELPOSNED9.FLAGS_REL_POS_VALID
    assert relpos.flags & NavRELPOSNED9.FLAGS_REL_POS_HEAD_VALID
    assert relpos.flags & NavRELPOSNED9.FLAGS_IS_MOVING
    assert relpos.flags & NavRELPOSNED9.FLAGS_CARR_SOLN_MASK == 0


def test_zero_baseline_is_rejected_instead_of_faking_heading():
    fix = _fix()
    try:
        build_relposned(fix, fix, position_std_m=0.1, moving=False)
    except ValueError as error:
        assert 'baseline' in str(error)
    else:
        raise AssertionError('zero physical baseline was accepted')
