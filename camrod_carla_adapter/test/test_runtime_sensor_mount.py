"""HH_260911 - Sensor calibration must not alter shared algorithm parameters."""
import copy
import pytest
from camrod_carla_adapter.runtime_sensor_mount import calibrate_gnss_mount


def test_only_gnss_extrinsics_change_and_inputs_remain_immutable():
    inputs={'/**':{'ros__parameters':{'gnss_antenna_offset_x_m':0.65,'gnss_antenna_offset_y_m':0.45,'enable_gnss_lever_arm_correction':True,'max_position_jump_m':2.0}}}
    robots={'/**':{'ros__parameters':{'gnss':{'x':0.65,'y':0.45,'z':0.0},'robot':{'length':1.3916}}}}
    spawn={'objects':[{'id':'ego_vehicle','sensors':[{'id':'gnss','spawn_point':{'x':0.0,'y':0.45}}]}]}
    old=copy.deepcopy((inputs,robots,spawn))
    actual_inputs,actual_robot=calibrate_gnss_mount(inputs,robots,spawn)
    expected=copy.deepcopy(inputs); expected['/**']['ros__parameters']['gnss_antenna_offset_x_m']=0.0
    expected_robot=copy.deepcopy(robots); expected_robot['/**']['ros__parameters']['gnss']['x']=0.0
    assert actual_inputs==expected and actual_robot==expected_robot
    assert (inputs,robots,spawn)==old

@pytest.mark.parametrize('x',[float('nan'),float('inf'),6.0])
def test_invalid_mount_is_rejected_instead_of_inventing_a_pose(x):
    spawn={'objects':[{'id':'ego_vehicle','sensors':[{'id':'gnss','spawn_point':{'x':x,'y':0.45}}]}]}
    with pytest.raises(ValueError): calibrate_gnss_mount({}, {}, spawn)


def test_missing_sensor_is_rejected():
    with pytest.raises(ValueError): calibrate_gnss_mount({}, {}, {'objects':[{'id':'ego_vehicle','sensors':[]}]})
