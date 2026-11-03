"""HH_261103 - Host-independent test geometry; scene asset provisioning remains an explicit separate prerequisite."""
import math

SOURCE_NAME = 'Woraksan_v27_service_tag_20261102'
TARGET_NAME = 'Woraksan_v27_charge_anchor_20261103'
STATION_XY = (-8.47366, 40.391)
BODY_YAW_DEG = 91.7873
SEPARATION_M = .60
MAP_YAW_RAD = 8.142811119961662e-7


def geometry():
    yaw = math.radians(BODY_YAW_DEG)
    dx, dy = -SEPARATION_M * math.cos(yaw), -SEPARATION_M * math.sin(yaw)
    c, s = math.cos(MAP_YAW_RAD), math.sin(MAP_YAW_RAD)
    return {'normal_station_ros_xy_m': STATION_XY,
            'charging_body_ros_xy_m': (STATION_XY[0] + dx, STATION_XY[1] + dy),
            'body_yaw_ros_deg': BODY_YAW_DEG, 'charging_behind_station_m': SEPARATION_M,
            'translation_ue_cm': (100 * (c * dx + s * dy), -100 * (-s * dx + c * dy), 0.0),
            'rotation_degrees': 0.0, 'preserve_absolute_tag_z': True,
            'cargo_position_tolerance_m': .20, 'unchanged_contact_radius_m': .35,
            'minimum_nominal_charger_gap_at_cargo_stop_m': SEPARATION_M - .20,
            'optical_depth_at_docked_body_m': .40}
