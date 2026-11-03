"""HH_261103 - Explicitly compose proven CARLA docking retries with separate test contact geometry."""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, LogInfo, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def service_test_arguments(adapter_share, test_share):
    # HH_261103 - Reuse the installed simulator profile, not copied absolute workspace launch paths.
    path = os.path.join(adapter_share, "launch", "camrod_carla_develop_site_geometry.launch.py")
    spec = importlib.util.spec_from_file_location("camrod_service_develop_site_geometry", path)
    profile = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(profile)
    arguments = profile.develop_site_geometry_arguments(adapter_share)
    arguments.update({
        "launch_charging_contact_emulator": "false",
        "carla_parking_runtime_override_param_file": os.path.join(test_share, "config", "parking_retry.yaml"),
    })
    return profile, arguments


def generate_launch_description():
    adapter = get_package_share_directory("camrod_carla_adapter")
    test = get_package_share_directory("camrod_carla_service_test")
    control = get_package_share_directory("camrod_control")
    profile, arguments = service_test_arguments(adapter, test)
    return LaunchDescription([
        SetLaunchConfiguration("yolo_labels_path", profile.develop_site_geometry_yolo_labels(adapter)),
        SetLaunchConfiguration("control_param_file", profile.develop_site_geometry_control_params(control)),
        # HH_261103 - Select only the test AprilTag binary; the dispatcher/reverse/cargo controllers stay native.
        SetLaunchConfiguration("apriltag_controller_package", "camrod_carla_service_test"),
        SetLaunchConfiguration("apriltag_controller_executable", "apriltag_parking_controller_node"),
        LogInfo(msg="CARLA service-test docking compatibility enabled; native station/cargo anchors unchanged"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            adapter, "launch", "camrod_carla_full.launch.py")), launch_arguments=arguments.items()),
        # HH_261103 - Scope the charging-only YAML so it cannot overwrite native parking/cargo launch arguments.
        GroupAction(scoped=True, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            adapter, "launch", "charging_contact_emulator.launch.py")), launch_arguments={
                "enable": "true",
                "drop_zones_yaml": os.path.join(test, "config", "charging_station.yaml"),
                "drop_zone_id": "drop_zone",
                "parking_status_topic": "/parking/status",
                "position_tolerance_m": "0.35",
                "speed_tolerance_mps": "0.05",
                "pose_timeout_s": "0.5",
                "odometry_timeout_s": "0.5",
                "state_timeout_s": "2.0",
                "dwell_s": "1.0",
            }.items())]),
    ])
