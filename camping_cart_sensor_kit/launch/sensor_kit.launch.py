from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Any, Tuple

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_params(params_path: str) -> Dict[str, Any]:
  with open(params_path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
  return data.get("/**", {}).get("ros__parameters", {})


def _sensor_pose(sensor_cfg: Dict[str, Any]) -> Tuple[str, str]:
  xyz = [
    float(sensor_cfg.get("x", 0.0)),
    float(sensor_cfg.get("y", 0.0)),
    float(sensor_cfg.get("z", 0.0)),
  ]
  rpy = [
    math.radians(float(sensor_cfg.get("roll", 0.0))),
    math.radians(float(sensor_cfg.get("pitch", 0.0))),
    math.radians(float(sensor_cfg.get("yaw", 0.0))),
  ]
  return (
    " ".join(f"{value:.6f}" for value in xyz),
    " ".join(f"{value:.6f}" for value in rpy),
  )


def _launch_setup(context, *args, **kwargs):
  # HH_260109 Sensor kit launch (URDF/TF) renamed package.
  pkg_share = Path(get_package_share_directory("camping_cart_sensor_kit"))
  params_file = LaunchConfiguration("params_file").perform(context)
  base_frame = LaunchConfiguration("base_frame_id").perform(context)
  sensor_kit_base_frame = LaunchConfiguration("sensor_kit_base_frame_id").perform(context)
  map_frame = LaunchConfiguration("map_frame_id").perform(context)
  params = _load_params(params_file)
  robot_cfg = params.get("robot", {})
  base_pose_cfg = params.get("base_pose", {})

  sensors = {}
  # HH_260220: Keep only active sensor frames under sensor_kit_base_link.
  for name in ["imu", "gnss", "lidar", "camera_front"]:
    sensors[name] = _sensor_pose(params.get(name, {}))

  base_length = float(robot_cfg.get("length", 1.4))
  base_width = float(robot_cfg.get("width", 0.7))
  base_height = float(robot_cfg.get("height", 1.2))

  xacro_file = PathJoinSubstitution([str(pkg_share), "urdf", "camping_cart_sensor_kit.xacro"])
  command_args = [
    "xacro ",
    xacro_file,
    " base_link:=",
    base_frame,
    " sensor_kit_base_link:=",
    sensor_kit_base_frame,
    " base_length:=",
    f"{base_length}",
    " base_width:=",
    f"{base_width}",
    " base_height:=",
    f"{base_height}",
  ]
  for sensor_name, (xyz_str, rpy_str) in sensors.items():
    command_args.extend([
      " ",
      f"{sensor_name}_xyz:=\"",
      xyz_str,
      "\"",
      " ",
      f"{sensor_name}_rpy:=\"",
      rpy_str,
      "\"",
    ])

  robot_description = ParameterValue(Command(command_args), value_type=str)

  # HH_260112 Namespace sensor kit nodes under /sensor_kit with short names.
  rsp_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    namespace="sensor_kit",
    parameters=[{"robot_description": robot_description}],
    output="screen",
  )

  # HH_260224 Visualization moved to camping_cart_platform/map packages.
  _ = map_frame
  _ = base_pose_cfg
  return [rsp_node]


def generate_launch_description():
  # 2026-02-02 10:32: Use centralized bringup config (synced from package config).
  default_params = Path(
    get_package_share_directory("camping_cart_bringup"),
    "config",
    "sensor_kit",
    "robot_params.yaml",
  )
  return LaunchDescription([
    DeclareLaunchArgument(
      "params_file",
      default_value=str(default_params),
      description="Robot geometry & sensor parameter file",
    ),
    DeclareLaunchArgument(
      "base_frame_id",
      default_value="robot_base_link",
      description="Base frame used for sensor kit URDF",
    ),
    DeclareLaunchArgument(
      "sensor_kit_base_frame_id",
      default_value="sensor_kit_base_link",
      description="Sensor kit frame under robot_base_link",
    ),
    DeclareLaunchArgument(
      "map_frame_id",
      default_value="map",
      description="Fixed world frame connected to the base frame",
    ),
    OpaqueFunction(function=_launch_setup),
  ])
