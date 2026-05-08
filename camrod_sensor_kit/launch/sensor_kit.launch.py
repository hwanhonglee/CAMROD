from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Any, Tuple, Sequence

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_params(params_path: str) -> Dict[str, Any]:
  # Loads root ROS parameter dictionary from a YAML file.
  with open(params_path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
  return data.get("/**", {}).get("ros__parameters", {})


def _sensor_pose(sensor_cfg: Dict[str, Any]) -> Tuple[str, str]:
  # Converts pose fields into xacro-ready xyz/rpy strings.
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


def _nested_sensor_cfg(root: Dict[str, Any], path: Sequence[str]) -> Dict[str, Any]:
  # Safely resolves nested dictionaries while tolerating missing keys.
  value: Any = root
  for key in path:
    if not isinstance(value, dict) or key not in value:
      return {}
    value = value[key]
  return value if isinstance(value, dict) else {}


def _sensor_cfg_compat(root: Dict[str, Any], nested_path: Sequence[str], flat_key: str) -> Dict[str, Any]:
  # HH_260326: Support both nested keys and legacy flat keys.
  nested = _nested_sensor_cfg(root, nested_path)
  if nested:
    return nested
  flat = root.get(flat_key, {})
  return flat if isinstance(flat, dict) else {}


def _launch_setup(context, *args, **kwargs):
  # Expands sensor parameters into xacro arguments and builds runtime nodes.
  pkg_share = Path(get_package_share_directory("camrod_sensor_kit"))
  params_file = LaunchConfiguration("params_file").perform(context)
  module_namespace = LaunchConfiguration("module_namespace").perform(context)
  base_frame = LaunchConfiguration("base_frame_id").perform(context)
  sensor_kit_base_frame = LaunchConfiguration("sensor_kit_base_frame_id").perform(context)

  params = _load_params(params_file)
  robot_cfg = params.get("robot", {})

  sensors = {}

  # ---------------------------------------------------------
  # 1. Flat sensors
  # ---------------------------------------------------------
  for name in ["imu", "gnss", "lidar", "camera"]:
    sensors[name] = _sensor_pose(params.get(name, {}))

  # ---------------------------------------------------------
  # 2. Nested radar sensors
  # ---------------------------------------------------------
  # HH_260507: Radar sensors are directly attached to sensor_kit_base_link.
  # There is no intermediate radar base frame anymore.
  for radar_name in ["front", "left1", "left2", "right1", "right2", "rear"]:
    key = f"radar_{radar_name}"
    sensors[key] = _sensor_pose(_sensor_cfg_compat(params, ("radar", radar_name), key))

  base_length = float(robot_cfg.get("length", 1.4))
  base_width = float(robot_cfg.get("width", 0.7))
  base_height = float(robot_cfg.get("height", 1.2))

  xacro_file = PathJoinSubstitution([str(pkg_share), "urdf", "camrod_sensor_kit.xacro"])
  command_args = [
    "xacro ",
    xacro_file,
    " robot_base_link:=",
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

  rsp_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    namespace=module_namespace,
    parameters=[{"robot_description": robot_description}],
    output="screen",
  )

  # HH_260326: Removed sensor_kit status runtime node as requested.
  return [rsp_node]


def generate_launch_description():
  # Declares launch arguments and defers runtime node creation to `_launch_setup`.
  default_params = Path(
    get_package_share_directory("camrod_sensor_kit"),
    "config",
    "robot_params.yaml",
  )
  return LaunchDescription([
    DeclareLaunchArgument(
      "params_file",
      default_value=str(default_params),
      description="Robot geometry & sensor parameter file",
    ),
    DeclareLaunchArgument(
      "module_namespace",
      default_value="sensor_kit",
      description="Namespace for sensor_kit module nodes",
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
    DeclareLaunchArgument(
      "enable_status",
      default_value="false",
      description="Deprecated (status node removed)",
    ),
    OpaqueFunction(function=_launch_setup),
  ])
