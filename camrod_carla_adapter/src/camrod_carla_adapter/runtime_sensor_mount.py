"""Keep simulator sensor extrinsics separate from the production algorithm."""
import copy
import json
import math
from pathlib import Path
import tempfile
import yaml


def calibrate_gnss_mount(input_params, robot_params, spawn, role_name="ego_vehicle"):
    # HH_260911 - Use the authored CARLA sensor mount, not the field GNSS lever arm.
    vehicles = [v for v in spawn.get("objects", []) if v.get("id") == role_name]
    if len(vehicles) != 1:
        raise ValueError("Expected exactly one configured CARLA vehicle")
    sensors = [s for s in vehicles[0].get("sensors", []) if s.get("id") == "gnss"]
    if len(sensors) != 1:
        raise ValueError("Expected exactly one configured left GNSS sensor")
    mount = sensors[0]["spawn_point"]
    x, y = float(mount["x"]), float(mount["y"])
    if not all(math.isfinite(v) and abs(v) <= 5.0 for v in (x,y)):
        raise ValueError("Invalid GNSS sensor mount")
    localization, robot = copy.deepcopy(input_params), copy.deepcopy(robot_params)
    localization["/**"]["ros__parameters"]["gnss_antenna_offset_x_m"] = x
    localization["/**"]["ros__parameters"]["gnss_antenna_offset_y_m"] = y
    robot["/**"]["ros__parameters"]["gnss"].update(x=x,y=y)
    return localization, robot


def materialize_sensor_mount(input_path, robot_path, defaults_path, spawn_path, role_name="ego_vehicle"):
    # HH_260911 - Generate launch-local copies; never rewrite shared hardware YAML.
    with open(input_path, encoding="utf-8") as f: input_data = yaml.safe_load(f)
    with open(robot_path, encoding="utf-8") as f: robot_data = yaml.safe_load(f)
    with open(defaults_path, encoding="utf-8") as f: defaults = yaml.safe_load(f)
    with open(spawn_path, encoding="utf-8") as f: spawn = json.load(f)
    localization, robot = calibrate_gnss_mount(input_data, robot_data, spawn, role_name)
    output = Path(tempfile.mkdtemp(prefix="camrod-carla-sensor-mount-"))
    inputs, robots, launch = output/"input_adapter.yaml", output/"robot_params.yaml", output/"launch_defaults.yaml"
    defaults = copy.deepcopy(defaults)
    defaults.setdefault("bringup", {}).setdefault("platform", {})["params_file"] = str(robots)
    for path, data in ((inputs, localization), (robots, robot), (launch, defaults)):
        path.write_text("# HH_260911 - CARLA sensor-mount calibration; algorithm parameters unchanged.\n" + yaml.safe_dump(data, sort_keys=False))
    (output/"provenance.json").write_text(json.dumps({"input_source":str(input_path),
        "robot_source":str(robot_path), "defaults_source":str(defaults_path),
        "spawn_source":str(spawn_path), "gnss_mount":robot["/**"]["ros__parameters"]["gnss"]}, indent=2))
    return str(inputs), str(launch)
