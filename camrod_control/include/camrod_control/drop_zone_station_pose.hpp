#pragma once

// HH_260721 - Load one semantic drop-zone station contract for maneuver and parking nodes.

#include <cmath>
#include <cstddef>
#include <exception>
#include <string>

#include <yaml-cpp/yaml.h>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace camrod_control
{

struct DropZoneStationPose
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

inline DropZoneStationPose loadDropZoneStationPose(
  const rclcpp::Logger & logger,
  const std::string & yaml_path,
  const std::string & drop_zone_id,
  const bool use_yaml,
  const DropZoneStationPose & fallback)
{
  if (!use_yaml || yaml_path.empty()) {
    return fallback;
  }
  try {
    const YAML::Node document = YAML::LoadFile(yaml_path);
    const YAML::Node zones = document["drop_zones"];
    if (!zones || !zones.IsSequence() || zones.size() == 0U) {
      return fallback;
    }
    YAML::Node selected = zones[0];
    for (std::size_t index = 0; index < zones.size(); ++index) {
      const YAML::Node zone = zones[index];
      const std::string id = zone["id"] ? zone["id"].as<std::string>() : "";
      const std::string type = zone["type"] ? zone["type"].as<std::string>() : "";
      if (id == drop_zone_id || type == drop_zone_id) {
        selected = zone;
        break;
      }
    }
    return DropZoneStationPose{
      selected["x"] ? selected["x"].as<double>() : fallback.x_m,
      selected["y"] ? selected["y"].as<double>() : fallback.y_m,
      (selected["yaw_deg"] ? selected["yaw_deg"].as<double>() :
      fallback.yaw_rad * 180.0 / M_PI) * M_PI / 180.0};
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      logger, "failed to read drop_zones_yaml=%s: %s",
      yaml_path.c_str(), error.what());
    return fallback;
  }
}

}  // namespace camrod_control
