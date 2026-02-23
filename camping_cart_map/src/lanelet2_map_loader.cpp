#include "camping_cart_map/lanelet2_map_loader.hpp"

#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/LocalCartesian.h>

#include "camping_cart_map/custom_regulatory_elements.hpp"

namespace camping_cart {
namespace map {

Lanelet2MapLoader::Lanelet2MapLoader(const LoaderConfig & cfg)
: cfg_(cfg)
{
}

lanelet::LaneletMapPtr Lanelet2MapLoader::load(const std::string & map_path)
{
  map_.reset();

  // HH_260114 Configure origin (WGS84).
  lanelet::GPSPoint gps;
  gps.lat = cfg_.offset_lat;
  gps.lon = cfg_.offset_lon;
  gps.ele = cfg_.offset_alt;

  lanelet::Origin origin(gps);
  lanelet::projection::LocalCartesianProjector projector(origin);

  try {
    map_ = lanelet::load(map_path, projector);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Lanelet2MapLoader"),
      "Exception while loading Lanelet2 map from '%s': %s",
      map_path.c_str(), e.what());
    return nullptr;
  }

  if (!map_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Lanelet2MapLoader"),
      "lanelet::load returned nullptr for '%s'", map_path.c_str());
    return nullptr;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("Lanelet2MapLoader"),
    "Loaded Lanelet2 map: %s (origin: %.8f, %.8f, %.2f)",
    map_path.c_str(), cfg_.offset_lat, cfg_.offset_lon, cfg_.offset_alt);

  return map_;
}

std::string Lanelet2MapLoader::getMapStats() const
{
  if (!map_) {
    return "No map loaded";
  }

  std::ostringstream oss;
  oss << "Lanelet2 Map Stats\n";
  oss << "  Lanelets : " << map_->laneletLayer.size() << "\n";
  oss << "  Areas    : " << map_->areaLayer.size() << "\n";
  oss << "  Points   : " << map_->pointLayer.size() << "\n";
  oss << "  Lines    : " << map_->lineStringLayer.size() << "\n";
  return oss.str();
}

}  // namespace map
}  // namespace camping_cart
