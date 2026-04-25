#ifndef CAMROD_MAP__LANELET2_MAP_LOADER_HPP_
#define CAMROD_MAP__LANELET2_MAP_LOADER_HPP_

#include <memory>
#include <string>

#include <lanelet2_core/LaneletMap.h>

namespace camrod {
namespace map {

struct LoaderConfig
{
  // WGS84 latitude origin for projector initialization.
  double offset_lat{0.0};
  // WGS84 longitude origin for projector initialization.
  double offset_lon{0.0};
  // WGS84 altitude origin for projector initialization.
  double offset_alt{0.0};
};

/**
 * @brief HH_260114 Helper to load Lanelet2 OSM into a LocalCartesian frame.
 */
class Lanelet2MapLoader
{
public:
  // Creates a loader with fixed projector-origin configuration.
  explicit Lanelet2MapLoader(const LoaderConfig & cfg);

  // Loads a lanelet2 OSM map and returns the parsed map object.
  lanelet::LaneletMapPtr load(const std::string & map_path);

  // Returns the currently cached lanelet map pointer.
  lanelet::LaneletMapPtr getMap() const { return map_; }

  // Builds a short human-readable map-layer statistics string.
  std::string getMapStats() const;

  // Returns the active loader configuration snapshot.
  const LoaderConfig & getConfig() const { return cfg_; }

private:
  LoaderConfig cfg_;
  lanelet::LaneletMapPtr map_;
};

}  // namespace map
}  // namespace camrod

#endif  // CAMROD_MAP__LANELET2_MAP_LOADER_HPP_
