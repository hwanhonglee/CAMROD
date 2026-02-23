#ifndef CAMPING_CART_MAP__LANELET2_MAP_LOADER_HPP_
#define CAMPING_CART_MAP__LANELET2_MAP_LOADER_HPP_

#include <memory>
#include <string>

#include <lanelet2_core/LaneletMap.h>

namespace camping_cart {
namespace map {

struct LoaderConfig
{
  double offset_lat{0.0};
  double offset_lon{0.0};
  double offset_alt{0.0};
};

/**
 * @brief HH_260114 Helper to load Lanelet2 OSM into a LocalCartesian frame.
 */
class Lanelet2MapLoader
{
public:
  explicit Lanelet2MapLoader(const LoaderConfig & cfg);

  lanelet::LaneletMapPtr load(const std::string & map_path);

  lanelet::LaneletMapPtr getMap() const { return map_; }

  std::string getMapStats() const;

  const LoaderConfig & getConfig() const { return cfg_; }

private:
  LoaderConfig cfg_;
  lanelet::LaneletMapPtr map_;
};

}  // namespace map
}  // namespace camping_cart

#endif  // CAMPING_CART_MAP__LANELET2_MAP_LOADER_HPP_
