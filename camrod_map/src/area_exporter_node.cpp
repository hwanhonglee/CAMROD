#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <avg_msgs/msg/avg_map_msgs.hpp>
#include <avg_msgs/msg/module_health.hpp>

#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include "camrod_map/lanelet2_map_loader.hpp"

namespace
{
constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_E2 = 6.69437999014e-3;

struct DropZone
{
  std::string id;
  std::string type;
  std::string semantic_type;
  std::string source;
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw_deg{0.0};
  std::vector<lanelet::BasicPoint3d> corners;
};

// Implements `toLowerCopy` behavior.
std::string toLowerCopy(const std::string & value)
{
  std::string lower = value;
  // Implements `transform` behavior.
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return lower;
}

template<typename PrimitiveT>
// Implements `getAttributeCaseInsensitive` behavior.
std::string getAttributeCaseInsensitive(const PrimitiveT & primitive, const std::string & key)
{
  std::string value = primitive.attributeOr(key.c_str(), "");
  if (!value.empty()) {
    return value;
  }
  std::string lower = key;
  // Implements `transform` behavior.
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  value = primitive.attributeOr(lower.c_str(), "");
  if (!value.empty()) {
    return value;
  }
  std::string upper = key;
  // Implements `transform` behavior.
  std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return primitive.attributeOr(upper.c_str(), "");
}

template<typename PrimitiveT>
// Implements `getSemanticType` behavior.
std::string getSemanticType(const PrimitiveT & primitive)
{
  auto subtype = getAttributeCaseInsensitive(primitive, "Subtype");
  if (!subtype.empty()) {
    return toLowerCopy(subtype);
  }
  auto type = getAttributeCaseInsensitive(primitive, "type");
  return toLowerCopy(type);
}

// Implements `startsWith` behavior.
bool startsWith(const std::string & value, const std::string & prefix)
{
  return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

// Implements `extractSuffixNumber` behavior.
std::optional<int> extractSuffixNumber(const std::string & value, const std::string & prefix)
{
  if (!startsWith(value, prefix)) {
    return std::nullopt;
  }
  const auto suffix = value.substr(prefix.size());
  if (suffix.empty()) {
    return std::nullopt;
  }
  for (const auto c : suffix) {
    if (!std::isdigit(static_cast<unsigned char>(c))) {
      return std::nullopt;
    }
  }
  return std::stoi(suffix);
}

// Implements `rad2deg` behavior.
double rad2deg(double rad)
{
  return rad * 180.0 / M_PI;
}

// Implements `deg2rad` behavior.
double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

struct Ecef
{
  double x;
  double y;
  double z;
};

// Implements `llhToEcef` behavior.
Ecef llhToEcef(double lat_rad, double lon_rad, double alt)
{
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);
  const double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
  return Ecef{
    (N + alt) * cos_lat * cos_lon,
    (N + alt) * cos_lat * sin_lon,
    (N * (1.0 - WGS84_E2) + alt) * sin_lat};
}

// Implements `ecefToEnu` behavior.
lanelet::BasicPoint3d ecefToEnu(
  const Ecef & ref, const Ecef & cur, double lat_ref, double lon_ref)
{
  const double sin_lat = std::sin(lat_ref);
  const double cos_lat = std::cos(lat_ref);
  const double sin_lon = std::sin(lon_ref);
  const double cos_lon = std::cos(lon_ref);
  const double dx = cur.x - ref.x;
  const double dy = cur.y - ref.y;
  const double dz = cur.z - ref.z;
  const double east = -sin_lon * dx + cos_lon * dy;
  const double north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
  const double up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
  return lanelet::BasicPoint3d(east, north, up);
}

// Computes `YawDegFromPoints` values.
double computeYawDegFromPoints(
  const std::vector<lanelet::BasicPoint3d> & pts, double default_yaw_deg)
{
  if (pts.size() < 2) {
    return default_yaw_deg;
  }
  double best_len = 0.0;
  double best_yaw = default_yaw_deg * M_PI / 180.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    const double dx = pts[i].x() - pts[i - 1].x();
    const double dy = pts[i].y() - pts[i - 1].y();
    const double len = std::hypot(dx, dy);
    if (len > best_len) {
      best_len = len;
      best_yaw = std::atan2(dy, dx);
    }
  }
  return rad2deg(best_yaw);
}

// Computes `Centroid` values.
lanelet::BasicPoint3d computeCentroid(const std::vector<lanelet::BasicPoint3d> & pts)
{
  if (pts.empty()) {
    return lanelet::BasicPoint3d(0.0, 0.0, 0.0);
  }
  double sx = 0.0;
  double sy = 0.0;
  double sz = 0.0;
  for (const auto & p : pts) {
    sx += p.x();
    sy += p.y();
    sz += p.z();
  }
  const double inv = 1.0 / static_cast<double>(pts.size());
  return lanelet::BasicPoint3d(sx * inv, sy * inv, sz * inv);
}

// Computes `PolygonCentroid` values.
lanelet::BasicPoint3d computePolygonCentroid(
  const std::vector<lanelet::BasicPoint3d> & pts)
{
  if (pts.size() < 3) {
    return computeCentroid(pts);
  }
  double area = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double sz = 0.0;
  const size_t n = pts.size();
  for (size_t i = 0; i < n; ++i) {
    const auto & p0 = pts[i];
    const auto & p1 = pts[(i + 1) % n];
    const double cross = p0.x() * p1.y() - p1.x() * p0.y();
    area += cross;
    cx += (p0.x() + p1.x()) * cross;
    cy += (p0.y() + p1.y()) * cross;
    sz += p0.z();
  }
  if (std::abs(area) < 1e-9) {
    return computeCentroid(pts);
  }
  area *= 0.5;
  cx /= (6.0 * area);
  cy /= (6.0 * area);
  const double z = sz / static_cast<double>(n);
  return lanelet::BasicPoint3d(cx, cy, z);
}

template<typename PrimitiveT>
// Implements `attributeIsTrue` behavior.
bool attributeIsTrue(const PrimitiveT & primitive, const std::string & key)
{
  const auto value = toLowerCopy(getAttributeCaseInsensitive(primitive, key));
  return value == "yes" || value == "true" || value == "1";
}

// Implements `getAttr` behavior.
std::string getAttr(const std::string & line, const std::string & key)
{
  const std::string pattern = key + "=\"";
  const auto pos = line.find(pattern);
  if (pos == std::string::npos) {
    return "";
  }
  const auto start = pos + pattern.size();
  const auto end = line.find("\"", start);
  if (end == std::string::npos) {
    return "";
  }
  return line.substr(start, end - start);
}
}  // namespace

namespace camping_cart
{
namespace map
{
class AreaExporterNode : public rclcpp::Node
{
public:
  // Implements `AreaExporterNode` behavior.
  AreaExporterNode()
  // 2026-01-30: Export drop-zone candidates from Lanelet2 map into YAML.
  : Node("area_exporter")
  {
    map_path_ = declare_parameter<std::string>("map_path", "");
    output_yaml_path_ = declare_parameter<std::string>("output_yaml_path", "");
    camping_sites_output_yaml_path_ = declare_parameter<std::string>(
      "camping_sites_output_yaml_path", "");
    origin_lat_ = declare_parameter<double>("origin_lat", 0.0);
    origin_lon_ = declare_parameter<double>("origin_lon", 0.0);
    origin_alt_ = declare_parameter<double>("origin_alt", 0.0);
    default_yaw_deg_ = declare_parameter<double>("default_yaw_deg", 0.0);
    dropzone_types_ = declare_parameter<std::vector<std::string>>(
      "dropzone_types", std::vector<std::string>{"drop_zone"});
    camping_site_prefixes_ = declare_parameter<std::vector<std::string>>(
      "camping_site_prefixes", std::vector<std::string>{"camping_site_"});
    publish_map_diagnostic_ = declare_parameter<bool>("publish_map_diagnostic", false);
    map_diagnostic_topic_ = declare_parameter<std::string>("map_diagnostic_topic", "/map/diagnostic");
    if (publish_map_diagnostic_) {
      avg_map_pub_ = create_publisher<avg_msgs::msg::AvgMapMsgs>(map_diagnostic_topic_, rclcpp::QoS(10));
    }
  }

  // Implements `run` behavior.
  int run()
  {
    if (map_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "map_path is empty.");
      publishStatus(avg_msgs::msg::ModuleHealth::ERROR, "map_path is empty");
      return 1;
    }

    LoaderConfig cfg;
    cfg.offset_lat = origin_lat_;
    cfg.offset_lon = origin_lon_;
    cfg.offset_alt = origin_alt_;
    Lanelet2MapLoader loader(cfg);
    auto map = loader.load(map_path_);
    if (!map) {
      RCLCPP_ERROR(get_logger(), "failed to load map: %s", map_path_.c_str());
      publishStatus(avg_msgs::msg::ModuleHealth::ERROR, "failed to load map");
      return 1;
    }

    std::unordered_set<std::string> dropzone_types;
    dropzone_types.reserve(dropzone_types_.size());
    for (const auto & t : dropzone_types_) {
      dropzone_types.insert(toLowerCopy(t));
    }
    std::vector<std::string> camping_prefixes;
    camping_prefixes.reserve(camping_site_prefixes_.size());
    for (const auto & p : camping_site_prefixes_) {
      camping_prefixes.push_back(toLowerCopy(p));
    }

    // 2026-03-19: Support both exact semantic types (drop_zone) and prefixed
    // semantics (camping_site_1..n) from lanelet2 type/subtype.
    auto classifySemanticType =
      [&dropzone_types, &camping_prefixes](const std::string & semantic_type) -> std::string {
        if (semantic_type.empty()) {
          return "";
        }
        if (dropzone_types.find(semantic_type) != dropzone_types.end()) {
          return "drop_zone";
        }
        for (const auto & prefix : camping_prefixes) {
          if (startsWith(semantic_type, prefix)) {
            return "camping_site";
          }
        }
        return "";
      };

    std::vector<DropZone> zones;

    for (const auto & area : map->areaLayer) {
      const auto type = getSemanticType(area);
      const auto zone_type = classifySemanticType(type);
      if (zone_type.empty()) {
        continue;
      }
      std::vector<lanelet::BasicPoint3d> pts;
      const auto & outer = area.outerBound();
      // 2026-01-30: Area outer bounds are a list of LineStrings; flatten to points.
      for (const auto & ls : outer) {
        pts.reserve(pts.size() + ls.size());
        for (const auto & p : ls) {
          pts.push_back(lanelet::BasicPoint3d(p.x(), p.y(), p.z()));
        }
      }
      const auto center = computeCentroid(pts);
      DropZone dz;
      dz.id = "dz_area_" + std::to_string(area.id());
      dz.type = zone_type;
      dz.semantic_type = type;
      dz.source = "area";
      dz.x = center.x();
      dz.y = center.y();
      dz.z = center.z();
      dz.yaw_deg = computeYawDegFromPoints(pts, default_yaw_deg_);
      zones.push_back(dz);
    }

    for (const auto & ls : map->lineStringLayer) {
      const auto type = getSemanticType(ls);
      const auto zone_type = classifySemanticType(type);
      if (zone_type.empty()) {
        continue;
      }
      std::vector<lanelet::BasicPoint3d> pts;
      pts.reserve(ls.size());
      for (const auto & p : ls) {
        pts.push_back(lanelet::BasicPoint3d(p.x(), p.y(), p.z()));
      }
      const bool is_area = attributeIsTrue(ls, "area");
      const auto center = is_area ? computePolygonCentroid(pts) : computeCentroid(pts);
      DropZone dz;
      dz.id = "dz_line_" + std::to_string(ls.id());
      dz.type = zone_type;
      dz.semantic_type = type;
      dz.source = is_area ? "linestring_area" : "linestring";
      dz.x = center.x();
      dz.y = center.y();
      dz.z = center.z();
      dz.yaw_deg = computeYawDegFromPoints(pts, default_yaw_deg_);
      zones.push_back(dz);
    }

    for (const auto & pt : map->pointLayer) {
      const auto type = getSemanticType(pt);
      const auto zone_type = classifySemanticType(type);
      if (zone_type.empty()) {
        continue;
      }
      DropZone dz;
      dz.id = "dz_point_" + std::to_string(pt.id());
      dz.type = zone_type;
      dz.semantic_type = type;
      dz.source = "point";
      dz.x = pt.x();
      dz.y = pt.y();
      dz.z = pt.z();
      dz.yaw_deg = default_yaw_deg_;
      zones.push_back(dz);
    }

    size_t layer_drop_zone_count = 0;
    size_t layer_camping_site_count = 0;
    for (const auto & zone : zones) {
      if (zone.type == "drop_zone") {
        ++layer_drop_zone_count;
      } else if (zone.type == "camping_site") {
        ++layer_camping_site_count;
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "semantic extraction from lanelet layers: drop_zone=%zu camping_site=%zu total=%zu",
      layer_drop_zone_count, layer_camping_site_count, zones.size());

    // 2026-03-19: Some maps expose camping-site polygons in lanelet layers
    // while drop-zone polygons exist only as raw OSM ways. To avoid losing one
    // semantic family when the other is present, always parse OSM fallback and
    // merge only missing logical types.
    const auto fallback_zones = parseDropZonesFromOsm(map_path_, classifySemanticType);
    size_t fallback_drop_zone_count = 0;
    size_t fallback_camping_site_count = 0;
    for (const auto & zone : fallback_zones) {
      if (zone.type == "drop_zone") {
        ++fallback_drop_zone_count;
      } else if (zone.type == "camping_site") {
        ++fallback_camping_site_count;
      }
    }
    RCLCPP_INFO(
      get_logger(), "semantic extraction from OSM fallback: drop_zone=%zu camping_site=%zu total=%zu",
      fallback_drop_zone_count, fallback_camping_site_count, fallback_zones.size());
    if (!fallback_zones.empty()) {
      size_t map_drop_zone_count = 0;
      size_t map_camping_site_count = 0;
      for (const auto & zone : zones) {
        if (zone.type == "drop_zone") {
          ++map_drop_zone_count;
        } else if (zone.type == "camping_site") {
          ++map_camping_site_count;
        }
      }

      for (const auto & zone : fallback_zones) {
        if (zone.type == "drop_zone" && map_drop_zone_count == 0) {
          zones.push_back(zone);
        } else if (zone.type == "camping_site" && map_camping_site_count == 0) {
          zones.push_back(zone);
        }
      }
    }

    std::vector<DropZone> drop_zones;
    std::vector<DropZone> camping_sites;
    for (const auto & zone : zones) {
      if (zone.type == "drop_zone") {
        drop_zones.push_back(zone);
      } else if (zone.type == "camping_site") {
        camping_sites.push_back(zone);
      }
    }

    // 2026-03-19: Deduplicate camping sites by semantic name to keep one center
    // per logical site (e.g. camping_site_1).
    std::unordered_set<std::string> seen_camping;
    std::vector<DropZone> unique_camping_sites;
    unique_camping_sites.reserve(camping_sites.size());
    for (const auto & site : camping_sites) {
      if (site.semantic_type.empty()) {
        unique_camping_sites.push_back(site);
        continue;
      }
      if (seen_camping.insert(site.semantic_type).second) {
        unique_camping_sites.push_back(site);
      }
    }
    camping_sites = std::move(unique_camping_sites);

    // Sort camping sites by numeric suffix for deterministic 1..n ordering.
    std::sort(camping_sites.begin(), camping_sites.end(), [&](const DropZone & a, const DropZone & b) {
      const std::string default_prefix =
        camping_prefixes.empty() ? std::string("camping_site_") : camping_prefixes.front();
      const auto a_num = extractSuffixNumber(a.semantic_type, default_prefix);
      const auto b_num = extractSuffixNumber(b.semantic_type, default_prefix);
      if (a_num.has_value() && b_num.has_value()) {
        if (a_num.value() != b_num.value()) {
          return a_num.value() < b_num.value();
        }
      } else if (a_num.has_value() != b_num.has_value()) {
        return a_num.has_value();
      }
      return a.semantic_type < b.semantic_type;
    });

    if (output_yaml_path_.empty() && camping_sites_output_yaml_path_.empty()) {
      RCLCPP_INFO(
        get_logger(), "found %zu drop zones and %zu camping sites (no output paths set)",
        drop_zones.size(), camping_sites.size());
      publishStatus(avg_msgs::msg::ModuleHealth::OK, "drop zone scan completed");
      return 0;
    }

    if (!output_yaml_path_.empty()) {
      std::ofstream out(output_yaml_path_);
      if (!out.is_open()) {
        RCLCPP_ERROR(get_logger(), "failed to open output_yaml_path: %s", output_yaml_path_.c_str());
        publishStatus(avg_msgs::msg::ModuleHealth::ERROR, "failed to open drop-zone yaml path");
        return 1;
      }

      out << "drop_zones:\n";
      for (const auto & dz : drop_zones) {
        out << "  - id: \"" << dz.id << "\"\n";
        out << "    type: \"" << dz.semantic_type << "\"\n";
        out << "    source: \"" << dz.source << "\"\n";
        out << "    x: " << dz.x << "\n";
        out << "    y: " << dz.y << "\n";
        out << "    z: " << dz.z << "\n";
        out << "    yaw_deg: " << dz.yaw_deg << "\n";
        if (!dz.corners.empty()) {
          out << "    corners:\n";
          for (const auto & c : dz.corners) {
            out << "      - x: " << c.x() << "\n";
            out << "        y: " << c.y() << "\n";
            out << "        z: " << c.z() << "\n";
          }
        }
      }
      out.close();
      RCLCPP_INFO(get_logger(), "wrote %zu drop zones to %s", drop_zones.size(), output_yaml_path_.c_str());
    }

    if (!camping_sites_output_yaml_path_.empty()) {
      std::ofstream out(camping_sites_output_yaml_path_);
      if (!out.is_open()) {
        RCLCPP_ERROR(
          get_logger(), "failed to open camping_sites_output_yaml_path: %s",
          camping_sites_output_yaml_path_.c_str());
        publishStatus(avg_msgs::msg::ModuleHealth::ERROR, "failed to open camping-sites yaml path");
        return 1;
      }

      out << "camping_sites:\n";
      for (const auto & site : camping_sites) {
        out << "  - id: \"" << site.id << "\"\n";
        out << "    type: \"" << site.semantic_type << "\"\n";
        out << "    source: \"" << site.source << "\"\n";
        out << "    x: " << site.x << "\n";
        out << "    y: " << site.y << "\n";
        out << "    z: " << site.z << "\n";
        out << "    yaw_deg: " << site.yaw_deg << "\n";
        if (!site.corners.empty()) {
          out << "    corners:\n";
          for (const auto & c : site.corners) {
            out << "      - x: " << c.x() << "\n";
            out << "        y: " << c.y() << "\n";
            out << "        z: " << c.z() << "\n";
          }
        }
      }
      out.close();
      RCLCPP_INFO(
        get_logger(), "wrote %zu camping sites to %s", camping_sites.size(),
        camping_sites_output_yaml_path_.c_str());
    }

    publishStatus(avg_msgs::msg::ModuleHealth::OK, "drop-zone/camping-site yaml export completed");
    return 0;
  }

private:
  // Publishes `Status` output.
  void publishStatus(uint8_t level, const std::string & message)
  {
    if (!publish_map_diagnostic_ || !avg_map_pub_) {
      return;
    }
    const auto stamp = now();
    avg_msgs::msg::AvgMapMsgs msg;
    msg.stamp = stamp;
    msg.health.stamp = stamp;
    msg.health.module_name = "map";
    msg.health.level = level;
    msg.health.message = message;
    avg_map_pub_->publish(msg);
  }

  struct OsmNode
  {
    bool has_local{false};
    double local_x{0.0};
    double local_y{0.0};
    double ele{0.0};
    double lat{0.0};
    double lon{0.0};
  };

  struct OsmWay
  {
    long long id{0};
    std::vector<long long> refs;
    std::string type;
    std::string subtype;
    bool area{false};
  };

  // Implements `parseDropZonesFromOsm` behavior.
  std::vector<DropZone> parseDropZonesFromOsm(
    const std::string & path,
    const std::function<std::string(const std::string &)> & classifySemanticType) const
  {
    std::ifstream in(path);
    if (!in.is_open()) {
      return {};
    }

    std::unordered_map<long long, OsmNode> nodes;
    std::vector<OsmWay> ways;

    OsmNode cur_node;
    long long cur_node_id = 0;
    bool in_node = false;

    OsmWay cur_way;
    bool in_way = false;

    std::string line;
    while (std::getline(in, line)) {
      if (line.find("<node") != std::string::npos && line.find("id=") != std::string::npos) {
        cur_node = OsmNode{};
        cur_node_id = std::stoll(getAttr(line, "id"));
        cur_node.lat = std::stod(getAttr(line, "lat"));
        cur_node.lon = std::stod(getAttr(line, "lon"));
        in_node = true;
        if (line.find("/>") != std::string::npos) {
          nodes[cur_node_id] = cur_node;
          in_node = false;
        }
        continue;
      }

      if (in_node) {
        if (line.find("<tag") != std::string::npos) {
          const auto key = getAttr(line, "k");
          const auto val = getAttr(line, "v");
          if (key == "local_x") {
            cur_node.local_x = std::stod(val);
            cur_node.has_local = true;
          } else if (key == "local_y") {
            cur_node.local_y = std::stod(val);
            cur_node.has_local = true;
          } else if (key == "ele") {
            cur_node.ele = std::stod(val);
          }
        }
        if (line.find("</node>") != std::string::npos) {
          nodes[cur_node_id] = cur_node;
          in_node = false;
        }
        continue;
      }

      if (line.find("<way") != std::string::npos && line.find("id=") != std::string::npos) {
        cur_way = OsmWay{};
        cur_way.id = std::stoll(getAttr(line, "id"));
        in_way = true;
        continue;
      }

      if (in_way) {
        if (line.find("<nd") != std::string::npos && line.find("ref=") != std::string::npos) {
          cur_way.refs.push_back(std::stoll(getAttr(line, "ref")));
          continue;
        }
        if (line.find("<tag") != std::string::npos) {
          const auto key = getAttr(line, "k");
          const auto val = getAttr(line, "v");
          if (key == "type") {
            cur_way.type = toLowerCopy(val);
          } else if (key == "subtype") {
            cur_way.subtype = toLowerCopy(val);
          } else if (key == "area") {
            const auto v = toLowerCopy(val);
            cur_way.area = (v == "yes" || v == "true" || v == "1");
          }
        }
        if (line.find("</way>") != std::string::npos) {
          const auto semantic_type = cur_way.subtype.empty() ? cur_way.type : cur_way.subtype;
          if (!classifySemanticType(semantic_type).empty()) {
            cur_way.type = semantic_type;
            ways.push_back(cur_way);
          }
          in_way = false;
        }
      }
    }

    std::vector<DropZone> zones;
    const double origin_lat_rad = deg2rad(origin_lat_);
    const double origin_lon_rad = deg2rad(origin_lon_);
    const auto ref_ecef = llhToEcef(origin_lat_rad, origin_lon_rad, origin_alt_);

    for (const auto & way : ways) {
      std::vector<lanelet::BasicPoint3d> pts;
      pts.reserve(way.refs.size());
      for (const auto ref : way.refs) {
        const auto it = nodes.find(ref);
        if (it == nodes.end()) {
          continue;
        }
        const auto & node = it->second;
        if (node.has_local) {
          pts.emplace_back(node.local_x, node.local_y, node.ele);
        } else {
          const auto cur_ecef = llhToEcef(deg2rad(node.lat), deg2rad(node.lon), node.ele);
          pts.push_back(ecefToEnu(ref_ecef, cur_ecef, origin_lat_rad, origin_lon_rad));
        }
      }
      if (pts.size() < 2) {
        continue;
      }
      std::vector<lanelet::BasicPoint3d> corners = pts;
      if (way.area && !corners.empty()) {
        // Remove closing duplicate if present.
        const auto & first = corners.front();
        const auto & last = corners.back();
        if (first.x() == last.x() && first.y() == last.y() && first.z() == last.z()) {
          corners.pop_back();
        }
      }

      lanelet::BasicPoint3d center;
      if (way.area && corners.size() == 4) {
        center = computeCentroid(corners);
      } else if (way.area) {
        center = computePolygonCentroid(pts);
      } else {
        center = computeCentroid(pts);
      }
      DropZone dz;
      dz.id = "dz_way_" + std::to_string(way.id);
      dz.type = classifySemanticType(way.type);
      dz.semantic_type = way.type;
      dz.source = way.area ? "osm_area" : "osm_way";
      dz.x = center.x();
      dz.y = center.y();
      dz.z = center.z();
      dz.yaw_deg = computeYawDegFromPoints(pts, default_yaw_deg_);
      if (dz.type == "drop_zone" && way.area && corners.size() >= 3) {
        dz.corners = corners;
      }
      zones.push_back(dz);
    }

    return zones;
  }

  std::string map_path_;
  std::string output_yaml_path_;
  std::string camping_sites_output_yaml_path_;
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_alt_{0.0};
  double default_yaw_deg_{0.0};
  std::vector<std::string> dropzone_types_;
  std::vector<std::string> camping_site_prefixes_;
  bool publish_map_diagnostic_{false};
  std::string map_diagnostic_topic_{"/map/diagnostic"};
  rclcpp::Publisher<avg_msgs::msg::AvgMapMsgs>::SharedPtr avg_map_pub_;
};
}  // namespace map
}  // namespace camping_cart

// Entry point for this executable.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<camping_cart::map::AreaExporterNode>();
  const int rc = node->run();
  rclcpp::shutdown();
  return rc;
}
