// C++ port of ndt_localization/geo.py (same series and formulas, so both give the same map coordinates).
#include "ndt_localization_cpp/geo.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace ndt_localization_cpp
{
namespace
{
constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_F = 1.0 / 298.257223563;
constexpr double WGS84_E2 = 2.0 * WGS84_F - WGS84_F * WGS84_F;
constexpr double EARTH_RADIUS_M = 6371000.0;
constexpr double DEG = M_PI / 180.0;

// meridian arc length and the series terms shared by UTM and TM
void tm_series(double lat_deg, double lon_deg, double lon0_deg, double k0, double & x, double & y)
{
  const double a = WGS84_A, e2 = WGS84_E2;
  const double ep2 = e2 / (1.0 - e2);
  const double lat = lat_deg * DEG;
  const double sin_lat = std::sin(lat), cos_lat = std::cos(lat), tan_lat = std::tan(lat);
  const double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
  const double T = tan_lat * tan_lat;
  const double C = ep2 * cos_lat * cos_lat;
  const double A = cos_lat * ((lon_deg - lon0_deg) * DEG);
  const double A2 = A * A, A3 = A2 * A, A4 = A3 * A, A5 = A4 * A, A6 = A5 * A;
  const double e4 = e2 * e2, e6 = e4 * e2;
  const double M = a * ((1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * lat -
                        (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0) * std::sin(2.0 * lat) +
                        (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * std::sin(4.0 * lat) -
                        (35.0 * e6 / 3072.0) * std::sin(6.0 * lat));
  x = k0 * N * (A + (1.0 - T + C) * A3 / 6.0 + (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ep2) * A5 / 120.0);
  y = k0 * (M + N * tan_lat * (A2 / 2.0 + (5.0 - T + 9.0 * C + 4.0 * C * C) * A4 / 24.0 +
                               (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ep2) * A6 / 720.0));
}
}  // namespace

UtmResult lat_lon_to_utm(double lat_deg, double lon_deg)
{
  const int zone = static_cast<int>((lon_deg + 180.0) / 6.0) + 1;
  const double lon0 = (zone - 1) * 6 - 180 + 3;
  double x, y;
  tm_series(lat_deg, lon_deg, lon0, 0.9996, x, y);
  UtmResult r{x + 500000.0, y, zone};
  if (lat_deg < 0.0) {
    r.north += 10000000.0;
  }
  return r;
}

std::array<double, 2> lat_lon_to_tm(double lat_deg, double lon_deg, double lon0_deg, double k0)
{
  double x, y;
  tm_series(lat_deg, lon_deg, lon0_deg, k0, x, y);
  return {x, y};
}

std::array<double, 3> lat_lon_h_to_ecef(double lat_deg, double lon_deg, double h)
{
  const double lat = lat_deg * DEG, lon = lon_deg * DEG;
  const double n = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat) * std::sin(lat));
  return {(n + h) * std::cos(lat) * std::cos(lon), (n + h) * std::cos(lat) * std::sin(lon),
          (n * (1.0 - WGS84_E2) + h) * std::sin(lat)};
}

MapProjection::MapProjection(double origin_lat, double origin_lon, Type type, double scale_factor, double origin_alt)
: type_(type), lon0_(origin_lon), k0_(scale_factor), origin_alt_(origin_alt)
{
  if (type_ == Type::LocalCartesian) {
    ref_ = lat_lon_h_to_ecef(origin_lat, origin_lon, origin_alt);
    const double la = origin_lat * DEG, lo = origin_lon * DEG;
    enu_rot_ = {{{-std::sin(lo), std::cos(lo), 0.0},
                 {-std::sin(la) * std::cos(lo), -std::sin(la) * std::sin(lo), std::cos(la)},
                 {std::cos(la) * std::cos(lo), std::cos(la) * std::sin(lo), std::sin(la)}}};
  } else if (type_ == Type::UTM) {
    const auto u = lat_lon_to_utm(origin_lat, origin_lon);
    origin_e_ = u.east;
    origin_n_ = u.north;
  } else {
    const auto t = lat_lon_to_tm(origin_lat, origin_lon, origin_lon, scale_factor);
    origin_e_ = t[0];
    origin_n_ = t[1];
  }
}

MapProjection MapProjection::from_projector_yaml(const std::string & path)
{
  const YAML::Node p = YAML::LoadFile(path);
  const YAML::Node o = p["map_origin"];
  const std::string t = p["projector_type"] ? p["projector_type"].as<std::string>() : "TransverseMercator";
  Type type;
  if (t == "TransverseMercator") {
    type = Type::TransverseMercator;
  } else if (t == "UTM") {
    type = Type::UTM;
  } else {
    throw std::runtime_error(path + ": projector_type " + t + " is not supported");
  }
  return MapProjection(o["latitude"].as<double>(), o["longitude"].as<double>(), type,
                       p["scale_factor"] ? p["scale_factor"].as<double>() : 0.9996,
                       o["altitude"] ? o["altitude"].as<double>() : 0.0);
}

MapProjection MapProjection::from_camrod_map_info(const std::string & path)
{
  YAML::Node p = YAML::LoadFile(path);
  if (p["/**"]) {
    p = p["/**"];
  }
  if (p["ros__parameters"]) {
    p = p["ros__parameters"];
  }
  if (p["yaw_offset_deg"] && p["yaw_offset_deg"].as<double>() != 0.0) {
    throw std::runtime_error(path + ": yaw_offset_deg != 0 is not supported");
  }
  return MapProjection(p["offset_lat"].as<double>(), p["offset_lon"].as<double>(), Type::LocalCartesian, 0.9996,
                       p["offset_alt"] ? p["offset_alt"].as<double>() : 0.0);
}

std::array<double, 3> MapProjection::enu(double lat_deg, double lon_deg, double h) const
{
  const auto x = lat_lon_h_to_ecef(lat_deg, lon_deg, h);
  const double d[3] = {x[0] - ref_[0], x[1] - ref_[1], x[2] - ref_[2]};
  std::array<double, 3> r{};
  for (int i = 0; i < 3; ++i) {
    r[i] = enu_rot_[i][0] * d[0] + enu_rot_[i][1] * d[1] + enu_rot_[i][2] * d[2];
  }
  return r;
}

std::array<double, 2> MapProjection::to_map(double lat_deg, double lon_deg, double h) const
{
  if (type_ == Type::LocalCartesian) {
    const auto e = enu(lat_deg, lon_deg, std::isnan(h) ? origin_alt_ : h);
    return {e[0], e[1]};
  }
  if (type_ == Type::UTM) {
    const auto u = lat_lon_to_utm(lat_deg, lon_deg);
    return {u.east - origin_e_, u.north - origin_n_};
  }
  const auto t = lat_lon_to_tm(lat_deg, lon_deg, lon0_, k0_);
  return {t[0] - origin_e_, t[1] - origin_n_};
}

double MapProjection::z_of(double ellipsoidal_height, double lat_deg, double lon_deg) const
{
  if (type_ == Type::LocalCartesian) {
    return enu(lat_deg, lon_deg, ellipsoidal_height)[2];
  }
  return ellipsoidal_height - origin_alt_;
}

double MapProjection::heading_to_yaw(double lat_deg, double lon_deg, double heading_deg) const
{
  // project a 1 m step along the heading: grid convergence without a sign convention to get wrong
  const double h = heading_deg * DEG;
  const double dlat = std::cos(h) / EARTH_RADIUS_M / DEG;
  const double dlon = std::sin(h) / (EARTH_RADIUS_M * std::cos(lat_deg * DEG)) / DEG;
  const auto p0 = to_map(lat_deg, lon_deg);
  const auto p1 = to_map(lat_deg + dlat, lon_deg + dlon);
  return std::atan2(p1[1] - p0[1], p1[0] - p0[0]);
}

std::string MapProjection::type_name() const
{
  switch (type_) {
    case Type::UTM: return "UTM";
    case Type::TransverseMercator: return "TransverseMercator";
    default: return "LocalCartesian";
  }
}

}  // namespace ndt_localization_cpp
