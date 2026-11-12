// Lat/lon -> map frame for the NDT localization nodes (C++ port of ndt_localization/geo.py).
//
// Map frames
//   UTM                UTM (zone of the origin) minus UTM(origin); z = ellipsoidal height
//   TransverseMercator Autoware map_projector_info.yaml: TM with the origin longitude as central meridian and
//                      scale_factor, minus the projected origin; z = ellipsoidal height - map_origin.altitude
//   LocalCartesian     camrod (camrod_map/config/map_info.yaml offset_lat/lon/alt): WGS84 lat/lon/h -> ECEF -> ENU
//                      at the origin, as camrod's lanelet2 LocalCartesianProjector and localization_input_adapter
#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace ndt_localization_cpp
{

struct UtmResult
{
  double east, north;
  int zone;
};

UtmResult lat_lon_to_utm(double lat_deg, double lon_deg);
// transverse Mercator, any central meridian / scale, no false easting; y from the equator
std::array<double, 2> lat_lon_to_tm(double lat_deg, double lon_deg, double lon0_deg, double k0);
std::array<double, 3> lat_lon_h_to_ecef(double lat_deg, double lon_deg, double h);

class MapProjection
{
public:
  enum class Type { UTM, TransverseMercator, LocalCartesian };

  MapProjection(double origin_lat, double origin_lon, Type type = Type::UTM, double scale_factor = 0.9996,
                double origin_alt = 0.0);
  // Autoware map_projector_info.yaml (projector_type TransverseMercator or UTM)
  static MapProjection from_projector_yaml(const std::string & path);
  // camrod_map/config/map_info.yaml (ROS params file: /** -> ros__parameters -> offset_lat/lon/alt)
  static MapProjection from_camrod_map_info(const std::string & path);

  // h: ellipsoidal height, only used by LocalCartesian (NaN = origin altitude)
  std::array<double, 2> to_map(double lat_deg, double lon_deg,
                               double h = std::numeric_limits<double>::quiet_NaN()) const;
  // map z of an ellipsoidal height (LocalCartesian: ENU up at lat/lon, with the curvature drop)
  double z_of(double ellipsoidal_height, double lat_deg, double lon_deg) const;
  // true heading (deg, clockwise from north) -> yaw in the map (grid) frame [rad]
  double heading_to_yaw(double lat_deg, double lon_deg, double heading_deg) const;

  Type type() const { return type_; }
  std::string type_name() const;
  double origin_e() const { return origin_e_; }
  double origin_n() const { return origin_n_; }
  double origin_alt() const { return origin_alt_; }

private:
  std::array<double, 3> enu(double lat_deg, double lon_deg, double h) const;

  Type type_;
  double lon0_, k0_, origin_alt_;
  double origin_e_{0.0}, origin_n_{0.0};
  std::array<double, 3> ref_{};                         // LocalCartesian: ECEF of the origin
  std::array<std::array<double, 3>, 3> enu_rot_{};      // LocalCartesian: ECEF -> ENU rotation
};

}  // namespace ndt_localization_cpp
