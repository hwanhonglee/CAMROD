// Initial pose stream for autoware_ndt_scan_matcher (its ekf_pose_with_covariance input).
// C++ port of ndt_localization/initial_pose_bridge.py; same parameters, topics and decisions.
//
// source:=odom      x, y, yaw from an EKF odometry in the map frame (camrod: /localization/odometry_ros)
// source:=gnss      NavPVT position + heading of motion (bag tests without an EKF)
// source:=gnss_ndt  GNSS position where it is good, NDT dead reckoning elsewhere; yaw ALWAYS from NDT once it runs
//   GNSS is used only if: RTK fixed or float with h_acc <= gnss_float_max_hacc; outside every ndt_zones_file polygon;
//   within gnss_max_diff_fixed / _float of the NDT prediction; coming back from NDT mode: for gnss_reacquire_time.
//   gnss_override: RTK fixed disagreeing with NDT for gnss_override_time wins (NDT slid).
//   gnss_sanity: any GNSS (h_acc <= gnss_sanity_max_hacc) farther than max(min_diff, k * h_acc) for sanity_time wins.
//   (Re)initialisation without NDT: good GNSS AND straight motion over init_heading_window (yaw = displacement).
// z, roll, pitch: latest NDT result (younger than feedback_timeout), else GNSS height - antenna z.
// Map frame: camrod_map_info (LocalCartesian ENU) > map_projector_info (TransverseMercator) > UTM - origin.
#include "ndt_localization_cpp/geo.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ndt_localization_cpp
{
namespace
{
struct Euler
{
  double roll, pitch, yaw;
};

Euler quat_to_euler(const geometry_msgs::msg::Quaternion & q)
{
  const double x = q.x, y = q.y, z = q.z, w = q.w;
  return {std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
          std::asin(std::max(-1.0, std::min(1.0, 2 * (w * y - z * x)))),
          std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))};
}

geometry_msgs::msg::Quaternion euler_to_quat(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2), cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  geometry_msgs::msg::Quaternion q;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  q.w = cr * cp * cy + sr * sp * sy;
  return q;
}
}  // namespace

class InitialPoseBridge : public rclcpp::Node
{
public:
  explicit InitialPoseBridge(const rclcpp::NodeOptions & options)
  : Node("ndt_initial_pose_bridge", options), proj_(0.0, 0.0)
  {
    source_ = declare_parameter<std::string>("source", "odom");
    if (source_ != "odom" && source_ != "gnss" && source_ != "gnss_ndt") {
      throw std::invalid_argument("source must be odom, gnss or gnss_ndt, got " + source_);
    }
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    const double origin_lat = declare_parameter<double>("origin_lat", 36.73011815844);
    const double origin_lon = declare_parameter<double>("origin_lon", 127.439640243);
    const auto mpi = declare_parameter<std::string>("map_projector_info", "");
    const auto cmi = declare_parameter<std::string>("camrod_map_info", "");
    proj_ = !cmi.empty() ? MapProjection::from_camrod_map_info(cmi)
          : !mpi.empty() ? MapProjection::from_projector_yaml(mpi)
                         : MapProjection(origin_lat, origin_lon);
    const auto ant = declare_parameter<std::vector<double>>("antenna_xyz", {0.75, 0.0, 1.12});
    if (ant.size() != 3) {
      throw std::invalid_argument("antenna_xyz needs 3 values");
    }
    antenna_ = {ant[0], ant[1], ant[2]};
    min_speed_ = declare_parameter<double>("min_speed_for_heading", 0.5);
    feedback_timeout_ = declare_parameter<double>("feedback_timeout", 2.0);
    const bool auto_trigger = declare_parameter<bool>("auto_trigger", true);
    const double rate = declare_parameter<double>("gnss_publish_rate", 20.0);
    max_extrap_ = declare_parameter<double>("gnss_max_extrapolation", 1.0);
    ndt_window_ = declare_parameter<double>("ndt_velocity_window", 0.5);
    ndt_max_extrap_ = declare_parameter<double>("ndt_max_extrapolation", 1.0);
    const auto zones_file = declare_parameter<std::string>("ndt_zones_file", "");
    accept_float_ = declare_parameter<bool>("gnss_accept_float", true);
    float_max_hacc_ = declare_parameter<double>("gnss_float_max_hacc", 0.5);
    max_diff_fixed_ = declare_parameter<double>("gnss_max_diff_fixed", 1.0);
    max_diff_float_ = declare_parameter<double>("gnss_max_diff_float", 0.5);
    reacquire_ = declare_parameter<double>("gnss_reacquire_time", 2.0);
    override_time_ = declare_parameter<double>("gnss_override_time", 1.0);
    s_hacc_ = declare_parameter<double>("gnss_sanity_max_hacc", 2.0);
    s_min_ = declare_parameter<double>("gnss_sanity_min_diff", 3.0);
    s_k_ = declare_parameter<double>("gnss_sanity_k", 3.0);
    s_time_ = declare_parameter<double>("gnss_sanity_time", 2.0);
    init_max_hacc_ = declare_parameter<double>("gnss_init_max_hacc", 1.0);
    max_speed_ = declare_parameter<double>("ndt_max_speed", 2.5);
    jump_reject_ = declare_parameter<double>("ndt_jump_reject", 1.0);
    yaw_rate_source_ = declare_parameter<std::string>("yaw_rate_source", "ndt");
    if (yaw_rate_source_ != "ndt" && yaw_rate_source_ != "imu") {
      throw std::invalid_argument("yaw_rate_source must be ndt or imu, got " + yaw_rate_source_);
    }
    init_window_ = declare_parameter<double>("init_heading_window", 3.0);
    init_min_dist_ = declare_parameter<double>("init_min_distance", 1.0);
    init_straight_ = declare_parameter<double>("init_min_straightness", 0.97);
    load_zones(zones_file);

    pub_mode_ = create_publisher<std_msgs::msg::String>("~/mode", 10);
    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/output", 10);
    sub_ndt_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/ndt_pose", 10, [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) { on_ndt(*m); });
    sub_navpvt_ = create_subscription<ublox_msgs::msg::NavPVT>(
      "~/navpvt", rclcpp::SensorDataQoS(), [this](ublox_msgs::msg::NavPVT::ConstSharedPtr m) { on_navpvt(*m); });
    if (source_ == "odom") {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "~/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { on_odom(*m); });
    } else {
      gnss_timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
                                         [this]() { on_gnss_timer(); });
    }
    if (source_ == "gnss_ndt" && yaw_rate_source_ == "imu") {
      sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "~/imu", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { gyro_z_ = m->angular_velocity.z; });
    }
    trigger_done_ = !auto_trigger;
    trigger_cli_ = create_client<std_srvs::srv::SetBool>("~/trigger_node");
    report_timer_ = create_wall_timer(std::chrono::seconds(10), [this]() { report(); });
    RCLCPP_INFO(get_logger(), "source=%s, map frame %s, origin %.3f %.3f, z offset %.3f", source_.c_str(),
                proj_.type_name().c_str(), proj_.origin_e(), proj_.origin_n(), proj_.origin_alt());
  }

private:
  struct Pred
  {
    double x, y, yaw;
  };
  struct Fix
  {
    double t, x, y;
    bool good;
  };
  struct Track
  {
    double t, x, y, yaw;
  };

  double now_s() { return get_clock()->now().seconds(); }
  static double stamp_s(const builtin_interfaces::msg::Time & s) { return s.sec + s.nanosec * 1e-9; }

  // ---- inputs ---------------------------------------------------------------------------------
  void on_ndt(const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    const Euler e = quat_to_euler(msg.pose.pose.orientation);
    const double t = stamp_s(msg.header.stamp);
    ndt_ = std::array<double, 4>{t, msg.pose.pose.position.z, e.roll, e.pitch};
    if (source_ == "gnss_ndt") {
      const auto & p = msg.pose.pose.position;
      const auto pred = ndt_predict(t);
      if (pred && std::hypot(p.x - pred->x, p.y - pred->y) > jump_reject_) {
        track_.clear();                                   // jump: no speed across it (last_v is kept)
      }
      track_.push_back({t, p.x, p.y, e.yaw});
      while (track_.size() > 2 && t - track_[1].t > ndt_window_) {
        track_.pop_front();
      }
    }
  }

  void on_navpvt(const ublox_msgs::msg::NavPVT & m)
  {
    if (!(m.flags & ublox_msgs::msg::NavPVT::FLAGS_GNSS_FIX_OK) || m.fix_type < ublox_msgs::msg::NavPVT::FIX_TYPE_3D) {
      return;
    }
    const double lat = m.lat * 1e-7, lon = m.lon * 1e-7, h = m.height * 1e-3;
    gnss_z_ = proj_.z_of(h, lat, lon) - antenna_[2];
    if (m.g_speed * 1e-3 >= min_speed_) {
      // NavPVT.heading = heading of motion (headMot) [1e-5 deg] (source:=gnss only)
      gnss_yaw_ = proj_.heading_to_yaw(lat, lon, m.heading * 1e-5);
    }
    if (source_ == "odom") {
      return;
    }
    const auto a = proj_.to_map(lat, lon, h);
    // velocity N/E (true) -> map grid: rotate by the grid convergence at this point
    const double grid = proj_.heading_to_yaw(lat, lon, 0.0) - M_PI / 2.0;
    const double ve = m.vel_e * 1e-3, vn = m.vel_n * 1e-3;
    const double vx = ve * std::cos(grid) - vn * std::sin(grid);
    const double vy = ve * std::sin(grid) + vn * std::cos(grid);
    // NavPVT has no header: stamp with the receive time (sim time when replaying a bag)
    const double t = now_s();
    gnss_state_ = std::array<double, 6>{t, a[0], a[1], vx, vy, std::pow(m.h_acc * 1e-3, 2)};
    gnss_fixed_ = (m.flags & 0xC0) == (2 << 6);           // carr_soln 2 = RTK fixed
    gnss_hacc_ = m.h_acc * 1e-3;
    fix_hist_.push_back({t, a[0], a[1], gnss_fixed_ || gnss_hacc_ <= init_max_hacc_});
    while (!fix_hist_.empty() && t - fix_hist_.front().t > init_window_) {
      fix_hist_.pop_front();
    }
  }

  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & c = msg.pose.covariance;
    publish(stamp_s(msg.header.stamp), msg.header.stamp, msg.pose.pose.position.x, msg.pose.pose.position.y,
            quat_to_euler(msg.pose.pose.orientation).yaw, std::max(c[0], c[7]), c[35]);
  }

  // ---- helpers --------------------------------------------------------------------------------
  std::array<double, 2> base_from_antenna(double ax, double ay, double yaw) const
  {
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {ax - (c * antenna_[0] - s * antenna_[1]), ay - (s * antenna_[0] + c * antenna_[1])};
  }

  // yaw from the antenna displacement over the last init_heading_window of good fixes, if straight and long enough
  std::optional<double> straight_heading() const
  {
    const auto & h = fix_hist_;
    if (h.size() < 5 || h.back().t - h.front().t < 0.5 * init_window_) {
      return std::nullopt;
    }
    double path = 0.0;
    for (size_t i = 0; i < h.size(); ++i) {
      if (!h[i].good) {
        return std::nullopt;
      }
      if (i) {
        path += std::hypot(h[i].x - h[i - 1].x, h[i].y - h[i - 1].y);
      }
    }
    const double dx = h.back().x - h.front().x, dy = h.back().y - h.front().y;
    const double disp = std::hypot(dx, dy);
    if (disp < init_min_dist_ || path <= 0.0 || disp / path < init_straight_) {
      return std::nullopt;
    }
    return std::atan2(dy, dx);
  }

  // latest NDT pose moved to time t (forward speed and yaw rate of the NDT track, or the IMU yaw rate)
  std::optional<Pred> ndt_predict(double t)
  {
    if (track_.empty()) {
      return std::nullopt;
    }
    const Track & a = track_.front();
    const Track & b = track_.back();
    double dt = t - b.t;
    if (dt < -0.5 || dt > ndt_max_extrap_) {
      return std::nullopt;
    }
    dt = std::max(dt, 0.0);
    double v;
    if (b.t - a.t >= 0.05) {                              // signed forward speed of the track, clamped
      v = ((b.x - a.x) * std::cos(b.yaw) + (b.y - a.y) * std::sin(b.yaw)) / (b.t - a.t);
      v = std::max(-max_speed_, std::min(max_speed_, v));
      last_v_ = v;
      last_w_ = std::atan2(std::sin(b.yaw - a.yaw), std::cos(b.yaw - a.yaw)) / (b.t - a.t);
    } else {
      v = last_v_;                                        // track just (re)started
    }
    const double w = yaw_rate_source_ == "imu" ? gyro_z_ : last_w_;
    const double yaw_m = b.yaw + 0.5 * w * dt;
    return Pred{b.x + v * std::cos(yaw_m) * dt, b.y + v * std::sin(yaw_m) * dt, b.yaw + w * dt};
  }

  void load_zones(const std::string & path)
  {
    if (path.empty()) {
      return;
    }
    std::string names;
    for (const auto & z : YAML::LoadFile(path)["zones"]) {
      std::vector<std::array<double, 2>> poly;
      for (const auto & p : z["polygon"]) {
        poly.push_back({p[0].as<double>(), p[1].as<double>()});
      }
      names += (z["name"] ? z["name"].as<std::string>() : "zone") + " (" + std::to_string(poly.size()) + " pts) ";
      zones_.push_back(poly);
    }
    RCLCPP_INFO(get_logger(), "NDT-only zones from %s: %s", path.c_str(), names.c_str());
  }

  bool in_zone(double x, double y) const
  {
    for (const auto & poly : zones_) {                    // ray casting
      bool inside = false;
      for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const auto & p1 = poly[j];
        const auto & p2 = poly[i];
        if ((p1[1] > y) != (p2[1] > y) && x < (p2[0] - p1[0]) * (y - p1[1]) / (p2[1] - p1[1]) + p1[0]) {
          inside = !inside;
        }
      }
      if (inside) {
        return true;
      }
    }
    return false;
  }

  void set_mode(const std::string & mode)
  {
    if (mode != mode_) {
      RCLCPP_INFO(get_logger(), "initial pose source: %s -> %s", mode_.c_str(), mode.c_str());
      mode_ = mode;
      std_msgs::msg::String s;
      s.data = mode;
      pub_mode_->publish(s);
    }
  }

  // ---- GNSS / NDT selection (timer) -------------------------------------------------------------
  void on_gnss_timer()
  {
    if (!gnss_state_) {
      return;
    }
    const rclcpp::Time now_t = get_clock()->now();
    const double t = now_t.seconds();
    const auto & g0 = *gnss_state_;
    const double dt = t - g0[0];
    const double h_var = g0[5];
    std::optional<std::array<double, 2>> ant;
    if (dt >= 0.0 && dt <= max_extrap_) {
      ant = std::array<double, 2>{g0[1] + g0[3] * dt, g0[2] + g0[4] * dt};
    }
    if (source_ == "gnss") {
      if (ant && gnss_yaw_) {
        const auto b = base_from_antenna((*ant)[0], (*ant)[1], *gnss_yaw_);
        publish(t, now_t, b[0], b[1], *gnss_yaw_, h_var, 0.05);
      }
      return;
    }
    const auto p = ndt_predict(t);
    if (!p) {                                             // no NDT yet / NDT lost: (re)initialise from GNSS
      const auto yaw0 = straight_heading();
      if (ant && yaw0 && (gnss_fixed_ || gnss_hacc_ <= init_max_hacc_)) {
        set_mode("gnss_init");
        ++count_["gnss"];
        const auto b = base_from_antenna((*ant)[0], (*ant)[1], *yaw0);
        publish(t, now_t, b[0], b[1], *yaw0, h_var, 0.05);
      } else {
        set_mode("none");
        if (ant) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                               "waiting for straight motion on good GNSS to initialise NDT");
        }
      }
      return;
    }
    // GNSS position, NDT yaw (the course of the antenna is not the body heading)
    std::optional<Pred> g;
    if (ant) {
      const auto b = base_from_antenna((*ant)[0], (*ant)[1], p->yaw);
      g = Pred{b[0], b[1], p->yaw};
    }
    bool ok = g.has_value();
    const bool zone = in_zone(p->x, p->y);
    const bool far = ok && !zone && gnss_hacc_ <= s_hacc_ &&
                     std::hypot(g->x - p->x, g->y - p->y) > std::max(s_min_, s_k_ * gnss_hacc_);
    if (far) {
      if (!sanity_since_) {
        sanity_since_ = t;
      }
    } else {
      sanity_since_.reset();
    }
    if (far && t - *sanity_since_ >= s_time_) {
      // NDT is lost: its yaw may be wrong too; use the straight-motion heading when there is one
      if (const auto yaw0 = straight_heading()) {
        const auto b = base_from_antenna((*ant)[0], (*ant)[1], *yaw0);
        g = Pred{b[0], b[1], *yaw0};
      }
      set_mode("gnss_sanity");
      ++count_["gnss_sanity"];
      publish(t, now_t, g->x, g->y, g->yaw, h_var, 0.05);
      return;
    }
    if (ok && !(gnss_fixed_ || (accept_float_ && gnss_hacc_ <= float_max_hacc_))) {
      ok = false;
      ++count_["gnss_rejected_quality"];
    }
    if (ok && zone) {
      ok = false;
      ++count_["gnss_rejected_zone"];
    }
    const bool mismatch =
      ok && std::hypot(g->x - p->x, g->y - p->y) > (gnss_fixed_ ? max_diff_fixed_ : max_diff_float_);
    if (mismatch && gnss_fixed_) {
      if (!mismatch_since_) {
        mismatch_since_ = t;
      }
    } else {
      mismatch_since_.reset();
    }
    if (mismatch && mismatch_since_ && t - *mismatch_since_ >= override_time_) {
      set_mode("gnss_override");                          // NDT slid: fixed GNSS wins
      ++count_["gnss_override"];
      publish(t, now_t, g->x, g->y, g->yaw, h_var, 0.05);
      return;
    }
    if (mismatch) {
      ok = false;
      ++count_["gnss_rejected_diff"];
    }
    if (ok) {
      if (!gnss_ok_since_) {
        gnss_ok_since_ = t;
      }
    } else {
      gnss_ok_since_.reset();
    }
    const bool gnss_mode = mode_ == "gnss" || mode_ == "gnss_init" || mode_ == "gnss_override" ||
                           mode_ == "gnss_sanity";
    if (ok && (gnss_mode || t - *gnss_ok_since_ >= reacquire_)) {
      set_mode("gnss");
      ++count_["gnss"];
      publish(t, now_t, g->x, g->y, g->yaw, h_var, 0.01);
    } else {
      set_mode("ndt");
      ++count_["dead_reckoned"];
      publish(t, now_t, p->x, p->y, p->yaw, 0.25, 0.01);
    }
  }

  // ---- output ---------------------------------------------------------------------------------
  void publish(double t, const builtin_interfaces::msg::Time & stamp, double x, double y, double yaw, double xy_var,
               double yaw_var)
  {
    double z, roll = 0.0, pitch = 0.0;
    if (ndt_ && std::abs(t - (*ndt_)[0]) < feedback_timeout_) {
      z = (*ndt_)[1];
      roll = (*ndt_)[2];
      pitch = (*ndt_)[3];
      ++count_["from_ndt"];
    } else if (gnss_z_) {
      z = *gnss_z_;
      ++count_["from_gnss_z"];
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no z yet (no NDT result, no GNSS fix)");
      return;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;
    msg.pose.pose.orientation = euler_to_quat(roll, pitch, yaw);
    auto & c = msg.pose.covariance;
    c.fill(0.0);
    c[0] = c[7] = std::max(xy_var, 0.01);
    c[14] = 1.0;
    c[21] = c[28] = 0.01;
    c[35] = std::max(yaw_var, 0.01);
    pub_->publish(msg);
    ++count_["published"];
    trigger_once();
  }

  void trigger_once()
  {
    if (trigger_done_ || trigger_pending_) {
      return;
    }
    if (!trigger_cli_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for NDT trigger service");
      return;
    }
    trigger_pending_ = true;
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    trigger_cli_->async_send_request(req, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
      trigger_pending_ = false;
      const auto res = f.get();
      if (res && res->success) {
        trigger_done_ = true;
        RCLCPP_INFO(get_logger(), "NDT activated (trigger_node: %s)", res->message.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "NDT trigger failed, retrying: %s", res ? res->message.c_str() : "no response");
      }
    });
  }

  void report()
  {
    RCLCPP_INFO(get_logger(), "initial poses published %ld (z/roll/pitch from NDT %ld, from GNSS height %ld, "
                "dead-reckoned from NDT %ld)", count_["published"], count_["from_ndt"], count_["from_gnss_z"],
                count_["dead_reckoned"]);
    if (source_ == "gnss_ndt") {
      RCLCPP_INFO(get_logger(), "mode %s | from GNSS %ld, GNSS rejected: quality %ld, zone %ld, NDT mismatch %ld, "
                  "fixed GNSS override %ld, NDT-lost GNSS reset %ld", mode_.c_str(), count_["gnss"],
                  count_["gnss_rejected_quality"], count_["gnss_rejected_zone"], count_["gnss_rejected_diff"],
                  count_["gnss_override"], count_["gnss_sanity"]);
    }
  }

  // parameters
  std::string source_, map_frame_, yaw_rate_source_;
  MapProjection proj_;
  std::array<double, 3> antenna_{};
  double min_speed_, feedback_timeout_, max_extrap_, ndt_window_, ndt_max_extrap_;
  bool accept_float_;
  double float_max_hacc_, max_diff_fixed_, max_diff_float_, reacquire_, override_time_;
  double s_hacc_, s_min_, s_k_, s_time_, init_max_hacc_, max_speed_, jump_reject_;
  double init_window_, init_min_dist_, init_straight_;
  std::vector<std::vector<std::array<double, 2>>> zones_;
  // state
  std::optional<std::array<double, 4>> ndt_;              // (t, z, roll, pitch) of the latest NDT result
  std::optional<double> gnss_z_, gnss_yaw_;
  std::optional<std::array<double, 6>> gnss_state_;       // (t, antenna x, y, vx, vy, xy_var), map frame
  bool gnss_fixed_{false};
  double gnss_hacc_{std::numeric_limits<double>::infinity()};
  std::deque<Track> track_;
  std::deque<Fix> fix_hist_;
  double gyro_z_{0.0}, last_v_{0.0}, last_w_{0.0};
  std::optional<double> mismatch_since_, sanity_since_, gnss_ok_since_;
  std::string mode_{"none"};
  std::map<std::string, long> count_;
  bool trigger_done_{false}, trigger_pending_{false};
  // ROS
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_ndt_;
  rclcpp::Subscription<ublox_msgs::msg::NavPVT>::SharedPtr sub_navpvt_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr trigger_cli_;
  rclcpp::TimerBase::SharedPtr gnss_timer_, report_timer_;
};

}  // namespace ndt_localization_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ndt_localization_cpp::InitialPoseBridge)
