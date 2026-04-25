#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include <avg_msgs/msg/avg_localization_msgs.hpp>
#include <avg_msgs/msg/module_state.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

// Stores one CSV pose row decoded from Kimera absolute/local trajectory outputs.
struct CsvPoseSample
{
  int64_t timestamp_ns{0};
  double tx{0.0};
  double ty{0.0};
  double tz{0.0};
  double qw{1.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  std::string source{};
};

std::string trim(const std::string & in)
{
  // Removes leading/trailing whitespace from one CSV token.
  const auto first = in.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = in.find_last_not_of(" \t\r\n");
  return in.substr(first, last - first + 1);
}

std::vector<std::string> splitCsv(const std::string & line)
{
  // Splits a comma-delimited line into trimmed token strings.
  std::vector<std::string> tokens;
  std::string token;
  std::stringstream ss(line);
  while (std::getline(ss, token, ',')) {
    tokens.push_back(trim(token));
  }
  return tokens;
}

bool parseInt64(const std::string & token, int64_t * out)
{
  // Parses a signed integer token and reports parse validity.
  if (!out) return false;
  try {
    *out = std::stoll(token);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parseDouble(const std::string & token, double * out)
{
  // Parses a finite floating-point token and rejects NaN/Inf values.
  if (!out) return false;
  try {
    *out = std::stod(token);
    return std::isfinite(*out);
  } catch (const std::exception &) {
    return false;
  }
}

int64_t normalizeTimestampNs(const int64_t timestamp_raw)
{
  // Accepts either second-based or nanosecond-based timestamps.
  // HH_260209: Support both seconds and nanoseconds in CSV inputs.
  if (timestamp_raw > 0 && timestamp_raw < 1000000000000LL) {
    return timestamp_raw * 1000000000LL;
  }
  return timestamp_raw;
}

builtin_interfaces::msg::Time toBuiltinTime(const int64_t timestamp_ns)
{
  // Converts nanoseconds to ROS builtin time representation.
  builtin_interfaces::msg::Time t;
  const int64_t clamped = std::max<int64_t>(0, timestamp_ns);
  t.sec = static_cast<int32_t>(clamped / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(clamped % 1000000000LL);
  return t;
}

void normalizeQuaternion(double * qw, double * qx, double * qy, double * qz)
{
  // Normalizes quaternion values and falls back to identity if norm is invalid.
  const double n =
    std::sqrt((*qw) * (*qw) + (*qx) * (*qx) + (*qy) * (*qy) + (*qz) * (*qz));
  if (n <= std::numeric_limits<double>::epsilon()) {
    *qw = 1.0;
    *qx = 0.0;
    *qy = 0.0;
    *qz = 0.0;
    return;
  }
  *qw /= n;
  *qx /= n;
  *qy /= n;
  *qz /= n;
}

double clampUnit(const double x)
{
  // Clamps one scalar into [-1, 1] for robust acos/asin inputs.
  return std::max(-1.0, std::min(1.0, x));
}

double wrapToPi(double angle_rad)
{
  // Normalizes one angle into [-pi, pi].
  static constexpr double kPi = 3.14159265358979323846;
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double yawFromQuaternion(
  const double qw, const double qx, const double qy, const double qz)
{
  // Extracts yaw from quaternion using ZYX intrinsic convention.
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

double angularDistanceQuaternion(
  const double qw0, const double qx0, const double qy0, const double qz0,
  const double qw1, const double qx1, const double qy1, const double qz1)
{
  // Computes shortest 3D rotation angle between two unit quaternions.
  const double dot =
    std::abs(qw0 * qw1 + qx0 * qx1 + qy0 * qy1 + qz0 * qz1);
  return 2.0 * std::acos(clampUnit(dot));
}

bool parsePoseSampleLine(const std::string & line, CsvPoseSample * sample)
{
  // Parses one Kimera CSV row into a pose sample model.
  if (!sample) return false;
  if (line.empty() || line[0] == '#') return false;

  const auto fields = splitCsv(line);
  // Expected minimum:
  // timestamp_ns,x,y,z,roll,pitch,yaw,qw,qx,qy,qz
  if (fields.size() < 11u) return false;

  int64_t timestamp_raw = 0;
  if (!parseInt64(fields[0], &timestamp_raw)) return false;
  sample->timestamp_ns = normalizeTimestampNs(timestamp_raw);

  if (!parseDouble(fields[1], &sample->tx)) return false;
  if (!parseDouble(fields[2], &sample->ty)) return false;
  if (!parseDouble(fields[3], &sample->tz)) return false;
  if (!parseDouble(fields[7], &sample->qw)) return false;
  if (!parseDouble(fields[8], &sample->qx)) return false;
  if (!parseDouble(fields[9], &sample->qy)) return false;
  if (!parseDouble(fields[10], &sample->qz)) return false;

  normalizeQuaternion(&sample->qw, &sample->qx, &sample->qy, &sample->qz);

  // HH_260209: zedLiveVIO abs CSV appends "source" as the last column.
  if (fields.size() >= 23u) {
    sample->source = fields.back();
  } else {
    sample->source.clear();
  }
  return true;
}

// Tracks tail-read offset and returns the latest valid pose sample from CSV.
class CsvTailReader
{
public:
  explicit CsvTailReader(std::string csv_path = "")
  : csv_path_(std::move(csv_path))
  {
  }

  void setPath(const std::string & csv_path)
  {
    csv_path_ = csv_path;
    read_offset_bytes_ = 0;
  }

  const std::string & path() const { return csv_path_; }

  bool pollLatest(CsvPoseSample * latest)
  {
    if (!latest || csv_path_.empty()) return false;
    if (!std::filesystem::exists(csv_path_)) return false;

    const std::uintmax_t file_size = std::filesystem::file_size(csv_path_);
    if (file_size < read_offset_bytes_) {
      // HH_260209: Handle log rotation/truncation gracefully.
      read_offset_bytes_ = 0;
    }

    std::ifstream in(csv_path_);
    if (!in.good()) return false;

    in.seekg(static_cast<std::streamoff>(read_offset_bytes_), std::ios::beg);
    CsvPoseSample parsed{};
    bool got_new_sample = false;
    std::string line;
    while (std::getline(in, line)) {
      if (parsePoseSampleLine(line, &parsed)) {
        last_sample_ = parsed;
        got_new_sample = true;
      }
    }

    read_offset_bytes_ = file_size;
    if (!got_new_sample) return false;

    *latest = last_sample_;
    return true;
  }

private:
  std::string csv_path_;
  std::uintmax_t read_offset_bytes_{0};
  CsvPoseSample last_sample_{};
};

}  // namespace

class KimeraCsvBridgeNode : public rclcpp::Node
{
public:
  // Declares bridge parameters and starts periodic CSV polling timer.
  KimeraCsvBridgeNode()
  : Node("kimera_csv_bridge")
  {
    // HH_260209: Bridge zedLiveVIO CSV outputs into ROS topics for fallback localization.
    abs_csv_path_ = declare_parameter<std::string>("abs_csv_path", "");
    local_csv_path_ = declare_parameter<std::string>("local_csv_path", "");
    publish_abs_ = declare_parameter<bool>("publish_abs", true);
    publish_local_ = declare_parameter<bool>("publish_local", true);
    poll_hz_ = declare_parameter<double>("poll_hz", 20.0);
    poll_hz_ = std::max(1.0, poll_hz_);
    // HH_260422: Reject implausible Kimera jumps to prevent downstream fly-away behavior.
    abs_enable_outlier_rejection_ = declare_parameter<bool>(
      "abs_enable_outlier_rejection", true);
    local_enable_outlier_rejection_ = declare_parameter<bool>(
      "local_enable_outlier_rejection", true);
    outlier_reseed_gap_sec_ = declare_parameter<double>(
      "outlier_reseed_gap_sec", 2.5);
    outlier_max_step_distance_m_ = declare_parameter<double>(
      "outlier_max_step_distance_m", 1.2);
    outlier_max_linear_speed_mps_ = declare_parameter<double>(
      "outlier_max_linear_speed_mps", 1.2);
    outlier_max_yaw_rate_radps_ = declare_parameter<double>(
      "outlier_max_yaw_rate_radps", 1.0);
    outlier_max_orientation_delta_rad_ = declare_parameter<double>(
      "outlier_max_orientation_delta_rad", 0.6);
    outlier_reseed_gap_sec_ = std::max(0.0, outlier_reseed_gap_sec_);
    outlier_max_step_distance_m_ = std::max(0.01, outlier_max_step_distance_m_);
    outlier_max_linear_speed_mps_ = std::max(0.05, outlier_max_linear_speed_mps_);
    outlier_max_yaw_rate_radps_ = std::max(0.05, outlier_max_yaw_rate_radps_);
    outlier_max_orientation_delta_rad_ = std::max(0.05, outlier_max_orientation_delta_rad_);

    map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
    local_frame_id_ = declare_parameter<std::string>("local_frame_id", "vio_local");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "robot_base_link");

    abs_pose_topic_ = declare_parameter<std::string>(
      "abs_pose_topic", "/localization/kimera_vio/pose");
    abs_pose_cov_topic_ = declare_parameter<std::string>(
      "abs_pose_cov_topic", "/localization/kimera_vio/pose_with_covariance");
    abs_odom_topic_ = declare_parameter<std::string>(
      "abs_odom_topic", "/localization/kimera_vio/odometry");
    local_pose_topic_ = declare_parameter<std::string>(
      "local_pose_topic", "/localization/kimera_vio/local_pose");
    local_odom_topic_ = declare_parameter<std::string>(
      "local_odom_topic", "/localization/kimera_vio/local_odometry");
    source_topic_ = declare_parameter<std::string>(
      "source_topic", "/localization/kimera_vio/source");
    // HH_260326: Use status/state parameter names consistently.
    publish_localization_status_ = declare_parameter<bool>(
      "publish_localization_status", false);
    localization_status_topic_ = declare_parameter<std::string>(
      "localization_status_topic", "/localization/status");

    position_stddev_m_ = declare_parameter<double>("position_stddev_m", 0.5);
    orientation_stddev_rad_ = declare_parameter<double>("orientation_stddev_rad", 0.35);
    position_stddev_m_ = std::max(1e-6, position_stddev_m_);
    orientation_stddev_rad_ = std::max(1e-6, orientation_stddev_rad_);

    abs_reader_.setPath(abs_csv_path_);
    local_reader_.setPath(local_csv_path_);

    // HH_260209: Keep latest sample latched for late subscribers without periodic republish.
    rclcpp::QoS latched_qos(rclcpp::KeepLast(1));
    latched_qos.transient_local().reliable();

    abs_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(abs_pose_topic_, latched_qos);
    abs_pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      abs_pose_cov_topic_, latched_qos);
    abs_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(abs_odom_topic_, latched_qos);
    local_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(local_pose_topic_, latched_qos);
    local_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(local_odom_topic_, latched_qos);
    source_pub_ = create_publisher<std_msgs::msg::String>(source_topic_, latched_qos);
    if (publish_localization_status_) {
      localization_status_pub_ = create_publisher<avg_msgs::msg::AvgLocalizationMsgs>(
        localization_status_topic_, rclcpp::QoS(10));
    }

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / poll_hz_)),
      std::bind(&KimeraCsvBridgeNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "kimera_csv_bridge started: publish_abs=%d publish_local=%d poll_hz=%.1f abs_csv='%s' local_csv='%s' gate(abs=%d local=%d step<=%.2fm speed<=%.2fm/s yaw_rate<=%.2frad/s dR<=%.2frad reseed_gap>=%.2fs)",
      publish_abs_ ? 1 : 0, publish_local_ ? 1 : 0, poll_hz_,
      abs_csv_path_.c_str(), local_csv_path_.c_str(),
      abs_enable_outlier_rejection_ ? 1 : 0, local_enable_outlier_rejection_ ? 1 : 0,
      outlier_max_step_distance_m_, outlier_max_linear_speed_mps_,
      outlier_max_yaw_rate_radps_, outlier_max_orientation_delta_rad_,
      outlier_reseed_gap_sec_);
  }

private:
  struct StreamOutlierGateState
  {
    bool has_last{false};
    CsvPoseSample last{};
  };

  // Validates one sample against previous accepted state for the same stream.
  bool passesOutlierGate(
    const CsvPoseSample & current, bool absolute_pose, std::string * reject_reason)
  {
    const bool gate_enabled =
      absolute_pose ? abs_enable_outlier_rejection_ : local_enable_outlier_rejection_;
    auto & gate_state = absolute_pose ? abs_outlier_gate_state_ : local_outlier_gate_state_;
    if (!gate_enabled) {
      gate_state.has_last = true;
      gate_state.last = current;
      return true;
    }

    if (!gate_state.has_last) {
      gate_state.has_last = true;
      gate_state.last = current;
      return true;
    }

    const auto & prev = gate_state.last;
    if (current.timestamp_ns <= prev.timestamp_ns) {
      if (reject_reason) {
        *reject_reason = "non_monotonic_timestamp";
      }
      return false;
    }

    const double dt = static_cast<double>(current.timestamp_ns - prev.timestamp_ns) * 1e-9;
    if (dt <= 0.0) {
      if (reject_reason) {
        *reject_reason = "non_positive_dt";
      }
      return false;
    }

    // HH_260422: After long source gaps, re-seed baseline but do not publish
    // the first post-gap sample. This prevents one-shot fly-away jumps from
    // being accepted as valid state after tracking recovers.
    if (outlier_reseed_gap_sec_ > 0.0 && dt >= outlier_reseed_gap_sec_) {
      gate_state.last = current;
      if (reject_reason) {
        std::ostringstream oss;
        oss << "reseed_after_gap dt_sec=" << dt;
        *reject_reason = oss.str();
      }
      return false;
    }

    const double dx = current.tx - prev.tx;
    const double dy = current.ty - prev.ty;
    const double dz = current.tz - prev.tz;
    const double step_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double speed_mps = step_m / dt;

    const double yaw_prev = yawFromQuaternion(prev.qw, prev.qx, prev.qy, prev.qz);
    const double yaw_curr = yawFromQuaternion(current.qw, current.qx, current.qy, current.qz);
    const double yaw_delta = std::abs(wrapToPi(yaw_curr - yaw_prev));
    const double yaw_rate = yaw_delta / dt;

    const double rotation_delta = angularDistanceQuaternion(
      prev.qw, prev.qx, prev.qy, prev.qz,
      current.qw, current.qx, current.qy, current.qz);

    // HH_260422: Allow larger per-sample step when sample interval is long,
    // but still cap absolute step to reject impossible fly-away jumps.
    const double dynamic_step_limit_m =
      outlier_max_linear_speed_mps_ * dt * 1.25;
    const double allowed_step_m =
      std::min(outlier_max_step_distance_m_, dynamic_step_limit_m);
    if (step_m > allowed_step_m) {
      if (reject_reason) {
        std::ostringstream oss;
        oss << "step_exceeded step_m=" << step_m << " allowed_m=" << allowed_step_m;
        *reject_reason = oss.str();
      }
      return false;
    }
    if (speed_mps > outlier_max_linear_speed_mps_) {
      if (reject_reason) {
        std::ostringstream oss;
        oss << "speed_exceeded speed_mps=" << speed_mps;
        *reject_reason = oss.str();
      }
      return false;
    }
    if (yaw_rate > outlier_max_yaw_rate_radps_) {
      if (reject_reason) {
        std::ostringstream oss;
        oss << "yaw_rate_exceeded yaw_rate_radps=" << yaw_rate;
        *reject_reason = oss.str();
      }
      return false;
    }
    if (rotation_delta > outlier_max_orientation_delta_rad_) {
      if (reject_reason) {
        std::ostringstream oss;
        oss << "rotation_exceeded delta_rad=" << rotation_delta;
        *reject_reason = oss.str();
      }
      return false;
    }

    gate_state.last = current;
    return true;
  }

  // Logs rate + outlier statistics periodically.
  void logPeriodicStats(const rclcpp::Time & now)
  {
    if ((now - last_stat_log_time_).seconds() < 1.0) {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "kimera_csv_bridge rates: abs_hz=%zu local_hz=%zu abs_reject=%zu local_reject=%zu last_abs_ns=%ld last_local_ns=%ld",
      abs_publish_count_, local_publish_count_, abs_reject_count_, local_reject_count_,
      last_abs_timestamp_ns_, last_local_timestamp_ns_);
    abs_publish_count_ = 0;
    local_publish_count_ = 0;
    abs_reject_count_ = 0;
    local_reject_count_ = 0;
    last_stat_log_time_ = now;
  }

  // Builds a diagonal pose covariance matrix from configured standard deviations.
  std::array<double, 36> makePoseCovariance() const
  {
    std::array<double, 36> cov{};
    cov.fill(0.0);
    const double p_var = position_stddev_m_ * position_stddev_m_;
    const double a_var = orientation_stddev_rad_ * orientation_stddev_rad_;
    cov[0] = p_var;
    cov[7] = p_var;
    cov[14] = p_var;
    cov[21] = a_var;
    cov[28] = a_var;
    cov[35] = a_var;
    return cov;
  }

  // Returns high-variance twist covariance because CSV stream has no twist estimate.
  static std::array<double, 36> makeTwistUnknownCovariance()
  {
    std::array<double, 36> cov{};
    cov.fill(0.0);
    cov[0] = cov[7] = cov[14] = 1e4;
    cov[21] = cov[28] = cov[35] = 1e4;
    return cov;
  }

  // Converts one CSV sample to a ROS pose stamped in the selected frame.
  void fillPose(
    const CsvPoseSample & s, const std::string & frame_id,
    geometry_msgs::msg::PoseStamped * pose) const
  {
    pose->header.stamp = toBuiltinTime(s.timestamp_ns);
    pose->header.frame_id = frame_id;
    pose->pose.position.x = s.tx;
    pose->pose.position.y = s.ty;
    pose->pose.position.z = s.tz;
    pose->pose.orientation.w = s.qw;
    pose->pose.orientation.x = s.qx;
    pose->pose.orientation.y = s.qy;
    pose->pose.orientation.z = s.qz;
  }

  // Publishes one decoded CSV sample to absolute or local topic set.
  void publishSample(
    const CsvPoseSample & s, bool absolute_pose)
  {
    const std::string & frame_id = absolute_pose ? map_frame_id_ : local_frame_id_;
    geometry_msgs::msg::PoseStamped pose_msg;
    fillPose(s, frame_id, &pose_msg);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
    pose_cov_msg.header = pose_msg.header;
    pose_cov_msg.pose.pose = pose_msg.pose;
    pose_cov_msg.pose.covariance = makePoseCovariance();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = base_frame_id_;
    odom_msg.pose.pose = pose_msg.pose;
    odom_msg.pose.covariance = makePoseCovariance();
    odom_msg.twist.covariance = makeTwistUnknownCovariance();

    if (absolute_pose) {
      abs_pose_pub_->publish(pose_msg);
      abs_pose_cov_pub_->publish(pose_cov_msg);
      abs_odom_pub_->publish(odom_msg);

      if (!s.source.empty()) {
        std_msgs::msg::String source_msg;
        source_msg.data = s.source;
        source_pub_->publish(source_msg);
      }
    } else {
      local_pose_pub_->publish(pose_msg);
      local_odom_pub_->publish(odom_msg);
    }

    if (publish_localization_status_ && localization_status_pub_) {
      avg_msgs::msg::AvgLocalizationMsgs avg_msg;
      avg_msg.stamp = pose_msg.header.stamp;
      avg_msg.localization_pose = pose_msg;
      avg_msg.localization_pose_cov = pose_cov_msg;
      avg_msg.localization_odom = odom_msg;
      avg_msg.state.stamp = pose_msg.header.stamp;
      avg_msg.state.module_name = "localization";
      avg_msg.state.level = avg_msgs::msg::ModuleState::OK;
      avg_msg.state.message = absolute_pose ? "kimera_csv_abs" : "kimera_csv_local";
      localization_status_pub_->publish(avg_msg);
    }
  }

  // Polls both CSV streams, publishes new samples, and reports publish rates.
  void onTimer()
  {
    if (publish_abs_) {
      CsvPoseSample abs_sample{};
      if (abs_reader_.pollLatest(&abs_sample) && abs_sample.timestamp_ns > last_abs_timestamp_ns_) {
        std::string reject_reason = "gate_rejected";
        if (passesOutlierGate(abs_sample, true, &reject_reason)) {
          publishSample(abs_sample, true);
          last_abs_timestamp_ns_ = abs_sample.timestamp_ns;
          abs_publish_count_++;
        } else {
          abs_reject_count_++;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Rejected absolute Kimera CSV sample: %s", reject_reason.c_str());
        }
      }
    }

    if (publish_local_) {
      CsvPoseSample local_sample{};
      if (local_reader_.pollLatest(&local_sample) && local_sample.timestamp_ns > last_local_timestamp_ns_) {
        std::string reject_reason = "gate_rejected";
        if (passesOutlierGate(local_sample, false, &reject_reason)) {
          publishSample(local_sample, false);
          last_local_timestamp_ns_ = local_sample.timestamp_ns;
          local_publish_count_++;
        } else {
          local_reject_count_++;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Rejected local Kimera CSV sample: %s", reject_reason.c_str());
        }
      }
    }

    logPeriodicStats(this->now());
  }

private:
  // Parameters
  std::string abs_csv_path_;
  std::string local_csv_path_;
  bool publish_abs_{true};
  bool publish_local_{true};
  double poll_hz_{20.0};
  bool abs_enable_outlier_rejection_{false};
  bool local_enable_outlier_rejection_{true};
  double outlier_reseed_gap_sec_{1.0};
  double outlier_max_step_distance_m_{1.2};
  double outlier_max_linear_speed_mps_{2.5};
  double outlier_max_yaw_rate_radps_{2.5};
  double outlier_max_orientation_delta_rad_{1.5};
  std::string map_frame_id_{"map"};
  std::string local_frame_id_{"vio_local"};
  std::string base_frame_id_{"robot_base_link"};
  std::string abs_pose_topic_;
  std::string abs_pose_cov_topic_;
  std::string abs_odom_topic_;
  std::string local_pose_topic_;
  std::string local_odom_topic_;
  std::string source_topic_;
  std::string localization_status_topic_;
  double position_stddev_m_{0.5};
  double orientation_stddev_rad_{0.35};
  bool publish_localization_status_{false};

  // Readers/state
  CsvTailReader abs_reader_;
  CsvTailReader local_reader_;
  int64_t last_abs_timestamp_ns_{-1};
  int64_t last_local_timestamp_ns_{-1};
  std::size_t abs_publish_count_{0};
  std::size_t local_publish_count_{0};
  std::size_t abs_reject_count_{0};
  std::size_t local_reject_count_{0};
  rclcpp::Time last_stat_log_time_{0, 0, RCL_ROS_TIME};
  StreamOutlierGateState abs_outlier_gate_state_{};
  StreamOutlierGateState local_outlier_gate_state_{};

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr abs_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr abs_pose_cov_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr abs_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr local_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_pub_;
  rclcpp::Publisher<avg_msgs::msg::AvgLocalizationMsgs>::SharedPtr localization_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  // Starts the CSV bridge node lifecycle.
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KimeraCsvBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
