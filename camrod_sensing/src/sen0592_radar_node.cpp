#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <sys/ioctl.h>
#include <termios.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <avg_msgs/conversions.hpp>
#include <avg_msgs/msg/avg_bool.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
// HH_260720 - Publish SEN0592 measurements on the generated CAMROD range contract.
#include <avg_msgs/msg/avg_range.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <avg_msgs/msg/avg_sensing_radar.hpp>

#include "camrod_sensing/radar_dummy_contract.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr uint16_t kRegRealtimeValue = 0x0101;   // DFRobot SEN0592 real-time distance (mm)
// HH_260729 - DFRobot defines 0x0208 as detection-angle level 1..4
// (4 is the factory default/widest), not a degree value.
constexpr uint16_t kRegAngleConfig    = 0x0208;
// HH_260729 - DFRobot defines 0x021F as range level 1..5, approximately
// 0.5/1.5/2.5/3.5/5.0 m. It does not accept a distance in millimetres.
constexpr uint16_t kRegRangeConfig    = 0x021F;
constexpr uint8_t  kFunctionReadHolding = 0x03;
constexpr uint8_t  kFunctionWriteSingle = 0x06;  // Modbus FC 0x06: write single register
constexpr size_t   kRespLen = 7;                 // [id][fc][len][hi][lo][crc_lo][crc_hi]
constexpr int kStartupReadbackAttempts = 5;
constexpr std::uint64_t kReconnectAfterConsecutiveMisses = 5U;

// Implements `modbus_crc` behavior.
uint16_t modbus_crc(const std::vector<uint8_t>& data)
{
  uint16_t crc = 0xFFFF;
  for (auto b : data) {
    crc ^= b;
    for (int i = 0; i < 8; ++i) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else              crc = (crc >> 1);
    }
  }
  return crc;
}

// Implements `make_read_req` behavior.
std::vector<uint8_t> make_read_req(uint8_t slave_id, uint16_t reg, uint16_t count = 1)
{
  std::vector<uint8_t> p = {
    slave_id,
    kFunctionReadHolding,
    static_cast<uint8_t>((reg >> 8) & 0xFF),
    static_cast<uint8_t>(reg & 0xFF),
    static_cast<uint8_t>((count >> 8) & 0xFF),
    static_cast<uint8_t>(count & 0xFF)
  };
  uint16_t crc = modbus_crc(p);
  p.push_back(static_cast<uint8_t>(crc & 0xFF));
  p.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return p;
}

// Builds a Modbus FC 0x06 (write single register) request frame.
std::vector<uint8_t> make_write_req(uint8_t slave_id, uint16_t reg, uint16_t value)
{
  std::vector<uint8_t> p = {
    slave_id,
    kFunctionWriteSingle,
    static_cast<uint8_t>((reg >> 8) & 0xFF),
    static_cast<uint8_t>(reg & 0xFF),
    static_cast<uint8_t>((value >> 8) & 0xFF),
    static_cast<uint8_t>(value & 0xFF)
  };
  uint16_t crc = modbus_crc(p);
  p.push_back(static_cast<uint8_t>(crc & 0xFF));
  p.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  return p;
}

std::string describe_range_level(const int level)
{
  switch (level) {
    case 1: return "1(~0.5m)";
    case 2: return "2(~1.5m)";
    case 3: return "3(~2.5m)";
    case 4: return "4(~3.5m)";
    case 5: return "5(~5.0m)";
    default: return std::to_string(level) + "(unmapped)";
  }
}

double approximate_range_level_m(const int level)
{
  switch (level) {
    case 1: return 0.5;
    case 2: return 1.5;
    case 3: return 2.5;
    case 4: return 3.5;
    case 5: return 5.0;
    default: return 0.0;
  }
}

// Implements `baud_to_termios` behavior.
speed_t baud_to_termios(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return B115200;
  }
}

// Implements `configure_serial` behavior.
bool configure_serial(int fd, int baud, int vtime_ds = 1, int vmin = 0)
{
  struct termios tty{};
  if (tcgetattr(fd, &tty) != 0) return false;

  cfmakeraw(&tty);
  speed_t spd = baud_to_termios(baud);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;   // no parity
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;       // 8 data bits
  tty.c_cflag &= ~CRTSCTS;  // no hw flow control

  // Non-blocking-ish read behavior:
  // VTIME in deciseconds, VMIN bytes minimum
  tty.c_cc[VTIME] = vtime_ds;  // e.g., 1 => 100ms
  tty.c_cc[VMIN]  = vmin;

  tcflush(fd, TCIOFLUSH);
  return (tcsetattr(fd, TCSANOW, &tty) == 0);
}

// Implements `read_exact_with_deadline` behavior.
bool read_exact_with_deadline(int fd, uint8_t* buf, size_t nbytes, std::chrono::milliseconds deadline)
{
  auto t_end = std::chrono::steady_clock::now() + deadline;
  size_t total = 0;

  while (total < nbytes && std::chrono::steady_clock::now() < t_end) {
    ssize_t n = ::read(fd, buf + total, nbytes - total);
    if (n > 0) {
      total += static_cast<size_t>(n);
    } else if (n == 0) {
      // timeout slice (VTIME)
    } else {
      if (errno == EINTR) continue;
      return false;
    }
  }
  return total == nbytes;
}

// Implements `drain_serial_rx` behavior.
void drain_serial_rx(int fd)
{
  int available = 0;
  if (ioctl(fd, FIONREAD, &available) == 0 && available > 0) {
    std::vector<uint8_t> tmp(static_cast<size_t>(available));
    // HH_260623 - Keep the drain result explicit so the radar build stays warning-free.
    const ssize_t drained = ::read(fd, tmp.data(), tmp.size());
    (void)drained;
  }
}

}  // namespace

class Sen0592RadarNode : public rclcpp::Node
{
public:
  // HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.

  // Implements `Sen0592RadarNode` behavior.
  Sen0592RadarNode() : Node("sen0592_radar_node")
  {
    // ----- Parameters -----
    // HH_260729 - Most SEN0592 settings are captured by worker threads or
    // written to hardware only while opening the serial port. Mark them
    // read-only so `ros2 param set` cannot report success without changing the
    // running sensor. Restart the node after editing these startup parameters.
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    startup_only.description = "Startup-only SEN0592 setting; restart the node to apply";
    auto fov_metadata = startup_only;
    fov_metadata.description =
      "Startup-only Range-message metadata; does not configure the physical beam";
    auto hardware_write_gate = startup_only;
    hardware_write_gate.description =
      "Startup-only gate for 0x0208/0x021F writes; false still permits readback";
    auto sensor_enable = startup_only;
    sensor_enable.description =
      "Startup-only per-channel enable; false opens no port and publishes dummy heartbeats";
    auto disabled_dummy_rate = startup_only;
    disabled_dummy_rate.description =
      "Startup-only no-target/dummy_active heartbeat rate for disabled channels";
    auto angle_levels = startup_only;
    angle_levels.description =
      "Startup-only expected 0x0208 levels: 1..4=narrowest..widest";
    auto software_ranges = startup_only;
    software_ranges.description =
      "Startup-only exact software max ranges in metres; always enforced";
    auto hardware_range_levels_descriptor = startup_only;
    hardware_range_levels_descriptor.description =
      "Startup-only expected 0x021F levels 1..5 (~0.5/1.5/2.5/3.5/5.0 m)";

    this->declare_parameter<int>("baud", 115200, startup_only);
    this->declare_parameter<int>("slave_id", 1, startup_only);
    this->declare_parameter<double>("poll_period_s", 0.06, startup_only);
    this->declare_parameter<double>("response_deadline_s", 0.20, startup_only);
    this->declare_parameter<double>("inter_frame_sleep_s", 0.003, startup_only);
    this->declare_parameter<double>(
      "disabled_channel_dummy_publish_rate_hz", 2.0, disabled_dummy_rate);
    this->declare_parameter<double>("software_min_range_m", 0.02, startup_only);
    this->declare_parameter<double>("software_default_max_range_m", 4.5, startup_only);
    // This value is Range-message metadata. The physical beam angle is the
    // verified/written 0x0208 register described by hardware_angle_levels.
    this->declare_parameter<double>(
      "range_message_field_of_view_rad", 0.26, fov_metadata);
    this->declare_parameter<std::string>(
      "radar_status_topic", "/sensing/radar/status", startup_only);
    this->declare_parameter<bool>("publish_radar_status", false);
    this->declare_parameter<bool>("log_status", false);
    this->declare_parameter<bool>(
      "hardware_write_on_startup", false, hardware_write_gate);
    // HH_260729 - Sensor enable is explicit and independent of its physical
    // angle/range register values.
    this->declare_parameter<std::vector<bool>>(
      "sensor_enabled", std::vector<bool>{}, sensor_enable);
    // HH_260729 - Per-sensor detection angle level expected at 0x0208.
    // DFRobot supports levels 1..4. A complete array is verified exactly;
    // an empty standalone default requests readback validation without a
    // configured exact value. Order must match sensor_names.
    this->declare_parameter<std::vector<int>>(
      "hardware_angle_levels", std::vector<int>{}, angle_levels);
    // HH_260422 - Exact per-sensor software cutoff in metres, ordered
    // identically to sensor_names and carried in Range.max_range.
    this->declare_parameter<std::vector<double>>(
      "software_max_ranges_m", std::vector<double>{}, software_ranges);
    // HH_260729 - Per-sensor range level expected at 0x021F. Levels 1..5 are
    // approximately 0.5/1.5/2.5/3.5/5.0 m. A complete array is verified
    // exactly; an empty standalone default has no configured exact value.
    this->declare_parameter<std::vector<int>>(
      "hardware_range_levels", std::vector<int>{}, hardware_range_levels_descriptor);

    // Sensor definitions (name, frame_id, port, topic)
    // Default values are Linux examples; replace with your actual /dev paths or udev symlinks.
    // HH_260623 - Match the latest todo/camrod_sensing radar wiring: two front radars plus
    // side/rear radars on fixed CH9344 ports. The auto: resolver was removed so an
    // unexpected USB reorder fails visibly instead of silently binding the wrong sensor.
    sensor_names_ = this->declare_parameter<std::vector<std::string>>(
      "sensor_names",
      {"FRONT1", "FRONT2", "LEFT1", "LEFT2", "RIGHT1", "RIGHT2", "REAR"},
      startup_only);

    frame_ids_ = this->declare_parameter<std::vector<std::string>>(
      "frame_ids", {"radar_front1_link", "radar_front2_link", "radar_left1_link",
                    "radar_left2_link", "radar_right1_link", "radar_right2_link",
                    "radar_rear_link"}, startup_only);

    // HH_260702 - Default port order mirrors the current field harness where
    // logical LEFT sensors are on USB4/USB5 and RIGHT sensors are on USB2/USB3.
    ports_ = this->declare_parameter<std::vector<std::string>>(
      "ports", {
        "/dev/ttyCH9344USB0", "/dev/ttyCH9344USB1", "/dev/ttyCH9344USB4",
        "/dev/ttyCH9344USB5", "/dev/ttyCH9344USB2", "/dev/ttyCH9344USB3",
        "/dev/ttyCH9344USB6"
      }, startup_only);

    topics_ = this->declare_parameter<std::vector<std::string>>(
      "topics", {
        "/sensing/radar/front1/range", "/sensing/radar/front2/range",
        "/sensing/radar/left1/range", "/sensing/radar/left2/range",
        "/sensing/radar/right1/range", "/sensing/radar/right2/range",
        "/sensing/radar/rear/range"
      }, startup_only);
    // HH_260720 - Reserve standard Range topics for RViz and other ROS-native tools.
    standard_ros_topics_ = this->declare_parameter<std::vector<std::string>>(
      "standard_ros_topics", {
        "/sensing/radar/front1/range_ros", "/sensing/radar/front2/range_ros",
        "/sensing/radar/left1/range_ros", "/sensing/radar/left2/range_ros",
        "/sensing/radar/right1/range_ros", "/sensing/radar/right2/range_ros",
        "/sensing/radar/rear/range_ros"
      }, startup_only);

    baud_ = this->get_parameter("baud").as_int();
    const auto slave_id_value = this->get_parameter("slave_id").as_int();
    poll_period_s_ = this->get_parameter("poll_period_s").as_double();
    response_deadline_s_ = this->get_parameter("response_deadline_s").as_double();
    inter_frame_sleep_s_ = this->get_parameter("inter_frame_sleep_s").as_double();
    disabled_channel_dummy_publish_rate_hz_ =
      this->get_parameter("disabled_channel_dummy_publish_rate_hz").as_double();
    software_min_range_m_ = this->get_parameter("software_min_range_m").as_double();
    software_default_max_range_m_ =
      this->get_parameter("software_default_max_range_m").as_double();
    range_message_field_of_view_rad_ =
      this->get_parameter("range_message_field_of_view_rad").as_double();
    radar_status_topic_ = this->get_parameter("radar_status_topic").as_string();
    publish_radar_status_.store(this->get_parameter("publish_radar_status").as_bool());
    log_status_.store(this->get_parameter("log_status").as_bool());
    hardware_write_on_startup_ =
      this->get_parameter("hardware_write_on_startup").as_bool();
    const auto supported_baud =
      baud_ == 9600 || baud_ == 19200 || baud_ == 38400 ||
      baud_ == 57600 || baud_ == 115200;
    if (!supported_baud) {
      throw std::runtime_error("baud must be one of 9600, 19200, 38400, 57600, 115200");
    }
    if (slave_id_value < 1 || slave_id_value > 254) {
      throw std::runtime_error("slave_id must be in the SEN0592 range 1..254");
    }
    slave_id_ = static_cast<uint8_t>(slave_id_value);
    if (!std::isfinite(poll_period_s_) || poll_period_s_ <= 0.0 ||
      !std::isfinite(response_deadline_s_) || response_deadline_s_ <= 0.0 ||
      !std::isfinite(inter_frame_sleep_s_) || inter_frame_sleep_s_ < 0.0)
    {
      throw std::runtime_error(
              "poll_period_s/response_deadline_s must be positive and "
              "inter_frame_sleep_s must be non-negative");
    }
    if (!std::isfinite(disabled_channel_dummy_publish_rate_hz_) ||
      disabled_channel_dummy_publish_rate_hz_ <= 0.0 ||
      disabled_channel_dummy_publish_rate_hz_ > 20.0)
    {
      throw std::runtime_error(
              "disabled_channel_dummy_publish_rate_hz must be in (0, 20]");
    }
    if (!std::isfinite(software_min_range_m_) ||
      !std::isfinite(software_default_max_range_m_) ||
      software_min_range_m_ <= 0.0 ||
      software_default_max_range_m_ <= software_min_range_m_)
    {
      throw std::runtime_error(
              "software range limits require 0 < software_min_range_m "
              "< software_default_max_range_m");
    }
    if (!std::isfinite(range_message_field_of_view_rad_) ||
      range_message_field_of_view_rad_ <= 0.0)
    {
      throw std::runtime_error(
              "range_message_field_of_view_rad must be finite and positive");
    }
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & param : params) {
          if (param.get_name() == "log_status") {
            log_status_.store(param.as_bool());
          } else if (param.get_name() == "publish_radar_status") {
            publish_radar_status_.store(param.as_bool());
          }
        }
        return result;
      });

    const auto n = ports_.size();
    if (n == 0U) {
      throw std::runtime_error("at least one SEN0592 sensor definition is required");
    }
    if (
      sensor_names_.size() != n || frame_ids_.size() != n || topics_.size() != n ||
      standard_ros_topics_.size() != n)
    {
      throw std::runtime_error(
              "sensor_names, frame_ids, ports, topics, standard_ros_topics must have the same length");
    }
    const auto require_nonempty_unique =
      [](const std::vector<std::string> & values, const char * name) {
        for (std::size_t i = 0U; i < values.size(); ++i) {
          if (values[i].empty()) {
            throw std::runtime_error(
                    std::string(name) + " entries must not be empty");
          }
          if (std::find(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(i),
              values[i]) != values.begin() + static_cast<std::ptrdiff_t>(i))
          {
            throw std::runtime_error(
                    std::string(name) + " entries must be unique: " + values[i]);
          }
        }
      };
    // HH_260729 - A duplicated port/topic previously let two logical radars
    // appear configured while only one physical channel was actually usable.
    require_nonempty_unique(sensor_names_, "sensor_names");
    require_nonempty_unique(frame_ids_, "frame_ids");
    require_nonempty_unique(ports_, "ports");
    require_nonempty_unique(topics_, "topics");
    require_nonempty_unique(standard_ros_topics_, "standard_ros_topics");

    const auto sensor_enabled = this->get_parameter("sensor_enabled").as_bool_array();
    const auto software_max_ranges =
      this->get_parameter("software_max_ranges_m").as_double_array();
    const auto hardware_range_levels =
      this->get_parameter("hardware_range_levels").as_integer_array();
    const auto hardware_angle_levels =
      this->get_parameter("hardware_angle_levels").as_integer_array();
    const auto require_complete_sensor_array =
      [n](const std::size_t size, const char * name) {
        if (size != 0U && size != n) {
          throw std::runtime_error(
                  std::string(name) + " must be empty or match sensor_names length");
        }
      };
    // HH_260729 - Partial arrays silently assigned fallback values to the
    // remaining ports and made field configuration difficult to audit.
    require_complete_sensor_array(sensor_enabled.size(), "sensor_enabled");
    require_complete_sensor_array(software_max_ranges.size(), "software_max_ranges_m");
    require_complete_sensor_array(
      hardware_range_levels.size(), "hardware_range_levels");
    require_complete_sensor_array(hardware_angle_levels.size(), "hardware_angle_levels");
    for (const auto value : software_max_ranges) {
      if (!std::isfinite(value) || value <= software_min_range_m_) {
        throw std::runtime_error(
                "software_max_ranges_m entries must be finite and greater than "
                "software_min_range_m");
      }
    }
    for (const auto value : hardware_range_levels) {
      if (value < 1 || value > 5) {
        throw std::runtime_error(
                "hardware_range_levels entries must be in DFRobot range 1..5");
      }
    }
    for (const auto value : hardware_angle_levels) {
      if (value < 1 || value > 4) {
        throw std::runtime_error(
                "hardware_angle_levels entries must be in DFRobot range 1..4");
      }
    }
    if (hardware_write_on_startup_ &&
      (hardware_angle_levels.empty() || hardware_range_levels.empty()))
    {
      throw std::runtime_error(
              "hardware_write_on_startup=true requires complete "
              "hardware_angle_levels and hardware_range_levels arrays");
    }
    avg_radar_pub_ = this->create_publisher<avg_msgs::msg::AvgSensingRadar>(radar_status_topic_, 10);

    sensors_.resize(n);
    pubs_.resize(n);
    standard_ros_pubs_.resize(n);
    disabled_channel_dummy_active_pubs_.resize(n);
    // HH_260414: Keep per-sensor publish stamps monotonic to avoid TF extrapolation
    // bursts when system time jitters backwards briefly.
    last_range_pub_stamp_.assign(n, rclcpp::Time(0, 0, this->get_clock()->get_clock_type()));

    // HH_260414: SensorDataQoS (best-effort, shallow queue) reduces stale backlog
    // delivery to RViz/message_filters and helps suppress flicker/extrapolation spikes.
    auto range_qos = rclcpp::SensorDataQoS().keep_last(5);

    for (size_t i = 0; i < n; ++i) {
      sensors_[i].name = sensor_names_[i];
      sensors_[i].frame_id = frame_ids_[i];
      sensors_[i].port = ports_[i];
      sensors_[i].fd = -1;
      sensors_[i].enabled = sensor_enabled.empty() || sensor_enabled[i];
      // HH_260422 - Use the exact per-sensor cutoff when provided; otherwise
      // use the standalone default software limit.
      sensors_[i].software_max_range_m =
        software_max_ranges.empty() ?
        software_default_max_range_m_ :
        software_max_ranges[i];
      // A zero internal expected level means that the standalone defaults did
      // not provide an exact register expectation. Zero is never written.
      sensors_[i].expected_hardware_range_level =
        hardware_range_levels.empty() ?
        0 :
        static_cast<int>(hardware_range_levels[i]);
      sensors_[i].expected_hardware_angle_level =
        hardware_angle_levels.empty() ?
        0 :
        static_cast<int>(hardware_angle_levels[i]);
      if (sensors_[i].expected_hardware_range_level > 0 &&
        sensors_[i].software_max_range_m > approximate_range_level_m(
          sensors_[i].expected_hardware_range_level) + 1e-9)
      {
        throw std::runtime_error(
                "hardware_range_levels must cover software_max_ranges_m for " +
                sensor_names_[i]);
      }
      pubs_[i] = this->create_publisher<avg_msgs::msg::AvgRange>(topics_[i], range_qos);
      // HH_260720 - Convert radar data only on the explicitly named standard-ROS boundary.
      standard_ros_pubs_[i] = this->create_publisher<sensor_msgs::msg::Range>(
        standard_ros_topics_[i], range_qos);
      if (!sensors_[i].enabled) {
        // HH_260729 - Create a per-channel marker only for an intentionally
        // disabled sensor. A physical channel therefore can never be labelled
        // dummy by this driver.
        const auto dummy_active_topic =
          camrod_sensing::radar_dummy_active_topic(topics_[i]);
        const auto dummy_state_qos =
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        disabled_channel_dummy_active_pubs_[i] =
          this->create_publisher<avg_msgs::msg::AvgBool>(
          dummy_active_topic, dummy_state_qos);
      }
    }

    stop_.store(false);

    const auto disabled_channel_count = static_cast<std::size_t>(
      std::count_if(
        sensors_.begin(), sensors_.end(),
        [](const SensorRuntime & sensor) {return !sensor.enabled;}));
    if (disabled_channel_count > 0U) {
      const auto dummy_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(
          1.0 / disabled_channel_dummy_publish_rate_hz_));
      disabled_channel_dummy_timer_ = this->create_wall_timer(
        dummy_period,
        std::bind(
          &Sen0592RadarNode::publish_disabled_channel_heartbeats, this));
      // Publish once at startup as well as periodically. Transient-local marker
      // QoS then identifies dummy ownership even to late diagnostic subscribers.
      publish_disabled_channel_heartbeats();
    }

    // Start worker threads
    for (size_t i = 0; i < n; ++i) {
      if (!sensors_[i].enabled) {
        RCLCPP_WARN(
          this->get_logger(),
          "[%s] explicitly disabled by sensor_enabled[%zu]=false; no port is opened "
          "and no-target/dummy_active heartbeats are published at %.1f Hz",
          sensors_[i].name.c_str(), i,
          disabled_channel_dummy_publish_rate_hz_);
        continue;
      }
      threads_.emplace_back(&Sen0592RadarNode::sensor_worker, this, i);
    }

    // Console status timer (like your Python pretty print)
    status_timer_ = this->create_wall_timer(500ms, std::bind(&Sen0592RadarNode::print_status, this));

    RCLCPP_INFO(this->get_logger(), "sen0592_radar_node started with %zu sensors", n);
  }

  // Cleans up resources used by `Sen0592RadarNode`.
  ~Sen0592RadarNode() override
  {
    stop_.store(true);
    for (auto& th : threads_) {
      if (th.joinable()) th.join();
    }
    for (auto& s : sensors_) {
      if (s.fd >= 0) ::close(s.fd);
      s.fd = -1;
    }
  }

  bool has_fatal_hardware_config_fault() const
  {
    return fatal_hardware_config_fault_.load();
  }

private:
  struct SensorRuntime
  {
    std::string name;
    std::string frame_id;
    std::string port;

    int fd{-1};
    // HH_260729 - Sensor activation is separate from its hardware angle code.
    bool enabled{true};
    // A configured physical detection envelope that cannot be verified is a
    // latched startup fault; publishing from that channel would imply safety
    // coverage that the driver has not confirmed.
    bool hardware_config_fault{false};

    // HH_260422 - Exact per-sensor software cutoff in metres, also carried in
    // Range.max_range.
    double software_max_range_m{4.5};
    // HH_260729 - Expected register levels. An internal zero means no exact
    // expectation was supplied by standalone defaults; valid hardware values
    // remain 1..5 for range and 1..4 for angle.
    int expected_hardware_range_level{0};
    int expected_hardware_angle_level{0};

    int last_mm{-1};
    double last_dt_ms{0.0};
    uint64_t ok{0};
    uint64_t miss{0};
    uint64_t invalid{0};
    std::uint64_t consecutive_misses{0};
    rclcpp::Time last_update;
    bool ever_received{false};
  };

  bool is_valid_measurement_mm(const SensorRuntime& s, int mm) const
  {
    // HH_260729 - software_min_range_m is an actual acceptance limit, not only
    // sensor_msgs/Range metadata. Replies below it become no-target heartbeats
    // and can never paint an obstacle cost.
    const int min_mm = static_cast<int>(software_min_range_m_ * 1000.0 + 0.5);
    if (mm <= 0 || (min_mm > 0 && mm < min_mm)) {
      return false;
    }
    // SEN0592/Modbus no-target or invalid replies can appear near uint16 max
    // (for example 65533 mm). Treat them as fresh invalid samples, not objects.
    if (mm >= 0xFFF0) {
      return false;
    }
    const int max_mm = static_cast<int>(s.software_max_range_m * 1000.0 + 0.5);
    return max_mm <= 0 || mm <= max_mm;
  }

  // Reads one holding register from the sensor. This is used both for live
  // range polling and for startup readback of the hardware-stored angle/range.
  bool read_holding_register(SensorRuntime& s, uint16_t reg, int& out_value)
  {
    if (s.fd < 0) return false;

    auto req = make_read_req(slave_id_, reg, 1);
    drain_serial_rx(s.fd);

    const ssize_t wn = ::write(s.fd, req.data(), req.size());
    if (wn != static_cast<ssize_t>(req.size())) {
      return false;
    }
    if (inter_frame_sleep_s_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(inter_frame_sleep_s_));
    }

    uint8_t resp[kRespLen] = {0};
    const bool ok = read_exact_with_deadline(
      s.fd, resp, kRespLen,
      std::chrono::milliseconds(static_cast<int>(response_deadline_s_ * 1000.0)));
    if (!ok || resp[0] != slave_id_ || resp[1] != kFunctionReadHolding || resp[2] != 0x02) {
      return false;
    }

    const std::vector<uint8_t> resp_wo_crc(resp, resp + 5);
    const uint16_t crc_calc = modbus_crc(resp_wo_crc);
    const uint16_t crc_recv =
      static_cast<uint16_t>(resp[5]) | (static_cast<uint16_t>(resp[6]) << 8);
    if (crc_calc != crc_recv) {
      return false;
    }

    out_value = (static_cast<int>(resp[3]) << 8) | static_cast<int>(resp[4]);
    return true;
  }

  // Sends a single Modbus FC 0x06 write and waits for the echo response. Returns true on success.
  bool write_single_register(SensorRuntime& s, uint16_t reg, uint16_t value, const char* label)
  {
    if (s.fd < 0) return false;
    auto req = make_write_req(slave_id_, reg, value);
    drain_serial_rx(s.fd);
    ssize_t wn = ::write(s.fd, req.data(), req.size());
    if (wn != static_cast<ssize_t>(req.size())) {
      RCLCPP_WARN(this->get_logger(), "[%s] %s write send failed", s.name.c_str(), label);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    uint8_t resp[8] = {0};
    const auto write_response_deadline = std::chrono::milliseconds(
      std::max(1, static_cast<int>(response_deadline_s_ * 1000.0)));
    if (!read_exact_with_deadline(s.fd, resp, 8, write_response_deadline)) {
      RCLCPP_WARN(this->get_logger(),
        "[%s] %s write timed out", s.name.c_str(), label);
      return false;
    }
    if (!std::equal(req.begin(), req.end(), resp)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[%s] %s write echo/CRC mismatch; register value is not confirmed",
        s.name.c_str(), label);
      return false;
    }
    RCLCPP_INFO(this->get_logger(),
      "[%s] reg 0x%04X write echo confirmed: %u (%s)",
      s.name.c_str(), reg, value, label);
    return true;
  }

  // HH_260729 - Read before writing so repeated bringup does not rewrite
  // already-correct nonvolatile registers.  Some units on the current RS485
  // harness apply FC06 but omit/delay its echo; final FC03 readback below is
  // therefore the authority, not the write-echo result alone.
  bool write_hardware_config_registers(SensorRuntime& s)
  {
    if (s.fd < 0) return false;
    if (s.expected_hardware_angle_level <= 0 ||
      s.expected_hardware_range_level <= 0)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "[%s] hardware writes require configured angle and range levels",
        s.name.c_str());
      return false;
    }

    int stored_angle_level = -1;
    int stored_range_level = -1;
    const bool angle_read =
      read_holding_register(s, kRegAngleConfig, stored_angle_level);
    const bool range_read =
      read_holding_register(s, kRegRangeConfig, stored_range_level);
    const bool angle_matches =
      angle_read && stored_angle_level == s.expected_hardware_angle_level;
    const bool range_matches =
      range_read && stored_range_level == s.expected_hardware_range_level;

    if (angle_matches && range_matches) {
      RCLCPP_INFO(
        this->get_logger(),
        "[%s] hardware registers already match angle=level%d range=%s; skip FC06",
        s.name.c_str(), stored_angle_level,
        describe_range_level(stored_range_level).c_str());
      return true;
    }

    const bool angle_written = angle_matches || write_single_register(
      s, kRegAngleConfig,
      static_cast<uint16_t>(s.expected_hardware_angle_level), "angle_level");
    const bool range_written = range_matches || write_single_register(
      s, kRegRangeConfig,
      static_cast<uint16_t>(s.expected_hardware_range_level), "range_level");
    return angle_written && range_written;
  }

  bool validate_hardware_config_readback(SensorRuntime& s, const bool final_attempt)
  {
    int angle_code = -1;
    int range_level = -1;
    const bool angle_ok = read_holding_register(s, kRegAngleConfig, angle_code);
    const bool range_ok = read_holding_register(s, kRegRangeConfig, range_level);

    const std::string expected_angle =
      s.expected_hardware_angle_level == 0 ?
      "not-configured" :
      "level" + std::to_string(s.expected_hardware_angle_level);
    const std::string expected_range =
      s.expected_hardware_range_level == 0 ?
      "not-configured" :
      describe_range_level(s.expected_hardware_range_level);
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] startup config: software_max=%.3fm hardware_writes=%s "
      "expected_angle=%s expected_range=%s",
      s.name.c_str(), s.software_max_range_m,
      hardware_write_on_startup_ ? "enabled" : "disabled",
      expected_angle.c_str(), expected_range.c_str());

    if (!angle_ok || !range_ok) {
      const std::string angle_readback =
        angle_ok ? std::to_string(angle_code) : "unavailable";
      const std::string range_readback =
        range_ok ? std::to_string(range_level) : "unavailable";
      if (final_attempt) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[%s] hardware config readback incomplete after retries: angle=%s range=%s; "
          "channel will stay unavailable because its detection envelope is unverified",
          s.name.c_str(), angle_readback.c_str(), range_readback.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "[%s] hardware config readback incomplete: angle=%s range=%s; retrying",
          s.name.c_str(), angle_readback.c_str(), range_readback.c_str());
      }
      return false;
    }

    const std::string angle_text =
      angle_code >= 1 && angle_code <= 4 ?
      "level" + std::to_string(angle_code) :
      std::to_string(angle_code) + "(unmapped)";
    const std::string range_text = describe_range_level(range_level);
    RCLCPP_INFO(
      this->get_logger(),
      "[%s] hardware readback: angle=0x0208:%s range=0x021F:%s",
      s.name.c_str(), angle_text.c_str(), range_text.c_str());

    if (angle_code < 1 || angle_code > 4 || range_level < 1 || range_level > 5) {
      if (final_attempt) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[%s] hardware readback contains an unsupported angle/range level; "
          "channel will stay unavailable",
          s.name.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "[%s] hardware readback contains an unsupported angle/range level; retrying",
          s.name.c_str());
      }
      return false;
    }
    if (approximate_range_level_m(range_level) + 1e-9 <
      s.software_max_range_m)
    {
      if (final_attempt) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[%s] hardware range readback %s is shorter than software_max=%.3fm; "
          "software cannot recover missing detections",
          s.name.c_str(), range_text.c_str(), s.software_max_range_m);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "[%s] hardware range readback %s is shorter than software_max=%.3fm; retrying",
          s.name.c_str(), range_text.c_str(), s.software_max_range_m);
      }
      return false;
    }
    if (s.expected_hardware_angle_level > 0 &&
      angle_code != s.expected_hardware_angle_level)
    {
      if (final_attempt) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[%s] angle readback mismatch: expected level%d, read level%d",
          s.name.c_str(), s.expected_hardware_angle_level, angle_code);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "[%s] angle readback mismatch: expected level%d, read level%d; retrying",
          s.name.c_str(), s.expected_hardware_angle_level, angle_code);
      }
      return false;
    }
    if (s.expected_hardware_range_level > 0 &&
      range_level != s.expected_hardware_range_level)
    {
      if (final_attempt) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[%s] range readback mismatch: expected %s, read %s",
          s.name.c_str(), expected_range.c_str(), range_text.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "[%s] range readback mismatch: expected %s, read %s; retrying",
          s.name.c_str(), expected_range.c_str(), range_text.c_str());
      }
      return false;
    }
    return true;
  }

  void latch_fatal_hardware_config_fault(SensorRuntime& s)
  {
    s.hardware_config_fault = true;
    fatal_hardware_config_fault_.store(true);
    stop_.store(true);
    // HH_260729 - A channel whose physical detection envelope cannot be
    // verified must not leave the multi-radar process appearing healthy.
    // End the ROS context so main returns a non-zero process status.
    rclcpp::shutdown();
  }

  // Implements `open_sensor_port` behavior.
  bool open_sensor_port(SensorRuntime& s)
  {
    if (s.fd >= 0) return true;

    int fd = ::open(s.port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      const int open_errno = errno;
      RCLCPP_ERROR(this->get_logger(), "[%s] Failed to open port %s: %s",
                   s.name.c_str(), s.port.c_str(), std::strerror(open_errno));
      return false;
    }

    if (!configure_serial(fd, baud_)) {
      RCLCPP_ERROR(this->get_logger(), "[%s] Failed to configure serial %s",
                   s.name.c_str(), s.port.c_str());
      ::close(fd);
      return false;
    }

    s.fd = fd;
    RCLCPP_INFO(this->get_logger(), "[%s] Opened %s @ %d", s.name.c_str(), s.port.c_str(), baud_);
    // HH_260729 - An FC06 echo may be absent even when the sensor accepted the
    // value.  Do not fail solely on that transport acknowledgement; the bounded
    // FC03 loop below must still confirm the exact requested levels or the
    // channel remains fail-closed.
    if (hardware_write_on_startup_ && !write_hardware_config_registers(s)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[%s] one or more FC06 write echoes were not confirmed; "
        "continuing to authoritative FC03 readback",
        s.name.c_str());
    }
    // HH_260729 - Read back the stored/written values so a loaded YAML value
    // is never confused with a value confirmed on the physical sensor.
    // HH_260729 - SEN0592 may need up to roughly one second after power-up.
    // Keep verification fail-closed, but do not turn one transient cold-start
    // timeout into a permanent seven-channel process failure.
    bool readback_verified = false;
    for (int attempt = 1; attempt <= kStartupReadbackAttempts; ++attempt) {
      const bool final_attempt = attempt == kStartupReadbackAttempts;
      if (validate_hardware_config_readback(s, final_attempt)) {
        readback_verified = true;
        break;
      }
      if (!final_attempt) {
        std::this_thread::sleep_for(250ms);
        // HH_260729 - A missing FC06 echo can also mean that the register
        // write itself was lost. Re-read and rewrite only the still-mismatched
        // register before the next authoritative FC03 verification. This was
        // observed on LEFT2 while changing angle level 2 -> 1; readback-only
        // retries could never recover because the old value remained stored.
        if (hardware_write_on_startup_ && !write_hardware_config_registers(s)) {
          RCLCPP_WARN(
            this->get_logger(),
            "[%s] hardware config rewrite was not echoed; "
            "the next FC03 readback remains authoritative",
            s.name.c_str());
        }
      }
    }
    if (!readback_verified) {
      ::close(s.fd);
      s.fd = -1;
      latch_fatal_hardware_config_fault(s);
      return false;
    }
    return true;
  }

  // Implements `read_realtime_mm` behavior.
  bool read_realtime_mm(SensorRuntime& s, int& out_mm)
  {
    return read_holding_register(s, kRegRealtimeValue, out_mm);
  }

  void publish_range_value(size_t idx, float range_m)
  {
    auto msg = avg_msgs::msg::AvgRange();
    auto stamp = this->get_clock()->now();
    if (idx < last_range_pub_stamp_.size() && stamp <= last_range_pub_stamp_[idx]) {
      stamp = last_range_pub_stamp_[idx] + rclcpp::Duration::from_nanoseconds(1);
    }
    if (idx < last_range_pub_stamp_.size()) {
      last_range_pub_stamp_[idx] = stamp;
    }
    msg.header.stamp = stamp;
    msg.header.frame_id = sensors_[idx].frame_id;

    // Ultrasonic
    msg.radiation_type = avg_msgs::msg::AvgRange::ULTRASOUND;
    msg.field_of_view = static_cast<float>(range_message_field_of_view_rad_);
    msg.min_range = static_cast<float>(software_min_range_m_);
    // HH_260422: Use per-sensor max_range so cost grid node filters per direction automatically.
    msg.max_range = static_cast<float>(sensors_[idx].software_max_range_m);
    msg.range = range_m;

    pubs_[idx]->publish(msg);
    // HH_260720 - Keep RViz radar visualization synchronized with the canonical message.
    standard_ros_pubs_[idx]->publish(avg_msgs::conversions::toRos(msg));
    publish_radar_status(idx, msg);
  }

  // Publishes `_range` output for a valid obstacle return.
  void publish_range(size_t idx, int mm)
  {
    publish_range_value(idx, static_cast<float>(mm) / 1000.0f);  // mm -> m
  }

  void publish_no_target_range(size_t idx)
  {
    // HH_260701 - Keep radar diagnostics alive on SEN0592 no-target replies.
    // Range consumers already ignore values above max_range, so this heartbeat
    // distinguishes "sensor responded with no object" from "serial port stale".
    publish_range_value(
      idx, static_cast<float>(
        camrod_sensing::radar_no_target_range_m(
          sensors_[idx].software_max_range_m)));
  }

  void publish_disabled_channel_heartbeats()
  {
    auto dummy_active = avg_msgs::msg::AvgBool();
    dummy_active.data = true;

    for (std::size_t idx = 0U; idx < sensors_.size(); ++idx) {
      if (sensors_[idx].enabled) {
        continue;
      }
      // HH_260729 - Disabled hardware must not open its serial port, but its
      // canonical AvgRange and `_ros` topics remain fresh with a value above
      // max_range. The per-channel marker prevents diagnostics from mistaking
      // this transport-only heartbeat for verified physical radar data.
      publish_no_target_range(idx);
      if (disabled_channel_dummy_active_pubs_[idx]) {
        disabled_channel_dummy_active_pubs_[idx]->publish(dummy_active);
      }
    }
  }

  // Publishes `_avg_radar` output.
  void publish_radar_status(size_t idx, const avg_msgs::msg::AvgRange & msg)
  {
    if (!publish_radar_status_.load() || !avg_radar_pub_) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(avg_mtx_);
      if (idx < topics_.size()) {
        const auto & topic = topics_[idx];
        // HH_260623 - Publish front1/front2 separately; merged front output was removed.
        if (topic.find("front1") != std::string::npos) {
          avg_radar_msg_.front1 = msg;
        } else if (topic.find("front2") != std::string::npos) {
          avg_radar_msg_.front2 = msg;
        } else if (topic.find("right1") != std::string::npos) {
          avg_radar_msg_.right1 = msg;
        } else if (topic.find("right2") != std::string::npos) {
          avg_radar_msg_.right2 = msg;
        } else if (topic.find("left1") != std::string::npos) {
          avg_radar_msg_.left1 = msg;
        } else if (topic.find("left2") != std::string::npos) {
          avg_radar_msg_.left2 = msg;
        } else if (topic.find("rear") != std::string::npos) {
          avg_radar_msg_.rear = msg;
        }
      }
      avg_radar_pub_->publish(avg_radar_msg_);
    }
  }

  // Implements `sensor_worker` behavior.
  void sensor_worker(size_t idx)
  {
    auto& s = sensors_[idx];
    const auto poll_period = std::chrono::duration<double>(poll_period_s_);

    while (rclcpp::ok() && !stop_.load()) {
      auto loop_start = std::chrono::steady_clock::now();

      if (s.hardware_config_fault) {
        std::this_thread::sleep_for(1s);
        continue;
      }
      if (!open_sensor_port(s)) {
        std::this_thread::sleep_for(1s);
        continue;
      }

      auto t0 = std::chrono::steady_clock::now();
      int mm = -1;
      bool ok = read_realtime_mm(s, mm);
      auto t1 = std::chrono::steady_clock::now();
      double dt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      const bool measurement_valid = ok && is_valid_measurement_mm(s, mm);

      bool reopen_port = false;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        s.last_dt_ms = dt_ms;

        if (measurement_valid) {
          s.ok++;
          s.last_mm = mm;
          s.last_update = this->now();
          s.ever_received = true;
          s.consecutive_misses = 0U;
        } else if (ok) {
          s.invalid++;
          s.last_mm = -1;
          s.last_update = this->now();
          s.ever_received = true;
          s.consecutive_misses = 0U;
        } else {
          s.miss++;
          s.consecutive_misses++;
          reopen_port =
            s.consecutive_misses >= kReconnectAfterConsecutiveMisses;
        }
      }

      if (measurement_valid) {
        publish_range(idx, mm);
      } else if (ok) {
        publish_no_target_range(idx);
      }
      if (reopen_port) {
        RCLCPP_ERROR(
          this->get_logger(),
          "[%s] %s (%s) missed %llu consecutive Modbus replies; closing and "
          "reopening the port with hardware readback verification",
          s.name.c_str(), s.port.c_str(), s.frame_id.c_str(),
          static_cast<unsigned long long>(kReconnectAfterConsecutiveMisses));
        ::close(s.fd);
        s.fd = -1;
        s.consecutive_misses = 0U;
      }

      auto elapsed = std::chrono::steady_clock::now() - loop_start;
      if (elapsed < poll_period) {
        std::this_thread::sleep_for(poll_period - elapsed);
      }
    }
  }

  // Implements `print_status` behavior.
  void print_status()
  {
    if (!log_status_.load()) {
      return;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    std::string row1, row2;

    for (size_t i = 0; i < sensors_.size(); ++i) {
      const auto& s = sensors_[i];
      if (!s.enabled) {
        if (i > 0) {
          row1 += " | ";
          row2 += " | ";
        }
        row1 += s.name + " DISABLED";
        row2 += "port closed; dummy no-target";
        continue;
      }
      double age = s.ever_received ? (this->now() - s.last_update).seconds() : -1.0;
      std::string port_label = s.port;
      const auto slash = port_label.find_last_of('/');
      if (slash != std::string::npos) {
        port_label = port_label.substr(slash + 1);
      }
      constexpr const char * kCh9344Prefix = "ttyCH9344";
      if (port_label.rfind(kCh9344Prefix, 0) == 0) {
        port_label = port_label.substr(std::strlen(kCh9344Prefix));
      }

      char c1[256];
      std::snprintf(c1, sizeof(c1),
                    "%-6s %-4s %4smm %5.1fms age%4.2f",
                    s.name.c_str(),
                    port_label.c_str(),
                    (s.last_mm < 0 ? "----" : std::to_string(s.last_mm).c_str()),
                    s.last_dt_ms,
                    (age < 0.0 ? 99.99 : age));

      char c2[128];
      std::snprintf(c2, sizeof(c2), "ok%06llu miss%04llu",
                    static_cast<unsigned long long>(s.ok),
                    static_cast<unsigned long long>(s.miss));
      if (s.invalid > 0) {
        char invalid_text[32];
        std::snprintf(invalid_text, sizeof(invalid_text), " inv%04llu",
                      static_cast<unsigned long long>(s.invalid));
        std::strncat(c2, invalid_text, sizeof(c2) - std::strlen(c2) - 1);
      }

      if (i > 0) {
        row1 += " | ";
        row2 += " | ";
      }
      row1 += c1;
      row2 += c2;
    }

    // HH_260629: `log_status` is an explicit launch/debug opt-in, so keep the
    // sensor-name + USB suffix + range/miss counters visible when it is enabled.
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", row1.c_str());
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", row2.c_str());
  }

private:
  std::vector<std::string> sensor_names_;
  std::vector<std::string> frame_ids_;
  std::vector<std::string> ports_;
  std::vector<std::string> topics_;
  std::vector<std::string> standard_ros_topics_;

  int baud_{115200};
  uint8_t slave_id_{1};
  double poll_period_s_{0.06};
  double response_deadline_s_{0.20};
  double inter_frame_sleep_s_{0.003};
  double disabled_channel_dummy_publish_rate_hz_{2.0};
  double software_min_range_m_{0.02};
  double software_default_max_range_m_{4.5};
  double range_message_field_of_view_rad_{0.26};
  std::string radar_status_topic_;
  std::atomic_bool publish_radar_status_{false};
  std::atomic_bool log_status_{false};
  bool hardware_write_on_startup_{false};

  std::vector<SensorRuntime> sensors_;
  std::vector<rclcpp::Publisher<avg_msgs::msg::AvgRange>::SharedPtr> pubs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> standard_ros_pubs_;
  std::vector<rclcpp::Publisher<avg_msgs::msg::AvgBool>::SharedPtr>
  disabled_channel_dummy_active_pubs_;
  std::vector<rclcpp::Time> last_range_pub_stamp_;
  rclcpp::Publisher<avg_msgs::msg::AvgSensingRadar>::SharedPtr avg_radar_pub_;
  avg_msgs::msg::AvgSensingRadar avg_radar_msg_;

  std::vector<std::thread> threads_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> fatal_hardware_config_fault_{false};
  std::mutex mtx_;
  std::mutex avg_mtx_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr disabled_channel_dummy_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<Sen0592RadarNode>();
    rclcpp::spin(node);
    if (node->has_fatal_hardware_config_fault()) {
      exit_code = 1;
    }
  } catch (const std::exception& e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
