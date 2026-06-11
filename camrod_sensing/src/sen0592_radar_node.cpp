#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <glob.h>
#include <mutex>
#include <sys/ioctl.h>
#include <termios.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <avg_msgs/msg/range.hpp>

#include <avg_msgs/msg/avg_sensing_radar.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr uint16_t kRegRealtimeValue = 0x0101;   // DFRobot SEN0592 real-time distance (mm)
// HH_260422: Detection angle register. Values 1–5 map to 15°–75° on SEN0592.
constexpr uint16_t kRegAngleConfig    = 0x0208;
// HH_260422: Detection range register (mm). Hardware-level filter — sensor does not report
//   objects beyond this value. Set per direction based on stopping-distance physics:
//   FRONT 1500mm (5km/h wet-road stop), SIDE 800mm, REAR 500mm (3km/h reverse stop).
constexpr uint16_t kRegRangeConfig    = 0x021F;
constexpr uint8_t  kFunctionReadHolding = 0x03;
constexpr uint8_t  kFunctionWriteSingle = 0x06;  // Modbus FC 0x06: write single register
constexpr size_t   kRespLen = 7;                 // [id][fc][len][hi][lo][crc_lo][crc_hi]

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
    (void)::read(fd, tmp.data(), tmp.size());
  }
}

}  // namespace

class Sen0592RadarNode : public rclcpp::Node
{
public:
  using AvgSensingRadar = avg_msgs::msg::AvgSensingRadar;

  // Implements `Sen0592RadarNode` behavior.
  Sen0592RadarNode() : Node("sen0592_radar_node")
  {
    // ----- Parameters -----
    this->declare_parameter<int>("baud", 115200);
    this->declare_parameter<int>("slave_id", 1);
    this->declare_parameter<double>("poll_period_s", 0.06);
    this->declare_parameter<double>("response_deadline_s", 0.20);
    this->declare_parameter<double>("inter_frame_sleep_s", 0.003);
    this->declare_parameter<double>("min_range_m", 0.02);
    this->declare_parameter<double>("max_range_m", 4.5);
    this->declare_parameter<double>("field_of_view_rad", 0.26);
    this->declare_parameter<std::string>("radar_status_topic", "/sensing/radar/status");
    this->declare_parameter<bool>("publish_radar_status", false);
    // HH_260527: Per-sensor detection angle written to register 0x0208 on port open.
    //   Values: 1=15° 2=30° 3=45° 4=60° 5=75°. 0 = skip (sensor retains last hardware value).
    //   Order must match sensor_names.
    this->declare_parameter<std::vector<int>>(
      "sensor_angle_config_values", std::vector<int>{});
    // HH_260422: sensor_max_ranges_m: per-sensor max range (m), ordered identically to sensor_names.
    //   Carried in Range.max_range for software-side filtering in cost_grid_node.
    this->declare_parameter<std::vector<double>>(
      "sensor_max_ranges_m", std::vector<double>{});
    // HH_260422: sensor_range_config_mm: hardware detection range per sensor (mm) written to 0x021F.
    //   Order: REAR=500, LEFT2=800, LEFT1=800, RIGHT2=800, RIGHT1=800, FRONT=1500.
    //   Basis: wet-road stopping distance + reaction time + margin at operating speed per direction.
    this->declare_parameter<std::vector<int>>(
      "sensor_range_config_mm", std::vector<int>{500, 800, 800, 800, 800, 1500});

    // Sensor definitions (name, frame_id, port, topic)
    // Default values are Linux examples; replace with your actual /dev paths or udev symlinks.
    sensor_names_ = this->declare_parameter<std::vector<std::string>>(
      "sensor_names", {"REAR", "LEFT2", "LEFT1", "RIGHT2", "RIGHT1", "FRONT"});

    frame_ids_ = this->declare_parameter<std::vector<std::string>>(
      "frame_ids", {"radar_rear_link", "radar_left2_link", "radar_left1_link",
                    "radar_right2_link", "radar_right1_link", "radar_front_link"});

    ports_ = this->declare_parameter<std::vector<std::string>>(
      "ports", {
        "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
        "/dev/ttyUSB3", "/dev/ttyUSB4", "/dev/ttyUSB5"
      });

    topics_ = this->declare_parameter<std::vector<std::string>>(
      "topics", {
        "/sensing/radar/rear/range", "/sensing/radar/left2/range", "/sensing/radar/left1/range",
        "/sensing/radar/right2/range", "/sensing/radar/right1/range", "/sensing/radar/front/range"
      });

    baud_ = this->get_parameter("baud").as_int();
    slave_id_ = static_cast<uint8_t>(this->get_parameter("slave_id").as_int());
    poll_period_s_ = this->get_parameter("poll_period_s").as_double();
    response_deadline_s_ = this->get_parameter("response_deadline_s").as_double();
    inter_frame_sleep_s_ = this->get_parameter("inter_frame_sleep_s").as_double();
    min_range_m_ = this->get_parameter("min_range_m").as_double();
    max_range_m_ = this->get_parameter("max_range_m").as_double();
    fov_rad_ = this->get_parameter("field_of_view_rad").as_double();
    radar_status_topic_ = this->get_parameter("radar_status_topic").as_string();
    publish_radar_status_ = this->get_parameter("publish_radar_status").as_bool();

    resolve_auto_ports();

    const auto n = ports_.size();
    if (sensor_names_.size() != n || frame_ids_.size() != n || topics_.size() != n) {
      throw std::runtime_error("sensor_names, frame_ids, ports, topics must have the same length");
    }

    const auto sensor_max_ranges = this->get_parameter("sensor_max_ranges_m").as_double_array();
    const auto sensor_range_config_mm = this->get_parameter("sensor_range_config_mm").as_integer_array();
    const auto sensor_angle_config = this->get_parameter("sensor_angle_config_values").as_integer_array();
    avg_radar_pub_ = this->create_publisher<AvgSensingRadar>(radar_status_topic_, 10);

    sensors_.resize(n);
    pubs_.resize(n);
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
      // HH_260422: Use per-sensor max range when provided; fall back to global max_range_m_.
      sensors_[i].max_range_m =
        (i < sensor_max_ranges.size() && sensor_max_ranges[i] > 0.0)
        ? sensor_max_ranges[i] : max_range_m_;
      // HH_260422: Hardware range limit per sensor (mm). Written to 0x021F at port open.
      sensors_[i].range_config_mm =
        (i < sensor_range_config_mm.size() && sensor_range_config_mm[i] > 0)
        ? static_cast<int>(sensor_range_config_mm[i]) : 1500;
      // HH_260527: Per-sensor detection angle written to 0x0208 on port open. 0=skip.
      sensors_[i].angle_config_value =
        (i < static_cast<size_t>(sensor_angle_config.size()) && sensor_angle_config[i] > 0)
        ? static_cast<int>(sensor_angle_config[i]) : 0;
      pubs_[i] = this->create_publisher<avg_msgs::msg::Range>(topics_[i], range_qos);
    }

    stop_.store(false);

    // Start worker threads
    for (size_t i = 0; i < n; ++i) {
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

private:
  struct SensorRuntime
  {
    std::string name;
    std::string frame_id;
    std::string port;

    int fd{-1};

    // HH_260422: Per-sensor max detection range in metres (carried in Range.max_range).
    //   Populated from sensor_max_ranges_m; falls back to global max_range_m_.
    double max_range_m{4.5};
    // HH_260422: Hardware detection range written to register 0x021F (mm).
    //   Stops the sensor from reporting objects beyond this distance at the hardware level.
    //   Default values match stopping-distance physics per direction.
    int range_config_mm{1500};
    // HH_260527: Per-sensor detection angle (1=15°…5=75°). 0=skip angle write.
    int angle_config_value{0};

    int last_mm{-1};
    double last_dt_ms{0.0};
    uint64_t ok{0};
    uint64_t miss{0};
    rclcpp::Time last_update;
    bool ever_received{false};
  };

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
    if (!read_exact_with_deadline(s.fd, resp, 8, std::chrono::milliseconds(200))) {
      RCLCPP_WARN(this->get_logger(),
        "[%s] %s write timed out (continuing anyway)", s.name.c_str(), label);
      return false;
    }
    RCLCPP_INFO(this->get_logger(),
      "[%s] reg 0x%04X = %u (%s)", s.name.c_str(), reg, value, label);
    return true;
  }

  // HH_260527: Writes per-sensor detection angle (0x0208) and hardware range limit (0x021F) on port open.
  void write_angle_config_register(SensorRuntime& s)
  {
    if (s.fd < 0) return;
    if (s.angle_config_value > 0) {
      write_single_register(s, kRegAngleConfig,
        static_cast<uint16_t>(s.angle_config_value), "angle_config");
    }
    write_single_register(s, kRegRangeConfig,
      static_cast<uint16_t>(s.range_config_mm), "range_config");
  }

  bool is_auto_port_spec(const std::string& port) const
  {
    return port == "auto" || port.rfind("auto:", 0) == 0;
  }

  // HH_260611: Resolve "auto" radar ports at runtime because the CH9344 USB
  // serial indices can change when GNSS/radar converters are unplugged or swapped.
  std::string auto_port_pattern(const std::string& port) const
  {
    if (port == "auto") {
      return "/dev/ttyCH9344USB*";
    }
    return port.substr(std::strlen("auto:"));
  }

  std::vector<std::string> expand_port_pattern(const std::string& pattern) const
  {
    glob_t matches{};
    std::vector<std::string> paths;
    const int rc = ::glob(pattern.c_str(), 0, nullptr, &matches);
    if (rc == 0) {
      for (size_t i = 0; i < matches.gl_pathc; ++i) {
        paths.emplace_back(matches.gl_pathv[i]);
      }
      std::sort(paths.begin(), paths.end());
    }
    ::globfree(&matches);
    return paths;
  }

  bool probe_sensor_port(const std::string& port, int& out_mm)
  {
    // HH_260611: Probe candidates by reading one SEN0592 sample so a single
    // connected radar can be found without hard-coding /dev/ttyCH9344USBN.
    int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      return false;
    }

    if (!configure_serial(fd, baud_)) {
      ::close(fd);
      return false;
    }

    SensorRuntime probe;
    probe.name = "AUTO";
    probe.port = port;
    probe.fd = fd;
    const bool ok = read_realtime_mm(probe, out_mm);
    ::close(fd);
    probe.fd = -1;
    return ok;
  }

  void log_port_open_hint_once(const SensorRuntime& sensor, int open_errno)
  {
    // HH_260611: Emit actionable one-time hints for the common bench failures:
    // missing CH9344 kernel device nodes or user permission on serial devices.
    if (open_errno == ENOENT &&
        sensor.port.find("/dev/ttyCH9344USB") != std::string::npos) {
      bool expected = false;
      if (ch9344_driver_hint_logged_.compare_exchange_strong(expected, true)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "CH9344 radar serial devices are missing. The launch is using '%s', but /dev/ttyCH9344USB* does not exist. Load/check the CH9344 kernel driver, then verify with: sudo insmod /home/hong/camrod_ws/build/radar_driver/ch9344.ko && ls /dev/ttyCH9344USB*",
          sensor.port.c_str());
      }
      return;
    }

    if (open_errno == EACCES) {
      bool expected = false;
      if (serial_permission_hint_logged_.compare_exchange_strong(expected, true)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Radar serial port '%s' exists but is not accessible. Check dialout group/udev permissions for the current user.",
          sensor.port.c_str());
      }
    }
  }

  void resolve_auto_ports()
  {
    // HH_260611: Expand each auto pattern and bind it to the first unused port
    // that responds as a SEN0592 radar.
    std::vector<std::string> used_ports;

    for (auto& port : ports_) {
      if (!is_auto_port_spec(port)) {
        used_ports.push_back(port);
        continue;
      }

      const auto pattern = auto_port_pattern(port);
      const auto candidates = expand_port_pattern(pattern);
      if (candidates.empty()) {
        RCLCPP_WARN(
          this->get_logger(),
          "No serial ports matched radar auto pattern '%s'; leaving it unresolved",
          pattern.c_str());
        port = pattern;
        continue;
      }

      bool resolved = false;
      for (const auto& candidate : candidates) {
        if (std::find(used_ports.begin(), used_ports.end(), candidate) != used_ports.end()) {
          continue;
        }

        int mm = -1;
        if (probe_sensor_port(candidate, mm)) {
          port = candidate;
          used_ports.push_back(candidate);
          resolved = true;
          RCLCPP_INFO(
            this->get_logger(),
            "Resolved radar auto port '%s' -> '%s' (%d mm)",
            pattern.c_str(), candidate.c_str(), mm);
          break;
        }
      }

      if (!resolved) {
        RCLCPP_WARN(
          this->get_logger(),
          "No responding SEN0592 radar found for auto pattern '%s'; leaving it unresolved",
          pattern.c_str());
        port = pattern;
      }
    }
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
      log_port_open_hint_once(s, open_errno);
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
    // HH_260422: Configure detection angle on first open.
    write_angle_config_register(s);
    return true;
  }

  // Implements `read_realtime_mm` behavior.
  bool read_realtime_mm(SensorRuntime& s, int& out_mm)
  {
    if (s.fd < 0) return false;

    auto req = make_read_req(slave_id_, kRegRealtimeValue, 1);

    drain_serial_rx(s.fd);

    ssize_t wn = ::write(s.fd, req.data(), req.size());
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

    if (!ok) return false;

    // Basic frame validation
    if (resp[0] != slave_id_ || resp[1] != kFunctionReadHolding || resp[2] != 0x02) {
      return false;
    }

    // CRC check
    std::vector<uint8_t> resp_wo_crc(resp, resp + 5);
    uint16_t crc_calc = modbus_crc(resp_wo_crc);
    uint16_t crc_recv = static_cast<uint16_t>(resp[5]) | (static_cast<uint16_t>(resp[6]) << 8);
    if (crc_calc != crc_recv) {
      return false;
    }

    out_mm = (static_cast<int>(resp[3]) << 8) | static_cast<int>(resp[4]);
    return true;
  }

  // Publishes `_range` output.
  void publish_range(size_t idx, int mm)
  {
    auto msg = avg_msgs::msg::Range();
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
    msg.radiation_type = avg_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = static_cast<float>(fov_rad_);
    msg.min_range = static_cast<float>(min_range_m_);
    // HH_260422: Use per-sensor max_range so cost grid node filters per direction automatically.
    msg.max_range = static_cast<float>(sensors_[idx].max_range_m);
    msg.range = static_cast<float>(mm) / 1000.0f;  // mm -> m

    // clamp/invalid handling (optional)
    if (msg.range < msg.min_range || msg.range > msg.max_range) {
      // Keep publishing, but clamp to max for safety if needed:
      // msg.range = msg.max_range;
      // For now publish raw converted value.
    }

    pubs_[idx]->publish(msg);
    publish_radar_status(idx, msg);
  }

  // Publishes `_avg_radar` output.
  void publish_radar_status(size_t idx, const avg_msgs::msg::Range & msg)
  {
    if (!publish_radar_status_ || !avg_radar_pub_) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(avg_mtx_);
      if (idx < topics_.size()) {
        const auto & topic = topics_[idx];
        if (topic.find("front") != std::string::npos) {
          avg_radar_msg_.front = msg;
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

      if (!open_sensor_port(s)) {
        std::this_thread::sleep_for(1s);
        continue;
      }

      auto t0 = std::chrono::steady_clock::now();
      int mm = -1;
      bool ok = read_realtime_mm(s, mm);
      auto t1 = std::chrono::steady_clock::now();
      double dt_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

      {
        std::lock_guard<std::mutex> lock(mtx_);
        s.last_dt_ms = dt_ms;
        s.last_update = this->now();

        if (ok) {
          s.ok++;
          s.last_mm = mm;
          s.ever_received = true;
        } else {
          s.miss++;
        }
      }

      if (ok) {
        publish_range(idx, mm);
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
    std::lock_guard<std::mutex> lock(mtx_);
    std::string row1, row2;

    for (size_t i = 0; i < sensors_.size(); ++i) {
      const auto& s = sensors_[i];
      double age = s.ever_received ? (this->now() - s.last_update).seconds() : -1.0;

      char c1[256];
      std::snprintf(c1, sizeof(c1),
                    "%-6s %4smm %5.1fms age%4.2f",
                    s.name.c_str(),
                    (s.last_mm < 0 ? "----" : std::to_string(s.last_mm).c_str()),
                    s.last_dt_ms,
                    (age < 0.0 ? 99.99 : age));

      char c2[128];
      std::snprintf(c2, sizeof(c2), "ok%06llu miss%04llu",
                    static_cast<unsigned long long>(s.ok),
                    static_cast<unsigned long long>(s.miss));

      if (i > 0) {
        row1 += " | ";
        row2 += " | ";
      }
      row1 += c1;
      row2 += c2;
    }

    // DEBUG: You can adjust the throttle duration or remove it to see every update, but it may be too verbose.
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", row1.c_str());
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", row2.c_str());
  }

private:
  std::vector<std::string> sensor_names_;
  std::vector<std::string> frame_ids_;
  std::vector<std::string> ports_;
  std::vector<std::string> topics_;

  int baud_{115200};
  uint8_t slave_id_{1};
  double poll_period_s_{0.06};
  double response_deadline_s_{0.20};
  double inter_frame_sleep_s_{0.003};
  double min_range_m_{0.02};
  double max_range_m_{4.5};
  double fov_rad_{0.26};
  std::string radar_status_topic_;
  bool publish_radar_status_{false};

  std::vector<SensorRuntime> sensors_;
  std::vector<rclcpp::Publisher<avg_msgs::msg::Range>::SharedPtr> pubs_;
  std::vector<rclcpp::Time> last_range_pub_stamp_;
  rclcpp::Publisher<AvgSensingRadar>::SharedPtr avg_radar_pub_;
  AvgSensingRadar avg_radar_msg_;

  std::vector<std::thread> threads_;
  std::atomic<bool> stop_{false};
  // HH_260611: Keep port-open hints one-shot to avoid flooding logs while retrying disconnected radars.
  std::atomic<bool> ch9344_driver_hint_logged_{false};
  std::atomic<bool> serial_permission_hint_logged_{false};
  std::mutex mtx_;
  std::mutex avg_mtx_;

  rclcpp::TimerBase::SharedPtr status_timer_;
};

// Entry point for this executable.
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Sen0592RadarNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
