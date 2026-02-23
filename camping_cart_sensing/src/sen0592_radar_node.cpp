#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
#include <termios.h>
#include <sys/ioctl.h>

using namespace std::chrono_literals;

namespace
{
constexpr uint16_t kRegRealtimeValue = 0x0101;   // DFRobot SEN0592 real-time distance (mm)
constexpr uint8_t  kFunctionReadHolding = 0x03;
constexpr size_t   kRespLen = 7;                 // [id][fc][len][hi][lo][crc_lo][crc_hi]

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
  p.push_back(static_cast<uint8_t>(crc & 0xFF));         // CRC low
  p.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));  // CRC high
  return p;
}

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
  Sen0592RadarNode() : Node("sen0592_radar_node")
  {
    // ----- Parameters -----
    this->declare_parameter<int>("baud", 115200);
    this->declare_parameter<int>("slave_id", 1);
    this->declare_parameter<double>("poll_period_s", 0.06);
    this->declare_parameter<double>("response_deadline_s", 0.20);
    this->declare_parameter<double>("inter_frame_sleep_s", 0.003);
    this->declare_parameter<bool>("use_range_msg", true);
    this->declare_parameter<double>("min_range_m", 0.02);
    this->declare_parameter<double>("max_range_m", 4.5);
    this->declare_parameter<double>("field_of_view_rad", 0.26);

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
        "/radar/rear/range", "/radar/left2/range", "/radar/left1/range",
        "/radar/right2/range", "/radar/right1/range", "/radar/front/range"
      });

    const auto n = ports_.size();
    if (sensor_names_.size() != n || frame_ids_.size() != n || topics_.size() != n) {
      throw std::runtime_error("sensor_names, frame_ids, ports, topics must have the same length");
    }

    baud_ = this->get_parameter("baud").as_int();
    slave_id_ = static_cast<uint8_t>(this->get_parameter("slave_id").as_int());
    poll_period_s_ = this->get_parameter("poll_period_s").as_double();
    response_deadline_s_ = this->get_parameter("response_deadline_s").as_double();
    inter_frame_sleep_s_ = this->get_parameter("inter_frame_sleep_s").as_double();
    min_range_m_ = this->get_parameter("min_range_m").as_double();
    max_range_m_ = this->get_parameter("max_range_m").as_double();
    fov_rad_ = this->get_parameter("field_of_view_rad").as_double();

    sensors_.resize(n);
    pubs_.resize(n);

    for (size_t i = 0; i < n; ++i) {
      sensors_[i].name = sensor_names_[i];
      sensors_[i].frame_id = frame_ids_[i];
      sensors_[i].port = ports_[i];
      sensors_[i].fd = -1;
      pubs_[i] = this->create_publisher<sensor_msgs::msg::Range>(topics_[i], 10);
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

    int last_mm{-1};
    double last_dt_ms{0.0};
    uint64_t ok{0};
    uint64_t miss{0};
    rclcpp::Time last_update;
    bool ever_received{false};
  };

  bool open_sensor_port(SensorRuntime& s)
  {
    if (s.fd >= 0) return true;

    int fd = ::open(s.port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "[%s] Failed to open port %s: %s",
                   s.name.c_str(), s.port.c_str(), std::strerror(errno));
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
    return true;
  }

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

  void publish_range(size_t idx, int mm)
  {
    auto msg = sensor_msgs::msg::Range();
    msg.header.stamp = this->now();
    msg.header.frame_id = sensors_[idx].frame_id;

    // Ultrasonic
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = static_cast<float>(fov_rad_);
    msg.min_range = static_cast<float>(min_range_m_);
    msg.max_range = static_cast<float>(max_range_m_);
    msg.range = static_cast<float>(mm) / 1000.0f;  // mm -> m

    // clamp/invalid handling (optional)
    if (msg.range < msg.min_range || msg.range > msg.max_range) {
      // Keep publishing, but clamp to max for safety if needed:
      // msg.range = msg.max_range;
      // For now publish raw converted value.
    }

    pubs_[idx]->publish(msg);
  }

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

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", row1.c_str());
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", row2.c_str());
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

  std::vector<SensorRuntime> sensors_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> pubs_;

  std::vector<std::thread> threads_;
  std::atomic<bool> stop_{false};
  std::mutex mtx_;

  rclcpp::TimerBase::SharedPtr status_timer_;
};

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
