// 260708 - Serial bridge between light_controller and the Arduino Nano light MCU.
// Wire protocol (115200 8N1, newline-terminated JSON, docs/lights-design-doc.html):
//   PC -> MCU @ tx_rate_hz : {"h":0|1,"i":"O"|"L"|"R"|"H","q":<seq>}
//   MCU -> PC @ ~1 Hz      : {"a":<last seq>,"h":0|1,"i":"...","e":<error code>}
// The MCU blinks autonomously and falls back to HAZARD when frames stop for
// mcu_timeout_s — this node only keeps the frame stream and reports link state.
// Serial I/O is plain POSIX termios (same approach as the vendored ugv_sdk),
// JSON via header-only nlohmann (declared like nav2_smac_planner does).
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "avg_msgs/msg/avg_light_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace camrod_platform
{

class McuSerialBridgeNode : public rclcpp::Node
{
public:
  McuSerialBridgeNode()
  : rclcpp::Node("mcu_serial_bridge")
  {
    lights_command_topic_ = declare_parameter<std::string>(
      "lights_command_topic", "/platform/lights/command");
    lights_status_topic_ = declare_parameter<std::string>(
      "lights_status_topic", "/platform/lights/status");
    serial_port_ = declare_parameter<std::string>(
      "serial_port",
      "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG003YCF-if00-port0");
    serial_baud_ = declare_parameter<int>("serial_baud", 115200);
    tx_rate_hz_ = declare_parameter<double>("tx_rate_hz", 5.0);
    reconnect_backoff_s_ = declare_parameter<double>("reconnect_backoff_s", 2.0);

    tx_rate_hz_ = std::clamp(tx_rate_hz_, 1.0, 50.0);

    sub_command_ = create_subscription<avg_msgs::msg::AvgLightCommand>(
      lights_command_topic_, 10,
      [this](avg_msgs::msg::AvgLightCommand::ConstSharedPtr msg) {
        headlight_on_ = msg->headlight;
        indicator_ = msg->indicator;
      });
    pub_status_ = create_publisher<avg_msgs::msg::AvgLightCommand>(lights_status_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / tx_rate_hz_),
      [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(), "mcu_serial_bridge ready: port=%s baud=%d tx=%.1fHz",
      serial_port_.c_str(), serial_baud_, tx_rate_hz_);
  }

  ~McuSerialBridgeNode() override
  {
    closePort();
  }

private:
  static char indicatorToWire(uint8_t indicator)
  {
    switch (indicator) {
      case avg_msgs::msg::AvgLightCommand::INDICATOR_LEFT: return 'L';
      case avg_msgs::msg::AvgLightCommand::INDICATOR_RIGHT: return 'R';
      case avg_msgs::msg::AvgLightCommand::INDICATOR_HAZARD: return 'H';
      default: return 'O';
    }
  }

  static uint8_t wireToIndicator(const std::string & wire)
  {
    if (wire == "L") {return avg_msgs::msg::AvgLightCommand::INDICATOR_LEFT;}
    if (wire == "R") {return avg_msgs::msg::AvgLightCommand::INDICATOR_RIGHT;}
    if (wire == "H") {return avg_msgs::msg::AvgLightCommand::INDICATOR_HAZARD;}
    return avg_msgs::msg::AvgLightCommand::INDICATOR_OFF;
  }

  bool openPort()
  {
    fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      closePort();
      return false;
    }
    cfmakeraw(&tty);
    const speed_t speed = serial_baud_ == 57600 ? B57600 :
      (serial_baud_ == 230400 ? B230400 : B115200);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      closePort();
      return false;
    }
    tcflush(fd_, TCIOFLUSH);
    rx_buffer_.clear();
    return true;
  }

  void closePort()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void onTimer()
  {
    if (fd_ < 0) {
      const auto now_time = now();
      if (last_open_attempt_.nanoseconds() != 0 &&
        (now_time - last_open_attempt_).seconds() < reconnect_backoff_s_)
      {
        return;
      }
      last_open_attempt_ = now_time;
      if (!openPort()) {
        if (!open_failure_logged_) {
          RCLCPP_WARN(
            get_logger(), "light MCU port open failed: %s (retrying every %.1fs)",
            serial_port_.c_str(), reconnect_backoff_s_);
          open_failure_logged_ = true;
        }
        return;
      }
      open_failure_logged_ = false;
      RCLCPP_INFO(get_logger(), "light MCU port opened: %s", serial_port_.c_str());
    }

    sendFrame();
    readAcks();
  }

  void sendFrame()
  {
    ++sequence_;
    nlohmann::json frame;
    frame["h"] = headlight_on_ ? 1 : 0;
    frame["i"] = std::string(1, indicatorToWire(indicator_));
    frame["q"] = sequence_ % 1000000;
    const std::string payload = frame.dump() + "\n";

    const ssize_t written = ::write(fd_, payload.data(), payload.size());
    if (written < 0 || static_cast<std::size_t>(written) != payload.size()) {
      RCLCPP_WARN(get_logger(), "light MCU write failed — reopening port");
      closePort();
    }
  }

  void readAcks()
  {
    if (fd_ < 0) {
      return;
    }
    std::array<char, 256> chunk{};
    while (true) {
      const ssize_t count = ::read(fd_, chunk.data(), chunk.size());
      if (count <= 0) {
        break;
      }
      rx_buffer_.append(chunk.data(), static_cast<std::size_t>(count));
    }
    // Guard against a wedged MCU spamming bytes without newlines.
    if (rx_buffer_.size() > 4096U) {
      rx_buffer_.clear();
      return;
    }

    std::size_t newline_pos = 0U;
    while ((newline_pos = rx_buffer_.find('\n')) != std::string::npos) {
      const std::string line = rx_buffer_.substr(0U, newline_pos);
      rx_buffer_.erase(0U, newline_pos + 1U);
      handleAckLine(line);
    }
  }

  void handleAckLine(const std::string & line)
  {
    if (line.empty()) {
      return;
    }
    nlohmann::json ack = nlohmann::json::parse(line, nullptr, false);
    if (ack.is_discarded() || !ack.is_object()) {
      return;
    }
    avg_msgs::msg::AvgLightCommand status;
    status.header.stamp = now();
    status.headlight = ack.value("h", 0) != 0;
    status.indicator = wireToIndicator(ack.value("i", std::string("O")));
    const int error_code = ack.value("e", 0);
    status.source = error_code == 0 ? "mcu_ack" : ("mcu_error:" + std::to_string(error_code));
    pub_status_->publish(status);
    if (error_code != 0 && error_code != last_error_code_) {
      RCLCPP_WARN(get_logger(), "light MCU reported error code %d", error_code);
    }
    last_error_code_ = error_code;
  }

  std::string lights_command_topic_;
  std::string lights_status_topic_;
  std::string serial_port_;
  int serial_baud_{115200};
  double tx_rate_hz_{5.0};
  double reconnect_backoff_s_{2.0};

  int fd_{-1};
  bool open_failure_logged_{false};
  rclcpp::Time last_open_attempt_{0, 0, RCL_ROS_TIME};
  uint32_t sequence_{0U};
  bool headlight_on_{false};
  uint8_t indicator_{avg_msgs::msg::AvgLightCommand::INDICATOR_OFF};
  std::string rx_buffer_;
  int last_error_code_{0};

  rclcpp::Subscription<avg_msgs::msg::AvgLightCommand>::SharedPtr sub_command_;
  rclcpp::Publisher<avg_msgs::msg::AvgLightCommand>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace camrod_platform

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camrod_platform::McuSerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
