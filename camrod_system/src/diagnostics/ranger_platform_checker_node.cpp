/**
 * Ranger Platform Checker Node
 *
 * ranger_ros2 가 발행하는 섀시 관련 토픽들을 구독하여
 * 플랫폼 상태를 /diagnostics 토픽으로 발행한다.
 *
 * 구독 토픽
 * ---------
 *   /system_state    (ranger_msgs/SystemState)
 *   /battery_state   (sensor_msgs/BatteryState)
 *   /actuator_state  (ranger_msgs/ActuatorStateArray)
 *   /odom            (nav_msgs/Odometry)
 *
 * 진단 항목
 * ---------
 *   /platform/ranger/connection   - 섀시 CAN 통신 연결 (system_state 수신 여부)
 *   /platform/ranger/vehicle      - 차량 상태 (NORMAL / ESTOP / EXCEPTION)
 *   /platform/ranger/control_mode - 제어 모드 (CAN=OK / RC·STANDBY=WARN)
 *   /platform/ranger/error_code   - CAN 오류 코드 비트 분석
 *   /platform/ranger/battery      - 배터리 전압 / SOC / 온도
 *   /platform/ranger/odom         - 오도메트리 freshness & Hz
 *   /platform/ranger/actuator     - 액추에이터 드라이버·모터 상태
 *
 * 주의사항
 * --------
 *   제어 모드 판단 시 ranger_msgs::msg::SystemState 의 CONTROL_MODE_RC 상수(=0)를
 *   사용하지 않는다. ugv_sdk agilex_types.h 기준 실제 CAN 값:
 *     STANDBY=0x00, CAN=0x01, UART=0x02, RC=0x03
 */

#include <cmath>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

#include <ranger_msgs/msg/actuator_state_array.hpp>
#include <ranger_msgs/msg/system_state.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── CAN 오류 코드 비트 (ugv_sdk/details/interface/agilex_types.h) ──────────
static constexpr uint16_t ERR_BATTERY_FAULT  = 0x0001;
static constexpr uint16_t ERR_BATTERY_WARN   = 0x0002;
static constexpr uint16_t ERR_RC_SIGNAL_LOSS = 0x0004;
static constexpr uint16_t ERR_MOTOR1_COMM    = 0x0008;
static constexpr uint16_t ERR_MOTOR2_COMM    = 0x0010;
static constexpr uint16_t ERR_MOTOR3_COMM    = 0x0020;
static constexpr uint16_t ERR_MOTOR4_COMM    = 0x0040;
static constexpr uint16_t ERR_STEER_ENCODER  = 0x0080;
static constexpr uint16_t ERR_MOTOR_DRIVER   = 0x0100;
static constexpr uint16_t ERR_HL_COMM        = 0x0200;

// ── 드라이버 상태 비트 ──────────────────────────────────────────────────────
static constexpr uint8_t DRV_INPUT_VOLT_LOW = 0x01;
static constexpr uint8_t DRV_MOTOR_OVERHEAT = 0x02;
static constexpr uint8_t DRV_OVERLOAD       = 0x04;
static constexpr uint8_t DRV_OVERHEAT       = 0x08;
static constexpr uint8_t DRV_SENSOR_FAULT   = 0x20;
static constexpr uint8_t DRV_FAULT          = 0x40;

// ── 제어 모드 값 (ugv_sdk AgxControlMode) ─────────────────────────────────
static constexpr uint8_t CTRL_STANDBY = 0x00;
static constexpr uint8_t CTRL_CAN     = 0x01;
static constexpr uint8_t CTRL_UART    = 0x02;
static constexpr uint8_t CTRL_RC      = 0x03;

// ── 차량 상태 값 (ugv_sdk AgxVehicleState) ────────────────────────────────
static constexpr uint8_t VEH_NORMAL    = 0x00;
static constexpr uint8_t VEH_ESTOP     = 0x01;
static constexpr uint8_t VEH_EXCEPTION = 0x02;

// ── 주행 모드 이름 ─────────────────────────────────────────────────────────
static const char * motion_mode_name(uint8_t m)
{
  switch (m) {
    case 0: return "DUAL_ACKERMAN";
    case 1: return "PARALLEL";
    case 2: return "SPINNING";
    case 3: return "PARK";
    case 4: return "SIDE_SLIP";
    default: return "UNKNOWN";
  }
}

// ── 플랫폼 상태 구조체 ─────────────────────────────────────────────────────
struct PlatformState
{
  std::mutex mtx;

  // /system_state
  bool has_system_state{false};
  rclcpp::Time system_state_time{0, 0, RCL_ROS_TIME};
  uint8_t  vehicle_state{0};
  uint8_t  control_mode{0};
  uint16_t error_code{0};
  double   battery_voltage_sys{0.0};
  uint8_t  motion_mode{0};

  // /battery_state
  bool has_battery{false};
  rclcpp::Time battery_time{0, 0, RCL_ROS_TIME};
  float batt_voltage{0.0f};
  float batt_current{0.0f};
  float batt_percentage{0.0f};
  float batt_temperature{0.0f};

  // /odom
  bool has_odom{false};
  rclcpp::Time odom_time{0, 0, RCL_ROS_TIME};
  std::deque<rclcpp::Time> odom_timestamps;

  // /actuator_state
  bool has_actuator{false};
  rclcpp::Time actuator_time{0, 0, RCL_ROS_TIME};
  std::vector<ranger_msgs::msg::ActuatorState> actuators;
};

// ── RangerPlatformCheckerNode ──────────────────────────────────────────────
class RangerPlatformCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  RangerPlatformCheckerNode()
  : robot_diagnostics_base::BaseChecker("ranger_platform_checker", "ranger_platform")
  {
    base_init();
  }

protected:
  // ── 파라미터 선언 ──────────────────────────────────────────────────────
  void declare_parameters_() override
  {
    declare_parameter("system_state_topic",  std::string("/system_state"));
    declare_parameter("battery_state_topic", std::string("/battery_state"));
    declare_parameter("actuator_state_topic",std::string("/actuator_state"));
    declare_parameter("odom_topic",          std::string("/odom"));

    declare_parameter("stale_timeout_s",       1.0);

    declare_parameter("odom_expected_hz",    50.0);
    declare_parameter("odom_hz_warn_ratio",  0.7);
    declare_parameter("odom_hz_error_ratio", 0.4);
    declare_parameter("odom_stale_timeout_s",  1.0);

    declare_parameter("battery.voltage_warn",    43.0);
    declare_parameter("battery.voltage_error",   41.0);
    declare_parameter("battery.soc_warn",        20.0);
    declare_parameter("battery.soc_error",       10.0);
    declare_parameter("battery.temp_warn",       45.0);
    declare_parameter("battery.temp_error",      55.0);

    declare_parameter("actuator.driver_temp_warn",    60.0);
    declare_parameter("actuator.driver_temp_error",   75.0);
    declare_parameter("actuator.motor_temp_warn",     70.0);
    declare_parameter("actuator.motor_temp_error",    85.0);
    declare_parameter("actuator.driver_voltage_warn", 40.0);
    declare_parameter("actuator.driver_voltage_error",38.0);
  }

  // ── 파라미터 로드 ──────────────────────────────────────────────────────
  void load_parameters_() override
  {
    system_state_topic_  = get_parameter("system_state_topic").as_string();
    battery_state_topic_ = get_parameter("battery_state_topic").as_string();
    actuator_state_topic_= get_parameter("actuator_state_topic").as_string();
    odom_topic_          = get_parameter("odom_topic").as_string();

    stale_timeout_ = get_param_with_alias<double>("stale_timeout_s", stale_timeout_, {"stale_timeout"});

    odom_expected_hz_    = get_parameter("odom_expected_hz").as_double();
    odom_hz_warn_ratio_  = get_parameter("odom_hz_warn_ratio").as_double();
    odom_hz_error_ratio_ = get_parameter("odom_hz_error_ratio").as_double();
    odom_stale_timeout_ = get_param_with_alias<double>("odom_stale_timeout_s", odom_stale_timeout_, {"odom_stale_timeout"});

    batt_volt_warn_      = get_parameter("battery.voltage_warn").as_double();
    batt_volt_error_     = get_parameter("battery.voltage_error").as_double();
    batt_soc_warn_       = get_parameter("battery.soc_warn").as_double();
    batt_soc_error_      = get_parameter("battery.soc_error").as_double();
    batt_temp_warn_      = get_parameter("battery.temp_warn").as_double();
    batt_temp_error_     = get_parameter("battery.temp_error").as_double();

    act_drv_temp_warn_   = get_parameter("actuator.driver_temp_warn").as_double();
    act_drv_temp_error_  = get_parameter("actuator.driver_temp_error").as_double();
    act_mot_temp_warn_   = get_parameter("actuator.motor_temp_warn").as_double();
    act_mot_temp_error_  = get_parameter("actuator.motor_temp_error").as_double();
    act_drv_volt_warn_   = get_parameter("actuator.driver_voltage_warn").as_double();
    act_drv_volt_error_  = get_parameter("actuator.driver_voltage_error").as_double();
  }

  // ── 진단 태스크 등록 ───────────────────────────────────────────────────
  void setup_tasks_() override
  {
    // /system_state 구독
    system_state_sub_ = create_subscription<ranger_msgs::msg::SystemState>(
      system_state_topic_, 10,
      [this](const ranger_msgs::msg::SystemState::ConstSharedPtr msg) {
        onSystemState(msg);
      });

    // /battery_state 구독
    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      battery_state_topic_, 10,
      [this](const sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {
        onBattery(msg);
      });

    // /actuator_state 구독
    actuator_sub_ = create_subscription<ranger_msgs::msg::ActuatorStateArray>(
      actuator_state_topic_, 10,
      [this](const ranger_msgs::msg::ActuatorStateArray::ConstSharedPtr msg) {
        onActuator(msg);
      });

    // /odom 구독
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        onOdom(msg);
      });

    // 태스크 등록
    add_task("/platform/ranger/connection",
      [this](StatusWrapper & s) { checkConnection(s); });

    add_task("/platform/ranger/vehicle",
      [this](StatusWrapper & s) { checkVehicle(s); });

    add_task("/platform/ranger/control_mode",
      [this](StatusWrapper & s) { checkControlMode(s); });

    add_task("/platform/ranger/error_code",
      [this](StatusWrapper & s) { checkErrorCode(s); });

    add_task("/platform/ranger/battery",
      [this](StatusWrapper & s) { checkBattery(s); });

    add_task("/platform/ranger/odom",
      [this](StatusWrapper & s) { checkOdom(s); });

    add_task("/platform/ranger/actuator",
      [this](StatusWrapper & s) { checkActuator(s); });

    RCLCPP_INFO(get_logger(),
      "Ranger Platform 체커 시작\n"
      "  system_state : %s\n"
      "  battery_state: %s\n"
      "  actuator_state: %s\n"
      "  odom         : %s",
      system_state_topic_.c_str(), battery_state_topic_.c_str(),
      actuator_state_topic_.c_str(), odom_topic_.c_str());
  }

private:
  // ── 콜백 ──────────────────────────────────────────────────────────────

  void onSystemState(const ranger_msgs::msg::SystemState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);
    state_.has_system_state    = true;
    state_.system_state_time   = this->now();
    state_.vehicle_state       = msg->vehicle_state;
    state_.control_mode        = msg->control_mode;
    state_.error_code          = msg->error_code;
    state_.battery_voltage_sys = msg->battery_voltage;
    state_.motion_mode         = msg->motion_mode;
  }

  void onBattery(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);
    state_.has_battery      = true;
    state_.battery_time     = this->now();
    state_.batt_voltage     = msg->voltage;
    state_.batt_current     = msg->current;
    state_.batt_percentage  = msg->percentage;
    state_.batt_temperature = msg->temperature;
  }

  void onActuator(const ranger_msgs::msg::ActuatorStateArray::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);
    state_.has_actuator   = true;
    state_.actuator_time  = this->now();
    state_.actuators      = msg->states;
  }

  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr /*msg*/)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);
    auto now = this->now();
    state_.has_odom   = true;
    state_.odom_time  = now;

    state_.odom_timestamps.push_back(now);
    while (!state_.odom_timestamps.empty() &&
           (now - state_.odom_timestamps.front()).seconds() > 2.0)
    {
      state_.odom_timestamps.pop_front();
    }
  }

  // ── 진단 체크 함수 ─────────────────────────────────────────────────────

  // 1. 섀시 통신 연결 상태
  void checkConnection(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    if (!state_.has_system_state) {
      stat.summary(DiagnosticStatus::STALE,
        "토픽 수신 없음: " + system_state_topic_);
      stat.add("topic", system_state_topic_);
      return;
    }

    double elapsed = (this->now() - state_.system_state_time).seconds();
    if (elapsed > stale_timeout_) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "CAN 통신 끊김 (%.2fs 무응답, timeout=%.1fs)", elapsed, stale_timeout_);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
      stat.add("elapsed_sec", elapsed);
      return;
    }

    char buf[48];
    std::snprintf(buf, sizeof(buf), "OK (%.2fs ago)", elapsed);
    stat.summary(DiagnosticStatus::OK, std::string(buf));
    stat.add("last_msg_sec_ago", elapsed);
    stat.add("motion_mode", motion_mode_name(state_.motion_mode));
  }

  // 2. 차량 상태
  void checkVehicle(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    if (!state_.has_system_state) {
      stat.summary(DiagnosticStatus::STALE, "system_state 수신 없음");
      return;
    }

    const uint8_t vs = state_.vehicle_state;
    int8_t lvl;
    const char * msg_str;

    if (vs == VEH_NORMAL) {
      lvl     = DiagnosticStatus::OK;
      msg_str = "NORMAL";
    } else if (vs == VEH_ESTOP) {
      lvl     = DiagnosticStatus::ERROR;
      msg_str = "비상 정지 (E-STOP) 활성화";
    } else if (vs == VEH_EXCEPTION) {
      lvl     = DiagnosticStatus::ERROR;
      msg_str = "시스템 예외 발생 (EXCEPTION)";
    } else {
      lvl     = DiagnosticStatus::WARN;
      msg_str = "알 수 없는 차량 상태";
    }

    stat.summary(lvl, msg_str);
    stat.add("vehicle_state_raw", static_cast<int>(vs));
    stat.add("motion_mode", motion_mode_name(state_.motion_mode));
  }

  // 3. 제어 모드
  void checkControlMode(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    if (!state_.has_system_state) {
      stat.summary(DiagnosticStatus::STALE, "system_state 수신 없음");
      return;
    }

    // 주의: ranger_msgs::msg::SystemState::CONTROL_MODE_RC = 0 으로 잘못 정의됨.
    //       ugv_sdk agilex_types.h 기준 실제 CAN 값을 직접 사용한다:
    //       STANDBY=0x00, CAN=0x01, UART=0x02, RC=0x03
    const uint8_t cm = state_.control_mode;
    int8_t lvl;
    const char * msg_str;

    if (cm == CTRL_CAN) {
      lvl     = DiagnosticStatus::OK;
      msg_str = "CAN 명령 제어 모드 (정상)";
    } else if (cm == CTRL_RC) {
      lvl     = DiagnosticStatus::WARN;
      msg_str = "RC 리모컨 제어 중 — CAN 명령 무시됨";
    } else if (cm == CTRL_STANDBY) {
      lvl     = DiagnosticStatus::WARN;
      msg_str = "대기 모드 (STANDBY) — CAN 제어 비활성";
    } else if (cm == CTRL_UART) {
      lvl     = DiagnosticStatus::WARN;
      msg_str = "UART 제어 모드 (비정상)";
    } else {
      lvl     = DiagnosticStatus::ERROR;
      msg_str = "알 수 없는 제어 모드";
    }

    stat.summary(lvl, msg_str);
    stat.add("control_mode_raw", static_cast<int>(cm));
  }

  // 4. CAN 오류 코드
  void checkErrorCode(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    if (!state_.has_system_state) {
      stat.summary(DiagnosticStatus::STALE, "system_state 수신 없음");
      return;
    }

    const uint16_t ec = state_.error_code;

    if (ec == 0) {
      stat.summary(DiagnosticStatus::OK, "오류 없음");
      stat.add("error_code_hex", "0x0000");
      return;
    }

    // ERROR 비트 판정
    bool has_error =
      (ec & ERR_BATTERY_FAULT) || (ec & ERR_MOTOR1_COMM) ||
      (ec & ERR_MOTOR2_COMM)   || (ec & ERR_MOTOR3_COMM) ||
      (ec & ERR_MOTOR4_COMM)   || (ec & ERR_STEER_ENCODER) ||
      (ec & ERR_MOTOR_DRIVER)  || (ec & ERR_HL_COMM);

    bool has_warn =
      (ec & ERR_BATTERY_WARN) || (ec & ERR_RC_SIGNAL_LOSS);

    int8_t lvl = has_error ? DiagnosticStatus::ERROR
               : has_warn  ? DiagnosticStatus::WARN
               : DiagnosticStatus::WARN;

    char hex[12];
    std::snprintf(hex, sizeof(hex), "0x%04X", static_cast<unsigned>(ec));
    stat.summary(lvl, std::string("오류 코드 활성: ") + hex);

    stat.add("error_code_hex", std::string(hex));

    stat.add("BATTERY_FAULT",  (ec & ERR_BATTERY_FAULT)  ? "active" : "-");
    stat.add("BATTERY_WARN",   (ec & ERR_BATTERY_WARN)   ? "active" : "-");
    stat.add("RC_SIGNAL_LOSS", (ec & ERR_RC_SIGNAL_LOSS) ? "active" : "-");
    stat.add("MOTOR1_COMM",    (ec & ERR_MOTOR1_COMM)    ? "active" : "-");
    stat.add("MOTOR2_COMM",    (ec & ERR_MOTOR2_COMM)    ? "active" : "-");
    stat.add("MOTOR3_COMM",    (ec & ERR_MOTOR3_COMM)    ? "active" : "-");
    stat.add("MOTOR4_COMM",    (ec & ERR_MOTOR4_COMM)    ? "active" : "-");
    stat.add("STEER_ENCODER",  (ec & ERR_STEER_ENCODER)  ? "active" : "-");
    stat.add("MOTOR_DRIVER",   (ec & ERR_MOTOR_DRIVER)   ? "active" : "-");
    stat.add("HL_COMM",        (ec & ERR_HL_COMM)        ? "active" : "-");
  }

  // 5. 배터리
  void checkBattery(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    if (!state_.has_battery) {
      // battery_state 없으면 system_state 전압으로 대체
      if (!state_.has_system_state) {
        stat.summary(DiagnosticStatus::STALE, "battery_state 수신 없음");
        return;
      }
      double v = state_.battery_voltage_sys;
      int8_t lvl = check_low(v, batt_volt_warn_, batt_volt_error_);
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%.1f V (system_state 기준)", v);
      stat.summary(lvl, std::string(buf));
      stat.add("voltage_V (system_state)", v);
      return;
    }

    // 전압 레벨
    int8_t volt_lvl = check_low(
      static_cast<double>(state_.batt_voltage), batt_volt_warn_, batt_volt_error_);

    // SOC 레벨
    int8_t soc_lvl = check_low(
      static_cast<double>(state_.batt_percentage), batt_soc_warn_, batt_soc_error_);

    // 온도 레벨
    int8_t temp_lvl = check_high(
      static_cast<double>(state_.batt_temperature), batt_temp_warn_, batt_temp_error_);

    int8_t lvl = std::max({volt_lvl, soc_lvl, temp_lvl});

    char buf[80];
    if (lvl == DiagnosticStatus::OK) {
      std::snprintf(buf, sizeof(buf),
        "OK (%.1f V, %.0f%%, %.1f°C)",
        state_.batt_voltage, state_.batt_percentage, state_.batt_temperature);
    } else if (volt_lvl >= soc_lvl && volt_lvl >= temp_lvl) {
      std::snprintf(buf, sizeof(buf), "배터리 전압 낮음 (%.1f V)", state_.batt_voltage);
    } else if (soc_lvl >= temp_lvl) {
      std::snprintf(buf, sizeof(buf), "배터리 잔량 낮음 (%.0f%%)", state_.batt_percentage);
    } else {
      std::snprintf(buf, sizeof(buf), "배터리 온도 이상 (%.1f°C)", state_.batt_temperature);
    }

    stat.summary(lvl, std::string(buf));

    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.2f", static_cast<double>(state_.batt_voltage));
    stat.add("voltage_V",     std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", static_cast<double>(state_.batt_current));
    stat.add("current_A",     std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", static_cast<double>(state_.batt_percentage));
    stat.add("soc_%",         std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", static_cast<double>(state_.batt_temperature));
    stat.add("temperature_C", std::string(tmp));
  }

  // 6. 오도메트리
  void checkOdom(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    if (!state_.has_odom) {
      stat.summary(DiagnosticStatus::STALE, "토픽 수신 없음: " + odom_topic_);
      stat.add("topic", odom_topic_);
      return;
    }

    double elapsed = (this->now() - state_.odom_time).seconds();
    if (elapsed > odom_stale_timeout_) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "오도메트리 끊김 (%.2fs 무응답, timeout=%.1fs)",
        elapsed, odom_stale_timeout_);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
      stat.add("elapsed_sec", elapsed);
      return;
    }

    // Hz 계산 (2s rolling window)
    double actual_hz = 0.0;
    if (state_.odom_timestamps.size() >= 2) {
      double window = (state_.odom_timestamps.back() -
                       state_.odom_timestamps.front()).seconds();
      if (window > 0.0) {
        actual_hz = static_cast<double>(state_.odom_timestamps.size() - 1) / window;
      }
    }

    int8_t lvl = DiagnosticStatus::OK;
    std::string msg_str;

    if (odom_expected_hz_ > 0.0) {
      double ratio = actual_hz / odom_expected_hz_;
      lvl = check_low(ratio, odom_hz_warn_ratio_, odom_hz_error_ratio_);
    }

    if (lvl == DiagnosticStatus::OK) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "OK (%.0f Hz)", actual_hz);
      msg_str = buf;
    } else {
      char buf[64];
      std::snprintf(buf, sizeof(buf),
        "수신 속도 %s (%.0f / %.0f Hz)",
        lvl == DiagnosticStatus::ERROR ? "심각 저하" : "저하",
        actual_hz, odom_expected_hz_);
      msg_str = buf;
    }

    stat.summary(lvl, msg_str);

    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", actual_hz);
    stat.add("actual_hz",   std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", odom_expected_hz_);
    stat.add("expected_hz", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", elapsed);
    stat.add("last_msg_sec_ago", std::string(tmp));
  }

  // 7. 액추에이터 상태
  void checkActuator(StatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(state_.mtx);

    if (!state_.has_actuator) {
      stat.summary(DiagnosticStatus::STALE, "actuator_state 수신 없음");
      stat.add("topic", actuator_state_topic_);
      return;
    }

    double elapsed = (this->now() - state_.actuator_time).seconds();
    if (elapsed > stale_timeout_) {
      char buf[96];
      std::snprintf(buf, sizeof(buf),
        "액추에이터 데이터 끊김 (%.2fs)", elapsed);
      stat.summary(DiagnosticStatus::STALE, std::string(buf));
      return;
    }

    int8_t worst_lvl = DiagnosticStatus::OK;
    std::string worst_msg = "OK";
    int fault_count = 0;

    for (const auto & act : state_.actuators) {
      const uint32_t drv_state = act.driver.driver_state;
      const double   drv_temp  = act.driver.driver_temperature;
      const double   mot_temp  = act.driver.motor_temperature;
      const double   drv_volt  = act.driver.driver_voltage;

      // 드라이버 상태 비트 체크
      if (drv_state & (DRV_FAULT | DRV_SENSOR_FAULT)) {
        worst_lvl = DiagnosticStatus::ERROR;
        char buf[64];
        std::snprintf(buf, sizeof(buf),
          "액추에이터[%u] 드라이버 결함", act.id);
        worst_msg = buf;
        fault_count++;
      } else if (drv_state & (DRV_OVERHEAT | DRV_MOTOR_OVERHEAT |
                               DRV_OVERLOAD | DRV_INPUT_VOLT_LOW)) {
        if (worst_lvl < DiagnosticStatus::WARN) {
          worst_lvl = DiagnosticStatus::WARN;
          char buf[64];
          std::snprintf(buf, sizeof(buf),
            "액추에이터[%u] 경고 비트 활성", act.id);
          worst_msg = buf;
        }
        fault_count++;
      }

      // 드라이버 온도
      int8_t dtlvl = check_high(drv_temp, act_drv_temp_warn_, act_drv_temp_error_);
      if (dtlvl > worst_lvl) {
        worst_lvl = dtlvl;
        char buf[64];
        std::snprintf(buf, sizeof(buf),
          "액추에이터[%u] 드라이버 온도 %.0f°C", act.id, drv_temp);
        worst_msg = buf;
      }

      // 모터 온도
      int8_t mtlvl = check_high(mot_temp, act_mot_temp_warn_, act_mot_temp_error_);
      if (mtlvl > worst_lvl) {
        worst_lvl = mtlvl;
        char buf[64];
        std::snprintf(buf, sizeof(buf),
          "액추에이터[%u] 모터 온도 %.0f°C", act.id, mot_temp);
        worst_msg = buf;
      }

      // 드라이버 전압
      int8_t dvlvl = check_low(drv_volt, act_drv_volt_warn_, act_drv_volt_error_);
      if (dvlvl > worst_lvl) {
        worst_lvl = dvlvl;
        char buf[64];
        std::snprintf(buf, sizeof(buf),
          "액추에이터[%u] 드라이버 전압 %.1f V", act.id, drv_volt);
        worst_msg = buf;
      }

      // 상세 정보
      char key[32], val[32];
      std::snprintf(key, sizeof(key), "act[%u].driver_voltage_V", act.id);
      std::snprintf(val, sizeof(val), "%.2f", drv_volt);
      stat.add(std::string(key), std::string(val));

      std::snprintf(key, sizeof(key), "act[%u].driver_temp_C", act.id);
      std::snprintf(val, sizeof(val), "%.1f", drv_temp);
      stat.add(std::string(key), std::string(val));

      std::snprintf(key, sizeof(key), "act[%u].motor_temp_C", act.id);
      std::snprintf(val, sizeof(val), "%.1f", mot_temp);
      stat.add(std::string(key), std::string(val));

      std::snprintf(key, sizeof(key), "act[%u].rpm", act.id);
      std::snprintf(val, sizeof(val), "%d", act.motor.rpm);
      stat.add(std::string(key), std::string(val));
    }

    if (worst_lvl == DiagnosticStatus::OK) {
      char buf[48];
      std::snprintf(buf, sizeof(buf),
        "OK (%zu 액추에이터)", state_.actuators.size());
      worst_msg = buf;
    }

    stat.summary(worst_lvl, worst_msg);
    stat.add("actuator_count", static_cast<int>(state_.actuators.size()));
    if (fault_count > 0) {
      stat.add("fault_count", fault_count);
    }
  }

  // ── 멤버 변수 ──────────────────────────────────────────────────────────
  PlatformState state_;

  std::string system_state_topic_;
  std::string battery_state_topic_;
  std::string actuator_state_topic_;
  std::string odom_topic_;

  double stale_timeout_{1.0};

  double odom_expected_hz_{50.0};
  double odom_hz_warn_ratio_{0.7};
  double odom_hz_error_ratio_{0.4};
  double odom_stale_timeout_{1.0};

  double batt_volt_warn_{43.0};
  double batt_volt_error_{41.0};
  double batt_soc_warn_{20.0};
  double batt_soc_error_{10.0};
  double batt_temp_warn_{45.0};
  double batt_temp_error_{55.0};

  double act_drv_temp_warn_{60.0};
  double act_drv_temp_error_{75.0};
  double act_mot_temp_warn_{70.0};
  double act_mot_temp_error_{85.0};
  double act_drv_volt_warn_{40.0};
  double act_drv_volt_error_{38.0};

  rclcpp::Subscription<ranger_msgs::msg::SystemState>::SharedPtr    system_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr   battery_sub_;
  rclcpp::Subscription<ranger_msgs::msg::ActuatorStateArray>::SharedPtr actuator_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RangerPlatformCheckerNode>());
  rclcpp::shutdown();
  return 0;
}
