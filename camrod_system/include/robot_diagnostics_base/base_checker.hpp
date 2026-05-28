#pragma once

#include <algorithm>
#include <functional>
#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robot_diagnostics_base
{

/**
 * diagnostic_updater::Updater 를 내장한 추상 베이스 체커 노드.
 * 로봇의 모든 컴포넌트(HW, 센서, 액추에이터, 통신 등)에서 공통으로 사용한다.
 *
 * Template Method 패턴 (C++ 버전)
 * ---------------------------------
 * C++ 에서는 기본 클래스 생성자 안에서 순수 가상 함수를 호출할 수 없으므로
 * 서브클래스 생성자 마지막에 base_init() 을 반드시 호출해야 한다.
 *
 *   1. declare_parameters_()  → ROS2 파라미터 선언
 *   2. load_parameters_()     → 파라미터 읽기 & 멤버 변수 초기화
 *   3. setup_tasks_()         → add_task() 로 진단 작업 등록
 *
 * 사용 예시
 * ---------
 *   class BatteryChecker : public robot_common::BaseChecker
 *   {
 *   public:
 *     BatteryChecker() : BaseChecker("battery_checker", "battery") {
 *       base_init();  // ← 생성자 마지막에 반드시 호출
 *     }
 *
 *   protected:
 *     void declare_parameters_() override {
 *       declare_parameter("warn_voltage", 22.0);
 *       declare_parameter("error_voltage", 20.0);
 *     }
 *     void load_parameters_() override {
 *       warn_v_  = get_parameter("warn_voltage").as_double();
 *       error_v_ = get_parameter("error_voltage").as_double();
 *     }
 *     void setup_tasks_() override {
 *       add_task("/power/battery", [this](auto & stat) { check_battery(stat); });
 *     }
 *
 *   private:
 *     void check_battery(diagnostic_updater::DiagnosticStatusWrapper & stat) {
 *       double v = read_voltage();
 *       stat.summary(check_low(v, warn_v_, error_v_), "Battery " + std::to_string(v) + " V");
 *       stat.add("voltage_V", v);
 *     }
 *     double warn_v_{22.0}, error_v_{20.0};
 *   };
 */
class BaseChecker : public rclcpp::Node
{
public:
  using S       = diagnostic_msgs::msg::DiagnosticStatus;
  using TaskFn  = std::function<void(diagnostic_updater::DiagnosticStatusWrapper &)>;

  explicit BaseChecker(
    const std::string & node_name,
    const std::string & hardware_id = "none")
  : rclcpp::Node(node_name), hardware_id_(hardware_id)
  {
    // Declare both canonical and legacy publish-rate parameters.
    // Canonical: publish_rate_hz
    // Legacy:    publish_rate
    // ⚠️ C++ 제약: 기본 클래스 생성자에서 순수 가상 함수(setup_tasks_)를 호출하면
    //    vtable 이 파생 클래스로 완성되기 전이므로 링커 오류가 발생한다.
    //    → 서브클래스 생성자 마지막에 base_init() 을 반드시 호출한다.
    declare_parameter("publish_rate_hz", 1.0);
    declare_parameter("publish_rate", 1.0);
  }

  // ── 공개 API ─────────────────────────────────────────────────────────

  void add_task(const std::string & name, TaskFn callback)
  {
    updater_->add(name, callback);
  }

  // ── 레벨 판정 헬퍼 ───────────────────────────────────────────────────

  /** 높을수록 위험 (CPU 사용률, 온도, 메모리 등). */
  static int8_t check_high(double value, double warn, double error)
  {
    if (value >= error) return S::ERROR;
    if (value >= warn)  return S::WARN;
    return S::OK;
  }

  /** 낮을수록 위험 (배터리 전압, 신호 강도 등). */
  static int8_t check_low(double value, double warn, double error)
  {
    if (value <= error) return S::ERROR;
    if (value <= warn)  return S::WARN;
    return S::OK;
  }

  /** Boolean 상태 (센서 연결 여부, Fault Flag 등). */
  static int8_t check_flag(bool ok, int8_t on_false = S::ERROR)
  {
    return ok ? static_cast<int8_t>(S::OK) : on_false;
  }

  /**
   * Enum / 상태 코드 (GPS Fix Type, 드라이브 모드 등).
   * state ∈ ok_states → OK, state ∈ warn_states → WARN, 둘 다 아니면 ERROR.
   */
  template<typename T>
  static int8_t check_enum(
    const T & state,
    std::initializer_list<T> ok_states,
    std::initializer_list<T> warn_states = {})
  {
    for (const auto & s : ok_states)   { if (state == s) return S::OK;   }
    for (const auto & s : warn_states) { if (state == s) return S::WARN; }
    return S::ERROR;
  }

  template<typename T>
  static int8_t check_enum(
    const T & state,
    const std::vector<T> & ok_states,
    const std::vector<T> & warn_states = {})
  {
    for (const auto & s : ok_states)   { if (state == s) return S::OK;   }
    for (const auto & s : warn_states) { if (state == s) return S::WARN; }
    return S::ERROR;
  }

  /**
   * Read a canonical parameter key.
   *
   * HH_260522: legacy aliases are intentionally ignored so diagnostics
   * use one canonical naming scheme across modules.
   */
  template<typename T>
  T get_param_with_alias(
    const std::string & canonical_name,
    const T & default_value,
    const std::vector<std::string> & legacy_names = {})
  {
    (void)legacy_names;
    if (!has_parameter(canonical_name)) {
      declare_parameter(canonical_name, default_value);
    }
    return get_parameter(canonical_name).template get_value<T>();
  }

protected:
  // ── Template Method 훅 ────────────────────────────────────────────────

  /// 파라미터 선언 (파라미터 없으면 오버라이드 불필요)
  virtual void declare_parameters_() {}

  /// 파라미터 로드 (파라미터 없으면 오버라이드 불필요)
  virtual void load_parameters_() {}

  /// 진단 작업 등록 (필수 구현)
  virtual void setup_tasks_() = 0;

  /**
   * Template Method 초기화 — 서브클래스 생성자 마지막에 반드시 호출.
   *
   * 호출 순서:
   *   declare_parameters_() → Updater 생성 → load_parameters_() → setup_tasks_()
   */
  void base_init()
  {
    declare_parameters_();

    double rate_hz = get_param_with_alias<double>(
      "publish_rate_hz", 1.0, {"publish_rate"});
    if (rate_hz <= 1e-6) {
      RCLCPP_WARN(
        get_logger(),
        "publish_rate_hz must be > 0. Clamping to 1.0 Hz.");
      rate_hz = 1.0;
    }
    updater_ = std::make_unique<diagnostic_updater::Updater>(this, 1.0 / rate_hz);
    updater_->setHardwareID(hardware_id_);

    load_parameters_();
    setup_tasks_();
  }

  std::unique_ptr<diagnostic_updater::Updater> updater_;

private:
  std::string hardware_id_;
};

}  // namespace robot_diagnostics_base
