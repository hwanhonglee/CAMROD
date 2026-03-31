# robot_common_cpp

로봇의 모든 컴포넌트(하드웨어, 센서, 액추에이터, 통신 등)에서 공통으로 사용하는 **C++ 베이스 클래스 모음**입니다.
현재는 `BaseChecker` 하나를 제공하며, `diagnostic_updater` 기반으로 로봇 상태를 ROS2 진단 토픽으로 발행합니다.

---

## 목차

1. [패키지 개요](#1-패키지-개요)
2. [BaseChecker 란?](#2-basechecker-란)
3. [핵심 개념 — Template Method 패턴](#3-핵심-개념--template-method-패턴)
4. [초기화 순서 (왜 base_init() 을 직접 호출해야 하나?)](#4-초기화-순서-왜-base_init-을-직접-호출해야-하나)
5. [레벨 판정 헬퍼 함수](#5-레벨-판정-헬퍼-함수)
6. [내 체커 만들기 — 단계별 가이드](#6-내-체커-만들기--단계별-가이드)
7. [CMakeLists.txt 에서 의존성 추가하기](#7-cmakeliststxt-에서-의존성-추가하기)
8. [자주 하는 실수 & 주의사항](#8-자주-하는-실수--주의사항)
9. [의존 패키지](#9-의존-패키지)

---

## 1. 패키지 개요

| 항목 | 내용 |
|------|------|
| 패키지 이름 | `robot_common_cpp` |
| 빌드 방식 | `ament_cmake` (헤더 전용 INTERFACE 라이브러리) |
| 헤더 위치 | `include/robot_common_cpp/base_checker.hpp` |
| 네임스페이스 | `robot_common` |

> **헤더 전용(Header-Only)** 라이브러리이기 때문에 `.cpp` 소스 파일이 없고, `#include` 한 번으로 모든 구현을 가져올 수 있습니다.

---

## 2. BaseChecker 란?

`BaseChecker`는 **ROS2 진단(Diagnostics) 노드를 쉽게 만들기 위한 추상 기반 클래스**입니다.

ROS2에는 `diagnostic_updater`라는 패키지가 있습니다.
이 패키지는 주기적으로 `/diagnostics` 토픽에 센서·하드웨어의 상태(OK / WARN / ERROR)를 발행하는 표준 방법을 제공합니다.

하지만 매번 노드를 새로 만들 때마다 같은 설정 코드를 반복해야 합니다.
`BaseChecker`는 그 반복 작업을 **한 번만 구현**해 두고, 개발자는 **"어떤 값을 진단할지"만 집중**할 수 있도록 만들어진 클래스입니다.

```
              ┌─────────────────────────┐
              │      rclcpp::Node        │  ← ROS2 기본 노드
              └────────────┬────────────┘
                           │ 상속
              ┌────────────▼────────────┐
              │       BaseChecker        │  ← 이 패키지가 제공
              │  - diagnostic_updater   │
              │  - 레벨 판정 헬퍼      │
              │  - 파라미터 초기화 틀  │
              └────────────┬────────────┘
                           │ 상속
         ┌─────────────────┼─────────────────┐
         │                 │                 │
  BatteryChecker    TemperatureChecker   GpsChecker   ← 개발자가 만드는 클래스
```

---

## 3. 핵심 개념 — Template Method 패턴

`BaseChecker`는 **Template Method 패턴**을 사용합니다.

> Template Method 패턴이란?
> "전체 알고리즘의 뼈대(순서)는 기본 클래스가 고정하고,
> 각 단계의 세부 내용만 서브클래스가 채우는 설계 방식"입니다.

`BaseChecker`에서 고정된 초기화 순서는 다음과 같습니다:

```
base_init() 호출
       │
       ▼
1. declare_parameters_()   ← 파라미터 이름과 기본값 선언
       │
       ▼
2. Updater 생성            ← BaseChecker 내부가 자동 처리
       │
       ▼
3. load_parameters_()      ← 파라미터 값을 읽어 멤버 변수에 저장
       │
       ▼
4. setup_tasks_()          ← 진단 작업(콜백 함수) 등록
```

개발자는 이 네 단계 중 **1, 3, 4번만 오버라이드**하면 됩니다.
(파라미터가 없다면 1번과 3번은 생략 가능합니다.)

| 메서드 | 필수 여부 | 역할 |
|--------|-----------|------|
| `declare_parameters_()` | 선택 | ROS2 파라미터 이름·기본값 선언 |
| `load_parameters_()` | 선택 | 파라미터 값을 읽어 멤버 변수에 저장 |
| `setup_tasks_()` | **필수** | `add_task()` 로 진단 콜백 등록 |

---

## 4. 초기화 순서 (왜 base_init() 을 직접 호출해야 하나?)

C++의 중요한 규칙이 있습니다:

> **기본 클래스의 생성자가 실행되는 시점에는 파생 클래스의 가상 함수 테이블(vtable)이 아직 완성되지 않습니다.**

쉽게 말하면, `BaseChecker`의 생성자 안에서 `setup_tasks_()`를 바로 호출하면
아직 `BatteryChecker::setup_tasks_()` 가 "연결"되기 전이라 오류가 발생합니다.

그래서 `BaseChecker`는 초기화 로직을 생성자 밖의 `base_init()` 메서드로 분리했습니다.
**서브클래스의 생성자가 완전히 실행된 뒤** `base_init()`을 호출해야만
파생 클래스의 `setup_tasks_()`가 올바르게 호출됩니다.

```cpp
// 반드시 이 구조를 따르세요
class BatteryChecker : public robot_common::BaseChecker
{
public:
  BatteryChecker() : BaseChecker("battery_checker", "battery")
  {
    // ... 멤버 변수 초기화 ...

    base_init();  // ← 반드시 생성자 마지막에 호출!
  }
  // ...
};
```

---

## 5. 레벨 판정 헬퍼 함수

`stat.summary(레벨, 메시지)` 를 호출할 때 레벨 값을 직접 계산하지 않아도 되도록
4가지 정적(static) 헬퍼 함수를 제공합니다.

### 5-1. `level()` — 높을수록 위험한 값

CPU 사용률, 온도, 메모리 사용량처럼 **값이 클수록 위험**한 경우에 사용합니다.

```
값 >= error  →  ERROR
값 >= warn   →  WARN
그 외        →  OK
```

```cpp
// CPU 사용률: 90% 이상이면 ERROR, 80% 이상이면 WARN
int8_t lv = level(cpu_usage, /*warn=*/80.0, /*error=*/90.0);
stat.summary(lv, "CPU " + std::to_string(cpu_usage) + "%");
```

### 5-2. `level_low()` — 낮을수록 위험한 값

배터리 전압, 신호 강도처럼 **값이 작을수록 위험**한 경우에 사용합니다.

```
값 <= error  →  ERROR
값 <= warn   →  WARN
그 외        →  OK
```

```cpp
// 배터리 전압: 20V 이하면 ERROR, 22V 이하면 WARN
int8_t lv = level_low(voltage, /*warn=*/22.0, /*error=*/20.0);
stat.summary(lv, "Battery " + std::to_string(voltage) + " V");
```

### 5-3. `level_bool()` — 참/거짓 상태

센서 연결 여부, 오류 플래그처럼 **true/false로 판단**하는 경우에 사용합니다.

```
ok == true   →  OK
ok == false  →  ERROR (기본값, 두 번째 인수로 WARN 으로 바꿀 수 있음)
```

```cpp
bool connected = check_sensor_connection();
int8_t lv = level_bool(connected);                       // false → ERROR
int8_t lv = level_bool(connected, DiagnosticStatus::WARN); // false → WARN
stat.summary(lv, connected ? "Connected" : "Disconnected");
```

### 5-4. `level_enum()` — 열거형/상태 코드

GPS Fix 타입, 드라이브 모드 등 **미리 정의된 상태 목록**으로 판단하는 경우에 사용합니다.

```
state ∈ ok_states    →  OK
state ∈ warn_states  →  WARN
둘 다 아님           →  ERROR
```

```cpp
// GPS fix_type: 3D Fix(3), RTK Fixed(4), RTK Float(5) 이면 OK
// No Fix(1) 이면 WARN, 나머지는 ERROR
int8_t lv = level_enum(fix_type,
  /*ok_states=*/  {3, 4, 5},
  /*warn_states=*/{1}
);
```

---

## 6. 내 체커 만들기 — 단계별 가이드

### 예시: 배터리 전압을 진단하는 `BatteryChecker`

#### Step 1 — 헤더 포함 및 클래스 선언

```cpp
#include "robot_common_cpp/base_checker.hpp"

class BatteryChecker : public robot_common::BaseChecker
{
public:
  BatteryChecker();

protected:
  void declare_parameters_() override;
  void load_parameters_()    override;
  void setup_tasks_()        override;

private:
  void check_battery(diagnostic_updater::DiagnosticStatusWrapper & stat);

  double warn_v_{22.0};   // 경고 전압 (기본값)
  double error_v_{20.0};  // 오류 전압 (기본값)
};
```

#### Step 2 — 생성자: `base_init()` 마지막에 호출

```cpp
BatteryChecker::BatteryChecker()
: robot_common::BaseChecker(
    "battery_checker",  // ROS2 노드 이름
    "battery"           // hardware_id (진단 메시지에 표시되는 장치 이름)
  )
{
  base_init();  // ← 반드시 마지막에!
}
```

#### Step 3 — 파라미터 선언

```cpp
void BatteryChecker::declare_parameters_()
{
  declare_parameter("warn_voltage",  22.0);  // 파라미터 이름, 기본값
  declare_parameter("error_voltage", 20.0);
}
```

> `ros2 run` 또는 `ros2 launch` 시 `--ros-args -p warn_voltage:=21.5` 로 외부에서 바꿀 수 있습니다.

#### Step 4 — 파라미터 로드

```cpp
void BatteryChecker::load_parameters_()
{
  warn_v_  = get_parameter("warn_voltage").as_double();
  error_v_ = get_parameter("error_voltage").as_double();
}
```

#### Step 5 — 진단 작업 등록

```cpp
void BatteryChecker::setup_tasks_()
{
  // "/power/battery" 라는 이름으로 진단 항목을 등록
  add_task("/power/battery", [this](auto & stat) {
    check_battery(stat);
  });
}
```

#### Step 6 — 실제 진단 콜백 구현

```cpp
void BatteryChecker::check_battery(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  double v = read_voltage();  // 실제 전압 읽기 (하드웨어 드라이버 호출)

  // level_low: 낮을수록 위험 → warn_v_ 이하면 WARN, error_v_ 이하면 ERROR
  stat.summary(level_low(v, warn_v_, error_v_),
               "Battery " + std::to_string(v) + " V");

  stat.add("voltage_V", v);  // 추가 키-값 정보
}
```

#### 완성된 진단 메시지 (예시)

```
/diagnostics 토픽:
  name:        /power/battery
  hardware_id: battery
  level:       0 (OK)
  message:     Battery 23.4 V
  values:
    - key:   voltage_V
      value: 23.4
```

---

## 7. CMakeLists.txt 에서 의존성 추가하기

내 패키지에서 `robot_common_cpp`를 사용하려면 아래와 같이 설정합니다.

```cmake
# CMakeLists.txt

find_package(robot_common_cpp REQUIRED)

add_executable(battery_checker_node src/battery_checker_node.cpp)
ament_target_dependencies(battery_checker_node robot_common_cpp)
```

`package.xml`에도 추가:

```xml
<depend>robot_common_cpp</depend>
```

---

## 8. 자주 하는 실수 & 주의사항

### `base_init()` 을 생성자 마지막에 호출하지 않은 경우

```cpp
// 잘못된 예
BatteryChecker() : BaseChecker("battery_checker", "battery")
{
  // base_init() 없음!  → setup_tasks_() 가 호출되지 않음
  //                       → /diagnostics 에 아무것도 발행 안 됨
}
```

```cpp
// 올바른 예
BatteryChecker() : BaseChecker("battery_checker", "battery")
{
  base_init();  // ← 반드시!
}
```

### `setup_tasks_()` 를 구현하지 않은 경우

`setup_tasks_()`는 순수 가상 함수(pure virtual)입니다.
반드시 오버라이드하지 않으면 **컴파일 오류**가 발생합니다.

### `level()` 과 `level_low()` 혼동

| 상황 | 사용할 함수 |
|------|------------|
| 값이 크면 위험 (CPU, 온도, 메모리) | `level()` |
| 값이 작으면 위험 (전압, 신호 강도) | `level_low()` |

### `publish_rate` 파라미터

`BaseChecker`는 기본적으로 `publish_rate` 파라미터(기본값: `1.0` Hz)를 내장합니다.
진단 메시지를 더 자주 발행하려면 실행 시 파라미터를 바꾸세요:

```bash
ros2 run my_pkg battery_checker_node --ros-args -p publish_rate:=5.0
```

---

## 9. 의존 패키지

| 패키지 | 역할 |
|--------|------|
| `rclcpp` | ROS2 C++ 클라이언트 라이브러리 |
| `diagnostic_msgs` | `DiagnosticStatus` 메시지 타입 (OK/WARN/ERROR 레벨 상수 포함) |
| `diagnostic_updater` | 주기적으로 `/diagnostics` 토픽을 발행하는 Updater 클래스 |
