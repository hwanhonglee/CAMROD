# Camping Cart Autonomous Platform

본 리포지토리는 캠핑 카트를 자율주행 로봇으로 확장하기 위한 ROS 2 기반 소프트웨어 스택을 정리하고 구현하는 공간입니다. 하드웨어 구성, 핵심 소프트웨어 패키지, 시뮬레이션·테스트 절차, 안전 요구사항까지 전체 로드맵을 문서화하고, 실제 코드를 통해 단계적으로 실현하는 것을 목표로 합니다.

## 프로젝트 목적 및 범위
- **목표**: 차동구동 캠핑 카트가 야외 환경에서 스스로 주행하며, 원격 모니터링과 안전 절차를 충족하도록 제어·인지·계획 파이프라인을 완성합니다.
- **범위**: ROS 2 Humble, Lanelet2 HD 맵, GNSS+IMU 융합, LiDAR 기반 장애물 감지, Pure Pursuit/MPC 제어, Gazebo/Ignition 시뮬레이션, E-Stop 및 시스템 모니터링.
- **진행 원칙**: 소프트웨어·하드웨어 요구사항을 동기화하고, 체크리스트 기반으로 구현-테스트-피드백 루프를 반복합니다.

## 시스템 아키텍처 개요
1. **하드웨어 계층**: 좌/우 독립 구동 모터, BMS 포함 배터리, ROS 2 호환 SBC + MCU, GNSS RTK, IMU, 전후방 LiDAR/카메라, 근접 센서, 무선 통신 모듈.
2. **소프트웨어 계층**:
   - 로봇 정보/TF 브로드캐스트(`camping_cart_common`, `camping_cart_bringup`).
   - Lanelet2 맵 로딩과 시각화(`camping_cart_map`).
   - GNSS+IMU 융합 및 맵 매칭 로컬라이제이션(계획 중 `camping_cart_localization`).
   - 글로벌/로컬/비헤이비어 플래닝(`camping_cart_planning`).
   - LiDAR·카메라 기반 퍼셉션(`camping_cart_perception`).
   - Pure Pursuit/MPC 제어 및 안전 감시(계획 중 `camping_cart_control`).
   - 시스템 모니터링·UI, 시뮬레이션·테스트 인프라.
3. **운영 시나리오**: 수동↔자율 전환, 야외 맵 주행, 사람 추종 모드, 로그 수집과 원격 모니터링.

## 현재 구현된 패키지
| 패키지 | 역할 |
| --- | --- |
| `camping_cart_common` | 로봇 제원·센서 위치 파라미터를 `RobotInfo` 메시지로 퍼블리시하고, 공용 메시지/헬퍼 코드를 제공. |
| `camping_cart_bringup` | 대표 노드들을 실행하는 launch 구성을 관리. 현재는 `robot_info_node` 중심 bringup. |
| `camping_cart_map` | Lanelet2 `.osm` 맵을 로드해 `/map`, `/lanelet2map` 토픽을 퍼블리시하고 시각화를 제공. |
| `camping_cart_planning` | `/goal_pose` 입력을 받아 `/planning/global_path`를 내보내는 전역 플래너 뼈대. |
| `camping_cart_perception` | LiDAR/카메라 파이프라인을 위한 비어있는 패키지 스캐폴드. |
| `lanelet2_map.osm` | 샘플 HD 맵 데이터. |

## 향후 생성·확장 예정 패키지
1. `camping_cart_localization`: GNSS RTK + IMU 융합(EKF/UKF), Lanelet2 매칭, `nav_msgs/Odometry`/`PoseWithCovariance` 출력.
2. `camping_cart_control`: Pure Pursuit/MPC 제어, 안전 한계 감시, 모터 인터페이스.
3. `camping_cart_perception` 상세화: LiDAR 장애물 리스트, 근접 센서 통합, 객체 분류(옵션 카메라).
4. `camping_cart_system_monitor`(가칭): 센서 주기·RTK 상태·E-Stop 토픽 감시, UI/대시보드 연동.
5. `camping_cart_simulation`: URDF/xacro 모델, Gazebo/Ignition 월드, smoke test launch.

## 개발 로드맵 요약
1. **기초 정의**
   - 차동구동 파라미터·센서 장착 위치 확정.
   - URDF/xacro 모델 및 센서 플러그인 작성.
2. **지도·로봇 정보 체인 완성**
   - Lanelet2 맵 로딩·시각화 검증.
   - RobotInfo 브로드캐스트 및 TF 정합.
3. **로컬라이제이션·플래닝 통합**
   - GNSS+IMU 필터 구축, Lanelet 매칭 적용.
   - 전역 경로 탐색(A*/Dijkstra) + 로컬 플래너/비헤이비어 로직 추가.
4. **인지·제어 채널 확보**
   - LiDAR 장애물 감지 → 비용맵 생성.
   - Pure Pursuit/MPC 제어, 속도/각속도 제한, Fail-safe 설계.
5. **시뮬레이션·시험·운영**
   - Gazebo/Ignition 환경 bringup, `colcon test` + `ros2 launch` 스모크 테스트.
   - 수동↔자율 모드 전환, 사람 추종 모드, 로그·모니터링 파이프라인.
6. **안전·인증**
   - 하드웨어/소프트웨어 E-Stop 이중화.
   - 센서 이중화·로그 보존, OTA 롤백 전략.

## 리포지토리 활용 방법
1. `rosdep install --from-paths src --ignore-src -y` 등으로 의존성 설치.
2. `colcon build` 후 `. install/setup.bash`를 source.
3. 예시 실행: `ros2 launch camping_cart_bringup robot_info.launch.py` 로 공용 정보 노드 확인.
4. RViz에서 `/map`, `/lanelet2map`, `/planning/global_path` 토픽을 구독해 데이터 플로우를 검증.

## 협업 및 문서화 원칙
- 기능 구현 시 `docs/autonomy_plan.md`와 README를 동시에 업데이트해 계획과 구현이 항상 일치하도록 유지합니다.
- 패키지 추가나 주요 리팩터링 전에 설계 의도를 README의 "향후 생성·확장 예정" 섹션에 기록합니다.
- 테스트·시뮬레이션 결과는 추후 `docs/` 내 별도 리포트로 축적할 예정입니다.

이 리포지토리는 캠핑 카트 자율주행 시스템을 단계적으로 완성하기 위한 설계도이자 코드 베이스입니다. 문서화된 로드맵을 따라 패키지를 확장하며, 각 단계의 성과와 교훈을 기록해 신뢰성 높은 야외 자율주행 플랫폼을 구축해 나갑니다.
