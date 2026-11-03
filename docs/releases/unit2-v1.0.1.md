<!-- HH_261103 - Publish native defaults and evidence limits separately from simulator-only compatibility work. -->
# unit2-v1.0.1

`origin/unit2/main`의 `c7d2f9c88`을 기준으로 한 순수 ROS·제어·UI 변경이다.
CARLA 실행기, 도킹 호환 제어기 사본, 시험용 충전 좌표·지도는 이 릴리스에 포함하지 않는다.

## 주요 변경

- 충전 전용 화면의 **앞으로 이동**을 확인하면 기존 지도 목표와 차체 여유 영역을 검증해 `drop_zone` 안에서 정차한다. 정차 후 기존 배달·호출 선택을 사용하며 별도 완료·강제 재도킹 절차는 없다.
- 고객 화면은 짧은 상태·안내·다음 행동 중심으로 정리했다. 운행 정지 확인, 회전·접점 접근 경고, 미션 단계·소유권 검증은 유지했다. 새 직원 승인·로그인·휴대폰 인증은 추가하지 않았으며 무단 이용자 식별을 구현한 것은 아니다.
- radar 입력 소실·오래된 값·잘못된 형식/TF가 최신 ‘장애물 없음’이 되지 않게 하고, float32 거리 경계와 저속 비영점 정지 검사를 보완했다. 지면 분리 후 취득 시각도 보존한다.
- 별도 전방 LiDAR 보조 모드와 관련 시험을 추가했다. Robot/Guest의 배터리 변환을 공유하고 호출 없는 보조 함수만 정리했다.

## 기본값과 유지한 기준

| 항목 | 이 릴리스의 기본값 |
| --- | --- |
| 필수 radar 최신성 검사 | **ON** — `radar_grid_freshness_stop_enable: true` |
| 과거 고정 반사 제외 대역 | **OFF** — `fixed_return_filter_enable: false`; 대역 기록은 보존 |
| 전방 LiDAR 보조 | **OFF** — `enable_lidar_front_guard`로 감지·최종 제어를 함께 켜는 시험용 opt-in |
| 신규 출발 배터리 | **35% 이상, 기존 기준 유지**; 바로 아래 float32 값은 허용하지 않음 |
| radar 거리·후방 정책 | 전방 센서면 0.30 m, 측면 0.10 m, 후방 비활성 유지 |

일반 복귀의 후진 주차·배터리 기준 자동 도킹 정책과 네이티브 도킹 최종 허용 오차는 완화하지 않았다.
센서면 거리, 차체 전면 거리, 실제 반사 표면 간격은 서로 다르다.

## 증거와 남은 검증

게시용 깨끗한 소스에서 control·sensing·UI·지면 분리 패키지를 새로 빌드·설치했다.
아래 검사는 모두 통과했고 건너뛴 항목은 없다. 전체 workspace·실물/ARM 시험을 뜻하지 않는다.

| 게시 전 재검사 | 결과·원본 |
| --- | --- |
| 제어 | 13묶음·237개 [XML](../evidence/unit2_v1_0_1_release/control_ctest.xml) |
| 센서 | 16묶음·112개 [XML](../evidence/unit2_v1_0_1_release/sensing_ctest.xml), 합성 입력 네 조건 92/92 일치 |
| UI Python / React | 1,057개 [XML](../evidence/unit2_v1_0_1_release/ui_python.xml) / 338개·20묶음 [JSON](../evidence/unit2_v1_0_1_release/ui_react.json) |
| Bringup | 19개 [XML](../evidence/unit2_v1_0_1_release/bringup_python.xml) |
| 설치 UI 자산 | Ranger GLB·도색 등 58개 [확인 기록](../evidence/unit2_v1_0_1_release/ui_assets.log) |

- [11월 2일 변경·시험 기록](../service_usability_20261102.md)과 [충전·Guest PNG/GIF](../evidence/service_usability_20261102/charging_ui/README.md)는 당시의 **UI fixture**다. [네 조건 비교](../evidence/service_usability_20261102/nearfield_comparison.json)는 실제 ROS 노드에 합성 센서 입력을 공급한 시험이며 실제 차량 제동 측정이 아니다. 기록된 해시·시험 수는 해당 시점의 증거다.
- 이후 [실제 CARLA 시험 자료](https://github.com/hwanhonglee/CAMROD/blob/virtual/carla/docs/evidence/service_usability_20261103/live_carla/README.md)는 `virtual/carla`에서 별도로 관리한다. 그 근접 관측도 정지 상태 센서 비교이며 **20–30 cm 보호 성능이나 실제 제동 PASS를 입증하지 않는다**. CARLA 결과를 이 순수 릴리스의 실물 검증으로 옮겨 적지 않는다.
- 실제 로봇의 센서 장착·시야·노면/경사·제동 거리, 충전기 전기적 차단·케이블/접점 분리, 어르신 대상 사용성은 미검증이다. 플랫폼에는 충전기 전원 차단 API가 없어 이동 확인이 전기적 분리 보증은 아니다.
- 기존 지도/과거 자료 검사 **4개 실패**를 보존·공개한다. 현재 지도와 과거 기대 해시/사본의 불일치이며, 통과시키기 위해 지도나 기대값을 바꾸지 않았다. 검사명·원본 결과는 [당시 전체 보고서](../service_usability_20261102.md#최종-검사-결과와-발견한-기존-불일치)에 있다. ‘전체 회귀 모두 통과’나 현장 안전 인증을 주장하지 않는다.
