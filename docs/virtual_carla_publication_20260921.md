# virtual/carla 2026-09-21 배포 점검

## 반영 범위

기존 로컬 `virtual/carla`의 미반영 5개 커밋을 보존하고,
순수 CAMROD `develop b91981d55`까지의 실증 집계·왕복 미션 기록·UI 개선을 병합했다.
CARLA 수동주행 패널, 승인/종료 처리, 센서 연결과 wrapper는 유지했다.
UI backend 충돌은 서비스 상태의 자체 발행 메시지 식별을 유지하면서 새 기록 훅을
호출하는 방식으로 해결했으며, 병합 결과는 기존 로컬에서 사용하던 해당 파일과 일치한다.
`develop`이나 기존 태그를 변경하는 배포가 아니다.

HH_260921 추가 수정:

- CARLA full launch 및 두 파생 profile은 기록 환경을 `simulation`으로 고정한다.
- 새 미션 기록의 기본 저장소는
  `$RANGER_WORK_ROOT/camrod/mission_records`이다.
  `CAMROD_CARLA_MISSION_RECORDS_ROOT` 또는 launch의 `mission_records_root`로
  별도 시험 저장소를 명시할 수 있다.
- Ranger 환경 없이 직접 full launch를 쓰면
  `~/.local/state/camrod_carla/mission_records`를 사용한다.
- UI backend 조회와 기록 노드가 같은 경로를 공유한다.
  `XDG_STATE_HOME`, 기존 `service_metrics.sqlite3`, 실차 데이터는 변경하지 않는다.
- site-access 문서의 기본 profile을 실제 `v224_dropzone`과 맞추고
  원본 LFS 맵만으로 해당 custom-map 자산이 준비되지 않는다는 조건을 명시했다.
- 선택형 `woraksan_carla_lanelet2.osm`만 기존 생성기로 다시 생성해,
  production v27에 이미 반영된 좌표와 B12 중심선 변경을 따라잡았다.
  production `lanelet2_maps.osm`, 생성기, Unreal 지형은 변경하지 않았다.
  현재 production 대비 허용된 67개 보간 노드와 기존 노드 3개·중심선 3개만
  다르며, 경계와 모든 69개 relation 보존 검사를 통과했다.

## 검증

실행 환경과 별도 source/build/install에서 확인했다.

- UI Python: 995 통과.
- React: 61 통과, production build 성공.
  CARLA 수동제어와 새 기록 API, 기존 통계 API·SVG·표가 생성 bundle에 함께 존재한다.
- `camrod_carla_adapter`, `camrod_ui`: 두 ROS 패키지 빌드 성공.
- 기록 분리 회귀 검사: 10 통과. 실제 제품 launch를 비실행 Node capture로 검사해
  backend/recorder 경로 공유와 `simulation` label을 확인했다.
- 소스 계약 검사: 144 통과, 기존 프로필 동일성 검사 1 실패.
- 전체 adapter 검사: **714 통과, 같은 기존 프로필 검사 1 실패**(총 715).
  새 기록 분리 10개와 파생 지도 검사 10개도 이 실행에 포함된다.
  `virtual_check/adapter_source_final.xml`이 최종 결과이며,
  수정 전·중단된 검사 로그를 최종 결과로 사용하지 않는다.

파생 지도 SHA-256: `add38a6932e4c049868b00f415d788a4622a92a1cd24650f1664142b5ed5bfa8`.
변경하지 않은 production 지도 SHA-256:
`57cd044cb714f3f4c899868b5287c6c435e14395422e4f75eb66fd8eaa091fbb`.

### 기존 프로필 불일치 — 이번 작업에서 제어값을 임의로 바꾸지 않음

`test_full_launch_defaults_carla_route_heading_to_production_profile`는
pure production 설정과 CARLA wrapper가 동일해야 한다고 검사한다.
기존 `dba467cc3`부터 production은 진입 각도 105도/전방 참조 1.2m,
CARLA wrapper는 75도/2.0m로 다르다. 첫 assertion에서 실패한다.
이는 이번 UI 병합 전에도 존재한 차이이며, 통과시키려고
조향 정책 값을 바꾸거나 해당 검사를 약화하지 않았다.

이 기록은 실제 차량/시뮬레이터 주행, B1-B13 전체 왕복·도킹 성공이나
새 물리 acceptance gate 발급을 의미하지 않는다. 기존 실증/미션 DB를 열어 수정하지 않았고,
맵 지형이나 모델도 이번에 변경하지 않았다.

## 다른 장비에 전달하는 범위

소스는 Git으로 배포하지만 engine/ROS 빌드 출력, 운영 DB,
`virtual_test_results.zip` 및 미추적 지형 실험 파일은 포함하지 않는다.
기존 생성 custom map이 없는 장비는 해당 level과 참조 자산/sidecar를 별도로 준비해야 한다.
Ranger의 `config/versions.env`는 이번 virtual 배포 commit을 고정하도록 함께 갱신한다.

검증 로그: 로컬
`/home/hong/camrod_ws/_maintenance/20260921_publication/` 아래
`virtual_ui_tests.xml`, `react_tests.json`, `react_build.log`, `virtual_check/`.
현재 소스 검증을 과거 이미지나 다른 버전 gate로 대체하지 않는다.
