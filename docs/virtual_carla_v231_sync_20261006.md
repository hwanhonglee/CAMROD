# virtual/carla — v2.3.1 공통 수정 반영

기준: `virtual/carla`의 `662ccb1df`와 순수 CAMROD `develop`/`v2.3.1`의
`e9876b123`. 기존 CARLA 연동을 유지하면서 v2.3.1 수정과 동일한 공통 기능을
반영합니다. 전체 develop 트리를 덮어쓰거나, 두 브랜치의 이력을 강제로 합치지
않았습니다. 기존 태그를 이동하지 않습니다.

## 반영 내용

| 구분 | 코드와 변경 |
| --- | --- |
| 공통 모델 배포 | `verify_frontend_assets.py`, `setup.py`, `sync_frontend_build.sh`: GLB·도색·공개 파일 58개 및 번들의 누락/불일치를 차단 |
| 공통 빌드 | `colcon_build.sh`: colcon 그래프 기반 UI 선택, 설치 검증. virtual 전용 빌드 식별자·fingerprint·외부 underlay 처리는 유지 |
| 공통 WebKit | `operator_ui_window.py`, `navigationGraphicsLease.js`, `RangerNavigationScene.js`: 기본값은 유지하고 네비게이션이 열린 동안만 WebGL 사용 |
| 공통 HTTP | `frontend_serving.py`, `ui_backend_node.py`: 누락 모델/이미지 404, GET/HEAD 지원, 정상 SPA·symlink 설치 지원 |
| CARLA 진단 | system/bringup의 CARLA 진단 YAML에 `parking/parking_dispatcher`를 `parking/runtime`, timeout 3초로 등록 |
| CARLA 회귀 검사 | 수동 미션 재개 기능이 포함된 WebSocket 처리 테스트에 실제 reducer를 사용. 주차 진단 설정 테스트에 CARLA 프로파일 포함 |
| 시험 도구 | `capture_live_basemap.py`: 미션·engage·수동 제어 중 새로고침 방지, 구역/시점/JS 오류 검사. `live_navigation_ui_action.py`: 정확한 generation의 시험 미션만 UI 확인 절차로 중지, 실패를 성공으로 기록하지 않음 |

기존 수동주행·명시적 재개 버튼·CARLA bridge/adapter는 유지됩니다. 카메라와
AprilTag 기본값, 실제 로봇 제어·복귀·도킹·배터리·GNSS 정책은 변경하지 않습니다.
새 설명은 코드 안에 `HH_261006 - ...` 영어 주석으로 남겼으며, 기존 10월 2일
시험 도구 보완 주석은 실제 작업 날짜인 `HH_261002`를 유지했습니다.

## 이번 재검증

- UI Python + build policy + 주차 진단 + 설정 mirror: **955 passed**.
- 프런트엔드: **17 suites / 307 tests passed**, production build 성공.
- `./scripts/virtual_carla/test.sh --source-only`: **150 passed**.
- `./colcon_build.sh --packages-select camrod_ui --executor sequential`: 성공.
  원본/빌드/설치의 공개 파일 **58개 일치**, runtime identity 생성 확인.
  기존 파일/심볼릭 링크가 섞인 설치에 직접 `colcon`을 실행했을 때에는
  `File exists`가 발생했으며, 정식 wrapper가 UI build/install 산출물만
  정리하고 재설치하여 해결했습니다. 소스·미션 데이터는 삭제하지 않았습니다.
- 공통 helper·모델 파일은 v2.3.1과 비교했고, backend/build wrapper는 CARLA
  전용 부분을 보존한 변경임을 확인했습니다.
- 이번 push 작업에서는 차량을 출발시키거나 새 자율주행/도킹 시나리오를
  실행하지 않았습니다. 아래 실제 연결 자료는 같은 날 앞서 수행한 정지 상태
  UI 검증 및 후방 센서 CPU 측정입니다.

## 자료

- [수정 전 진단 및 재현](evidence/runtime_portability_20261006/README.md)
- [수정 후 상세 설명과 PNG/GIF](evidence/runtime_portability_20261006/fixed/README.md)
- [실제 CARLA 연결 WebKit PNG](evidence/runtime_portability_20261006/fixed/live_webkit/navigation_webkit.png)
- [실제 CARLA 연결 시점 전환 GIF](evidence/runtime_portability_20261006/fixed/live_webkit/navigation_webkit.gif)
- [후방 AprilTag 컨테이너 CPU 결과·재현 코드](evidence/runtime_portability_20261006/rear_container_cpu/README.md)
- [순수 v2.3.1 릴리스 설명](https://github.com/hwanhonglee/CAMROD/blob/v2.3.1/docs/V2_3_1_RELEASE_NOTES.md)

CPU 45.28% → 30.33%는 x86·960×720·약 9.5Hz에서 기존 디버그 JPEG 구독을
해제한 결과입니다. 새로운 CPU 최적화나 실물 ARM 카메라 컨테이너의 200% 문제
해결을 의미하지 않습니다. 합성 UI 시연과 실제 센서 연결 자료를 구분해 표기했습니다.
완성된 PNG/GIF와 원본 측정 JSON을 보존하며 중복 GIF 프레임은 커밋하지 않습니다.

## 제외한 로컬 변경

과거의 광범위한 주석 정리, 생성된 드라이버의 이 PC 전용 경로, 예전 결과물
해시 갱신, 저장소 밖 실행 wrapper를 설명하는 로컬 정리 기록은 이번 릴리스에
포함하지 않았습니다. 삭제하거나 되돌리지 않고 작업 트리에 그대로 보존했습니다.
