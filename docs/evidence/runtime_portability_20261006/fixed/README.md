# 2026-10-06 수정 및 재검증 결과

앞선 [진단 보고서](../README.md)의 UI/배포 누락을 로컬 `virtual/carla` 작업 트리에서 수정했습니다. 아래는 수정 직후 검증 기록이며, 당시에는 커밋·push 전이었습니다. 이후 순수 CAMROD 변경은 `develop`/`v2.3.1`에 배포했고, CARLA 쪽 반영 범위와 재검증은 [동기화 기록](../../../virtual_carla_v231_sync_20261006.md)에 정리했습니다. 월악산 실제 로봇에는 배포하거나 접속하지 않았습니다.

## 1. 무엇을 어떻게 수정했는가

| 문제 | 수정 코드 | 동작 |
|---|---|---|
| WebKit 기본 실행기에서 3D 표시 불가 | `operator_ui_window.py`, `navigationGraphicsLease.js`, `RangerNavigationScene.js` | 기본 엔진 WebKit, 기본 WebGL-off 유지. 네비게이션을 연 동안만 native 메시지로 그래픽 사용을 요청하고, 허용 응답 후 renderer 생성. 홈 복귀·페이지 새로고침·종료에서 권한 해제. 문서별 ID로 이전 페이지 응답과 충돌 방지. |
| 의존성 선택 빌드에서 npm 준비 누락 가능 | `colcon_build.sh` | colcon이 해석한 실제 패키지 선택 결과를 사용. 의존성·정규식 선택 지원. 그래프 조회 실패를 UI 제외로 오인하지 않고 오류 처리. |
| 모델/이미지 없이도 설치 또는 게시 가능 | `verify_frontend_assets.py`, `sync_frontend_build.sh`, `setup.py` | 모델·4면 도색에 쓰이는 PNG와 공개 정적 파일 58개, 번들 참조를 검증. 원본/빌드/설치 바이트 비교. 불완전한 빌드는 기존 화면을 덮기 전에 차단. metadata 조회만 하는 colcon list는 빌드 전에도 허용. |
| 없는 GLB 경로도 HTML 200 반환 | `frontend_serving.py`, `ui_backend_node.py` | 누락된 모델·이미지·JS는 404. 정상 SPA 화면 경로는 기존대로 200. GET/HEAD 검사 지원. 빈 frontend_dir가 현재 작업 폴더가 되는 문제 수정. 정상 colcon symlink 설치 지원. |
| parking dispatcher가 unknown 진단 그룹으로 분류 | CARLA diagnostics YAML 2개 | 기존 `parking/runtime`, timeout 3.0 항목을 CARLA 설정에도 등록. 기본/실제 로봇 및 sim 설정에는 이미 있어 수정하지 않음. |
| 브라우저 탭 아이콘 누락 | frontend `public/index.html` | 저장소에 있던 월악산 로고를 favicon으로 지정. 새 이미지 의존성 없음. |

실행 코드의 새 설명은 `HH_261006 - ...` 형식의 영어 주석으로 남겼습니다.

### 그대로 유지한 것

- WebKit 기본 엔진, 기본 WebGL-off, 전체 화면·스크롤 등 기존 키오스크 기본 설정.
- 카메라 해상도/FPS/JPEG 주기, AprilTag 검출 파라미터, 기존 최적화.
- 주행·정지·복귀·도킹·배터리 정책, GNSS/TF 및 제어 알고리즘.
- 새 그래픽 메시지는 화면 렌더링만 다룹니다. 로봇 명령·네트워크 설정·파일 조작 권한이 없습니다.

WebGL을 완전히 금지한 채 현재 Three.js 3D 화면을 그릴 수는 없습니다. 따라서 전역 기본값을 뒤집는 대신 **새 네비게이션 화면이 열린 동안만 필요한 기능을 허용**했습니다. 기본 화면으로 복귀하면 다시 꺼집니다.

## 2. 검증 결과

| 검사 | 결과 |
|---|---|
| 전체 프런트엔드 Jest | 17 suites, **307 tests passed** |
| UI Python + build policy + parking registry + YAML mirrors | **955 tests passed** |
| npm production build | 성공: `main.6a8c195c.js`, `main.a6df9e9c.css` |
| `colcon build --packages-select camrod_ui --symlink-install --executor sequential` | 성공, 1 package |
| 원본/빌드/설치 모델·이미지 | **58개 바이트 일치** |
| 실제 WebKit 합성 데이터 UI | 3D·래핑·홈 복귀·재진입·새 문서 reload 확인 |
| 실제 CARLA 센서에 연결한 WebKit | 기본 지도, 14개 구역, 로봇·도색 표시, 홈 복귀·재진입·reload 확인 |
| 실행 중 실제 서버 `/models/ranger-navigation.glb` HEAD | 200, `model/gltf-binary`, 5,576,620 bytes |
| 실행 중 실제 서버 `/models/does-not-exist.glb` HEAD | **404**, HTML로 위장하지 않음 |
| 실행 중 진단 | SYSTEM OK / UI ready. dispatcher unknown 경고 재발 없음 |

회귀 검사 중 발견된 과거 테스트의 코드 추출 문제도 수정했습니다. WebSocket fixture가 실제 `ManualMissionResume.js` helper를 포함하도록 하고, 캐시 검사는 파일 내 문자열 대신 실제 HTTP 동작을 검사합니다. 이를 위해 앱의 서비스/주행 알고리즘을 바꾸지 않았습니다.

WebKit 실제 관측 이벤트는 **false → true → false → true → false → true → false**입니다. 네비게이션 중 새로고침 시험은 이전 문서가 아니라 새 문서의 load-finished 후 모델이 표시되는지 확인했습니다.

캡처 검사도 보완했습니다. GLB 로딩 직후 상태값은 ready여도 GTK 합성 화면에 아직 첫 프레임이 표시되기 전일 수 있어, 최소 60개 렌더 프레임이 진행된 뒤 PNG를 저장합니다. 실제 GIF의 후속 프레임에는 이미 로봇이 표시됐으며, 최종 PNG도 직접 열어 확인했습니다.

## 3. 시각자료

### 실제 CARLA 연결, 정지 상태

- [WebKit 네비게이션 PNG](live_webkit/navigation_webkit.png)
- [기존 홈으로 복귀한 PNG](live_webkit/home_webkit.png)
- [새로고침 후 다시 표시된 PNG](live_webkit/navigation_reloaded_webkit.png)
- [외관 보기 전환 GIF](live_webkit/navigation_webkit.gif)
- [실제 수신 지도·모델 상태와 설정 전환 JSON](live_webkit/webkit_lifecycle.json)

이는 **실제 CARLA 연결의 정지 상태 UI 시험**입니다. 이동 목표나 도킹 명령을 보내지 않았으며, 배송/호출/복귀 주행 성공 증거가 아닙니다. 경로가 없는 대기 상태에서 기본 지도는 표시됩니다.

### 합성 데이터로 재현 가능한 UI 시험

- [합성 시연 PNG](webkit/navigation_webkit.png)
- [합성 시연 GIF](webkit/navigation_webkit.gif)
- [합성 시험 JSON](webkit/webkit_lifecycle.json)

합성 시연은 화면에 별도로 표기됩니다. 실제 센서 검출이나 실제 주행으로 해석하면 안 됩니다.

## 4. 배포와 재현

이 변경이 전달된 작업 트리에서 UI를 빌드하고 기존 실행 명령을 사용합니다. 기존 WebKit 설정을 Chromium으로 바꿀 필요가 없습니다.

```bash
cd /home/hong/camrod_ws/src
./colcon_build.sh --packages-select camrod_ui
source /home/hong/camrod_ws/install/setup.bash
# 이후 사용하던 기존 CAMROD launch 명령 실행
```

다른 PC는 실제 workspace 경로로 바꿉니다. 위 명령은 **이 변경이 포함된 코드에서** 실행해야 합니다. 아직 push하지 않았으므로 현재 원격을 받는 것만으로 이번 수정이 적용되지는 않습니다. 모델이나 도색을 다른 폴더에서 따로 복사할 필요는 없습니다.

검증 도구:

```bash
python3 camrod_ui/scripts/verify_frontend_assets.py \
  --frontend camrod_ui/camrod_ui_robot/assets/frontend \
  --build camrod_ui/camrod_ui_robot/assets/frontend/build \
  --installed ../install/camrod_ui/share/camrod_ui/camrod_ui_robot/assets/frontend/build
```

`camrod_ui/tools/check_webkit_navigation.py`는 기본 WebKit 실행 함수를 직접 검사합니다. 합성 시연 또는 `--live-idle`만 허용하고, 연결 시험에서는 활성 미션이 있으면 중단합니다.

## 5. 검증 범위와 남은 확인

- 이 PC는 x86_64/RTX 3060이며 실제 ARM64 Jetson의 GPU 드라이버/WebKit 버전까지 검증한 것은 아닙니다.
- 월악산 로봇의 200% CPU 현상은 아직 재현·원인 확정하지 못했습니다. 이전 [CARLA 후방 CPU 측정](../live/rear_cpu_probe_summary.md)을 Jetson 결과로 대체하지 않습니다. 이 문제를 숨기기 위해 해상도·검출 주기를 낮추지 않았습니다.
- 첫 진단 때의 bridge 초기 오류는 CARLA가 준비되기 전 연결한 상황이었습니다. 재시험은 `get_world()` 성공 확인 후 bridge를 시작해 정상 연결했습니다. 외부 ROS bridge의 초기화 실패 시 정리 코드 자체를 변경한 것은 아닙니다.
- 일반 CAMROD에 필요한 변경은 UI·패키징·테스트이고, CARLA 전용 진단 YAML은 virtual/carla에만 속합니다. commit 분리 시 이 구분을 유지해야 합니다.

검증 종료 후 이번에 시작한 임시 `camrod-fixed-*` 및 UI preview 서비스는 종료했습니다. 기존 개인 브라우저나 다른 작업 프로세스를 일괄 종료하지 않았습니다.
