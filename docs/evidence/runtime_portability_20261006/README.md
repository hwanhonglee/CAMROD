# 2026-10-06 실행 점검: 모델 배포·기본 실행기·후방 카메라

> 이 문서는 수정 전 진단 기록입니다. 이후 사용자 승인으로 진행한 코드 수정과 재검증은 [수정 결과·최신 PNG/GIF](fixed/README.md)에 정리했습니다.

## 결론

**모델/도색 파일은 Git에 포함되어 있지만, 기본 WebKit 실행기와 새 3D 화면의 연결에 누락이 있습니다.**

기존 기본값은 변경하지 않았습니다. 이번 작업은 실행·읽기 전용 점검·진단 화면 구독·증거 저장입니다. 이동 목표, 수동 주행 명령, 도킹 명령을 보내지 않았습니다. 실제 월악산 로봇에는 접속하지 않았으므로 아래 로컬 결과를 그 로봇의 실행 결과로 취급하면 안 됩니다.

## 확인 환경과 버전

- 실행 PC: `htop`, x86_64, NVIDIA RTX 3060. 실제 배포 대상인 ARM64 Jetson Orin이 아닙니다.
- 독립 프런트엔드 검증: 원격 develop/v2.3.0 커밋 `0387c56682bc6c1095556547a46b4ea11063e3ab`의 파일만 `git archive`로 추출하여 새 `npm ci`와 빌드를 실행했습니다.
- 실제 연결 검증: `/home/hong/camrod_ws/src`, 로컬 `virtual/carla`. 이 작업 트리에는 기존 미커밋 변경이 있으므로 원격 커밋만의 실행과 구분합니다.
- 필수 모델·도색·로고 파일은 develop 위 커밋과 virtual/carla `662ccb1dfd6ee065f779f60924d5a4792836e7da`에서 동일합니다.
- 로컬 연결: CARLA → ROS bridge → sensor relay/CAMROD → `http://127.0.0.1:8010/`. ROS domain 188, localhost 제한. 실제 로봇 CAN 장치가 아닌 CARLA 센서 입력입니다.

## 발견한 누락

### 1. 기존 기본 WebKit 실행기는 WebGL을 명시적으로 끕니다

`camrod_ui/runtime/python/camrod_ui/operator_ui_window.py`의 `_run_webkit()`에는 `set_enable_webgl: False`가 남아 있습니다. `camrod_ui_robot/launch/ui.launch.py`와 실행기 자체의 기본 엔진도 `webkit`입니다. 과거 2D UI에서 사용하지 않던 WebGL을 꺼둔 설정입니다.

새 `RangerNavigationScene.js`는 `THREE.WebGLRenderer`를 만들며, 실패하면 `webgl-unavailable` 상태로 종료합니다. 따라서 **기본 WebKit 실행 경로에서는 GLB가 있어도 3D 화면을 만들 수 없습니다.** Chrome에서 모델이 나오는 것만 확인해서는 로봇 기본 실행기를 검증한 것이 아닙니다.

월악산에서 실제 사용한 실행기·파일 응답은 아직 확인하지 못했습니다. WebKit 기본 실행기를 사용했다면 이 문제가 직접 해당하지만, 별도 Chrome에서 접속한 경우에는 추가 확인이 필요합니다. 사용자 요청에 따라 기본 엔진과 WebGL 설정 모두 그대로 보존했습니다.

같은 `_run_webkit()`를 수정 없이 실행하여 로컬에서 재현했습니다. 모델을 정상 제공하는 현재 서버의 합성 UI 시연 경로를 열었으며, 실제 주행은 아닙니다. 관측 결과 `settings_enable_webgl=false`, `model_status=webgl-unavailable`, `webgl2_available=false`, 콘솔 오류 `THREE.WebGLRenderer: Error creating WebGL context.`였습니다. WebGL을 켜는 비교 시험도 실행하지 않았습니다.

- [기본 WebKit에서 재현한 화면](live/webkit_default_webgl_disabled.png)
- [기본 WebKit 설정·실행 관측 JSON](live/webkit_default_webgl_disabled.json)

### 2. 의존성으로 선택한 UI 패키지의 npm 빌드가 생략될 수 있습니다

`colcon_build.sh`의 `_build_scope_includes_pkg`는 선택한 이름을 검사합니다. `--packages-select camrod_ui`와 전체 빌드는 UI 준비 대상으로 잡지만, `--packages-up-to camrod_bringup`은 `camrod_ui`가 의존성으로 포함되더라도 UI 프런트엔드 준비/검증을 건너뛸 수 있습니다.

`camrod_ui/setup.py`는 당시 `frontend/build`에 있는 파일을 재귀 설치할 뿐 npm을 실행하지 않습니다. Git에 파일이 있다는 사실만으로 이전 로봇의 빌드·설치까지 최신이라는 뜻은 아닙니다. ARM만 모델을 제외하는 분기는 발견하지 않았습니다.

### 3. 파일이 없어도 HTTP 200이 나올 수 있습니다

UI 서버의 SPA fallback은 없는 경로에도 `index.html`을 반환합니다. 따라서 GLB를 확인할 때 HTTP 200만 보면 안 됩니다. MIME, 길이, 내용도 확인해야 합니다.

이번 로컬 실제 서버 응답:

| 경로 | 결과 | 크기 | MIME |
|---|---|---:|---|
| `/models/ranger-navigation.glb` | 정상 모델 | 5,576,620 B | `model/gltf-binary` |
| `/models/woraksan-rear-wrap.png` | 정상 이미지 | 1,084,559 B | `image/png` |

모델 GLB의 시작 4바이트는 `glTF`이며 SHA-256은 `da468915fa41de59955c0d297ccfadb0002248cac8f7f60414a7e731cfa8d202`입니다. 경로는 UI 서버와 같은 출처의 `/models/...`이며 개발 PC의 다른 폴더나 외부 모델 서버를 참조하지 않습니다. Blender/Unreal은 이 웹 UI 모델 표시의 실행 의존성이 아닙니다.

### 4. 기타

- 진단 집계 설정에 `parking/parking_dispatcher` 항목이 없어 unknown 그룹으로 분류되는 경고가 있습니다. 프로세스 미실행과는 다릅니다.
- 독립 정적 프런트엔드에서 `/favicon.ico`가 없습니다. 브라우저 탭 아이콘 문제이며 모델 누락 원인은 아닙니다.
- 첫 bridge 시작을 CARLA 준비 전에 시도하여 연결 시간 초과가 났고, bridge 종료 처리에서 `shutdown` 속성 오류도 발생했습니다. 서버 준비 후 다시 시작한 bridge는 연결됐습니다. 이 시작 순서 오류를 모델/패키지 누락으로 분류하지 않았습니다.

## 실제 실행 결과와 자료

- 독립 프런트엔드: `npm ci`, production build 성공. 공개 정적 파일 58개의 원본/빌드/HTTP 내용 일치. 기존 홈 이미지 및 모델, 앞·뒤·좌·우 래핑 확인.
- 로컬 CARLA 전체 launch: 시작한 61개 프로세스 생존 확인. 관련 최상위 CAMROD 패키지 15개 설치 확인. 초기 local costmap 진단 오류는 다음 주기에 해소되어 `SYSTEM OK`, UI `ready` 확인.
- 실제 연결 UI: 기본 지도, 초록 경계선, 14개 구역(B1~B13 및 드롭존), 중앙 로봇, 적재 박스 및 4면 도색 표시 확인. 실제 수신 pose를 사용했습니다.
- 이번 검사는 정지 상태의 연결·표시 검사입니다. 배송/호출/복귀/도킹 성공 또는 전 구역 주행 성공을 의미하지 않습니다. idle 상태이므로 미션 경로는 0개가 정상입니다. 도로변 나무 등은 명시적으로 표시된 예시 배경이며 실측 장애물이 아닙니다.

자료:

- [원격 커밋만으로 새 빌드한 상세 검증](committed_ui/README.md)
- [원격 커밋 UI PNG](committed_ui/light.png), [기존 홈 PNG](committed_ui/existing_home.png)
- [합성 데이터 UI 시연 GIF — 실제 주행 아님](committed_ui/navigation_preview.gif)
- [CARLA 실연결 기본 지도 PNG](live/navigation/idle_filled_roads_surroundings.png)
- [CARLA 실연결 외관 PNG](live/navigation/idle_filled_roads_exterior.png)
- [CARLA 실연결 시점 전환 GIF — 이동 아님](live/navigation/idle_filled_roads_camera_views.gif)
- [지도·모델·구역 수신 검증 JSON](live/navigation/idle_filled_roads_capture.json)

## 후방 카메라 / AprilTag CPU

도킹 진단 구독 닫힘 → 열림 → 닫힘을 실제 CARLA 영상으로 측정했습니다. CPU 100%는 논리 코어 하나입니다.

| 프로세스 | 닫힘 15초 | 열림 20초 | 다시 닫힘 15초 |
|---|---:|---:|---:|
| AprilTag | 30.96% | 41.40% | 30.82% |
| CARLA sensor relay | 32.02% | 32.13% | 32.69% |
| UI backend | 27.36% | 29.83% | 27.09% |

입력은 960×720 BGRA8, 관측 5.38 Hz입니다. CARLA용 실제 detector 설정은 `quad_decimate=1`, `n_threads=2`, debug JPEG 품질 80입니다. 태그 검출 성공 시험이 아니며, 표본에서 검출은 false였습니다.

도킹 화면 구독 시 detector의 프레임별 debug JPEG 인코딩이 활성화됩니다. 해당 출력에는 별도의 Hz 제한이 없으며, 후방 카메라의 모니터 JPEG 제한과 다른 경로입니다. 이번 단기 A/B/A에서는 detector가 약 10.52%p 증가했다가 원복했습니다.

기존 카메라 JPEG 구독/주기 제한, 비동기 인코딩, 동일 컨테이너 intra-process 및 rectification 관련 최적화 코드는 v2.2.9에서 삭제되지 않았습니다. 다만 **Jetson의 물리 카메라+VIC+통합 컨테이너 CPU 200% 현상이나 100% 미만 달성을 이 PC 결과로 확인할 수는 없습니다.** 카메라·검출 파라미터는 변경하지 않았습니다.

- [측정 요약](live/rear_cpu_probe_summary.md)
- [원본 측정값·실제 파라미터](live/rear_cpu_probe_results.json)
- [재현 가능한 읽기 전용 측정 도구](live/rear_cpu_probe.py)

## 점검 종료

측정 후 진단 구독을 닫고 이번 점검에서 만든 `camrod-check-*-20261006` 임시 실행 단위만 종료했습니다. CAMROD, spawn, bridge를 먼저 종료하고 마지막에 pacer/CARLA를 종료했습니다. 기본 실행 설정, 카메라 설정 및 소스 알고리즘은 바꾸지 않았고 commit/push도 하지 않았습니다. 재현 자료는 이 폴더에 보존합니다.
