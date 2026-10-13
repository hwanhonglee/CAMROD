# 워크스페이스 경로 정리 — 2026-09-30

<!-- HH_261013 - Retain the historical workspace note referenced by the v2.2.9 report; publication performs no new deletion or host reconfiguration. -->
이 문서는 당시 경로 정리 기록이다. 현재 시험 설정·자료·외부 의존성의 기준은
[2026-10-13 virtual/carla 안내](virtual_carla_release_20261013.md)를 따른다.
이번 문서 공개에서는 아래 삭제·이동을 새로 실행하지 않았다.

## 2026-10-01 최종 보관함 삭제

사용자의 보관함 전체 삭제 요청에 따라 `/home/hong/camrod_archive` 약 9.7 GiB를
영구 삭제했다. 대상은 아래 과거 기록에 나오는 `past_work`, `source_backups`,
`git_recovery`, `removed_dependency_sources`, `retained_source_documents`와
보관함 README다. 이제 해당 경로와 과거 `_maintenance` 자료는 존재하지 않는다.

현재 CAMROD/Ranger 소스, 실제 Git 저장소·브랜치·태그, build/install,
config/scripts/state, src/docs에 정리된 최신 PNG/GIF는 삭제하지 않았다.
현재 worktree·실행 설정·Git hooks·객체 alternates·설치 심볼릭 링크가
보관함에 의존하지 않는 것을 확인한 뒤 삭제했다.
실차 및 시뮬레이션 DB SHA256은 아래 기록과 동일하게 유지됐다.

휴지통을 거치지 않았으므로 Git에 들어 있지 않았던 과거 원시 로그·복구 패치는
이 보관함에서 복원할 수 없다. 모든 삭제 파일이 원격 태그에 있다는 검증은
하지 않았으며, 사용자가 이전 자료 보존을 더 이상 요구하지 않아 삭제한 것이다.
아래 내용은 정리 과정의 역사 기록이다.

## 추가 정리 완료 상태

아래 초기 조사 이후, 중복을 검증하고 작업 사본 4개 및 임시 빌드/캐시를 삭제했다.
약 5.5 GiB를 확보했다(파일시스템 여유 공간 변화 기준, 다른 프로세스 영향 가능).
`develop` 브랜치는 그대로 존재하지만 별도 `.codex-develop-20260904` 작업 폴더는
제거했다. CAMROD에는 `src` worktree만, Ranger에는 기존 main worktree만 남았다.

- 구 `_maintenance` → `~/camrod_archive/past_work`
- 구 `_sync_backups` → `~/camrod_archive/source_backups`
- 구 `_history_safety` → `~/camrod_archive/git_recovery`
- 구 `submodule_removed_backup` → `~/camrod_archive/removed_dependency_sources`
- workspace 루트의 과거 TF 도표 4개 → `~/camrod_archive/past_work/tf_diagrams`

과거 미션 작업 사본의 수정/추가 소스는 v2.2.8과 blob 단위로 같았다.
Ranger 사본 수정은 현재 main에 있고, 달랐던 문서 3개는 별도 보관했다.
실제 src의 node_modules, build/install, config/scripts/state, 실차 DB는 유지했다.
구 날짜별 runtime.env 경로도 보관함으로 이동했으므로 새 scripts/carla.sh만 사용한다.
전체 보관함을 중복으로 판정해 삭제한 것은 아니다. 미확인 복구 자료는 남아 있다.
세부 보관·삭제 목록: `~/camrod_archive/README.md`.

## 최초 분리 시 조사 기록 (아래 옛 위치는 현재 위치가 아님)

날짜별 임시 작업 폴더에 실행 환경이 종속되지 않도록 현재 PC 설정을 분리했다.
CAMROD 알고리즘, 맵, 토크, 센서 및 안전 파라미터는 이 정리에서 변경하지 않았다.

| 경로 (`~/camrod_ws/` 기준) | 역할 |
|---|---|
| `src/` | 현재 `virtual/carla` 소스 저장소 |
| `scripts/carla.sh` | 현재 PC의 실행 진입점. 어느 디렉터리에서나 실행 가능 |
| `config/virtual_carla.env` | 설치 경로·맵·ROS domain·UI·시뮬레이션 저장 경로 |
| `state/virtual_carla/` | 기존 시뮬레이션 SQLite 및 향후 스냅샷 저장 영역 |
| `log/virtual_carla/` | 기존 시험 ROS 로그 및 향후 ROS 로그 |
| `build/`, `install/` | 현재 빌드와 실행 설치 결과. 이번 정리에서 유지 |
| `src/docs/evidence/virtual_carla/v2_2_9_20260930/` | 정리된 실제 시험 PNG/GIF/결과 |

`scripts/carla.sh`는 설정을 자식 프로세스에만 적용한 후 기존
`src/scripts/virtual_carla/run.sh`를 실행한다. 인자가 없으면 명령 안내만 출력한다.
시뮬레이터를 자동으로 시작하지 않는다. 설정과 진입점은 로컬 파일이며
`src` Git 저장소에 포함되지 않는다. 타 PC에는 설치 경로를 맞춰야 한다.

## 폴더별 판단

| 폴더 | 확인 크기 | 판단 |
|---|---:|---|
| `.pytest_cache` | 40 KiB | 재생성 가능한 테스트 캐시. 실행에는 불필요 |
| `.colcon` | 8 KiB | 빌드 설정. `.bashrc`의 `COLCON_DEFAULTS_FILE`이 실제 참조하므로 무조건 삭제하면 안 됨. 내부에는 옛 `camping_cart_*` 경로가 남아 있어 별도 갱신 대상 |
| `.codex-develop-20260904` | 1.3 GiB | `develop`의 등록된 Git worktree. CARLA 실행에는 불필요하지만 단순 캐시가 아님 |
| `.camrod-build` | 1.6 MiB | 이전 별도 UI 빌드 결과. 현재 `build/install`과 다른 위치 |
| `_sync_backups` | 758 MiB | 과거 소스 사본·검증 로그·이미지가 혼재. 고유 변경/증거 확인 전 일괄 삭제 보류 |
| `submodule_removed_backup` | 760 KiB | 제거 당시 DBoW2 C++ 소스 백업. 단순 로그가 아님 |
| `_maintenance` | 약 4.8 GiB | 여러 날짜의 검증 작업·Git worktree·자료. 현재 실행 설정/상태/ROS 로그만 새 경로로 분리 |
| `_history_safety` | 8.4 GiB | Git 이력 정리 전 bare 저장소·staged/unstaged patch·변경 파일 tar. 복구 자료이므로 단순 로그로 삭제하면 안 됨 |

현재 등록된 CAMROD worktree에는 `src`, `.codex-develop-20260904`,
`_maintenance/20260915_mission_recorder/release_source`,
`_maintenance/20260921_publication/virtual_release`가 있다.
삭제/이동 시 Git worktree 상태와 미반영 변경을 확인해야 한다.

## 보존 및 확인

- 구 `runtime.env`는 새 설정을 source하는 호환 파일로 남겼다. 새 실행에는 필요 없다.
- 당시 `state`와 `ros_logs` 디렉터리는 복제하지 않고 각각 새 위치로 이동했다.
- 다른 과거 실험 폴더와 백업은 삭제하지 않았다. 전체 정리가 완료됐다는 뜻은 아니다.
- 실차 데이터 `~/.local/state/camrod/service_metrics.sqlite3`는 그대로 보존했다.
- 실차 DB SHA256: `43e292886f786f0c359675b88b8219af150ac24d8a4a171c205db48db5c46261`.
- 이동한 시뮬레이션 DB SHA256: `7748c352487e1d8a898ffea0c8d8b6dac0009f678e0647f32ef2b50c17e00466`.
- 승인 manifest는 기존에 검증된 `camrod_kimhakjoo_ws/evidence/ranger_gate/`의 파일을 유지했다.
  이 외부 의존성은 아직 있으므로 해당 폴더를 먼저 삭제하면 안 된다.

경로 변경 후 검증: 셸 문법 검사, 구 runtime.env 호환 검사, `/tmp`에서 명령 안내
실행, 새 진입점 `doctor` 모두 통과했다. doctor에서 baseline/physical 4WS gate,
ROS 패키지, Python physical 4WD API, TensorRT 엔진을 확인했다. `vulkaninfo`
미설치로 선택적 Vulkan 요약 검사는 생략됐다. 서버/UI 포트 2000/8010/8012/9224는
닫힌 상태를 유지했다. 시뮬레이터 재기동 또는 주행 재시험은 이번 정리에서 하지 않았다.

실행 순서/명령은 [v2.2.9 검증 보고서](virtual_carla_v2_2_9_validation_20260930.md#6-이-pc에서-다시-실행)에 있다.
