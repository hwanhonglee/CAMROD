# B1–B13 선택 완료 사례의 최종 독립 증거 감사

<!-- HH_261012 - Audit actual finalized trials and existing retained-file hashes; preserve mixed-profile scope and all historical failures. -->

**B1–B13 각각에 대해 선택한 완료 사례 13개가 요청된 증거 검사를 모두 통과했다.** 마지막 B10의 trial·supervisor·capture가 모두 종료된 뒤 저장 파일을 독립적으로 다시 읽어 검증했다. 누락·불일치는 발견하지 않았다.

중요한 범위는 **V5 9개 + V6 2개 + V7 1개 + V9 restart1 1개**다. 이는 모든 사이트에서 실제 delivery 왕복 완료 사례를 확보했다는 뜻이며, **최신 V9 restart1 동일 profile로 13개 전체를 다시 실행했다는 뜻은 아니다.** 이전 실패·GPU/인프라 중단을 삭제하거나 성공으로 재분류하지 않았다. 각 trial의 `formal_acceptance=false`도 그대로 유지한다.

## 합계와 검증 기준

- 선택된 13개 trial의 wall time 합계: **4827.259922 s = 80분 27.259922초**.
- 저장된 physical body-XY 표본 경로 길이 합계: **2788.748913 m = 2.788749 km**, physical samples **20,811개**.
- Outbound 합계: **1859.789639 s / 788.667632 m**.
- Return 합계: **2965.147635 s / 2000.081281 m**.
- 전체 시간과 두 구간 시간 합계의 **2.322648 s** 차이는 구간 경계 정의에 따른다. 거리·시간을 UI 속도 적분이나 추정값으로 대체하지 않았다.
- 최종 localization XY와 station **(-8.47366, 40.391)**의 거리를 각 파일에서 직접 `hypot(dx,dy)`로 재계산했다. 범위 **0.206369652–0.233476055 m**, 모두 변경하지 않은 **0.25 m** 이내다.
- 13개 모두 `COMPLETED_OBSERVED`, `roundtrip_verified=true`; supervisor는 `finished`, trial/capture 종료 코드 **0/0**.
- 13개 모두 최종 reverse controller 및 dispatcher가 `PARKED`, gate가 `STANDBY`다.
- 13개 모두 collision publisher **1**, subscriber/publisher 발견 확인. `event_count`, `sampled_event_count`, `omitted_event_count`, `stale_queued_event_count` 모두 **0**다.
- 사례별 supervisor/trial/최종 physical identity/전체 저장 physical sample의 actor ID가 일치하고, 최종 physical gate accepted는 true다.
- [현재 runner](archive/site_completion_20261012_v9_restart1/run_all.py)가 재사용하는 기존 `validate_visual(folder)`를 import하여 **13개 모두 실제로 검사**했다. 실제 X11 화면, capture/GIF 5 fps, capture 전후 동일 창 geometry, 비어 있지 않은 PNG/GIF 및 manifest 해시가 확인됐다. `main()`이나 runtime API는 호출하지 않았다.
- 사례별 `visual/sha256sums.txt`에 기록된 **5개 retained file × 13 = 65개**의 SHA-256을 재계산하여 모두 일치함을 확인했다. PNG/GIF/`ffprobe.json`/`exact_commands.txt`/`capture_manifest.json`을 포함한다.

이는 선택 trial들의 합계이지 한 번의 연속 endurance 주행이 아니다. 실제 **10 km**, recall 서비스, 충전 docking, 실차 인증 또는 전체 mission-record archive 검증으로 확대하지 않는다. 캡처 원본 MP4는 정상 파생 후 제거될 수 있으며 잔존 PNG/GIF 및 manifest를 검증 대상으로 삼았다. 이 감사는 자동 수치·구조·해시 검증으로, 13개 영상을 모두 새로 육안 판독했다는 주장도 하지 않는다.

## 선택한 정확한 13개 사례

시간 s / 물리 표본 거리 m. 각 trial 링크와 같은 폴더에 `supervisor.json`, `ANALYSIS.md`, `visual/capture_manifest.json`이 있다.

| 사례 / trial | 실제 profile | actor / generation | 전체 s / m | outbound s / m | return s / m | 최종 station 거리 m |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| [B1_attempt02](B1/trial.json.gz) | v5 | 124 / 1791778802475001 | 369.236993 / 215.385603 | 170.901555 / 79.037441 | 198.184522 / 136.348162 | 0.225141205 |
| [B2](B2/trial.json.gz) | v6 | 300 / 1791783794049001 | 389.133823 / 216.251340 | 172.031062 / 78.687330 | 216.962828 / 137.564010 | 0.220897669 |
| [B3](B3/trial.json.gz) | v6 | 252 / 1791782600687001 | 397.796144 / 215.957646 | 163.111274 / 72.771374 | 234.482943 / 143.186272 | 0.227134771 |
| [B4](B4/trial.json.gz) | v5 | 28 / 1791775921124001 | 394.763985 / 216.713311 | 165.848208 / 72.112116 | 228.743867 / 144.601195 | 0.225581043 |
| [B5](B5/trial.json.gz) | v5 | 44 / 1791776563087001 | 380.745712 / 215.389545 | 154.228515 / 66.533315 | 226.313628 / 148.856230 | 0.214906527 |
| [B6_attempt02](B6/trial.json.gz) | v5 | 172 / 1791780055444001 | 367.014864 / 215.693355 | 153.548430 / 65.388667 | 213.248010 / 150.304688 | 0.225386285 |
| [B7_attempt02](B7/trial.json.gz) | v5 | 188 / 1791780536997001 | 378.606991 / 215.203380 | 146.459080 / 60.612064 | 231.890415 / 154.591317 | 0.232555458 |
| [B8_attempt02](B8/trial.json.gz) | v5 | 204 / 1791781029252001 | 373.202094 / 215.554549 | 143.311175 / 58.557997 | 229.735069 / 156.996552 | 0.231756783 |
| [B9](B9/trial.json.gz) | v7 | 316 / 1791785015292001 | 401.002762 / 215.956555 | 139.218789 / 53.720730 | 261.566569 / 162.235825 | 0.206369652 |
| [B10](B10/trial.json.gz) | v9_restart1 | 12 / 1791791602473001 | 409.820287 / 217.426499 | 137.411399 / 52.741827 | 272.260041 / 164.684671 | 0.221308763 |
| [B11_attempt02](B11/trial.json.gz) | v5 | 76 / 1791777509670001 | 323.298424 / 209.620022 | 107.750034 / 45.769330 | 215.403576 / 163.850692 | 0.221055551 |
| [B12_attempt02](B12/trial.json.gz) | v5 | 92 / 1791777942795001 | 321.669866 / 209.832739 | 106.426403 / 43.755611 | 215.081543 / 166.077128 | 0.233476055 |
| [B13_attempt02](B13/trial.json.gz) | v5 | 108 / 1791778373066001 | 320.967977 / 209.764368 | 99.543717 / 38.979830 | 221.274624 / 170.784539 | 0.227488228 |

## 마지막 B10의 종료 확인

[B10 trial](B10/trial.json.gz)의 실제 종료는 **2026-10-12 17:01:31.747149 KST**, [supervisor](B10/supervisor.json) 및 capture 후처리 종료는 **17:02:32.049438 KST**다. 이 두 종료를 확인하기 전에는 최종 감사에 B10을 포함하지 않았다.

- Actor **12**, mission generation **1791791602473001**.
- 최종 localization XY **(-8.644358455324104, 40.531853136613265)**, station 거리 **0.221308763369 m**.
- 저장된 실제 site status에서 활성 profile **center 0.015 m / trigger 0.035 m / dwell 0.400 s / recovery limit 20**을 확인했다. 관측 최대 recovery count는 **13**으로, 이전 기본 cap 10을 넘는 분기가 실제 실행됐다.
- 실제 Return acknowledgement token: `g1791791602473001-s1-e2946fa831936`; controller source에는 `robot_ui:usage_complete:...:site_exit_first`가 기록됐다.
- 완료 결과는 `COMPLETED_OBSERVED`/`roundtrip_verified=true`; 최종 reverse·dispatcher `PARKED`, gate `STANDBY`, collision counter 0이다. [Capture manifest](B10/visual/capture_manifest.json), [PNG](B10/visual/representative_contact_sheet.png), [GIF](B10/visual/representative_motion.gif)의 존재·해시도 별도로 확인했다.
- 자세한 동작·guard·journal 해석은 [B10 분석](B10/ANALYSIS.md)에 보존한다. 이 감사는 허용오차나 deadline을 변경하지 않았다.

## 실제 trial SHA-256 및 profile 구분

아래 SHA-256은 감사 시 읽은 실제 trial bytes의 fingerprint다. 과거 별도 서명과 대조한 인증이라는 뜻은 아니다. Visual 파일들은 위의 기존 hash 목록과 실제 대조했다. 앞선 12개에 대한 상세 선행 감사는 [PRIOR_SITE_AUDIT.md](archive/site_completion_20261012_v8/PRIOR_SITE_AUDIT.md)에 있다.

| 실제 profile / 사례 | trial.json SHA-256 |
| --- | --- |
| v5 B1_attempt02 | `5f2a49942d90382fbb4a8b7a33a7eaaf1060986c9939fbd13b94b54d2a649cd3` |
| v6 B2 | `366d3e896e2b2b7688d709155a8711fb64a8c25d40f878b425e957776f59e49e` |
| v6 B3 | `c76730e742a4a2914f615bf8d32048467781999f986a8f6df2118694af766b2f` |
| v5 B4 | `b8c718004148e046cb5509f4c1e7e91b3973f8cc10660998e8c025e360024ac1` |
| v5 B5 | `fd011f4d083b7a907e93b3393cb8d203c8688556fb589b30668d260292a955b4` |
| v5 B6_attempt02 | `540d8275fd7ba99f102977099d551075e50e41c18000420f157c0e9feb660a9c` |
| v5 B7_attempt02 | `41ac3e3918b27e7b0b95c2359dece3de7cb1e62337c91932fe6b42add9eb7996` |
| v5 B8_attempt02 | `3319943ad1322d2e2ea45dc3d5a7b1db90f63004426d0224b07e4e6e28c4d5c1` |
| v7 B9 | `68709142aecf0b662fc84637ef987e1fc26e63537d2f94b6b332b52f74e4c0c7` |
| v9_restart1 B10 | `8302c5c87f79f65ce25c1fe01f075d451c175f7a8989a8317947b7730e40b300` |
| v5 B11_attempt02 | `a1d008b4970676646a046609ce3af8f9f0916510d0cf84d44a40efa3fb73e1a3` |
| v5 B12_attempt02 | `4abf5781878a38066418d885d7e44b56b8d533437d0558e8ec6f5b6c788ae08c` |
| v5 B13_attempt02 | `d86dc31abbedb2176cf86238fa908f80a822417ae902012dd02fafe882d677a5` |

각 최종 physical identity가 기록한 gate fingerprint는 다음과 같이 다르다.

- V5 9개: `155733157023793752263a8201894ff55b7ccba4d45c3aa67fa6239c4009d727`.
- V6 2개: `07d38ccc60e4920340107021b0a2c97a849e6b4919c4c7d03ec7e28fcfe10c16`.
- V7 B9: `feb3de0673729a54b3272492222e345d9302071a7503f59496e9e047ac7ed26b`.
- V9 restart1 B10: `00fd65560116523478830575f50cad7ebc5ba07b693b750b58d5620cb87a795e`.

현재 source freeze 및 fresh gate의 deep verification은 실행 담당 root의 별도 검증 범위다. 이 독립 감사에서는 이를 중복 실행하거나 과거 profile이 현재 source와 동일하다고 주장하지 않았다. Source/config/runtime 및 기존 evidence는 수정하지 않았으며 **이 새 Markdown만 작성했다.**

