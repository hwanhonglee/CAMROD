# V4 준비 중 UE4 서버 중단 — GPU device loss, B4 실제 시험 전

<!-- HH_261012 - Infrastructure failure before motion is not a failed controller trial; preserve observed GPU/engine evidence without claiming the initiating hardware/driver cause. -->

V4 B4 실제 mission이 시작되기 전에 기존 UE4Editor **PID 3206930**, user unit **`camrod-sites-20261012-server.service`**가 종료됐다. 당시 `B4/trial.json`은 없었으므로 **V4 segmented-pivot controller의 물리 시험 실패로 계산하지 않는다**.

## 실제 로그 시간과 stack

아래는 **2026-10-12 KST (UTC+09:00)**이다.

| 시각 | 기록 |
| --- | --- |
| 12:15:38.607264 | kernel NVIDIA **Xid 109**, `pid=3206930`, `name=UE4Editor`, `CTX SWITCH TIMEOUT` |
| 12:15:39.310769 | UE4 Vulkan **`VkResult=-4`, `VK_ERROR_DEVICE_LOST`** |
| 12:15:39.320560 | `VulkanUtil.cpp:803` fatal error; 이어 `Signal 11 caught` |
| 12:15:42.354442 | engine crash handler가 signal 11을 다시 발생시킴 |
| 12:15:42.938870 | systemd `Main process exited, code=dumped, status=11/SEGV` |

kernel 원문:

```text
NVRM: Xid (PCI:0000:01:00): 109, pid=3206930, name=UE4Editor, channel 0x00000010, errorString CTX SWITCH TIMEOUT, Info 0xcc012
```

render-thread fatal stack의 핵심 경로:

```text
VulkanRHI::VerifyVulkanResult                 VulkanUtil.cpp:802
VulkanRHI::FFenceManager::WaitForFence        VulkanMemory.cpp:4743
FVulkanCommandBufferManager::WaitForCmdBuffer VulkanCommandBuffer.cpp:457
FVulkanDynamicRHI::RHIGetRenderQueryResult    VulkanQuery.cpp:513
FRealtimeGPUProfilerEvent::GatherQueryResults RealtimeGPUProfiler.cpp:171
FRealtimeGPUProfilerFrame::UpdateStats        RealtimeGPUProfiler.cpp:416
FRealtimeGPUProfiler::EndFrame               RealtimeGPUProfiler.cpp:839
RenderingThreadMain / FRenderingThread::Run
```

로컬 engine source의 `VulkanMemory.cpp:4743`은 `vkWaitForFences` 반환값 검증 분기다. 따라서 관측된 직접 실패 경로는 **GPU context-switch timeout → Vulkan device loss → engine fatal/SEGV**다. 이 자료만으로 최초 원인이 GPU 하드웨어, 드라이버, 장시간 engine workload, 특정 sensor 중 무엇인지 단정할 수 없다. 컨트롤러 source의 SEGV나 safety guard 실패를 보여주는 stack은 아니다.

재현 가능한 읽기 전용 로그 조회:

```bash
journalctl --user -u camrod-sites-20261012-server \
  --since '2026-10-12 12:15:25' --until '2026-10-12 12:16:05' \
  --no-pager -o short-iso-precise
journalctl -k \
  --since '2026-10-12 12:14:30' --until '2026-10-12 12:16:10' \
  --no-pager -o short-iso-precise
```

## 중단 전 정리 증거와 재시작 범위

- actor 144 cleanup (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v4/interrupted_setup_actor_144_release.json`): **03:12:29 UTC**, `ABSENCE_VERIFIED`; 기존 중단된 B5 actor 144와 부착 sensor들이 제거됨. cleanup은 주행·왕복·측정 거리가 아니다.
- V4 bootstrap (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v4/bootstrap_20261012T031528967235828.json`): **03:15:31 UTC**, `ABSENCE_VERIFIED`, actor 0, `destroyed=[]`. 이후 약 7초 뒤 kernel GPU timeout이 발생했다.
- 재개 시 같은 UMAP과 V4 controller 후보를 유지하되 **새 서버에 대한 fresh physical acceptance/gate**가 필요하다. root가 `site_completion_20261012_v4_restart1` runtime/evidence roots로 새 검증을 준비하며, 실제 site evidence는 이 **기존 V4 harness directory** 아래에 둔다. 이전 서버의 gate를 새 서버의 물리 검증으로 이월하지 않는다.

이 문서는 실제 journal·kernel·cleanup JSON의 읽기 전용 검토로 작성했다. 작성자는 engine/source/runtime 설정·GPU 상태·차량 API를 변경하지 않았다. 재시작 완료나 이후 B4 성공을 이 문서 자체로 주장하지 않는다.
