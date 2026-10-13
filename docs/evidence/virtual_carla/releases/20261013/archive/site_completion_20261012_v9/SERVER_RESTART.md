# V9 bootstrap 중 GPU/Vulkan server 종료 — 실제 B10 mission 이전

<!-- HH_261012 - Separate infrastructure failure from controller trial outcomes; journal inspection only. -->

**V9의 첫 bootstrap은 실제 B10 dispatch 전에 종료됐다. Controller 주행 실패나 PASS로 집계하지 않는다.** Bootstrap receipt (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v9/bootstrap_20261012T074427316233327.json`)은 **2026-10-12 16:44:29.778756 KST**, actor **0**, `ABSENCE_VERIFIED`, `absence_verified=true`를 기록한다. 이 디렉터리에는 해당 시도의 B10 trial/supervisor 결과가 생성되지 않았다. Root가 이후 소유 bridge/pacer/UI를 정리했고, 재실행은 별도 `site_completion_20261012_v9_restart1` 증거로 분리한다.

사용 중이던 UE4Editor PID는 **3526820**이며 원래 server unit은 **`camrod-sites-20261012-v8-r1-server.service`**다. V9 client prefix와 server unit 이름이 다르므로 `v9-server.service`만 조회하면 이 실패를 놓친다.

읽기 전용 journal 확인 결과:

| 실제 KST | 관측 |
| --- | --- |
| 16:44:37.081374 | Kernel `NVRM: Xid 109`, PID 3526820, `CTX SWITCH TIMEOUT` |
| 16:44:37.594892 | UE4 Vulkan `VkResult=-4`, `VK_ERROR_DEVICE_LOST`, `VulkanMemory.cpp:4743` |
| 16:44:37.595825 | Vulkan fatal error; 다음 기록에 `Signal 11 caught` |
| 16:44:44.952647 | systemd `Main process exited, code=dumped, status=11/SEGV` |

Fatal stack은 `FFenceManager::WaitForFence` → `FVulkanCommandBufferManager::WaitForCmdBuffer` → `FVulkanDynamicRHI::RHIGetRenderQueryResult` → `FRealtimeGPUProfilerEvent::GatherQueryResults` → `FRealtimeGPUProfilerFrame::UpdateStats` → `FRealtimeGPUProfiler::EndFrame`의 render-query 경로를 포함한다. 이는 GPU context timeout 이후 rendering/profiling 경로에서 device loss가 감지된 증거다. CAMROD의 20회 recovery 후보가 실행되어 실패한 기록이 아니며, 이 stack만으로 GPU/driver의 궁극적 원인을 확정하지 않는다.

재현 가능한 읽기 명령:

```bash
journalctl -k --since '2026-10-12 16:44:25' --until '2026-10-12 16:45:20' --no-pager -o short-iso-precise
journalctl --user -u camrod-sites-20261012-v8-r1-server.service --since '2026-10-12 16:44:25' --until '2026-10-12 16:45:20' --no-pager -o short-iso-precise
```

Root는 같은 map의 새 server와 별도 fresh acceptance/gate로 재시도하며, GPU profiling을 명시적으로 끄는 좁은 runtime 옵션을 검토·적용 중이다. Profiling 우회는 이 stack 경로를 피하기 위한 infrastructure 조치이지, 물리/geometry/controller 한도를 완화하거나 driver 문제가 해결됐음을 인증하는 조치가 아니다. 실제 activation과 주행 결과는 restart1 증거에서 따로 확인해야 한다.

이 문서 작성자는 journal/receipt 읽기 및 Markdown 추가만 수행했다. Server/client 정리·restart·parameter/API/source 변경은 수행하지 않았다.
