"""Source-level regression checks for critical Robot UI operator flows."""

from pathlib import Path
import ast
import json
import re
import shutil
import subprocess
import unittest


APP_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "camrod_ui_robot"
    / "assets"
    / "frontend"
    / "src"
    / "App.js"
)
APP_CSS = APP_SOURCE.with_name("App.css")
TELEMETRY_SOURCE = APP_SOURCE.with_name("TelemetryWorkspace.js")
MANUAL_DRIVE_SOURCE = APP_SOURCE.with_name("ManualDrivePanel.js")
SERVICE_EVIDENCE_SOURCE = APP_SOURCE.with_name("ServiceEvidence.js")
PUBLIC_ASSETS = APP_SOURCE.parents[1] / "public"
UI_BACKEND_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "runtime"
    / "python"
    / "camrod_ui"
    / "ui_backend_node.py"
)


class RobotUiFrontendContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = APP_SOURCE.read_text(encoding="utf-8")
        cls.css = APP_CSS.read_text(encoding="utf-8")
        cls.telemetry_source = TELEMETRY_SOURCE.read_text(encoding="utf-8")
        cls.manual_drive_source = MANUAL_DRIVE_SOURCE.read_text(
            encoding="utf-8"
        )
        cls.service_evidence_source = SERVICE_EVIDENCE_SOURCE.read_text(
            encoding="utf-8"
        )
        cls.backend_source = UI_BACKEND_SOURCE.read_text(encoding="utf-8")

    def test_site_verification_owns_virtual_keyboard_input(self) -> None:
        self.assertIn(": setMoveVerifyInput;", self.source)
        self.assertIn("activeField === 'moveVerify' || kbCaps", self.source)
        self.assertIn("setActiveField('moveVerify');", self.source)
        self.assertIn(
            'className="vkb-wrap move-verify-keyboard"',
            self.source,
        )

    def test_site_verification_normalizes_physical_keyboard_input(self) -> None:
        self.assertIn(
            "setMoveVerifyInput(e.target.value.toUpperCase())",
            self.source,
        )
        self.assertIn(
            "onFocus={() => setActiveField('moveVerify')}",
            self.source,
        )

    def test_return_status_exits_idle_screen(self) -> None:
        self.assertIn("if (isReturning && showWaiting)", self.source)
        self.assertIn("setShowWaiting(false);", self.source)

    def test_return_in_progress_exposes_authoritative_operator_stop(self) -> None:
        handler_start = self.source.index("const handleStopMove = () => {")
        handler = self.source[
            handler_start : self.source.index(
                "// ── 이용 완료 버튼", handler_start
            )
        ]
        self.assertIn("fetch('/ui/stop', { method: 'POST' })", handler)
        self.assertNotIn("if (activeSite)", handler)

        returning_preview = self.source[
            self.source.index(") : displayedReturning ? (") :
            self.source.index(") : activeSite ? (")
        ]
        self.assertIn("대기·충전 장소로 복귀 중입니다.", returning_preview)
        self.assertIn("필요하면 아래 버튼으로 운행을 중지할 수 있습니다.", returning_preview)
        self.assertIn(">운행 중지</button>", returning_preview)
        self.assertIn("onClick={handleStopMove}", returning_preview)
        self.assertIn("serviceStateName !== 'WAITING_FOR_CHARGING'", returning_preview)
        self.assertIn(
            "주차를 마치고 충전기 연결을 기다리고 있습니다.",
            returning_preview,
        )

        returning_states = self.source[
            self.source.index("const RETURNING_STATES = new Set([") :
            self.source.index("const MOVING_SERVICE_STATES")
        ]
        self.assertIn("SERVICE_STATE.DROP_ZONE_PARKING", returning_states)
        self.assertIn("SERVICE_STATE.WAITING_FOR_CHARGING", returning_states)

    def test_site_keypad_layout_is_bounded_for_windowed_operation(self) -> None:
        self.assertIn("max-height: 94vh", self.css)
        self.assertIn(".move-verify-keyboard .vkb-key", self.css)
        self.assertIn(".move-verify-keyboard .vkb-space", self.css)

    def test_service_motion_copy_defers_to_stop_or_error_without_claiming_false_motion(self) -> None:
        start = self.source.index("function serviceMotionNotice(")
        helper = self.source[start:self.source.index("function robotCanCompleteMission(", start)]
        cases = [
            ["SAFETY_STOP", "ERROR"], ["SAFETY_STOP", "OK"],
            ["STOPPED", "OK"], ["DRIVING", "ERROR"], ["ERROR", "OK"],
            ["DRIVING", "OK"], ["DRIVING", "WARNING"], ["ARRIVED", "OK"],
        ]
        result = subprocess.run(
            ["node"], input=helper + "\nprocess.stdout.write(JSON.stringify(" + json.dumps(cases)
            + ".map(([phase,health])=>serviceMotionNotice(phase,health))));",
            text=True, capture_output=True, check=True,
        )
        notices = json.loads(result.stdout)
        self.assertEqual(notices[0], notices[1])
        self.assertEqual(notices[0]["label"], "안전 정지")
        self.assertIn("일시 정지", notices[0]["message"])
        self.assertEqual(notices[2]["message"], "운행이 정지되었습니다.")
        self.assertEqual(notices[3], notices[4])
        self.assertEqual(notices[3]["label"], "시스템 오류")
        self.assertNotIn("정지", notices[3]["message"])
        for notice in notices[:5]:
            self.assertNotIn("이동 중", notice["message"])
            self.assertNotIn("복귀 중", notice["message"])
        self.assertEqual(notices[5:], [None, None, None])

    def test_motion_notice_only_replaces_motion_copy_and_preserves_arrival_actions(self) -> None:
        self.assertIn("const motionNotice = serviceMotionNotice(missionPhase, systemHealth);", self.source)
        returning = self.source[self.source.index(") : displayedReturning ? ("):
                                self.source.index(") : activeSite ? (")]
        self.assertIn("motionNotice?.label || recallProgress.label", returning)
        self.assertIn("motionNotice?.message || (recallReturnPresentation", returning)
        self.assertIn("onClick={handleStopMove}", returning)
        for first, last in ((") : activeSite ? (", ") : activeRecallSite ? ("),
                            (") : activeRecallSite ? (", ") : manualDriveActive ? ("),
                            (") : manualDriveActive ? (", ") : serviceStateName === 'OPERATOR_STOPPED'")):
            block = self.source[self.source.index(first):self.source.index(last)]
            self.assertIn("motionNotice?.message ||", block)
        self.assertEqual(self.source.count("{guestAdmissionStatus}"), 2)
        self.assertNotIn('className="guest-recall-overlay"', self.source)
        arrival = self.source[self.source.index(") : arrivedSite ? ("):
                              self.source.index(") : displayedReturning ? (")]
        self.assertNotIn("motionNotice", arrival)
        self.assertIn("recallReturnInstructions(arrivedSite, recallFinalReturnReady)", arrival)
        self.assertIn("recallCompletionLabel(arrivedSite, recallFinalReturnReady)", arrival)
        self.assertIn("onClick={handleArrivalComplete}", arrival)

    def test_imported_png_assets_are_present_and_referenced(self) -> None:
        for filename in ("information_nobg.png", "hiking_trail_nobg.png"):
            asset = PUBLIC_ASSETS / filename
            self.assertTrue(asset.is_file())
            self.assertEqual(asset.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
            self.assertIn(f"/{filename}", self.source)

    def test_operator_telemetry_tabs_cover_rviz_runtime_surfaces(self) -> None:
        for label in (
            "GNSS · IMU", "레이더 · LiDAR", "카메라", "주행 궤적",
            "지도 · 인지", "안전 · 제어", "도킹 · 주차",
        ):
            self.assertIn(label, self.telemetry_source)
        self.assertIn("TelemetryWorkspace", self.source)
        self.assertIn("diag-tab-bar", self.css)

    def test_raw_lidar_overlay_preserves_develop_default(self) -> None:
        """Raw LiDAR retains develop's always-visible telemetry behavior."""
        for token in (
            "const raw = telemetry.lidar?.streams?.raw?.points || []",
            '<span><i className="legend-lidar-raw" />LiDAR raw</span>',
        ):
            self.assertIn(token, self.telemetry_source)
        self.assertNotIn("showRawLidar", self.telemetry_source)
        self.assertNotIn(".lidar-raw-toggle", self.css)

    def test_perception_view_distinguishes_semantic_and_raw_visual_layers(self) -> None:
        for token in (
            "raw_lidar_bbox_overlay_enabled: true",
            "telemetry.options?.raw_lidar_bbox_overlay_enabled !== false",
            "Semantic fusion cost",
            "Semantic obstacle cloud",
            "Raw LiDAR bbox · visual only",
            "Raw BBox outlines · visual only",
        ):
            self.assertIn(token, self.telemetry_source)

    def test_operator_telemetry_lease_is_closed_on_unmount(self) -> None:
        self.assertIn("/ws/telemetry?view=${view}", self.telemetry_source)
        self.assertIn("/api/telemetry/session?active=true", self.telemetry_source)
        self.assertIn("/api/telemetry/session?active=false", self.telemetry_source)
        self.assertIn("view=${view}", self.telemetry_source)
        self.assertIn("keepalive: true", self.telemetry_source)
        self.assertIn("currentSocket.send('lease')", self.telemetry_source)
        self.assertIn("}, 4000);", self.telemetry_source)

    def test_carla_manual_drive_is_hold_to_move_and_fail_closed(self) -> None:
        self.assertEqual(self.telemetry_source.count("<ManualDrivePanel />"), 1)
        camera_view = self.telemetry_source.split(
            "function CameraView", 1
        )[1].split("function pointList", 1)[0]
        self.assertNotIn("<ManualDrivePanel />", camera_view)
        self.assertRegex(
            self.telemetry_source,
            r"\{view\}\s*<ManualDrivePanel />",
        )
        for token in (
            "/ws/manual-drive",
            "{ type, seq: sequenceRef.current, ...payload }",
            "setInterval(sendDrive, 100)",
            "window.addEventListener('keyup', keyUp)",
            "window.addEventListener('blur', stopForBlur)",
            "window.addEventListener('pagehide', disarmForPageLifecycle)",
            "document.addEventListener('visibilitychange', visibility)",
            "socket.send(JSON.stringify({ type: 'disarm'",
            "event.code === 4403",
            "event.code === 4409",
            "if (send('arm'))",
            "send('disarm')",
            "requestedLinearMps",
            'step="0.05"',
        ):
            self.assertIn(token, self.manual_drive_source)

    def test_default_disabled_manual_drive_does_not_capture_global_input(self) -> None:
        self.assertIn(
            "const manualEnabled = manual.available || manual.connected;",
            self.manual_drive_source,
        )
        self.assertGreaterEqual(
            self.manual_drive_source.count("if (!manualEnabled) return undefined;"),
            2,
        )
        self.assertIn("if (!manualEnabled) return null;", self.manual_drive_source)
        for mode in ("ackermann", "zero_turn", "crab"):
            self.assertIn(mode, self.manual_drive_source)
        for token in (
            "const classifiedCommand = (pressed, currentMode)",
            "const hasLongitudinal =",
            "const hasTurn =",
            "const hasCrab =",
            "if (hasCrab && (hasLongitudinal || hasTurn))",
            "mode: 'ackermann'",
            "mode: 'zero_turn'",
            "mode: 'crab'",
            "mode: command.mode",
            "mode,\n      forward: 0,\n      turn: 0,\n      crab: 0",
            "if (!sendZero(previousMode)) return;",
            "driveModeRef.current = command.mode;",
            "forward === 0 ? 0 : turn",
            "role=\"status\"",
            "aria-label=\"자동 분류 주행 모드\"",
            "모드 변경 시 자동 ZERO",
            "const canTrackMotion = manualRef.current.armed || armPendingRef.current",
            "if (!canTrackMotion) return",
            "if (event.repeat || pressedRef.current.has(event.code)) return",
            "if (manualRef.current.armed) sendDrive()",
            "if (nextManual?.armed && armPendingRef.current)",
            "armPendingRef.current = false",
            "armPendingRef.current = true",
            "수동주행 준비 중 · ARM ACK 대기",
            "? { axis: 'Yaw', value: requestedAngularRadps, unit: 'rad/s' }",
            "? { axis: 'Y', value: requestedLateralMps, unit: 'm/s' }",
            ": { axis: 'X', value: requestedLinearMps, unit: 'm/s' }",
            "속도 {activeSpeed.axis} {activeSpeed.value.toFixed(2)} {activeSpeed.unit}",
        ):
            self.assertIn(token, self.manual_drive_source)
        mode_handoff = self.manual_drive_source.split(
            "if (command.mode !== driveModeRef.current)", 1
        )[1].split("send('drive'", 1)[0]
        self.assertLess(
            mode_handoff.index("sendZero(previousMode)"),
            mode_handoff.index("driveModeRef.current = command.mode"),
        )
        self.assertLess(
            mode_handoff.index("driveModeRef.current = command.mode"),
            mode_handoff.rindex("return;"),
        )
        for key in ("KeyW", "KeyA", "KeyS", "KeyD", "KeyZ", "KeyC"):
            self.assertIn(key, self.manual_drive_source)
        self.assertIn(".manual-drive-panel", self.css)
        self.assertIn(".manual-drive-mode-selector", self.css)
        self.assertIn(".manual-drive-mode.active", self.css)
        self.assertIn(".manual-drive-state.pending", self.css)
        self.assertIn("touch-action: none", self.css)

    def test_carla_manual_drive_is_a_collapsible_non_overlay_dock(self) -> None:
        for token in (
            "const [collapsed, setCollapsed] = useState(true)",
            "collapsed ? 'collapsed' : 'expanded'",
            'aria-expanded={!collapsed}',
            'aria-controls="manual-drive-expanded-controls"',
            'hidden={collapsed}',
            "collapsed ? '제어 열기' : '제어 접기'",
            "if (!collapsed) clearMotion(false)",
            'onClick={toggleCollapsed}',
            '>ZERO</button>',
            '>DISARM</button>',
        ):
            self.assertIn(token, self.manual_drive_source)
        collapse_handler = self.manual_drive_source.split(
            "const toggleCollapsed = useCallback", 1
        )[1].split("}, [clearMotion, collapsed]);", 1)[0]
        self.assertLess(
            collapse_handler.index("clearMotion(false)"),
            collapse_handler.index("setCollapsed"),
        )
        for token in (
            ".manual-drive-panel.collapsed",
            ".manual-drive-layout[hidden]",
            ".manual-drive-compact-actions",
            "flex: 0 0 auto",
            "max-height: min(46vh, 430px)",
        ):
            self.assertIn(token, self.css)

    def test_carla_manual_drive_lifecycle_has_zero_and_disarm_boundaries(self) -> None:
        for token in (
            "if (!manualRef.current.armed || disarmPendingRef.current) return false",
            "if (!manualRef.current.armed || disarmPendingRef.current) return",
            "if (nextManual && !nextManual.armed)",
            "const stopForBlur = () => clearMotion(false);",
            "const disarmForPageLifecycle = () => clearMotion(true);",
            "window.addEventListener('blur', stopForBlur)",
            "window.addEventListener('pagehide', disarmForPageLifecycle)",
            "if (document.hidden) disarmForPageLifecycle()",
            "const manualTransitionActive = manualRef.current.armed || armPendingRef.current;",
            "socket.readyState === WebSocket.OPEN && manualTransitionActive",
        ):
            self.assertIn(token, self.manual_drive_source)

        clear_motion = self.manual_drive_source.split(
            "const clearMotion", 1
        )[1].split("useEffect", 1)[0]
        self.assertLess(
            clear_motion.index("sendZero();"),
            clear_motion.index("disarmPendingRef.current = true;"),
        )
        self.assertLess(
            clear_motion.index("disarmPendingRef.current = true;"),
            clear_motion.index("send('disarm');"),
        )

    def test_admin_diagnostics_remain_available_across_service_screens(self) -> None:
        # HH_260810 - Arrival, return, and waiting transitions must not unmount
        # the authenticated diagnostics workspace or hide its entry gesture.
        self.assertIn("activeModal === 'settings'", self.source)
        self.assertIn("admin-runtime-shell", self.source)
        self.assertIn("diag-secret-zone-global", self.source)
        self.assertIn(
            "setActiveModal(current => current === 'settings' ? current : null)",
            self.source,
        )

    def test_operator_map_can_publish_a_confirmed_manual_goal(self) -> None:
        # HH_260810 - The production UI must retain the RViz 2D Goal semantics:
        # pointer position selects x/y, drag selects yaw, confirmation calls API.
        for token in (
            "onPointerDown={beginGoalSelection}",
            "onPointerMove={updateGoalHeading}",
            "manual-goal-marker",
            "goalSelectionMapPoints",
            "/ui/manual_goal?${query.toString()}",
            "선택한 목표로 출발하시겠습니까?",
        ):
            self.assertIn(token, self.telemetry_source)
        self.assertIn("trajectory-plot-goal-active", self.css)
        self.assertIn("manual-goal-confirm", self.css)

    def test_docking_workspace_exposes_commands_image_path_and_charge(self) -> None:
        # HH_260818 - A field docking test must be observable and commandable
        # from the managed UI without opening RViz or a separate browser tool.
        for token in (
            "/ui/manual_return",
            "const rearFallbackEnabled = telemetry.options?.docking_rear_camera_fallback_enabled === true",
            "const useRearFallback = rearFallbackEnabled && !hasFreshDockingDebug",
            "const dockingCameraName = useRearFallback ? 'rear' : 'docking'",
            ": 'AprilTag 도킹 디버그 영상';",
            "{rearFallbackEnabled && (",
            "도킹 화면 · CARLA 실제 후방카메라 대체 영상",
            "CARLA 실제 후방카메라 · 대체 영상",
            "camera={dockingCameraName}",
            "DockingPathPlot",
            "tag_detected",
            "is_charging",
        ):
            self.assertIn(token, self.telemetry_source)
        self.assertIn("requestManualReturn", self.source)
        for token in (
            "waiting_for_disconnect",
            "parking_alignment_waiting_for_can",
            "리모컨을 CAN 모드로 전환하세요",
        ):
            self.assertIn(token, self.source)
            self.assertIn(token, self.telemetry_source)
        self.assertIn("'redock_pending' in data", self.source)
        self.assertIn("'redock_waiting_for_can' in data", self.source)
        self.assertIn("'redock_status' in data", self.source)
        self.assertIn("'redock_message' in data", self.source)
        self.assertIn(
            "expired: '재도킹 요청 시간이 만료되었습니다",
            self.source,
        )
        self.assertIn("data.redock_message ?? data.message", self.source)
        self.assertIn("setRedockStatus(previous =>", self.source)
        self.assertIn("redockStatus={redockStatus}", self.source)
        self.assertIn("function DockingView({ telemetry, redockStatus", self.telemetry_source)
        self.assertIn(
            "redock_status = UiBackendNode._redock_status_snapshot(node)",
            self.backend_source,
        )
        self.assertIn("**redock_status,", self.backend_source)
        self.assertIn(
            "await UiBackendNode._send_ws_json(node, ws, initial_payload)",
            self.backend_source,
        )
        self.assertNotIn("await ws.send_json(initial_payload)", self.backend_source)
        self.assertNotIn("await ws.send_json(redock_status)", self.backend_source)
        self.assertIn(
            "snapshot.update(UiBackendNode._redock_status_snapshot(self))",
            self.backend_source,
        )
        # HH_260819 - The obsolete Parking ON/OFF switch must not bypass the
        # state-aware Return command or appear in either operator surface.
        for removed in ("manual_parking", "toggleManualParking", "Parking OFF"):
            self.assertNotIn(removed, self.source)
            self.assertNotIn(removed, self.telemetry_source)
        self.assertIn(".docking-layout", self.css)

    def test_robot_operator_recall_is_distinct_from_campsite_delivery(self) -> None:
        # A roadside recall must never fall through to the existing destination
        # WebSocket command, which is intentionally ordinary campsite delivery.
        for token in (
            "const [destinationIntent, setDestinationIntent] = useState('delivery')",
            "이용객 호출 · 도로 대기",
            "배송 · 사이트 내부 진입",
            "사이트 내부로 들어가지 않고 도로 측 대기점",
            "destinationIntent === 'delivery'",
            "텐트 · 호출 가능",
            "도로 측 대기점에 도착했습니다.",
            "적재 완료 · 복귀",
            "수령 완료 · 복귀",
            "짐 정리를 마친 후",
            "짐 싣기를 모두 마친 후",
            "배송 물품을 모두 내린 후",
            "serviceStateName === 'GUEST_LOADING_WAIT'",
            "serviceStateName === 'RETURN_WITH_CARGO'",
            "이용객의 짐을 싣고 대기·충전 장소로 복귀 중입니다.",
            "배송을 마치고 대기·충전 장소로 복귀 중입니다.",
        ):
            self.assertIn(token, self.source)

        recall_start = self.source.index("const requestCampingSiteRecall")
        recall_end = self.source.index("const handleToggle", recall_start)
        recall_source = self.source[recall_start:recall_end]
        self.assertIn(
            "/ui/camping_site_recall?site=${encodeURIComponent(site)}&intent=recall",
            recall_source,
        )
        self.assertIn("body.intent !== 'recall'", recall_source)
        self.assertIn("'robot_recall_site' in data", self.source)
        self.assertIn('"site": active_mission_site', self.backend_source)
        self.assertIn(
            '"robot_recall_site": recall_site,',
            self.backend_source,
        )
        self.assertNotIn(
            'await ws.send_json({"robot_recall_site": recall_site})',
            self.backend_source,
        )
        self.assertIn("mission_dispatch_active", self.backend_source)
        self.assertIn("missionDispatchActiveRef.current", self.source)
        self.assertIn("!missionDispatchActiveRef.current", self.source)
        self.assertRegex(
            self.backend_source,
            r"active_mission_site\s+and UiBackendNode\._is_guest_recall_source\(",
        )
        self.assertNotIn(
            "states.get(active_mission_site, False)\n                        and UiBackendNode._is_guest_recall_source",
            self.backend_source,
        )
        self.assertIn(
            "const activeSite = activeRecallSite ? null : activeStateSite",
            self.source,
        )
        self.assertNotIn("fetch('/ui/destination", recall_source)
        self.assertNotIn("fetch(`/ui/destination", recall_source)
        self.assertNotIn("applyToggle", recall_source)

        presentation = self.source[
            self.source.index("const hasExplicitRecallIntent") :
            self.source.index("// ── 운영시간 게이트 확인")
        ]
        self.assertIn(
            "hasExplicitRecallIntent\n    && serviceStateName === 'RETURN_WITH_CARGO'",
            presentation,
        )
        self.assertNotIn(
            "serviceStateName === 'RETURN_WITH_CARGO'\n    ||",
            presentation,
        )
        self.assertIn(
            "serviceState === SERVICE_STATE.WAITING_FOR_CHARGING",
            self.source,
        )
        terminal_start = self.source.index(
            "serviceState === SERVICE_STATE.DROP_ZONE_WAIT"
        )
        terminal_end = self.source.index(
            "} else if (serviceState === SERVICE_STATE.OPERATOR_STOPPED)",
            terminal_start,
        )
        self.assertNotIn(
            "setActiveRecallSite(null)",
            self.source[terminal_start:terminal_end],
        )
        self.assertIn(
            "destinationIntentRef.current !== 'recall'",
            self.source[terminal_start:terminal_end],
        )
        self.assertIn(
            "&& !missionDispatchActiveRef.current",
            self.source[terminal_start:terminal_end],
        )
        self.assertIn(
            "authoritative\n          // empty robot_recall_site",
            self.source[terminal_start:terminal_end],
        )
        replay_start = self.source.index("if ('robot_recall_site' in data)")
        replay_end = self.source.index(
            "if ('occupied_sites' in data", replay_start
        )
        replay_source = self.source[replay_start:replay_end]
        self.assertIn("setShowWaiting(false)", replay_source)
        self.assertIn("destinationIntentRef.current = 'delivery'", replay_source)

    def test_robot_ws_commands_echo_mission_generation_and_reconnect_is_atomic(
        self,
    ) -> None:
        self.assertIn(
            "body.mission_dispatch_generation || 0",
            self.source,
        )
        self.assertIn(
            "missionDispatchGenerationRef.current = admittedGeneration",
            self.source,
        )
        self.assertIn(
            "missionDispatchOwnerRef.current = admittedOwner",
            self.source,
        )
        self.assertIn(
            "mission_generation: st ? 0 : missionDispatchGenerationRef.current",
            self.source,
        )
        self.assertIn(
            "mission_generation: missionDispatchGenerationRef.current",
            self.source,
        )
        self.assertIn(
            '"error": "stale_or_unowned_destination_stop"',
            self.backend_source,
        )
        self.assertIn(
            '"error": "stale_or_unowned_return"',
            self.backend_source,
        )
        registration = self.backend_source.index(
            "node._ws_initializing_clients[ws] = []"
        )
        send_lock_registration = self.backend_source.index(
            "node._ws_client_send_locks[ws] = asyncio.Lock()", registration
        )
        initial_send = self.backend_source.index(
            "await UiBackendNode._send_ws_json(node, ws, initial_payload)",
            send_lock_registration,
        )
        activation = self.backend_source.index(
            "node._ws_clients.add(ws)", initial_send
        )
        self.assertLess(registration, initial_send)
        self.assertLess(send_lock_registration, initial_send)
        self.assertLess(initial_send, activation)
        self.assertIn(
            "queued.append(copy.deepcopy(payload))",
            self.backend_source,
        )
        self.assertIn("async with send_lock:", self.backend_source)
        self.assertIn(
            "node._ws_client_send_locks.pop(ws, None)",
            self.backend_source,
        )

    def test_robot_recall_ignores_late_http_authority_and_orphan_socket(self) -> None:
        recall_start = self.source.index("const requestCampingSiteRecall")
        recall_end = self.source.index("const handleToggle", recall_start)
        recall_source = self.source[recall_start:recall_end]
        for token in (
            "const authorityRevisionAtRequest = missionAuthorityRevisionRef.current",
            "recallRequestEpochRef.current !== requestEpoch",
            "missionAuthorityRevisionRef.current !== authorityRevisionAtRequest",
            "currentAuthorityMatches",
            "이전 호출 응답을 무시했습니다.",
        ):
            self.assertIn(token, recall_source)

        connect_start = self.source.index("const connect = useCallback")
        connect_end = self.source.index(
            "// ── 컴포넌트 마운트/언마운트 시 WebSocket 관리", connect_start
        )
        connect_source = self.source[connect_start:connect_end]
        for token in (
            "wsMountedRef.current",
            "wsGenerationRef.current !== connectionGeneration",
            "wsRef.current !== ws",
            "wsReconnectTimerRef.current = setTimeout",
        ):
            self.assertIn(token, connect_source)

    def test_visible_operator_acceptance_has_stable_minimal_dom_hooks(self) -> None:
        for hook in (
            'data-ui="operator-waiting-screen"',
            'data-ui="operator-open-destination"',
            'data-ui="operator-control-screen"',
            'data-ui="operator-intent-delivery"',
            'data-ui="operator-intent-recall"',
            'data-ui={`operator-site-${site}`}',
            'data-ui={`operator-site-page-${i}`}',
            'data-ui="operator-site-preview-confirm"',
            'data-ui="operator-move-confirm-yes"',
            'data-ui="operator-site-code-input"',
            'data-ui="operator-site-code-confirm"',
            'data-ui="operator-arrival-return-confirm"',
        ):
            self.assertIn(hook, self.source)

        for hook in (
            'data-ui="operator-admin-entry"',
            'data-ui="operator-admin-login-modal"',
            'data-ui="operator-admin-id"',
            'data-ui="operator-admin-password"',
            'data-ui="operator-admin-login"',
            'data-ui={`operator-diagnostic-tab-${tab.id}`}',
        ):
            self.assertIn(hook, self.source)

        for hook in (
            'data-ui="manual-drive-panel"',
            'data-ui="manual-drive-state"',
            'data-ui="manual-drive-toggle"',
            'data-ui="manual-drive-arm"',
            'data-ui="manual-drive-zero"',
            'data-ui="manual-drive-disarm"',
            'data-ui="manual-drive-scale"',
            "data-connected={manual.connected ? 'true' : 'false'}",
            "data-armed={manual.armed ? 'true' : 'false'}",
            "data-mode={driveMode}",
        ):
            self.assertIn(hook, self.manual_drive_source)

    def test_primary_robot_status_and_camera_headings_are_korean(self) -> None:
        for text in (
            "INITIALIZING: '초기화 중'",
            "OK: '시스템 정상'",
            "배터리 상태 확인 중",
            "수동 운행",
            "캠핑 사이트 선택",
        ):
            self.assertIn(text, self.source)
        for text in (
            "전방 카메라",
            "후방 카메라",
            "도킹 상태",
            "주차 접근 경로",
            "영상 없음",
        ):
            self.assertIn(text, self.telemetry_source)

    def test_public_service_evidence_uses_summary_and_bounded_history_apis(self) -> None:
        self.assertIn("/api/service-metrics/summary", self.service_evidence_source)
        self.assertIn("/api/service-metrics?days=30", self.service_evidence_source)
        for field in (
            "current_service",
            "last_completed_service",
            "today",
            "lifetime",
            "daily_history",
            "recent_services",
            "site_summaries",
            "generated_at",
            "persistence",
        ):
            self.assertIn(field, self.service_evidence_source)

    def test_public_service_evidence_preserves_waiting_screen_two_by_two_grid(self) -> None:
        self.assertIn("<ServiceEvidenceSummary", self.source)
        self.assertIn("setActiveModal('service-evidence')", self.source)
        self.assertIn("<ServiceEvidenceDashboard", self.source)
        self.assertIn(
            "grid-template-rows: auto repeat(2, minmax(0, 1fr));",
            self.css,
        )
        self.assertIn(".evidence-summary-strip", self.css)
        self.assertIn("grid-column: 1 / -1;", self.css)
        # Destination plus the three established information cards remain the
        # only children that fill the two service-card rows.
        self.assertIn(
            "SIDE_BUTTONS.filter(btn => btn.id !== 'settings').map",
            self.source,
        )

    def test_active_service_screen_exposes_current_trip_distance(self) -> None:
        self.assertIn("<ServiceTripBadge", self.source)
        self.assertIn("serviceMetrics.data?.current_service", self.source)
        self.assertIn(
            "activeSite || activeRecallSite || arrivedSite || displayedReturning",
            self.source,
        )
        self.assertIn("이번 서비스", self.service_evidence_source)
        self.assertIn(".evidence-trip-badge", self.css)

    def test_active_service_screen_can_open_full_service_evidence(self) -> None:
        self.assertIn(
            "onOpen={() => setActiveModal('service-evidence')}",
            self.source,
        )
        self.assertIn("const serviceEvidenceModal = serviceEvidenceModalOpen", self.source)
        self.assertGreaterEqual(self.source.count("{serviceEvidenceModal}"), 2)
        self.assertIn("실증 운행 현황 상세 보기", self.service_evidence_source)
        self.assertIn("evidence-trip-more", self.service_evidence_source)

    def test_frontend_entry_document_is_never_served_from_stale_cache(self) -> None:
        self.assertIn(
            '"Cache-Control": "no-store, no-cache, must-revalidate"',
            self.backend_source,
        )
        self.assertIn("if real == index_real", self.backend_source)
        self.assertIn(
            "return FileResponse(str(index_real), headers=no_store_headers)",
            self.backend_source,
        )

    def test_evidence_modal_merges_live_summary_into_bounded_history(self) -> None:
        self.assertIn("...detailData", self.service_evidence_source)
        self.assertIn("...summaryData", self.service_evidence_source)
        self.assertIn(
            "daily_history: detailData.daily_history",
            self.service_evidence_source,
        )
        self.assertIn(
            "recent_services: detailData.recent_services",
            self.service_evidence_source,
        )
        self.assertIn(
            "const combinedError = detailError || summaryError;",
            self.service_evidence_source,
        )

    def test_service_evidence_never_substitutes_missing_data_with_zero(self) -> None:
        # Loading, transport failure, and a successful empty history are three
        # different evidence states. Only an actual numeric API value may show 0.
        for message in ("불러오는 중", "확인 불가", "기록 없음"):
            self.assertIn(message, self.service_evidence_source)
        self.assertIn(
            "value === null || value === undefined",
            self.service_evidence_source,
        )
        self.assertNotIn("distance_m || 0", self.service_evidence_source)
        self.assertNotIn("completed_service_count || 0", self.service_evidence_source)

    def test_service_evidence_modal_has_responsive_bounded_layout(self) -> None:
        self.assertIn("service-evidence-modal", self.source)
        self.assertIn(".service-evidence-modal", self.css)
        self.assertIn(".evidence-table-scroll", self.css)
        self.assertIn("@media (max-width: 1000px)", self.css)
        self.assertIn("@media (max-width: 700px)", self.css)

    def test_service_evidence_compares_all_sites_with_live_progress(self) -> None:
        for token in (
            "B1-B13 서비스 비교",
            "SiteTrendChart",
            "buildTrendSeries",
            "evidence-site-trend-line",
            "항목별 독립 척도",
            "average_distance_m",
            "average_duration_s",
            "latest_service",
            "current_service",
            "current_distance_progress_percentage",
            "current_duration_progress_percentage",
            "완료 평균 대비",
        ):
            self.assertIn(token, self.service_evidence_source)
        self.assertIn(".evidence-site-chart-row", self.css)
        self.assertIn(".evidence-site-trend-scroll", self.css)
        self.assertIn(".evidence-site-table", self.css)

    def test_radar_echo_is_not_presented_as_stopping_cost(self) -> None:
        for token in (
            "radarCostSensors",
            "safety.radar_evidence",
            "return 'COST'",
            "return finite(sample.range_m) ? 'ECHO'",
            "Radar echo",
            "Radar cost",
        ):
            self.assertIn(token, self.telemetry_source)
        self.assertNotIn("Radar return", self.telemetry_source)
        self.assertIn(".radar-arc-echo", self.css)
        self.assertIn(".radar-arc-cost", self.css)

    def test_docking_view_shows_exact_lanelet_parking_approach(self) -> None:
        for token in (
            "drop_zone_parking",
            "Lanelet 주차 지점",
            "정확한 Lanelet 지점",
            "docking-path-approach",
        ):
            self.assertIn(token, self.telemetry_source)

    def test_robot_commands_are_bound_to_backend_mission_generation(
        self,
    ) -> None:
        for token in (
            "const missionDispatchGenerationRef = useRef(0)",
            "const missionDispatchSiteRef = useRef('')",
            "const missionDispatchOwnerRef = useRef('')",
            "body.mission_dispatch_generation || 0",
            "missionDispatchGenerationRef.current = admittedGeneration",
            "mission_generation: st ? 0 : missionDispatchGenerationRef.current",
            "mission_generation: missionDispatchGenerationRef.current",
        ):
            self.assertIn(token, self.source)
        self.assertIn(
            '"error": "stale_or_unowned_destination_stop"',
            self.backend_source,
        )
        self.assertIn(
            '"error": "stale_or_unowned_return"',
            self.backend_source,
        )

    def test_robot_recall_rejects_late_http_and_orphan_socket_authority(
        self,
    ) -> None:
        recall_start = self.source.index("const requestCampingSiteRecall")
        recall_end = self.source.index("const handleToggle", recall_start)
        recall_source = self.source[recall_start:recall_end]
        for token in (
            "const authorityRevisionAtRequest = missionAuthorityRevisionRef.current",
            "recallRequestEpochRef.current !== requestEpoch",
            "missionAuthorityRevisionRef.current !== authorityRevisionAtRequest",
            "currentAuthorityMatches",
            "이전 호출 응답을 무시했습니다.",
        ):
            self.assertIn(token, recall_source)

        connect_start = self.source.index("const connect = useCallback")
        connect_end = self.source.index(
            "// ── 컴포넌트 마운트/언마운트 시 WebSocket 관리",
            connect_start,
        )
        connect_source = self.source[connect_start:connect_end]
        for token in (
            "wsMountedRef.current",
            "wsGenerationRef.current !== connectionGeneration",
            "wsRef.current !== ws",
            "wsReconnectTimerRef.current = setTimeout",
        ):
            self.assertIn(token, connect_source)

    def test_guest_owned_recall_exposes_only_bound_robot_completion(
        self,
    ) -> None:
        self.assertIn(
            "const [missionDispatch, setMissionDispatch] = useState",
            self.source,
        )
        self.assertIn("const robotOwnsReturn =", self.source)
        self.assertIn("const guestOwnsReturn =", self.source)
        self.assertIn(
            "['operator', 'robot'].includes(dispatch.owner)",
            self.source,
        )
        self.assertIn("{robotOwnsReturn ? (", self.source)
        self.assertIn(
            "이 임무의 완료·복귀는 이용객 화면에서 진행합니다.",
            self.source,
        )
        self.assertIn(
            "현재 임무의 복귀 권한을 확인하고 있습니다.",
            self.source,
        )
        self.assertIn(
            "if (!dispatchActive || (dispatchOwner === 'guest' && dispatchIntent !== 'recall'))",
            self.source,
        )
        self.assertNotIn(
            "missionDispatchActiveRef.current = Boolean(newState)",
            self.source,
        )
        handler_start = self.source.index(
            "const handleArrivalComplete = () => {"
        )
        handler_end = self.source.index(
            "// ── 실제 토글 적용",
            handler_start,
        )
        handler = self.source[handler_start:handler_end]
        self.assertIn("if (!robotOwnsReturn)", handler)
        self.assertIn("현재 운행의 복귀 권한이 이 화면에 없습니다.", handler)
        self.assertNotIn("setIsReturning(true)", handler)
        self.assertIn(
            "wsRef.current.readyState !== WebSocket.OPEN",
            handler,
        )

    def test_recall_return_progress_matches_both_uis_for_every_site(
        self,
    ) -> None:
        guest_source = (
            APP_SOURCE.parents[4]
            / "camrod_ui_guest"
            / "assets"
            / "guest_frontend"
            / "index.html"
        ).read_text(encoding="utf-8")
        phases = {
            "RECALL_CLEARANCE_WAIT": "사이트 비움 안내 중",
            "ALIGN_ENTRY_YAW": "사이트 재진입 준비",
            "CRAB_IN": "사이트 재진입 중",
            "ROTATE_180": "사이트 안에서 180도 회전 중",
            "RECALL_RETURN_WAIT": "짐 싣기 완료 확인 대기",
            "ALIGN_RETRACE_YAW": "도로 출차 준비",
            "CRAB_OUT": "사이트에서 도로로 출차 중",
        }
        inputs = [
            [
                f"B{site}",
                f"camping_site_maneuver_controller:{phase}:active",
            ]
            for site in range(1, 14)
            for phase in phases
        ] + [["B3", "other_controller:ROTATE_180:active"], ["B3", ""]]
        outputs = []
        for source, end_marker in (
            (self.source, "// HH_260904 - Re-dock events"),
            (guest_source, "function updateUI()"),
        ):
            start = source.index("function recallReturnInstructions(site,")
            helpers = source[start : source.index(end_marker, start)]
            script = (
                helpers
                + "\nprocess.stdout.write(JSON.stringify("
                + json.dumps(inputs)
                + ".map(([site, description]) => "
                + "({instructions: recallReturnInstructions(site), "
                + "...recallReturnProgress(site, description)}))));"
            )
            result = subprocess.run(
                ["node"],
                input=script,
                text=True,
                capture_output=True,
                check=True,
            )
            outputs.append(json.loads(result.stdout))
        self.assertEqual(outputs[0], outputs[1])
        for (site, description), output in zip(inputs, outputs[0]):
            with self.subTest(site=site, description=description):
                phase = (
                    description.split(":")[1] if ":" in description else ""
                )
                if int(site[1:]) <= 10 and description.startswith(
                    "camping_site_maneuver_controller:"
                ):
                    self.assertEqual(output["label"], phases[phase])
                    self.assertIn("180도", output["instructions"])
                else:
                    self.assertEqual(output["label"], "짐을 싣고 복귀 중")
                if int(site[1:]) >= 11:
                    self.assertIn("기존 반대쪽 경로", output["message"])
                    self.assertNotIn("180도", output["instructions"])

    def test_robot_completion_keeps_guest_mission_identity_and_phase_guards(
        self,
    ) -> None:
        start = self.source.index("function robotCanCompleteMission(")
        helper = self.source[
            start : self.source.index(
                "// HH_260904 - Re-dock events",
                start,
            )
        ]
        admitted = {
            "active": True,
            "site": "B4",
            "generation": 12,
            "owner": "guest",
            "intent": "recall",
        }
        cases = [
            [admitted, "B4", "GUEST_LOADING_WAIT"],
            [admitted, "B5", "GUEST_LOADING_WAIT"],
            [admitted, "B4", "RETURN_WITH_CARGO"],
            [{**admitted, "active": False}, "B4", "GUEST_LOADING_WAIT"],
            [{**admitted, "intent": "delivery"}, "B4", "GUEST_LOADING_WAIT"],
            [{**admitted, "generation": 0}, "B4", "GUEST_LOADING_WAIT"],
            [{**admitted, "generation": "12"}, "B4", "GUEST_LOADING_WAIT"],
            [{**admitted, "owner": "robot"}, "B4", "GUEST_LOADING_WAIT"],
            [
                {**admitted, "owner": "operator", "intent": "delivery"},
                "B4",
                "UNLOAD_WAIT",
            ],
        ]
        script = (
            helper
            + "\nprocess.stdout.write(JSON.stringify("
            + json.dumps(cases)
            + ".map(args => robotCanCompleteMission(...args))));"
        )
        result = subprocess.run(
            ["node"],
            input=script,
            text=True,
            capture_output=True,
            check=True,
        )
        self.assertEqual(
            json.loads(result.stdout),
            [True, False, False, False, False, False, False, True, True],
        )

    def test_two_stage_completion_sends_explicit_stage_once_and_blocks_execution_failure(
        self,
    ) -> None:
        guest_source = (
            APP_SOURCE.parents[4]
            / "camrod_ui_guest"
            / "assets"
            / "guest_frontend"
            / "index.html"
        ).read_text(encoding="utf-8")
        helper_start = self.source.index(
            "function recallReturnInstructions(site,"
        )
        helpers = self.source[
            helper_start : self.source.index(
                "// HH_260904 - Re-dock events",
                helper_start,
            )
        ]
        robot_start = self.source.index(
            "const handleArrivalComplete = () => {"
        )
        robot_action = self.source[
            robot_start : self.source.index(
                "// ── 실제 토글",
                robot_start,
            )
        ]
        guest_start = guest_source.index("function sendUsageComplete() {")
        guest_action = guest_source[
            guest_start : guest_source.index(
                "function sendCancel()",
                guest_start,
            )
        ]
        script = helpers + r"""
const robotCalls = [], guestCalls = [], warnings = [];
const WebSocket = {OPEN: 1};
let recallFinalReturnReady = false;
const missionExecutionErrorRef = {current: ''}, returnRequestPendingRef = {current: false};
const missionDispatchSiteRef = {current: 'B4'}, missionDispatchGenerationRef = {current: 12};
const robotOwnsReturn = true;
const wsRef = {current: {readyState: 1, send: raw => robotCalls.push(JSON.parse(raw))}};
const setMissionBlockMessage = message => warnings.push(message);
const setReturnRequestPending = () => {}, setShowArrivalComplete = () => {};
""" + robot_action + r"""
handleArrivalComplete(); handleArrivalComplete();
returnRequestPendingRef.current = false; recallFinalReturnReady = true;
handleArrivalComplete(); handleArrivalComplete();
returnRequestPendingRef.current = false; missionExecutionErrorRef.current = 'site entry failed';
handleArrivalComplete();
let missionExecutionError = '', usageCompletePending = false, usageCompleteError = '';
const activeRequestIntent = 'recall', activeRequestOwner = 'guest';
const currentServiceStateName = 'GUEST_LOADING_WAIT', lastDestSite = 'B4';
const ws = {readyState: 1, send: raw => guestCalls.push(JSON.parse(raw))};
const window = {confirm: () => true}, updateUI = () => {};
""" + guest_action + r"""
recallFinalReturnReady = false; sendUsageComplete(); sendUsageComplete();
usageCompletePending = false; recallFinalReturnReady = true;
sendUsageComplete(); sendUsageComplete();
usageCompletePending = false; missionExecutionError = 'site entry failed'; sendUsageComplete();
const labels = [recallCompletionLabel('B4', false), recallCompletionLabel('B4', true), recallCompletionLabel('B11', false)];
process.stdout.write(JSON.stringify({robotCalls, guestCalls, warnings, labels}));
"""
        result = subprocess.run(
            ["node"],
            input=script,
            text=True,
            capture_output=True,
            check=True,
        )
        output = json.loads(result.stdout)
        self.assertEqual(
            output["robotCalls"],
            [
                {
                    "usage_complete": True,
                    "site": "B4",
                    "mission_generation": 12,
                    "recall_final_return": stage,
                }
                for stage in (False, True)
            ],
        )
        self.assertEqual(
            output["guestCalls"],
            [
                {"action": "usage_complete", "recall_final_return": stage}
                for stage in (False, True)
            ],
        )
        self.assertIn("관리자", output["warnings"][0])
        self.assertEqual(
            output["labels"],
            [
                "정리 완료 · 사이트 재진입",
                "짐 싣기 완료 · 복귀",
                "적재 완료 · 복귀",
            ],
        )

    def test_robot_reconnect_restores_completion_and_preserves_minimal_phase_frames(
        self,
    ) -> None:
        # Use the actual initial backend frame and actual browser message
        # handler. A helper-only test cannot catch missing snapshot fields or
        # the extra minimal state frame erasing the preceding phase detail.
        endpoint = next(
            node
            for node in ast.walk(ast.parse(self.backend_source))
            if isinstance(node, ast.AsyncFunctionDef)
            and node.name == "websocket_endpoint"
        )
        snapshot = next(
            node.value
            for node in ast.walk(endpoint)
            if isinstance(node, ast.Assign)
            and any(
                isinstance(target, ast.Name)
                and target.id == "initial_payload"
                for target in node.targets
            )
            and isinstance(node.value, ast.Dict)
            and any(
                isinstance(key, ast.Constant)
                and key.value == "service_state_description"
                for key in node.value.keys
            )
        )
        arrival_frame = eval(
            compile(
                ast.Expression(snapshot),
                "initial_robot_snapshot",
                "eval",
            ),
            {},
            {
                "states": {},
                "recall_site": "B4",
                "occupied_sites": [],
                "engage": True,
                "ready": True,
                "ready_message": "ready",
                "mission_phase": "ARRIVED",
                "mission_source": "service",
                "system_health": "OK",
                "service_state": 8,
                "service_state_name": "GUEST_LOADING_WAIT",
                "service_state_description": (
                    "camping_site_maneuver_controller:WAIT_RETURN:loading"
                ),
                "active_mission_site": "B4",
                "mission_dispatch": {},
                "redock_status": {},
                "battery_parking_policy": {},
            },
        )
        prefix = self.source[
            self.source.index("const SERVICE_STATE =") : self.source.index(
                "// HH_260904 - Re-dock events"
            )
        ]
        battery_helpers_start = self.source.index(
            "const emptyBatteryReturnState ="
        )
        prefix += self.source[
            battery_helpers_start : self.source.index(
                "// HH_260721 - Reuse one health",
                battery_helpers_start,
            )
        ]
        handler_start = self.source.index("ws.onmessage = (event) => {")
        handler = self.source[
            handler_start : self.source.index(
                "// HH_260708 - Reconnect the operator WebSocket",
                handler_start,
            )
        ]
        setters = sorted(set(re.findall(r"\b(set[A-Z]\w*)\(", handler)))
        refs = sorted(
            set(re.findall(r"\b(\w+Ref)\.current", handler)) - {"wsRef"}
        )
        setup = (
            "\n".join(
                f"const {name} = value => {{uiState.{name} = "
                f"typeof value === 'function' ? value(uiState.{name}) : value;}};"
                for name in setters
            )
            + "\n"
            + "\n".join(
                f"const {name} = {{current: null}};" for name in refs
            )
        )
        script = (
            prefix
            + "\nconst uiState = {}; const ws = {}; "
            + "const wsRef = {current: ws};\n"
            + setup
            + "\nuiState.setOccupiedSites = [];\n"
            + r"""
const SITE_NAMES = Array.from({length: 13}, (_, i) => `B${i + 1}`);
const connectionGeneration = 1;
wsMountedRef.current = true;
wsGenerationRef.current = 1;
missionAuthorityRevisionRef.current = 0;
destinationIntentRef.current = 'delivery';
batteryReturnStateRef.current = emptyBatteryReturnState();
"""
            + handler
            + "\nconst send = frame => ws.onmessage({data: JSON.stringify(frame)});\n"
        )
        script += "send(" + json.dumps(arrival_frame) + ");\n" + r"""
send({mission_dispatch_active: true, mission_dispatch_generation: 12,
      mission_dispatch_site: 'B4', mission_dispatch_owner: 'guest',
      mission_dispatch_intent: 'recall'});
const restored = {
  arrived: uiState.setArrivedSite,
  modal: uiState.setShowArrivalComplete,
  permitted: robotCanCompleteMission(uiState.setMissionDispatch, uiState.setArrivedSite, uiState.setServiceStateName),
};
send({battery_return_pending: false});
const afterBatteryHeartbeat = {
  arrived: uiState.setArrivedSite,
  modal: uiState.setShowArrivalComplete,
  permitted: robotCanCompleteMission(uiState.setMissionDispatch, uiState.setArrivedSite, uiState.setServiceStateName),
};
send({service_state: 9, service_state_name: 'RETURN_WITH_CARGO',
      service_state_description: 'camping_site_maneuver_controller:RECALL_CLEARANCE_WAIT:active'});
send({service_state: 9, returning: true});
const preserved = uiState.setServiceStateDescription;
send({service_state: 10});
send({battery_return_pending: true, battery_return_started: true, battery_return_urgent: true});
send({battery_return_urgent: false});
send({parking_policy_mode: 'auto', parking_selected_method: 'apriltag', charging_required: true});
send({mission_execution_error: 'prepareRecallTurnaroundEntry failed', recall_final_return_ready: false});
send({service_state: 8, site: 'B4'});
const failed = {reason: uiState.setMissionExecutionError, modal: uiState.setShowArrivalComplete};
send({mission_execution_error: '', recall_final_return_ready: true});
send({service_state: 8, site: 'B4'});
const finalStage = {ready: uiState.setRecallFinalReturnReady, error: uiState.setMissionExecutionError};
const batteryReturn = uiState.setBatteryReturnState;
send({mission_dispatch_active: true, mission_dispatch_generation: 13,
      mission_dispatch_site: 'B1', mission_dispatch_owner: 'operator',
      mission_dispatch_intent: 'delivery', service_state: 11,
      service_state_name: 'WAITING_FOR_RETURN_REQUEST', site: 'B1'});
send({battery_return_pending: false});
const operatorAfterHeartbeat = {
  arrived: uiState.setArrivedSite,
  modal: uiState.setShowArrivalComplete,
  permitted: robotCanCompleteMission(uiState.setMissionDispatch, uiState.setArrivedSite, uiState.setServiceStateName),
};
process.stdout.write(JSON.stringify({restored, afterBatteryHeartbeat, operatorAfterHeartbeat, preserved, cleared: uiState.setServiceStateDescription,
  batteryReturn, parking: uiState.setParkingPolicy, failed, finalStage}));
"""
        result = subprocess.run(
            ["node"],
            input=script,
            text=True,
            capture_output=True,
            check=True,
        )
        output = json.loads(result.stdout)
        self.assertEqual(
            output["restored"],
            {"arrived": "B4", "modal": True, "permitted": True},
        )
        self.assertEqual(output["afterBatteryHeartbeat"], output["restored"])
        self.assertEqual(
            output["operatorAfterHeartbeat"],
            {"arrived": "B1", "modal": True, "permitted": True},
        )
        self.assertEqual(
            output["preserved"],
            "camping_site_maneuver_controller:RECALL_CLEARANCE_WAIT:active",
        )
        self.assertEqual(output["cleared"], "")
        self.assertTrue(output["batteryReturn"]["pending"])
        self.assertTrue(output["batteryReturn"]["started"])
        self.assertFalse(output["batteryReturn"]["urgent"])
        self.assertEqual(
            output["parking"]["parking_selected_method"],
            "apriltag",
        )
        self.assertTrue(output["parking"]["charging_required"])
        self.assertEqual(
            output["failed"],
            {"reason": "prepareRecallTurnaroundEntry failed", "modal": False},
        )
        self.assertEqual(
            output["finalStage"],
            {"ready": True, "error": ""},
        )

    def test_battery_messages_distinguish_urgent_return_and_normal_parking_boundaries(
        self,
    ) -> None:
        start = self.source.index("const emptyBatteryReturnState =")
        helpers = self.source[
            start : self.source.index(
                "// HH_260721 - Reuse one health",
                start,
            )
        ]
        guest_source = (
            APP_SOURCE.parents[4]
            / "camrod_ui_guest"
            / "assets"
            / "guest_frontend"
            / "index.html"
        ).read_text(encoding="utf-8")
        guest_start = guest_source.index("function guestBatteryPolicyMessage(")
        guest_helper = guest_source[
            guest_start : guest_source.index(
                "/* ── UI state machine",
                guest_start,
            )
        ]
        script = (
            "const URGENT_BATTERY_RETURN_PERCENT = 25; "
            "const MISSION_DISPATCH_MINIMUM_PERCENT = 35;\n"
            + helpers
            + guest_helper
            + r"""
const values = [null, 20, 24.9, 25, 34.9, 35, 80];
const robot = values.map(value => batteryPolicyStatus(value, emptyBatteryReturnState()));
const guest = values.map(value => guestBatteryPolicyMessage(value, false, false, ''));
const urgent = formatBatteryReturnMessage({battery_percentage: 24, battery_return_urgent: true});
process.stdout.write(JSON.stringify({robot, guest, urgent}));
"""
        )
        result = subprocess.run(
            ["node"],
            input=script,
            text=True,
            capture_output=True,
            check=True,
        )
        output = json.loads(result.stdout)
        self.assertEqual(
            output["robot"][0]["label"],
            "배터리 상태 확인 중",
        )
        for index in (1, 2):
            self.assertIn("긴급 복귀", output["robot"][index]["label"])
            self.assertIn("25% 미만", output["guest"][index])
        for index in (3, 4):
            self.assertEqual(output["robot"][index]["tone"], "warning")
            self.assertIn("현재 작업 완료 후", output["guest"][index])
        for index in (5, 6):
            self.assertEqual(output["robot"][index]["tone"], "ok")
            self.assertIn("충전하지 않고", output["guest"][index])
        self.assertIn("현재 작업을 중단", output["urgent"])
        self.assertNotIn("Critical battery stop", self.source)
        self.assertNotIn("/ui/dock", guest_source)

    def test_explicit_dock_posts_only_dock_and_reports_backend_rejection(
        self,
    ) -> None:
        start = self.telemetry_source.index(
            "export async function postDockingRequest("
        )
        helper = self.telemetry_source[
            start : self.telemetry_source.index(
                "export function DockingCommandButton(",
                start,
            )
        ]
        helper = helper.replace("export ", "")
        script = helper + r"""
(async () => {
  const calls = [];
  const accepted = await postDockingRequest(async (url, options) => {
    calls.push({url, options});
    return {ok: true, json: async () => ({success: true, action: 'force_docking'})};
  });
  let error = '';
  try { await postDockingRequest(async (url, options) => {
    calls.push({url, options});
    return {ok: false, json: async () => ({success: false, message: 'CAN unavailable'})};
  }); } catch (failure) { error = failure.message; }
  const reverse = parkingPolicyMessage({parking_policy_mode: 'auto', parking_selected_method: 'reverse'});
  const april = parkingPolicyMessage({parking_policy_mode: 'auto', parking_selected_method: 'apriltag'});
  const stationAllowed = ['DROP_ZONE_WAIT', 'WAITING_FOR_CHARGING', 'CHARGING', 'DROP_ZONE_PARKING']
    .map(dockingAllowedAtServiceState);
  const awayBlocked = ['', 'PREPARING', 'OPERATOR_STOPPED', 'GOING_TO_SITE', 'GUEST_LOADING_WAIT', 'RETURN_WITH_CARGO']
    .map(dockingAllowedAtServiceState);
  process.stdout.write(JSON.stringify({calls, accepted, error, reverse, april, stationAllowed, awayBlocked}));
})();
"""
        result = subprocess.run(
            ["node"],
            input=script,
            text=True,
            capture_output=True,
            check=True,
        )
        output = json.loads(result.stdout)
        self.assertEqual(
            output["calls"],
            [{"url": "/ui/dock", "options": {"method": "POST"}}] * 2,
        )
        self.assertEqual(output["accepted"]["action"], "force_docking")
        self.assertEqual(output["error"], "CAN unavailable")
        self.assertIn("충전하지 않음", output["reverse"])
        self.assertEqual(output["april"], "충전 도킹 선택됨")
        self.assertEqual(output["stationAllowed"], [True] * 4)
        self.assertEqual(output["awayBlocked"], [False] * 6)
        self.assertGreaterEqual(self.source.count("<DockingCommandButton"), 3)
        self.assertIn("<DockingCommandButton", self.telemetry_source)


if __name__ == "__main__":
    unittest.main()
