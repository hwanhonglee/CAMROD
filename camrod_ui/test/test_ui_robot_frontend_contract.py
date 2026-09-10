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

    def test_parking_and_charging_lifecycle_has_distinct_labels(self) -> None:
        for label in (
            "충전 중",
            "충전 연결 대기 중",
            "주차 진행 중",
            "도킹 진행 중",
            "Drop-zone parking in progress",
            "Parked at drop zone",
        ):
            self.assertIn(label, self.source)
        self.assertIn("parkingLifecycleStatus(", self.source)
        self.assertIn("배달 서비스 및 호출 서비스 이용이 가능합니다.", self.source)
        self.assertIn(
            "주차 정렬이 완료되었습니다. 충전 접점 연결을 기다리고 있습니다.",
            self.source,
        )
        self.assertIn('className="preview-service-available"', self.source)
        self.assertIn("충전 완료", self.source)
        self.assertIn("배터리가 100%로 충전되었습니다.", self.source)
        self.assertIn("setBatteryChargeComplete", self.source)
        self.assertIn("serviceStateName={serviceStateName}", self.source)
        self.assertIn("serviceStateDescription={serviceStateDescription}", self.source)
        self.assertIn("tone: 'parking',", self.source)
        self.assertIn('className="waiting-runtime-dot"', self.source)
        # The green header keeps only Wi-Fi, SOC, and the clock.
        self.assertNotIn("<RuntimeStatus", self.source)
        self.assertNotIn("ch-runtime", self.source)

        css_source = APP_CSS.read_text(encoding="utf-8")
        self.assertIn(".preview-service-available", css_source)
        self.assertIn(
            ".waiting-runtime-item.parking .waiting-runtime-dot",
            css_source,
        )
        self.assertNotIn("ch-runtime", css_source)

    def test_waiting_runtime_status_is_below_banner_and_above_evidence(self) -> None:
        waiting_start = self.source.index("if (showWaiting)")
        waiting_end = self.source.index("if (showServiceSelection)", waiting_start)
        waiting_source = self.source[waiting_start:waiting_end]

        header_start = waiting_source.index('className="waiting-header"')
        panel_start = waiting_source.index("<WaitingRuntimeStatusPanel", header_start)
        evidence_start = waiting_source.index("<ServiceEvidenceSummary", panel_start)
        self.assertNotIn("<RuntimeStatus", waiting_source[header_start:panel_start])
        self.assertLess(panel_start, evidence_start)
        self.assertIn('className="waiting-runtime-panel"', self.source)

        css_source = APP_CSS.read_text(encoding="utf-8")
        self.assertIn(".waiting-runtime-panel", css_source)
        self.assertIn(
            "grid-template-columns: repeat(4, minmax(0, 1fr));",
            css_source,
        )

    def test_charging_connection_wait_returns_to_idle_prompt_after_ten_seconds(self) -> None:
        self.assertIn("const chargingStandbyOpenedRef = useRef(false);", self.source)
        timer_start = self.source.index(
            "// 충전 접점 연결을 기다리는 동안 상태 안내를 10초간 유지한 뒤"
        )
        timer_end = self.source.index("// HJ_260804", timer_start)
        timer = self.source[timer_start:timer_end]
        for expected in (
            "serviceStateName !== 'WAITING_FOR_CHARGING'",
            "missionDispatch.active",
            "chargingStandbyOpenedRef.current = true;",
            "setShowServiceSelection(false);",
            "setShowWaiting(true);",
            "}, 10000);",
        ):
            self.assertIn(expected, timer)
        self.assertIn("서비스 선택 버튼을 눌러주세요", self.source)

    def test_completed_charge_returns_to_idle_prompt_after_ten_seconds(self) -> None:
        self.assertIn("const chargeCompleteStandbyOpenedRef = useRef(false);", self.source)
        timer_start = self.source.index(
            "// A confirmed full battery gets its own completion presentation"
        )
        timer_end = self.source.index("// HJ_260804", timer_start)
        timer = self.source[timer_start:timer_end]
        for expected in (
            "serviceStateName === 'CHARGING' && batteryChargeComplete",
            "missionDispatch.active",
            "chargeCompleteStandbyOpenedRef.current = true;",
            "setShowServiceSelection(false);",
            "setShowWaiting(true);",
            "}, 10000);",
        ):
            self.assertIn(expected, timer)

    def test_destination_entry_opens_three_block_service_menu(self) -> None:
        handler_start = self.source.index("const handleWaitingClick = () => {")
        handler_end = self.source.index("};", handler_start)
        handler = self.source[handler_start:handler_end]
        self.assertIn("setShowServiceSelection(true);", handler)
        self.assertIn("setShowWaiting(false);", handler)

        menu_start = self.source.index("if (showServiceSelection)")
        # Bound the actual production control layout without a simulator-only hook.
        menu_end = self.source.index('<div className="main-layout" onClick=', menu_start)
        menu = self.source[menu_start:menu_end]
        for expected in (
            "배달 서비스",
            "선택한 캠핑 사이트 안으로 짐을 배달해드립니다.",
            "호출 서비스",
            "사이트 내부 진입 없이 도로 측 대기점으로 이동합니다.",
            "service-selection-dock-wrap",
            "대기·충전 장소에서 충전을 시작합니다.",
            "setShowDeliveryConfirm(true)",
            "activateDestinationService('recall')",
        ):
            self.assertIn(expected, menu)
        self.assertNotIn("관리자 인증", menu)
        # The service chooser shows the same operating-status band as standby.
        body_start = menu.index('<main className="service-selection-body">')
        grid_start = menu.index('<div className="service-selection-grid">', body_start)
        self.assertLess(body_start, grid_start)
        panel_start = menu.index("<WaitingRuntimeStatusPanel")
        self.assertLess(panel_start, body_start)
        self.assertIn("setShowServiceSelection(true)", self.source)
        # Recall and docking must confirm through the in-page styled dialog, never
        # through the browser's native confirm() chrome.
        self.assertIn("setShowRecallConfirm(true)", menu)
        self.assertIn("setShowDeliveryConfirm(true)", menu)
        self.assertNotIn("window.confirm", menu)
        for expected in (
            "호출 서비스는 사이트 내부로 진입하지 않습니다.<br />",
            "배달 서비스는 사이트 내부로 진입합니다.<br />",
            "사이트 내부의 텐트 및 장비가 있는지 확인해주세요. 진행하시겠습니까?",
            "도로 측 대기점으로 이동합니다. 진행하시겠습니까?",
            "배터리 잔량과 관계없이 충전을 요청합니다.<br />",
            "충전을 진행하시겠습니까?",
            'className="move-confirm-yes"',
            'className="move-confirm-no"',
        ):
            self.assertIn(expected, self.source)

        css_source = APP_CSS.read_text(encoding="utf-8")
        self.assertIn(".service-selection-grid", css_source)
        self.assertIn("grid-column: 1 / -1;", css_source)

    def test_service_card_is_the_only_place_the_mission_role_is_chosen(self) -> None:
        # 배달 서비스 and 호출 서비스 each own exactly one role. The destination
        # screen must show the confirmed role read-only instead of asking again.
        self.assertNotIn('aria-label="사이트 운행 목적"', self.source)
        self.assertNotIn("selectDestinationIntent('delivery')", self.source)
        self.assertNotIn("selectDestinationIntent('recall')", self.source)
        # activateDestinationService holds the only call site left.
        self.assertEqual(self.source.count("selectDestinationIntent("), 1)
        # The confirmed role sits on its own full-width band above the site
        # viewer and the site grid, not inside the right-hand panel.
        self.assertIn(
            "className={`mission-role-banner role-${destinationIntent}`}",
            self.source,
        )
        banner_start = self.source.index('className={`mission-role-banner')
        body_start = self.source.index('<div className="control-body">', banner_start)
        panel_start = self.source.index('<div className="app">', body_start)
        self.assertLess(banner_start, body_start)
        self.assertLess(body_start, panel_start)
        self.assertNotIn("destination-intent-badge", self.source)
        # Standby, the service chooser, and the destination screen all carry
        # the operating-status band.
        self.assertEqual(self.source.count("<WaitingRuntimeStatusPanel"), 3)
        # The status band sits above the role banner on this screen.
        destination_panel = self.source.rindex(
            "<WaitingRuntimeStatusPanel", 0, banner_start
        )
        self.assertLess(destination_panel, banner_start)
        self.assertLess(banner_start, body_start)

        # An idle "no mission" snapshot must not silently downgrade a recall
        # visit back to delivery once the toggle is gone.
        self.assertIn(
            "} else if (!dispatchIntent && !intentPinnedRef.current) {",
            self.source,
        )
        activate_start = self.source.index("const activateDestinationService = (intent) => {")
        activate_end = self.source.index("};", activate_start)
        self.assertIn(
            "intentPinnedRef.current = true;",
            self.source[activate_start:activate_end],
        )
        self.assertIn("if (showWaiting) intentPinnedRef.current = false;", self.source)

        css_source = APP_CSS.read_text(encoding="utf-8")
        self.assertIn(".mission-role-banner", css_source)
        self.assertIn(".mission-role-banner.role-recall", css_source)
        self.assertNotIn(".destination-intent-badge", css_source)

    def test_engage_and_light_commands_live_in_operator_diagnostics(self) -> None:
        # ENGAGE is an operator authority, not a public campsite tile. It must
        # not sit in the site grid, and the safety/control tab owns it.
        self.assertNotIn('<span className="site-label">ENGAGE</span>', self.source)
        self.assertNotIn("onClick={handleEngage}", self.source)
        # Neither command is a campsite tile any more.
        self.assertNotIn("engage-card", self.source)
        self.assertNotIn('<span className="site-label">LIGHT</span>', self.source)
        self.assertIn("onToggleEngage={handleEngage}", self.source)
        self.assertIn("engageDisabled={isReturning}", self.source)

        self.assertIn("className=\"safety-engage-row\"", self.telemetry_source)
        self.assertIn("onClick={onToggleEngage}", self.telemetry_source)
        self.assertIn("disabled={engageDisabled}", self.telemetry_source)
        self.assertIn("{engageState ? 'ENGAGE OFF' : 'ENGAGE ON'}", self.telemetry_source)
        safety_start = self.telemetry_source.index("function SafetyView(")
        engage_start = self.telemetry_source.index("safety-engage-row", safety_start)
        layout_start = self.telemetry_source.index(
            "telemetry-safety-layout", safety_start
        )
        self.assertLess(engage_start, layout_start)

        # LIGHT belongs to the diagnostics System tab command bar.
        self.assertIn("onToggleHeadlight={handleHeadlight}", self.source)
        self.assertIn("headlight-command-btn", self.source)
        self.assertIn("{headlightState ? 'LIGHT OFF' : 'LIGHT ON'}", self.source)
        # LIGHT owns its own panel below the Manual Motion bar.
        motion_bar = self.source.index('className="diag-control-bar"')
        light_bar = self.source.index('className="diag-control-bar light-control-bar"')
        tuning_start = self.source.index("steering-tuning-card", motion_bar)
        self.assertLess(motion_bar, light_bar)
        self.assertLess(light_bar, tuning_start)
        light_btn = self.source.index("headlight-command-btn", light_bar)
        self.assertLess(light_btn, tuning_start)
        self.assertIn("Light Control", self.source)

        css_source = APP_CSS.read_text(encoding="utf-8")
        self.assertIn(".safety-engage-btn", css_source)
        self.assertIn(".headlight-command-btn", css_source)
        self.assertNotIn(".engage-card", css_source)

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
        self.assertIn("운행을 정지하시겠습니까?", returning_preview)
        self.assertIn("onClick={handleStopMove}", returning_preview)

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
        self.assertIn("{activeRecallSite} 호출", self.source)
        self.assertEqual(self.source.count("{guestAdmissionStatus}"), 2)
        self.assertNotIn('className="guest-recall-overlay"', self.source)
        arrival = self.source[self.source.index(") : arrivedSite ? ("):
                              self.source.index(") : ['CHARGING'")]
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
            "지도 · 인지", "안전 · 제어", "충전 · 주차",
        ):
            self.assertIn(label, self.telemetry_source)
        self.assertIn("TelemetryWorkspace", self.source)
        self.assertIn("diag-tab-bar", self.css)

    def test_operator_telemetry_lease_is_closed_on_unmount(self) -> None:
        self.assertIn("/ws/telemetry?view=${view}", self.telemetry_source)
        self.assertIn("/api/telemetry/session?active=true", self.telemetry_source)
        self.assertIn("/api/telemetry/session?active=false", self.telemetry_source)
        self.assertIn("view=${view}", self.telemetry_source)
        self.assertIn("keepalive: true", self.telemetry_source)
        self.assertIn("currentSocket.send('lease')", self.telemetry_source)
        self.assertIn("}, 4000);", self.telemetry_source)

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
            'camera="docking"',
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
        self.assertIn("await ws.send_json(redock_status)", self.backend_source)
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
            "'호출 서비스' : '배달 서비스'",
            "사이트 내부로 들어가지 않고 도로 측 대기점",
            "destinationIntent === 'delivery'",
            "텐트 · 호출 가능",
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
        self.assertIn(
            "const activeSite = activeRecallSite ? null : activeStateSite",
            self.source,
        )
        self.assertNotIn("fetch('/ui/destination", recall_source)
        self.assertNotIn("fetch(`/ui/destination", recall_source)
        self.assertNotIn("applyToggle", recall_source)

    def test_robot_commands_are_bound_to_backend_mission_generation(self) -> None:
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
        self.assertIn('"error": "stale_or_unowned_destination_stop"', self.backend_source)
        self.assertIn('"error": "stale_or_unowned_return"', self.backend_source)

    def test_robot_recall_rejects_late_http_and_orphan_socket_authority(self) -> None:
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

    def test_guest_owned_recall_exposes_only_bound_robot_completion(self) -> None:
        self.assertIn("const [missionDispatch, setMissionDispatch] = useState", self.source)
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
        self.assertIn("현재 임무의 복귀 권한을 확인하고 있습니다.", self.source)
        self.assertIn(
            "if (!dispatchActive || (dispatchOwner === 'guest' && dispatchIntent !== 'recall'))",
            self.source,
        )
        self.assertNotIn(
            "missionDispatchActiveRef.current = Boolean(newState)",
            self.source,
        )
        handler_start = self.source.index("const handleArrivalComplete = () => {")
        handler_end = self.source.index("// ── 실제 토글 적용", handler_start)
        handler = self.source[handler_start:handler_end]
        self.assertIn("if (!robotOwnsReturn)", handler)
        self.assertIn("현재 운행의 복귀 권한이 이 화면에 없습니다.", handler)
        self.assertNotIn("setIsReturning(true)", handler)
        self.assertIn("wsRef.current.readyState !== WebSocket.OPEN", handler)

    @unittest.skipUnless(shutil.which("node"), "Node.js is required for frontend behavior checks")
    def test_recall_return_progress_matches_both_uis_for_every_site(self) -> None:
        guest_source = (
            APP_SOURCE.parents[4] / "camrod_ui_guest" / "assets"
            / "guest_frontend" / "index.html"
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
            [f"B{site}", f"camping_site_maneuver_controller:{phase}:active"]
            for site in range(1, 14) for phase in phases
        ] + [["B3", "other_controller:ROTATE_180:active"], ["B3", ""]]
        outputs = []
        for source, end_marker in (
            (self.source, "// HH_260904 - Re-dock events"),
            (guest_source, "function updateUI()"),
        ):
            start = source.index("function recallReturnInstructions(site,")
            helpers = source[start:source.index(end_marker, start)]
            script = helpers + "\nprocess.stdout.write(JSON.stringify(" + json.dumps(inputs) + ".map(([site, description]) => ({instructions: recallReturnInstructions(site), ...recallReturnProgress(site, description)}))));"
            result = subprocess.run(
                ["node"], input=script, text=True, capture_output=True, check=True,
            )
            outputs.append(json.loads(result.stdout))
        self.assertEqual(outputs[0], outputs[1])
        for (site, description), output in zip(inputs, outputs[0]):
            with self.subTest(site=site, description=description):
                phase = description.split(":")[1] if ":" in description else ""
                if int(site[1:]) <= 10 and description.startswith("camping_site_maneuver_controller:"):
                    self.assertEqual(output["label"], phases[phase])
                    self.assertIn("180도", output["instructions"])
                else:
                    self.assertEqual(output["label"], "짐을 싣고 복귀 중")
                if int(site[1:]) >= 11:
                    self.assertIn("기존 반대쪽 경로", output["message"])
                    self.assertNotIn("180도", output["instructions"])

    @unittest.skipUnless(shutil.which("node"), "Node.js is required for frontend behavior checks")
    def test_robot_completion_keeps_guest_mission_identity_and_phase_guards(self) -> None:
        start = self.source.index("function robotCanCompleteMission(")
        helper = self.source[start:self.source.index("// HH_260904 - Re-dock events", start)]
        admitted = {"active": True, "site": "B4", "generation": 12, "owner": "guest", "intent": "recall"}
        cases = [
            [admitted, "B4", "GUEST_LOADING_WAIT"],
            [admitted, "B5", "GUEST_LOADING_WAIT"],
            [admitted, "B4", "RETURN_WITH_CARGO"],
            [{**admitted, "active": False}, "B4", "GUEST_LOADING_WAIT"],
            [{**admitted, "intent": "delivery"}, "B4", "GUEST_LOADING_WAIT"],
            [{**admitted, "generation": 0}, "B4", "GUEST_LOADING_WAIT"],
            [{**admitted, "generation": "12"}, "B4", "GUEST_LOADING_WAIT"],
            [{**admitted, "owner": "robot"}, "B4", "GUEST_LOADING_WAIT"],
            [{**admitted, "owner": "operator", "intent": "delivery"}, "B4", "UNLOAD_WAIT"],
        ]
        result = subprocess.run(
            ["node"], input=helper + "\nprocess.stdout.write(JSON.stringify(" + json.dumps(cases) + ".map(args => robotCanCompleteMission(...args))));",
            text=True, capture_output=True, check=True,
        )
        self.assertEqual(json.loads(result.stdout), [True, False, False, False, False, False, False, True, True])

    @unittest.skipUnless(shutil.which("node"), "Node.js is required for frontend behavior checks")
    def test_two_stage_completion_sends_explicit_stage_once_and_blocks_execution_failure(self) -> None:
        guest_source = (APP_SOURCE.parents[4] / "camrod_ui_guest" / "assets"
                        / "guest_frontend" / "index.html").read_text(encoding="utf-8")
        helper_start = self.source.index("function recallReturnInstructions(site,")
        helpers = self.source[helper_start:self.source.index("// HH_260904 - Re-dock events", helper_start)]
        robot_start = self.source.index("const handleArrivalComplete = () => {")
        robot_action = self.source[robot_start:self.source.index("// ── 실제 토글", robot_start)]
        guest_start = guest_source.index("function sendUsageComplete() {")
        guest_action = guest_source[guest_start:guest_source.index("function sendCancel()", guest_start)]
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
        result = subprocess.run(["node"], input=script, text=True, capture_output=True, check=True)
        output = json.loads(result.stdout)
        self.assertEqual(output["robotCalls"], [
            {"usage_complete": True, "site": "B4", "mission_generation": 12, "recall_final_return": stage}
            for stage in (False, True)
        ])
        self.assertEqual(output["guestCalls"], [
            {"action": "usage_complete", "recall_final_return": stage} for stage in (False, True)
        ])
        self.assertIn("관리자", output["warnings"][0])
        self.assertEqual(output["labels"], ["정리 완료 · 사이트 재진입", "짐 싣기 완료 · 복귀", "적재 완료 · 복귀"])

    @unittest.skipUnless(shutil.which("node"), "Node.js is required for frontend behavior checks")
    def test_robot_reconnect_restores_completion_and_preserves_minimal_phase_frames(self) -> None:
        # Use the actual initial backend frame and actual browser message
        # handler. A helper-only test cannot catch missing snapshot fields or
        # the extra minimal state frame erasing the preceding phase detail.
        endpoint = next(
            node for node in ast.walk(ast.parse(self.backend_source))
            if isinstance(node, ast.AsyncFunctionDef) and node.name == "websocket_endpoint"
        )
        snapshot = next(
            node.value.args[0] for node in ast.walk(endpoint)
            if isinstance(node, ast.Await)
            and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Attribute)
            and node.value.func.attr == "send_json"
            and node.value.args and isinstance(node.value.args[0], ast.Dict)
            and any(isinstance(key, ast.Constant) and key.value == "service_state_description"
                    for key in node.value.args[0].keys)
        )
        arrival_frame = eval(compile(ast.Expression(snapshot), "initial_robot_snapshot", "eval"), {}, {
            "service_state": 8,
            "service_state_name": "GUEST_LOADING_WAIT",
            "service_state_description": "camping_site_maneuver_controller:WAIT_RETURN:loading",
            "active_mission_site": "B4",
        })
        prefix = self.source[
            self.source.index("const SERVICE_STATE ="):
            self.source.index("// HH_260904 - Re-dock events")
        ]
        battery_helpers_start = self.source.index("const emptyBatteryReturnState =")
        prefix += self.source[battery_helpers_start:self.source.index(
            "function WaitingRuntimeStatusPanel(", battery_helpers_start
        )]
        handler_start = self.source.index("ws.onmessage = (event) => {")
        handler = self.source[handler_start:self.source.index(
            "// HH_260708 - Reconnect the operator WebSocket", handler_start
        )]
        setters = sorted(set(re.findall(r"\b(set[A-Z]\w*)\(", handler)))
        refs = sorted(set(re.findall(r"\b(\w+Ref)\.current", handler)) - {"wsRef"})
        setup = "\n".join(
            f"const {name} = value => {{uiState.{name} = typeof value === 'function' ? value(uiState.{name}) : value;}};"
            for name in setters
        ) + "\n" + "\n".join(f"const {name} = {{current: null}};" for name in refs)
        script = prefix + "\nconst uiState = {}; const ws = {}; const wsRef = {current: ws};\n" + setup + "\n" + r"""
const SITE_NAMES = Array.from({length: 13}, (_, i) => `B${i + 1}`);
const connectionGeneration = 1;
wsMountedRef.current = true;
wsGenerationRef.current = 1;
missionAuthorityRevisionRef.current = 0;
destinationIntentRef.current = 'delivery';
batteryReturnStateRef.current = emptyBatteryReturnState();
""" + handler + "\nconst send = frame => ws.onmessage({data: JSON.stringify(frame)});\n"
        script += "send(" + json.dumps(arrival_frame) + ");\n" + r"""
send({mission_dispatch_active: true, mission_dispatch_generation: 12,
      mission_dispatch_site: 'B4', mission_dispatch_owner: 'guest',
      mission_dispatch_intent: 'recall'});
const restored = {
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
const priorReplay = {restored, preserved, cleared: uiState.setServiceStateDescription,
  batteryReturn: uiState.setBatteryReturnState, parking: uiState.setParkingPolicy, failed, finalStage};
send({mission_dispatch_active: true, mission_dispatch_generation: 13,
      mission_dispatch_site: 'B1', mission_dispatch_owner: 'operator',
      mission_dispatch_intent: 'delivery'});
send({service_state: 11, service_state_name: 'WAITING_FOR_RETURN_REQUEST', site: 'B1'});
const completionState = () => ({
  arrived: uiState.setArrivedSite,
  modal: uiState.setShowArrivalComplete,
  permitted: robotCanCompleteMission(uiState.setMissionDispatch, uiState.setArrivedSite, uiState.setServiceStateName),
});
const beforeBatteryHeartbeat = completionState();
send({battery_return_pending: false});
const afterBatteryHeartbeat = completionState();
process.stdout.write(JSON.stringify({...priorReplay, beforeBatteryHeartbeat, afterBatteryHeartbeat}));
"""
        result = subprocess.run(["node"], input=script, text=True, capture_output=True, check=True)
        output = json.loads(result.stdout)
        self.assertEqual(output["restored"], {"arrived": "B4", "modal": True, "permitted": True})
        self.assertEqual(output["preserved"], "camping_site_maneuver_controller:RECALL_CLEARANCE_WAIT:active")
        self.assertEqual(output["cleared"], "")
        self.assertTrue(output["batteryReturn"]["pending"])
        self.assertTrue(output["batteryReturn"]["started"])
        self.assertFalse(output["batteryReturn"]["urgent"])
        self.assertEqual(output["parking"]["parking_selected_method"], "apriltag")
        self.assertTrue(output["parking"]["charging_required"])
        self.assertEqual(output["failed"], {"reason": "prepareRecallTurnaroundEntry failed", "modal": False})
        self.assertEqual(output["finalStage"], {"ready": True, "error": ""})
        expected_operator_wait = {"arrived": "B1", "modal": True, "permitted": True}
        self.assertEqual(output["beforeBatteryHeartbeat"], expected_operator_wait)
        self.assertEqual(output["afterBatteryHeartbeat"], expected_operator_wait)

    @unittest.skipUnless(shutil.which("node"), "Node.js is required for frontend behavior checks")
    def test_battery_messages_distinguish_urgent_return_and_normal_parking_boundaries(self) -> None:
        start = self.source.index("const emptyBatteryReturnState =")
        helpers = self.source[start:self.source.index("function WaitingRuntimeStatusPanel(", start)]
        guest_source = (APP_SOURCE.parents[4] / "camrod_ui_guest" / "assets"
                        / "guest_frontend" / "index.html").read_text(encoding="utf-8")
        guest_start = guest_source.index("function guestBatteryPolicyMessage(")
        guest_helper = guest_source[guest_start:guest_source.index("/* ── UI state machine", guest_start)]
        script = "const URGENT_BATTERY_RETURN_PERCENT = 25; const MISSION_DISPATCH_MINIMUM_PERCENT = 35;\n" + helpers + guest_helper + r"""
const values = [null, 20, 24.9, 25, 34.9, 35, 80];
const robot = values.map(value => batteryPolicyStatus(value, emptyBatteryReturnState()));
const guest = values.map(value => guestBatteryPolicyMessage(value, false, false, ''));
const urgent = formatBatteryReturnMessage({battery_percentage: 24, battery_return_urgent: true});
process.stdout.write(JSON.stringify({robot, guest, urgent}));
"""
        result = subprocess.run(["node"], input=script, text=True, capture_output=True, check=True)
        output = json.loads(result.stdout)
        self.assertIn("pending", output["robot"][0]["label"])
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

    @unittest.skipUnless(shutil.which("node"), "Node.js is required for frontend behavior checks")
    def test_explicit_dock_posts_only_dock_and_reports_backend_rejection(self) -> None:
        start = self.telemetry_source.index("export async function postDockingRequest(")
        helper = self.telemetry_source[start:self.telemetry_source.index("export function DockingCommandButton(", start)]
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
  const stationAllowed = ['DROP_ZONE_WAIT', 'WAITING_FOR_CHARGING', 'CHARGING', 'DROP_ZONE_PARKING', 'OPERATOR_STOPPED']
    .map(dockingAllowedAtServiceState);
  const awayBlocked = ['', 'PREPARING', 'GOING_TO_SITE', 'GUEST_LOADING_WAIT', 'RETURN_WITH_CARGO']
    .map(dockingAllowedAtServiceState);
  process.stdout.write(JSON.stringify({calls, accepted, error, reverse, april, stationAllowed, awayBlocked}));
})();
"""
        result = subprocess.run(["node"], input=script, text=True, capture_output=True, check=True)
        output = json.loads(result.stdout)
        self.assertEqual(output["calls"], [{"url": "/ui/dock", "options": {"method": "POST"}}] * 2)
        self.assertEqual(output["accepted"]["action"], "force_docking")
        self.assertEqual(output["error"], "CAN unavailable")
        self.assertIn("충전하지 않음", output["reverse"])
        # Pending/completed docking copy belongs to the lifecycle, not policy selection.
        self.assertEqual(output["april"], "")
        self.assertEqual(output["stationAllowed"], [True] * 5)
        self.assertEqual(output["awayBlocked"], [False] * 5)
        # Explicit docking is offered in the service menu, and once in diagnostics.
        self.assertEqual(self.source.count("<DockingCommandButton"), 1)
        self.assertIn("<DockingCommandButton", self.telemetry_source)

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
            "Lanelet parking point",
            "Exact lanelet point",
            "docking-path-approach",
        ):
            self.assertIn(token, self.telemetry_source)


if __name__ == "__main__":
    unittest.main()
