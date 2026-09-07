"""Source-level regression checks for critical Robot UI operator flows."""

from pathlib import Path
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
            "Recall · 도로 대기",
            "배송 · 사이트 진입",
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

    def test_guest_owned_recall_does_not_expose_robot_return(self) -> None:
        self.assertIn("const [missionDispatch, setMissionDispatch] = useState", self.source)
        self.assertIn("const robotOwnsReturn =", self.source)
        self.assertIn("const guestOwnsReturn =", self.source)
        self.assertIn(
            "['operator', 'robot'].includes(missionDispatch.owner)",
            self.source,
        )
        self.assertIn("{robotOwnsReturn ? (", self.source)
        self.assertIn(
            "이 임무의 완료·복귀는 이용객 화면에서 진행합니다.",
            self.source,
        )
        self.assertIn("현재 임무의 복귀 권한을 확인하고 있습니다.", self.source)
        self.assertIn("if (!dispatchActive || dispatchOwner === 'guest')", self.source)
        self.assertNotIn(
            "missionDispatchActiveRef.current = Boolean(newState)",
            self.source,
        )
        handler_start = self.source.index("const handleArrivalComplete = () => {")
        handler_end = self.source.index("// ── 실제 토글 적용", handler_start)
        handler = self.source[handler_start:handler_end]
        self.assertIn("if (!robotOwnsReturn)", handler)
        self.assertIn("현재 운행의 복귀 권한이 이 화면에 없습니다.", handler)

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
