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
MANUAL_DRIVE_SOURCE = APP_SOURCE.with_name("ManualDrivePanel.js")
SERVICE_EVIDENCE_SOURCE = APP_SOURCE.with_name("ServiceEvidence.js")
PUBLIC_ASSETS = APP_SOURCE.parents[1] / "public"


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
            "const hasFreshDockingDebug =",
            "dockingCamera.available === true",
            "dockingSource.state === 'live'",
            "hasFreshDockingDebug ? 'docking' : 'rear'",
            'source="camera.rear" label="CARLA rear fallback"',
            "Docking camera · CARLA rear fallback",
            "camera={dockingCameraName}",
            "DockingPathPlot",
            "tag_detected",
            "is_charging",
        ):
            self.assertIn(token, self.telemetry_source)
        self.assertIn("requestManualReturn", self.source)
        # HH_260819 - The obsolete Parking ON/OFF switch must not bypass the
        # state-aware Return command or appear in either operator surface.
        for removed in ("manual_parking", "toggleManualParking", "Parking OFF"):
            self.assertNotIn(removed, self.source)
            self.assertNotIn(removed, self.telemetry_source)
        self.assertIn(".docking-layout", self.css)

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
            "serviceActive={Boolean(activeSite || arrivedSite || "
            "displayedReturning)}",
            self.source,
        )
        self.assertIn("이번 서비스", self.service_evidence_source)
        self.assertIn(".evidence-trip-badge", self.css)

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


if __name__ == "__main__":
    unittest.main()
