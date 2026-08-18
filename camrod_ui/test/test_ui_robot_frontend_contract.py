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
PUBLIC_ASSETS = APP_SOURCE.parents[1] / "public"


class RobotUiFrontendContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.source = APP_SOURCE.read_text(encoding="utf-8")
        cls.css = APP_CSS.read_text(encoding="utf-8")
        cls.telemetry_source = TELEMETRY_SOURCE.read_text(encoding="utf-8")

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
        # HH_260819 - The obsolete Parking ON/OFF switch must not bypass the
        # state-aware Return command or appear in either operator surface.
        for removed in ("manual_parking", "toggleManualParking", "Parking OFF"):
            self.assertNotIn(removed, self.source)
            self.assertNotIn(removed, self.telemetry_source)
        self.assertIn(".docking-layout", self.css)


if __name__ == "__main__":
    unittest.main()
