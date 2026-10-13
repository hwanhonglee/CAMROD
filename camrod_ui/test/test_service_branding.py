"""HH_261007 - Public branding must not rename runtime or asset contracts."""

from html.parser import HTMLParser
from pathlib import Path


UI_ROOT = Path(__file__).resolve().parents[1]
FRONTEND = UI_ROOT / "camrod_ui_robot/assets/frontend"


class _VisibleHtmlText(HTMLParser):
    def __init__(self):
        super().__init__()
        self.hidden_depth = 0
        self.text = []

    def handle_starttag(self, tag, attrs):
        if tag in {"script", "style"}:
            self.hidden_depth += 1

    def handle_endtag(self, tag):
        if tag in {"script", "style"}:
            self.hidden_depth -= 1

    def handle_data(self, data):
        if not self.hidden_depth:
            self.text.append(data)


def test_guest_branding_preserves_park_logo_and_runtime_title_contract():
    source = (UI_ROOT / "camrod_ui_guest/assets/guest_frontend/index.html").read_text()
    parser = _VisibleHtmlText()
    parser.feed(source)
    text = " ".join(parser.text)
    assert "CAM-Geni" in text
    assert "AVGenius" in text
    assert "월악산 국립공원" in text
    assert "CAMROD" not in text
    assert 'src="/월악산_국립공원_로고.jpg"' in source
    assert "<title>Robot UI</title>" in (FRONTEND / "public/index.html").read_text()
    assert (FRONTEND / "public/models/ranger-navigation.glb").is_file()


def test_brand_module_is_presentation_only_and_reused_by_public_screens():
    source = (FRONTEND / "src/serviceBranding.js").read_text()
    assert "export const SERVICE_BRAND = 'CAM-Geni'" in source
    assert "export const SERVICE_COMPANY = 'AVGenius'" in source
    for name in ("App.js", "DrivingDisplay.js", "DrivingPreview.js", "ServiceEvidence.js"):
        assert "from './serviceBranding'" in (FRONTEND / "src" / name).read_text()
    assert "camrodNavigationGraphics" in (FRONTEND / "src/navigationGraphicsLease.js").read_text()
