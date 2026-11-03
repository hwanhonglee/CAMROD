#!/usr/bin/env python3
"""HH_261102 - Reuse production CDP input helpers with the new charging screen.

Inspect is passive; click/type/dispatch act only when explicitly selected.
No connection is made on import. The historical helper is loaded, never edited.
"""

import argparse
import base64
import importlib.util
import json
from pathlib import Path
import sys
import time
from urllib.parse import urlsplit
from urllib.request import Request, urlopen


MATRIX_PATH = Path("/home/hong/camrod_ws/src/scripts/virtual_carla/camping_site_matrix.py")
spec = importlib.util.spec_from_file_location("live_carla_original_matrix", MATRIX_PATH)
matrix = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = matrix
spec.loader.exec_module(matrix)


def local_url(value, scheme="http", origin=False):
    parsed = urlsplit(value)
    if (parsed.scheme != scheme or parsed.hostname not in {"127.0.0.1", "localhost"}
            or parsed.username or parsed.password or parsed.query or parsed.fragment
            or (origin and parsed.path not in {"", "/"})):
        raise matrix.MatrixError("browser endpoints must be credential-free localhost URLs")
    return value.rstrip("/")


def select_target(targets, ui_url):
    ui_url = local_url(ui_url, origin=True)
    if not isinstance(targets, list):
        raise matrix.MatrixError("CDP /json response must be a list")
    matches = [item for item in targets if isinstance(item, dict) and item.get("type") == "page"
               and str(item.get("url", "")).rstrip("/") == ui_url]
    if len(matches) != 1:
        raise matrix.MatrixError(f"expected one Robot UI page at {ui_url}, found {len(matches)}")
    local_url(str(matches[0].get("webSocketDebuggerUrl", "")), scheme="ws")
    return dict(matches[0])


def validate_page(ready, ui_url):
    if (not isinstance(ready, dict) or ready.get("ready") != "complete"
            or str(ready.get("href", "")).rstrip("/") != ui_url.rstrip("/")
            or ready.get("title") != matrix.OPERATOR_UI_TITLE
            or ready.get("visibility") != "visible" or ready.get("rootReady") is not True
            or ready.get("screenReady") is not True):
        raise matrix.MatrixError(f"visible production Robot UI is not ready: {ready!r}")


class LiveOperatorBrowser(matrix.OperatorBrowserClient):
    def _connect_operator(self):
        # HH_261102 - Admit the authored charging root without weakening page identity or pointer checks.
        local_url(self.debugging_url, origin=True)
        local_url(self.operator_ui_url, origin=True)
        request = Request(self.debugging_url + "/json", method="GET")
        with urlopen(request, timeout=self.timeout_s) as response:
            raw = response.read(1048577)
        if len(raw) > 1048576:
            raise matrix.MatrixError("CDP target response exceeds 1 MiB")
        self._target = select_target(json.loads(raw), self.operator_ui_url)
        import websocket
        self._connection = websocket.create_connection(self._target["webSocketDebuggerUrl"],
            timeout=self.timeout_s, origin=self.operator_ui_url)
        self._call("Page.bringToFront", {})
        ready = self._evaluate("""(() => ({title:document.title,href:location.href,
            ready:document.readyState,visibility:document.visibilityState,
            rootReady:Boolean(document.getElementById('root')),
            screenReady:Boolean(document.querySelector('[data-ui="charging-screen"],
                [data-ui="operator-waiting-screen"], [data-ui="operator-service-selection-screen"],
                [data-ui="operator-control-screen"]'))}))()""".replace("\n", " "))
        validate_page(ready, self.operator_ui_url)

    def open_service_menu(self):
        # HH_261102 - Leave charging through its real menu button and retain the safety acknowledgement.
        if self._element('[data-ui="charging-screen"]').get("visibleCount") == 1:
            self._click('[data-ui="charging-open-services"]', "open services from charging")
            deadline = time.monotonic() + self.timeout_s
            while time.monotonic() < deadline:
                if self._element('[data-ui="service-safety-gate"]').get("visibleCount") == 1:
                    self._click('[data-ui="usage-safety-confirm"]', "acknowledge safety notice")
                    self._wait_element('[data-ui="operator-service-selection-screen"]', "service selection screen")
                    break
                if self._element('[data-ui="operator-service-selection-screen"]').get("visibleCount") == 1:
                    break
                time.sleep(0.05)
            else:
                raise matrix.MatrixError("charging services did not show the safety gate or service menu")
        return super().open_service_menu()

    def inspect(self):
        # HH_261102 - Inspect rendered controls without invoking application callbacks or backend fetches.
        return self._evaluate("""(() => ({title:document.title,url:location.href,
            chargingPhase:document.querySelector('[data-ui="charging-screen"]')?.dataset.phase ?? null,
            text:document.body.innerText.slice(0,2000),
            buttons:Array.from(document.querySelectorAll('button,input')).filter(el=>{
                const r=el.getBoundingClientRect(),s=getComputedStyle(el);
                return r.width>0&&r.height>0&&s.display!=='none'&&s.visibility!=='hidden';
            }).slice(0,80).map(el=>({tag:el.tagName,dataUi:el.dataset.ui??null,
                text:(el.innerText||el.placeholder||'').slice(0,120),disabled:Boolean(el.disabled),
                value:el.tagName==='INPUT'?el.value:null}))}))()""")

    def screenshot(self, path):
        # HH_261102 - Preserve the visible page as a new PNG, never replace an earlier capture.
        with Path(path).open("xb") as output:
            result = self._call("Page.captureScreenshot", {"format": "png", "fromSurface": True})
            png = base64.b64decode(result["result"]["data"], validate=True)
            if not png.startswith(b"\x89PNG\r\n\x1a\n"):
                raise matrix.MatrixError("CDP screenshot did not contain PNG data")
            output.write(png)
        return {"path": str(Path(path).resolve()), "bytes": len(png)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cdp-url", default="http://127.0.0.1:9224")
    parser.add_argument("--ui-url", default="http://127.0.0.1:8010")
    actions = parser.add_subparsers(dest="action", required=True)
    actions.add_parser("inspect")
    click = actions.add_parser("click")
    click.add_argument("selector")
    typing = actions.add_parser("type")
    typing.add_argument("selector")
    typing.add_argument("text")
    screenshot = actions.add_parser("screenshot")
    screenshot.add_argument("path", type=Path)
    dispatch = actions.add_parser("dispatch")
    dispatch.add_argument("site", choices=matrix.DEFAULT_SITES)
    dispatch.add_argument("intent", choices=("delivery", "recall"))
    args = parser.parse_args()
    client = LiveOperatorBrowser(args.cdp_url, args.ui_url)
    try:
        if args.action == "inspect":
            result = client.inspect()
        elif args.action == "click":
            result = client._click(args.selector, "explicit operator-selected control")
        elif args.action == "type":
            result = client._insert_text(args.selector, args.text)
        elif args.action == "screenshot":
            result = client.screenshot(args.path)
        else:
            result = client.dispatch(args.site, args.intent)
        print(json.dumps(result, ensure_ascii=False))
    finally:
        client.close()


if __name__ == "__main__":
    main()
