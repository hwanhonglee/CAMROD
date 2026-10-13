#!/usr/bin/env python3
"""HH_261006 - Request a distinct local simulator recovery via visible Robot UI.

Never relabel an interrupted delivery as a completed roundtrip. This helper
requires inactive, unarmed control before administrator login and before click.
"""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts/virtual_carla"))
from camping_site_matrix import UIClient
from manual_4ws_evidence import ManualDriveBrowser


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("refusing to overwrite evidence")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ui = UIClient("http://127.0.0.1:8010")
    browser = ManualDriveBrowser("http://127.0.0.1:9224", "http://127.0.0.1:8010", timeout_s=15)
    record = {"action": "standalone_return_via_visible_UI", "full_roundtrip": False}
    try:
        before = ui.state()
        if before.get("mission_dispatch_active") is not False or browser.snapshot().get("armed"):
            raise RuntimeError("another mission or manual owner is active")
        if before.get("backend_startup_recovery") or (before.get("manual_resume") or {}).get("pending"):
            raise RuntimeError("startup/manual recovery already owns the operation")
        record["before"] = before
        record["login"] = browser.ensure_fresh_administrator_session()
        # HH_261006 - The return control belongs to System, not the camera tab
        # used by the shared login helper. Navigate by a real visible click.
        browser._click('[data-ui="operator-diagnostic-tab-system"]', "open System return controls")
        current = ui.state()
        if current.get("mission_dispatch_active") is not False or browser.snapshot().get("armed"):
            raise RuntimeError("control authority changed during login")
        record["request_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        browser._click('button[title="현재 서비스 상태와 관계없이 drop zone 복귀를 요청"]',
                       "request separate Drop Zone return and automatic parking")
        record["interactions"] = list(browser._interactions)
        record["after"] = ui.state()
        record["request_clicked"] = True
        record["parking_confirmed"] = False
    except Exception as error:
        record["error"] = str(error)
        raise
    finally:
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(record, stream, ensure_ascii=False, indent=2)
        browser.close()
    print(json.dumps({"request_clicked": True, "output": str(args.output), "parking_confirmed": False}))


if __name__ == "__main__":
    main()
