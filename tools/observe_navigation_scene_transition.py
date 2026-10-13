#!/usr/bin/env python3
"""HH_261007 - Bound passive scene-transition timing; never send robot/UI actions."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
from urllib.request import urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "camrod_ui/tools"))
from capture_driving_preview import Devtools

QUERY = """(() => {
 const root = document.querySelector('.driving-display');
 const host = document.querySelector('.ranger-navigation-scene');
 const canvas = host?.querySelector('canvas');
 const bounds = root?.getBoundingClientRect();
 let render = null;
 try { render = JSON.parse(canvas?.dataset.navigationState || 'null'); } catch (_) {}
 return {visible: Boolean(bounds?.width && bounds?.height),
   canvasPresent: Boolean(canvas), modelStatus: host?.dataset.modelStatus ?? null,
   renderedFrames: render?.renderedFrames ?? null,
   renderedTriangles: render?.renderTriangles ?? null,
   statusText: document.querySelector('.dd-scene')?.innerText?.slice(0, 180) ?? ''};
})()"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration-s", type=float, default=240)
    args = parser.parse_args()
    if not 0 < args.duration_s <= 300:
        parser.error("duration must be in (0,300] seconds")
    args.output.mkdir(parents=True, exist_ok=False)
    with urlopen("http://127.0.0.1:9224/json", timeout=3) as response:
        pages = json.load(response)
    page = next(p for p in pages if p.get("type") == "page"
                and p.get("url") == "http://127.0.0.1:8010/")
    client = Devtools(page["webSocketDebuggerUrl"])
    started = time.monotonic()
    previous = None
    transitions = []
    samples = 0
    max_gap = 0.0
    last_elapsed = None
    error = None
    try:
        with (args.output / "samples.jsonl").open("x", encoding="utf-8") as stream:
            while time.monotonic() - started < args.duration_s:
                row = client.evaluate(QUERY)
                elapsed = time.monotonic() - started
                row.update(utc=datetime.now(timezone.utc).isoformat(), elapsed_s=elapsed)
                if last_elapsed is not None:
                    max_gap = max(max_gap, elapsed - last_elapsed)
                last_elapsed = elapsed
                signature = (row["visible"], row["canvasPresent"],
                             row["modelStatus"], row["renderedFrames"] is not None)
                if signature != previous:
                    transitions.append(row)
                    print(json.dumps(row, ensure_ascii=False), flush=True)
                    previous = signature
                stream.write(json.dumps(row, ensure_ascii=False) + "\n")
                stream.flush()
                samples += 1
                time.sleep(0.05)
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        client.socket.close()
        report = {"read_only": True, "robot_or_ui_commands_sent": False,
                  "measurement": "DOM and existing navigation debug state; not pixel completeness",
                  "samples": samples, "elapsed_s": time.monotonic() - started,
                  "max_sample_gap_s": max_gap, "transitions": transitions, "error": error}
        with (args.output / "summary.json").open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()
