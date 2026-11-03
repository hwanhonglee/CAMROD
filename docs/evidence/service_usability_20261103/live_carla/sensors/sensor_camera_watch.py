#!/usr/bin/env python3
"""HH_261103 - Capture only fresh real camera frames after measured actor placement; never command the world."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time


TOPIC = "/sensing/camera/econ_front/image_rect/compressed"
CASES = {"baseline_end", "front_10cm", "front_20cm", "front_30cm", "front_40cm", "roadside_left", "roadside_right"}


def drain_receipts(source):
    # HH_261103 - Drain the backlog in one spin, retaining an unfinished append for the next spin.
    events = []
    while True:
        position = source.tell()
        line = source.readline()
        if not line.endswith("\n"):
            source.seek(position)
            return events
        row = json.loads(line)
        if row.get("event") in ("measured_placement", "publishers_after_baseline", "owned_actor_removed", "complete"):
            events.append(row)


def capture_status(event, image, wall_s, monotonic_s):
    event_s = datetime.fromisoformat(event["utc"]).timestamp()
    age = wall_s-event_s
    if not math.isfinite(age) or age < 0:
        return "unavailable: future/invalid placement receipt"
    if age > 3:
        return "unavailable: placement photo window expired"
    if not image:
        return "waiting"
    stamp, receipt_age = image["stamp_s"], monotonic_s-image["received"]
    # HH_261103 - A pre-placement cached image is not a photograph of this case; allow half a second to settle.
    if (not math.isfinite(stamp) or stamp < event_s+.5 or not 0 <= wall_s-stamp <= .5
            or not 0 <= receipt_age <= .5 or not image["data"].startswith(b"\xff\xd8")):
        return "waiting"
    return "capture"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--seconds", type=float, default=180)
    args = parser.parse_args()
    if not args.run:
        print(json.dumps({"mode": "PLAN_ONLY_NO_CONNECTION", "topic": TOPIC,
            "settle_seconds": .5, "maximum_photo_window_seconds": 3, "publishes": False}))
        return
    if (not args.input or not args.output or args.input.resolve() == args.output.resolve()
            or args.output.exists() or not math.isfinite(args.seconds) or not 1 <= args.seconds <= 600):
        parser.error("choose distinct input/new output and 1..600 seconds")
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    rclpy.init(args=[])
    node = rclpy.create_node("readonly_sensor_case_camera", parameter_overrides=[Parameter("use_sim_time", value=False)])
    latest, pending, seen = {}, {}, set()

    def receive(message):
        latest.update(data=bytes(message.data), received=time.monotonic(),
            stamp_s=message.header.stamp.sec+message.header.stamp.nanosec/1e9, frame=message.header.frame_id)

    subscription = node.create_subscription(CompressedImage, TOPIC, receive, qos_profile_sensor_data)
    source = None
    try:
        with args.output.open("x") as manifest:
            def record(row):
                manifest.write(json.dumps({"utc": datetime.now(timezone.utc).isoformat(), **row})+"\n")
                manifest.flush()

            def finish(case, reason):
                record({"case": case, "receipt": pending.pop(case), "image": None, "reason": reason})

            record({"event": "watcher_ready", "source_topic": TOPIC, "source_clock": "wall",
                "evidence_type": "ACTUAL_CAMERA_NOT_UI_FIXTURE", "publishes": False})
            print(json.dumps({"event": "watcher_ready", "manifest": str(args.output)}), flush=True)
            deadline, complete = time.monotonic()+args.seconds, False
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                if source is None and args.input.exists():
                    source = args.input.open()
                for event in drain_receipts(source) if source else ():
                    if event["event"] == "complete":
                        complete = True
                    elif event["event"] == "owned_actor_removed":
                        for case in list(pending):
                            if pending[case].get("actor_id") == event["actor_id"]:
                                finish(case, "actor removed before a fresh settled frame")
                    else:
                        case = event.get("case", "baseline_end")
                        if case not in CASES or case in seen:
                            raise ValueError("unexpected or duplicate case receipt: "+str(case))
                        pending[case], seen = event, seen | {case}
                for case, event in list(pending.items()):
                    status = capture_status(event, latest, time.time(), time.monotonic())
                    if status == "capture":
                        target = args.output.with_name(args.output.stem+"_"+case+"_front.jpg")
                        with target.open("xb") as image:
                            image.write(latest["data"])
                        record({"case": case, "receipt": pending.pop(case), "image": str(target),
                            "image_sha256": hashlib.sha256(latest["data"]).hexdigest(), "source_topic": TOPIC,
                            "source_clock": "wall", "source_stamp_s": latest["stamp_s"],
                            "source_age_s": time.time()-latest["stamp_s"],
                            "receipt_age_s": time.monotonic()-latest["received"], "frame": latest["frame"]})
                    elif status.startswith("unavailable:"):
                        finish(case, status)
                if complete and not pending:
                    break
            for case in list(pending):
                finish(case, "watcher deadline before a fresh settled frame")
            record({"event": "watcher_complete", "probe_complete_received": complete, "cases_seen": sorted(seen)})
    finally:
        if source:
            source.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
