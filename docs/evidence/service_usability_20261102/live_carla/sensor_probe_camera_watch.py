#!/usr/bin/env python3
"""HH_261102 - Save unmodified real front-camera JPEGs beside fresh placement receipts; never foreground a UI or publish."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=150)
    args = parser.parse_args()
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    rclpy.init(args=[])
    node = rclpy.create_node("readonly_sensor_probe_camera", parameter_overrides=[Parameter("use_sim_time", value=False)])
    latest = {}

    def receive(message):
        latest.update(data=bytes(message.data), received=time.monotonic(),
            stamp_s=message.header.stamp.sec+message.header.stamp.nanosec/1e9, frame=message.header.frame_id)

    subscription = node.create_subscription(CompressedImage, "/sensing/camera/econ_front/image_rect/compressed", receive, qos_profile_sensor_data)
    try:
        with args.input.open() as source, args.output.open("x") as manifest:
            deadline = time.monotonic()+args.seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                line = source.readline()
                if not line or not line.endswith("\n"):
                    if line:
                        source.seek(source.tell()-len(line))
                    continue
                event = json.loads(line)
                if event.get("event") == "complete":
                    break
                if event.get("event") not in ("measured_placement", "publishers_after_baseline"):
                    continue
                case = event.get("case", "baseline_end")
                record = {"case": case, "receipt": event, "source_clock": "wall", "source_topic": "/sensing/camera/econ_front/image_rect/compressed"}
                event_age = time.time()-datetime.fromisoformat(event["utc"]).timestamp()
                fresh = latest and 0 <= time.monotonic()-latest["received"] <= .5 and 0 <= time.time()-latest["stamp_s"] <= .5
                if 0 <= event_age <= 1 and fresh and latest["data"].startswith(b"\xff\xd8"):
                    target = args.output.with_name(args.output.stem+"_"+case+"_front.jpg")
                    with target.open("xb") as image:
                        image.write(latest["data"])
                    record.update(image=str(target), image_sha256=hashlib.sha256(latest["data"]).hexdigest(),
                        source_stamp_s=latest["stamp_s"], source_age_s=time.time()-latest["stamp_s"], frame=latest["frame"])
                else:
                    record.update(image=None, reason="fresh JPEG/placement receipt unavailable", receipt_age_s=event_age)
                manifest.write(json.dumps(record)+"\n")
                manifest.flush()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
