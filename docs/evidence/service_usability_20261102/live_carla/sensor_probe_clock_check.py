#!/usr/bin/env python3
"""HH_261102 - Bounded read-only clock/TF diagnosis; safe while ego moves, with no CARLA client or publishers."""
import argparse
import json
import math
from pathlib import Path
import time

from sensor_probe import CLOCK_DOMAINS, json_safe, source_timing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--seconds", type=float, default=3)
    args = parser.parse_args()
    if not args.run:
        print(json.dumps({"mode": "PLAN_ONLY_NO_CONNECTION", "seconds": args.seconds, "clock_contract": CLOCK_DOMAINS}))
        return 0
    if args.output is None or not math.isfinite(args.seconds) or not 1 <= args.seconds <= 10:
        parser.error("--run requires a new --output and 1..10 seconds")
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import PointCloud2, Range
    from avg_msgs.msg import AvgOccupancyGrid

    rclpy.init(args=[])
    node = rclpy.create_node("readonly_sensor_probe_clock_check", parameter_overrides=[Parameter("use_sim_time", value=False)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    sim_clock = {"seconds": None, "received": None}
    result = {"authority": "READ_ONLY_NO_ACTOR_NO_PUBLISH_NO_CONTROL", "observer_use_sim_time": False,
              "duration_s": args.seconds, "clock_contract": CLOCK_DOMAINS, "topics": {}}

    def receive_clock(message):
        sim_clock.update(seconds=message.clock.sec+message.clock.nanosec/1e9, received=time.monotonic())

    def receive(name, topic, message):
        stamp = message.header.stamp
        clock_age = time.monotonic()-sim_clock["received"] if sim_clock["received"] is not None else None
        timing = source_timing(name, stamp.sec+stamp.nanosec/1e9, time.time(), sim_clock["seconds"], clock_age)
        record = result["topics"].setdefault(name, {"topic": topic, "messages": 0, "source_ages_s": []})
        record["messages"] += 1
        if timing["source_age_s"] is not None:
            record["source_ages_s"].append(timing["source_age_s"])
        record["last_received_monotonic_s"] = time.monotonic()
        record["last_timing"] = timing
        record["frame"] = message.header.frame_id
        if isinstance(message, AvgOccupancyGrid):
            try:
                exact = buffer.lookup_transform("robot_center_link", message.header.frame_id, Time.from_msg(stamp))
                record["exact_stamp_tf"] = {"available": True, "stamp_s": exact.header.stamp.sec+exact.header.stamp.nanosec/1e9}
            except Exception as error:
                record["exact_stamp_tf"] = {"available": False, "error": str(error)}
            try:
                latest = buffer.lookup_transform("robot_center_link", message.header.frame_id, Time())
                latest_stamp = latest.header.stamp.sec+latest.header.stamp.nanosec/1e9
                record["latest_tf"] = {"stamp_s": latest_stamp, "wall_age_s": time.time()-latest_stamp}
            except Exception as error:
                record["latest_tf"] = {"error": str(error)}

    topics = [("raw_lidar", PointCloud2, "/carla/ego_vehicle/lidar_front"),
              ("raw_radar_front1", PointCloud2, "/carla/ego_vehicle/radar_front1"),
              ("filtered_lidar", PointCloud2, "/sensing/lidar/points_filtered"),
              ("range_front1", Range, "/sensing/radar/front1/range_ros"),
              ("radar_grid", AvgOccupancyGrid, "/sensing/cost_grid/radar"),
              ("guard_grid", AvgOccupancyGrid, "/sensing/cost_grid/lidar_front_guard")]
    subscriptions = [node.create_subscription(Clock, "/clock", receive_clock, qos_profile_sensor_data)]
    subscriptions += [node.create_subscription(kind, topic, lambda msg, name=name, topic=topic: receive(name, topic, msg), qos_profile_sensor_data)
                      for name, kind, topic in topics]
    try:
        with args.output.open("x", encoding="utf8") as stream:
            deadline = time.monotonic()+args.seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            for record in result["topics"].values():
                ages = record.pop("source_ages_s")
                record["source_age_range_s"] = [min(ages), max(ages)] if ages else None
                record["receipt_age_s"] = time.monotonic()-record["last_received_monotonic_s"]
            result["missing_topics"] = [name for name, _, _ in topics if name not in result["topics"]]
            json.dump(json_safe(result), stream, indent=2, allow_nan=False)
            stream.write("\n")
        print(json.dumps({"output": str(args.output), "missing_topics": result["missing_topics"]}))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
