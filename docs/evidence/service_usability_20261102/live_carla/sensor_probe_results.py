#!/usr/bin/env python3
"""HH_261102 - Derive a counts-only report/PNG from unchanged actual JSONL; never render imagined obstacle geometry."""
import argparse
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path

from sensor_probe import shadow_grid_hit


def summarize(path):
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    baseline = [row for row in rows if row.get("event") == "sample" and row.get("case") == "no_test_actor_ground_roadside_baseline"]
    result = {"source": str(path), "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "evidence_type": "ACTUAL_CARLA_SAVED_SAMPLE_COUNTS_NOT_SCENE_RENDER", "baseline_samples": len(baseline),
        "baseline_first_utc": baseline[0]["utc"], "baseline_last_utc": baseline[-1]["utc"],
        "carla_frames": [baseline[0]["carla_frame"], baseline[-1]["carla_frame"]],
        "completed": any(row.get("event") == "complete" for row in rows),
        "measured_placements": [row for row in rows if row.get("event") == "measured_placement"],
        "actor_receipts": [row for row in rows if row.get("event") in ("owned_actor_spawned", "owned_actor_removed")],
        "exact_ground_rejection_retained": any(row.get("event") == "placement_ground_check" and row.get("rejection_phase") for row in rows),
        "interpretation": "Retrospective fresh high-cost-cell HIT/NO_HIT; NO_HIT is not proof of free space, final-gate isolation or successful stopping.",
        "modes": {mode: {"HIT": 0, "NO_HIT": 0, "UNAVAILABLE": 0} for mode in ("radar_only", "guard_only", "both")}}
    for row in baseline:
        radar = shadow_grid_hit(row.get("sensors", {}).get("radar_grid", {}))
        guard = shadow_grid_hit(row.get("sensors", {}).get("guard_grid", {}))
        both = radar or guard if radar is not None and guard is not None else None
        for mode, hit in (("radar_only", radar), ("guard_only", guard), ("both", both)):
            result["modes"][mode]["UNAVAILABLE" if hit is None else "HIT" if hit else "NO_HIT"] += 1
    result["topics"] = {}
    for name in ("radar_grid", "guard_grid", "raw_lidar", "filtered_lidar", "raw_radar_front1", "raw_radar_front2"):
        samples = [row.get("sensors", {}).get(name, {}) for row in baseline]
        metrics = [sample["value"] for sample in samples if "value" in sample]
        def bounds(values):
            return [min(values), max(values)] if values else None
        result["topics"][name] = {"samples_with_metrics": len(metrics), "error_samples": sum("error" in sample for sample in samples),
            "point_count_range": bounds([value["points"] for value in metrics if "points" in value]),
            "occupied_grid_cell_range": bounds([value["occupied_cells_ge85"] for value in metrics if "occupied_cells_ge85" in value]),
            "unknown_grid_cell_range": bounds([value["unknown_cells"] for value in metrics if "unknown_cells" in value]),
            "source_age_range_s": bounds([sample["clock_now"]["source_age_s"] for sample in samples
                                         if sample.get("clock_now", {}).get("source_age_s") is not None])}
    result["case_results"] = []
    for placement in result["measured_placements"]:
        samples = [row for row in rows if row.get("event") == "sample" and row.get("case") == placement["case"]]
        def metrics(name):
            return [row["sensors"][name]["value"] for row in samples if "value" in row.get("sensors", {}).get(name, {})]
        def field_range(name, field):
            values = [value["fields"][field]["min"] for value in metrics(name)
                      if value.get("fields", {}).get(field, {}).get("min") is not None]
            return [min(values), max(values)] if values else None
        modes = {mode: {"HIT": 0, "NO_HIT": 0, "UNAVAILABLE": 0} for mode in ("radar_only", "guard_only", "both")}
        for sample in samples:
            radar = shadow_grid_hit(sample["sensors"].get("radar_grid", {}))
            guard = shadow_grid_hit(sample["sensors"].get("guard_grid", {}))
            both = (radar or guard) if radar is not None and guard is not None else None
            for mode, hit in (("radar_only", radar), ("guard_only", guard), ("both", both)):
                modes[mode]["UNAVAILABLE" if hit is None else "HIT" if hit else "NO_HIT"] += 1
        filtered = [value["points"] for value in metrics("filtered_lidar")]
        result["case_results"].append({"case": placement["case"], "actor_id": placement["actor_id"],
            "placement_utc": placement["utc"], "measured_body_gap_m": placement["measured_front_body_gap_m"],
            "samples": len(samples), "raw_front1_min_range_m": field_range("raw_radar_front1", "Range"),
            "raw_front2_min_range_m": field_range("raw_radar_front2", "Range"),
            "filtered_point_count_range": [min(filtered), max(filtered)] if filtered else None,
            "filtered_x_min_range_m": field_range("filtered_lidar", "x"),
            "occupied_cell_ranges": {name: bounds([value["occupied_cells_ge85"] for value in metrics(name)])
                                     for name in ("radar_grid", "guard_grid")}, "modes": modes})
    result["gate_operating_states"] = sorted({row["sensors"]["gate"]["value"]["operating_state"] for row in rows
        if row.get("event") == "sample" and "value" in row.get("sensors", {}).get("gate", {})})
    result["physical_minus_native_front_m"] = rows[0]["physical_bbox_projected_m"]["forward"][1]-rows[0]["native_guard_body_m"]["front"]
    result["post_removal_transition_hits"] = []
    last_removal = None
    for row in rows:
        if row.get("event") == "owned_actor_removed":
            last_removal = row["utc"]
        if row.get("event") == "sample" and row.get("case", "").startswith("no_owned_actor_before_"):
            hits = {name: shadow_grid_hit(row["sensors"].get(name, {})) for name in ("radar_grid", "guard_grid")}
            if any(value is True for value in hits.values()):
                result["post_removal_transition_hits"].append({"case": row["case"], "utc": row["utc"], "hits": hits,
                    "seconds_after_removal": (datetime.fromisoformat(row["utc"])-datetime.fromisoformat(last_removal)).total_seconds() if last_removal else None})
    camera = path.with_name(path.stem+"_camera.jsonl")
    result["camera_receipts"] = [json.loads(line) for line in camera.read_text().splitlines() if line.strip()] if camera.exists() else []
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--png", type=Path, required=True)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()
    if args.output.exists() or args.png.exists() or (args.csv and args.csv.exists()):
        parser.error("choose new output/PNG paths; existing evidence is never overwritten")
    result = summarize(args.input)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    has_cases = bool(result["case_results"])
    fig, ax = plt.subplots(figsize=(18, 7) if has_cases else (12, 5))
    ax.axis("off")
    modes = result["modes"]
    if has_cases:
        fig.suptitle("ACTUAL CARLA — MEASURED BODY-GAP SENSOR COMPARISON (2026-11-02 UTC)", fontsize=17, fontweight="bold")
        def cell_range(value, digits=3):
            return "no return" if value is None else f"{value[0]:.{digits}f}–{value[1]:.{digits}f}"
        def mode_cell(mode):
            return f'{mode["HIT"]} / {mode["NO_HIT"]} / {mode["UNAVAILABLE"]}'
        cells = [[case["case"], f'{case["measured_body_gap_m"]*100:.4f}',
                  cell_range(case["raw_front1_min_range_m"]), cell_range(case["raw_front2_min_range_m"]),
                  cell_range(case["filtered_point_count_range"], 0),
                  mode_cell(case["modes"]["radar_only"]), mode_cell(case["modes"]["guard_only"]), mode_cell(case["modes"]["both"])]
                 for case in result["case_results"]]
        labels = ["Actual case", "BBox gap cm", "Raw F1 min m", "Raw F2 min m", "Filtered points", "Radar H/N/U", "Guard H/N/U", "Both H/N/U"]
        table = ax.table(cellText=cells, colLabels=labels, loc="center", cellLoc="center")
        note = "H/N/U = high-cost HIT / NO_HIT / UNAVAILABLE. Gate was already FAULT_HOLD/estop: NOT a braking test.\n"
        note += f'Physical front − native guard front = {result["physical_minus_native_front_m"]:.4f} m; actor BBox gap is not radar/mesh-return range.\n'
        note += "60 s no-test-actor baseline: no high-cost hits; both roadside controls: no front high-cost hits.\n"
        note += "One transient guard hit immediately after 10 cm actor removal is retained in JSON. Fresh camera: roadside_right only; other photos unavailable."
    else:
        fig.suptitle("ACTUAL CARLA BASELINE — 60 s / " + str(result["baseline_samples"]) + " samples", fontsize=17, fontweight="bold")
        cells = [[name, value["HIT"], value["NO_HIT"], value["UNAVAILABLE"]] for name, value in modes.items()]
        labels = ["Read-only shadow mode", "HIT", "NO_HIT", "UNAVAILABLE"]
        table = ax.table(cellText=cells, colLabels=labels, loc="center", cellLoc="center")
        note = "Near-obstacle 10/20/30/40 cm: NOT TESTED — placement aborted before first case.\n"
        note += "NO_HIT = no observed high-cost front cell; NOT free-space or stopping certification.\n"
        note += "Guard exact-TF gaps are included as UNAVAILABLE. This figure plots saved counts, not scene imagery."
    table.auto_set_font_size(False)
    table.set_fontsize(11 if has_cases else 13)
    table.scale(1, 2.4)
    fig.text(.04, .08 if has_cases else .12, note, fontsize=11)
    fig.savefig(args.png, dpi=140, bbox_inches="tight")
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2)
        stream.write("\n")
    if args.csv:
        with args.csv.open("x", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(labels)
            writer.writerows(cells)
    plt.close(fig)
    print(json.dumps({"output": str(args.output), "png": str(args.png), "modes": modes}))


if __name__ == "__main__":
    main()
