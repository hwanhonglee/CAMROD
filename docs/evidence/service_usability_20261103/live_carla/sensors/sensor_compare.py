#!/usr/bin/env python3
"""HH_261103 - Reuse the original actual-data reducer for side-by-side counts, never synthesize a scene or braking claim."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys


LEGACY = Path(__file__).resolve().parents[3]/"service_usability_20261102/live_carla"
sys.path.insert(0, str(LEGACY))
from sensor_probe_results import summarize


LABELS = ["Run / actual case", "BBox gap cm", "Raw F1 min m", "Raw F2 min m", "Filtered points", "Radar H/N/U", "Guard H/N/U", "Both H/N/U"]


def comparison_row(run, case):
    def interval(value, digits=3):
        return "no return" if value is None else f"{value[0]:.{digits}f}–{value[1]:.{digits}f}"
    def counts(mode):
        return " / ".join(str(mode[key]) for key in ("HIT", "NO_HIT", "UNAVAILABLE"))
    return [run.replace("nearfield_actual_", "actual_")+"\n"+case["case"], f'{case["measured_body_gap_m"]*100:.4f}',
        interval(case["raw_front1_min_range_m"]), interval(case["raw_front2_min_range_m"]),
        interval(case["filtered_point_count_range"], 0), *(counts(case["modes"][mode]) for mode in ("radar_only", "guard_only", "both"))]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output-prefix", type=Path, required=True)
    args = parser.parse_args()
    outputs = {extension: args.output_prefix.with_suffix("."+extension) for extension in ("json", "csv", "png")}
    if len({path.resolve() for path in args.inputs}) != len(args.inputs):
        parser.error("each comparison run must be a distinct actual trace")
    if any(path.exists() for path in outputs.values()):
        parser.error("choose a new prefix; evidence is never overwritten")
    results = [summarize(path) for path in args.inputs]
    rows = [comparison_row(path.stem, case) for path, result in zip(args.inputs, results) for case in result["case_results"]]
    document = {"evidence_type": "ACTUAL_CARLA_SAVED_COUNTS_NOT_SCENE_RENDER",
        "interpretation": "STATIONARY SENSOR COMPARISON — NOT A BRAKING TEST; NO_HIT is not FREE space or safety certification.",
        "reducer_sha256": hashlib.sha256((LEGACY/"sensor_probe_results.py").read_bytes()).hexdigest(), "runs": results}
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(20, max(6, len(rows)*.48+3)))
    ax.axis("off")
    fig.suptitle("ACTUAL CARLA — STATIONARY NEAR-FIELD SENSOR COMPARISON", fontsize=17, fontweight="bold")
    table = ax.table(cellText=rows, colLabels=LABELS, loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 2.2)
    notes = ["H/N/U = high-cost HIT / NO_HIT / UNAVAILABLE. Real saved samples, not a scene render. NOT A BRAKING TEST."]
    for path, result in zip(args.inputs, results):
        baseline = "; ".join(mode+"="+"/".join(str(counts[k]) for k in ("HIT", "NO_HIT", "UNAVAILABLE")) for mode, counts in result["modes"].items())
        notes.append(f'{path.stem}: baseline {result["baseline_samples"]} samples; {baseline}; gate={",".join(result["gate_operating_states"])}')
        notes.append(f'  physical/native front difference={result["physical_minus_native_front_m"]:.4f} m; post-removal hit records={len(result["post_removal_transition_hits"])}; raw source SHA={result["source_sha256"][:12]}')
    fig.text(.03, .025, "\n".join(notes), fontsize=9)
    # HH_261103 - Exclusive creation prevents a report invocation from erasing any prior real evidence.
    with outputs["png"].open("xb") as target:
        fig.savefig(target, format="png", dpi=140, bbox_inches="tight")
    plt.close(fig)
    with outputs["csv"].open("x", newline="") as target:
        writer = csv.writer(target)
        writer.writerow(LABELS)
        writer.writerows(rows)
    with outputs["json"].open("x") as target:
        json.dump(document, target, ensure_ascii=False, indent=2)
        target.write("\n")
    print(json.dumps({"outputs": {key: str(value) for key, value in outputs.items()}, "cases": len(rows), "braking_claim": False}))


if __name__ == "__main__":
    main()
