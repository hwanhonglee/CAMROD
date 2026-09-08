#!/usr/bin/env python3
"""Offline derivative of existing observations; not a new motion experiment.

Choose the earliest entire truth-only rotation episode with >=90deg net yaw,
<=.15m center path and <=.5s truth gaps. Then keep its FIRST actual sample
crossing 90deg. GNSS/estimation errors are not read by this selection policy.
"""
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
TRACE = HERE.parent / "observations.jsonl"
OBSERVER = HERE.parent.parent / "observe_gnss_rotation.py"
TITLE = "First 90.42 deg of a recorded 178 deg turn — NOT a 90 deg command-and-stop test"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def delta(a,b):
    return math.atan2(math.sin(b["yaw_rad"]-a["yaw_rad"]),
                      math.cos(b["yaw_rad"]-a["yaw_rad"]))


def path_length(points):
    return sum(math.hypot(b["x"]-a["x"],b["y"]-a["y"]) for a,b in zip(points,points[1:]))


def descriptor(points):
    return {
        "start": {k:points[0][k] for k in ("received_utc","received_monotonic_s","stamp_s","source_line_1based","truth_index_0based")},
        "end": {k:points[-1][k] for k in ("received_utc","received_monotonic_s","stamp_s","source_line_1based","truth_index_0based")},
        "samples":len(points),
        "duration_s":points[-1]["received_monotonic_s"]-points[0]["received_monotonic_s"],
        "net_yaw_deg":math.degrees(sum(delta(a,b) for a,b in zip(points,points[1:]))),
        "angular_travel_deg":math.degrees(sum(abs(delta(a,b)) for a,b in zip(points,points[1:]))),
        "center_path_m":path_length(points),
        "center_end_displacement_m":math.hypot(points[-1]["x"]-points[0]["x"],points[-1]["y"]-points[0]["y"]),
        "maximum_gap_s":max(b["received_monotonic_s"]-a["received_monotonic_s"] for a,b in zip(points,points[1:])),
    }


def main():
    original_lines=TRACE.read_bytes().splitlines(keepends=True)
    rows=[dict(json.loads(line),source_line_1based=i+1) for i,line in enumerate(original_lines)]
    truth=[dict(r,truth_index_0based=i) for i,r in enumerate(r for r in rows if r["kind"]=="truth" and "invalid" not in r)]
    assert all(r["frame"]=="map" for r in truth)
    active=[]
    for i in range(1,len(truth)):
        dt=truth[i]["received_monotonic_s"]-truth[i-1]["received_monotonic_s"]
        if 0<dt<=.5 and abs(math.degrees(delta(truth[i-1],truth[i]))/dt)>1.0:
            if not active or truth[i]["received_monotonic_s"]-truth[active[-1][-1]]["received_monotonic_s"]>1.0:
                active.append([i])
            else:
                active[-1].append(i)
    candidates=[]
    for episode in active:
        points=truth[episode[0]-1:episode[-1]+1]
        info=descriptor(points)
        if abs(info["net_yaw_deg"])>=90 and info["center_path_m"]<=.15 and info["maximum_gap_s"]<=.5:
            candidates.append((points,info))
    if not candidates:
        raise RuntimeError("no qualifying recorded rotation; no figures generated")
    entire,whole=candidates[0]
    angle=0.0
    for i in range(1,len(entire)):
        angle+=math.degrees(delta(entire[i-1],entire[i]))
        if abs(angle)>=90:
            selected=entire[:i+1]
            break
    selection=descriptor(selected)
    start,end=selected[0]["received_monotonic_s"],selected[-1]["received_monotonic_s"]
    # Bind this derivative to the already reported truth-only selection.
    assert selection["start"]["received_utc"]=="2026-09-08T16:32:28.934659+00:00"
    assert selection["end"]["received_utc"]=="2026-09-08T16:32:33.444187+00:00"
    spec=importlib.util.spec_from_file_location("observer_math",OBSERVER)
    observer=importlib.util.module_from_spec(spec);spec.loader.exec_module(observer)
    streams={key:[r for r in rows if r["kind"]==key and start<=r["received_monotonic_s"]<=end and "invalid" not in r] for key in observer.TOPICS}
    pairs={};metrics={};quality={}
    for key in ("corrected","localization"):
        metrics[key+"_minus_truth"],pairs[key]=observer.match_errors(streams[key],selected,.20)
        assert len(pairs[key])>=3
    for key in ("truth","corrected","localization"):
        samples=streams[key]
        metrics[key]={"samples":len(samples),
            "end_displacement_m":math.hypot(samples[-1]["x"]-samples[0]["x"],samples[-1]["y"]-samples[0]["y"]),
            "path_m":path_length(samples)}
    for key,samples in streams.items():
        ts=[r["received_monotonic_s"] for r in samples]
        quality[key]={"samples":len(samples),"frames":sorted(set(str(r.get("frame")) for r in samples)),
            "maximum_receive_gap_s":max((b-a for a,b in zip(ts,ts[1:])),default=None),
            "maximum_boundary_gap_s":max(ts[0]-start,end-ts[-1]) if ts else None}
    raw=observer.raw_enu(streams["raw_gnss"])
    metrics["raw_gnss"]={"samples":len(raw),"coordinate_frame":"independent local ENU; not map aligned",
        "end_displacement_m":math.hypot(raw[-1]["x"],raw[-1]["y"]),
        "end_enu_m":[raw[-1]["x"],raw[-1]["y"]],
        "fix_status_values":sorted(set(r["fix_status"] for r in streams["raw_gnss"]))}
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(2,2,figsize=(16,10),dpi=150)
    fig.suptitle(TITLE+"\nExisting CARLA measurements; selection used truth rotation/path only, not GNSS error",fontsize=13)
    origin=selected[0]
    for key in ("truth","corrected","localization"):
        samples=streams[key]
        ax[0,0].plot([p["x"]-origin["x"] for p in samples],[p["y"]-origin["y"] for p in samples],".-",label=key)
    ax[0,0].set(title="Map-frame centers relative to initial truth",xlabel="map X [m]",ylabel="map Y [m]",aspect="equal")
    ax[0,1].plot([r["x"] for r in raw],[r["y"] for r in raw],".-",label="raw GNSS antenna")
    ax[0,1].set(title="Raw antenna local ENU (not map aligned)",xlabel="east [m]",ylabel="north [m]",aspect="equal")
    for key in ("truth","corrected","localization","heading"):
        samples=streams[key];angles=[0.]
        for a,b in zip(samples,samples[1:]):angles.append(angles[-1]+math.degrees(delta(a,b)))
        ax[1,0].plot([r["received_monotonic_s"]-start for r in samples],angles,label=key)
    ax[1,0].axhline(-90,linestyle="--",color="gray",label="first 90deg crossing")
    ax[1,0].set(title="Actual yaw change; no pause at the last selected sample",xlabel="selected interval time [s]",ylabel="yaw change [deg]")
    for key,values in pairs.items():
        first=values[0]
        ax[1,1].plot([p["t"]-start for p in values],
            [math.hypot(p["x"]-first["x"],p["y"]-first["y"]) for p in values],label=key+" error change")
    ax[1,1].set(title="Change in (estimated center − matched actual center)",xlabel="selected interval time [s]",ylabel="error-vector change [m]")
    for panel in ax.flat:panel.grid(True,alpha=.3);panel.legend()
    fig.tight_layout()
    png=HERE/"first90_of_178deg_actual.png";fig.savefig(png);plt.close(fig)
    subset=HERE/"selected_original_lines.jsonl"
    subset.write_bytes(b"".join(original_lines[r["source_line_1based"]-1] for r in rows if start<=r["received_monotonic_s"]<=end))
    report={"schema":"camrod.first90_recorded_turn_measurement.v1","created_utc":datetime.now(timezone.utc).isoformat(),
        "status":"RECORDED_LARGER_TURN_FIRST_90_DEG_SUBINTERVAL_ONLY","acceptance":"NOT_A_90_DEG_COMMAND_AND_STOP_TEST; no GNSS PASS threshold asserted",
        "selection_rule":{"input":"CARLA truth pose only; GNSS errors NOT used",
            "active_yaw_rate_threshold_degps":1.,"join_active_gap_s":1.,"maximum_truth_gap_s":.5,
            "minimum_whole_episode_net_angle_deg":90.,"maximum_whole_episode_center_path_m":.15,
            "episode_choice":"earliest qualifying whole episode","endpoint":"first ORIGINAL truth sample with abs accumulated yaw >=90deg; no interpolation"},
        "original_trace":{"path":str(TRACE),"sha256":sha(TRACE)},
        "recording_plan":json.loads((HERE.parent/"plan.json").read_text()),
        "analyzer":{"path":str(Path(__file__).resolve()),"sha256":sha(Path(__file__))},
        "reused_observer_math":{"path":str(OBSERVER),"sha256":sha(OBSERVER)},
        "full_recorded_turn":whole,"selected_first90_interval":selection,"metrics":metrics,"quality":quality,
        "png":{"path":str(png),"sha256":sha(png),"size_pixels":[2400,1500]},
        "source_subset":{"path":str(subset),"sha256":sha(subset),"format":"byte-identical original JSONL lines in received-time interval"},
        "scope":"Measured CARLA antenna body X=0,Y=+.45m; NOT hardware front-antenna calibration; original77deg NOT90 report unchanged"}
    (HERE/"first90_measurement.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
    print(json.dumps({"report":str(HERE/"first90_measurement.json"),"metrics":metrics,"quality":quality,"selection":selection},indent=2))


if __name__=="__main__":main()
