#!/usr/bin/env python3
"""HH_261007 - Bounded normal-UI CARLA trial; never site/return acceptance.

Loads the existing local runtime environment. No motion occurs without running
this script explicitly. Every command is a real browser pointer/key event.
"""
import argparse, hashlib, json, math, os
from pathlib import Path
import signal, sys, threading, time
from probe_manual_crab_once import Observations, require_idle_mission, motion_metrics
ROOT = Path(__file__).resolve().parents[1]
UI, CDP = "http://127.0.0.1:8010", "http://127.0.0.1:9224"
PROFILE = {"/carla_extended_ackermann_control": {
    "experimental_crab_yaw_hold_enabled": True, "extreme_crab_curb_boost_enable": True,
    **{"experimental_crab_yaw_hold_"+key: value for key,value in {
        "heading_gain_nm_per_rad":8., "rate_gain_nm_per_radps":16.,
        "maximum_reduction_fraction":.4, "maximum_heading_error_deg":12.,
        "maximum_yaw_rate_degps":18., "stall_timeout_sec":4.,
        "stall_progress_mps":.05, "odometry_timeout_sec":.25, "torque_limit_nm":16.}.items()}},
    "/physical_four_wheel_bridge": {"experimental_crab_yaw_hold_enabled":True,
                                    "wheel_torque_safety_cap_nm":20.}}
def duration(value):
    number = float(value)
    if not math.isfinite(number) or not 0. < number <= 2.:
        raise argparse.ArgumentTypeError("duration must be in (0, 2] seconds")
    return number
def require_profile(actual):
    if not isinstance(actual,dict) or set(actual)!=set(PROFILE): raise RuntimeError("unknown profile nodes")
    for node,values in PROFILE.items():
        if not isinstance(actual[node],dict) or set(actual[node])!=set(values): raise RuntimeError("unknown profile fields")
        for key,expected in values.items():
            value = actual.get(node,{}).get(key)
            if type(value) is not type(expected) or value != expected:
                raise RuntimeError("unverified experimental profile: "+node+"/"+key)
def bounded(sample, origin, stationary=False):
    metrics = motion_metrics(sample,origin)
    yaw_rate=sample["actor"]["angular_velocity_degrees_per_second"]["z"]
    if (sample["actor"]["id"] != origin["actor"]["id"] or len(sample["wheels"]) != 4
            or any(w.get("is_in_air") is not False for w in sample["wheels"])
            or metrics["speed_mps"] > (.02 if stationary else .35)
            or abs(metrics["yaw_drift_deg"]) > (1. if stationary else 3.)
            or metrics["displacement_m"] > (.02 if stationary else .60)
            or not math.isfinite(yaw_rate) or (stationary and abs(yaw_rate)>1.)):
        raise RuntimeError("physical identity/contact/motion bound exceeded: "+str(metrics))
    return metrics
def reserve_output(path):
    path=Path(path)
    if path.suffix != ".json" or not path.parent.is_dir():
        raise ValueError("output must be a NEW .json in an existing directory")
    return path.open("x",encoding="utf-8")
def run(args,result):
    sys.path.insert(0,str(ROOT/"scripts/virtual_carla"))
    import carla
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python
    from rosidl_runtime_py.convert import message_to_ordereddict
    from carla_extended_ackermann_msgs.msg import ExtendedControlInfo
    from manual_4ws_evidence import ManualDriveBrowser, RosTraceObserver
    from camping_site_matrix import UIClient
    from record_physical_wheel_telemetry import capture_sample, select_unique_ranger
    observed=Observations(); browser=release=trace=None; owned=False; timer=None
    ui=UIClient(UI,timeout_s=.2)
    try:
        require_idle_mission(ui.state(),before_arm=True)
        browser=ManualDriveBrowser(CDP,UI,timeout_s=2.)
        if browser.snapshot().get("armed") is not False: raise RuntimeError("existing manual owner")
        release=ManualDriveBrowser(CDP,UI,timeout_s=.2)
        if browser._target.get("id") != release._target.get("id"): raise RuntimeError("CDP page mismatch")
        client=carla.Client("127.0.0.1",2000);client.set_timeout(3.)
        world=client.get_world();world.wait_for_tick(3.);actor=select_unique_ranger(world)
        client.set_timeout(.2); result["actor_id"]=actor.id
        trace=RosTraceObserver(args.output.with_suffix(".ros.jsonl"));trace.expected_actor_id=actor.id
        trace.wait_ready(3.)
        trace.node.create_subscription(ExtendedControlInfo,
            "/carla/ego_vehicle/extended_ackermann_control/control_info",
            lambda msg:observed.record("control",message_to_ordereddict(msg)),10)
        def ros_loop():
            while not observed.stop.is_set(): trace.spin_for(.02)
        def physical_loop():
            while not observed.stop.is_set():
                sample=capture_sample(carla,actor,len(observed.rows),time.monotonic()-observed.start,time.time_ns())
                observed.record("physical",sample,sample["world_frame"]);observed.stop.wait(.05)
        observed.worker(ros_loop);observed.worker(physical_loop)
        actual={}
        for name,expected in PROFILE.items():
            request=trace.node.create_client(GetParameters,name+"/get_parameters")
            if not request.wait_for_service(timeout_sec=2.): raise RuntimeError("parameter service missing: "+name)
            future=request.call_async(GetParameters.Request(names=list(expected))); deadline=time.monotonic()+3.
            while not future.done() and time.monotonic()<deadline: time.sleep(.02)
            if not future.done(): raise RuntimeError("parameter request timeout")
            actual[name]=dict(zip(expected,[parameter_value_to_python(v) for v in future.result().values]))
        # HH_261007 - ros_compatibility.get_param uses get_parameter_or without
        # declaring missing defaults. Only gate-bound numeric defaults may resolve None.
        from carla_extended_ackermann_control import experimental_crab_yaw_hold as hold
        raw=Path(os.environ["RANGER_PHYSICAL_MANIFEST"]).read_bytes();gate=json.loads(raw)
        status=next(r["message"] for r in reversed(trace.records) if r.get("topic","").endswith("/physical_four_wheel_status"))
        if hashlib.sha256(raw).hexdigest()!=status["physical_manifest_sha256"]: raise RuntimeError("live gate digest mismatch")
        for key,name in (("experimental_crab_yaw_hold_source","experimental_crab_yaw_hold.py"),("controller_source","carla_extended_ackermann_control_node.py")):
            if hashlib.sha256(Path(hold.__file__).with_name(name).read_bytes()).hexdigest()!=gate["sources"][key]["sha256"]: raise RuntimeError("installed profile source differs from live gate")
        result["live_parameters"]=json.loads(json.dumps(actual));result["numeric_default_resolution"]={"gate_sha256":status["physical_manifest_sha256"],"source":gate["sources"]["experimental_crab_yaw_hold_source"],"semantics":"get_parameter_or fallback; enable flags never inferred"}
        defaults=hold.CrabYawHoldConfig();controller=actual["/carla_extended_ackermann_control"]
        for field in defaults.__dataclass_fields__:
            key="experimental_crab_yaw_hold_"+field
            if controller.get(key) is None: controller[key]=getattr(defaults,field)
        result["effective_parameters"]=actual;require_profile(actual)
        origin=observed.get("physical",advancing=True);bounded(origin,origin,True)
        require_idle_mission(ui.state(),before_arm=True)
        if browser.snapshot().get("armed") is not False: raise RuntimeError("manual owner changed")
        result["login"]=browser.ensure_fresh_administrator_session()
        browser.scroll_pointer_target_into_view('[data-ui="manual-drive-scale"]',"speed scale")
        browser._click('[data-ui="manual-drive-scale"]',"select speed scale")
        for event in ("rawKeyDown","keyUp"):
            browser._call("Input.dispatchKeyEvent",{"type":event,"key":"Home","code":"Home","windowsVirtualKeyCode":36})
        panel=browser.wait_snapshot(lambda p:p.get("scale")==.1,"actual UI scale 0.1")
        if panel.get("scale") != .1 or panel.get("lateralLimitMps") != 2.: raise RuntimeError("UI must show scale0.1 and lateral limit2")
        browser.scroll_pointer_target_into_view('.manual-drive-title',"manual heading")
        browser._click('.manual-drive-title',"focus real key controls")
        require_idle_mission(ui.state(),before_arm=True);owned=True;result["arm"]=browser.arm()
        browser.timeout_s=.2
        def latest(topic):
            rows=[r for r in trace.records if r.get("topic")==topic]
            if not rows or (time.time_ns()-rows[-1]["received_at_unix_ns"])/1e9>.8: raise RuntimeError("stale "+topic)
            return rows[-1]["message"]
        def check(reference,stationary=False,sign=0):
            require_idle_mission(ui.state());trace.assert_motion_liveness(.5)
            panel=browser.snapshot()
            if panel.get("armed") is not True or panel.get("scale") != .1 or panel.get("lateralLimitMps") != 2.: raise RuntimeError("manual state changed")
            if latest("/control/cmd_vel_safety_gate/status")["operating_state"] != "ENABLED": raise RuntimeError("gate not ENABLED")
            if observed.get("control").get("interlock_reason") not in ("","awaiting physical steering acknowledgement"): raise RuntimeError("controller interlock")
            for topic in ("/control/manual_cmd_vel_ros","/control/cmd_vel_ros"):
                cmd=latest(topic);axes=[*cmd["linear"].values(),*cmd["angular"].values()]
                if any(not math.isfinite(v) for v in axes) or any(abs(v)>.001 for i,v in enumerate(axes) if i!=1): raise RuntimeError("unexpected command axes")
                if abs(axes[1])>(.001 if stationary else .201) or axes[1]*sign<-.001: raise RuntimeError("unexpected lateral command")
            return bounded(observed.get("physical",advancing=True),reference,stationary)
        deadline=time.monotonic()+4.
        while True:
            try: check(origin,True);break
            except RuntimeError:
                if time.monotonic()>deadline: raise
                time.sleep(.05)
        result["pulses"]=[]
        for key,sign in (("KeyZ",1),("KeyC",-1)):
            origin=observed.get("physical",advancing=True);start=time.monotonic();metrics=[]
            def cutoff(held_key=key):
                try:
                    try: release.dispatch_key(held_key,False)
                    finally: release.zero()
                except Exception as error: observed.errors.append("deadline release: "+str(error))
            timer=threading.Timer(args.duration_s,cutoff);timer.daemon=True;timer.start()
            try:
                browser.dispatch_key(key,True)  # UI classifies Z/C as CRAB; no mode injection.
                while time.monotonic()-start<args.duration_s:
                    metrics.append(check(origin,sign=sign));time.sleep(.02)
            finally:
                try: browser.dispatch_key(key,False)
                finally: browser.zero();timer.cancel();timer.join(.6)
            result["pulses"].append({"key":key,"origin":origin,"metrics":metrics})
            deadline=time.monotonic()+3.
            while True:
                try: check(observed.get("physical",advancing=True),True);break
                except RuntimeError:
                    if time.monotonic()>deadline: raise
                    time.sleep(.05)
            if not metrics or max(m["displacement_m"] for m in metrics)<.02: raise RuntimeError("no measured translation; not qualified")
        result["status"]="bounded_UI_pulses_recorded_not_site_proof"
    finally:
        if timer: timer.cancel();timer.join(.6)
        if owned and release:
            result["cleanup_unarmed"]=release.fail_safe(("KeyZ","KeyC"));time.sleep(.3)
            try:
                final=observed.get("physical",advancing=True);bounded(final,final,True)
                result["cleanup_stationary_zero_torque"]=all(abs(w["external_drive_torque_nm"])<1e-6 for w in final["wheels"])
            except Exception as error: result["cleanup_error"]=str(error)
        if trace: result["raw_ros_records"]=list(trace.records)
        result["observations"]=list(observed.rows);result["observer_errors"]=list(observed.errors)
        if browser:
            try: result["real_UI_commands"]=browser.probe();result["interactions"]=list(browser._interactions)
            except Exception as error: result["UI_evidence_error"]=str(error)
        observed.stop.set()
        for thread in observed.threads: thread.join(.5)
        for page in (browser,release):
            if page: page.close()
        if trace: trace.close()
def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--duration-s",type=duration,default=2.);args=parser.parse_args()
    if os.environ.get("CAMROD_CRAB_QUAL_ENV_READY") != "1":
        shell='source "$1" && source "$2" && virtual_carla_source_ros true && export PYTHONPATH="$RANGER_CARLA_PYTHON_EGG:$PYTHONPATH" CAMROD_CRAB_QUAL_ENV_READY=1 && exec /usr/bin/python3 "$3" "${@:4}"'
        os.execv("/bin/bash",["bash","-c",shell,"crab-qualification",str(ROOT.parent/"config/virtual_carla.env"),str(ROOT/"scripts/virtual_carla/env.sh"),str(Path(__file__).resolve()),*sys.argv[1:]])
    if os.environ.get("ROS_DOMAIN_ID") != "188": parser.error("local CARLA domain188 required")
    result={"scenario_pass":None,"status":"preflight_rejected","duration_s":args.duration_s}
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt("terminated")))
    with reserve_output(args.output) as stream:
        try: run(args,result)
        except (Exception,KeyboardInterrupt) as error: result.update(status="aborted",error=str(error))
        finally: json.dump(result,stream,indent=2,allow_nan=False);stream.flush();os.fsync(stream.fileno())
    return 0 if result.get("cleanup_unarmed") and result.get("cleanup_stationary_zero_torque") and result["status"].startswith("bounded_") else 1

if __name__ == "__main__": raise SystemExit(main())
