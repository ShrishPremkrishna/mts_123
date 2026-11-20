"""Enroll one target Pi using the staged release and its installed ROS platform."""
from __future__ import annotations

import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import fc_session

ENTRY = '''#!/usr/bin/env python3
# Managed Pupper FC entry; release and settings switch together in session.json.
import json, os, sys
from pathlib import Path
profile = Path.home() / ".config/pupper-fc/session.json"
selected = json.loads(profile.read_text())
program = Path(selected["release"]) / "foundation/hardware/fc_session.py"
os.execv(sys.executable, [sys.executable, "-B", "-s", str(program), *sys.argv[1:]])
'''
NATIVE_LIBS = {"native": ("neural_controller", "lib/libneural_controller.so"),
               "imu": ("control_board_hardware_interface", "lib/libcontrol_board_hardware_interface.so"),
               "gateway": ("joy_utils", "lib/joy_utils/estop_controller")}


def snapshot(directory, name, value):
    data = json.dumps(value, sort_keys=True)
    path = directory / (name + "-" + hashlib.sha256(data.encode()).hexdigest()[:16] + ".json")
    if path.exists():
        if fc_session.read(path) != value:
            raise ValueError("enrollment input snapshot differs: " + str(path))
    else:
        fc_session.write_new(path, value)
    return str(path)


def selected_package(env, package, relative):
    for entry in env.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if entry and (Path(entry) / "share/ament_index/resource_index/packages" / package).is_file():
            return (Path(entry) / relative).resolve(strict=True)
    raise ValueError("install/select the FC target overlay for ROS package " + package)


def platform(args, runs):
    from fc_walk_kick_release import file_digest, inspect_native
    template_path = args.platform_template
    if template_path is None and not args.platform_env:
        for path in sorted(runs.glob("*/selected-session.json"), key=lambda p: p.stat().st_mtime_ns, reverse=True):
            if not (path.parent / "native-readiness.json").is_file():
                continue
            prior = fc_session.read(path)
            if prior.get("prepared") and prior.get("environment", {}).get("PUPPER_SESSION_MOTION_MODE") == "fc_walk_kick":
                template_path = path
                break
    prior = fc_session.read(template_path) if template_path else None
    if prior:
        if (not prior.get("prepared") or prior.get("controllers") != ["gemini_walk","trick_leg_kick"]
                or prior.get("environment",{}).get("PUPPER_SESSION_MOTION_MODE") != "fc_walk_kick"):
            raise ValueError("select a prepared FC walk/kick platform from this Pi")
        env = {k: v for k,v in prior["environment"].items() if k in fc_session.PLATFORM_KEYS}
        python = prior["interpreter"]["executable"]
    else:
        env = {k: v for k,v in os.environ.items() if k in fc_session.PLATFORM_KEYS}
        python = sys.executable
        script = args.platform_env
        if not script and not env.get("AMENT_PREFIX_PATH"):
            choices = sorted((runs.parent / "pupper-fc-releases").glob("fc-platform-*/environment.sh"))
            if len(choices) != 1:
                raise ValueError("select the installed FC ROS overlay with --platform-env /path/to/environment.sh")
            script = choices[0]
        if script:
            # Evaluate only the operator's selected installed ROS environment; export only the required keys.
            read_env = "import os,sys,json;print(json.dumps([sys.executable,{k:os.environ[k] for k in " + repr(fc_session.PLATFORM_KEYS) + " if k in os.environ}]))"
            result = subprocess.check_output(["bash", "-c", 'source "$1" >/dev/null && exec python3 -c "$2"',
                                             "pupper-fc", str(script.resolve(strict=True)), read_env], timeout=20)
            python, env = json.loads(result)
    if args.imu_prefix:
        prefix = str(args.imu_prefix.resolve(strict=True))
        for key, path in (("AMENT_PREFIX_PATH",prefix),("CMAKE_PREFIX_PATH",prefix),
                          ("LD_LIBRARY_PATH",str(Path(prefix)/"lib"))):
            env[key] = os.pathsep.join([path, *filter(None, env.get(key,"").split(os.pathsep))])
    parts = {}
    for name, (pkg, relative) in NATIVE_LIBS.items():
        path = selected_package(env, pkg, relative)
        digest = file_digest(path)
        if prior and name in prior["components"] and not (name == "imu" and args.imu_prefix):
            old = prior["components"][name]
            if path != Path(old["path"]).resolve(strict=True) or digest != old["sha256"]:
                raise ValueError("the current ROS overlay differs from selected native component " + name)
        parts[name] = {"path":str(path), "sha256":digest}
    inspect_native(parts["native"]["path"])
    driver = Path(parts["imu"]["path"]).read_bytes()
    gateway = Path(parts["gateway"]["path"]).read_bytes()
    if (driver[:6] != b"\x7fELF\x02\x01" or driver[18:20] != b"\xb7\x00" or
            gateway[:6] != b"\x7fELF\x02\x01" or gateway[18:20] != b"\xb7\x00" or
            any(marker not in gateway for marker in (b"pupper_stop_v1",b"/estop_controller/start_motion",b"/estop_controller/request_stop"))):
        raise ValueError("install the Pi AArch64 driver and FC software Start/Stop gateway first")
    if b"FC_IMU_PUBLISH_PER_REPORT_V1" not in driver:
        raise ValueError("build the bundled control_board_hardware_interface and select its install prefix "
                         "with --imu-prefix; this driver lacks per-report IMU publication")
    if args.imu_recovery != "strict" and b"FC_IMU_RECOVERY_CONTRACT_V1_SUPPORTED_REHOME" not in driver:
        raise ValueError("requested IMU recovery is absent from the selected target driver")
    deps = ("import cv2,numpy,rclpy,hailo_platform; from joy_utils.srv import StartMotion; "
            "from controller_manager_msgs.srv import ListControllers, ListHardwareComponents, SetHardwareComponentState")
    subprocess.run([python,"-B","-s","-c",deps],env={**os.environ,**env},check=True,timeout=20)
    return {"prepared":True, "components":parts, "controllers":["gemini_walk","trick_leg_kick"],
            "interpreter":{"executable":str(Path(python).absolute())}, "run_dir":str(runs/"fc-enrolled"),
            "environment":{**env,"PUPPER_SESSION_MOTION_MODE":"fc_walk_kick"}}


def detector(args, runs, source):
    from fc_walk_kick_release import file_digest
    from final_project.behaviors.body_command_guard import ARENA_DETECTOR_IDENTITY
    expected = ARENA_DETECTOR_IDENTITY["model_sha256"]
    candidates = [args.detector_model] if args.detector_model else [
        *sorted(runs.glob("*/inputs/yolov8n_seg.hef"), reverse=True),
        source/"follow_me_pupper/robot/yolov8n_seg.hef", Path.home()/"follow_me_pupper/robot/yolov8n_seg.hef"]
    for path in candidates:
        if path.is_file() and file_digest(path) == expected:
            return str(path.resolve())
    raise ValueError("select the approved installed Hailo HEF with --detector-model /path/to/yolov8n_seg.hef")


def advertise(release, robot, action):
    tool = release / "foundation/hardware/fc_advertise.py"
    try:
        result = subprocess.run(["sudo","-n","/usr/bin/python3","-I",str(tool),action,"--robot-id",robot],
                                capture_output=True,text=True,timeout=5)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"available":False,"reason":str(exc),"fallback":"configured SSH host"}
    return {"available":result.returncode == 0, "reason":result.stderr.strip()[-300:],
            "fallback":"configured SSH host"}


def check_idle(config):
    current = config / "session.current.json"
    if not current.exists():
        return
    state = fc_session.read(current)
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if state.get("boot") == boot:
        owned = Path(state["run_dir"]) / "owned_session.json"
        if owned.exists() and fc_session.read(owned).get("status") != "stopped":
            raise ValueError("end the current native session before switching installations")


def install_entry(bin_dir):
    entry = bin_dir / "fc_session.py"
    bin_dir.mkdir(exist_ok=True,parents=True)
    if entry.is_file() and not entry.is_symlink():
        if entry.read_text() != ENTRY:
            raise ValueError("another program already owns " + str(entry))
        return
    if entry.is_symlink() and entry.resolve().parts[-3:] != ("foundation","hardware","fc_session.py"):
        raise ValueError("another program already owns " + str(entry))
    with tempfile.NamedTemporaryFile(mode="w", dir=bin_dir,delete=False) as fp:
        path = Path(fp.name)
        fp.write(ENTRY)
    try:
        path.chmod(0o755)
        os.replace(path,entry)
    finally:
        path.unlink(missing_ok=True)


def run(args, *, home=None):
    from fc_walk_kick_release import ROOT, file_digest, selected_paths, stage, verify
    from final_project.behaviors.fc_executor_bridge import load_fields
    home = home or Path.home()
    config, runs = home/".config/pupper-fc", home/"pupper-fc-runs"
    config.mkdir(mode=0o700,parents=True,exist_ok=True)
    runs.mkdir(mode=0o700,parents=True,exist_ok=True)
    active, backup = config/"session.json", config/"session.previous.json"
    with (config/"session.lock").open("a") as owner:
        fcntl.flock(owner,fcntl.LOCK_EX | fcntl.LOCK_NB)
        check_idle(config)
        old = fc_session.read(active) if active.exists() else None
        original_entry = home/"pupper-fc/bin/fc_session.py"
        if old and "release" not in old:
            if not original_entry.is_symlink():
                raise ValueError("existing profile lacks its installed release link")
            old["release"] = str(original_entry.resolve(strict=True).parents[2])
            legacy = True
        else:
            legacy = False
        if args.action == "rollback":
            if not old:
                disabled = config/"session.disabled.json"
                if disabled.is_file():
                    return {"state":"enrollment disabled","saved_profile":str(disabled),"motors_started":False}
                raise ValueError("there is no active Pupper FC enrollment")
            if backup.is_file() and fc_session.read(backup) == old:
                return {"state":"already restored","release":old["release"],"robot_id":old["robot_id"],
                        "motors_started":False}
            if not backup.is_file():
                if legacy or original_entry.is_symlink() or not original_entry.is_file() or original_entry.read_text() != ENTRY:
                    raise ValueError("only a managed first enrollment may be disabled")
                discovery = advertise(Path(old["release"]),old["robot_id"],"--remove")
                active.replace(config/"session.disabled.json")
                return {"robot_id":old["robot_id"], "state":"enrollment disabled", "motors_started":False,
                        "discovery":discovery, "saved_profile":str(config/"session.disabled.json")}
            selected = fc_session.read(backup)
            root = Path(selected["release"])
            if file_digest(root/"app-stage-manifest.json") != selected["manifest_sha256"]:
                raise ValueError("previous enrollment manifest differs from its saved digest")
            subprocess.run([sys.executable, "-B", "-s", str(root/"foundation/hardware/fc_walk_kick_release.py"),
                            "verify", "--release", str(root), "--manifest-sha256", selected["manifest_sha256"]],
                           check=True, timeout=60, stdout=subprocess.DEVNULL)
        else:
            model = detector(args,runs,ROOT)
            template = platform(args,runs)
            if args.release:
                root = args.release.resolve(strict=True)
            else:
                fingerprint = hashlib.sha256()
                for path in selected_paths(ROOT):
                    fingerprint.update((path+"="+file_digest(ROOT/path)+"\n").encode())
                root = home/"pupper-fc-releases"/("fc-"+fingerprint.hexdigest()[:16])
                if not root.exists():
                    stage(ROOT,root)
            verified = verify(root)
            if args.field_config:
                field = fc_session.read(args.field_config)
            else:
                arena = fc_session.read(root/"final_project/pupper_fc/arena.json")
                field = {"field":{**arena["field"],"robot_radius":.16,"bumper_offset":.19},
                         "attack_signs":{"pupper-1":-1,"pupper-2":1}}
            load_fields(field)
            selected = {"release":str(root),"robot_id":args.robot_id, "side":args.side,
                        "manifest_sha256":verified["manifest_sha256"],"platform_template":snapshot(config,"platform",template),
                        "detector_model":model,"imu_recovery":args.imu_recovery,"recording":args.recording,
                        "field_config":snapshot(config,"field",field),"producer_id":"fc-field",
                        "camera":{"port":5005,"track":"striker" if args.robot_id=="pupper-1" else "keeper"}}
            fc_session.check_profile(selected,args.robot_id,template)
        if legacy:
            fc_session.write_new(active,old)
        install_entry(original_entry.parent)
        if selected != old:
            if old and args.action != "rollback":
                fc_session.write_new(backup,old)
            fc_session.write_new(active,selected)
        discovery = advertise(root,selected["robot_id"],"--install")
        return {"robot_id":selected["robot_id"],"release":str(root),"manifest_sha256":selected["manifest_sha256"],
                "discovery":discovery,"state":"enrolled","motors_started":False,
                "rollback_available":backup.is_file() and fc_session.read(backup) != selected}

