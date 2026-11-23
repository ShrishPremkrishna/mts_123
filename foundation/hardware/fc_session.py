#!/usr/bin/env python3
"""Operate the selected strict Pi session. Home moves motors; Connect may close a terminal session or start perception."""
import argparse
from contextlib import ExitStack
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
PROC = Path("/proc")
DEFAULT_PROFILE = Path("~/.config/pupper-fc/session.json")
PLATFORM_KEYS = ("PATH", "PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH",
                 "LD_LIBRARY_PATH", "ROS_DISTRO", "RMW_IMPLEMENTATION", "PUPPER_MONOREPO_DIR")
DISCOVERY_OVERRIDES = ("ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER", "ROS_AUTOMATIC_DISCOVERY_RANGE",
                       "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE", "CYCLONEDDS_URI")


def read(path):
    return json.loads(Path(path).read_text())


def write_new(path, contents):
    temporary = path.with_name("." + path.name + "." + uuid.uuid4().hex)
    try:
        temporary.write_text(json.dumps(contents) + "\n")
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def environment(template, profile):
    # The selected template supplies the Pi ABI, ROS overlay and native libraries.
    # Never import the previous run's session ID, node addresses or run directory.
    env = {k: os.environ[k] for k in ("HOME", "USER", "LANG", "LC_ALL", "TMPDIR") if k in os.environ}
    env.update({k: template["environment"][k] for k in PLATFORM_KEYS if k in template["environment"]})
    if os.environ.get("PUPPER_IMU_STARTUP_TRACE"):
        env["PUPPER_IMU_STARTUP_TRACE"] = "1"  # prepare selects the path inside the new run
    env.update(ROS_DOMAIN_ID="168", ROS_LOCALHOST_ONLY="1", PUPPER_FC_IMU_RECOVERY="strict",
               PUPPER_FC_RECORDING=profile.get("recording","events"), PUPPER_SESSION_SKILLS="leg_kick",
               PUPPER_SESSION_MOTION_MODE="fc_walk_kick", PUPPER_FOLLOWER_MODE="disabled",
               PUPPER_FC_DETECTOR_MODEL=str(Path(profile["detector_model"]).resolve(strict=True)))
    if gateway := template.get("components", {}).get("gateway"):
        env.update(PUPPER_FC_GATEWAY=gateway["path"], PUPPER_FC_GATEWAY_SHA256=gateway["sha256"])
        if template.get("fc_operator_control") is True:
            env["PUPPER_FC_OPERATOR_CONTROL"] = "1"
    return env


def check_profile(profile, requested_robot, template):
    if profile["robot_id"] not in ("pupper-1", "pupper-2") or (
            requested_robot and requested_robot != profile["robot_id"]):
        raise ValueError("Pi team identity differs from the requested robot")
    if (not template.get("prepared") or template["environment"].get("PUPPER_SESSION_MOTION_MODE") != "fc_walk_kick"
            or template.get("controllers") != ["gemini_walk", "trick_leg_kick"]):
        raise ValueError("select a previously prepared FC platform template")
    if profile.get("recording","events") not in ("events","full"):
        raise ValueError("unknown FC recording selection")
    if profile.get("imu_recovery", "strict") != "strict":
        raise ValueError("this candidate supports strict IMU fault handling only")
    if profile.get("side", "right") not in ("left", "right"):
        raise ValueError("kick side must be left or right")


def check_run(run, profile, env):
    plan = read(run / "selected-session.json")
    if (not plan.get("prepared") or Path(plan["app_root"]) != ROOT or Path(plan["run_dir"]) != run
            or plan["environment"].get("ROS_LOCALHOST_ONLY") != "1"
            or plan["environment"].get("PUPPER_FC_DETECTOR_MODEL") != env["PUPPER_FC_DETECTOR_MODEL"]
            or plan["environment"].get("PUPPER_FC_IMU_RECOVERY", "strict") != env["PUPPER_FC_IMU_RECOVERY"]
            or plan["environment"].get("PUPPER_FC_OPERATOR_CONTROL") != env.get("PUPPER_FC_OPERATOR_CONTROL")
            or plan["environment"].get("PUPPER_FC_RECORDING","full") != env["PUPPER_FC_RECORDING"]
            or plan["components"]["app_manifest"]["sha256"] != profile["manifest_sha256"]):
        raise ValueError("run differs from selected FC profile or lacks loopback isolation; prepare a fresh session")
    return plan


def manager_exited(run):
    """True only when the recorded manager PID is gone or was reused; unreadable state raises."""
    manager = read(run / "owned_session.json")["processes"]["manager"]
    try:
        start = (PROC / str(manager["pid"]) / "stat").read_text().rsplit(")", 1)[1].split()[19]
    except (FileNotFoundError, ProcessLookupError):
        return True
    return str(start) != str(manager["start_ticks"])


def check_manager(run, plan, boot):
    receipt = read(run / "owned_session.json")
    if (receipt["boot_id"] != boot or receipt["session_id"] != plan["session_id"]
            or receipt["status"] != "running" or not (run / "native-readiness.json").is_file()):
        raise ValueError("session is stopped, starting or incomplete; inspect its receipt before recovery")
    readiness = read(run / "native-readiness.json")
    if readiness.get("session_id") != plan["session_id"] or readiness.get("homing_finished") is not True:
        raise ValueError("no confirmed Home for the selected native session")
    if manager_exited(run):
        raise ValueError("The controller manager for this Home exited. Reconnect to release the session, then Home on the stand.")
    manager = receipt["processes"]["manager"]
    proc = PROC / str(manager["pid"])
    start = (proc / "stat").read_text().rsplit(")", 1)[1].split()[19]
    environ = dict(entry.split(b"=", 1) for entry in (proc / "environ").read_bytes().split(b"\0") if b"=" in entry)
    if (str(start) != str(manager["start_ticks"]) or os.readlink(proc / "exe") != manager["executable"]
            or Path(manager["executable"]).name != "ros2_control_node"
            or environ.get(b"PUPPER_PROBE_SESSION_ID") != plan["session_id"].encode()
            or environ.get(b"ROS_LOCALHOST_ONLY") != b"1" or environ.get(b"ROS_DOMAIN_ID") != b"168"
            or any(environ.get(k.encode()) for k in DISCOVERY_OVERRIDES)):
        raise ValueError("running manager identity or local-only ROS environment differs")
    # The combined runner additionally verifies loaded libraries, controller state and exclusive ownership.
    # An inactive hardware interface may leave this same manager PID alive.


def current_board_mode(run):
    """Read the current manager's strict hardware receipt before reusing a saved Home.

    Strict status values: 0 unknown, 1 healthy, 4 terminal, 5 homing. Generation
    (row[4]) is nonzero only after a completed Home on this manager.
    """
    from final_project.behaviors.board_imu import AGE_NS, BoardReceipt
    manager = read(run / "owned_session.json")["processes"]["manager"]["pid"]
    board = BoardReceipt(run / "fc-board-health.bin", manager, "strict")
    try:
        row = board.read()
    finally:
        board.close()
    if row[7] == 4:
        return "terminal"
    if not row[4] or row[7] != 1 or row[21] != 3 or not 0 <= time.monotonic_ns() - row[8] < AGE_NS:
        raise ValueError("Current board feedback does not confirm an active Home. Check Blue or Red before standing.")
    return "healthy"


def terminal_reason(run):
    """Why a running Home can no longer be reused, or None. Only Connect/prepare may release it."""
    from fc_operator import board_error
    receipt = read(run / "owned_session.json")
    if receipt.get("status") != "running" or "manager" not in receipt.get("processes", {}):
        return None
    if manager_exited(run):
        return board_error(run) or "The controller manager exited after Home"
    if current_board_mode(run) == "terminal":
        return board_error(run) or "The board deactivated after Home"
    return None


def run_home_probe(command, *, env, timeout):
    # The terminal delivers Ctrl-C or hangup to the parent and foreground probe.
    # Let the probe finish its Stop acknowledgement and ROS cleanup before the
    # parent releases ownership. Never kill the probe: subprocess.run would, on
    # both KeyboardInterrupt and timeout, skipping its Stop. An expired bound
    # interrupts it instead, and its own bounded Stop path runs to completion.
    interrupted = False
    def defer(signum, frame):
        nonlocal interrupted
        interrupted = True
    with ExitStack() as cleanup:
        for sig in (signal.SIGINT, signal.SIGHUP):
            cleanup.callback(signal.signal, sig, signal.signal(sig, defer))
        try:
            probe = subprocess.Popen(command, env=env)
            try:
                code = probe.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                probe.send_signal(signal.SIGINT)
                probe.wait()
                raise ValueError(f"Home probe exceeded {timeout} s; it was interrupted and finished its Stop path")
            if code:
                raise subprocess.CalledProcessError(code, command)
        finally:
            if interrupted:
                raise KeyboardInterrupt


def start_deferred_perception(run, python, helper, env):
    # Helper transactions keep their own deadlines; killing one mid-spawn would
    # leave an ambiguous ownership receipt.
    if read(run / "owned_session.json").get("perception_deferred") is True:
        subprocess.run([python, "-B", "-s", helper, "start-perception", "--run-dir", run],
                       env=env, check=True, stdout=subprocess.DEVNULL)


def acquire_run(action, args, profile_path, profile, template, env, boot, python, helper):
    current = profile_path.with_suffix(".current.json")
    state = read(current) if current.exists() else {}
    run = Path(state["run_dir"]) if state.get("boot") == boot else None
    profile_key = hashlib.sha256(json.dumps([str(ROOT),profile["manifest_sha256"],profile["platform_template"],
                                             profile.get("imu_recovery", "strict"),profile.get("recording","events"),profile["detector_model"]],
                                            separators=(",",":")).encode()).hexdigest()
    if run and state.get("profile", profile_key) != profile_key:
        owned = run / "owned_session.json"
        if action != "prepare" or (owned.exists() and read(owned).get("status") != "stopped"):
            raise ValueError("previous native selection is active or prepare is required")
        run = None
    if run and run.is_dir() and action == "prepare" and (run / "owned_session.json").is_file():
        if read(run / "owned_session.json").get("status") == "stopped":
            run = None
    if run is None:
        if action != "prepare":
            raise ValueError("no selected session for this boot; run prepare first")
        parent = Path(template["run_dir"]).parent.resolve(strict=True)
        run = parent / ("fc-" + profile["robot_id"] + "-" + uuid.uuid4().hex[:12])
        parts = template["components"]
        topic_file = "fc-match-events.json" if env["PUPPER_FC_RECORDING"] == "events" else "fc-walk-kick-topics.json"
        cmd = [python, "-B", "-s", helper, "prepare", "--run-dir", run,
               "--app-manifest", ROOT / "app-stage-manifest.json",
               "--app-manifest-sha256", profile["manifest_sha256"],
               "--telemetry-topics", ROOT / "foundation/hardware" / topic_file]
        for name in ("native", "imu"):
            cmd.extend(["--" + name + "-library", parts[name]["path"],
                        "--" + name + "-sha256", parts[name]["sha256"]])
        if getattr(args, "connect", False):
            print(json.dumps(dict(event="preparing", robot_id=profile["robot_id"])), flush=True)
        subprocess.run(cmd, env=env, check=True, stdout=subprocess.DEVNULL)
        state = {"run_dir": str(run), "boot": boot, "robot_id": profile["robot_id"], "profile": profile_key}
        write_new(current, state)
    if state["robot_id"] != profile["robot_id"]:
        raise ValueError("active session belongs to another robot")
    plan = check_run(run, profile, env)
    # A started prepare/Connect run is checked by main, which may release a terminal Home.
    if action == "home":
        if not (run / "owned_session.json").exists():
            if not args.supported:
                raise ValueError("support the robot on its stand; use home --supported to start the motors")
            write_new(run / "awaiting-standing.json", {"session_id": plan["session_id"]})
            command = [python, "-B", "-s", helper, "start-native", "--run-dir", run]
            if env["PUPPER_FC_RECORDING"] == "events":
                command.append("--defer-perception")
            # start-native owns its readiness deadline and cleanup; never kill it mid-Home.
            subprocess.run(command, env=env, check=True, stdout=subprocess.DEVNULL)
        check_manager(run, plan, boot)
        if current_board_mode(run) == "terminal":
            from fc_operator import board_error
            raise ValueError((board_error(run) or "The board deactivated after Home")
                             + ". Support the robot and reconnect for a fresh Home.")
        probe = [python, "-B", "-s", str(Path(__file__).resolve())]
        if args.supported and not args.home_only:
            probe.append("--dashboard-stand" if args.dashboard else "--supported-stand")
        else:
            probe.append("--read-hardware")
        with ExitStack() as stack:
            if args.supported and not args.home_only:
                # Cold/warm motor enable may only belong to this supervised terminal.
                for path in ("fc-runner.lock", "field-source.lock"):
                    owner = stack.enter_context((run / path).open("a"))
                    fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
                start_deferred_perception(run, python, helper, env)
            run_home_probe(probe, env={**env, **plan["environment"]},
                           timeout=80)
            if args.supported and not args.home_only:
                (run / "awaiting-standing.json").unlink(missing_ok=True)
    elif action in ("match", "field", "operator"):
        check_manager(run, plan, boot)
    return run, plan


def session_field(profile, run, pitch=None):
    geometry = Path(profile["field_config"]).read_bytes()
    if pitch is not None:
        if str(ROOT) not in sys.path:
            sys.path.insert(0, str(ROOT))
        from final_project.pupper_fc.model import Field
        config = json.loads(geometry)
        dimensions = json.loads(pitch)
        measured = Field(**config["field"]).measured_pitch(dimensions)
        config["field"].update({key: getattr(measured, key) for key in dimensions})
        geometry = json.dumps(config, sort_keys=True).encode()
    field_path = run / ("field-" + hashlib.sha256(geometry).hexdigest() + ".json")
    if not field_path.exists():
        write_new(field_path, json.loads(geometry))
    return str(field_path)


def ssh_address(index):
    peer = os.environ.get("SSH_CONNECTION", "").split()
    if len(peer) == 4:
        return str(ipaddress.ip_address(peer[index]))
    return None


def camera_host(profile):
    return ssh_address(0) or profile.get("camera", {}).get("host")


def describe_profile(profile, field):
    # check_profile admits strict profiles only; the fields remain for laptop compatibility.
    return {"robot_id": profile["robot_id"], "producer_id": profile.get("producer_id", "fc-field"),
            "imu_recovery": "strict", "match_pitch_version": 1, "match_capable": True,
            "source_host": camera_host(profile), "robot_host": ssh_address(2),
            "port": profile.get("camera", {}).get("port", 5005), "field_config": field}


def field_command(args, profile, run, python, epoch, *, continuous):
    cam = profile.get("camera", {})
    host = args.source_host or camera_host(profile)
    session = args.source_session or args.camera_session or cam.get("session")
    revision = args.revision if args.revision is not None else cam.get("revision")
    robot = profile["robot_id"]
    if not (host and session and type(revision) is int and revision >= 1):
        raise ValueError("field needs current source host, session and calibration revision")
    cmd = [python, "-B", "-s", str(ROOT / "final_project/behaviors/live_field_attack.py"),
           "--release", str(ROOT), "--field-config", session_field(profile, run, getattr(args, "pitch", None)), "--robot-id", robot,
           "--producer-id", profile.get("producer_id", "fc-field"), "--epoch", epoch,
           "--source-host", host, "--source-session", session, "--port", str(cam.get("port", 5005)),
           "--calibration-revision", str(revision), "--publish", "--duration", str(args.duration)]
    if profile.get("recording","events") == "full":
        cmd.extend(["--trace", str(run / ("field-" + uuid.uuid4().hex + ".jsonl"))])
    tracks = json.loads(args.tracks) if args.tracks else profile.get("tracks", {cam.get("track", "striker"): robot})
    if not isinstance(tracks, dict) or robot not in tracks.values():
        raise ValueError("selected robot is missing from the field's active tracks")
    if continuous:
        cmd.extend(["--match-tracks", json.dumps(tracks)])
    else:
        cmd.extend(["--source-robot", next(k for k, v in tracks.items() if v == robot)])
    return cmd


def discover_match(args, profile, timeout=8):
    """Read the selected laptop's actual broadcast binding before an explicit terminal match."""
    cam = profile.get("camera", {})
    host = args.source_host or camera_host(profile)
    if not host:
        raise ValueError("run match from the camera laptop over SSH or supply --source-host")
    host = socket.gethostbyname(host)
    end = time.monotonic() + timeout
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("0.0.0.0",cam.get("port",5005)))
        while time.monotonic() < end:
            receiver.settimeout(max(.001,end-time.monotonic()))
            try:
                data, sender = receiver.recvfrom(65535)
            except socket.timeout:
                break
            if sender[0] != host:
                continue
            try:
                frame = json.loads(data)
                wire, match = frame["_transport"], frame.get("fc_match",{})
                if (wire["protocol"] != "fieldcam_state_v1" or wire["source_id"] != "fieldcam" or wire["live"] is not True
                        or frame["calibration_valid"] is not True or type(frame["calibration_revision"]) is not int):
                    continue
                if not args.one_shot:
                    if (profile["robot_id"] not in match.get("roster",[]) or not isinstance(match.get("tracks"),dict)
                            or not isinstance(match.get("epoch"),str) or args.epoch and args.epoch != match["epoch"]):
                        continue
                    args.epoch = match["epoch"]
                    args.tracks = args.tracks or json.dumps(match["tracks"])
                args.source_session = wire["source_session"]
                args.revision = frame["calibration_revision"]
                return
            except (KeyError, ValueError, TypeError, AttributeError):
                continue
    raise ValueError("no calibrated match session from the selected laptop; start its overhead camera first")


def run_match(args, profile, plan, run, python, env):
    producer = profile.get("producer_id", "fc-field")
    robot = profile["robot_id"]
    field = session_field(profile, run)
    cmd = [python, "-B", "-s", str(ROOT / "final_project/behaviors/combined_fc_runner.py"),
           "--run-dir", str(run), "--robot-id", robot, "--side", profile.get("side", "right"),
           "--field-config", field, "--producer-id", producer]
    env.update(plan["environment"])
    if args.runner_only:
        if ssh_address(2):
            print(json.dumps({"event": "profile", **describe_profile(profile, read(field))}), flush=True)
        cmd.append("--operator-controls")
        os.execve(python, cmd, env)  # SSH stdin, stdout, EOF and signals go directly to the sole runner.
    camera = profile.get("camera", {})
    if not ((args.source_session or args.camera_session or camera.get("session"))
            and (args.revision or camera.get("revision"))):
        discover_match(args, profile)
    epoch = args.epoch or camera.get("epoch") or (
        uuid.uuid4().hex if args.one_shot else args.source_session or args.camera_session or camera["session"])
    source = field_command(args, profile, run, python, epoch, continuous=not args.one_shot)
    # Keep a second attempt out before its UDP producer can publish into an existing match.
    with (run / "field-source.lock").open("a") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with (run / ("field-" + uuid.uuid4().hex + ".log")).open("x") as log:
            def interrupted(signum, frame):
                raise KeyboardInterrupt
            previous = {sig: signal.signal(sig, interrupted) for sig in (signal.SIGHUP, signal.SIGTERM)}
            try:
                return start_pair(cmd, source, log, env, epoch)
            finally:
                for sig, handler in previous.items():
                    signal.signal(sig, handler)


def start_pair(cmd, source, log, env, epoch):
    # Only start publishing field intents once THIS runner acquired its own lock and opened disarmed.
    import selectors
    runner = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, bufsize=0)
    producer = None
    try:
        with selectors.DefaultSelector() as poll:
            poll.register(runner.stdout, selectors.EVENT_READ)
            deadline = time.monotonic() + 20
            while producer is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not poll.select(remaining):
                    raise ValueError("runner did not open; check walker activation and the front camera")
                line = runner.stdout.readline()
                if not line:
                    raise ValueError("runner refused the session; see its diagnostic")
                sys.stdout.buffer.write(line)
                sys.stdout.flush()
                try:
                    ready = read_event(line) == "ready"
                except (ValueError, AttributeError):
                    ready = False  # Preceding library notices do not establish readiness.
                if ready:
                    producer = subprocess.Popen(source, env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=log)
            print("Field input started. To start this attempt use " + json.dumps({"command": "arm", "epoch": epoch}),
                  file=sys.stderr, flush=True)
            stopping = None
            while runner.poll() is None:
                if producer.poll() is not None and stopping is None:
                    runner.send_signal(signal.SIGINT)
                    stopping = time.monotonic()
                if stopping is not None and time.monotonic() - stopping > 8:
                    raise ValueError("runner did not finish Stop after the field source exited")
                if poll.select(.2):
                    line = runner.stdout.readline()
                    if not line:
                        break
                    sys.stdout.buffer.write(line)
                    sys.stdout.flush()
        sys.stdout.buffer.write(runner.stdout.read())
        sys.stdout.flush()
        if producer.returncode:
            raise ValueError("field input exited unexpectedly; see " + log.name)
        return runner.wait(timeout=8)
    finally:
        # Both children also receive terminal interrupt. Stop only these owned processes.
        for child in (runner, producer):
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    child.wait(timeout=5)


def read_event(line):
    message = json.loads(line)
    return message.get("event") if message.get("armed") is False else None


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("action", choices=("prepare", "home", "match", "field", "describe", "operator"))
    p.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    p.add_argument("--robot-id", choices=("pupper-1", "pupper-2"))
    p.add_argument("--supported", action="store_true")
    p.add_argument("--dashboard", action="store_true", help="report the live standing cue to the laptop setup screen")
    p.add_argument("--home-only", action="store_true", help="home on the stand without requesting walker Start")
    p.add_argument("--runner-only", action="store_true")
    p.add_argument("--connect", action="store_true", help="Prepare on a cold dashboard connection; prompt before Home")
    p.add_argument("--owner", help="Laptop owner of the dedicated operator connection")
    p.add_argument("--operator-owner", help="Standing owner attaching its match connection")
    p.add_argument("--camera-session")
    p.add_argument("--source-session")
    p.add_argument("--source-host")
    p.add_argument("--pitch", help="Measured floor length and width for this match")
    p.add_argument("--tracks", help="JSON mapping from camera track names to team IDs")
    p.add_argument("--epoch")
    p.add_argument("--revision", type=int)
    p.add_argument("--one-shot", action="store_true", help="legacy field source for one supervised kick trial")
    p.add_argument("--duration", type=int, choices=range(1,3601), metavar="SECONDS")
    args = p.parse_args(argv)
    if args.duration is None:
        args.duration = 3600 if args.action == "field" else 60
    if args.dashboard and (args.action not in ("prepare", "home")
                           or (args.action == "home" and not args.supported)):
        p.error("--dashboard is for ordinary prepare and supported home")
    if args.home_only and (args.action != "home" or not args.supported):
        p.error("home-only requires home --supported --home-only")
    if (args.supported and args.action != "home") or (args.runner_only and args.action != "match"):
        p.error("--supported is for home and --runner-only is for match")
    profile_path = args.profile.expanduser().resolve(strict=True)
    profile = read(profile_path)
    template = read(profile["platform_template"])
    check_profile(profile, args.robot_id, template)
    if args.action == "describe":
        print(json.dumps(describe_profile(profile, read(profile["field_config"]))))
        return 0
    if args.connect and (args.action != "match" or not args.runner_only):
        p.error("--connect requires match --runner-only")
    if args.pitch is not None and args.action != "field":
        p.error("--pitch belongs to a match field feed")
    if args.action == "field" and not args.epoch:
        p.error("field needs the current match epoch")
    from fc_operator import attached, valid_owner
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    if (args.action == "operator" and not valid_owner(args.owner)) or (args.owner is not None and args.action != "operator"):
        p.error("operator requires its browser owner")
    if args.operator_owner is not None and (not args.connect or not valid_owner(args.operator_owner)):
        p.error("--operator-owner requires a valid owner and match --runner-only --connect")
    env = environment(template, profile)
    fc_control = env.get("PUPPER_FC_OPERATOR_CONTROL") == "1"
    if (args.action == "operator" or args.operator_owner) and not fc_control:
        raise ValueError("Selected platform does not provide laptop Stop")
    boot = (PROC / "sys/kernel/random/boot_id").read_text().strip()
    python = template["interpreter"]["executable"]
    helper = ROOT / "foundation/hardware/persistent_session.py"
    with profile_path.with_suffix(".lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        run, plan = acquire_run("prepare" if args.connect else args.action, args, profile_path, profile,
                                template, env, boot, python, helper)
        previous_fault = None
        # Home checked its board before its probe; every other started run is checked here.
        if (run / "owned_session.json").is_file() and args.action != "home":
            previous_fault = terminal_reason(run)
            if previous_fault is None:
                check_manager(run, plan, boot)
            elif not ((args.connect and not args.operator_owner) or args.action == "prepare"):
                raise ValueError(previous_fault + ". Support the robot and reconnect for a fresh Home.")
            else:
                # The board already sent neutral or its manager is gone. Release only this
                # exact terminal session; the next Home is a separate supported request.
                subprocess.run([python, "-B", "-s", helper, "shutdown", "--run-dir", run], env=env,
                               check=True, stdout=subprocess.DEVNULL)
                run, plan = acquire_run("prepare", args, profile_path, profile, template, env, boot, python, helper)
        homed = (run / "owned_session.json").is_file() and (run / "native-readiness.json").is_file()
        if args.connect:
            if args.operator_owner and not attached(run, plan["session_id"], args.operator_owner):
                raise ValueError("The standing operator no longer owns this robot")
            if not homed or (run / "awaiting-standing.json").exists() or (fc_control and not args.operator_owner):
                print(json.dumps(dict(event="home_required", robot_id=profile["robot_id"], run=str(run),
                                      imu_recovery="strict", match_capable=True,
                                      fc_operator_control=fc_control, homed=homed, previous_fault=previous_fault,
                                      joystick_listed=any(Path("/dev/input").glob("js[0-9]*")))),
                      flush=True)
                return 0
        if args.action == "match":
            start_deferred_perception(run, python, helper, env)
    if args.action == "operator":
        os.execve(python, [python, "-B", "-s", str(Path(__file__).resolve()), "--serve-fc-operator",
                           str(run), profile["robot_id"], args.owner], {**env, **plan["environment"]})
    if args.action == "field":
        cmd = field_command(args, profile, run, python, args.epoch, continuous=True)
        cmd.append("--stop-on-eof")
        env.update(plan["environment"])
        with (run / "field-source.lock").open("a") as owner:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            os.set_inheritable(owner.fileno(), True)
            os.execve(python, cmd, env)
    if args.action == "match":
        return run_match(args, profile, plan, run, python, env)
    print(json.dumps({"run": str(run), "robot_id": profile["robot_id"], "stage": args.action,
                      "state": "session prepared" if args.action == "prepare" else "homed, hardware active; match disarmed",
                      "match_armed": False, "home_only": args.home_only, "fc_operator_control": fc_control,
                      "homed": homed, "previous_fault": previous_fault, "imu_recovery": "strict"}))
    return 0


def service(node, rclpy, service_type, endpoint, *, request=None, timeout=2):
    client = node.create_client(service_type, endpoint)
    try:
        if not client.wait_for_service(timeout_sec=2):
            raise ValueError("service unavailable: " + endpoint)
        reply = client.call_async(request if request is not None else service_type.Request())
        rclpy.spin_until_future_complete(node, reply, timeout_sec=timeout)
        if not reply.done() or reply.result() is None:
            raise ValueError("no response from " + endpoint)
        return reply.result()
    finally:
        node.destroy_client(client)


def supervised_stand(node, rclpy, *, dashboard=False):
    import select
    import termios
    from controller_manager_msgs.srv import ListControllers
    from software_start import attach
    def state():
        response = service(node, rclpy, ListControllers, "/controller_manager/list_controllers")
        inventory = {c.name: c.state for c in response.controller}
        if inventory.get("trick_leg_kick") != "inactive" or inventory.get("gemini_walk") not in ("active", "inactive"):
            raise ValueError("unexpected walker/kick controller inventory")
        return inventory["gemini_walk"]
    if state() == "active":
        return
    gateway = attach(node, selected_controller="gemini_walk")
    deadline = time.monotonic() + 3
    while not (gateway.stop_capable and gateway.ready() and gateway.stop_publisher.get_subscription_count()):
        if time.monotonic() >= deadline:
            raise ValueError("walker Start/Stop gateway unavailable")
        rclpy.spin_once(node, timeout_sec=.04)
    if not sys.stdin.isatty():
        raise ValueError("walker inactive; use a supervised terminal for home --supported (ssh -t)")
    termios.tcflush(sys.stdin, termios.TCIFLUSH)
    if dashboard:
        print(json.dumps({"event": "stand_prompt"}), flush=True)
    else:
        cue = "enable standing"
        print(f"Keep Pupper supported. Press Enter to {cue}, or use gamepad Options.", file=sys.stderr, flush=True)
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
        if gateway.state and not gateway.state["latched"]:
            if state() == "active":
                return
        if select.select([sys.stdin], [], [], 0)[0]:
            if sys.stdin.readline() != "\n":
                raise ValueError("standing cancelled")
            ticket = gateway.capture(pressed_at=time.monotonic())
            try:
                gateway.send(ticket)
                while True:
                    rclpy.spin_once(node, timeout_sec=.02)
                    result = gateway.poll()
                    if result is not None:
                        if not result["success"] or state() != "active":
                            raise ValueError("standing Start refused or unconfirmed: " + result["message"])
                        return
            except BaseException:
                gateway.request_stop("standing Start interrupted or unconfirmed")
                until = time.monotonic() + 1.2
                while time.monotonic() < until and not gateway.poll_stop():
                    rclpy.spin_once(node, timeout_sec=.02)
                raise
    raise ValueError("standing input timed out; no software Start submitted")


def read_hardware(stand=False, *, dashboard=False):
    """Check current hardware state even when an old manager survived a sensor fault."""
    import rclpy
    from controller_manager_msgs.srv import ListHardwareComponents
    from rclpy.signals import SignalHandlerOptions
    # The standing probe owns the Stop/ack path on terminal interrupt. Keep
    # its DDS context live until the finally block releases the node.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO if stand else None)
    node = rclpy.create_node("fc_home_status", enable_rosout=False, start_parameter_services=False)
    try:
        hardware = service(node, rclpy, ListHardwareComponents, "/controller_manager/list_hardware_components")
        matches = [c.state.label for c in hardware.component if c.name == "control_board_hardware_interface"]
        if matches != ["active"]:
            raise ValueError("hardware is not active; inspect the motor fault before recovery")
        if stand:
            supervised_stand(node, rclpy, dashboard=dashboard)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        if len(sys.argv) == 5 and sys.argv[1] == "--serve-fc-operator":
            from fc_operator import serve
            from fc_walk_kick_release import verify_running_session
            run = Path(sys.argv[2]).resolve(strict=True)
            plan = read(run / "selected-session.json")
            if plan["environment"].get("PUPPER_FC_OPERATOR_CONTROL") != "1" or sys.argv[3] not in ("pupper-1", "pupper-2"):
                raise ValueError("This run has no selected laptop Stop gateway")
            check_manager(run, plan, (PROC / "sys/kernel/random/boot_id").read_text().strip())
            verify_running_session(run, ROOT, operator_controls=True)
            sys.exit(serve(run, sys.argv[3], sys.argv[4]))
        if sys.argv[1:2] in (["--dashboard-stand"], ["--supported-stand"]):
            signal.signal(signal.SIGHUP, signal.default_int_handler)
        if sys.argv[1:] in (["--read-hardware"], ["--supported-stand"], ["--dashboard-stand"]):
            sys.exit(read_hardware(sys.argv[1] != "--read-hardware", dashboard=sys.argv[1] == "--dashboard-stand"))
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
        print("Pupper FC: " + str(exc), file=sys.stderr)
        sys.exit(1)
