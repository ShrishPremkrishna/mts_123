#!/usr/bin/env python3
"""Run the selected native walk -> kick -> restore and Stop chain without devices.

`execute` runs on Linux ROS Jazzy against two isolated builds produced by
foundation/hardware/build_native_candidate.sh: the neural controller and the
Stop gateway. Each build is accepted only when its installed-files.json has the
digest given on the command line and every prefix file still matches it. The
release YAML and runtime manifest come from the maintained release profile
(fc_walk_kick_release.py _profile). client.py then starts the selected gateway,
the production guard, operator relay and field runner and acts as browser,
overhead field, detector and strict board receipt.

Manager backends:
  ros2-control  real ros2_control_node and spawners with mock_components/GenericSystem
                memory hardware: the selected URDF with only its hardware plugin
                and driver parameters replaced. Use on the Pi target.
  fixture       controllers.cpp: the selected NeuralController in memory with
                fixture-driven lifecycle and IMU broadcast. Use where the manager is
                not installed (the Mac container).

`container` (Mac host) runs `execute --manager fixture` in the aarch64 Jazzy
image. Its ros2_control differs from the Pi and its builds are container-local;
it checks host logic, not the target binaries.

Scenarios:
  walk-kick-restore-stop     approach, kick, restored walking, then Stop
  stop-during-kick           Stop at the kick motion midpoint
  kick-entry-arena-latency   field captures dated by the selected arena camera
                             latency (final_project/pupper_fc/arena.json
                             camera.latency_s), as arena_camera.py dates live
                             frames. Its outcome (completed, refused or late kick
                             entry, runtime Stop) is recorded, not required.

Each scenario writes timing.json: CLOCK_MONOTONIC observations from the scenario,
runner and relay processes (and the fixture manager's native stamps), with
per-request budget, entry gates, service rounds and native entry evidence.

No driver, Home, motor, sensor, camera, service or installed/active profile is
used. Synthetic inputs are labeled in every result. Physical neutral delivery
and full manager/driver integration remain outside this check.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
IMAGE = "cs123-ros-jazzy-slew-check:latest"
SCENARIOS = ("walk-kick-restore-stop", "stop-during-kick", "kick-entry-arena-latency")
ARENA_CONFIG = "final_project/pupper_fc/arena.json"
# Production FC (168) and the build recipe's reserved test domains.
FORBIDDEN_DOMAINS = {0, 164, 165, 166, 168, 190}
HARDWARE_PROCESSES = {"ros2_control_node", "estop_controller", "joy_node", "joy_linux_node"}
SESSION_MARKERS = ("PUPPER_PROBE_SESSION_ID", "PUPPER_SESSION_MANIFEST", "PUPPER_FC_BOARD_HEALTH_PATH",
                   "PUPPER_FC_PREPARED_MANIFEST_SHA256", "ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER",
                   "ROS_AUTOMATIC_DISCOVERY_RANGE", "FASTRTPS_DEFAULT_PROFILES_FILE",
                   "FASTDDS_DEFAULT_PROFILES_FILE", "CYCLONEDDS_URI")
ALLOWED_DEVICES = {"/dev/null", "/dev/random", "/dev/urandom"}
PLATFORM_KEYS = ("PATH", "PYTHONPATH", "LD_LIBRARY_PATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH",
                 "ROS_DISTRO", "ROS_VERSION", "ROS_PYTHON_VERSION", "RMW_IMPLEMENTATION",
                 "HOME", "USER", "LANG", "LC_ALL")
# Checkout files the controller closure overlays (request overlay_origins) and the fixture includes.
CONTROLLER_OVERLAYS = {"pupper_gait_deploy/test/test_neural_controller_safety.cpp": "test/test_neural_controller_safety.cpp",
                       "pupper_gait_deploy/policies/default_policy.json": "launch/default_policy.json"}
GATEWAY_SOURCE = "pupperv3-monorepo/ros2_ws/src/joy_utils"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def controller_overlays():
    rows = dict(CONTROLLER_OVERLAYS)
    for path in sorted((ROOT / "pupper_gait_deploy/src").iterdir()):
        if path.suffix == ".hpp":
            rows[f"pupper_gait_deploy/src/{path.name}"] = f"include/neural_controller/{path.name}"
        elif path.name in ("neural_controller.cpp", "neural_controller_parameters.yaml"):
            rows[f"pupper_gait_deploy/src/{path.name}"] = f"src/{path.name}"
        elif path.name in ("CMakeLists.txt", "package.xml"):
            rows[f"pupper_gait_deploy/src/{path.name}"] = path.name
    return rows


def verify_build(build, installed_sha, package):
    """Accept one recipe output only when it is complete and its prefix still matches its receipt."""
    build = Path(build).resolve(strict=True)
    if (build / "exit-code").read_text().strip() != "0":
        raise ValueError(f"{package} build did not complete")
    installed = build / "installed-files.json"
    if digest(installed) != installed_sha:
        raise ValueError(f"{package} installed-files.json differs from the selected build receipt")
    expected = json.loads(installed.read_text())
    prefix = build / "prefix"
    actual = {p.relative_to(prefix).as_posix(): digest(p) for p in sorted(prefix.rglob("*")) if p.is_file()}
    if actual != expected:
        changed = sorted(set(actual.items()) ^ set(expected.items()))[:4]
        raise ValueError(f"{package} prefix differs from its installed-files receipt: {changed}")
    closure = json.loads((build / "source-closure.json").read_text())
    if closure.get("package") != package:
        raise ValueError(f"{build} is not a {package} build")
    return {"build": str(build), "prefix": str(prefix), "installed_files_sha256": installed_sha,
            "source_closure_sha256": digest(build / "source-closure.json"),
            "source_commit": closure["source_commit"], "source_files": len(closure["files"]),
            "recipe_sha256": (build / "recipe.sha256").read_text().split()[0], "closure": closure["files"]}


def checkout_matches(closure, pairs):
    """The executing checkout must be the source this build compiled."""
    rows, bad = {}, []
    for checkout, member in pairs.items():
        actual = digest(ROOT / checkout)
        rows[checkout] = actual
        if closure.get(member) != actual:
            bad.append(checkout)
    if bad:
        raise ValueError("checkout differs from the selected build closure: " + ", ".join(bad))
    return rows


def refuse_device_context(domain, manager):
    """Reject a normal robot session or system profile instead of trusting file names."""
    if not sys.platform.startswith("linux"):
        raise ValueError("execute runs only under Linux ROS")
    if domain in FORBIDDEN_DOMAINS or not 1 <= domain <= 101:
        raise ValueError(f"ROS domain {domain} is reserved; choose an unused isolated domain")
    # LOCALHOST only narrows discovery; children never inherit discovery settings either way.
    present = [key for key in SESSION_MARKERS if os.environ.get(key)
               and not (key == "ROS_AUTOMATIC_DISCOVERY_RANGE" and os.environ[key] == "LOCALHOST")]
    if present:
        raise ValueError("invoked from a robot session or discovery profile: " + ", ".join(present))
    if os.environ.get("AMENT_PREFIX_PATH") != "/opt/ros/jazzy":
        raise ValueError("source only /opt/ros/jazzy; an installed robot workspace overlay is refused")
    if manager == "ros2-control" and os.geteuid() == 0:
        raise ValueError("refusing root on the target; device permissions must stay with the user session")
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            argv0 = (proc / "cmdline").read_bytes().split(b"\0")[0]
            if Path(os.fsdecode(argv0)).name in HARDWARE_PROCESSES:
                raise ValueError(f"a robot session process is running: pid {proc.name}; stop it first")
            if proc.stat().st_uid == os.getuid() and f"ROS_DOMAIN_ID={domain}".encode() in (
                    proc / "environ").read_bytes().split(b"\0"):
                raise ValueError(f"ROS domain {domain} is already in use by pid {proc.name}")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue


def child_environment(controller, gateway, domain, out):
    """A clean ROS environment: system underlay plus only the two selected private prefixes."""
    env = {k: os.environ[k] for k in PLATFORM_KEYS if k in os.environ}
    bindings = sorted(Path(gateway["prefix"]).glob("lib/python3*/*-packages"))
    if len(bindings) != 1 or not (bindings[0] / "joy_utils/srv").is_dir():
        raise ValueError("selected gateway prefix lacks its Python service bindings")
    prefixes = [controller["prefix"], gateway["prefix"]]

    def prepend(key, values):
        env[key] = ":".join([*values, *filter(None, env.get(key, "").split(":"))])

    prepend("AMENT_PREFIX_PATH", prefixes)
    prepend("CMAKE_PREFIX_PATH", prefixes)
    prepend("LD_LIBRARY_PATH", [p + "/lib" for p in prefixes])
    prepend("PYTHONPATH", [str(bindings[0]), str(ROOT)])
    env.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1", PUPPER_ISOLATED_ROS_TEST="1",
               PYTHONDONTWRITEBYTECODE="1", ROS_HOME=str(out / "ros-home"), ROS_LOG_DIR=str(out / "ros-logs"))
    return env


def release_environment(env, run):
    """The release profile's own selections, in this run directory and isolated domain."""
    sys.path.insert(0, str(ROOT / "foundation/hardware"))
    from fc_walk_kick_release import ENV
    selected = dict(env, **{k: v for k, v in ENV.items() if k != "ROS_DOMAIN_ID"})
    selected.update(PUPPER_GEMINI_DIR=str(ROOT / "gemini_pupper"), PUPPER_GAIT_DEPLOY_DIR=str(ROOT / "pupper_gait_deploy"),
                    PUPPER_SESSION_MANIFEST=str(run / "runtime-manifest.json"),
                    PUPPER_FC_BOARD_HEALTH_PATH=str(run / "fc-board-health.bin"),
                    PUPPER_FC_RECORDING="events", PUPPER_FC_IMU_RECOVERY="strict", PUPPER_FC_OPERATOR_CONTROL="1",
                    FOLLOW_USE_COLOR="0", FOLLOW_AVOID_OBSTACLES="0", FOLLOW_ALWAYS_DETECT="0")
    return selected


def compile_fixture(controller, out):
    """Compile controllers.cpp with the selected build's own safety-test flags; link its tree."""
    build = Path(controller["build"]) / "build"
    target = build / "CMakeFiles/test_neural_controller_safety.dir"
    flags = []
    for line in (target / "flags.make").read_text().splitlines():
        if line.startswith(("CXX_DEFINES =", "CXX_INCLUDES =", "CXX_FLAGS =")):
            flags.extend(line.split(" = ", 1)[1].split())
    flags += ["-isystem", "/opt/ros/jazzy/include/controller_manager_msgs"]
    obj = out / "controllers.o"
    link = (target / "link.txt").read_text().split()
    link = [str(obj) if x.endswith(".cpp.o") else x for x in link]
    link[link.index("-o") + 1] = str(out / "controllers")
    link += [f"/opt/ros/jazzy/lib/libcontroller_manager_msgs__rosidl_{kind}.so"
             for kind in ("typesupport_cpp", "generator_c", "typesupport_fastrtps_cpp")]
    for name, cmd in (("compile", ["/usr/bin/c++", *flags, "-c", str(HERE / "controllers.cpp"), "-o", str(obj)]),
                      ("link", link)):
        with (out / f"fixture-{name}.log").open("w") as log:
            subprocess.run(cmd, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True)
    return {"binary": str(out / "controllers"), "sha256": digest(out / "controllers"),
            "source_sha256": digest(HERE / "controllers.cpp")}


def strace_command(trace_dir):
    strace = shutil.which("strace")
    if strace is None:
        return None
    seccomp = "--seccomp-bpf" in subprocess.run([strace, "-h"], capture_output=True, text=True).stdout
    trace_dir.mkdir()
    return [strace, "-f", "-ff", *(["--seccomp-bpf"] if seccomp else []), "-e", "trace=%file",
            "-o", str(trace_dir / "syscalls")]


def audit_trace(trace_dir):
    devices = set()
    files = sorted(trace_dir.glob("syscalls.*"))
    for path in files:
        for name in re.findall(r'"(/dev/[^"\n]*)"', path.read_text(errors="replace")):
            if name not in ALLOWED_DEVICES and not name.startswith("/dev/shm/"):
                devices.add(name)
    return {"trace_files": len(files), "unexpected_devices": sorted(devices)}


def execute(args):
    refuse_device_context(args.domain, args.manager)
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    controller = verify_build(args.controller_build, args.controller_installed_sha256, "neural_controller")
    gateway = verify_build(args.gateway_build, args.gateway_installed_sha256, "joy_utils")
    tool = {name: digest(HERE / name) for name in ("run.py", "client.py", "controllers.cpp", "test_walk_kick_wire.py")}
    identity = {
        "controller": {k: v for k, v in controller.items() if k != "closure"},
        "gateway": {k: v for k, v in gateway.items() if k != "closure"},
        "controller_checkout": checkout_matches(controller["closure"], controller_overlays()),
        "gateway_checkout": checkout_matches(gateway["closure"], {
            f"{GATEWAY_SOURCE}/{rel}": rel for rel in gateway["closure"]}),
        "native_library_sha256": json.loads(Path(controller["build"], "installed-files.json").read_text())[
            "lib/libneural_controller.so"],
        "gateway_executable_sha256": json.loads(Path(gateway["build"], "installed-files.json").read_text())[
            "lib/joy_utils/estop_controller"],
        "tool": tool, "manager": args.manager, "domain": args.domain, "python": sys.version.split()[0],
        "machine": os.uname().machine, "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
    }
    hardware_interface = Path("/opt/ros/jazzy/share/hardware_interface/package.xml")
    identity["ros2_control_version"] = re.search(r"<version>([^<]+)", hardware_interface.read_text()).group(1)
    if args.manager == "ros2-control":
        manager = Path("/opt/ros/jazzy/lib/controller_manager/ros2_control_node")
        identity["manager_executable"] = {"path": str(manager), "sha256": digest(manager)}
    else:
        identity["fixture"] = compile_fixture(controller, out)
    (out / "identity.json").write_text(json.dumps(identity, indent=2) + "\n")
    env = child_environment(controller, gateway, args.domain, out)
    results, failed = {}, False
    for scenario in args.scenario:
        run = out / scenario
        run.mkdir()
        selected = release_environment(env, run)
        # The maintained release preparation: file-only profile, manifest and controller YAML.
        profile = subprocess.run([sys.executable, "-B", "-s", str(ROOT / "foundation/hardware/fc_walk_kick_release.py"),
                                  "_profile"], env=selected, cwd=ROOT, capture_output=True, text=True)
        (run / "profile.log").write_text(profile.stderr)
        if profile.returncode:
            raise ValueError("selected release profile failed; see " + str(run / "profile.log"))
        (run / "profile.json").write_text(profile.stdout)
        spec = {"scenario": scenario, "side": args.side, "manager": args.manager, "run": str(run),
                "env": selected, "controller_prefix": controller["prefix"], "gateway_prefix": gateway["prefix"],
                "native_library_sha256": identity["native_library_sha256"],
                "gateway_executable_sha256": identity["gateway_executable_sha256"],
                "fixture": identity.get("fixture", {}).get("binary"), **capture_dating(scenario)}
        (run / "spec.json").write_text(json.dumps(spec, indent=2) + "\n")
        trace = strace_command(run / "trace") if not args.no_trace else None
        if trace is None and args.manager == "ros2-control":
            raise ValueError("target runs require strace for the file-syscall device audit")
        cmd = [*(trace or []), sys.executable, "-B", "-s", str(HERE / "client.py"), "scenario", str(run / "spec.json")]
        print(f"[{scenario}] starting; evidence {run}", flush=True)
        child = subprocess.Popen(cmd, env=selected, cwd=ROOT, stdin=subprocess.DEVNULL, start_new_session=True)
        try:
            code = child.wait()
        except KeyboardInterrupt:
            # The scenario owns its children and reaps them on SIGINT.
            os.killpg(child.pid, signal.SIGINT)
            code = child.wait()
            raise
        result = json.loads((run / "result.json").read_text()) if (run / "result.json").is_file() else {}
        result.setdefault("timing", {"error": "scenario wrote no timing"})
        result["exit_code"] = code
        result["syscall_audit"] = audit_trace(run / "trace") if trace else {
            "trace_files": 0, "unexpected_devices": None,
            "note": "strace unavailable here; container exposes only Docker's default /dev nodes"}
        passed = code == 0 and result.get("passed") is True and not result["syscall_audit"]["unexpected_devices"]
        result["passed"] = passed
        failed |= not passed
        results[scenario] = result
        (run / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        print(f"[{scenario}] {'passed' if passed else 'FAILED'} (exit {code})", flush=True)
    summary = {"passed": not failed, "manager": args.manager, "identity": "identity.json",
               "scenarios": {k: {"passed": v["passed"], "exit_code": v["exit_code"], "failure": v.get("failure"),
                                 "outcome": (v.get("checks") or {}).get("outcome"),
                                 "runtime_stop": (v.get("checks") or {}).get("runtime_stop"), "checks": v.get("checks"),
                                 "children": v.get("children"), "syscall_audit": v["syscall_audit"],
                                 "field_capture_dating": v.get("field_capture_dating"), "timing": v["timing"]}
                             for k, v in results.items()},
               "boundary": boundary(args.manager)}
    (out / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k: summary[k] for k in ("passed", "manager")} | {
        "scenarios": {k: v["passed"] for k, v in summary["scenarios"].items()}, "evidence": str(out)}))
    return 0 if not failed else 1


def capture_dating(scenario):
    """The selected arena camera latency for the latency scenario; zero (dated at publication) otherwise."""
    if scenario != "kick-entry-arena-latency":
        return {"capture_latency_s": 0.}
    latency = json.loads((ROOT / ARENA_CONFIG).read_text())["camera"]["latency_s"]
    return {"capture_latency_s": latency,
            "capture_latency_source": {"path": ARENA_CONFIG, "sha256": digest(ROOT / ARENA_CONFIG),
                                       "key": "camera.latency_s",
                                       "intent_lease": "overhead_ingress.MAX_FRAME_AGE_S (live_field_attack lease)"}}


def boundary(manager):
    lifecycle = ("real ros2_control_node, spawners and switch semantics with mock_components/GenericSystem memory "
                 "hardware (selected URDF, hardware plugin and driver parameters replaced)" if manager == "ros2-control"
                 else "fixture-driven init/activate/deactivate, update schedule and IMU broadcast (controllers.cpp)")
    return {"exercised": ["selected neural controller library, release YAML and policies",
                          "selected gateway executable, FcStartMotion, heartbeat lease and Stop latch",
                          "production body guard, operator relay, Robot/CombinedRuntime field runner", lifecycle],
            "synthetic": ["browser heartbeats and Stand", "overhead field frames and intents", "detector frames",
                          "strict board health receipt (driver not loaded)",
                          "field capture dating: publication time, or publication minus the selected arena latency"],
            "instrumented": ["pass-through CLOCK_MONOTONIC probes in the runner and relay processes (client.PROBES)",
                             *(["fixture manager native stamps around on_activate/on_deactivate and first update"]
                               if manager == "fixture" else []),
                             "native command_authority activation/sample times as published by the controller"],
            "substituted": ["persistent-session supervision checks (verify_running_session, check_manager) "
                            "replaced by this harness's own identity checks"],
            "not_exercised": ["control board driver, IMU hardware, Home", "physical neutral or damping delivery",
                              "physical walking, kick contact or stillness", "camera, sensors, services, active profile"]}


def container(args):
    """Mac host: run execute in the container against container builds of the same closures."""
    native = args.native_root.resolve(strict=True)
    out = args.out.resolve()
    if out.exists():
        raise ValueError("output must be new")
    out.parent.mkdir(parents=True, exist_ok=True)
    shas = {name: digest(native / name / "installed-files.json") for name in ("controller", "gateway")}
    image = subprocess.check_output(["docker", "image", "inspect", IMAGE, "--format", "{{.Id}}"], text=True).strip()
    inner = ["python3", "-B", "-s", "/repo/final_project/behaviors/tools/walk_kick_wire/run.py", "execute",
             "--manager", "fixture", "--domain", str(args.domain), "--side", args.side, "--no-trace",
             "--controller-build", "/native/controller", "--controller-installed-sha256", shas["controller"],
             "--gateway-build", "/native/gateway", "--gateway-installed-sha256", shas["gateway"],
             "--out", f"/evidence/{out.name}", *[x for s in args.scenario for x in ("--scenario", s)]]
    shell = "source /opt/ros/jazzy/setup.bash && exec " + " ".join(inner)
    cmd = ["docker", "run", "--rm", "--init", "--network", "none", "--cpus", "4", "--memory", "6g",
           "--cap-drop", "ALL", "--security-opt", "no-new-privileges", "-e", "PYTHONDONTWRITEBYTECODE=1",
           "-e", "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"]  # the image defaults to SUBNET
    # Builds embed absolute paths; mount them where the recipe produced them.
    for host, target, mode in ((ROOT, "/repo", "ro"), (native, "/native", "ro"),
                               (args.controller_input.resolve(strict=True), "/inputs/controller", "ro"),
                               (args.gateway_input.resolve(strict=True), "/inputs/gateway", "ro"),
                               (out.parent, "/evidence", "rw")):
        cmd += ["-v", f"{host}:{target}:{mode}"]
    print("Container image", image, "evidence", out, flush=True)
    code = subprocess.run([*cmd, IMAGE, "bash", "-c", shell]).returncode
    if (out / "result.json").is_file():
        summary = json.loads((out / "result.json").read_text())
        summary.update(environment="Mac aarch64 Jazzy container; container-local builds; not target evidence",
                       docker_image=image, exit_code=code)
        (out / "result.json").write_text(json.dumps(summary, indent=2) + "\n")
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="action", required=True)
    run = commands.add_parser("execute", help="Linux ROS: run the scenarios against verified builds")
    run.add_argument("--manager", choices=("ros2-control", "fixture"), required=True)
    for name in ("controller", "gateway"):
        run.add_argument(f"--{name}-build", type=Path, required=True, help=f"{name} recipe output directory")
        run.add_argument(f"--{name}-installed-sha256", required=True, help="digest of its installed-files.json receipt")
    run.add_argument("--no-trace", action="store_true", help="fixture backend only: no strace available")
    host = commands.add_parser("container", help="Mac host: fixture backend in the Jazzy container")
    host.add_argument("--native-root", type=Path, required=True, help="directory with controller/ and gateway/ builds")
    host.add_argument("--controller-input", type=Path, required=True, help="controller closure parent mounted at /inputs/controller")
    host.add_argument("--gateway-input", type=Path, required=True, help="gateway closure parent mounted at /inputs/gateway")
    for sub in (run, host):
        sub.add_argument("--out", type=Path, required=True, help="new evidence directory")
        sub.add_argument("--domain", type=int, default=94, help="isolated ROS domain")
        sub.add_argument("--side", choices=("left", "right"), default="right")
        sub.add_argument("--scenario", action="append", choices=SCENARIOS)
    args = parser.parse_args()
    args.scenario = args.scenario or list(SCENARIOS)
    if args.action == "execute" and args.no_trace and args.manager != "fixture":
        parser.error("the ros2-control target run requires the syscall trace")
    try:
        return execute(args) if args.action == "execute" else container(args)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print("walk_kick_wire: " + str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
