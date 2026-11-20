#!/usr/bin/env python3
"""Run the selected native walk → kick → walk wire check in isolated Linux ROS.

Uses the existing Jazzy offline native build; refuses a mismatch with current
controller source. Generates the actual release YAML, compiles only the memory
interface fixture, runs the production guard on DDS and checks the native model
inputs and applied clip receipt. No ROS controller manager or driver is started.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
CACHE = ROOT / "artifacts/pupper-fc/2026-11-17"
IMAGE = "cs123-ros-jazzy-slew-check:latest"
# Build-tree binary linked by the original strict-trial-native/source-pins.json.
BUILD_LIBRARY_SHA = "624dc560c03a1f2f2b0bab44580900d699fdabcce8a1557ac6acd49550180a75"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def compile_fixture():
    build = Path("/fresh-build")
    target = build / "CMakeFiles/test_neural_controller_safety.dir"

    def remap(text):
        return text.replace("/evidence/build", "/fresh-build").replace("/source/", "/native-source/")

    flags = []
    for line in (target / "flags.make").read_text().splitlines():
        if line.startswith(("CXX_DEFINES =", "CXX_INCLUDES =", "CXX_FLAGS =")):
            flags.extend(shlex.split(remap(line.split(" = ", 1)[1])))
    output = Path("/evidence")
    obj = str(output / "controllers.o")
    compile_cmd = ["/usr/bin/c++", *flags, "-c", str(HERE / "controllers.cpp"), "-o", obj]
    link = shlex.split(remap((target / "link.txt").read_text()))
    link = [obj if x.endswith(".cpp.o") else x for x in link]
    link[link.index("-o") + 1] = str(output / "controllers")
    for cmd, name in ((compile_cmd, "compile"), (link, "link")):
        with (output / (name + ".log")).open("w") as log:
            subprocess.run(cmd, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120)


def inside():
    from foundation.hardware.fc_walk_kick_release import ENV
    output = Path("/evidence")
    os.environ.update(ENV, PUPPER_GEMINI_DIR=str(ROOT / "gemini_pupper"),
                      PUPPER_GAIT_DEPLOY_DIR=str(ROOT / "pupper_gait_deploy"),
                      PUPPER_SESSION_MANIFEST=str(output / "runtime_manifest.json"))
    hw = ROOT / "foundation/hardware"
    spec = importlib.util.spec_from_file_location("selected_profile", hw / "lab6_session.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    launch.prepare_session_profile(hw / "lab6-session.yaml", hw / "floor_stock/baseline-expanded.urdf")
    compile_fixture()
    with (output / "native.log").open("w") as log:
        native = subprocess.Popen([str(output / "controllers"), "--gtest_filter=ProductionRuntime.SelectedWalkKickWire"],
                                  stdout=log, stderr=subprocess.STDOUT)
        try:
            subprocess.run([sys.executable, str(HERE / "client.py")], check=True, timeout=27)
            if native.wait(timeout=5):
                raise RuntimeError("native lifecycle test failed; see native.log")
        finally:
            if native.poll() is None:
                native.kill()
                native.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, default=CACHE, help="November 17 native/source and DDS dependency cache")
    parser.add_argument("--out", type=Path, help="new evidence directory (default: temporary directory)")
    parser.add_argument("--inside", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.inside:
        if os.environ.get("PUPPER_ISOLATED_ROS_TEST") != "1" or ROOT != Path("/repo"):
            parser.error("internal entry requires the isolated container")
        inside()
        return
    kit = args.cache.resolve() / "fc-deadline-source-kit-20261117T144254Z"
    build = kit / "native-build-evidence/build"
    src = kit / "extracted-native/neural_controller"
    deps = args.cache.resolve() / "ros-fc-executor-20261117T110500Z/dependencies"
    pins = {}
    for path in (ROOT / "pupper_gait_deploy/src").iterdir():
        if path.name in ("neural_controller.cpp", "neural_controller_parameters.yaml"):
            cached = src / "src" / path.name
        elif path.suffix == ".hpp":
            cached = src / "include/neural_controller" / path.name
        else:
            continue
        if digest(path) != digest(cached):
            raise ValueError(f"cached build source differs at {path.name}")
        pins[str(path.relative_to(ROOT))] = digest(path)
    lib = build / "libneural_controller.so"
    if digest(lib) != BUILD_LIBRARY_SHA:
        raise ValueError("cached native library differs from the original build receipt")
    pins["libneural_controller.so"] = digest(lib)
    for name in ("controllers.cpp", "client.py", "run.py"):
        pins[name] = digest(HERE / name)
    out = args.out.resolve() if args.out else Path(tempfile.mkdtemp(prefix="fc-native-wire-"))
    if args.out:
        out.mkdir(parents=True, exist_ok=False)
    pins["docker_image"] = subprocess.check_output(["docker", "image", "inspect", IMAGE, "--format", "{{.Id}}"],
                                                   text=True).strip()
    (out / "pins.json").write_text(json.dumps(pins, indent=2) + "\n")
    container_tool = "/repo/final_project/behaviors/tools/walk_kick_wire/run.py"
    shell = ('source /opt/ros/jazzy/setup.bash && '
             'export PYTHONPATH=/repo:/deps/gateway-runtime/python:"$PYTHONPATH" '
             'LD_LIBRARY_PATH=/fresh-build:/deps/gateway-runtime:"$LD_LIBRARY_PATH" && '
             'exec python3 ' + container_tool + ' --inside')
    cmd = ["docker", "run", "--rm", "--network", "none", "--cpus", "3", "--memory", "4g",
           "--cap-drop", "ALL", "--security-opt", "no-new-privileges", "-e", "ROS_DOMAIN_ID=141",
           "-e", "ROS_LOCALHOST_ONLY=1", "-e", "PUPPER_ISOLATED_ROS_TEST=1", "-e", "PYTHONDONTWRITEBYTECODE=1"]
    for host_path, target, mode in ((ROOT, "/repo", "ro"), (src.parent, "/native-source", "ro"),
                                    (build, "/fresh-build", "ro"), (deps, "/deps", "ro"), (out, "/evidence", "rw")):
        cmd.extend(["-v", f"{host_path}:{target}:{mode}"])
    print("Evidence directory", out, flush=True)
    subprocess.run([*cmd, IMAGE, "bash", "-c", shell], check=True, timeout=180)
    print((out / "result.json").read_text())


if __name__ == "__main__":
    main()
