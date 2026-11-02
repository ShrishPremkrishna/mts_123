#!/usr/bin/env python3
"""Build real ROS controllers and fake-I2C tests locally, without robot access.

Requires a running Docker engine and the image described in ros-offline.Dockerfile.
Uses an immutable staged copy, networking disabled, and no host device mounts.
Never starts controller_manager or the hardware driver. Does not install on Pi.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

IMAGE = "cs123-ros-jazzy-slew-check:latest"
RUN = r'''#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=166 ROS_LOCALHOST_ONLY=1
dpkg-query -W 'ros-jazzy-*' > /output/ros-packages.txt
uname -a > /output/platform.txt
cmake -S /input/pupperv3-monorepo/ros2_ws/src/neural_controller -B /output/neural-build -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo '-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-O2 -g -UNDEBUG' > /output/neural-configure.log 2>&1
cmake --build /output/neural-build --parallel 2 > /output/neural-build.log 2>&1
ctest --test-dir /output/neural-build --no-tests=error --output-on-failure --output-junit /output/neural-tests.xml -R '^(gait_reference_parity|mixed_jump_reference_parity|command_filter|test_neural_controller_safety|reference_execution|position_target_slew)$' > /output/neural-tests.log 2>&1
cmake -S /input/pupperv3-monorepo/ros2_ws/src/joy_utils -B /output/joy-build -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo '-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-O2 -g -UNDEBUG' > /output/joy-configure.log 2>&1
cmake --build /output/joy-build --parallel 2 > /output/joy-build.log 2>&1
ctest --test-dir /output/joy-build --no-tests=error --output-on-failure --output-junit /output/joy-tests.xml > /output/joy-tests.log 2>&1
g++ -std=c++17 -Wall -Wextra -Werror -pedantic /input/imu_diagnostic/diagnostic.cpp -o /output/bno_diagnostic > /output/imu-build.log 2>&1
g++ -std=c++17 -Wall -Wextra -Werror -pedantic /input/imu_diagnostic/test_diagnostic.cpp -o /output/test_diagnostic >> /output/imu-build.log 2>&1
/output/test_diagnostic > /output/imu-tests.log 2>&1
'''


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stage(root: Path, destination: Path) -> dict[str, str]:
    mono = destination / "pupperv3-monorepo"
    for name in ("neural_controller", "joy_utils"):
        shutil.copytree(root / "pupperv3-monorepo/ros2_ws/src" / name,
                        mono / "ros2_ws/src" / name,
                        ignore=shutil.ignore_patterns(".git", "*.backup", "__pycache__"))
    # Reuse deployment mappings, without executing install/rebuild or applying
    # machine-specific trims. Only the new staged package is modified.
    spec = importlib.util.spec_from_file_location("deploy_install", root / "pupper_gait_deploy/install.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for source, target, _ in module.file_mappings(mono):
        if not source.exists():
            if source.name in ("default_policy.json", "test_policy.json", "mixed_jump_golden.json"):
                continue
            raise FileNotFoundError(source)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    shutil.copytree(root / "foundation/hardware/imu_diagnostic", destination / "imu_diagnostic",
                    ignore=shutil.ignore_patterns("__pycache__"))
    return {str(p.relative_to(destination)): digest(p)
            for p in sorted(destination.rglob("*")) if p.is_file()}


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", default=IMAGE)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timeout", type=int, default=1800, help="Total container seconds; default 1800")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    output = (args.output or root / "artifacts/validation" / ("ros-" + stamp)).resolve()
    # Refuse reused directories: failed runs and source snapshots stay intact.
    try:
        output.mkdir(parents=True, exist_ok=False)
    except OSError as exc:
        print(f"Cannot create fresh output directory {output}: {exc}. Use --output with a new writable path.", file=sys.stderr)
        return 2
    receipt = {"created_utc": stamp, "passed": False, "hardware": False,
               "scope": "Linux ARM ROS memory-interface and fake-I2C checks; no physical validation"}
    destination = output / "checks.json"
    container = "cs123-offline-" + stamp.lower()
    started = time.monotonic()
    container_may_be_running = False
    try:
        image = subprocess.run(["docker", "image", "inspect", args.image, "--format", "{{.Id}}"],
                               text=True, capture_output=True, timeout=20, check=True).stdout.strip()
        receipt["image_id"] = image
        receipt["git_revision"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True, timeout=10).strip()
        receipt["runner_sha256"] = digest(Path(__file__))
        receipt["staged_sha256"] = stage(root, output / "input")
        (output / "output").mkdir()
        (output / "run.sh").write_text(RUN)
        command = ["docker", "run", "--rm", "--name", container, "--network", "none",
                   "--cpus", "2", "--memory", "4g", "--cap-drop", "ALL",
                   "--security-opt", "no-new-privileges",
                   "--mount", f"type=bind,src={output / 'input'},dst=/input,readonly",
                   "--mount", f"type=bind,src={output / 'output'},dst=/output",
                   "--mount", f"type=bind,src={output / 'run.sh'},dst=/run-check.sh,readonly",
                   image, "bash", "/run-check.sh"]
        receipt["command"] = command
        destination.write_text(json.dumps(receipt, indent=2) + "\n")
        print(f"Running offline ROS checks; logs: {output}", flush=True)
        with (output / "docker.log").open("w") as log:
            container_may_be_running = True
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
            container_may_be_running = False
        receipt["returncode"] = result.returncode
        if result.returncode == 0:
            expected = {
                "neural-tests.xml": {"gait_reference_parity", "mixed_jump_reference_parity", "command_filter",
                                     "test_neural_controller_safety", "reference_execution", "position_target_slew"},
                "joy-tests.xml": {"stop_arbiter_deterministic", "stop_inventory_freshness",
                                  "estop_parameters_default", "estop_parameters_overrides",
                                  *("stop_gateway_" + x for x in ("delayed", "timeout", "shutdown", "service_loss", "negative", "broadcaster_type"))},
            }
            receipt["test_suites"] = {}
            for name, required in expected.items():
                cases = list(ET.parse(output / "output" / name).getroot().iter("testcase"))
                names = {case.attrib["name"] for case in cases}
                if not required.issubset(names) or any(case.find(tag) is not None for case in cases for tag in ("failure", "error", "skipped")):
                    raise RuntimeError(f"Missing/failed/skipped required cases in {name}")
                receipt["test_suites"][name] = sorted(names)
            gtest = ET.parse(output / "output/neural-build/test_results/neural_controller/test_neural_controller_safety.gtest.xml").getroot()
            cases = list(gtest.iter("testcase"))
            required = {"StaticLoopRequestPreservesFadeFullClockAndAppliedReceipt", "MovingLoopKeepsBoundaryAndStopPreemptsStaticRequest"}
            if not required.issubset({c.attrib["name"] for c in cases}) or any(c.find(tag) is not None for c in cases for tag in ("failure", "error", "skipped")):
                raise RuntimeError("Required production transition tests missing, failed or skipped")
            receipt["production_gtest"] = gtest.attrib
            changed = [name for name, value in receipt["staged_sha256"].items() if digest(output / "input" / name) != value]
            if changed:
                raise RuntimeError(f"Staged inputs changed during tests: {changed}")
            receipt["passed"] = True
    except (Exception, KeyboardInterrupt) as exc:
        receipt["error"] = f"{type(exc).__name__}: {exc}"
        # The only container we can stop is the unique one owned by this run.
        if container_may_be_running:
            try:
                cleanup = subprocess.run(["docker", "rm", "-f", container], capture_output=True, timeout=15)
                receipt["cleanup_returncode"] = cleanup.returncode
                receipt["cleanup_uncertain"] = cleanup.returncode != 0
            except (OSError, subprocess.TimeoutExpired):
                receipt["cleanup_uncertain"] = True
        print(receipt["error"], file=sys.stderr)
    finally:
        receipt["seconds"] = time.monotonic() - started
        destination.write_text(json.dumps(receipt, indent=2) + "\n")
    print(destination)
    return 0 if receipt["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
