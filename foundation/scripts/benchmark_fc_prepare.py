#!/usr/bin/env python3
"""Time two motor-inactive Prepare commands and construction of the Home launch description.

Run in Linux ROS with the same native overlay used for offline qualification.
Use a staged release, the ROS workspace's install/ directory and the approved HEF.
The temporary profile uses the real boot ID and native file digests. This script
never executes the launch description, opens hardware or changes enrollment.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def benchmark(root, install, detector, temporary):
    hardware = root / "foundation/hardware"
    sys.path.insert(0, str(hardware))
    from fc_enrollment import NATIVE_LIBS
    from fc_session import PLATFORM_KEYS

    environment = {key: os.environ[key] for key in PLATFORM_KEYS if key in os.environ}
    environment["PUPPER_SESSION_MOTION_MODE"] = "fc_walk_kick"
    components = {}
    for role, (package, filename) in NATIVE_LIBS.items():
        library = install / package / filename
        components[role] = {"path": str(library), "sha256": digest(library)}
    template = temporary / "template.json"
    template.write_text(json.dumps({
        "prepared": True, "controllers": ["gemini_walk", "trick_leg_kick"],
        "environment": environment, "components": components,
        "interpreter": {"executable": sys.executable}, "run_dir": str(temporary / "slot"),
    }))
    profile = temporary / "session.json"
    manifest = root / "app-stage-manifest.json"
    profile.write_text(json.dumps({
        "robot_id": "pupper-1", "detector_model": str(detector),
        "platform_template": str(template), "manifest_sha256": digest(manifest),
    }))

    timings, runs = [], []
    for _ in range(2):
        start = time.monotonic()
        reply = subprocess.check_output([sys.executable, "-B", str(hardware / "fc_session.py"),
                                         "prepare", "--profile", str(profile)], text=True)
        timings.append(round(time.monotonic() - start, 4))
        runs.append(Path(json.loads(reply)["run"]))
    if runs[0] != runs[1] or (runs[0] / "owned_session.json").exists():
        raise RuntimeError("Prepare did not reuse the same motor-inactive session")
    plan = json.loads((runs[0] / "selected-session.json").read_text())
    os.environ.update(plan["environment"])
    pinned = plan["runtime_manifest"]["sha256"]
    if os.environ["PUPPER_FC_PREPARED_MANIFEST_SHA256"] != pinned:
        raise RuntimeError("Home did not select the profile from Prepare")

    spec = importlib.util.spec_from_file_location("fc_bench_launch", hardware / "lab6_session.launch.py")
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    start = time.monotonic()
    description = launch.generate_launch_description()
    launch_seconds = time.monotonic() - start
    if digest(runs[0] / "runtime-manifest.json") != pinned:
        raise RuntimeError("Home changed the prepared profile")
    return {"source_commit": json.loads(manifest.read_text())["source_commit"],
            "cold_prepare_process_s": timings[0], "warm_prepare_process_s": timings[1],
            "home_launch_description_s": round(launch_seconds, 4), "actions_constructed": len(description.entities),
            "hardware_processes_started": 0}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release", required=True, type=Path)
    parser.add_argument("--install", required=True, type=Path)
    parser.add_argument("--detector-model", required=True, type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="fc-prepare-") as path:
        result = benchmark(args.release.resolve(strict=True), args.install.resolve(strict=True),
                           args.detector_model.resolve(strict=True), Path(path))
    print(json.dumps(result, indent=2))
