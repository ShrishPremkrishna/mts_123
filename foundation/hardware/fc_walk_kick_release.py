#!/usr/bin/env python3
"""Stage or verify the single-Pupper FC software profile. Never starts ROS or motors."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT / "gemini_pupper/robot"))
from stage_app import checked_path, verify_selected
from fc_release_profile import CONTROLLERS, FIELD_FILES, GUARD, GUARD_ARGS, KICK_DIR, MODE, TELEMETRY, WALK_MODEL

LAUNCH = "foundation/hardware/lab6_session.launch.py"
CONFIG = "foundation/hardware/lab6-session.yaml"
URDF = "foundation/hardware/floor_stock/baseline-expanded.urdf"
MANIFEST = "app-stage-manifest.json"
ENV = {"PUPPER_SESSION_MOTION_MODE": MODE, "PUPPER_SESSION_SKILLS": "leg_kick",
       "PUPPER_FOLLOWER_MODE": "disabled", "ROS_DOMAIN_ID": "168"}
SCHEMA = "pupper_fc_walk_kick_release_v1"
NATIVE_MARKERS = (b"fc_deadline_v1 requires walk-only gait velocity and no alternate input authority",
                  b"/neural_controller_default/command_authority",
                  b"interactive_clip_status_protocol", b"invalid execution_request_id; refusing activation")


def file_digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inspect_native(library):
    """Static requirements for a Pi candidate. Runtime ownership and receipts are checked separately."""
    path = Path(library).resolve(strict=True)
    data = path.read_bytes()
    # ELF64, little-endian, shared object (ET_DYN), EM_AARCH64.
    if data[:6] != b"\x7fELF\x02\x01" or data[16:20] != b"\x03\x00\xb7\x00":
        raise ValueError("combined release needs a 64-bit AArch64 controller shared object")
    if not all(marker in data for marker in NATIVE_MARKERS):
        raise ValueError("controller missing required native deadline or clip receipt markers")
    return {"path": str(path), "sha256": hashlib.sha256(data).hexdigest(),
            "inspection": "static AArch64 and capability markers; live activation unverified"}


def selected_paths(root):
    fixed = {*FIELD_FILES, LAUNCH, CONFIG, URDF, WALK_MODEL,
             "pupper_gait_deploy/download_policy.py",
             "gemini_pupper/lab/prompt.md",
             *(f"{KICK_DIR}/{name}" for name in ("artifact_manifest.json", "trick.yaml", "policy.json"))}
    # Stage the installed robot API and its lab imports as a unit. Tools may be
    # dynamically registered; optional policies and the web client are excluded.
    for folder in ("gemini_pupper/robot", "gemini_pupper/lab"):
        fixed.update(p.relative_to(root).as_posix() for p in (root / folder).glob("*.py"))
    return sorted(fixed)


def validate_selection(root, components, manifest):
    roles = {"launch": LAUNCH, "config": CONFIG, "urdf": URDF, "app_manifest": MANIFEST,
             "telemetry_topics": TELEMETRY, "helper": "foundation/hardware/persistent_session.py",
             "lifecycle": "gemini_pupper/robot/session_processes.py"}
    if any(Path(components[role]["path"]) != root / relative for role, relative in roles.items()):
        raise ValueError("combined session must select its frozen launch, telemetry, helpers and manifest")
    if (manifest.get("schema") != SCHEMA or manifest.get("environment") != ENV
            or manifest.get("destination_remote_path") != str(root)
            or [entry["path"] for entry in manifest["final_selected_files"]] != selected_paths(root)):
        raise ValueError("combined release manifest differs from selected profile")


def verify_guard(life, tag, root):
    """Check the selected guard belongs to this native launch before arming."""
    guard = str(root / GUARD)
    expected = ["-B", "-s", guard, *GUARD_ARGS]
    count = 0
    for proc in life.PROC.iterdir():
        if not proc.name.isdigit():
            continue
        try:
            arguments = (proc / "cmdline").read_bytes().split(b"\0")[:-1]
            if os.fsencode(guard) not in arguments:
                continue
            found = life.capture(int(proc.name), tag, guard)
        except (FileNotFoundError, ProcessLookupError):
            continue
        if found["argv"][1:] != expected:
            raise ValueError("selected guard has unexpected arguments")
        count += 1
    if count != 1:
        raise ValueError("require one running guard owned by this native session")


def verify_running_session(run_dir, release_root):
    """Read the prepared run and verify mapped native/IMU identities on the Pi."""
    import persistent_session
    run = Path(run_dir).resolve(strict=True)
    root = Path(release_root).resolve(strict=True)
    if root != ROOT:
        raise ValueError("invoke the selected release's own field runner")
    plan, life = persistent_session.load(run)
    if Path(plan["app_root"]) != root or plan["controllers"] != list(CONTROLLERS):
        raise ValueError("field runner does not own this release and controller set")
    if any(plan["environment"].get(k) != v for k, v in ENV.items()):
        raise ValueError("prepared run has a different FC environment")
    record = json.loads((root / MANIFEST).read_text())
    validate_selection(root, plan["components"], record)
    if (Path(plan["runtime_manifest"]["path"]) != run / "runtime-manifest.json"
            or Path(plan["generated_config"]["path"]) != run / "persistent_controllers.yaml"):
        raise ValueError("combined profile and configuration must belong to this run")
    receipt = persistent_session.receipt_for(life, run, plan)
    manager = receipt["processes"].get("manager")
    if (receipt["status"] != "running" or not manager or life.state(manager, plan["session_id"]) != "running"
            or "server" in receipt["processes"] or "server" in receipt["pending"]):
        raise ValueError("owned manager must be ready with no ordinary API server")
    persistent_session.check_loaded(plan, manager)
    verify_guard(life, plan["session_id"], root)
    return json.loads(Path(plan["runtime_manifest"]["path"]).read_text())


def check_profile(root):
    """Execute the actual release's file-only launch preparation in a fresh process."""
    with tempfile.TemporaryDirectory(prefix="fc-profile-") as directory:
        env = dict(os.environ, **ENV, PUPPER_GEMINI_DIR=str(root / "gemini_pupper"),
                   PUPPER_GAIT_DEPLOY_DIR=str(root / "pupper_gait_deploy"),
                   PUPPER_SESSION_MANIFEST=str(Path(directory) / "runtime-manifest.json"),
                   FOLLOW_USE_COLOR="0", FOLLOW_AVOID_OBSTACLES="0", FOLLOW_ALWAYS_DETECT="0")
        # An absent release module must not be supplied by the builder's checkout.
        env.pop("PYTHONPATH", None)
        output = subprocess.run([sys.executable, "-B", "-s", str(root / "foundation/hardware/fc_walk_kick_release.py"),
                                 "_profile"], env=env, cwd=root, capture_output=True, text=True)
        if output.returncode:
            raise ValueError("selected launch profile failed: " + output.stderr.strip())
        return json.loads(output.stdout)


def prepare_offline_profile():
    spec = importlib.util.spec_from_file_location("fc_lab6_launch", ROOT / LAUNCH)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    if file_digest(ROOT / URDF) != launch.URDF_SHA or file_digest(ROOT / CONFIG) != launch.CONFIG_SHA:
        raise ValueError("combined release changed maintained URDF or baseline controller configuration")
    _, controllers = launch.prepare_session_profile(ROOT / CONFIG, ROOT / URDF)
    import runtime_config
    from tricks import load_tricks
    selected = load_tricks()
    manifest = runtime_config.validate_session_catalog(selected)
    if not manifest or controllers != list(CONTROLLERS) or runtime_config.session_motion_mode() != MODE:
        raise ValueError("wrong field release generated")
    # Import and instantiate the real field planner from these staged files.
    sys.path.insert(0, str(ROOT))
    from final_project.pupper_fc.integration import Executor
    from final_project.pupper_fc.kick_handoff import KickHandoff
    executor = Executor(kick_side="right")
    step = executor.kick_step(KickHandoff("pupper-1"), 0)
    if step.decision.phase != "stopped" or step.request is not None:
        raise ValueError("combined executor failed disarmed startup")
    trick = selected["leg_kick"]
    if not trick.operator_only or any(trick.clip_length("kick_" + side) < 2.6 for side in ("left", "right")):
        raise ValueError("leg kick missing required clip or operator-only selection")
    # Exercise the deployed CLI's own import path from the frozen release.
    subprocess.run([sys.executable, "-B", "-s", str(ROOT / "final_project/behaviors/combined_fc_runner.py"),
                    "--help"], cwd=ROOT, stdout=subprocess.DEVNULL, check=True, timeout=15)
    return {"controllers": controllers, "motion_mode": MODE, "walker_command_contract": "fc_deadline_v1",
            "guard_arguments": list(GUARD_ARGS), "clip_names": ["kick_left", "kick_right"],
            "bounds": runtime_config.session_motion_bounds(manifest), "motor_started": False}


def git_revision(root, paths):
    status = subprocess.run(["git", "-C", str(root), "status", "--porcelain", "--untracked-files=all", "--", *paths],
                            capture_output=True, text=True)
    if status.returncode or status.stdout:
        return None
    return subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()


def verify(root, *, expected_manifest=None):
    root = root.resolve(strict=True)
    path = root / MANIFEST
    if expected_manifest is not None and file_digest(path) != expected_manifest:
        raise ValueError("combined release manifest digest mismatch")
    record = json.loads(path.read_text())
    entries = record["final_selected_files"]
    if (record.get("schema") != SCHEMA or record.get("environment") != ENV
            or record.get("destination_remote_path") != str(root)
            or [entry["path"] for entry in entries] != selected_paths(root)):
        raise ValueError("inconsistent FC release manifest or missing required files")
    verify_selected(root, entries)
    return {"release": str(root), "source_commit": record["source_commit"], "files": len(entries),
            "manifest_sha256": file_digest(path), "profile": check_profile(root),
            "native_binaries": "source-only; requires selected target-native controller, driver and gateway"}


def stage(source, destination, expected_commit=None):
    source = source.resolve(strict=True)
    destination = destination.expanduser().absolute()
    if destination.exists() or destination.is_symlink():
        raise ValueError("destination exists; verify the existing release or choose an unused path")
    paths = selected_paths(source)
    revision = git_revision(source, paths)
    if expected_commit is not None and revision != expected_commit:
        raise ValueError("selected source differs from the expected clean commit")
    destination.parent.mkdir(exist_ok=True, parents=True)
    with tempfile.TemporaryDirectory(prefix=".fc-stage-", dir=destination.parent) as temp:
        tree = Path(temp) / "release"
        tree.mkdir()
        entries = []
        for relative in paths:
            src = checked_path(source, relative)
            dest = tree / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src, dest)
            entries.append({"path": relative, "sha256": file_digest(dest)})
        verify_selected(source, entries)
        if expected_commit is not None and git_revision(source, paths) != expected_commit:
            raise ValueError("selected source changed during release assembly")
        profile = check_profile(tree)
        record = {"schema": SCHEMA, "source_commit": revision, "environment": ENV,
                  "baseline_remote_path": str(source), "destination_remote_path": str(destination.resolve()),
                  "final_selected_files": entries}
        (tree / MANIFEST).write_text(json.dumps(record, indent=2) + "\n")
        # The prepared profile uses absolute paths. It was generated in a disposable
        # directory and must be regenerated for its final location before activation.
        tree.rename(destination)
    return {"release": str(destination.resolve()), "source_commit": revision, "files": len(entries),
            "manifest_sha256": file_digest(destination / MANIFEST), "profile": profile,
            "native_binaries": "source-only; requires selected target-native controller, driver and gateway"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="action", required=True)
    builder = commands.add_parser("stage")
    builder.add_argument("--source", type=Path, default=ROOT)
    builder.add_argument("--destination", type=Path, required=True)
    builder.add_argument("--expected-commit")
    checker = commands.add_parser("verify")
    checker.add_argument("--release", type=Path, required=True)
    checker.add_argument("--manifest-sha256")
    native = commands.add_parser("inspect-native", help="screen a target controller binary without loading it")
    native.add_argument("--library", type=Path, required=True)
    commands.add_parser("_profile", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.action == "stage":
        result = stage(args.source, args.destination, args.expected_commit)
    elif args.action == "verify":
        result = verify(args.release, expected_manifest=args.manifest_sha256)
    elif args.action == "inspect-native":
        result = inspect_native(args.library)
    else:
        result = prepare_offline_profile()
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
