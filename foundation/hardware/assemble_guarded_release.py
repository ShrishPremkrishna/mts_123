#!/usr/bin/env python3
"""Assemble a new guarded walk release from pinned sources and a native baseline.

Run on the target filesystem after selecting the baseline and ROS Python. This
copies files only: no ROS init, deployment, builds, homing or process startup.
The native binaries, policy and gains stay unchanged. Target preflight is still
required; this does not add the newer native software-Start service.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil


SOURCE_ROLES = {
    "launch": "foundation/hardware/stock_session.launch.py",
    "runtime_checker": "foundation/hardware/runtime_identity.py",
    "trial_tool": "final_project/behaviors/tools/movement_trial.py",
    "software_start": "foundation/hardware/software_start.py",
    "guard_script": "final_project/behaviors/body_command_guard.py",
    "guard_obstacles": "final_project/pupper_fc/obstacles.py",
    "guard_package_init": "final_project/pupper_fc/__init__.py",
    "command_path_evidence": "final_project/behaviors/command_path_evidence.py",
    "bridge_script": "final_project/behaviors/fc_executor_bridge.py",
    "fc_integration": "final_project/pupper_fc/integration.py",
    "fc_controller": "final_project/pupper_fc/controller.py",
    "fc_model": "final_project/pupper_fc/model.py",
    "fc_tactics": "final_project/pupper_fc/tactics.py",
}
NATIVE_ROLES = {"driver", "controller", "policy", "gateway", "mux"}


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def pinned_json(path, expected):
    if sha(path) != expected:
        raise ValueError("Manifest digest mismatch: " + str(path))
    return json.loads(Path(path).read_text())


def within(root, relative):
    path = root / relative
    if Path(relative).is_absolute() or path.resolve().is_relative_to(root) is False:
        raise ValueError("File escapes selected root: " + str(relative))
    return path


def assemble(baseline_manifest, baseline_sha256, source_root, source_index,
             source_sha256, destination, guard_python, arena_ball_contact=False):
    if not isinstance(arena_ball_contact, bool):
        raise ValueError("Arena ball contact selection must be boolean")
    baseline_manifest = Path(baseline_manifest).resolve(strict=True)
    base = baseline_manifest.parent
    sources = Path(source_root).resolve(strict=True)
    destination = Path(destination).absolute()
    if destination.exists() or destination.is_symlink():
        raise ValueError("Destination exists; choose a new release path")
    if destination.resolve().is_relative_to(base) or base.is_relative_to(destination.resolve()):
        raise ValueError("Destination must be separate from the baseline")
    guard_python = Path(guard_python).absolute()
    if not guard_python.is_file():
        raise ValueError("Select an existing target ROS Python executable")
    baseline = pinned_json(baseline_manifest, baseline_sha256)
    if baseline.get("schema") != "pupper_native_release_v1" or baseline.get("trial_profile") != "walk":
        raise ValueError("Only a selected walk baseline is supported")
    files = baseline["files"]
    if not NATIVE_ROLES | {"config", "launch", "urdf", "runtime_checker", "trial_tool", "shot_node", "shot_core"} <= files.keys():
        raise ValueError("Baseline omits required stock files")
    if any(role.startswith("guard_") for role in files):
        raise ValueError("Select the native stock baseline, not a prior guarded candidate")
    relative_files = {}
    for role, item in files.items():
        path = Path(item["path"])
        relative = path.relative_to(base)
        if within(base, relative).resolve() != path.resolve() or sha(path) != item["sha256"]:
            raise ValueError("Baseline file mismatch: " + role)
        relative_files[role] = relative
    if relative_files["config"] != Path("walk.yaml"):
        raise ValueError("Expected expanded walk.yaml at release root")
    # Refuse hidden cross-release dependencies when cloning the small selected
    # tree. Native prefix metadata is retained along with its unchanged binaries.
    if any(path.is_symlink() for path in base.rglob("*")):
        raise ValueError("Baseline contains symlinks; resolve its release closure before assembly")
    index = pinned_json(source_index, source_sha256)
    if index.get("schema") != "pupper_guard_sources_v1" or set(index["files"]) != set(SOURCE_ROLES.values()):
        raise ValueError("Source index must contain exactly the selected guarded/bridge source files")
    for relative, digest in index["files"].items():
        if sha(within(sources, relative)) != digest:
            raise ValueError("Source file mismatch: " + relative)
    config_bytes = (base / "walk.yaml").read_bytes()
    if hashlib.sha256(config_bytes).hexdigest() != files["config"]["sha256"]:
        raise ValueError("Baseline config changed while selecting inputs")
    config = config_bytes.decode()
    environment = (base / "environment.sh").read_text()
    if "$(" in config or str(base) not in config or str(base) not in environment:
        raise ValueError("Baseline config/environment must select its expanded release paths")

    shutil.copytree(base, destination, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    # Old receipts describe a different release. Do not carry those claims into
    # the candidate; the immutable baseline remains available for rollback.
    for name in ("STAGING.json", "FILE_SHA256.json", "sprint-manifest.json", "sprint.yaml"):
        (destination / name).unlink(missing_ok=True)
    for role, relative in SOURCE_ROLES.items():
        path = destination / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(sources / relative, path)
        relative_files[role] = Path(relative)
    mode = "arena_ball_contact" if arena_ball_contact else "all_obstacles"
    policy_path = Path("foundation/hardware/guard-policy.json")
    (destination / policy_path).write_text(json.dumps({"schema": "pupper_guard_policy_v1", "mode": mode}, indent=2) + "\n")
    relative_files["guard_policy"] = policy_path
    (destination / "walk.yaml").write_text(config.replace(str(base), str(destination)))
    (destination / "environment.sh").write_text(environment.replace(str(base), str(destination)))
    selected = {role: {"path": str(destination / relative), "sha256": sha(destination / relative)}
                for role, relative in relative_files.items()}
    selected["guard_python"] = {"path": str(guard_python), "sha256": sha(guard_python)}
    for role in files.keys() - SOURCE_ROLES.keys() - {"config"}:
        if selected[role]["sha256"] != files[role]["sha256"]:
            raise ValueError("Preserved baseline changed while assembling: " + role)
    # Detect a concurrent source edit or failed copy before calling it assembled.
    for role, relative in SOURCE_ROLES.items():
        if selected[role]["sha256"] != index["files"][relative]:
            raise ValueError("Selected source changed while assembling: " + relative)
    manifest = {**baseline, "files": selected, "physical_acceptance": False,
                "build_target": "Unchanged selected Pi native binaries; guarded Python source candidate. Target preflight required.",
                "source_freeze_sha256": {**baseline.get("source_freeze_sha256", {}), **index["files"]}}
    (destination / "walk-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    record = {"schema": "pupper_guarded_assembly_v1", "created_utc": datetime.now(timezone.utc).isoformat(),
              "base_release": str(base), "base_manifest_sha256": baseline_sha256,
              "source_index_sha256": source_sha256, "release": str(destination),
              "manifest_sha256": sha(destination / "walk-manifest.json"),
              "native_unchanged": True, "hardware_started": False, "physical_acceptance": False,
              "target_preflight_required": True, "software_start_service_added": False}
    record["guard_mode"] = mode
    (destination / "STAGING.json").write_text(json.dumps(record, indent=2) + "\n")
    payload = {str(path.relative_to(destination)): sha(path)
               for path in sorted(destination.rglob("*")) if path.is_file() and path.name != "FILE_SHA256.json"}
    (destination / "FILE_SHA256.json").write_text(json.dumps(payload, indent=2) + "\n")
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("baseline-manifest", "baseline-sha256", "source-root", "source-index", "source-sha256", "destination", "guard-python"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--arena-ball-contact", action="store_true",
                        help="Explicit candidate: allow verified lab-model sports-ball detections; all other obstacles still gate motion")
    args = parser.parse_args()
    try:
        print(json.dumps(assemble(**vars(args)), indent=2))
    except (OSError, KeyError, TypeError, ValueError) as error:
        parser.exit(2, "Guarded assembly refused (partial destination, if any, retained): " + str(error) + "\n")


if __name__ == "__main__":
    main()
