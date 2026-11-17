#!/usr/bin/env python3
"""Read-only native release gate. No ROS import, deployment, homing or commands.

Create a preflight receipt while the controller is stopped; verify that receipt
against the next running manager before a motion trial. This binds the selected
files to the boot, startup order, actually mapped driver/controller libraries,
and selected gateway/mux executable processes.
It does not prove physical behavior or infer loaded weights from ROS parameters.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

import yaml


ROLES = {"driver", "controller", "config", "policy", "launch"}
LIBRARIES = {"driver": "libcontrol_board_hardware_interface.so",
             "controller": "libneural_controller.so"}
EXECUTABLES = {"gateway": "estop_controller", "mux": "cmd_vel_mux_node"}
SCHEMA = "pupper_native_release_v1"


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def read_json(path):
    path = Path(path)
    if path.stat().st_size > 131072:
        raise ValueError("Oversize identity record")
    return json.loads(path.read_text())


def boot_id(proc):
    return (proc / "sys/kernel/random/boot_id").read_text().strip()


def process_stat(proc, pid):
    # comm may contain spaces/parentheses; fields after its final ')' begin at 3.
    fields = (proc / str(pid) / "stat").read_text().rsplit(")", 1)[-1].split()
    if len(fields) < 20:
        raise ValueError("Incomplete process identity: " + str(pid))
    return fields


def start_ticks(proc, pid):
    return int(process_stat(proc, pid)[19])


def managers(proc):
    found = []
    for directory in proc.iterdir():
        if not directory.name.isdigit():
            continue
        try:
            argv = (directory / "cmdline").read_bytes().split(b"\0")
            if argv and Path(os.fsdecode(argv[0])).name == "ros2_control_node":
                found.append(int(directory.name))
        except (FileNotFoundError, ProcessLookupError):
            continue
    return found


def executable_pids(proc, role, selected):
    """Find role candidates by argv or actual executable, including old releases."""
    names = {EXECUTABLES[role], Path(selected["path"]).name}
    found = []
    for directory in proc.iterdir():
        if not directory.name.isdigit():
            continue
        try:
            argv = (directory / "cmdline").read_bytes().split(b"\0")
        except (FileNotFoundError, ProcessLookupError):
            continue
        if argv and Path(os.fsdecode(argv[0])).name in names:
            found.append(int(directory.name))
            continue
        try:
            # argv[0] can be changed. The kernel's executable link also catches
            # a selected binary started with an unrelated process name.
            target = os.readlink(directory / "exe")
            if Path(target.removesuffix(" (deleted)")).name in names:
                found.append(int(directory.name))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            # Unrelated processes owned by another user commonly deny exe
            # access. A role identified by cmdline is checked strictly below.
            continue
    return sorted(found)


def executable_identity(proc, role, selected):
    pids = executable_pids(proc, role, selected)
    if len(pids) != 1:
        raise ValueError("Expected exactly one running " + role + " executable")
    pid = pids[0]
    try:
        started = start_ticks(proc, pid)
        executable = proc / str(pid) / "exe"
        path = os.readlink(executable)
        if path != selected["path"]:
            raise ValueError("Running executable differs from selected file: " + role)
        # Stat the kernel's reference to the running image, not its on-disk
        # pathname: replacement/deletion can leave an old inode executing.
        st = executable.stat()
        if st.st_dev != selected["device"] or st.st_ino != selected["inode"]:
            raise ValueError("Running executable differs from selected file: " + role)
        if process_stat(proc, pid)[0] in {"T", "t", "Z", "X", "x"}:
            raise ValueError("Selected executable process is stopped or dead: " + role)
        if start_ticks(proc, pid) != started:
            raise ValueError("Executable identity changed during verification: " + role)
    except OSError as error:
        raise ValueError("Cannot establish running executable identity: " + role) from error
    return {"pid": pid, "start_ticks": started, "path": path,
            "device": st.st_dev, "inode": st.st_ino}


def verify_files(manifest):
    if manifest.get("schema") != SCHEMA or not ROLES <= manifest.get("files", {}).keys():
        raise ValueError("Release needs driver, controller, config, policy and launch identities")
    result = {}
    for role, item in manifest["files"].items():
        path = Path(item["path"])
        if not path.is_absolute() or not re.fullmatch(r"[0-9a-f]{64}", item["sha256"]):
            raise ValueError("Invalid identity: " + role)
        if role in LIBRARIES and path.name != LIBRARIES[role]:
            raise ValueError("Unexpected library filename: " + role)
        if not path.is_file() or sha(path) != item["sha256"]:
            raise ValueError("Selected file mismatch: " + role + " " + str(path))
        resolved = path.resolve(strict=True)
        st = resolved.stat()
        result[role] = {"path": str(resolved), "sha256": item["sha256"],
                        "device": st.st_dev, "inode": st.st_ino,
                        "size": st.st_size, "mtime_ns": st.st_mtime_ns}
    config = yaml.safe_load(Path(result["config"]["path"]).read_text())
    model = config["neural_controller_default"]["ros__parameters"]["model_path"]
    if not Path(model).is_absolute() or str(Path(model).resolve(strict=True)) != result["policy"]["path"]:
        raise ValueError("Expanded stock config does not select the pinned policy")
    return result


def check_dependencies(pins):
    for role in ("driver", "controller", "gateway", "mux"):
        if role not in pins:
            continue
        result = subprocess.run(["ldd", pins[role]["path"]], capture_output=True,
                                text=True, timeout=10, check=False)
        output = result.stdout + result.stderr
        if result.returncode or "not found" in output:
            raise ValueError(f"{role} is incompatible with this machine: {output.strip()}")


def preflight(manifest_path, expected_sha, *, proc=Path("/proc"), dependency_checker=check_dependencies):
    if not re.fullmatch(r"[0-9a-f]{64}", expected_sha) or sha(manifest_path) != expected_sha:
        raise ValueError("Release manifest identity mismatch")
    manifest = read_json(manifest_path)
    pins = verify_files(manifest)
    if managers(proc):
        raise ValueError("A controller manager is running; do not prepare over another session")
    for role in EXECUTABLES.keys() & pins.keys():
        if executable_pids(proc, role, pins[role]):
            raise ValueError("A " + role + " executable is running; do not prepare over another session")
    dependency_checker(pins)
    # Linux uptime and process start ticks share time since boot, including suspend.
    ticks = float((proc / "uptime").read_text().split()[0]) * os.sysconf("SC_CLK_TCK")
    return {"schema": "pupper_native_preflight_v1", "boot_id": boot_id(proc),
            "preflight_ticks": ticks, "created_unix": time.time(),
            "manifest_path": str(Path(manifest_path).resolve()), "manifest_sha256": expected_sha,
            "files": pins, "motion_started": False}


def verify_prepared(receipt, *, proc=Path("/proc")):
    if receipt.get("schema") != "pupper_native_preflight_v1" or receipt["boot_id"] != boot_id(proc):
        raise ValueError("Preflight is missing or belongs to a different boot")
    if sha(receipt["manifest_path"]) != receipt["manifest_sha256"]:
        raise ValueError("Selected manifest changed after preflight")
    current = verify_files(read_json(receipt["manifest_path"]))
    if current != receipt["files"]:
        raise ValueError("Selected files changed after preflight; prepare again with motors stopped")
    return current


def verify_loaded(receipt, *, proc=Path("/proc")):
    current = verify_prepared(receipt, proc=proc)
    pids = managers(proc)
    if len(pids) != 1:
        raise ValueError("Expected exactly one running controller manager")
    pid = pids[0]
    started = start_ticks(proc, pid)
    if started <= receipt["preflight_ticks"]:
        raise ValueError("Manager predates preflight; initialized configuration is not established")
    argv = [os.fsdecode(x) for x in (proc / str(pid) / "cmdline").read_bytes().split(b"\0") if x]
    parameter_paths = [Path(argv[i + 1]) for i, arg in enumerate(argv[:-1]) if arg == "--params-file"]
    # The selected config must be fully expanded before preflight. ROS may copy
    # it into /tmp; compare its bytes rather than guessing the generated name.
    if (len(parameter_paths) != 1 or any(arg in ("-p", "--param") for arg in argv) or
            not parameter_paths[0].is_file() or
            sha(parameter_paths[0]) != current["config"]["sha256"]):
        raise ValueError("Manager did not receive the selected expanded configuration")
    maps = []
    for line in (proc / str(pid) / "maps").read_text().splitlines():
        parts = line.split(None, 5)
        if len(parts) == 6:
            maps.append(parts)
    for role, basename in LIBRARIES.items():
        selected = current[role]
        matching = [m for m in maps if Path(m[5].removesuffix(" (deleted)")).name == basename]
        if not matching:
            raise ValueError("Selected library not loaded: " + role)
        for m in matching:
            major, minor = (int(x, 16) for x in m[3].split(":"))
            if (m[5] != selected["path"] or int(m[4]) != selected["inode"] or
                    os.makedev(major, minor) != selected["device"]):
                raise ValueError("Loaded library differs from selected file: " + role)
    executables = {}
    for role in EXECUTABLES.keys() & current.keys():
        executables[role] = executable_identity(proc, role, current[role])
        if executables[role]["start_ticks"] <= receipt["preflight_ticks"]:
            raise ValueError(role + " executable predates preflight")
    evidence = {"verified": True, "manager_pid": pid, "manager_start_ticks": started,
                "boot_id": receipt["boot_id"], "files": current, "executables": executables,
                "scope": ("Selected files, supplied config, actual mapped libraries and selected "
                          "executable processes; not physical acceptance")}
    if not same_manager(evidence, proc=proc):
        raise ValueError("Controller identity changed during verification")
    return evidence


def same_manager(evidence, *, proc=Path("/proc")):
    """In-trial manager/executable check; full file/mapping check precedes the pulse."""
    try:
        if (boot_id(proc) != evidence["boot_id"] or
                managers(proc) != [evidence["manager_pid"]] or
                start_ticks(proc, evidence["manager_pid"]) != evidence["manager_start_ticks"] or
                process_stat(proc, evidence["manager_pid"])[0] in {"T", "t", "Z", "X", "x"}):
            return False
        for role in EXECUTABLES.keys() & evidence.get("files", {}).keys():
            if executable_identity(proc, role, evidence["files"][role]) != evidence["executables"][role]:
                return False
        return True
    except (OSError, KeyError, ValueError):
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    before = sub.add_parser("preflight", help="Verify selected files with no manager running")
    before.add_argument("--manifest", type=Path, required=True)
    before.add_argument("--sha256", required=True)
    before.add_argument("--output", type=Path, required=True)
    after = sub.add_parser("loaded", help="Check the next manager's actually mapped libraries")
    after.add_argument("--receipt", type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.action == "preflight":
            if args.output.exists():
                raise ValueError("Receipt exists; choose a fresh run path")
            result = preflight(args.manifest, args.sha256)
            with args.output.open("x") as stream:
                json.dump(result, stream, indent=2, allow_nan=False)
                stream.write("\n")
        else:
            result = verify_loaded(read_json(args.receipt))
        print(json.dumps(result, indent=2, allow_nan=False))
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(2, "Runtime identity refused: " + str(error) + "\n")


if __name__ == "__main__":
    main()
