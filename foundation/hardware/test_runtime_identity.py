"""Synthetic /proc and files exercise the real read-only release gate."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location("runtime_identity", Path(__file__).with_name("runtime_identity.py"))
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)


def write_stat(directory, ticks=20000, state="S"):
    fields = [state, *(["0"] * 18), str(ticks), "0"]
    (directory / "stat").write_text(directory.name + " (process (test) name) " + " ".join(fields))


@pytest.fixture
def setup(tmp_path):
    proc = tmp_path / "proc"
    (proc / "sys/kernel/random").mkdir(parents=True)
    (proc / "sys/kernel/random/boot_id").write_text("test-boot")
    (proc / "uptime").write_text("100.0 100.0")
    files = {}
    for role in runtime.ROLES:
        p = tmp_path / runtime.LIBRARIES.get(role, role + ".txt")
        p.write_text(role)
        files[role] = {"path": str(p), "sha256": runtime.sha(p)}
    Path(files["config"]["path"]).write_text("neural_controller_default:\n  ros__parameters:\n    model_path: " + files["policy"]["path"] + "\n")
    files["config"]["sha256"] = runtime.sha(files["config"]["path"])
    manifest = tmp_path / "selected.json"
    manifest.write_text(json.dumps({"schema": runtime.SCHEMA, "files": files}))
    receipt = runtime.preflight(manifest, runtime.sha(manifest), proc=proc, dependency_checker=lambda _: None)

    def manager(pid=123, ticks=20000):
        d = proc / str(pid)
        d.mkdir()
        (d / "cmdline").write_bytes(b"/opt/ros/bin/ros2_control_node\0--ros-args\0--params-file\0" + files['config']['path'].encode() + b"\0")
        write_stat(d, ticks)
        lines = []
        for role in runtime.LIBRARIES:
            p = Path(files[role]["path"])
            st = p.stat()
            lines.append(f"123-456 r-xp 0 {os.major(st.st_dev):02x}:{os.minor(st.st_dev):02x} {st.st_ino} {p}")
        (d / "maps").write_text("\n".join(lines))
        return d
    return proc, manifest, files, receipt, manager


@pytest.fixture
def executable_setup(setup):
    proc, manifest, files, _, manager = setup
    for role, basename in runtime.EXECUTABLES.items():
        path = manifest.parent / basename
        path.write_bytes((role + "-selected-binary").encode())
        files[role] = {"path": str(path), "sha256": runtime.sha(path)}
    manifest.write_text(json.dumps({"schema": runtime.SCHEMA, "files": files}))
    receipt = runtime.preflight(manifest, runtime.sha(manifest), proc=proc, dependency_checker=lambda _: None)

    def executable(role, pid=None, ticks=20000, actual=None, argv=None, state="S"):
        pid = pid if pid is not None else {"gateway": 201, "mux": 202}[role]
        directory = proc / str(pid)
        directory.mkdir()
        (directory / "cmdline").write_bytes(os.fsencode(argv or files[role]["path"]) + b"\0")
        (directory / "exe").symlink_to(actual or files[role]["path"])
        write_stat(directory, ticks, state)
        return directory

    return proc, manifest, files, receipt, manager, executable


def test_verified_files_and_new_manager_are_bound_to_actual_maps(setup):
    proc, _, _, receipt, manager = setup
    manager()
    result = runtime.verify_loaded(receipt, proc=proc)
    assert result["verified"] and result["manager_pid"] == 123
    assert result["manager_start_ticks"] == 20000
    assert result["executables"] == {}
    assert runtime.same_manager(result, proc=proc)
    assert result["scope"].endswith("not physical acceptance")


def test_preflight_refuses_existing_manager(setup):
    proc, manifest, _, _, manager = setup
    manager()
    with pytest.raises(ValueError, match="manager is running"):
        runtime.preflight(manifest, runtime.sha(manifest), proc=proc, dependency_checker=lambda _: None)


@pytest.mark.parametrize("role", sorted(runtime.ROLES))
def test_any_selected_file_change_invalidates_receipt(setup, role):
    proc, _, files, receipt, manager = setup
    manager()
    Path(files[role]["path"]).write_text("changed")
    with pytest.raises(ValueError, match="mismatch"):
        runtime.verify_loaded(receipt, proc=proc)


@pytest.mark.parametrize("fault", ["wrong_path", "deleted", "different_inode", "missing"])
def test_different_loaded_driver_is_refused_even_if_selected_file_is_good(setup, fault):
    proc, _, files, receipt, manager = setup
    d = manager()
    lines = (d / "maps").read_text().splitlines()
    line = lines[0]
    if fault == "wrong_path":
        line = line.replace(files["driver"]["path"], "/old/libcontrol_board_hardware_interface.so")
    elif fault == "deleted":
        line += " (deleted)"
    elif fault == "different_inode":
        parts = line.split(None, 5)
        parts[4] = "9999999"
        line = " ".join(parts)
    else:
        line = ""
    (d / "maps").write_text("\n".join([line, *lines[1:]]))
    with pytest.raises(ValueError, match="library"):
        runtime.verify_loaded(receipt, proc=proc)


@pytest.mark.parametrize("fault", ["boot", "old_manager", "two_managers", "no_manager", "config_not_supplied", "config_override"])
def test_wrong_boot_manager_or_config_refused(setup, fault):
    proc, _, _, receipt, manager = setup
    if fault != "no_manager":
        d = manager(ticks=1 if fault == "old_manager" else 20000)
    if fault == "boot":
        (proc / "sys/kernel/random/boot_id").write_text("rebooted")
    if fault == "two_managers":
        manager(pid=456)
    if fault == "config_not_supplied":
        (d / "cmdline").write_bytes(b"/opt/ros/bin/ros2_control_node\0")
    if fault == "config_override":
        with (d / "cmdline").open("ab") as stream:
            stream.write(b"-p\0max_vx_forward:=10.0\0")
    with pytest.raises(ValueError):
        runtime.verify_loaded(receipt, proc=proc)


def test_manifest_rewrite_cannot_authorize_new_file_after_preflight(setup):
    proc, manifest, files, receipt, manager = setup
    manager()
    Path(files["driver"]["path"]).write_text("unreviewed")
    files["driver"]["sha256"] = runtime.sha(files["driver"]["path"])
    manifest.write_text(json.dumps({"schema": runtime.SCHEMA, "files": files}))
    with pytest.raises(ValueError, match="manifest changed"):
        runtime.verify_loaded(receipt, proc=proc)


def test_dependency_gate_refuses_newer_glibc_even_when_file_hashes_match(monkeypatch):
  from types import SimpleNamespace
  monkeypatch.setattr(runtime.subprocess, "run", lambda *a, **k: SimpleNamespace(returncode=0,stdout="GLIBC_2.38 not found",stderr=""))
  with pytest.raises(ValueError,match="incompatible"):
    runtime.check_dependencies({"controller":{"path":"/selected/libneural_controller.so"}})


def test_loaded_config_cannot_select_a_different_policy(setup):
  proc, manifest, files, receipt, manager = setup
  other=manifest.parent / "other-policy.json"
  other.write_text("other")
  Path(files["config"]["path"]).write_text("neural_controller_default:\n  ros__parameters:\n    model_path: " + str(other) + "\n")
  files["config"]["sha256"]=runtime.sha(files["config"]["path"])
  with pytest.raises(ValueError,match="pinned policy"):
    runtime.verify_files({"schema":runtime.SCHEMA,"files":files})


def test_manager_replacement_during_verification_refuses(setup,monkeypatch):
  proc, manifest, files, receipt, manager = setup
  manager()
  old=runtime.start_ticks
  calls=[]
  def changing(proc,pid):
    calls.append(pid)
    return old(proc,pid) if len(calls)==1 else 20001
  monkeypatch.setattr(runtime,"start_ticks",changing)
  with pytest.raises(ValueError,match="changed during"):
    runtime.verify_loaded(receipt,proc=proc)


def test_actual_gateway_and_mux_images_are_bound_even_with_changed_argv(executable_setup):
    proc, _, files, receipt, manager, executable = executable_setup
    manager()
    for role in runtime.EXECUTABLES:
        executable(role, argv="/unrelated/process-name")
    evidence = runtime.verify_loaded(receipt, proc=proc)
    for role, pid in (("gateway", 201), ("mux", 202)):
        st = Path(files[role]["path"]).stat()
        assert evidence["executables"][role] == {
            "pid": pid, "start_ticks": 20000, "path": files[role]["path"],
            "device": st.st_dev, "inode": st.st_ino,
        }
    assert runtime.same_manager(evidence, proc=proc)


@pytest.mark.parametrize("role", runtime.EXECUTABLES)
def test_one_optional_executable_role_is_sufficient(executable_setup, role):
    proc, manifest, files, _, manager, executable = executable_setup
    files = {name: item for name, item in files.items()
             if name not in runtime.EXECUTABLES or name == role}
    manifest.write_text(json.dumps({"schema": runtime.SCHEMA, "files": files}))
    receipt = runtime.preflight(manifest, runtime.sha(manifest), proc=proc, dependency_checker=lambda _: None)
    manager()
    executable(role)
    evidence = runtime.verify_loaded(receipt, proc=proc)
    assert set(evidence["executables"]) == {role}
    assert runtime.same_manager(evidence, proc=proc)


@pytest.mark.parametrize("role", runtime.EXECUTABLES)
@pytest.mark.parametrize("renamed_argv", [False, True])
def test_preflight_refuses_preexisting_gateway_or_mux(executable_setup, role, renamed_argv):
    proc, manifest, _, _, _, executable = executable_setup
    executable(role, argv="/unrelated/name" if renamed_argv else None)
    with pytest.raises(ValueError, match=role + " executable is running"):
        runtime.preflight(manifest, runtime.sha(manifest), proc=proc, dependency_checker=lambda _: None)


@pytest.mark.parametrize("role", runtime.EXECUTABLES)
@pytest.mark.parametrize("fault", ["missing", "wrong_path", "deleted", "duplicate", "old",
                                  "at_preflight", "missing_exe", "stopped", "zombie"])
def test_loaded_executable_faults_refuse_valid_selected_files(executable_setup, role, fault):
    proc, manifest, files, receipt, manager, executable = executable_setup
    manager()
    for selected_role in runtime.EXECUTABLES:
        if selected_role != role:
            executable(selected_role)
    if fault != "missing":
        ticks = 1 if fault == "old" else receipt["preflight_ticks"] if fault == "at_preflight" else 20000
        directory = executable(role, ticks=int(ticks), state={"stopped": "T", "zombie": "Z"}.get(fault, "S"))
        if fault in {"wrong_path", "deleted", "missing_exe"}:
            (directory / "exe").unlink()
            if fault == "wrong_path":
                wrong = manifest.parent / "old" / runtime.EXECUTABLES[role]
                wrong.parent.mkdir()
                wrong.write_text("older binary")
                (directory / "exe").symlink_to(wrong)
            elif fault == "deleted":
                (directory / "exe").symlink_to(files[role]["path"] + " (deleted)")
        if fault == "duplicate":
            executable(role, pid=303, argv="/unrelated/name")
    with pytest.raises(ValueError, match=role):
        runtime.verify_loaded(receipt, proc=proc)


@pytest.mark.parametrize("role", runtime.EXECUTABLES)
@pytest.mark.parametrize("field", ["st_dev", "st_ino"])
def test_running_image_inode_and_device_must_match_selected_file(executable_setup, monkeypatch, role, field):
    proc, _, _, receipt, manager, executable = executable_setup
    manager()
    for selected_role in runtime.EXECUTABLES:
        directory = executable(selected_role)
        if selected_role == role:
            target = directory / "exe"
    original_stat = Path.stat

    def changed_stat(path, *args, **kwargs):
        st = original_stat(path, *args, **kwargs)
        if path == target:
            fields = {"st_dev": st.st_dev, "st_ino": st.st_ino}
            fields[field] += 1
            return SimpleNamespace(**fields)
        return st

    # /proc/<pid>/exe is a kernel reference, unlike the ordinary symlink in
    # this fixture; model its old inode while the selected on-disk file is good.
    monkeypatch.setattr(Path, "stat", changed_stat)
    with pytest.raises(ValueError, match="differs.*" + role):
        runtime.verify_loaded(receipt, proc=proc)


@pytest.mark.parametrize("role", runtime.EXECUTABLES)
@pytest.mark.parametrize("fault", ["exit", "new_pid", "pid_reused", "exe_changed", "exe_deleted",
                                  "duplicate", "stopped", "zombie"])
def test_trial_identity_fails_when_gateway_or_mux_changes(executable_setup, role, fault):
    proc, manifest, files, receipt, manager, executable = executable_setup
    manager()
    directories = {name: executable(name) for name in runtime.EXECUTABLES}
    evidence = runtime.verify_loaded(receipt, proc=proc)
    directory = directories[role]
    if fault in {"exit", "new_pid"}:
        shutil.rmtree(directory)
        if fault == "new_pid":
            executable(role, pid=404, ticks=20001)
    elif fault == "pid_reused":
        write_stat(directory, ticks=20001)
    elif fault in {"exe_changed", "exe_deleted"}:
        (directory / "exe").unlink()
        if fault == "exe_changed":
            replacement = manifest.parent / "replacement-binary"
            replacement.write_text("different executable")
            (directory / "exe").symlink_to(replacement)
        else:
            (directory / "exe").symlink_to(files[role]["path"] + " (deleted)")
    elif fault == "duplicate":
        executable(role, pid=404)
    else:
        write_stat(directory, state={"stopped": "T", "zombie": "Z"}[fault])
    assert runtime.managers(proc) == [123]
    assert not runtime.same_manager(evidence, proc=proc)


@pytest.mark.parametrize("role", runtime.EXECUTABLES)
def test_executable_replacement_during_loaded_verification_refuses(executable_setup, monkeypatch, role):
    proc, _, _, receipt, manager, executable = executable_setup
    manager()
    for selected_role in runtime.EXECUTABLES:
        executable(selected_role)
    original_identity = runtime.executable_identity
    changed = False

    def replaced_identity(proc, selected_role, selected):
        nonlocal changed
        identity = original_identity(proc, selected_role, selected)
        if selected_role == role and not changed:
            changed = True
            write_stat(proc / str(identity["pid"]), ticks=20001)
        return identity

    monkeypatch.setattr(runtime, "executable_identity", replaced_identity)
    with pytest.raises(ValueError, match="changed during"):
        runtime.verify_loaded(receipt, proc=proc)


def test_trial_refuses_old_evidence_missing_selected_executable_identities(executable_setup):
    proc, _, _, receipt, manager, executable = executable_setup
    manager()
    for role in runtime.EXECUTABLES:
        executable(role)
    evidence = runtime.verify_loaded(receipt, proc=proc)
    del evidence["executables"]
    assert not runtime.same_manager(evidence, proc=proc)


@pytest.mark.parametrize("role", runtime.EXECUTABLES)
def test_identified_role_with_unreadable_executable_is_refused(executable_setup, monkeypatch, role):
    proc, _, _, receipt, manager, executable = executable_setup
    manager()
    directories = {name: executable(name) for name in runtime.EXECUTABLES}
    original_readlink = os.readlink

    def unreadable(path, *args, **kwargs):
        if path == directories[role] / "exe":
            raise PermissionError("unreadable executable")
        return original_readlink(path, *args, **kwargs)

    monkeypatch.setattr(os, "readlink", unreadable)
    with pytest.raises(ValueError, match="Cannot establish.*" + role):
        runtime.verify_loaded(receipt, proc=proc)


@pytest.mark.parametrize("state", ["T", "Z"])
def test_stopped_or_zombie_manager_refuses_loaded_verification_and_trial(setup, state):
    proc, _, _, receipt, manager = setup
    directory = manager()
    evidence = runtime.verify_loaded(receipt, proc=proc)
    write_stat(directory, state=state)
    assert not runtime.same_manager(evidence, proc=proc)
    with pytest.raises(ValueError, match="changed during"):
        runtime.verify_loaded(receipt, proc=proc)
