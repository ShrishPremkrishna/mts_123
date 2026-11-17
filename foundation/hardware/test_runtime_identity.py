"""Synthetic /proc and files exercise the real read-only release gate."""
import importlib.util
import json
import os
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location("runtime_identity", Path(__file__).with_name("runtime_identity.py"))
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)


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
        fields = ["S", *(["0"] * 18), str(ticks), "0"]
        (d / "stat").write_text(str(pid) + " (ros control node) " + " ".join(fields))
        lines = []
        for role in runtime.LIBRARIES:
            p = Path(files[role]["path"])
            st = p.stat()
            lines.append(f"123-456 r-xp 0 {os.major(st.st_dev):02x}:{os.minor(st.st_dev):02x} {st.st_ino} {p}")
        (d / "maps").write_text("\n".join(lines))
        return d
    return proc, manifest, files, receipt, manager


def test_verified_files_and_new_manager_are_bound_to_actual_maps(setup):
    proc, _, _, receipt, manager = setup
    manager()
    result = runtime.verify_loaded(receipt, proc=proc)
    assert result["verified"] and result["manager_pid"] == 123
    assert result["manager_start_ticks"] == 20000
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
