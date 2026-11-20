"""Offline enrollment against an isolated target layout, with no live device imports or sudo."""
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fc_enrollment as enrollment
import fc_walk_kick_release as release
from final_project.behaviors.body_command_guard import ARENA_DETECTOR_IDENTITY


def test_pinned_platform_profile_advertisement_fallback_and_release_rollback(tmp_path, monkeypatch):
    home = tmp_path / "target"
    home.mkdir()
    prefix = home / "installed"
    parts = {}
    header = bytearray(48)
    header[:6] = b"\x7fELF\x02\x01"
    header[16:20] = b"\x03\x00\xb7\x00"
    for name, (package, relative) in enrollment.NATIVE_LIBS.items():
        marker = prefix / "share/ament_index/resource_index/packages" / package
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.touch()
        binary = prefix / relative
        binary.parent.mkdir(parents=True, exist_ok=True)
        binary.write_bytes(header + b" ".join(release.NATIVE_MARKERS) + b" FC_IMU_PUBLISH_PER_REPORT_V1 FC_BOARD_HEALTH_V1" +
                           b" pupper_stop_v1 /estop_controller/start_motion /estop_controller/request_stop")
        parts[name] = {"path":str(binary),"sha256":release.file_digest(binary)}
    run = home / "pupper-fc-runs/previous"
    run.mkdir(parents=True)
    (run/"native-readiness.json").write_text("{}")
    (run/"selected-session.json").write_text(json.dumps({"prepared":True, "components":parts, "controllers":["gemini_walk","trick_leg_kick"],
        "environment":{"PUPPER_SESSION_MOTION_MODE":"fc_walk_kick","AMENT_PREFIX_PATH":str(prefix)},
        "interpreter":{"executable":sys.executable}}))
    model = run / "inputs/yolov8n_seg.hef"
    model.parent.mkdir()
    model.write_bytes(b"fixture")
    monkeypatch.setitem(ARENA_DETECTOR_IDENTITY,"model_sha256",sha256(model.read_bytes()).hexdigest())
    run_process = subprocess.run
    observed = []
    def no_devices(cmd, **kwargs):
        if "-c" in cmd and any("import cv2,numpy,rclpy,hailo_platform" in str(item) for item in cmd):
            observed.append("verified imports")
            return subprocess.CompletedProcess(cmd,0)
        if cmd[0] == "sudo":
            observed.append("avahi")
            return subprocess.CompletedProcess(cmd,1,"","sudo unavailable for fixture")
        return run_process(cmd,**kwargs)
    monkeypatch.setattr(enrollment.subprocess,"run",no_devices)
    args = SimpleNamespace(action="enroll",release=None,robot_id="pupper-1",platform_template=None,
        platform_env=None,imu_prefix=None,imu_recovery="strict",recording="events",field_config=None,detector_model=None,side="right")
    driver = Path(parts["imu"]["path"])
    updated_driver = driver.read_bytes()
    driver.write_bytes(updated_driver.replace(b"FC_IMU_PUBLISH_PER_REPORT_V1", b"previous_driver"))
    template = run / "selected-session.json"
    original = template.read_text()
    previous = json.loads(original)
    previous["components"]["imu"]["sha256"] = release.file_digest(driver)
    # Even a previously qualified ELF/digest must meet the new release's timing contract.
    template.write_text(json.dumps(previous))
    with pytest.raises(ValueError, match="per-report IMU publication"):
        enrollment.run(args, home=home)
    template.write_text(original)
    assert not (home / ".config/pupper-fc/session.json").exists()
    driver.write_bytes(updated_driver)
    first = enrollment.run(args, home=home)
    assert first["motors_started"] is False and not first["discovery"]["available"]
    assert "configured SSH" in first["discovery"]["fallback"] and not first["rollback_available"]
    active = home/".config/pupper-fc/session.json"
    first_profile = json.loads(active.read_text())
    first_template = json.loads(Path(first_profile["platform_template"]).read_text())
    assert first_template["components"] == parts
    assert not first_template["fc_operator_control"]
    assert json.loads(Path(first_profile["field_config"]).read_text())["attack_signs"] == {"pupper-1":-1,"pupper-2":1}
    assert (home/"pupper-fc/bin/fc_session.py").stat().st_mode & 0o111
    assert "active" not in first and "current" not in first
    assert not (active.parent/"session.current.json").exists()
    assert enrollment.run(args,home=home)["release"] == first["release"] and not (active.parent/"session.previous.json").exists()
    disabled = enrollment.run(SimpleNamespace(action="rollback"),home=home)
    assert disabled["state"] == "enrollment disabled" and not active.exists()
    assert json.loads(Path(disabled["saved_profile"]).read_text()) == first_profile

    assert enrollment.run(SimpleNamespace(action="rollback"),home=home)["state"] == "enrollment disabled"
    enrollment.run(args,home=home)
    args.side = "left"
    updated = enrollment.run(args,home=home)
    assert updated["rollback_available"] and json.loads(active.read_text())["side"]=="left"
    enrollment.run(SimpleNamespace(action="rollback"),home=home)
    assert json.loads(active.read_text()) == first_profile
    # A library changed in the selected overlay cannot replace a previous verified one.
    Path(parts["native"]["path"]).write_bytes(b"another controller")
    with pytest.raises(ValueError,match="differs"):
        enrollment.run(args,home=home)
    assert json.loads(active.read_text()) == first_profile
    assert observed.count("avahi")==6
    assert enrollment.run(SimpleNamespace(action="rollback"),home=home)["state"] == "already restored"
    assert json.loads(active.read_text()) == first_profile


def test_fc_operator_requires_paired_gateway_and_service_types(tmp_path, monkeypatch):
    header = bytes.fromhex("7f454c460201") + bytes(10) + bytes.fromhex("0300b700")
    base, gateway = tmp_path / "old", tmp_path / "new"
    for name, (package, relative) in enrollment.NATIVE_LIBS.items():
        path = base / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        marker = base / "share/ament_index/resource_index/packages" / package
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.touch()
        path.write_bytes(header + b" ".join((*release.NATIVE_MARKERS, b" FC_BOARD_HEALTH_V1 FC_IMU_PUBLISH_PER_REPORT_V1",
                                              b"pupper_stop_v1 /estop_controller/start_motion /estop_controller/request_stop")))
    old = base / enrollment.NATIVE_LIBS["gateway"][1]
    new = gateway / enrollment.NATIVE_LIBS["gateway"][1]
    new.parent.mkdir(parents=True)
    new.write_bytes(old.read_bytes())
    marker = gateway / "share/ament_index/resource_index/packages/joy_utils"
    marker.parent.mkdir(parents=True)
    marker.touch()
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(base))
    args = SimpleNamespace(platform_template=None, platform_env=None, imu_prefix=None, gateway_prefix=gateway, imu_recovery="strict")
    with pytest.raises(ValueError, match="Python services"):
        enrollment.platform(args, tmp_path)
    modules = gateway / "local/lib/python3.11/dist-packages"
    (modules / "joy_utils").mkdir(parents=True)
    with pytest.raises(ValueError, match="FC operator heartbeat"):
        enrollment.platform(args, tmp_path)
    new.write_bytes(new.read_bytes() + b" /estop_controller/fc_operator_heartbeat /estop_controller/fc_start_motion")
    def failed_import(cmd, **kwargs):
        assert cmd[-1] == str(gateway)
        assert kwargs["env"]["PYTHONPATH"].split(":")[0] == str(modules)
        raise subprocess.CalledProcessError(1, cmd)
    monkeypatch.setattr(subprocess, "run", failed_import)
    with pytest.raises(subprocess.CalledProcessError):
        enrollment.platform(args, tmp_path)
    monkeypatch.setattr(subprocess, "run", lambda *a, **kw: None)
    template = enrollment.platform(args, tmp_path)
    assert template["fc_operator_control"]
    assert template["components"]["gateway"] == {"path": str(new), "sha256": release.file_digest(new)}
    monkeypatch.setattr(release, "check_imu_recovery", lambda *_: None)
    args.imu_recovery = "supported_standby"
    assert not enrollment.platform(args, tmp_path)["fc_operator_control"]
