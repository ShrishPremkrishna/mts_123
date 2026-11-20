"""No services or hardware. Stage, prepare and verify the exact deployable tree."""
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import fc_walk_kick_release as release
import persistent_session


def test_staged_profile_is_self_contained_and_detects_a_changed_runner(tmp_path, monkeypatch):
    monkeypatch.setenv("PUPPER_FC_RECORDING", "full")
    monkeypatch.setenv("PUPPER_FC_IMU_RECOVERY", "supported_standby")
    root = tmp_path / "candidate"
    receipt = release.stage(release.ROOT, root)
    result = release.verify(root, expected_manifest=receipt["manifest_sha256"])
    assert result["profile"]["controllers"] == ["gemini_walk", "trick_leg_kick"]
    assert result["profile"]["motor_started"] is False
    assert result["profile"]["imu_recovery"] == "strict"
    assert result["profile"]["feedback_broadcasters"] == ["imu_sensor_broadcaster"]
    assert [p.name for p in (root / "gemini_pupper/lab").iterdir()] == ["operator_kick.py"]
    assert not (root / "gemini_pupper/robot/server.py").exists()
    assert not (root / "gemini_pupper/robot/tool_registry.py").exists()
    with pytest.raises(ValueError, match="exists"):
        release.stage(release.ROOT, root)
    # The release may be staged or inspected on a different machine without
    # the Mac checkout, ROS packages or any native motor service.
    runner = root / "final_project/behaviors/combined_fc_runner.py"
    runner.write_text(runner.read_text() + "\n# mutated\n")
    with pytest.raises(ValueError, match="different"):
        release.verify(root, expected_manifest=receipt["manifest_sha256"])


def test_ordinary_api_cannot_claim_fc_session():
    with pytest.raises(ValueError, match="owned by the field runner"):
        persistent_session.start_api(None, {"environment": release.ENV}, None)


@pytest.mark.parametrize("recording", [None, "events"])
def test_selected_bundle_persistent_prepare_is_file_only(tmp_path, recording):
    root = tmp_path / "frozen"
    receipt = release.stage(release.ROOT, root)
    # Synthetic native library only exercises the bounded file screening step.
    # It cannot be loaded or used as evidence of a compatible controller.
    native = tmp_path / "candidate.so"
    data = bytearray(64)
    data[:6] = b"\x7fELF\x02\x01"
    data[16:20] = b"\x03\x00\xb7\x00"
    native.write_bytes(data + b"\0".join(release.NATIVE_MARKERS))
    imu = tmp_path / "imu.so"
    imu.write_text("fake imu")
    run = tmp_path / "prepared"
    command = [sys.executable, "-B", str(root / "foundation/hardware/persistent_session.py"),
               "prepare", "--run-dir", str(run), "--app-manifest", str(root / release.MANIFEST),
               "--app-manifest-sha256", receipt["manifest_sha256"],
               "--native-library", str(native), "--native-sha256", release.file_digest(native),
               "--imu-library", str(imu), "--imu-sha256", release.file_digest(imu),
               "--telemetry-topics", str(root / release.TELEMETRY)]
    env = dict(os.environ, **release.ENV, PUPPER_GAIT_DEPLOY_DIR="/stale/checkout")
    if recording:
        env["PUPPER_FC_RECORDING"] = recording
        command[-1] = str(root/release.EVENT_TELEMETRY)
    output = subprocess.run(command, env=env, text=True, capture_output=True, timeout=10, check=True)
    assert json.loads(output.stdout)["motor_launch"] is False
    plan = json.loads((run / "selected-session.json").read_text())
    assert plan["controllers"] == list(release.CONTROLLERS)
    topics = json.loads(Path(plan["components"]["telemetry_topics"]["path"]).read_text())
    if recording == "events":
        assert topics == ["/trick_leg_kick/clip", "/fc/kick_result", "/estop_controller/request_stop", "/emergency_stop"]
    else:
        assert {"/joint_states", "/imu_sensor_broadcaster/imu", "/fc/body_command/admitted"} <= set(topics)

    assert plan["environment"]["PUPPER_GAIT_DEPLOY_DIR"] == str(root / "pupper_gait_deploy")
    assert not (run / "owned_session.json").exists()
    native.write_bytes(data)
    with pytest.raises(ValueError, match="markers"):
        release.inspect_native(native)
    native.write_text("libneural_controller.so")
    with pytest.raises(ValueError, match="AArch64"):
        release.inspect_native(native)


def test_guard_requires_matching_arguments_and_one_session_owner(tmp_path):
    proc = tmp_path / "proc"
    proc.mkdir()
    argv = [sys.executable, "-B", "-s", str(tmp_path / release.GUARD), *release.GUARD_ARGS]
    def process(pid, args):
        path = proc / str(pid)
        path.mkdir(exist_ok=True)
        (path / "cmdline").write_bytes(b"\0".join(os.fsencode(s) for s in args) + b"\0")

    def capture(pid, tag, guard):
        assert tag == "expected" and guard == str(tmp_path / release.GUARD)
        return {"argv": (proc / str(pid) / "cmdline").read_bytes().decode().split("\0")[:-1]}

    life = SimpleNamespace(PROC=proc, capture=capture)
    with pytest.raises(ValueError, match="one running"):
        release.verify_guard(life, "expected", tmp_path)
    process(123, argv)
    release.verify_guard(life, "expected", tmp_path)
    process(124, argv)
    with pytest.raises(ValueError, match="one running"):
        release.verify_guard(life, "expected", tmp_path)
    process(124, argv[:-1])
    with pytest.raises(ValueError, match="unexpected arguments"):
        release.verify_guard(life, "expected", tmp_path)
