"""Offline operator contract. No ROS graph, network or actuators are opened."""
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fc_session as cli


@pytest.fixture
def setup(tmp_path, monkeypatch):
    root = tmp_path / "release"
    root.mkdir()
    proc = tmp_path / "proc"
    boot = proc / "sys/kernel/random/boot_id"
    boot.parent.mkdir(parents=True)
    boot.write_text("boot-one")
    monkeypatch.setattr(cli, "PROC", proc)
    monkeypatch.setattr(cli, "ROOT", root)
    model = tmp_path / "model"; model.write_bytes(b"fake")
    template = tmp_path / "platform.json"
    parts = {kind: {"path": "/" + kind, "sha256": kind} for kind in ("native", "imu")}
    template.write_text(json.dumps({"prepared": True, "controllers": ["gemini_walk", "trick_leg_kick"],
        "run_dir": str(tmp_path / "former"), "components": parts, "interpreter": {"executable": sys.executable},
        "environment": {"PATH": "/usr/bin", "ROS_DOMAIN_ID": "168", "PUPPER_SESSION_MOTION_MODE": "fc_walk_kick"}}))
    field = tmp_path / "field.json"
    field.write_text(json.dumps({"field": {}, "attack_signs": {"pupper-1": 1, "pupper-2": -1}}))
    profile = tmp_path / "config.json"
    profile.write_text(json.dumps({"robot_id": "pupper-1", "platform_template": str(template), "field_config": str(field),
                                   "manifest_sha256": "abc", "detector_model": str(model)}))
    calls = []
    def run(argv, *, env, **kwargs):
        assert env["ROS_LOCALHOST_ONLY"] == "1" and "ROS_STATIC_PEERS" not in env
        args = list(map(str, argv))
        if "--run-dir" not in args:
            assert args[-1] in ("--read-hardware", "--supported-stand")
            calls.append(args[-1])
            return
        folder = Path(args[args.index("--run-dir") + 1])
        if "prepare" in args:
            expected = "fc-match-events.json" if env["PUPPER_FC_RECORDING"]=="events" else "fc-walk-kick-topics.json"
            assert Path(args[args.index("--telemetry-topics")+1]).name == expected
            calls.append("prepare")
            folder.mkdir()
            (folder / "selected-session.json").write_text(json.dumps({"run_dir": str(folder), "app_root": str(root),
                "session_id":"session", "prepared": True, "environment": env,
                "components": {"app_manifest": {"sha256": "abc"}}}))
        else:
            assert "start-native" in args
            calls.append("home")
            (folder / "owned_session.json").write_text(json.dumps({"session_id":"session", "status":"running",
                "boot_id":"boot-one", "processes":{"manager":{"pid":123,"start_ticks":"4",
                "executable":"/opt/ros/ros2_control_node"}}}))
            (folder / "native-readiness.json").write_text("{}")
            mgr = proc / "123"; mgr.mkdir()
            (mgr / "stat").write_text("123 (name) " + "0 " * 19 + "4")
            (mgr / "exe").symlink_to("/opt/ros/ros2_control_node")
            (mgr / "environ").write_bytes(b"ROS_DOMAIN_ID=168\0ROS_LOCALHOST_ONLY=1\0PUPPER_PROBE_SESSION_ID=session\0")
    monkeypatch.setattr(cli.subprocess, "run", run)
    monkeypatch.setenv("ROS_STATIC_PEERS", "other-robot")
    return SimpleNamespace(call=lambda *a: cli.main([*a, "--profile", str(profile)]), calls=calls,
                           proc=proc, profile=profile, field=field)


def test_stages_reuse_home_disarm_and_address_one_robot(setup, monkeypatch):
    with pytest.raises(ValueError, match="prepare first"):
        setup.call("match", "--runner-only")
    setup.call("prepare"); setup.call("prepare")
    assert setup.calls == ["prepare"]
    with pytest.raises(ValueError, match="supported"):
        setup.call("home")
    setup.call("home", "--supported")
    setup.call("home")
    assert setup.calls == ["prepare", "home", "--supported-stand", "--read-hardware"]
    with pytest.raises(ValueError, match="identity"):
        setup.call("match", "--runner-only", "--robot-id", "pupper-2")
    def replace(python, argv, env):
        assert argv[-1] == "--operator-controls" and "--run-dir" in argv
        assert env["ROS_LOCALHOST_ONLY"] == "1"
        raise SystemExit(0)
    monkeypatch.setattr(os, "execve", replace)
    with pytest.raises(SystemExit) as result:
        setup.call("match", "--runner-only", "--robot-id", "pupper-1")
    assert result.value.code == 0 and setup.calls.count("home") == 1
    (setup.proc / "123/environ").write_bytes(b"ROS_DOMAIN_ID=168\0PUPPER_PROBE_SESSION_ID=session\0")
    with pytest.raises(ValueError, match="local-only"):
        setup.call("match", "--runner-only")


def test_never_rehome_on_stale_start_or_connect(setup):
    setup.call("prepare"); setup.call("home", "--supported")
    current = cli.read(setup.profile.with_suffix(".current.json"))
    folder = Path(current["run_dir"])
    (folder / "owned_session.json").write_text(json.dumps({"boot_id":"boot-one", "session_id":"session","status":"starting"}))
    with pytest.raises(ValueError, match="incomplete"):
        setup.call("home", "--supported")
    with pytest.raises(ValueError, match="incomplete"):
        setup.call("match", "--runner-only")
    setup.calls.clear()
    (folder / "owned_session.json").write_text(json.dumps({"status":"stopped"}))
    setup.call("prepare")
    assert setup.calls == ["prepare"] and cli.read(setup.profile.with_suffix(".current.json")) != current


def test_home_does_not_depend_on_field_camera(setup):
    setup.field.unlink()
    setup.call("prepare"); setup.call("home", "--supported")
    with pytest.raises(FileNotFoundError):
        setup.call("match", "--runner-only")
    assert setup.calls.count("home") == 1


def test_recovery_consumes_fault_once_and_requires_positive_board_identity(tmp_path, monkeypatch):
    import types
    board_name = "control_board_hardware_interface"
    field = lambda **kw: SimpleNamespace(**kw)
    run = tmp_path / "run"
    run.mkdir()
    (run/"robot.log").write_text("IMU fault phase=read reason=stale gyro sampled_mono_us=99\n")
    boot = tmp_path / "proc/sys/kernel/random/boot_id"
    boot.parent.mkdir(parents=True)
    boot.write_text("test")
    monkeypatch.setattr(cli, "PROC", tmp_path/"proc")
    (run/"owned_session.json").write_text('{"processes":{"manager":{"pid":12}}}')
    (run/"manifest.json").write_text('{"fc_imu_recovery":"supported_rehome"}')
    selected = {"environment":{"PUPPER_FC_IMU_RECOVERY":"supported_rehome"},
                "components":{"imu":{"path":"/tmp/driver","sha256":"pin"}},
                "runtime_manifest":{"path":str(run/"manifest.json"),"sha256":"pin"}}
    (run/"selected-session.json").write_text(json.dumps(selected))
    monkeypatch.setattr(cli, "check_manager", lambda *a: None)
    monkeypatch.setitem(sys.modules, "persistent_session", field(check_loaded=lambda *a: True, pinned=lambda *a: True))
    calls = []
    gw = field(stop_capable=True, state={"latched":False}, capture=lambda **kw: None)
    monkeypatch.setitem(sys.modules, "software_start", field(attach=lambda *a, **kw: gw))
    class SetHardware:
        class Request:
            def __init__(self): self.target_state=field()
    class Switch:
        class Request:
            STRICT=2
            def __init__(self): self.timeout=field()
    services = types.ModuleType("controller_manager_msgs.srv")
    services.SetHardwareComponentState, services.ListHardwareComponents = SetHardware, type("ListHardware", (), {})
    services.ListControllers, services.SwitchController = type("ListControllers", (), {}), Switch
    monkeypatch.setitem(sys.modules, "controller_manager_msgs", types.ModuleType("controller_manager_msgs"))
    monkeypatch.setitem(sys.modules, "controller_manager_msgs.srv", services)
    board = field(name=board_name,state=field(id=1),command_interfaces=[field(name=f"leg{joint}/port{port}",
        is_claimed=False,is_available=False) for joint in range(12) for port in range(5)])
    controllers={name:field(name=name,state="inactive",claimed_interfaces=[],type=kind) for name,kind in (
        ("gemini_walk","native"),("trick_leg_kick","native"),("joint_state_broadcaster","joint_state_broadcaster/JointStateBroadcaster"),
        ("imu_sensor_broadcaster","imu_sensor_broadcaster/IMUSensorBroadcaster"))}
    def query(node, ros, svc, endpoint, *, request=None, timeout=None):
        if svc == services.ListHardwareComponents: return field(component=[board])
        if svc == services.ListControllers: return field(controller=list(controllers.values()))
        if svc == SetHardware:
            calls.append("rehome")
            assert request.name == board_name and request.target_state.id == 3 and timeout == 60
            board.state.id = 3
            for handle in board.command_interfaces: handle.is_available=True
            return field(ok=True)
        if svc == Switch:
            calls.append("state only")
            assert set(request.activate_controllers) == {"joint_state_broadcaster","imu_sensor_broadcaster"}
            for name in request.activate_controllers: controllers[name].state="active"
            return field(ok=True)
        raise AssertionError(endpoint)
    monkeypatch.setattr(cli,"service",query)
    with pytest.raises(ValueError,match="latched"):
        cli.recover_board(None,None,run)
    assert not calls
    gw.state["latched"] = True
    controllers["gemini_walk"].state="active"
    with pytest.raises(ValueError,match="stop all leg"):
        cli.recover_board(None,None,run)
    controllers["gemini_walk"].state="inactive"
    cli.recover_board(None,None,run)
    assert calls == ["rehome","state only"]
    assert cli.read(run/"last-supported-imu-recovery.json")["restored"]
    with pytest.raises(ValueError,match="no new board"):
        cli.board_fault_marker(run)


def test_strict_home_and_reconnect_cannot_select_recovery(setup):
    setup.call("prepare")
    with pytest.raises(ValueError, match="no existing"):
        setup.call("home","--supported","--recover-imu")
    setup.call("home","--supported")
    with pytest.raises(ValueError, match="enrolled"):
        setup.call("home","--supported","--recover-imu")
    assert setup.calls.count("home") == 1




def test_camera_description_is_read_only_and_field_binds_the_actual_pi(setup, monkeypatch, capsys):
    monkeypatch.setenv("SSH_CONNECTION", "192.0.2.5 51000 192.0.2.70 22")
    setup.call("describe", "--robot-id", "pupper-1")
    description = json.loads(capsys.readouterr().out)
    assert description == {"robot_id": "pupper-1", "source_host": "192.0.2.5", "robot_host": "192.0.2.70",
                           "field_config": cli.read(setup.field), "port": 5005, "producer_id": "fc-field"}
    assert setup.calls == []
    setup.call("prepare")
    setup.call("home", "--supported")
    def capture(executable, argv, env):
        assert "combined_fc_runner.py" not in " ".join(argv)
        assert argv[-1] == "--stop-on-eof" and "--trace" not in argv
        assert argv[argv.index("--duration") + 1] == "3600"
        assert argv[argv.index("--source-host") + 1] == "192.0.2.5"
        assert argv[argv.index("--source-session") + 1] == "roof-camera"
        assert argv[argv.index("--epoch") + 1] == "game"
        assert env["ROS_LOCALHOST_ONLY"] == "1"
        raise SystemExit(0)
    monkeypatch.setattr(os, "execve", capture)
    with pytest.raises(SystemExit):
        setup.call("field", "--robot-id", "pupper-1", "--epoch", "game", "--source-session", "roof-camera",
                   "--revision", "3", "--tracks", '{"striker":"pupper-1"}')
    assert setup.calls.count("home") == 1


def test_full_diagnostics_is_explicit_and_preserved_on_reuse(setup):
    profile = cli.read(setup.profile)
    profile["recording"] = "full"
    cli.write_new(setup.profile,profile)
    setup.call("prepare"); setup.call("prepare")
    run = Path(cli.read(setup.profile.with_suffix(".current.json"))["run_dir"])
    plan = cli.read(run/"selected-session.json")
    assert plan["environment"]["PUPPER_FC_RECORDING"] == "full" and setup.calls == ["prepare"]
    command = cli.field_command(SimpleNamespace(source_host="laptop",source_session="camera",revision=1,
        camera_session=None,duration=120,tracks=None),profile,run,sys.executable,"game",continuous=True)
    assert "--trace" in command


def test_controller_free_home_only_starts_hardware_once_and_never_submits_walker_start(setup):
    setup.call("prepare")
    setup.call("home","--supported","--home-only")
    setup.call("home","--supported","--home-only")
    assert setup.calls == ["prepare","home","--read-hardware","--read-hardware"]
    setup.call("home","--supported")
    assert setup.calls[-1] == "--supported-stand" and setup.calls.count("home") == 1
    with pytest.raises(SystemExit):
        setup.call("match","--home-only")


def test_runner_emits_the_bound_ssh_profile_before_exec(setup, monkeypatch, capsys):
    setup.call("prepare")
    setup.call("home", "--supported")
    capsys.readouterr()
    monkeypatch.setenv("SSH_CONNECTION", "192.0.2.5 51000 192.0.2.70 22")
    def capture(*_):
        raise SystemExit(0)
    monkeypatch.setattr(os, "execve", capture)
    with pytest.raises(SystemExit):
        setup.call("match", "--runner-only", "--robot-id", "pupper-1")
    profile = json.loads(capsys.readouterr().out)
    assert profile == {"event": "profile", "robot_id": "pupper-1", "source_host": "192.0.2.5",
                       "robot_host": "192.0.2.70", "port": 5005, "producer_id": "fc-field",
                       "field_config": cli.read(setup.field)}
