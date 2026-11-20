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
            assert args[-1] in ("--read-hardware", "--supported-stand", "--dashboard-stand")
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
        elif "start-perception" in args:
            receipt = cli.read(folder / "owned_session.json")
            assert receipt["status"] == "running" and receipt["perception_deferred"]
            receipt["perception_deferred"] = False
            cli.write_new(folder / "owned_session.json", receipt)
            calls.append("perception")
        else:
            assert "start-native" in args
            calls.append("home")
            (folder / "owned_session.json").write_text(json.dumps({"session_id":"session", "status":"running",
                "boot_id":"boot-one", "perception_deferred":"--defer-perception" in args,
                "processes":{"manager":{"pid":123,"start_ticks":"4",
                "executable":"/opt/ros/ros2_control_node"}}}))
            cli.write_new(folder / "native-readiness.json", {"session_id":"session","homing_finished":True})
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
    assert setup.calls == ["prepare", "home", "perception", "--supported-stand", "--read-hardware"]
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
    with pytest.raises(ValueError, match="incomplete"):
        setup.call("match", "--runner-only", "--connect")
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


@pytest.mark.parametrize("feedback", [None, ["imu_sensor_broadcaster"]])
def test_recovery_consumes_fault_once_and_requires_positive_board_identity(tmp_path, monkeypatch, feedback):
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
    native_manifest = {"fc_imu_recovery": "supported_rehome"}
    if feedback is not None:
        native_manifest["feedback_broadcasters"] = feedback
    (run/"manifest.json").write_text(json.dumps(native_manifest))
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
    if feedback is not None:
        del controllers["joint_state_broadcaster"]
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
            assert set(request.activate_controllers) == set(feedback or ["joint_state_broadcaster","imu_sensor_broadcaster"])
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
    assert description == {"imu_recovery": "strict", "match_pitch_version": 1, "match_capable": True, "robot_id": "pupper-1", "source_host": "192.0.2.5", "robot_host": "192.0.2.70",
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
    assert profile == {"event": "profile", "imu_recovery": "strict", "match_pitch_version": 1, "match_capable": True, "robot_id": "pupper-1", "source_host": "192.0.2.5",
                       "robot_host": "192.0.2.70", "port": 5005, "producer_id": "fc-field",
                       "field_config": cli.read(setup.field)}

def test_dashboard_uses_same_home_once_and_reports_validated_team(setup, capsys):
    setup.call("prepare", "--dashboard", "--robot-id", "pupper-1")
    prepared = json.loads(capsys.readouterr().out)
    assert prepared["stage"] == "prepare" and prepared["robot_id"] == "pupper-1"
    with pytest.raises(ValueError, match="team identity"):
        setup.call("home","--dashboard","--supported","--robot-id","pupper-2")
    assert setup.calls == ["prepare"]
    setup.call("home","--dashboard","--supported","--home-only")
    setup.call("home","--dashboard","--supported")
    receipts = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert all(r["run"] == prepared["run"] and not r["match_armed"] for r in receipts)
    assert [r["home_only"] for r in receipts] == [True, False]
    assert setup.calls == ["prepare", "home", "--read-hardware", "perception", "--dashboard-stand"]


def test_dashboard_standing_prompt_discards_early_enter_on_a_real_terminal(monkeypatch):
    import io
    import pty
    import select
    from contextlib import ExitStack
    from types import ModuleType

    # Keep the real terminal flush, readiness loop and input checks. Only the
    # ROS controller inventory and gamepad gateway are device-free stand-ins.
    active = []
    gateway = SimpleNamespace(
        stop_capable=True, ready=lambda: True, stop_publisher=SimpleNamespace(get_subscription_count=lambda: 1),
        state={"latched": True}, capture=lambda *, pressed_at: pressed_at, send=active.append,
        poll=lambda: {"success": True}, request_stop=lambda _: None, poll_stop=lambda: True)
    modules = {"controller_manager_msgs.srv": {"ListControllers": object},
               "software_start": {"attach": lambda *args, **kwargs: gateway}}
    for name, members in modules.items():
        mod = ModuleType(name)
        vars(mod).update(members)
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.setattr(cli, "service", lambda *args: SimpleNamespace(controller=[
        SimpleNamespace(name="gemini_walk",state="active" if active else "inactive"),
        SimpleNamespace(name="trick_leg_kick",state="inactive")]))
    master, slave = pty.openpty()
    with ExitStack() as stack:
        stream = stack.enter_context(os.fdopen(slave, "r"))
        stack.callback(os.close, master)
        assert stream.isatty()
        monkeypatch.setattr(sys, "stdin", stream)
        output = io.StringIO()
        monkeypatch.setattr(sys, "stdout", output)
        os.write(master, b"\n")
        assert select.select([stream], [], [], .1)[0]
        def spin(node, timeout_sec):
            if not active:
                assert json.loads(output.getvalue()) == {"event":"stand_prompt"}
                assert not select.select([stream], [], [], 0)[0]
                os.write(master, b"\n")
        cli.supervised_stand(None, SimpleNamespace(spin_once=spin), dashboard=True)
        assert len(active) == 1


def test_pty_interrupt_allows_probe_to_finish_stop_before_parent_releases(tmp_path):
    import pty
    import select
    import signal
    import time

    log = tmp_path/"events"
    child = tmp_path/"probe.py"
    hw = str(Path(cli.__file__).parent)
    child.write_text("""
import sys, time
from importlib.util import find_spec
from pathlib import Path
from types import ModuleType, SimpleNamespace as N
sys.path.insert(0,sys.argv[1])
import fc_session
log=Path(sys.argv[2])
def add(value):
    with log.open('a') as f: f.write(value+'\\n')
stop_at=[]
real_ros=find_spec('rclpy') is not None
if real_ros: import rclpy
def stop(reason):
    add('stop requested')
    stop_at.append(time.monotonic())
def stopped():
    assert not real_ros or rclpy.ok()
    if time.monotonic()-stop_at[0] < .55: return False
    add('stop confirmed')
    return True
g=N(stop_capable=True, ready=lambda:True, stop_publisher=N(get_subscription_count=lambda:1),
    state={'latched':True}, capture=lambda **kwargs:1, send=lambda ticket:add('start submitted'),
    poll=lambda:None, request_stop=stop, poll_stop=stopped)
for name,attrs in {'controller_manager_msgs.srv':{'ListControllers':object,'ListHardwareComponents':object},'software_start':{'attach':lambda *a,**kw:g}}.items():
    m=ModuleType(name);vars(m).update(attrs);sys.modules[name]=m
fc_session.service=lambda *a:N(component=[N(name='control_board_hardware_interface',state=N(label='active'))],
    controller=[N(name='gemini_walk',state='inactive'),N(name='trick_leg_kick',state='inactive')])
try:
    if real_ros: fc_session.read_hardware(True,dashboard=True)
    else: fc_session.supervised_stand(None,N(spin_once=lambda *a,**kw:time.sleep(.01)),dashboard=True)
except KeyboardInterrupt: pass
finally: add('probe cleanup')
""")
    parent = ("import os,sys;from pathlib import Path\n"
              "sys.path.insert(0,sys.argv[1]); import fc_session\n"
              "try: fc_session.run_home_probe([sys.executable,sys.argv[2],sys.argv[1],sys.argv[3]],env=os.environ,timeout=5)\n"
              "except KeyboardInterrupt:\n with Path(sys.argv[3]).open('a') as f: f.write('parent released\\n')\n")
    pid, terminal = pty.fork()
    if pid == 0:
        os.execl(sys.executable, sys.executable, "-c", parent, hw, str(child), str(log))
    try:
        assert select.select([terminal], [], [], 4)[0]
        assert b'stand_prompt' in os.read(terminal, 1024)
        os.write(terminal, b"\n")
        end = time.monotonic() + 3
        while time.monotonic() < end and (not log.exists() or "start submitted" not in log.read_text()):
            time.sleep(.01)
        assert "start submitted" in log.read_text()
        os.write(terminal, b"\x03")  # Real foreground group signal to parent and child.
        child_pid, status = os.waitpid(pid, 0)
        pid = None
        assert os.waitstatus_to_exitcode(status) == 0
        assert log.read_text().splitlines() == ["start submitted", "stop requested", "stop confirmed",
                                                "probe cleanup", "parent released"]
    finally:
        os.close(terminal)
        if pid is not None:
            try: os.killpg(pid, signal.SIGKILL)
            except ProcessLookupError: pass
            os.waitpid(pid, 0)


def test_dashboard_connect_prepares_once_with_zero_motor_commands_and_reuses_manager(setup, monkeypatch, capsys):
    args = ("match", "--runner-only", "--connect", "--robot-id", "pupper-1")
    setup.call(*args)
    receipt = json.loads(capsys.readouterr().out.splitlines()[-1])
    assert receipt["event"] == "home_required" and receipt["match_capable"]
    setup.call(*args)
    assert setup.calls == ["prepare"]
    setup.call("home", "--supported", "--home-only")
    setup.call(*args)
    assert json.loads(capsys.readouterr().out.splitlines()[-1])["event"] == "home_required"
    setup.call("home", "--supported")
    def exec_runner(python, argv, env):
        assert argv[-1] == "--operator-controls"
        raise SystemExit(0)
    monkeypatch.setattr(os, "execve", exec_runner)
    with pytest.raises(SystemExit):
        setup.call(*args)
    assert setup.calls == ["prepare", "home", "--read-hardware", "perception", "--supported-stand"]


def test_missing_same_session_home_receipt_never_connects_or_starts_perception(setup):
    setup.call("prepare")
    setup.call("home", "--supported", "--home-only")
    state = cli.read(setup.profile.with_suffix(".current.json"))
    cli.write_new(Path(state["run_dir"]) / "native-readiness.json", {"session_id":"another", "homing_finished":True})
    with pytest.raises(ValueError, match="no confirmed Home"):
        setup.call("match", "--runner-only", "--connect")
    assert setup.calls == ["prepare", "home", "--read-hardware"]


def test_explicit_startup_trace_uses_the_new_run_location(setup, monkeypatch):
    monkeypatch.setenv("PUPPER_IMU_STARTUP_TRACE", "/home/pi/old-run/trace.csv")
    setup.call("prepare")
    current = cli.read(setup.profile.with_suffix(".current.json"))
    assert cli.read(Path(current["run_dir"]) / "selected-session.json")["environment"]["PUPPER_IMU_STARTUP_TRACE"] == "1"


def test_pitch_is_scoped_to_field_feed_and_does_not_change_enrollment(setup, tmp_path):
    before = setup.field.read_bytes()
    field = cli.session_field(cli.read(setup.profile), tmp_path, '{"length":3.2,"width":2.0}')
    assert cli.read(field)["field"] == {"length":3.2, "width":2.0}
    assert setup.field.read_bytes() == before
    with pytest.raises(ValueError):
        cli.session_field(cli.read(setup.profile), tmp_path, '{"length":3.2,"width":0}')


@pytest.mark.parametrize("mode", ["supported_damping", "supported_standby"])
def test_described_bench_cannot_start_any_field_authority(setup, capsys, mode):
    profile = cli.read(setup.profile)
    profile["imu_recovery"] = mode
    cli.write_new(setup.profile, profile)
    setup.call("describe")
    description = json.loads(capsys.readouterr().out)
    assert description["imu_recovery"] == mode and description["match_capable"] is False
    for action in ("match", "field"):
        with pytest.raises(ValueError, match="permanent body support"):
            setup.call(action)
    assert not setup.calls


def test_imu_delivery_requires_enrolled_candidate_current_manager_and_support(setup, monkeypatch):
    with pytest.raises(SystemExit):
        setup.call("test-imu-delivery")
    with pytest.raises(ValueError, match="enrolled field"):
        setup.call("test-imu-delivery", "--supported")
    profile = cli.read(setup.profile)
    profile["imu_recovery"] = "field_redundant_imu"
    cli.write_new(setup.profile, profile)
    with pytest.raises(ValueError, match="prepare first"):
        setup.call("test-imu-delivery", "--supported")
    setup.call("prepare")
    with pytest.raises(FileNotFoundError):
        setup.call("test-imu-delivery", "--supported")
    setup.call("home", "--supported")
    probes = []
    monkeypatch.setattr(cli, "run_home_probe", lambda cmd, **kw: probes.append((cmd, kw)))
    setup.call("test-imu-delivery", "--supported")
    assert len(probes) == 1
    cmd, kw = probes[0]
    assert cmd[:3] == [sys.executable, "-B", "-s"] and cmd[-2] == "--supported-imu-delivery"
    assert kw["env"]["PUPPER_FC_IMU_RECOVERY"] == "field_redundant_imu"
    assert kw["env"]["ROS_LOCALHOST_ONLY"] == "1" and kw["env"]["ROS_DOMAIN_ID"] == "168"
    (setup.proc / "123/environ").write_bytes(b"")
    with pytest.raises(ValueError, match="manager identity"):
        setup.call("test-imu-delivery", "--supported")
    assert len(probes) == 1


@pytest.mark.parametrize("case", ["ok", "timeout", "interrupt", "drop_refused", "lost_restore",
                                  "ignored_stop", "ignored_restore", "no_samples", "kick", "wrong_plugin", "motor_claim"])
def test_imu_delivery_restores_only_the_sensor_without_resuming(case):
    rows = [SimpleNamespace(name="imu_sensor_broadcaster", type="imu_sensor_broadcaster/IMUSensorBroadcaster",
                            state="active", claimed_interfaces=[]),
            SimpleNamespace(name="gemini_walk", state="active"),
            SimpleNamespace(name="trick_leg_kick", state="inactive")]
    if case == "kick": rows[2].state = "active"
    if case == "wrong_plugin": rows[0].type = "custom/Controller"
    if case == "motor_claim": rows[0].claimed_interfaces = ["hip/position"]
    switches, samples = [], []
    def switch(active):
        switches.append(active)
        if not active:
            if case != "ignored_stop": rows[0].state = "inactive"
            if case == "timeout": raise TimeoutError("deactivate reply lost after applying")
            if case == "drop_refused": raise ValueError("declined")
        else:
            if case != "ignored_restore": rows[0].state = "active"
            if case == "lost_restore": raise TimeoutError("no restore reply")
    def sample():
        samples.append(rows[0].state)
        if case == "no_samples" and len(samples) == 2:
            raise ValueError("no IMU messages")
    def quiet():
        assert rows[0].state == "inactive"
        if case == "interrupt": raise KeyboardInterrupt
    def run():
        return cli.exercise_imu_delivery(lambda:rows, switch, sample, quiet, lambda:False)
    if case == "ok":
        assert run() == {"ros_imu_delivery_gap_s":2,"broadcaster_restored":True,"resume_requested":False}
    else:
        expected = KeyboardInterrupt if case == "interrupt" else TimeoutError if case == "timeout" else ValueError
        with pytest.raises(expected) as error:
            run()
        if case in ("lost_restore", "ignored_restore", "no_samples"):
            assert "restoration unconfirmed" in str(error.value)
    if case in ("kick", "wrong_plugin", "motor_claim"):
        assert not switches and not samples
    else:
        assert switches == [False, True]
        assert rows[0].state == ("inactive" if case == "ignored_restore" else "active")
    assert rows[1].state == "active" and rows[2].state == ("active" if case=="kick" else "inactive")


def test_laptop_stand_requires_same_live_owner_and_reuses_home(setup, capsys, monkeypatch):
    from fc_operator import valid_owner
    template_path = Path(cli.read(setup.profile)["platform_template"])
    template = cli.read(template_path)
    template["components"]["gateway"] = {"path": "/native/gateway", "sha256": "test"}
    template["fc_operator_control"] = True
    cli.write_new(template_path, template)
    connect = ("match", "--runner-only", "--connect")
    setup.call(*connect)
    row = json.loads(capsys.readouterr().out.splitlines()[-1])
    assert row["event"] == "home_required" and row["fc_operator_control"] and not row["homed"]
    setup.call("home", "--supported", "--home-only")
    assert json.loads(capsys.readouterr().out)["homed"]
    setup.call(*connect)
    assert json.loads(capsys.readouterr().out)["homed"]
    owner = "ab" * 16
    assert valid_owner(owner)
    with pytest.raises(ValueError, match="no longer owns"):
        setup.call(*connect, "--operator-owner", owner)
    assert setup.calls == ["prepare", "home", "--read-hardware"]
